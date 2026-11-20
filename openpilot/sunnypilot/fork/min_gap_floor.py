"""
Fork: low-speed minimum-gap brake floor (adurham/sunnypilot). Inert unless the ``MinGapBrakeFloor`` param is set.

Why this exists
---------------
Route 000001ac, 2026-11-19, I-80/94 stop-and-go at 12-14 mph, night, a flat-rear white RV trailer directly ahead.
From 59:37 to 59:45.7 the car crept from ~7.5 m to 1.4-2.7 m behind that trailer (THW 0.3-0.5 s) while the published ask
sat at exactly 0.00 and the car accelerated 4.08 -> 6.21 m/s; the driver braked and disengaged. Vision read the trailer as
pulling away (vRel +13.7/+15.0 m/s spikes while the gap shrank), so the MPC and the e2e candidates both stayed positive,
``NoCloseOnLead`` released twice, and nothing ever asked for a brake: the ask after 59:35 bottomed at -0.16, against the
FCA11 onset of -0.5. The gap itself was never wrong - the 0.5 s median dRel tracked the trailer at 1.4-2.7 m.

That is the hole this fills: every existing rule keys on the lead's SPEED (vRel / closing / vLead), and every one of them
is blind when the speed is wrong. The gap is the robust signal at these ranges, so the last resort keys on the gap alone.

What it does
------------
A last-stage FLOOR on the published accel request, applied after ``NoCloseCap`` and before the standstill debounce (so no
candidate and no later stage can raise it back):

    d = median of the last N_MED present dRel samples        # 0.5 s at 20 Hz - one vision jump cannot move it
    ON  while engaged, no driver gas, lead present and trusted, V_MIN < v_ego <= V_MAX, d < max(D_MIN, K_ENTER*v)
    OFF when d >= max(D_EXIT, K_EXIT*v) or v_ego < V_RELEASE
    while ON:  out = min(accel, A_FLOOR)                     # a real FCA11 brake request, below the -0.5 onset

It NEVER raises an ask and never turns a brake into a coast, so it cannot defeat a candidate that already brakes deeper,
and it cannot start a stop on its own above V_RELEASE. The brake is delivered by the existing FCA11 path - the floor only
asks; the car layer's continuous law and the panda gate remain the only things that decide what the ESC sees.

Fail-safe (each one is a unit test)
-----------------------------------
- lost lead (``present`` false): OFF and the median buffer is cleared, so a re-acquired lead starts from fresh samples.
- untrusted lead (``radarState.modelProb`` < MIN_LEAD_PROB): OFF. The 2026-11-19 track held prob 1.00 through the whole
  event, so this guard does not disarm the case it was built for - it stops the floor acting on a track the model itself
  doubts. (``leadOne`` has no prob field of its own; ``modelProb`` is the leadsV3 probability radard used for it.)
- driver gas: OFF (the driver is asking for torque; a brake floor must not fight it).
- launch / standstill: OFF below V_MIN = 1.5 m/s on entry and released below V_RELEASE = 1.0 m/s, and ``active`` false
  whenever the planner resets (long inactive / override). Because the median buffer is deliberately pre-warmed while the
  car sits still, standing still also starts a LAUNCH_GRACE_S = 2 s clock during which the floor may not ARM - without it
  a launch from an ordinary short queue stop (2.4-3.0 m) would brake a legitimate pull-away the instant v passed 1.5 m/s.
- above V_MAX = 10 m/s: OFF. Highway gaps are a different question (a -1.0 brake at 70 mph is not this rule's business).

Anti-chatter
------------
The 3 m / 5 m band is ~2 m wide, which at 2-8 m is inside the frame-to-frame vision noise, so the ON state is entered on
the median and left on the WIDER exit band only. On the retained corpus (25 routes, the only two episodes anywhere are
49:14 and 59:40 of this drive) the rule fires twice and restarts zero times; the closed-loop replay is where a limit cycle
would show up, and that is recorded in the lane report, not hidden here.
"""
from collections import deque

import numpy as np

from openpilot.common.realtime import DT_MDL

PARAM = "MinGapBrakeFloor"

V_MIN = 1.5        # m/s: no entry at/below this (launch from a standstill is owned by the standstill/launch logic)
V_MAX = 10.0       # m/s: no entry above this (36 km/h / 22 mph) - this is a low-speed creep rule, not a highway rule
D_MIN = 3.0        # m: entry gap floor
K_ENTER = 0.5      # s: entry gap = max(D_MIN, K_ENTER * v_ego)  -> 0.5 s headway
D_EXIT = 5.0       # m: exit gap floor
K_EXIT = 0.8       # s: exit gap = max(D_EXIT, K_EXIT * v_ego)  -> 0.8 s headway (the hysteresis band)
A_FLOOR = -1.0     # m/s^2: the ask the floor imposes; below every FCA11 onset threshold, so it always brakes
N_MED = 10         # samples of the 0.5 s median at 20 Hz
MIN_SAMPLES = 5    # a lead must supply at least 0.25 s of its own samples before the floor may ADD a brake
LAUNCH_GRACE_S = 2.0  # s after leaving a standstill during which the floor may not arm (see the launch fence below)
V_RELEASE = 1.0    # m/s: release below this even while the gap is short (never brake a car to a standstill)
MIN_LEAD_PROB = 0.5  # leadOne.prob below this = the model does not trust the track -> do nothing


def enter_gap(v_ego: float) -> float:
  """Gap (m) at/below which the floor may turn on."""
  return max(D_MIN, K_ENTER * float(v_ego))


def exit_gap(v_ego: float) -> float:
  """Gap (m) at/above which an active floor releases."""
  return max(D_EXIT, K_EXIT * float(v_ego))


def floor_ask(accel: float, on: bool) -> float:
  """The floor itself: only ever deepens an ask. Never raises, never touches a brake that is already deeper."""
  return min(float(accel), A_FLOOR) if on else float(accel)


class MinGapFloor:
  """Owned by LongitudinalPlanner. ``update`` returns the (possibly floored) final ask."""

  def __init__(self, params=None, dt: float = DT_MDL):
    self.dt = dt
    self._params = params
    self._frame = 0
    self.enabled = False
    self.on = False           # the floor is active on this frame (diagnostics / tests)
    self.gap_med = float("nan")  # the median gap the decision used on the last frame
    self.launch_hold = 0.0    # s of post-standstill grace remaining before the floor may arm
    n = max(1, int(round(N_MED)))
    self._buf: deque[float] = deque(maxlen=n)
    self._read_param()

  def _read_param(self) -> None:
    if self._params is None:
      try:
        from openpilot.common.params import Params
        self._params = Params()
      except Exception:
        self.enabled = False
        return
    try:
      self.enabled = bool(self._params.get_bool(PARAM))
    except Exception:  # UnknownKeyName on a libparams that predates the key: feature stays off
      self.enabled = False

  def _reset(self) -> None:
    self.on = False
    self.gap_med = float("nan")

  def update(self, accel: float, v_ego: float, active: bool, gas_pressed: bool = False,
             lead_present: bool = False, d_rel: float | None = None, lead_prob: float = 1.0,
             standstill: bool = False) -> float:
    """``active`` is the planner's not-reset flag. ``d_rel`` is radarState.leadOne.dRel (vision), ``lead_prob`` its prob.

    The median buffer is maintained whether or not the feature is on, so switching the param on mid-drive starts from a
    settled estimate instead of a cold start.
    """
    if self._frame % max(1, int(1. / self.dt)) == 0:
      self._read_param()
    self._frame += 1

    # LAUNCH FENCE. The median buffer is deliberately pre-warmed while the car sits at a standstill, so without this a
    # launch from an ordinary short queue stop (2.4-3.0 m behind the lead) would arm the floor the instant v passes
    # 1.5 m/s and brake a legitimate pull-away. Standing still (re)starts a LAUNCH_GRACE_S clock; the floor may not ARM
    # until it has run out. It never extends or releases an already-active floor.
    if standstill:
      self.launch_hold = LAUNCH_GRACE_S
    elif self.launch_hold > 0.0:
      self.launch_hold = max(0.0, self.launch_hold - self.dt)

    if lead_present and d_rel is not None and bool(np.isfinite(d_rel)) and float(lead_prob) >= MIN_LEAD_PROB:
      usable = True
      self._buf.append(float(d_rel))
    else:
      usable = False
      self._buf.clear()

    if not self.enabled or not active or gas_pressed or standstill or not usable:
      self._reset()
      return float(accel)

    d = float(np.median(self._buf))
    self.gap_med = d
    v = float(v_ego)
    if self.on:
      self.on = (v > V_RELEASE) and (v <= V_MAX) and d < exit_gap(v)
    else:
      # MIN_SAMPLES: a lead that has just appeared / re-appeared must produce 0.25 s of its own samples first, so one
      # noisy first frame can never add a brake. An already-active floor is exempt (it only ever releases on the gap).
      # launch_hold: never arm during the post-standstill grace window.
      self.on = ((len(self._buf) >= MIN_SAMPLES) and (self.launch_hold <= 0.0)
                 and (v > V_MIN) and (v <= V_MAX) and d < enter_gap(v))
    return floor_ask(accel, self.on)
