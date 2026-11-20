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

What it does (v2, 2026-11-20 rework)
-----------------------------------
A last-stage floor on the published accel request, applied after ``NoCloseCap`` and before the standstill debounce (so no
candidate and no later stage can raise it back). It is a four-state machine, and it only ever DEEPENS an ask (``min``):

    d = median of the last N_MED present dRel samples        # 0.5 s at 20 Hz - one vision jump cannot move it

    OFF -> BRAKE   engaged, no driver gas, lead present and trusted, V_MIN < v <= V_MAX,
                   d < max(D_MIN, K_ENTER*v), MIN_SAMPLES in the buffer, launch grace and re-arm cooldown expired
    BRAKE          out = min(accel, A_FLOOR)                 # a real FCA11 brake request, below the -0.5 onset
    BRAKE -> HOLD  d >= max(D_EXIT, K_EXIT*v) CONTINUOUSLY for EXIT_DWELL_S (1 s), or v <= V_RELEASE (1 m/s)
    HOLD           out = min(accel, A_HOLD)                  # zero throttle, NO brake: see "the hold" below
    HOLD -> OFF    d >= max(D_RECOVER, K_RECOVER*v) CONTINUOUSLY for RECOVER_DWELL_S, or v < V_STANDSTILL
    BRAKE -> BRIDGE, HOLD -> BRIDGE   the lead goes absent/untrusted: keep the median buffer, drop the brake, hold the
                   throttle at zero for at most LEAD_BRIDGE_S; a lead that returns inside the bridge resumes BRAKE at once
    BRIDGE -> OFF  the lead does not return inside the bridge (buffer cleared)

The hold (R2/R3 of the 2026-11-20 review)
-----------------------------------------
A release is the dangerous edge: it hands the ask back to a planner that may still believe the lead is pulling away, and
the pedal law's hold feedforward then pushes the car forward at ask 0 (measured: +0.19 m/s^2 median over 298 341 frames,
F-SIM-PEDAL-FF-2026-11-20). So a release does NOT free the ask. It enters HOLD, which caps the ask at A_HOLD = -0.45
m/s^2 - a number chosen so that the gas law's own coast branch commands EXACTLY zero pedal (it is below -coast(v) for
every v in this rule's band: coast(10 m/s) = 0.29) while staying ABOVE the FCA11 onset (-0.5), so the hold is engine
braking / pass-through, not a brake. This does not depend on the pedal-fit branch: it is the shipped continuous negative
branch of the gas law, the same one that delivers the 59:40 coast.

Below V_RELEASE the floor keeps the same hold instead of freeing the ask (v1 released at 1 m/s and left the last ~3 m to
the planner). That hold is a THROTTLE floor, not a brake: measurement (F-SIM-MINGAP-V2-2026-11-20, R10) shows the car can
still creep at ~0.3 m/s^2 with the pedal at zero below 1 m/s, so the stop itself remains the base planner's/driver's.

Re-arm
------
From OFF only, and only after REARM_COOLDOWN_S = 5 s since the last brake release, so one approach cannot produce an
arm -> release -> arm cycle shorter than 5 s (review R1's anti-chatter bar). A gap that collapses again inside the
cooldown is still covered by HOLD's zero-throttle cap (never by throttle).

Fail-safe (each one is a unit test)
-----------------------------------
- lost lead / untrusted lead (``modelProb`` < MIN_LEAD_PROB): the brake is dropped at once, but the throttle stays at zero
  for up to LEAD_BRIDGE_S and the median buffer survives, so a re-acquired lead resumes immediately instead of after a
  0.25 s re-warm. (The corpus has 0 close-lead dropouts in 2.94 engaged hours; 506 engaged dropouts at any range, median
  0.40 s, 73 % back within 1.5 s - the bridge is sized on that, see the report.)
- driver gas: OFF at once (the driver is asking for torque; a brake floor must not fight it).
- standstill / launch: OFF below V_STANDSTILL, and a 2 s LAUNCH_GRACE_S starts whenever v drops under 1 m/s (not only at a
  true standstill), so a crawl at 0.5-0.9 m/s that never stops and then launches at a 2.6 m gap is not braked.
- above V_MAX: the brake is released at once into the HOLD cap (never freed), so leaving the band cannot add throttle.
- planner reset / not active: OFF at once.

Anti-chatter
------------
The entry band (3 m / 0.5 s) and the exit band (5 m / 0.8 s) are inside the frame-to-frame vision noise at 2-9 m (the
event logged 4.6 -> 8.9 -> 3.1 m in 0.3 s). The release therefore needs the MEDIAN above the exit band for a full second,
and the post-release hold needs the MEDIAN above a higher recovery band (6 m / 1.0 s headway) for a second. On the
retained corpus the rule fires on the same two real episodes as v1 (49:14 and 59:40 of 2026-11-19) and restarts zero
times.

This module is pure: it reads no messaging, owns its own buffer, and returns a float. The brake is delivered by the
existing FCA11 path - the floor only asks; the car layer's continuous law and the panda gate remain the only things that
decide what the ESC sees.
"""
from collections import deque

import numpy as np

from openpilot.common.realtime import DT_MDL

PARAM = "MinGapBrakeFloor"

# --- geometry / speed fences -----------------------------------------------------------------------------------------
V_MIN = 1.5           # m/s: no entry at/below this (the launch from rest is the standstill/launch logic's)
V_MAX = 10.0          # m/s: no entry above this (36 km/h / 22 mph) - a low-speed creep rule, not a highway rule
V_RELEASE = 1.0       # m/s: an active brake enters HOLD below this (never brake a car to a standstill; never free the ask)
V_STANDSTILL = 0.5    # m/s: a held car ends the HOLD here - the launch logic owns the pull-away, covered by the grace
D_MIN = 3.0           # m: entry gap floor
K_ENTER = 0.5         # s: entry gap = max(D_MIN, K_ENTER * v_ego)  -> 0.5 s headway
D_EXIT = 5.0          # m: exit gap floor
K_EXIT = 0.8          # s: exit gap = max(D_EXIT, K_EXIT * v_ego)  -> 0.8 s headway
D_RECOVER = 6.0       # m: post-release recovery gap floor
K_RECOVER = 1.0       # s: recovery gap = max(D_RECOVER, K_RECOVER * v_ego) -> 1.0 s headway

# --- asks ------------------------------------------------------------------------------------------------------------
A_FLOOR = -1.0        # m/s^2: the brake ask while BRAKING; below every FCA11 onset threshold, so it always brakes
A_HOLD = -0.45        # m/s^2: the post-release / dropout / below-1 m/s ask. Below -coast(v) everywhere in the band (gas
                      # law -> exactly zero pedal, ENABLE=0 pass-through) and above the FCA11 onset -0.5 (no brake).
                      # Pinned by test_hold_ask_is_zero_throttle_and_not_a_brake.

# --- timing ----------------------------------------------------------------------------------------------------------
N_MED = 10            # samples of the 0.5 s median at 20 Hz
MIN_SAMPLES = 5       # a lead must supply at least 0.25 s of its own samples before the floor may ADD a brake
EXIT_DWELL_S = 1.0    # s the MEDIAN must stay above the exit band before a brake releases (review R1)
RECOVER_DWELL_S = 1.0  # s the MEDIAN must stay above the recovery band before the hold ends (review R2)
LEAD_BRIDGE_S = 1.0   # s a dropout may last with the throttle held at zero and the buffer kept (review R4)
LAUNCH_GRACE_S = 2.0  # s after v drops under 1 m/s during which the floor may not arm (review R5)
REARM_COOLDOWN_S = 5.0  # s after a brake release before the floor may arm again (review R1's 5 s anti-chatter bar)
MIN_LEAD_PROB = 0.5   # modelProb below this = the model does not trust the track -> no brake (bridge instead)

OFF, BRAKE, HOLD, BRIDGE = "off", "brake", "hold", "bridge"


def enter_gap(v_ego: float) -> float:
  """Gap (m) at/below which the floor may turn on."""
  return max(D_MIN, K_ENTER * float(v_ego))


def exit_gap(v_ego: float) -> float:
  """Gap (m) at/above which an active brake may begin the release dwell."""
  return max(D_EXIT, K_EXIT * float(v_ego))


def recover_gap(v_ego: float) -> float:
  """Gap (m) at/above which the post-release hold may end (higher than the exit band, review R2)."""
  return max(D_RECOVER, K_RECOVER * float(v_ego))


def floor_ask(accel: float, state: str) -> float:
  """The floor itself: only ever deepens an ask. Never raises, never touches a brake that is already deeper."""
  if state == BRAKE:
    return min(float(accel), A_FLOOR)
  if state in (HOLD, BRIDGE):
    return min(float(accel), A_HOLD)
  return float(accel)


class MinGapFloor:
  """Owned by LongitudinalPlanner. ``update`` returns the (possibly floored) final ask.

  Diagnostics (read by tests / the shadow replay, never by the planner): ``state`` (one of OFF/BRAKE/HOLD/BRIDGE),
  ``gap_med`` (the median the decision used), ``launch_hold`` and ``rearm_t`` (the two clocks, seconds remaining).
  """

  def __init__(self, params=None, dt: float = DT_MDL):
    self.dt = dt
    self._params = params
    self._frame = 0
    self.enabled = False
    self.state = OFF
    self.gap_med = float("nan")
    self.launch_hold = 0.0   # s of post-crawl grace remaining before the floor may arm
    self.rearm_t = 0.0       # s of re-arm cooldown remaining after a brake release
    self.exit_t = 0.0        # s the median has held above the exit band
    self.recover_t = 0.0     # s the median has held above the recovery band
    self.bridge_t = 0.0      # s into a lead dropout
    self._bridge_from = OFF  # the state the bridge interrupted (BRAKE resumes; HOLD stays in the hold)
    n = max(1, int(round(N_MED)))
    self._buf: deque[float] = deque(maxlen=n)
    self._read_param()

  # -- plumbing ------------------------------------------------------------------------------------------------------
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

  def _reset(self, clear_buffer: bool = False) -> None:
    self.state = OFF
    self.exit_t = 0.0
    self.recover_t = 0.0
    self.bridge_t = 0.0
    self.gap_med = float("nan")
    if clear_buffer:
      self._buf.clear()

  # -- the rule ------------------------------------------------------------------------------------------------------
  def update(self, accel: float, v_ego: float, active: bool, gas_pressed: bool = False,
             lead_present: bool = False, d_rel: float | None = None, lead_prob: float = 1.0,
             standstill: bool = False) -> float:
    """``active`` is the planner's not-reset flag. ``d_rel`` is radarState.leadOne.dRel (vision), ``lead_prob`` its
    ``leadOne.modelProb``.

    The median buffer is maintained whether or not the feature is on, so switching the param on mid-drive starts from a
    settled estimate instead of a cold start.
    """
    if self._frame % max(1, int(1. / self.dt)) == 0:
      self._read_param()
    self._frame += 1

    v = float(v_ego)

    # LAUNCH GRACE. The median buffer is pre-warmed while the car sits still, so without a fence a launch from an
    # ordinary short queue stop (2.4-3.0 m) would arm the floor the instant v passed 1.5 m/s and brake a legitimate
    # pull-away. The clock (re)starts whenever the car is under 1 m/s - not only at a true standstill - so a crawl at
    # 0.5-0.9 m/s that never stops still gets the grace when it launches. It never extends or releases an active floor.
    if standstill or v < V_RELEASE:
      self.launch_hold = LAUNCH_GRACE_S
    elif self.launch_hold > 0.0:
      self.launch_hold = max(0.0, self.launch_hold - self.dt)
    if self.rearm_t > 0.0:
      self.rearm_t = max(0.0, self.rearm_t - self.dt)

    if lead_present and d_rel is not None and bool(np.isfinite(d_rel)) and float(lead_prob) >= MIN_LEAD_PROB:
      usable = True
      self._buf.append(float(d_rel))
    else:
      usable = False

    # Hard fail-safes: no brake, and no state at all (the driver or the planner owns the frame).
    if not self.enabled or not active or gas_pressed:
      self._reset(clear_buffer=not usable)
      return float(accel)

    if not usable:
      # LEAD DROPOUT / UNTRUSTED LEAD. Drop the brake at once (braking on a vanished lead is the thing we must not do),
      # but keep the buffer and hold the throttle at zero for up to LEAD_BRIDGE_S so a lead that is only momentarily
      # lost does not hand the pedal back at a 2 m gap. From OFF there is nothing to protect: forget the buffer.
      if self.state in (BRAKE, HOLD, BRIDGE) and len(self._buf):
        if self.state != BRIDGE:
          self._bridge_from = self.state          # BRAKE -> resume the brake; HOLD -> stay in the hold (never re-brake)
        self.state = BRIDGE
        self.bridge_t += self.dt
        if self.bridge_t > LEAD_BRIDGE_S - 1e-9:
          self._reset(clear_buffer=True)
          return float(accel)
        return floor_ask(accel, BRIDGE)
      self._reset(clear_buffer=True)
      return float(accel)

    # A usable lead is back: the bridge ends and the state resumes on the median.
    self.bridge_t = 0.0
    if not len(self._buf):
      return float(accel)
    d = float(np.median(self._buf))
    self.gap_med = d

    if self.state == BRIDGE:
      # A lead that returns inside the bridge resumes BRAKE at once on the buffer it never lost - but ONLY if the bridge
      # came from BRAKE. A bridge from HOLD goes back to the hold: the release already happened and the 5 s re-arm
      # cooldown still applies, so a 0.3 s dropout cannot re-brake inside a second of the release (the chatter case).
      if self._bridge_from == BRAKE and d < exit_gap(v) and V_RELEASE < v <= V_MAX:
        self.state = BRAKE
      else:
        # The lead came back beyond the exit band (or the bridge started in the hold): this is a release, so the 5 s
        # re-arm cooldown applies to whatever comes next.
        self.state = HOLD
        self.rearm_t = max(self.rearm_t, REARM_COOLDOWN_S)
      self.exit_t, self.recover_t = 0.0, 0.0

    if self.state == BRAKE:
      if v > V_MAX or v <= V_RELEASE:
        # Leaving the band NEVER frees the ask: the brake steps down to the zero-throttle hold and the 5 s re-arm
        # cooldown starts, so leaving the band cannot add throttle and cannot produce a quick re-arm.
        self.state, self.rearm_t = HOLD, REARM_COOLDOWN_S
        self.exit_t, self.recover_t = 0.0, 0.0
      elif d >= exit_gap(v):
        self.exit_t += self.dt
        if self.exit_t >= EXIT_DWELL_S - 1e-9:
          self.state, self.rearm_t = HOLD, REARM_COOLDOWN_S
          self.exit_t, self.recover_t = 0.0, 0.0
      else:
        self.exit_t = 0.0

    elif self.state == HOLD:
      if v < V_STANDSTILL:
        # A stopped car is the standstill/launch logic's: end the hold, and the launch grace (restarted every frame under
        # 1 m/s) keeps the floor disarmed through the pull-away.
        self._reset()
      elif d >= recover_gap(v):
        self.recover_t += self.dt
        if self.recover_t >= RECOVER_DWELL_S - 1e-9:
          self._reset()
      else:
        self.recover_t = 0.0
      if self.state == HOLD and self.rearm_t <= 0.0 and d < enter_gap(v) and len(self._buf) >= MIN_SAMPLES:
        # The gap has collapsed again after the cooldown and the hold was not enough: a second, genuine approach.
        self.state = BRAKE
        self.exit_t = 0.0

    elif self.state == OFF:
      self.exit_t = self.recover_t = 0.0
      self.state = BRAKE if (self.launch_hold <= 0.0 and self.rearm_t <= 0.0 and len(self._buf) >= MIN_SAMPLES
                             and V_MIN < v <= V_MAX and d < enter_gap(v)) else OFF

    return floor_ask(accel, self.state)
