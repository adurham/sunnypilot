"""
Fork: never accelerate toward a lead (adurham/sunnypilot). Inert unless the ``NoCloseOnLead`` param is set.

Why this exists
---------------
Owner, after the 2026-11-01 drive: "it still accelerates towards a car that comes into its view to try and get TO the
follow distance, it never needs to do that." And 2026-11-04 (complaint 3): "cruising, then within ~5 s of a car ahead it
races toward max speed to close the gap. Must NEVER happen."

That is the MPC's steady-state solution, not a bug in it: when a slower lead is visible and the gap is wider than the
follow distance ``T*v + 6 m``, the optimal plan is to speed up until the gap equals the target, then match the lead. The
fork's far-lead e2e gate (raise to min(mpc, 0.4)) plus AccelBoostSP (+0.25) do the same from the e2e side: route 194
t=330-332 published exactly 0.65 = 0.40 gate + 0.25 boost while the model itself asked 0.04-0.10, with the gap shrinking
98 -> 44 m.

What it does
------------
A last-stage cap on the PUBLISHED accel request while a lead is tracked. Only a POSITIVE ask is ever touched (it is
scaled by a factor in [0, 1]); a zero or negative ask - coast or brake - passes through bit-for-bit, so the cap can
never deepen or delay a brake request. Applied after the ``min()`` over the MPC / cruise / e2e candidates, so no
candidate can raise it back; and because the final ask IS that min(), it is never above the lead-follow MPC candidate.

    closing  = LeadClosingFilter(dRel, vRel)                     # m/s, > 0 = the gap is shrinking (lead_closing.py)
    raw      = clip((enter(v) - closing) / (enter(v) - exit(v)), 0, 1)
    factor   = min(raw over the last RELEASE_DWELL_S)           # tighten immediately, release only after the dwell
    out      = accel * factor  if accel > 0  else accel

THE INVARIANT (2026-11-04, lane D). At speed (v_ego >= V_STRICT) enter = 0 and exit = -0.5 m/s: NO positive ask while
the gap is not growing - the car may approach a slower lead only by holding speed or coasting, and gets its throttle
back only to follow a lead that pulls away (full ask once it pulls away by >= 0.5 m/s). That is deliberately stronger
than "no positive ask while closing INSIDE the follow distance": every positive-ask-while-closing episode on routes
189-194 happened OUTSIDE the follow distance (1.1-4x d_des: the MPC chasing the gap is by definition outside it), so an
inside-the-gap rule would have changed nothing the owner felt. Below V_LOW (a launch from a stop - owned by the
standstill/launch logic, lane C) the previous band (enter 0.6 / exit 0.25 m/s) is kept; V_LOW..V_STRICT blends the two
linearly so the law has no step in speed.

The closing estimate is the other half of the fix. The old cap read the raw per-frame ``vEgo - vLead`` of a vision-only
lead, which (a) under-reads closing at range - the cap stayed at 1.0 while the gap shrank and the MPC chased it (193
t=297-302: +0.4..0.57 at 1.27x the follow gap, raw closing -0.3..0 while the gap shrank ~1.25 m/s) - and (b) jitters by
0.4-0.7 m/s per frame, across the whole old 0.35 m/s ramp, so the factor flipped 0 <-> 1 and the ask 0 <-> +0.65 several
times a second (194 t=429-431): exact-zero flips of this cap are 11 of 24 de-duplicated ask steps on route 193 and 15 of
31 on 194 - the "gas flutter" (complaint 4). A lead that drops out for a few frames (176-366 dropouts per engaged hour)
used to release the cap for those frames too; the filter now bridges dropouts up to 0.5 s and the cap holds through them.

The dwell is unchanged: the min-window delays only RELEASES (a rising factor), never a tighten, and adds no fall.
"""
from collections import deque

import numpy as np

from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.fork.lead_closing import LeadClosingFilter

PARAM = "NoCloseOnLead"

# at speed: never faster than the lead (closing >= 0 -> no positive ask); full ask once the lead pulls away by 0.5 m/s
ENTER = 0.0     # m/s of closing at/above which a positive ask is fully capped to 0
EXIT = -0.5     # m/s of closing (= the lead pulling away at 0.5 m/s) at/below which the cap is fully released
# launch regime: the pre-2026-11-04 band, unchanged (the standstill/launch decision belongs to lane C)
ENTER_LOW = 0.6
EXIT_LOW = 0.25
V_LOW = 3.0     # m/s: at/below this the launch band applies
V_STRICT = 5.0  # m/s: at/above this the strict band applies; linear blend between
RELEASE_DWELL_S = 0.5   # s: the factor is the minimum raw factor over this window - a release must hold this long
# closing used by the cap = max(filtered gap rate, mean of the raw vEgo - vLead over RAW_MEAN_S): an ENVELOPE, so either
# source showing closing caps the throttle. The KF alone trusts the gap slope, which at 80-100 m is polluted by vision
# switching between two objects (193 t=254: dRel 79/102/89 m in 2 s); the raw mean alone under-reads at range. The 1 s
# mean has ~1/4 of the raw per-frame jitter, so the envelope keeps the flutter fix.
RAW_MEAN_S = 1.0

# a lead whose speed is not usable (radarState vLead can be 0 for an unconfirmed track) must NOT read as "closing".
MIN_LEAD_SPEED_VALID = -1.0


def lead_speed_usable(v_lead: float | None) -> bool:
  return v_lead is not None and bool(np.isfinite(v_lead)) and v_lead >= MIN_LEAD_SPEED_VALID


def band(v_ego: float) -> tuple[float, float]:
  """(enter, exit) closing thresholds at this speed: launch band at/below V_LOW, strict band at/above V_STRICT."""
  w = float(np.clip((v_ego - V_LOW) / (V_STRICT - V_LOW), 0., 1.))
  return ENTER_LOW + w * (ENTER - ENTER_LOW), EXIT_LOW + w * (EXIT - EXIT_LOW)


def no_close_factor(closing: float, v_ego: float = V_STRICT) -> float:
  """Raw (un-dwelled) cap factor: 1 at/below exit(v), 0 at/above enter(v), linear between."""
  enter, exit_ = band(v_ego)
  return float(np.clip((enter - closing) / (enter - exit_), 0., 1.))


def cap_no_close(accel: float, v_ego: float, lead_present: bool, v_lead: float | None, closing: bool = False,
                 enabled: bool = True, factor: float | None = None) -> float:
  """The published ask with its positive part scaled by the cap factor (0..1). ``factor`` wins when given; otherwise the
  binary form (``closing`` -> 0, else 1). Never touches a zero/negative ask, an absent lead or an unusable lead speed."""
  if not enabled or not lead_present or accel <= 0.:
    return accel
  if not lead_speed_usable(v_lead):
    return accel
  if factor is None:
    factor = 0. if closing else 1.
  return accel * float(np.clip(factor, 0., 1.))


class NoCloseCap:
  """Owned by LongitudinalPlanner. ``update`` returns the (possibly capped) final ask."""

  def __init__(self, params=None, dt: float = DT_MDL):
    self.dt = dt
    self._params = params
    self._frame = 0
    self.enabled = False
    self.factor = 1.         # dwelled cap factor applied on the last frame (1 = no cap)
    self.closing = False     # factor < 1, i.e. some cap is in effect
    self.closing_speed = 0.  # m/s, the closing estimate used on the last frame
    self.lead_filter = LeadClosingFilter(dt)
    self._window: deque[float] = deque(maxlen=max(1, int(round(RELEASE_DWELL_S / dt))))
    self._raw: deque[float] = deque(maxlen=max(1, int(round(RAW_MEAN_S / dt))))
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
    self._window.clear()
    self.factor = 1.
    self.closing = False
    self.closing_speed = 0.

  def update(self, accel: float, v_ego: float, lead_present: bool, v_lead: float | None,
             d_rel: float | None = None, v_rel: float | None = None) -> float:
    """``d_rel``/``v_rel`` (radarState.leadOne) feed the closing filter. Without ``d_rel`` the closing speed is this
    frame's raw ``v_ego - v_lead`` (the pre-2026-11-04 input) and a dropout releases at once, as before."""
    if self._frame % int(1. / self.dt) == 0:
      self._read_param()
    self._frame += 1

    usable = lead_speed_usable(v_lead)
    filtered = d_rel is not None
    if filtered:
      # tracked whether or not the feature is on, so switching it on mid-drive starts from a settled estimate
      self.lead_filter.update(lead_present and usable, d_rel, v_rel)
      if lead_present and usable and v_lead is not None:
        self._raw.append(float(v_ego - v_lead))
      elif not self.lead_filter.valid:
        self._raw.clear()
    tracked = lead_present or (filtered and self.lead_filter.coasting)

    if not self.enabled or not tracked:
      self._reset()
      return accel

    if filtered and self.lead_filter.valid:
      raw_mean = sum(self._raw) / len(self._raw) if self._raw else self.lead_filter.closing
      self.closing_speed = max(self.lead_filter.closing, raw_mean)
      raw = no_close_factor(self.closing_speed, v_ego)
    elif usable and v_lead is not None:
      self.closing_speed = float(v_ego - v_lead)
      raw = no_close_factor(self.closing_speed, v_ego)
    else:
      # an unusable (but finite) lead speed reads as "not closing" for this frame without flushing the window, so one
      # bad frame cannot release a cap the dwell is holding.
      raw = 1.
    self._window.append(raw)
    self.factor = min(self._window)  # this frame is in the window: tighten is immediate; a rise needs the dwell
    self.closing = self.factor < 1.
    if accel <= 0. or not (usable or (filtered and self.lead_filter.coasting)):
      return accel  # brake/coast bit-for-bit; an unusable lead-speed frame is inert for that frame (as before)
    return accel * self.factor
