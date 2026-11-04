"""
Fork: never accelerate toward a lead (adurham/sunnypilot). Inert unless the ``NoCloseOnLead`` param is set.

Why this exists
---------------
Owner, after the 2026-11-01 drive: "it still accelerates towards a car that comes into its view to try and get TO the
follow distance, it never needs to do that."

That is the MPC's steady-state solution, not a bug in it: when a slower lead is visible and the gap is wider than the
follow distance ``T*v + 6 m``, the optimal plan is to speed up until the gap equals the target, then match the lead.
The car therefore accelerates at a car it can already see, and only starts giving the speed back once the gap is at the
target. The fork's far-lead e2e gate already stops the *model* from braking early for such a lead, and it caps how far
that gate may raise the ask, but nothing caps the cruise/MPC candidates from chasing the gap.

What it does
------------
A last-stage cap on the PUBLISHED accel request while a lead is present. Only a POSITIVE ask is ever touched (it is
scaled by a factor in [0, 1]); a zero or negative ask - coast or brake - passes through bit-for-bit, so the cap can
never deepen or delay a brake request. Applied after the ``min()`` over the MPC / cruise / e2e candidates, so no
candidate can raise it back.

The cap is CONTINUOUS in the closing speed ``v_rel = v_ego - v_lead`` (2026-11-03, route 191 "throttle flutter")::

    raw(v_rel) = clip((ENTER - v_rel) / (ENTER - EXIT), 0, 1)   # 1 at/below EXIT, 0 at/above ENTER, linear between
    factor     = min(raw over the last RELEASE_DWELL_S)          # the release dwell, generalised to a continuous factor
    out        = accel * factor  if accel > 0  else accel

Why continuous: the previous cap was binary (``min(accel, 0)`` while "closing", with ENTER/EXIT hysteresis). Whenever it
engaged under a positive ask the request fell from the full ask (route 191: the far-lead gate's 0.400) to exactly 0.000
in one frame - a ~33 LSB pedal drop, and the pedal law lets any fall > 8 LSB through in one frame. Hysteresis and a
dwell only make that cliff rarer, not lower. With the ramp the frame-to-frame fall of the output is bounded by
``accel * dv_rel / (ENTER - EXIT)`` plus the ask's own change: throttle fades as the closing speed rises, and the whole
ask can only vanish in one frame if the closing speed itself crosses the whole band in that frame.

How the dwell and the ramp interact (one mechanism each, not stacked):
  - The ramp REPLACES the hysteresis. Hysteresis exists only to stop a binary output chattering across one threshold;
    a continuous map has no threshold to chatter across, and the old ENTER..EXIT band becomes the ramp's span.
  - The dwell is kept for noise rejection in the RELEASE direction only, generalised from "release after EXIT has held
    RELEASE_DWELL_S" to a sliding-window MINIMUM of the raw factor over RELEASE_DWELL_S:
      * tighten is immediate: the current frame is always in the window, so a rising closing speed lowers the factor on
        the very frame it is seen. No filter and no rate limit on the engage direction - pulling throttle while closing
        is never delayed;
      * release needs evidence: the factor can only rise once the lower value has aged out of the window, so one noisy
        frame of "matched gap" never restores throttle;
      * the min-window never ADDS a fall: the factor falls only when the newest raw value is below everything in the
        window, and then by at most the raw signal's own fall that frame. Its only steps are upward (a value ageing out),
        and a rise is slewed downstream by the pedal's rate-up limit - only falls reach the pedal in one frame;
      * outside the band (raw 0 or 1) it reduces EXACTLY to the old behaviour: full cap from the first frame at/above
        ENTER, full release once RELEASE_DWELL_S has passed at/below EXIT.
  A first-order filter or rate limit on the factor was rejected: on the tighten side it delays pulling throttle
  (forbidden), and on the release side alone it turns each noise spike into a drop-then-ramp sawtooth.
"""
from collections import deque

import numpy as np

from openpilot.common.realtime import DT_MDL

PARAM = "NoCloseOnLead"

# 2026-11-03 (route 191, the owner's "throttle fluttering"): with a binary 0.3/0.1 cap the ask flipped 0.400 <-> 0.000
# (a 0.4 step, ~33 LSB of pedal) 6x per engaged minute, many of the zeros right on the release threshold where lead-speed
# noise alone crosses it. Raised to 0.6/0.25 with a RELEASE_DWELL_S release, then (same day) made continuous: ENTER/EXIT
# now bound a linear ramp of the cap factor (see the module docstring).
ENTER = 0.6   # m/s, v_ego - v_lead at/above this = closing on the lead: a positive ask is fully capped to 0
EXIT = 0.25   # m/s, ...at/below this the cap is fully released (once the dwell window has cleared); linear between
RELEASE_DWELL_S = 0.5   # s: the factor is the minimum raw factor over this window - a release must hold this long

# a lead whose speed is not usable (radarState vLead can be 0 for an unconfirmed track) must NOT read as "closing".
MIN_LEAD_SPEED_VALID = -1.0


def lead_speed_usable(v_lead: float | None) -> bool:
  return v_lead is not None and bool(np.isfinite(v_lead)) and v_lead >= MIN_LEAD_SPEED_VALID


def no_close_factor(v_rel: float) -> float:
  """Raw (un-dwelled) cap factor for a closing speed: 1 at/below EXIT, 0 at/above ENTER, linear between."""
  return float(np.clip((ENTER - v_rel) / (ENTER - EXIT), 0., 1.))


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
    self.factor = 1.      # dwelled cap factor applied on the last frame (1 = no cap)
    self.closing = False  # factor < 1, i.e. some cap is in effect
    self._window: deque[float] = deque(maxlen=max(1, int(round(RELEASE_DWELL_S / dt))))
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

  def update(self, accel: float, v_ego: float, lead_present: bool, v_lead: float | None) -> float:
    if self._frame % int(1. / self.dt) == 0:
      self._read_param()
    self._frame += 1

    if not self.enabled or not lead_present or v_lead is None or not np.isfinite(v_lead):
      self._reset()
      return accel

    # an unusable (but finite) lead speed reads as "not closing" for this frame without flushing the window, so one bad
    # frame cannot release a cap the dwell is holding.
    raw = no_close_factor(float(v_ego - v_lead)) if lead_speed_usable(v_lead) else 1.
    self._window.append(raw)
    self.factor = min(self._window)  # this frame is in the window: tighten is immediate; a rise needs the dwell
    self.closing = self.factor < 1.
    return cap_no_close(accel, v_ego, lead_present, v_lead, enabled=True, factor=self.factor)
