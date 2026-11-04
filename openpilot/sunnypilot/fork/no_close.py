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
A last-stage cap on the PUBLISHED accel request: while a lead is present and the ego is closing on it
(``v_ego - v_lead > ENTER``), the ask may not be positive. The car approaches by holding its speed or coasting, never by
speeding up. It never deepens a brake request (the cap is ``min(accel, 0)``), it does not touch a matched or opening
gap, and it releases with hysteresis (``EXIT``) so a lead hovering at the same speed cannot toggle it.

Applied after the ``min()`` over the MPC / cruise / e2e candidates, so no candidate can raise it back.
"""
import numpy as np

from openpilot.common.realtime import DT_MDL

PARAM = "NoCloseOnLead"

# 2026-11-03 (route 191, the owner's "throttle fluttering"): with 0.3/0.1 the cap toggled 6x per engaged minute - the
# ask flipped 0.400 <-> 0.000 (a 0.4 step, ~33 LSB of pedal) and 39 of the zeros sat at a lead speed difference of
# 0.1 m/s or less, i.e. right on the release threshold, where radar noise alone can cross it. Raised to 0.6/0.25 AND a
# release now needs the gap to stay matched for RELEASE_DWELL_S, so a lead hovering at the threshold cannot square-wave
# the pedal. The cap is still exactly min(accel, 0) - it never deepens a brake request.
ENTER = 0.6   # m/s, v_ego - v_lead above this = closing on the lead
EXIT = 0.25   # m/s, ...and below this the cap may release once it has stayed there for the dwell
RELEASE_DWELL_S = 0.5   # s: a release needs EXIT held this long, not one noisy frame

# a lead whose speed is not usable (radarState vLead can be 0 for an unconfirmed track) must NOT read as "closing".
MIN_LEAD_SPEED_VALID = -1.0


def cap_no_close(accel: float, v_ego: float, lead_present: bool, v_lead: float | None, closing: bool,
                 enabled: bool = True) -> float:
  """The published ask, capped at 0 while closing on a present lead."""
  if not enabled or not lead_present or accel <= 0.:
    return accel
  if v_lead is None or not np.isfinite(v_lead) or v_lead < MIN_LEAD_SPEED_VALID:
    return accel
  if closing:
    return 0.
  return accel


class NoCloseCap:
  """Owned by LongitudinalPlanner. ``update`` returns the (possibly capped) final ask."""

  def __init__(self, params=None, dt: float = DT_MDL):
    self.dt = dt
    self._params = params
    self._frame = 0
    self.enabled = False
    self.closing = False
    self._release_frames = 0
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

  def update(self, accel: float, v_ego: float, lead_present: bool, v_lead: float | None) -> float:
    if self._frame % int(1. / self.dt) == 0:
      self._read_param()
    self._frame += 1

    if not self.enabled or not lead_present or v_lead is None or not np.isfinite(v_lead):
      self.closing = False
      return accel

    v_rel = float(v_ego - v_lead)
    if not (self.closing or v_rel > ENTER):
      self.closing = False
      return cap_no_close(accel, v_ego, lead_present, v_lead, False, enabled=True)
    # engaged: stay engaged until the gap has been matched for RELEASE_DWELL_S (not one noisy frame)
    if v_rel > EXIT:
      self._release_frames = 0
      self.closing = True
    else:
      self._release_frames += 1
      if self._release_frames >= max(1, int(round(RELEASE_DWELL_S / self.dt))):
        self.closing = False
        self._release_frames = 0
    return cap_no_close(accel, v_ego, lead_present, v_lead, self.closing, enabled=True)
