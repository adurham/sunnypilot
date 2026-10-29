"""
Fork: no-lead stop-approach commitment (adurham/sunnypilot, route 17c Note 2). Inert unless the ``StopApproachCommit``
param is set.

Why this exists
---------------
On an empty-lane stop (red light, no car ahead) the published ask comes from the e2e candidate, i.e. the vision model's
``action.desiredAcceleration``. That value is the model's ask for the NEXT instant, and on route 17c it eased into the
stop: -0.13...-0.5 m/s^2 down to 17 m/s, -0.6...-0.9 through 17 -> 13 m/s, and -1.0...-1.7 only below ~12 m/s, so the
required decel kept rising as the distance shrank. All three no-lead stops of that drive were finished by the
driver's brake. On the same frames the model's own 10 s trajectory (``modelV2.position`` / ``velocity``) already
planned the stop: plan speed falling to 3-5 m/s from 17 m/s. The plan knew it had to stop; the instantaneous ask had
not committed to it yet. Occupied stops do not have this problem, because the MPC lead candidate commits as soon as
the lead closes.

What it does
------------
``commit_accel(x, v, t)`` is the plan's own distance-averaged requirement: over every plan point k with t_k >= T_MIN
and dx_k > DX_MIN, the constant decel that takes the plan's v_0 to the plan's v_k over the plan's own dx_k,
``min_k (v_k^2 - v_0^2) / (2 dx_k)`` (<= 0). This is the "required-to-stop curve" of the brief, taken from the model's
own stop point. It adds no distance estimate of its own, and the stopping margin is the model's.

``StopCommit.update`` returns ``e2e + delta`` with ``delta <= 0``, so it never raises the ask:

* weight ``w = w_ratio(v_min/v_0) * w_vstop(v_min)``. Both factors are plan-shape predicates: the plan must slow to
  <= R_NONE of its current speed AND to an absolute v_min <= V_STOP_NONE. A curve slowdown of 15 -> 6 m/s is not a
  stop, so it gives w = 0.
* ENTRY gate (arming): long active, no gas, ``v_ego >= V_ENTRY``, no lead (leadOne/leadTwo) for >= LEAD_CLEAR_FRAMES
  consecutive frames, and the gap ``commit - e2e < -GAP_ENTER``. A lead blocks ENTRY only. Once armed, a lead that
  appears mid-stop does not release the commitment: releasing there would ease the brake exactly where the MPC is
  about to take over. The final min() still lets the MPC go deeper. Occupied stops (lead present on the approach)
  never arm, so their path is byte-identical to the deployed build.
* armed target ``w * min(0, commit - e2e)``, followed with a bounded rate (J_IN deepening, J_OUT release, m/s^3), so
  the ask ramps and never steps (the planner-gate-arc rule: the trigger drives a ramp, not a veto or threshold layer).
  The commitment disarms once the gap closes (``commit - e2e > -GAP_EXIT``) or the plan stops planning a stop (w = 0);
  delta then ramps back to 0 at J_OUT.
* reset: long inactive or driver gas -> delta = 0 at once (the planner itself resets on long-off).

Applied in ``LongitudinalPlanner.update`` to the FINAL e2e candidate, i.e. after the far-lead gate, the G4 turn limit
and AccelBoost, immediately before the min() over the MPC / cruise / e2e candidates. An earlier stage therefore cannot
raise it back, and it never touches the MPC or the cruise candidate. With the param off (default), ``update`` returns
its input unchanged (identity), and nothing else in the planner differs.
"""
import numpy as np

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants

PARAM = "StopApproachCommit"

T_MIN = 1.0              # s, ignore plan points nearer than this (the first second is the instantaneous ask itself)
DX_MIN = 1.0             # m, ignore plan points this close to the car (division guard)
R_FULL, R_NONE = 0.25, 0.5          # plan v_min / v_0: full weight at <= 0.25, none at >= 0.5
V_STOP_FULL, V_STOP_NONE = 3.0, 6.0  # m/s, plan v_min: full weight at <= 3, none at >= 6 (curves are not stops)
V_ENTRY = 5.0            # m/s, arm only above this
LEAD_CLEAR_FRAMES = 20   # 1.0 s at 20 Hz with no lead before arming (a flickering lead track cannot arm/disarm it)
GAP_ENTER = 0.3          # m/s^2, arm when the model's instantaneous ask is this much shallower than its own plan
GAP_EXIT = 0.1           # m/s^2, disarm when the gap has closed to within this
J_IN = 1.0               # m/s^3, deepening rate of delta (the single comfort knob; inside the owner's 1.4 reference)
J_OUT = 1.4              # m/s^3, release rate of delta -- the owner's documented 1.4 m/s^3 comfort reference (FORK row 91)


def commit_accel(x, v, t) -> float:
  """Distance-averaged decel of the plan: min over points (t >= T_MIN, dx > DX_MIN) of (v_k^2 - v_0^2) / (2 dx_k),
  clipped to <= 0. 0.0 for an empty / malformed plan."""
  x = np.asarray(x, dtype=float)
  v = np.clip(np.asarray(v, dtype=float), 0., None)
  t = np.asarray(t, dtype=float)
  if x.size < 2 or x.size != v.size or x.size != t.size or not (np.all(np.isfinite(x)) and np.all(np.isfinite(v))):
    return 0.
  dx = x - x[0]
  ok = (t >= T_MIN) & (dx > DX_MIN)
  if not ok.any():
    return 0.
  return float(min(0., ((v[ok] ** 2 - v[0] ** 2) / (2. * dx[ok])).min()))


def stop_weight(v) -> float:
  """Plan-shape weight in [0, 1]: does the plan slow to a stop (relative AND absolute)?"""
  v = np.clip(np.asarray(v, dtype=float), 0., None)
  if v.size == 0 or not np.all(np.isfinite(v)):
    return 0.
  v_min = float(v.min())
  ratio = v_min / max(float(v[0]), 0.5)
  return float(np.interp(ratio, [R_FULL, R_NONE], [1., 0.]) * np.interp(v_min, [V_STOP_FULL, V_STOP_NONE], [1., 0.]))


class StopCommit:
  """Owned by LongitudinalPlanner. ``update`` returns the (possibly deepened) e2e candidate."""

  def __init__(self, params=None, dt: float = DT_MDL):
    self.dt = dt
    self._params = params
    self._frame = 0
    self.enabled = False
    self.armed = False
    self.delta = 0.
    self.commit = 0.
    self.weight = 0.
    self.lead_clear = 0
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

  def reset(self) -> None:
    self.armed = False
    self.delta = 0.

  def update(self, e2e: float, long_active: bool, gas_pressed: bool, v_ego: float, lead_present: bool,
             plan_x, plan_v, plan_t=None) -> float:
    if self._frame % int(1. / self.dt) == 0:
      self._read_param()
    self._frame += 1

    self.lead_clear = 0 if lead_present else self.lead_clear + 1
    if not self.enabled:
      self.reset()
      return e2e
    if not long_active or gas_pressed:
      self.reset()
      return e2e

    t = ModelConstants.T_IDXS if plan_t is None or len(plan_t) != len(plan_x) else plan_t
    self.commit = commit_accel(plan_x, plan_v, t)
    self.weight = stop_weight(plan_v)
    gap = self.commit - e2e
    if not self.armed:
      self.armed = (v_ego >= V_ENTRY and self.lead_clear >= LEAD_CLEAR_FRAMES and self.weight > 0. and gap < -GAP_ENTER)
    elif gap > -GAP_EXIT or self.weight <= 0.:
      self.armed = False

    target = self.weight * min(0., gap) if self.armed else 0.
    self.delta = float(np.clip(target, self.delta - J_IN * self.dt, min(0., self.delta + J_OUT * self.dt)))
    return e2e + self.delta
