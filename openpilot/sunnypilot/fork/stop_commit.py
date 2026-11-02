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

Stop line (route 185, owner: "braking at empty stop signs and stop lights still overshoots" -- and, told that the fix
would also stop at yields and roundabouts, "I'd rather that then blowing through fucking stop signs")
---------------------------------------------------------------------------------------------------------
``commit_accel`` prices the plan's own stop point, but at the empty stops the plan does not stop: it falls to a creep
plateau (1.6-2.5 m/s) and parks there, so the commitment only paid for reaching the creep speed. The ask stayed at
0.4-0.9 m/s^2 eight seconds out, exploded to 3-4.8 m/s^2 two seconds out, and the car finished 2.5-2.9 m/s^2 past the
plan's own point (measured +3.9 / +4.2 / +7.1 m on route 185). ``stop_line_accel`` recognises that plateau -- the rest
of the plan stays at or below V_CREEP + V_END_TOL, does not climb back, and has stopped braking (V_FLAT_DECEL, which
keeps a real stop that the 10 s horizon truncates on ``commit_accel``'s path) -- and returns the decel that ends the
car AT that point (capped at A_FLOOR so this rule alone is never harsher than -2.5 m/s^2). When it returns a value the
commitment takes the deeper of the two and weights it 1.0: the plan is stopping, the "did it slow down" weight curve is
the wrong question. Everything else -- arming, the no-lead entry gate, J_IN/J_OUT ramps, release, reset -- is unchanged.
This deliberately also stops at a yield/roundabout whose plan parks on a creep plateau; the owner prefers that.

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

# stop line (route 185): the model's plan falls to a CREEP plateau (1.6-2.5 m/s) and parks there instead of planning a
# stop. commit_accel() then only pays for reaching that creep speed, the ask stays shallow, and the car finishes the
# stop past the model's own point with a 2.5-2.9 m/s^2 panic brake. When the plateau is terminal the point IS the
# model's stop location, so the commitment aims at a full stop there instead.
V_CREEP = 3.0            # m/s, a plan that never gets this slow is not planning a stop
V_END_TOL = 0.5          # m/s, the rest of the plan must not climb back above the plateau by more than this
V_RISE_TOL = 0.3         # m/s, ...nor above its own first plateau value by more than this
V_TRUE_STOP = 1.0        # m/s, a plan that reaches this is a real stop plan: commit_accel owns it, this rule stands down
V0_MIN = 4.0             # m/s, below this the car is already crawling / finishing the stop
D_MIN = 3.0              # m, ignore plateau points this close to the car
STOP_MARGIN = 1.0        # m, stop this far short of the plan's plateau point
A_FLOOR = -2.5           # m/s^2, this rule never asks harder than this (the e2e candidate may still go deeper)
V_FLAT_DECEL = 0.35      # m/s^2, the plan must have stopped braking at the plateau by this much (a truncated real stop
                         # keeps braking hard at the horizon end and is left to commit_accel)

# LATCH (2026-11-01, owner: "brakes for lights and stop signs with no leading car still overshoots"): the model's
# terminal creep plateau SLIDES FORWARD as the car approaches it, so a rule that re-prices the plan's point every frame
# keeps aiming at a point that recedes - the car creeps past the intended stop. Measured on the 5 routes of that drive
# (196 qlogs): no-lead stops spend a median 3.4 s (p90 7.4 s, max 17.2 s) below 3 m/s before coming to rest, 45 % of it
# between 0.15 and 1.0 m/s, and there is NO panic braking in the last second (median |a| 0.77, p10 0.98 m/s^2) - a slow
# creep past the point, not a late hard stop. So the FIRST time the plan commits to a terminal plateau inside
# LATCH_MAX_D, its distance is frozen into an odometer position and the rule prices THAT point from then on.
LATCH_MAX_D = 60.0       # m, only latch a plateau this close (a far "plateau" is a curve, not a stop)
LATCH_MIN_D = 2.0        # m, ...and this far
LATCH_RELEASE_V = 2.0    # m/s, a latched stop is released if the car ends up crawling this fast with the point behind it
LATCH_STOPPED_V = 0.2    # m/s, at rest the latch is done



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


def stop_line_distance(x, v, t=None):
  """Decel that ends the car AT the plan's own creep plateau, or None when the plan is not parking on one.

  The plateau is the first plan point at <= V_CREEP. It counts as a stop point (rather than a curve, a slow zone, or a
  real stop that the 10 s horizon simply cuts short) only when
    * the rest of the plan stays at or below V_CREEP + V_END_TOL,
    * it does not climb back above its own first plateau value by more than V_RISE_TOL, and
    * the plan has stopped braking there: the tail's own decel is shallower than V_FLAT_DECEL. A uniform stop plan that
      the horizon truncates (e.g. 17 m/s braking to a stop 100 m out still reads 2.55 m/s at 10 s) keeps braking at
      the tail and is left to commit_accel(), which prices it exactly.
  Returns the plan's own plateau distance ``d`` (metres ahead), or None when the plan is not parking on one.
  """
  x = np.asarray(x, dtype=float)
  v = np.clip(np.asarray(v, dtype=float), 0., None)
  t = np.asarray(ModelConstants.T_IDXS if t is None else t, dtype=float)
  if x.size < 4 or x.size != v.size or t.size != v.size or not (np.all(np.isfinite(x)) and np.all(np.isfinite(v))):
    return None
  v0 = float(v[0])
  if v0 < V0_MIN or float(v.min()) <= V_TRUE_STOP:
    return None
  idx = np.where(v <= V_CREEP)[0]
  if idx.size == 0:
    return None
  i0 = int(idx[0])
  d = float(x[i0])
  if d <= D_MIN:
    return None
  tail = v[i0:]
  tail_dt = float(t[-1] - t[i0])
  if tail.size < 3 or tail_dt < 0.5:               # too little plan left to call it a plateau
    return None
  if float(tail.max()) > V_CREEP + V_END_TOL or float(tail[-1]) > float(tail[0]) + V_RISE_TOL:
    return None
  if (float(tail[-1]) - float(tail[0])) / tail_dt < -V_FLAT_DECEL:
    return None
  return d


def stop_line_accel(x, v, t=None):
  """Decel that ends the car at the plan's plateau point *as measured right now* (the unlatched rule)."""
  d = stop_line_distance(x, v, t)
  if d is None:
    return None
  return float(max(-(float(np.clip(np.asarray(v, dtype=float), 0., None)[0]) ** 2) / (2. * max(d - STOP_MARGIN, 1.0)), A_FLOOR))



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
    self._s = 0.0            # odometer (m) accumulated while long is active - the frame the latch is frozen in
    self.latch_s = None      # absolute odometer position of the latched stop point
    self._latched_done = False   # this stop episode already spent its one latch (do not re-latch at a moving point)
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
    self.latch_s = None
    self._latched_done = False

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
    self._s += max(v_ego, 0.) * self.dt
    self.commit = commit_accel(plan_x, plan_v, t)
    self.weight = stop_weight(plan_v)
    plateau_d = stop_line_distance(plan_x, plan_v)
    if plateau_d is not None and self.latch_s is None and not self._latched_done and LATCH_MIN_D < plateau_d <= LATCH_MAX_D:
      # first sighting of a terminal plateau this close: freeze the point (odometer frame) instead of re-pricing a
      # target that the model slides forward as the car approaches
      self.latch_s = self._s + plateau_d
    if self.latch_s is not None:
      remaining = self.latch_s - self._s - STOP_MARGIN
      if v_ego > LATCH_RELEASE_V and remaining <= 0.:
        self.latch_s = None               # the point is behind us and we are still rolling: it was not a stop
        self._latched_done = True         # ...and the sliding plateau must not re-latch it on the next frame
      elif plateau_d is None and self.weight < 0.05:
        self.latch_s = None               # the plan stopped planning a stop at all (curve / new lead)
        self._latched_done = False        # a later stop in the same engagement may latch again
    if plateau_d is not None or self.latch_s is not None:
      # the plan is parking on a creep plateau: the point is the model's stop location, so price a full stop there and
      # let the commitment do its job (the weight curve asks "did the plan slow down", which is not the question here)
      if self.latch_s is not None:
        remaining = max(self.latch_s - self._s - STOP_MARGIN, 1.0)
        line = float(max(-(v_ego * v_ego) / (2. * remaining), A_FLOOR))
      else:
        line = stop_line_accel(plan_x, plan_v)
      self.commit = min(self.commit, line)
      self.weight = 1.0
    gap = self.commit - e2e
    if not self.armed:
      self.armed = (v_ego >= V_ENTRY and self.lead_clear >= LEAD_CLEAR_FRAMES and self.weight > 0. and gap < -GAP_ENTER)
    elif gap > -GAP_EXIT or self.weight <= 0.:
      self.armed = False

    target = self.weight * min(0., gap) if self.armed else 0.
    self.delta = float(np.clip(target, self.delta - J_IN * self.dt, min(0., self.delta + J_OUT * self.dt)))
    return e2e + self.delta
