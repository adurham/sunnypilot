"""
Fork: personality-dependent set-speed easing (adurham/sunnypilot).

Why this exists (car-features/drive-133-134-report.md §A, routes 00000133 / 00000134)
---------------------------------------------------------------------------------------
Raising the set speed (+10 mph press, Speed Limit Assist raising the limit, a long-press-down that sets the current
speed, an SCC curve target releasing, a lead pulling away) makes the planner chase the new speed at once. The cruise
candidate in ``longitudinal_planner.get_cruise_accel`` is ``clip(v_cruise - v_ego, A_CRUISE_MIN, max_accel)``: a P term
on the speed error (gain 1/s) with a personality-INDEPENDENT ceiling (``A_CRUISE_MAX_VALS`` 1.6/1.2/0.8/0.6 at
0/10/25/40 m/s; ``ACCEL_MAX`` = 2.0 in e2e). Any error over ~1 m/s asks for the ceiling, held flat until the car is
within ~1 m/s of the target (134 @157: 40 -> 80 mph, request 1.0-1.2 m/s^2, pedal at its 0.35 cap for ~20 s).
``LongitudinalPersonality`` only changes the MPC follow gap / jerk weights, so it does not change this at all. The
owner: "just because I raised max speed 10 mph doesn't mean I need to be going that speed right away; it's robotic".

What it does
------------
The cruise speed handed to ``get_cruise_accel`` is replaced by an eased speed ``v_ease``:

* Ramp: ``v_ease`` rises toward the target at ``rate(v_ego, personality)`` m/s per s. The cruise term is a 1/s P
  controller on ``v_ease - v_ego``, so the request builds up over ~1 s (no step), settles near ``rate`` and tapers into
  the new speed.
* Leash: ``v_ease <= v_ego + rate * LEASH_S``. The cruise request can never exceed ~``LEASH_S * rate`` (car lagging on
  a hill / at the pedal cap, or the car was held below the target by a lead or a curve and the constraint goes away).
* DOWN is immediate: ``v_ease <= target`` always (down press, long-press-down to current speed, SLA lowering, SCC
  curve slowdown, forceDecel -> 0).
* Never a slowdown: ``v_ease >= min(v_ego, target)``. So the eased cruise request lies between ``min(0, upstream)``
  and ``upstream``: it can lower a POSITIVE request but never creates or deepens a decel. The final plan is the min over
  the MPC (lead) / cruise / e2e candidates, so lead braking, lead-follow decel, curve slowdowns and stops are never
  weaker than upstream.
* While longitudinal is not engaged or the driver overrides with the gas, the target is passed through unchanged
  (identical to upstream) and the ramp is re-armed at the current speed, so an engage at a stored set speed ramps
  from the current speed.
* Launch latch: engaged below ``LAUNCH_V`` (stop / crawl), the cruise target is passed through unchanged (exactly the
  shipped launch) until the launch is over: the car reaches the target (``LAUNCH_DONE_MARGIN``) or the plan has asked
  for less than ``LAUNCH_END_A`` for ``LAUNCH_END_S`` (e.g. it settled behind a lead). At that point the ramp restarts
  from the current speed.
* Merge gate (``MergeGate``): easing is for DISCRETIONARY speed-ups (a set-speed bump, an SLA raise, a lead pulling away
  while the car is already at road speed). It steps aside, and the target is passed through exactly as upstream, while
  the car is well below a highway-class speed: an on-ramp / merge / catching up to highway traffic is not discretionary.
  Reference speed, from signals the fork already has (no new capnp field):
    - highway evidence (max of): ``liveMapDataSP.roadType`` highway/interstate (its limit, or 55 mph without one); a
      valid map limit >= 55 mph; the car's own limit (``carStateSP.speedLimit``, the cluster / camera sign) >= 55 mph;
      a valid map limit AHEAD >= 55 mph within ``MERGE_AHEAD_DIST`` (an on-ramp is often an unnamed OSM link without a
      limit; the highway's limit is the next one). 55 mph = ``road_type_classifier.HIGHWAY_SPEED_THRESHOLD``, the same
      threshold the fork's road-type classifier uses. ``ref = min(highway speed, set speed)``; merging while
      ``v_ego < ref - MERGE_DEFICIT`` (15 mph). So the gate needs BOTH a highway-class road AND a target >= 15 mph
      above the car: a +5/+10 mph bump never trips it.
    - else positive non-highway evidence (roadType urban, or a valid map / car limit under 55 mph) and NOT a valid
      highway-class limit within ``MERGE_AHEAD_DIST`` ahead: no merge, ease. (A >= 55 mph limit within 500 m ahead
      deliberately overrides ``roadType`` urban: approaching a highway from a surface street / frontage road IS the
      on-ramp case, so this is checked before the urban exclusion below.)
    - else NO data at all (fail safe): the set speed stands in for the road's speed when it is highway-class: merging
      while ``v_ego < v_set - NODATA_DEFICIT`` (25 mph; a +10 mph bump never reaches that).
  ``carStateSP.speedLimit`` staleness (known, benign): the car's own limit (the Hyundai cluster / camera sign) is a
  LAST-SEEN display value, read live from a CAN signal; ``CarStateSP`` carries only a bare ``Float32`` (no validity
  bit, no timestamp), so it has no freshness channel and it persists after the car leaves the sign. A lone stale
  >= 55 mph reading can therefore keep the gate active on a normal road while the car is > 15 mph below a
  highway-class set speed. The effect is benign by construction: the gate only SUPPRESSES easing, i.e. restores
  exact upstream behaviour; it never adds acceleration or weakens braking. A freshness guard was investigated and
  rejected: it cannot be built from the signals available (no age / valid field on the message) and an
  evidence-veto version would suppress the gate on the real 134 on-ramp merge (``roadType`` reads ``urban`` at merge
  start), breaking the "never suppress a real merge" constraint. Proper remediation is upstream: add a validity /
  age field to ``CarStateSP.speedLimit``. (The map limit is read directly from ``liveMapDataSP`` -- with its
  ``speedLimitValid`` -- bypassing the resolver's GPS-age gate, so a frozen valid map limit can suppress easing the
  same benign way.)
  The set speed (``v_cruise`` before the SCC / SLA arbitration) is used, not the arbitrated target: a curve on the ramp
  (SCC) dipping the target must not end the merge mid-ramp. While merging the ARBITRATED target is passed through, so
  a curve / SLA still limits exactly as upstream.
  Hysteresis ``MERGE_HYST`` on the speed boundary, and a ``MERGE_REF_HOLD_S`` debounce on the reference boundary (a
  flapping ``roadType`` / limit-valid must not toggle ``merging`` tick-to-tick; holding suppresses easing slightly
  longer, the benign direction). While merging ``v_ease`` = target; on leaving, the leash hands over at
  ``v_ego + rate * LEASH_S``, so the request steps down to ~1.5 x rate (close to the upstream ceiling at highway
  speed), not to zero, and then tapers in at the personality rate.
  ``SetSpeedEase.merge_state`` exposes the gate so the planner can hand the SAME tick's result to the SCC-V exclusion
  (``fork/scc.py``): while merging, a predicted-only SCC-V (no current-lateral evidence) is released so it cannot bind
  the arbitrated target during the merge. Evaluating the gate once per tick keeps the debounce honest.
* Personality is the ONLY input that changes the feel: relaxed gentlest, standard middle, aggressive ~= upstream.
  When the drive-mode CAN signal is decoded, Eco/Normal/Sport -> relaxed/standard/aggressive feed this same input.

Scope: only the cruise candidate. The MPC lead candidate and the e2e model candidate are untouched (a closer lead
always wins the min). An accelerating lead is followed at up to the eased cruise request; a lead pulling away faster
than that is not chased as hard (that is the intent). ``longitudinalPlanSP.vTarget`` keeps the raw target (ICBM and
the UI are unchanged).

Placement: one hook in ``LongitudinalPlannerSP.update_targets`` (return value) + construction in ``__init__``.
Upstream ``longitudinal_planner.py`` / ``get_cruise_accel`` / ``cruise.py`` are untouched.
``fork/tests/test_setspeed_ease.py::TestUpstreamHooks`` pins ``update_targets`` / ``get_cruise_accel`` and checks the hook
end to end through the real ``LongitudinalPlanner``.
"""
import numpy as np

from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.sunnypilot.mapd.lib.road_type_classifier import HIGHWAY_SPEED_THRESHOLD, RoadType

Personality = log.LongitudinalPersonality

# ramp rate (m/s per s ~= the settled cruise accel request, m/s^2) vs v_ego (m/s), per personality.
# Upstream ceiling for comparison (ACC): A_CRUISE_MAX 1.2 @ 10, 0.8 @ 25, 0.6 @ 40 m/s; e2e 2.0.
RATE_BP = [10., 20., 29.]
RATE_V = {
  Personality.relaxed: [0.55, 0.45, 0.33],
  Personality.standard: [0.75, 0.60, 0.40],
  Personality.aggressive: [1.20, 0.95, 0.80],
}
LEASH_S = 1.5             # s; v_ease <= v_ego + rate * LEASH_S
LAUNCH_V = 3.0            # m/s; engaged below this = a launch (not eased)
LAUNCH_DONE_MARGIN = 1.0  # m/s; launch over once v_ego >= target - this
LAUNCH_END_A = 0.3        # m/s^2; ... or once the plan asked for less than this
LAUNCH_END_S = 1.0        # s ... for this long
RAMP_LOG_MIN_GAP = 1.0    # m/s; target - v_ease above this = "ramping" (diagnostics event on each edge)

MPH = 0.44704
# merge gate (see the module doc): pass the target through while the car is well below a highway-class speed
MERGE_DEFICIT = 15 * MPH   # m/s; merging while v_ego < highway reference - this (owner's bumps are +5/+10 mph)
NODATA_DEFICIT = 25 * MPH  # m/s; no map / car limit at all: merging while v_ego < v_target - this (highway-class target)
MERGE_HYST = 3 * MPH       # m/s; leave the merge only once v_ego >= ref - (deficit - this)
MERGE_REF_HOLD_S = 0.5     # s; ref-boundary debounce: a ref that drops away must stay away this long before easing
                           # resumes (a flapping roadType / limit-valid must not toggle merging tick-to-tick). Keeping the
                           # gate up suppresses easing = upstream behaviour, the benign direction.
MERGE_AHEAD_DIST = 500.    # m; a highway-class map limit this close ahead counts (on-ramp onto it)
HIGHWAY_ROAD_TYPES = ('highway', 'interstate')
_ROAD_TYPE_NAMES = {v: k for k, v in RoadType.schema.enumerants.items()}  # int -> name


def road_type_name(road_type) -> str:
  """liveMapDataSP.roadType arrives as a capnp enum (str() = name), classify_road_type returns the raw int, tests use
  text. Normalize to the name; anything unrecognised -> 'unknown' (the fail-safe branch)."""
  if isinstance(road_type, int):
    return _ROAD_TYPE_NAMES.get(road_type, 'unknown')
  return str(road_type)


def personality_id(personality) -> int:
  """selfdriveState.personality is a capnp _DynamicEnum, which equals the int but does NOT hash like it (a dict lookup
  with it misses). Normalize to the raw int. Anything that does not cast (a renamed / non-numeric enum) -> standard."""
  try:
    return int(getattr(personality, 'raw', personality))
  except (TypeError, ValueError):
    return int(Personality.standard)


def get_rate(personality, v_ego: float) -> float:
  # an unknown personality (a value cereal adds later) gets the STANDARD table: the middle, never the un-eased one
  table = RATE_V.get(personality_id(personality), RATE_V[Personality.standard])
  return float(np.interp(v_ego, RATE_BP, table))


def merge_reference(road_type, map_limit: float, map_limit_valid: bool, ahead_limit: float, ahead_valid: bool,
                    ahead_dist: float, car_limit: float, v_set: float) -> tuple[float, float]:
  """(reference speed, deficit) for the merge gate; (0, 0) = no merge context. All speeds m/s. See the module doc.
  v_set = the cruise set speed (before the SCC curve / SLA arbitration: a curve on the ramp must not end the merge)."""
  map_limit = map_limit if map_limit_valid and map_limit > 0. else 0.
  car_limit = car_limit if car_limit > 0. else 0.
  hw = 0.
  road_type = road_type_name(road_type)
  if road_type in HIGHWAY_ROAD_TYPES:
    hw = map_limit if map_limit > 0. else HIGHWAY_SPEED_THRESHOLD
  for lim in (map_limit, car_limit):
    if lim >= HIGHWAY_SPEED_THRESHOLD:
      hw = max(hw, lim)
  if ahead_valid and ahead_limit >= HIGHWAY_SPEED_THRESHOLD and 0. <= ahead_dist <= MERGE_AHEAD_DIST:
    hw = max(hw, ahead_limit)
  if hw > 0.:
    # capped at the driver's set speed: the car never needs to "merge" past what the driver asked for, so a +5/+10 mph
    # bump on a highway driven below its limit (traffic, weather) is still eased
    return min(hw, v_set), MERGE_DEFICIT
  if road_type == 'urban' or map_limit > 0. or car_limit > 0.:
    return 0., 0.                                             # positively a non-highway road: ease
  if v_set >= HIGHWAY_SPEED_THRESHOLD:
    return v_set, NODATA_DEFICIT                              # no data at all: fail safe toward not easing
  return 0., 0.


class MergeGate:
  """True while the car is merging onto / catching up to a highway-class speed (easing steps aside). Speed hysteresis
  (``MERGE_HYST``) plus a reference-hold debounce (``MERGE_REF_HOLD_S``) so a flapping ref cannot toggle ``merging``
  tick-to-tick; both only ever keep the gate UP (suppress easing = upstream behaviour), never down."""

  def __init__(self):
    self.merging = False
    self._no_ref_t = 0.

  def reset(self) -> None:
    self.merging = False
    self._no_ref_t = 0.

  def update(self, ref: float, deficit: float, v_ego: float, dt: float = DT_MDL) -> bool:
    if ref <= 0.:
      # the highway reference dropped away: hold the gate briefly. A flapping roadType / limit-valid must not hand the
      # target back to the ease every other tick (holding suppresses easing = upstream behaviour, the benign direction).
      self._no_ref_t += dt
      if self._no_ref_t >= MERGE_REF_HOLD_S:
        self.merging = False
    else:
      self._no_ref_t = 0.
      if self.merging:
        self.merging = v_ego < ref - (deficit - MERGE_HYST)
      else:
        self.merging = v_ego < ref - deficit
    return self.merging

  def update_sm(self, sm, v_set: float, v_ego: float, dt: float = DT_MDL) -> bool:
    lm = sm['liveMapDataSP']
    ref, deficit = merge_reference(lm.roadType, lm.speedLimit, lm.speedLimitValid, lm.speedLimitAhead, lm.speedLimitAheadValid,
                                   lm.speedLimitAheadDistance, sm['carStateSP'].speedLimit, v_set)
    return self.update(ref, deficit, v_ego, dt)


class SetSpeedEase:
  """Owned by LongitudinalPlannerSP. ``update`` returns the eased cruise speed for get_cruise_accel."""

  def __init__(self, dt: float = DT_MDL):
    self.dt = dt
    self.v_ease: float | None = None
    self.launching = False
    self._low_accel_t = 0.
    self.ramping = False
    self.merge_gate = MergeGate()
    self.merging = False

  def step(self, v_target: float, v_ego: float, personality, active: bool, a_plan: float = 0., merging: bool = False) -> float:
    """Pure update. active = longitudinal engaged and the driver not overriding; a_plan = last plan accel;
    merging = MergeGate output (target passed through, like a launch)."""
    if not active:
      # not engaged / overriding: pass the target through (identical to upstream) and arm the ramp at the current speed
      self.launching = False
      self.v_ease = min(v_ego, v_target)
      return v_target

    if v_ego < LAUNCH_V:
      self.launching = True
      self._low_accel_t = 0.
    elif self.launching:
      self._low_accel_t = self._low_accel_t + self.dt if a_plan < LAUNCH_END_A else 0.
      if v_ego >= v_target - LAUNCH_DONE_MARGIN or self._low_accel_t >= LAUNCH_END_S:
        self.launching = False
        self.v_ease = None                                      # launch over: ramp from here, not from the target

    if self.launching or merging:
      # shipped launch / merge behaviour (raw target). After a merge the ramp's own leash hands over at
      # v_ego + rate * LEASH_S: the request steps down to ~1.5 x rate, never to zero
      v_ease = v_target
    elif self.v_ease is None:
      v_ease = min(v_ego, v_target)
    else:
      rate = get_rate(personality, v_ego)
      v_ease = min(self.v_ease + rate * self.dt, v_ego + rate * LEASH_S)  # ramp, leashed to the car
      v_ease = max(v_ease, min(v_ego, v_target))                # never below the car (no easing-made slowdown)
    self.v_ease = min(v_ease, v_target)                         # down is immediate; never above the target
    return self.v_ease

  def merge_state(self, sm, v_set: float, v_ego: float) -> bool:
    """Evaluate the merge gate ONCE for this tick. Shared by the SCC-V exclusion (fork/scc.py) and the ease, so the
    ref-boundary debounce is not advanced twice per tick. Also resets the gate while not in control."""
    active = sm['carControl'].enabled and not sm['carControl'].cruiseControl.override and \
             sm['controlsState'].longControlState != LongCtrlState.off
    merging = active and self.merge_gate.update_sm(sm, v_set, v_ego, self.dt)
    if not active:
      self.merge_gate.reset()
    return merging

  def update(self, sm, v_target: float, long_enabled: bool, long_override: bool, a_plan: float,
             v_set: float | None = None, merging: bool | None = None) -> float:
    """v_target = arbitrated target (cruise / SCC / SLA min); v_set = the cruise set speed (merge-gate cap);
    merging = the merge-gate result already evaluated this tick by the planner (None = evaluate here, standalone use)."""
    cs = sm['carState']
    v_set = v_target if v_set is None else max(v_set, v_target)
    active = long_enabled and not long_override and sm['controlsState'].longControlState != LongCtrlState.off
    personality = sm['selfdriveState'].personality
    if merging is None:
      merging = active and self.merge_gate.update_sm(sm, v_set, cs.vEgo, self.dt)
    if not active:
      self.merge_gate.reset()
    v = self.step(v_target, cs.vEgo, personality, active, a_plan, merging)
    ramping = active and (v_target - v) > RAMP_LOG_MIN_GAP
    if ramping != self.ramping or merging != self.merging:
      self.ramping, self.merging = ramping, merging
      cloudlog.event("setspeed_ease", ramping=ramping, merging=merging, v_ego=round(float(cs.vEgo), 2),
                     v_target=round(float(v_target), 2), v_ease=round(float(v), 2), personality=personality_id(personality))
    return v
