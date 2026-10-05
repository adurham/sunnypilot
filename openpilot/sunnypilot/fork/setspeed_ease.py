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


def personality_id(personality) -> int:
  """selfdriveState.personality is a capnp _DynamicEnum, which equals the int but does NOT hash like it (a dict lookup
  with it misses). Normalize to the raw int."""
  return int(getattr(personality, 'raw', personality))


def get_rate(personality, v_ego: float) -> float:
  table = RATE_V.get(personality_id(personality), RATE_V[Personality.standard])  # unknown -> standard
  return float(np.interp(v_ego, RATE_BP, table))


class SetSpeedEase:
  """Owned by LongitudinalPlannerSP. ``update`` returns the eased cruise speed for get_cruise_accel."""

  def __init__(self, dt: float = DT_MDL):
    self.dt = dt
    self.v_ease: float | None = None
    self.launching = False
    self._low_accel_t = 0.
    self.ramping = False

  def step(self, v_target: float, v_ego: float, personality, active: bool, a_plan: float = 0.) -> float:
    """Pure update. active = longitudinal engaged and the driver not overriding; a_plan = last plan accel."""
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

    if self.launching:
      v_ease = v_target                                         # shipped launch behaviour
    elif self.v_ease is None:
      v_ease = min(v_ego, v_target)
    else:
      rate = get_rate(personality, v_ego)
      v_ease = min(self.v_ease + rate * self.dt, v_ego + rate * LEASH_S)  # ramp, leashed to the car
      v_ease = max(v_ease, min(v_ego, v_target))                # never below the car (no easing-made slowdown)
    self.v_ease = min(v_ease, v_target)                         # down is immediate; never above the target
    return self.v_ease

  def update(self, sm, v_target: float, long_enabled: bool, long_override: bool, a_plan: float) -> float:
    cs = sm['carState']
    active = long_enabled and not long_override and sm['controlsState'].longControlState != LongCtrlState.off
    personality = sm['selfdriveState'].personality
    v = self.step(v_target, cs.vEgo, personality, active, a_plan)
    ramping = active and (v_target - v) > RAMP_LOG_MIN_GAP
    if ramping != self.ramping:
      self.ramping = ramping
      cloudlog.event("setspeed_ease", ramping=ramping, v_ego=round(float(cs.vEgo), 2), v_target=round(float(v_target), 2),
                     v_ease=round(float(v), 2), personality=personality_id(personality))
    return v
