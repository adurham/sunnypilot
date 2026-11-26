"""
Fork: earlier planner braking for lead-follow / stop approaches (adurham/sunnypilot).

Why this exists
---------------
The owner's throttle-only Elantra N has no brake actuator below the FCA11 friction-brake gate (~-0.5 m/s^2), so the
first part of every slowdown must be covered by engine braking / coasting (~-0.30..-0.36 m/s^2). The measured problem on
routes 1ba/1bb (F-DRIVE-REVIEW-launch-brake-2026-11-25.md) is not a short TTC (first brake frame at TTC 14.6 s) but a
coast after the planner first asks to slow: the ask goes negative 1.34 s before the lead brakes, then the car coasts at
-0.21 m/s^2 for ~1.7 s waiting for the ask to reach the -0.5 m/s^2 gate. The FCA11 brake map itself cannot be lowered:
a previous lane proved the ESC's minimum bite is 0.40 m/s^2, so any friction-brake overshoots a small deficit. The only
remaining lever is the planner asking to slow a little earlier, so the approach spends longer on engine braking and
reaches a bite-sized deficit sooner.

What this module does
---------------------
Adds a fixed time-gap offset to the MPC follow distance when the ``PlannerEarlierBraking`` param is ON. The offset is
applied AFTER ``AdaptiveFollow`` (so it does not fight the existing adaptive-follow tuning) and is passed through to
the e2e far-lead gate so both the MPC and the gate see the same larger desired distance. The result is a more
conservative lead-follow / stop-approach plan: the ask goes negative earlier and reaches the -0.5 m/s^2 friction-brake
threshold sooner, giving engine braking more of the first part of the slowdown.

Scope / containment
-------------------
* Moves: MPC time gap (``t_follow`` parameter), e2e far-lead gate ``d_des`` (to keep gate thresholds consistent with the
  new MPC distance), and therefore the timing of the lead/stop-approach candidate.
* Does NOT move: FCA11 brake map, panda, engine-braking physics, ACCEL_MIN/MAX, AccelBoost, NoClose cap, standstill
  launch hold, stop-commit, min-gap floor, SCC/SLA cruise candidate arbitration, or the model's raw e2e wish.
* Default OFF: the param is read once per second; when OFF or missing the offset is zero and the planner is bit-identical
  to the pre-change path.
"""
from openpilot.common.realtime import DT_MDL

PARAM = "PlannerEarlierBraking"

# Time-gap offset added to the MPC's follow distance when the param is ON. Sized on 1ba/1bb so the ask reaches the
# -0.5 m/s^2 friction-brake gate roughly 0.5-0.7 s earlier on a highway lead-braking approach while keeping the steady
# highway gap reasonable (stock standard 1.45 s; adaptive normal already runs ~1.0-1.25 s on the highway). 0.5 s is a
# conservative first step: it is large enough to matter, small enough not to turn every follow into a distant parade.
EXTRA_T_FOLLOW = 0.5  # s

READ_PERIOD_FRAMES = int(1.0 / DT_MDL)  # re-check param once per second at 20 Hz


class PlannerEarlierBraking:
  """Default-OFF wrapper that adds EXTRA_T_FOLLOW to the MPC/gate time gap when enabled."""

  def __init__(self, params=None, dt: float = DT_MDL):
    self._params = params
    self._frame = 0
    self._period = int(1.0 / dt)
    self.enabled = False
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
    except Exception:  # UnknownKeyName on an older libparams: feature stays OFF
      self.enabled = False

  def adjust(self, t_follow: float | None) -> float | None:
    """Return the effective time gap for the MPC (and the e2e gate). OFF -> identity; None stays None."""
    if self._frame % self._period == 0:
      self._read_param()
    self._frame += 1
    if t_follow is None:
      return None
    return t_follow + (EXTRA_T_FOLLOW if self.enabled else 0.0)
