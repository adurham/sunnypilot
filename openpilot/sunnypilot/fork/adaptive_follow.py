"""
Fork: adaptive follow distance (adurham/sunnypilot). Inert unless the ``AdaptiveFollowDistance`` param is set.

Why this exists
---------------
openpilot's longitudinal MPC follows a lead at a fixed time gap per personality (``get_T_FOLLOW``: relaxed 1.75 s,
standard 1.45 s, aggressive 1.25 s), whatever the road or speed. The owner's car (2022 Elantra N, comma-pedal
throttle-only longitudinal, NO brake actuator, camera-only leads) wants more room at interstate speed, behind trucks
included, but not more in town. On top of that, a throttle-only car can only shed speed at engine-braking rates
(~0.3-0.5 m/s^2), so the MPC's assumption that a closing gap can be fixed with up to ``COMFORT_BRAKE`` (2.5 m/s^2) is
optimistic for it.

What it does
------------
``t_follow = get_T_FOLLOW(personality) * road_speed_factor(road_type, v_ego) + closing_margin(...)``, rate-limited.

* road/speed factor: 1.0 in urban (no change in town; shortening the gap on a car that cannot brake is the wrong
  direction). On highway/interstate it ramps 1.0 -> 1.2 between 20 and 29 m/s (45 -> 65 mph). On unknown road type
  it ramps 1.0 -> 1.1 on speed alone. modelV2 leads carry no class/size, so trucks cannot be singled out. The bump
  applies to every lead at highway speed.
* closing margin (throttle-only cars only): extra distance needed to cancel the closing speed by coasting instead of
  braking at ``COMFORT_BRAKE``: ``vrel^2/2 * (1/a_coast(v) - 1/COMFORT_BRAKE)``, as time at v_ego, capped at
  ``MAX_CLOSING_MARGIN``. It only acts while the lead is slower. A same-speed lead that brakes hard gets no extra margin
  from this; that case still relies on FCW + the driver.

The time gap is a runtime MPC parameter (p[4]), so the OCP and the generated solver are unchanged.
"""
import numpy as np

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import COMFORT_BRAKE, get_T_FOLLOW

PARAM = "AdaptiveFollowDistance"

# road-type x speed multiplier on the personality T_FOLLOW (vEgo m/s)
HIGHWAY_FACTOR_BP = [20., 29.]
HIGHWAY_FACTOR_V = [1.0, 1.2]
UNKNOWN_FACTOR_V = [1.0, 1.1]
HIGHWAY_ROAD_TYPES = ("highway", "interstate")

# Throttle-only flat-ground coast decel magnitude (m/s^2, vEgo m/s). Measured on routes 00000127/00000128 with the
# pedal command at 0 for >= 1 s, grade-corrected medians: 0.32 @ 10-18, 0.35 @ 18-24, 0.41 @ 24-30, 0.50 @ 30-36 m/s.
# Taken ~15 % below the medians, since a downgrade eats coast decel (no trustworthy pitch in plannerd: carControl
# orientationNED[1] reads +0.028 rad vs GPS grade on this car).
COAST_DECEL_BP = [10., 18., 24., 30., 36.]
COAST_DECEL_V = [0.27, 0.30, 0.35, 0.42, 0.42]
MAX_CLOSING_MARGIN = 0.6  # s

# rate limits on the output (s of gap per s): add margin fast, give it back slowly
T_FOLLOW_RATE_UP = 1.0
T_FOLLOW_RATE_DOWN = 0.2


def is_throttle_only(CP, CP_SP) -> bool:
  """Hyundai comma-pedal longitudinal has no brake actuator (lift = coast). Other interceptor brands keep brakes."""
  return bool(CP_SP.enableGasInterceptor) and CP.brand == "hyundai"


def road_speed_factor(road_type: str, v_ego: float) -> float:
  if road_type in HIGHWAY_ROAD_TYPES:
    return float(np.interp(v_ego, HIGHWAY_FACTOR_BP, HIGHWAY_FACTOR_V))
  if road_type == "unknown":
    return float(np.interp(v_ego, HIGHWAY_FACTOR_BP, UNKNOWN_FACTOR_V))
  return 1.0  # urban


def coast_decel(v_ego: float) -> float:
  return float(np.interp(v_ego, COAST_DECEL_BP, COAST_DECEL_V))


def closing_margin(v_ego: float, v_lead: float) -> float:
  """Extra time gap (s) for a throttle-only car to cancel the closing speed by coasting."""
  v_rel = v_ego - v_lead
  if v_rel <= 0. or v_ego <= 1.:
    return 0.
  extra_dist = v_rel ** 2 / 2. * (1. / coast_decel(v_ego) - 1. / COMFORT_BRAKE)
  return float(np.clip(extra_dist / v_ego, 0., MAX_CLOSING_MARGIN))


def get_adaptive_t_follow(personality, v_ego: float, road_type: str, lead_present: bool, v_lead: float,
                          throttle_only: bool) -> float:
  """Target time gap before rate limiting."""
  t_follow = get_T_FOLLOW(personality) * road_speed_factor(road_type, v_ego)
  if throttle_only and lead_present:
    t_follow += closing_margin(v_ego, v_lead)
  return t_follow


class AdaptiveFollow:
  """Owned by LongitudinalPlannerSP. ``update`` returns None when disabled (the MPC then uses its stock T_FOLLOW)."""

  def __init__(self, throttle_only: bool, params=None, dt: float = DT_MDL):
    self.throttle_only = throttle_only
    self.dt = dt
    self._params = params
    self._frame = 0
    self.enabled = False
    self.t_follow: float | None = None
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

  def update(self, personality, v_ego: float, road_type: str, lead_present: bool, v_lead: float) -> float | None:
    if self._frame % int(1. / self.dt) == 0:
      self._read_param()
    self._frame += 1

    if not self.enabled:
      self.t_follow = None
      return None

    target = get_adaptive_t_follow(personality, v_ego, road_type, lead_present, v_lead, self.throttle_only)
    if self.t_follow is None:
      self.t_follow = target
    else:
      self.t_follow = float(np.clip(target, self.t_follow - T_FOLLOW_RATE_DOWN * self.dt,
                                    self.t_follow + T_FOLLOW_RATE_UP * self.dt))
    return self.t_follow

