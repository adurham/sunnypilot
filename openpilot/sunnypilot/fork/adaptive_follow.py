"""
Fork: adaptive follow distance (adurham/sunnypilot). Inert unless the ``AdaptiveFollowDistance`` param is set.

Why this exists
---------------
openpilot's longitudinal MPC follows a lead at a fixed time gap per personality (``get_T_FOLLOW``: relaxed 1.75 s,
standard 1.45 s, aggressive 1.25 s), whatever the road or speed. The owner's car (2022 Elantra N, comma-pedal
throttle-only longitudinal, NO brake actuator, camera-only leads) does not want a personality ladder at all: a
shorter "aggressive" gap is the wrong trade for a car that can only shed speed at engine-braking rates
(~0.3-0.5 m/s^2), and the one place the gap is visibly too wide is the interstate.

What it does
------------
``t_follow = BASE_T_FOLLOW * speed_gap_factor(road_type, v_ego) + closing_margin(...)``, rate-limited.

* ONE base gap for every personality (1.45 s, the stock standard value). Owner's ruling after the 2026-11-01
  drive: "aggressive follow distance needs to be normal follow distance". The personality still sets launch /
  pull behaviour elsewhere; it no longer moves the steady-state gap.
* Elastic, not stepped: the gap is a continuous function of speed, never a 1 / 2 / 3 s preset. On highway /
  interstate it SHORTENS as speed rises, 1.0 -> 0.82 between 16 and 32 m/s (1.45 -> 1.19 s, i.e. 55 -> 40 m at
  65 mph). Unknown road type follows the same speed curve. Urban keeps the full base gap: shortening it on a car
  that cannot brake is the wrong direction in town.
* closing margin (throttle-only cars only): extra distance needed to cancel the closing speed by coasting instead
  of braking at ``COMFORT_BRAKE``: ``vrel^2/2 * (1/a_coast(v) - 1/COMFORT_BRAKE)``, as time at v_ego, capped at
  ``MAX_CLOSING_MARGIN`` = 0.25 s and only while closing faster than ``CLOSING_MARGIN_MIN_VREL`` = 0.5 m/s. It only acts while the lead is slower. A same-speed lead that brakes hard gets no extra
  margin from this; that case still relies on FCW + the driver.

The time gap is a runtime MPC parameter (p[4]), so the OCP and the generated solver are unchanged.
"""
import numpy as np

from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import COMFORT_BRAKE, get_T_FOLLOW

PARAM = "AdaptiveFollowDistance"

# 2026-11-01 (owner, verbatim): "I just want whatever the aggressive profile to be the new normal profile and the
# aggressive profile to be like 1s +/- 0.4s or something". So NORMAL now runs at stock AGGRESSIVE's 1.25 s, and
# AGGRESSIVE gets its own 1.00 s base - the elastic highway factor then takes it to 0.60 s (1.0 - 0.4), which is the
# bottom of the range he asked for. RELAXED (unused on this car) follows NORMAL rather than being wider than it.
BASE_T_NORMAL = get_T_FOLLOW(log.LongitudinalPersonality.aggressive)   # 1.25 s
BASE_T_AGGRESSIVE = 1.00
BASE_T_FOLLOW = BASE_T_NORMAL   # kept for callers that want "the" base gap

# road-type x speed factor on BASE_T_FOLLOW (vEgo m/s). Highway gap SHRINKS with speed: 1.45 s at <= 16 m/s,
# 1.19 s at >= 32 m/s. The +6 m standstill offset in the MPC makes 1.19 s read as ~1.4 s at 65 mph, which is the
# distance the owner asked for after the 2026-11-01 drive ("normal follow distance right now is too far on the
# interstate").
GAP_FACTOR_BP = [16., 32.]
GAP_FACTOR_V = [1.0, 0.82]          # NORMAL: 1.25 -> 1.02 s (45 -> 37 m at 65 mph)
GAP_FACTOR_V_AGGRESSIVE = [1.0, 0.60]  # AGGRESSIVE: 1.00 -> 0.60 s (36 -> 22 m at 65 mph)
HIGHWAY_ROAD_TYPES = ("highway", "interstate")

# Throttle-only flat-ground coast decel magnitude (m/s^2, vEgo m/s). Measured on routes 00000127/00000128 with the
# pedal command at 0 for >= 1 s, grade-corrected medians: 0.32 @ 10-18, 0.35 @ 18-24, 0.41 @ 24-30, 0.50 @ 30-36 m/s.
# Taken ~15 % below the medians, since a downgrade eats coast decel (no trustworthy pitch in plannerd: carControl
# orientationNED[1] reads +0.028 rad vs GPS grade on this car).
COAST_DECEL_BP = [10., 18., 24., 30., 36.]
COAST_DECEL_V = [0.27, 0.30, 0.35, 0.42, 0.42]
# 2026-11-01 (measured on that drive, routes 187-18b, 14,220 engaged-with-lead frames): the car held 1.66-1.72 s
# implied T at 25-36 m/s while the law commanded 1.19 s. The difference is this margin: it is active whenever the lead
# is slower - i.e. almost always on the interstate - and the 0.6 s cap is what the owner kept feeling as "too far".
# The margin's job is only to cancel a REAL closing speed by coasting; at v_rel < 0.5 m/s it is noise. Cap 0.25 s and
# gate at 0.5 m/s: the highway steady gap lands at the law's own 1.19-1.30 s instead of 1.7-1.8 s.
MAX_CLOSING_MARGIN = 0.25  # s
CLOSING_MARGIN_MIN_VREL = 0.5  # m/s

# rate limits on the output (s of gap per s): add margin fast, give it back slowly
T_FOLLOW_RATE_UP = 1.0
T_FOLLOW_RATE_DOWN = 0.2


def is_throttle_only(CP, CP_SP) -> bool:
  """Hyundai comma-pedal longitudinal has no brake actuator (lift = coast). Other interceptor brands keep brakes."""
  return bool(CP_SP.enableGasInterceptor) and CP.brand == "hyundai"


def speed_gap_factor(road_type: str, v_ego: float, table=None) -> float:
  """Multiplier on the personality's base gap: 1.0 in town, shrinking to table[-1] on the open road as speed rises."""
  if road_type == "urban":
    return 1.0  # never shorten in town: a throttle-only car needs the room where stops are sudden
  return float(np.interp(v_ego, GAP_FACTOR_BP, GAP_FACTOR_V if table is None else table))


def coast_decel(v_ego: float) -> float:
  return float(np.interp(v_ego, COAST_DECEL_BP, COAST_DECEL_V))


def closing_margin(v_ego: float, v_lead: float) -> float:
  """Extra time gap (s) for a throttle-only car to cancel the closing speed by coasting."""
  v_rel = v_ego - v_lead
  if v_rel < CLOSING_MARGIN_MIN_VREL or v_ego <= 1.:
    return 0.
  extra_dist = v_rel ** 2 / 2. * (1. / coast_decel(v_ego) - 1. / COMFORT_BRAKE)
  return float(np.clip(extra_dist / v_ego, 0., MAX_CLOSING_MARGIN))


def is_aggressive(personality) -> bool:
  """True for the AGGRESSIVE personality. Works for both the plain int and the capnp enum the planner passes."""
  return personality == log.LongitudinalPersonality.aggressive


def base_t_follow(personality) -> float:
  """Base gap for a personality: AGGRESSIVE 1.00 s, everything else NORMAL 1.25 s (stock aggressive's value)."""
  return BASE_T_AGGRESSIVE if is_aggressive(personality) else BASE_T_NORMAL


def get_adaptive_t_follow(personality, v_ego: float, road_type: str, lead_present: bool, v_lead: float,
                          throttle_only: bool) -> float:
  """Target time gap before rate limiting. Two bases, one elastic speed law per base (see GAP_FACTOR_V*)."""
  aggressive = is_aggressive(personality)
  t_follow = base_t_follow(personality) * speed_gap_factor(road_type, v_ego,
                                                           GAP_FACTOR_V_AGGRESSIVE if aggressive else GAP_FACTOR_V)
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
