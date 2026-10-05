"""
Fork: Smart Cruise Control (SCC-V vision / SCC-M map) fixes + throttle-only guard (adurham/sunnypilot).

Why this exists (car-features/drive-12e-12f-report.md §B; routes 0000012e / 0000012f)
-----------------------------------------------------------------------------------------
On the owner's throttle-only car (comma pedal, no brake actuator) a negative SCC target is a coast: the pedal goes to 0
and the car sheds ~0.3-0.5 m/s^2. Three SCC defects turned into "openpilot won't accelerate":

1. Vision, launch into an intersection turn (12e @215.75, 12f @1575.0): the moment v crossed MIN_V (5.56 m/s) mid-turn,
   SCC-V went ENTERING (predicted lat accel 2.1-2.5 for the rest of the 90 deg turn) and output
   ``max(v_turn, MIN_V) + a_target * 4 s`` = 2.5-3.2 m/s, i.e. BELOW its own MIN_V floor (the negative term is added
   after the floor). The plan went to -1.2 and the pedal to 0 until the driver overrode.
2. Map, on-ramp merge (12f @698.6-705.0): SCC-M sat in TURNING with a 14.93 m/s target for 6.4 s at 20.7 m/s while mapd
   was unmatched (roadType unknown, no name, no speed limit). ``update_calculations`` finds the nearest
   MapTargetVelocities point with ``min_dist = 1000`` and never checks it, so a stale/off-path list keeps producing
   targets.
3. Nothing bounds how long either controller can hold a coast on a car that can only coast.

What this module does (all inert for a non-throttle-only car except the two bug fixes, which are generic)
--------------------------------------------------------------------------------------------------------
* ``ForkSCCVision`` (subclass of upstream ``SmartCruiseControlVision``):
  - output floor: the active v target is ``max(upstream, MIN_V)`` (bug fix 1);
  - ENTERING is not entered below ``VISION_MIN_ENTER_V`` = 9 m/s (20 mph). Intersection turns from a stop are taken at
    4-7 m/s; every non-launch ENTERING on 12e/12f was at >= 10 m/s (12e 337, 12f 141, 689, 696);
  - if the car is already turning when ENTERING first fires (current lat accel >= ``VISION_ALREADY_TURNING_LAT_ACC``
    = 1.0 m/s^2), go straight to TURNING (gentle +0.5..0 m/s^2) instead of ENTERING (decel).
  Feature stays ON.
* ``ForkSCCMap`` (subclass of upstream ``SmartCruiseControlMap``):
  - ignore the map path when the nearest MapTargetVelocities point is > ``MAP_MAX_NEAREST_DIST`` (25 m) from the car,
    or mapd has no current way match (``roadType == unknown and not speedLimitValid and roadName == ''``). Rejection
    takes effect at once while SCC-M is not active (it can never start binding on a path the car is not on) and after
    ``MAP_REJECT_HOLD_S`` of continuous rejection while it is active (a one-sample GPS/match blip mid-curve does not
    snap the target back to cruise);
  - diagnostics: a ``scc_map_diag`` cloudlog event (logMessage -> rlog; no capnp schema change) at 1 Hz while
    longitudinal is engaged and on every state/gate change (rate-capped): points, nearest distance/index, current
    target and its distance, position, mapd road info, gate reason.
* ``ForkSmartCruiseControl`` (subclass of upstream ``SmartCruiseControl``) builds the two and adds the throttle-only
  guard: when an SCC output is asking to slow (below vEgo - 0.5 m/s) and the previous plan accel is a coast
  (< -0.05 m/s^2) WITHOUT lateral corroboration (current lat accel < 1.0 and model-predicted lat accel < 1.3, i.e.
  no sign of a curve now or within the model horizon), time accumulates. After ``GUARD_UNCORROBORATED_S`` = 2 s the
  guard latches and the SCC outputs are released (V_CRUISE_UNSET) until lateral evidence appears, the raw SCC targets
  no longer ask to slow (raw >= vEgo + 1 m/s, hysteresis on the RAW outputs so the release cannot sawtooth), or
  longitudinal disengages. Sized on 12e/12f (an12ef/scc_uncorr.py): the only uncorroborated SCC coast > 0.7 s on both
  routes is the 12f @699 phantom (5.4 s); every legit curve slowdown, incl. the 9.5 s 12f @1349 highway curve, had
  predicted lat >= 1.3 from its start. Why release instead of "hold speed": the controller's own premise (a curve) has
  no evidence for 2 s; holding 20 m/s on a merge into 30 m/s traffic is still the failure; a real curve becomes visible
  to the model and re-arms SCC (corroboration resets the guard). Why not a blind timer on all SCC decel: it would cap
  legit slowdowns (12f @1349 needed 9.5 s) on a car whose only decel is a coast.

Placement: upstream SCC files are untouched (subclass + override), so upstream SCC changes merge cleanly. The single
hook is in ``LongitudinalPlannerSP.__init__`` (``self.scc = ForkSmartCruiseControl(...)``). The overridden upstream
members are pinned by ``fork/tests/test_scc.py::TestUpstreamHooks`` so a rename upstream fails a test instead of
silently dropping the fix.
"""
import json
import math

import openpilot.cereal.messaging as messaging
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import MIN_V
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.map_controller import (
  MapState, SmartCruiseControlMap, TO_RADIANS, distance_to_point)
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.smart_cruise_control import SmartCruiseControl
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.vision_controller import (
  ACTIVE_STATES as VISION_ACTIVE_STATES, ENABLED_STATES as VISION_ENABLED_STATES, SmartCruiseControlVision, VisionState)

# --- vision ---
VISION_MIN_ENTER_V = 9.0               # m/s; no ENTERING below 20 mph (intersection turns from a stop: 4-7 m/s)
VISION_ALREADY_TURNING_LAT_ACC = 1.0   # m/s^2; already in the turn when ENTERING first fires -> TURNING

# --- map ---
MAP_MAX_NEAREST_DIST = 25.             # m; nearest MapTargetVelocities point farther than this -> not on that path
MAP_REJECT_HOLD_S = 1.0                # s of continuous rejection before an ACTIVE SCC-M is dropped
MAP_DIAG_PERIOD_S = 1.0                # periodic diagnostics while engaged
MAP_DIAG_MIN_GAP_S = 0.2               # cap on change-triggered diagnostics
MAP_PATH_MIN_GAP_S = 2.0               # route data: at most one scc_map_path event per this, only when the path changed
MAP_PATH_MAX_POINTS = 400              # route data: cap per event (~12 KB worst case)

# --- throttle-only guard ---
GUARD_UNCORROBORATED_S = 2.0           # s of uncorroborated SCC coast before the SCC targets are released
GUARD_SLOW_MARGIN = 0.5                # m/s; SCC output below vEgo - this = "asking to slow"
GUARD_RESET_MARGIN = 1.0               # m/s; raw SCC output above vEgo + this = no longer asking to slow (hysteresis)
GUARD_COAST_ACCEL = -0.05              # m/s^2; previous plan accel below this = coasting
GUARD_LAT_ACC_TH = 1.0                 # m/s^2; current lat accel at/above = in a curve (corroborated)
GUARD_PRED_LAT_ACC_TH = 1.3            # m/s^2; predicted lat accel at/above = curve ahead (= SCC-V entry threshold)


class ForkSCCVision(SmartCruiseControlVision):
  def get_v_target_from_control(self) -> float:
    v_target = super().get_v_target_from_control()
    if self.is_active:
      # upstream adds a_target * horizon AFTER its own MIN_V floor, so the output can go below it (2.5-3.2 m/s on 12e/12f)
      return max(v_target, MIN_V)
    return v_target

  def _update_state_machine(self) -> tuple[bool, bool]:
    prev_state = self.state
    super()._update_state_machine()
    if prev_state == VisionState.enabled and self.state == VisionState.entering:
      if self.v_ego < VISION_MIN_ENTER_V:
        self.state = VisionState.enabled
      elif self.current_lat_acc >= VISION_ALREADY_TURNING_LAT_ACC:
        self.state = VisionState.turning
    return self.state in VISION_ENABLED_STATES, self.state in VISION_ACTIVE_STATES


def map_unmatched(live_map) -> bool:
  """mapd has no current way match: no road type, no speed limit, no name (12f 682.5-705.5)."""
  if live_map is None:
    return True
  return str(live_map.roadType) == "unknown" and not live_map.speedLimitValid and str(live_map.roadName) == ""


class ForkSCCMap(SmartCruiseControlMap):
  def __init__(self):
    super().__init__()
    self.live_map = None              # liveMapDataSP, handed in by ForkSmartCruiseControl each cycle
    self.nearest_dist = -1.           # m, -1 = no path
    self.nearest_idx = -1
    self.target_dist = -1.            # m from the car to the current target point, -1 = none
    self.reject_reason = ""           # "" = path accepted
    self.rejected = False             # rejection in effect (after the hold, if active)
    self._reject_frames = 0
    self._last_diag_frame = -10 ** 9
    self._last_diag_key = None
    self._last_path_frame = -10 ** 9
    self._last_path_key = hash(json.dumps([]))  # an empty path is not logged until a real one appears

  def _nearest(self) -> tuple[float, int]:
    if not self.target_velocities or self.last_position is None:
      return -1., -1
    lat = self.last_position.latitude * TO_RADIANS
    lon = self.last_position.longitude * TO_RADIANS
    best, best_i = math.inf, -1
    for i, tv in enumerate(self.target_velocities):
      try:
        d = distance_to_point(lat, lon, tv["latitude"] * TO_RADIANS, tv["longitude"] * TO_RADIANS)
      except (KeyError, TypeError):
        continue
      if d < best:
        best, best_i = d, i
    return (best, best_i) if best_i >= 0 else (-1., -1)

  def update_calculations(self) -> None:
    super().update_calculations()
    self.nearest_dist, self.nearest_idx = self._nearest()

    if self.nearest_idx < 0:
      reason = ""  # no path at all: upstream already produces no target
    elif self.nearest_dist > MAP_MAX_NEAREST_DIST:
      reason = "far"
    elif map_unmatched(self.live_map):
      reason = "unmatched"
    else:
      reason = ""
    self.reject_reason = reason

    self._reject_frames = self._reject_frames + 1 if reason else 0
    hold_frames = int(round(MAP_REJECT_HOLD_S / DT_MDL)) if self.state == MapState.turning else 1
    self.rejected = self._reject_frames >= hold_frames
    if self.rejected:
      # upstream's own "no target" reset: the state machine (run right after this, same cycle) drops TURNING -> ENABLED
      # and ENABLED never enters TURNING with v_target == 0
      self.v_target = 0.
      self.target_lat = 0.
      self.target_lon = 0.

    if self.target_lat or self.target_lon:
      self.target_dist = distance_to_point(self.last_position.latitude * TO_RADIANS, self.last_position.longitude * TO_RADIANS,
                                           self.target_lat * TO_RADIANS, self.target_lon * TO_RADIANS)
    else:
      self.target_dist = -1.

  def get_v_target_from_control(self) -> float:
    if self.rejected:
      return V_CRUISE_UNSET  # belt and braces: never bind on a rejected path, whatever the state machine did
    return super().get_v_target_from_control()

  def diag(self) -> dict:
    lm = self.live_map
    return {
      "state": str(self.state), "reject": self.reject_reason, "rejected": self.rejected,
      "points": len(self.target_velocities or []), "nearest_m": round(self.nearest_dist, 1), "nearest_idx": self.nearest_idx,
      "v_target": round(float(self.v_target), 2), "out_v": round(float(self.output_v_target), 2),
      "target_lat": self.target_lat, "target_lon": self.target_lon, "target_m": round(self.target_dist, 1),
      "lat": self.last_position.latitude if self.last_position else None,
      "lon": self.last_position.longitude if self.last_position else None,
      "v_ego": round(float(self.v_ego), 2), "v_cruise": round(float(self.v_cruise), 2),
      "road_type": str(lm.roadType) if lm is not None else None, "road_name": str(lm.roadName) if lm is not None else None,
      "speed_limit_valid": bool(lm.speedLimitValid) if lm is not None else None,
    }

  def _log_diag(self) -> None:
    if not self.long_enabled:
      self._last_diag_key = None
      return
    key = (str(self.state), self.reject_reason, self.rejected)
    periodic = self.frame - self._last_diag_frame >= int(round(MAP_DIAG_PERIOD_S / DT_MDL))
    changed = key != self._last_diag_key and self.frame - self._last_diag_frame >= int(round(MAP_DIAG_MIN_GAP_S / DT_MDL))
    if periodic or changed:
      cloudlog.event("scc_map_diag", **self.diag())
      self._last_diag_frame = self.frame
      self._last_diag_key = key

  def _log_path(self) -> None:
    """Route data: the MapTargetVelocities list lives only in /dev/shm params (not in the rlog), so offline replays had
    to fake SCC-M's input. Log the list itself, whenever its content changes, engaged or not (rate-capped,
    coordinates rounded to ~0.1 m). Bounded storage: <= 1 event / MAP_PATH_MIN_GAP_S, <= MAP_PATH_MAX_POINTS points."""
    tv = self.target_velocities or []
    try:
      pts = [[round(float(p["latitude"]), 6), round(float(p["longitude"]), 6), round(float(p["velocity"]), 2)]
             for p in tv[:MAP_PATH_MAX_POINTS]]
    except (KeyError, TypeError, ValueError):
      pts = []
    key = hash(json.dumps(pts))
    if key == self._last_path_key or self.frame - self._last_path_frame < int(round(MAP_PATH_MIN_GAP_S / DT_MDL)):
      return
    cloudlog.event("scc_map_path", n=len(tv), truncated=len(tv) > MAP_PATH_MAX_POINTS, points=pts)
    self._last_path_key = key
    self._last_path_frame = self.frame

  def update(self, long_enabled: bool, long_override: bool, v_ego, a_ego, v_cruise) -> None:
    super().update(long_enabled, long_override, v_ego, a_ego, v_cruise)
    self._log_diag()
    self._log_path()


class ThrottleOnlySCCGuard:
  """Releases SCC targets that hold a coast without lateral evidence of a curve (see module docstring)."""

  def __init__(self, dt: float = DT_MDL):
    self.dt = dt
    self.uncorroborated_t = 0.
    self.latched = False

  def reset(self) -> None:
    self.uncorroborated_t = 0.
    self.latched = False

  def update(self, long_enabled: bool, v_ego: float, a_prev: float, raw_v_targets: tuple[float, ...],
             current_lat_acc: float, max_pred_lat_acc: float) -> bool:
    """-> True while the SCC targets must be released."""
    raw_min = min(raw_v_targets)
    corroborated = current_lat_acc >= GUARD_LAT_ACC_TH or max_pred_lat_acc >= GUARD_PRED_LAT_ACC_TH
    if not long_enabled or corroborated or raw_min >= v_ego + GUARD_RESET_MARGIN:
      self.reset()
      return False
    if raw_min < v_ego - GUARD_SLOW_MARGIN and a_prev < GUARD_COAST_ACCEL:
      self.uncorroborated_t += self.dt
    if self.uncorroborated_t >= GUARD_UNCORROBORATED_S:
      self.latched = True
    return self.latched


class ForkSmartCruiseControl(SmartCruiseControl):
  def __init__(self, throttle_only: bool = False):
    # upstream builds plain controllers; replace them with the fork subclasses (same constructor contract)
    super().__init__()
    self.vision = ForkSCCVision()
    self.map = ForkSCCMap()
    self.throttle_only = throttle_only
    self.guard = ThrottleOnlySCCGuard()
    self.guard_active = False

  def update(self, sm: messaging.SubMaster, long_enabled: bool, long_override: bool, v_ego: float, a_ego: float,
             v_cruise: float) -> None:
    try:
      self.map.live_map = sm['liveMapDataSP']
    except (KeyError, IndexError):
      self.map.live_map = None
    super().update(sm, long_enabled, long_override, v_ego, a_ego, v_cruise)

    was_active = self.guard_active
    self.guard_active = False
    if self.throttle_only:
      # a_ego here is the planner's previous output accel (LongitudinalPlanner passes self.output_a_target)
      raw = (self.vision.output_v_target, self.map.output_v_target)
      self.guard_active = self.guard.update(long_enabled, v_ego, a_ego, raw,
                                            self.vision.current_lat_acc, self.vision.max_pred_lat_acc)
      if self.guard_active != was_active:
        cloudlog.event("scc_throttle_only_guard", released=self.guard_active, v_ego=round(float(v_ego), 2),
                       a_prev=round(float(a_ego), 2), vision_v=round(float(raw[0]), 2), map_v=round(float(raw[1]), 2),
                       vision_state=str(self.vision.state), map_state=str(self.map.state),
                       cur_lat=round(float(self.vision.current_lat_acc), 2), pred_lat=round(float(self.vision.max_pred_lat_acc), 2))
      if self.guard_active:
        self.vision.output_v_target = V_CRUISE_UNSET
        self.map.output_v_target = V_CRUISE_UNSET
