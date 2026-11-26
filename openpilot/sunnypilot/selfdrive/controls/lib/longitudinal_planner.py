"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.cereal import messaging, custom
from opendbc.car import structs
from openpilot.common.constants import CV
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX
from openpilot.selfdrive.controls.lib.accel_boost import AccelBoost
from openpilot.sunnypilot.selfdrive.controls.lib.accel_boost import AccelBoostSP
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_alerts_helper import E2EAlertsHelper
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist import SpeedLimitAssist
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_resolver import SpeedLimitResolver
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.models.helpers import get_active_bundle
from openpilot.sunnypilot.fork.adaptive_follow import AdaptiveFollow, is_throttle_only
from openpilot.sunnypilot.fork.planner_earlier import PlannerEarlierBraking
from openpilot.sunnypilot.fork.scc import ForkSmartCruiseControl
from openpilot.sunnypilot.fork.setspeed_ease import SetSpeedEase

LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource


class LongitudinalPlannerSP:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP, mpc):
    self.events_sp = EventsSP()
    self.resolver = SpeedLimitResolver()
    self.scc = ForkSmartCruiseControl(is_throttle_only(CP, CP_SP))  # fork: SCC fixes + throttle-only guard (fork/scc.py)
    self.resolver = SpeedLimitResolver()
    self.sla = SpeedLimitAssist(CP, CP_SP)
    self.generation = int(model_bundle.generation) if (model_bundle := get_active_bundle()) else None
    self.source = LongitudinalPlanSource.cruise
    self.e2e_alerts_helper = E2EAlertsHelper()
    self.accel_boost_sp = AccelBoostSP()
    self.openpilot_longitudinal = CP.openpilotLongitudinalControl
    self.adaptive_follow = AdaptiveFollow(is_throttle_only(CP, CP_SP))
    self.planner_earlier = PlannerEarlierBraking()  # fork: optional earlier lead/stop-approach braking
    self.setspeed_ease = SetSpeedEase()  # fork: gentler, personality-dependent chase of a raised target

    self.output_v_target = 0.
    self.output_a_target = 0.

  def get_t_follow(self, sm: messaging.SubMaster, v_ego: float) -> float | None:
    """Fork adaptive follow distance; None when the AdaptiveFollowDistance param is off (stock T_FOLLOW)."""
    road_type = str(sm['liveMapDataSP'].roadType)  # unknown (enum default) until mapd publishes
    lead = sm['radarState'].leadOne
    return self.adaptive_follow.update(sm['selfdriveState'].personality, v_ego, road_type, lead.present, lead.vLead)

  def get_effective_t_follow(self, sm: messaging.SubMaster, v_ego: float) -> float:
    """Fork: apply the optional PlannerEarlierBraking time-gap offset to the MPC/gate follow distance.

    When PlannerEarlierBraking is OFF (default) this returns the same t_follow the MPC would have used:
    the adaptive-follow value if AdaptiveFollowDistance is ON, otherwise the stock personality T_FOLLOW.
    When ON it adds planner_earlier.EXTRA_T_FOLLOW seconds to that base time gap, but only on the intended
    approach frames: a closing/slower lead (v_ego > v_lead + 0.5 m/s), a braking lead, or a model-predicted stop.
    The effective value is passed to LongitudinalMpc.update() as the lead_t_follow parameter and to the e2e
    far-lead gate as d_des. This moves the timing of the lead/stop-approach plan; it does NOT change the FCA11
    map, panda, engine braking, ACCEL_MIN/MAX, AccelBoost, NoClose cap, standstill-launch hold, stop-commit,
    min-gap floor, or SCC/SLA arbitration. A matched or pulling-away lead is unaffected.
    """
    from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import get_T_FOLLOW
    from openpilot.sunnypilot.fork.planner_earlier import EXTRA_T_FOLLOW
    t_follow = self.get_t_follow(sm, v_ego)
    if t_follow is None:
      t_follow = get_T_FOLLOW(sm['selfdriveState'].personality)

    apply = False
    lead = sm['radarState'].leadOne
    if lead.present:
      # closing/slower lead, or a stop directly behind a lead
      apply = v_ego > lead.vLead + 0.5 or bool(sm['modelV2'].action.shouldStop)
    else:
      # e2e stop with no lead
      apply = bool(sm['modelV2'].action.shouldStop)

    if self.planner_earlier.enabled and apply:
      t_follow += EXTRA_T_FOLLOW
    return t_follow

  def update_targets(self, sm: messaging.SubMaster, v_ego: float, a_ego: float, v_cruise: float) -> tuple[float, float]:
    CS = sm['carState']
    v_cruise_cluster_kph = min(CS.vCruiseCluster, V_CRUISE_MAX)
    v_cruise_cluster = v_cruise_cluster_kph * CV.KPH_TO_MS

    long_enabled = sm['carControl'].enabled
    long_override = sm['carControl'].cruiseControl.override

    # Smart Cruise Control (evaluate the merge gate first so SCC-V can be excluded during a merge window)
    merging = self.setspeed_ease.merge_state(sm, v_cruise, CS.vEgo)
    self.scc.update(sm, long_enabled, long_override, v_ego, a_ego, v_cruise, merging)

    # Speed Limit Resolver
    self.resolver.update(v_ego, sm)

    # Speed Limit Assist
    has_speed_limit = self.resolver.speed_limit_valid or self.resolver.speed_limit_last_valid
    self.sla.update(long_enabled, long_override, v_ego, a_ego, v_cruise_cluster, self.resolver.speed_limit,
                    self.resolver.speed_limit_final_last, has_speed_limit, self.resolver.distance, self.events_sp)

    targets = {
      LongitudinalPlanSource.cruise: (v_cruise, a_ego),
      LongitudinalPlanSource.sccVision: (self.scc.vision.output_v_target, self.scc.vision.output_a_target),
      LongitudinalPlanSource.sccMap: (self.scc.map.output_v_target, self.scc.map.output_a_target),
      LongitudinalPlanSource.speedLimitAssist: (self.sla.output_v_target, self.sla.output_a_target),
    }

    self.source = min(targets, key=lambda k: targets[k][0])
    self.output_v_target, self.output_a_target = targets[self.source]
    # fork: personality-dependent ramp toward a higher target (fork/setspeed_ease.py); vTarget stays raw.
    # `merging` was evaluated once this tick, above, and shared with the SCC-V exclusion.
    v_cruise_eased = self.setspeed_ease.update(sm, self.output_v_target, long_enabled, long_override, a_ego, v_cruise, merging)
    return v_cruise_eased, self.output_a_target

  def update(self, sm: messaging.SubMaster) -> None:
    if self.accel_boost_sp.update_params():
      self.accel_boost = self.accel_boost_sp if self.accel_boost_sp.enabled and self.openpilot_longitudinal else AccelBoost()

    self.events_sp.clear()
    self.e2e_alerts_helper.update(sm, self.events_sp)

  def publish_longitudinal_plan_sp(self, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    plan_sp_send = messaging.new_message('longitudinalPlanSP')

    plan_sp_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])

    longitudinalPlanSP = plan_sp_send.longitudinalPlanSP
    longitudinalPlanSP.longitudinalPlanSource = self.source
    longitudinalPlanSP.vTarget = float(self.output_v_target)
    longitudinalPlanSP.aTarget = float(self.output_a_target)
    longitudinalPlanSP.events = self.events_sp.to_msg()

    # Smart Cruise Control
    smartCruiseControl = longitudinalPlanSP.smartCruiseControl
    # Vision Control
    sccVision = smartCruiseControl.vision
    sccVision.state = self.scc.vision.state
    sccVision.vTarget = float(self.scc.vision.output_v_target)
    sccVision.aTarget = float(self.scc.vision.output_a_target)
    sccVision.currentLateralAccel = float(self.scc.vision.current_lat_acc)
    sccVision.maxPredictedLateralAccel = float(self.scc.vision.max_pred_lat_acc)
    sccVision.enabled = self.scc.vision.is_enabled
    sccVision.active = self.scc.vision.is_active
    # Map Control
    sccMap = smartCruiseControl.map
    sccMap.state = self.scc.map.state
    sccMap.vTarget = float(self.scc.map.output_v_target)
    sccMap.aTarget = float(self.scc.map.output_a_target)
    sccMap.enabled = self.scc.map.is_enabled
    sccMap.active = self.scc.map.is_active

    # Speed Limit
    speedLimit = longitudinalPlanSP.speedLimit
    resolver = speedLimit.resolver
    resolver.speedLimit = float(self.resolver.speed_limit)
    resolver.speedLimitLast = float(self.resolver.speed_limit_last)
    resolver.speedLimitFinal = float(self.resolver.speed_limit_final)
    resolver.speedLimitFinalLast = float(self.resolver.speed_limit_final_last)
    resolver.speedLimitValid = self.resolver.speed_limit_valid
    resolver.speedLimitLastValid = self.resolver.speed_limit_last_valid
    resolver.speedLimitOffset = float(self.resolver.speed_limit_offset)
    resolver.distToSpeedLimit = float(self.resolver.distance)
    resolver.source = self.resolver.source
    assist = speedLimit.assist
    assist.state = self.sla.state
    assist.enabled = self.sla.is_enabled
    assist.active = self.sla.is_active
    assist.vTarget = float(self.sla.output_v_target)
    assist.aTarget = float(self.sla.output_a_target)

    # E2E Alerts
    e2eAlerts = longitudinalPlanSP.e2eAlerts
    e2eAlerts.greenLightAlert = self.e2e_alerts_helper.green_light_alert
    e2eAlerts.leadDepartAlert = self.e2e_alerts_helper.lead_depart_alert

    # fork (2026-11-19): log-only fields appended by the planner subclass (LongitudinalPlanner -> `noClose`). A no-op
    # in this base class, so a planner that does not override it publishes exactly the message it published before.
    self.publish_fork_plan_log(longitudinalPlanSP)

    pm.send('longitudinalPlanSP', plan_sp_send)

  def publish_fork_plan_log(self, plan_sp) -> None:
    """fork hook: log-only planner telemetry, filled by LongitudinalPlanner. Nothing here is read by control code."""
