#!/usr/bin/env python3
import math
import numpy as np

import openpilot.cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.accel_boost import AccelBoost
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, LongitudinalPlanSource
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import STOP_DISTANCE, get_T_FOLLOW
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan, should_stop
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.common.swaglog import cloudlog

from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP

J_CRUISE_VALS = [1.6, 1.2, 0.8, 0.6]
J_CRUISE_BP = [0., 10.0, 25., 40.]
A_CRUISE_MIN = -0.5
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]


# fork (G4): lookup table for the turn (lateral) accel budget. Upstream dropped it together with the chill/ACC cruise
# path (#39097); the fork still needs it to turn-limit the e2e launch candidate below.
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]

# fork (cycle 1): far-lead e2e gate thresholds (see LongitudinalPlanner.update_e2e_gate). Enter is strict, stay is
# looser (hysteresis); gap factors multiply the MPC's desired distance STOP_DISTANCE + t_follow * v_ego.
E2E_GATE_MIN_SPEED = 3.0      # m/s, gate only above
E2E_GATE_ENTER_GAP = 1.2
E2E_GATE_ENTER_TTC = 12.0     # s
E2E_GATE_ENTER_VREL = -3.0    # m/s
E2E_GATE_ENTER_MPC = -0.1     # m/s^2
E2E_GATE_STAY_GAP = 1.1
E2E_GATE_STAY_TTC = 8.0
E2E_GATE_STAY_VREL = -3.5
E2E_GATE_STAY_MPC = -0.2
E2E_GATE_ENTER_FRAMES = 10    # 0.5 s at 20 Hz
# lead-decel veto (cycle 1 audit, 168 17:14:55): the gate may not enter, and drops at once, while the lead is braking.
# leadOne.aLeadK is the lead's ABSOLUTE accel (m/s^2, negative = lead decelerating): radard's Kalman accel state for a
# radar track, or modelV2.leadsV3[0].a[0] for a vision-only lead (get_RadarState_from_vision; on the Elantra N every
# lead is vision-only). It is the same value the MPC extrapolates the lead with (long_mpc.process_lead).
E2E_GATE_ENTER_ALEAD = -0.5   # m/s^2, enter only while aLeadK > this
E2E_GATE_STAY_ALEAD = -0.8    # m/s^2, exit the first frame aLeadK <= this


def turn_accel_limit(v_ego, angle_steers, CP):
  """Longitudinal accel budget left over after the lateral (turn) demand, from the a_total_max(v) ellipse
  (formerly also used by the ACC cruise candidate). Returns sqrt(a_total_max^2 - a_y^2) >= 0.
  angle_steers is the effective (offset-removed) road-wheel angle in degrees; abs() so a
  left/right turn both limit."""
  a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = abs(v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase))
  return math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))


def get_cruise_accel(v_cruise, v_ego, a_cruise_prev, dt):
  target_accel = np.clip(v_cruise - v_ego, A_CRUISE_MIN, ACCEL_MAX)
  j_cruise = np.interp(v_ego, J_CRUISE_BP, J_CRUISE_VALS)
  target_accel = float(np.clip(target_accel, a_cruise_prev - j_cruise * dt, a_cruise_prev + j_cruise * dt))

  return target_accel


class LongitudinalPlanner(LongitudinalPlannerSP):
  def __init__(self, CP, CP_SP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    LongitudinalPlannerSP.__init__(self, self.CP, CP_SP, self.mpc)
    self.fcw = False
    self.dt = dt
    self.accel_boost = AccelBoost()

    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.a_cruise = init_a
    self.output_a_target = init_a
    self.output_should_stop = False

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)

    self.e2e_gate = False
    self.e2e_gate_cnt = 0

  def update_e2e_gate(self, lead, v_ego, e2e_raw, output_a_target_mpc, should_stop_e2e, t_follow):
    """fork (cycle 1): far-lead e2e gate. While a lead is present, far (> 1.2x the MPC's desired gap), slow-closing
    (TTC > 12 s) and the MPC candidate is not braking, the vision model's accel wish may not undercut the MPC: e2e is
    raised to the MPC value. RAISE ONLY. Enters after 10 consecutive qualifying frames (0.5 s at 20 Hz), exits on the
    first frame the looser stay test fails (gap < 1.1x, TTC < 8 s, vRel < -3.5, MPC < -0.2: a cut-in drops it at once).
    Lead-decel veto: enter needs aLeadK > -0.5, stay needs aLeadK > -0.8, so the gate is never active while the lead is
    braking (a far but braking lead is exactly when the model's early ask must be kept).

    d_des matches the MPC's steady follow distance: get_safe_obstacle_distance(v_ego, t_follow) minus the lead's
    stopped-equivalence term is STOP_DISTANCE + t_follow * v_ego at equal speeds, with the same t_follow the MPC got."""
    d_des = STOP_DISTANCE + t_follow * v_ego
    ttc = lead.dRel / max(-lead.vRel, 0.1)
    base = lead.present and v_ego > E2E_GATE_MIN_SPEED and not should_stop_e2e
    enter = (base and lead.dRel > E2E_GATE_ENTER_GAP * d_des and ttc > E2E_GATE_ENTER_TTC and
             lead.vRel > E2E_GATE_ENTER_VREL and output_a_target_mpc >= E2E_GATE_ENTER_MPC and
             lead.aLeadK > E2E_GATE_ENTER_ALEAD)
    stay = (base and lead.dRel > E2E_GATE_STAY_GAP * d_des and ttc > E2E_GATE_STAY_TTC and
            lead.vRel > E2E_GATE_STAY_VREL and output_a_target_mpc >= E2E_GATE_STAY_MPC and
            lead.aLeadK > E2E_GATE_STAY_ALEAD)
    if self.e2e_gate:
      self.e2e_gate = bool(stay)  # exit is immediate
      self.e2e_gate_cnt = 0
    else:
      self.e2e_gate_cnt = self.e2e_gate_cnt + 1 if enter else 0
      self.e2e_gate = self.e2e_gate_cnt >= E2E_GATE_ENTER_FRAMES
    if self.e2e_gate:
      e2e_raw = max(e2e_raw, output_a_target_mpc)  # RAISE ONLY, never lower
    return e2e_raw

  def update(self, sm):
    LongitudinalPlannerSP.update(self, sm)

    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    if sm['controlsState'].forceDecel:
      v_cruise = 0.0

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    # PCM cruise speed may be updated a few cycles later, check if initialized
    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET
    reset_state = reset_state or not v_cruise_initialized

    steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['vehicleParameters'].angleOffsetDeg  # fork: G4

    if reset_state:
      self.v_desired_filter.x = v_ego
      self.output_a_target = np.clip(sm['carState'].aEgo, ACCEL_MIN, ACCEL_MAX)
      self.a_cruise = self.output_a_target

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    # Get new v_cruise and a_target from Smart Cruise Control and Speed Limit Assist
    v_cruise, self.output_a_target = LongitudinalPlannerSP.update_targets(self, sm, self.v_desired_filter.x, self.output_a_target, v_cruise)

    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)
    if self.CP.openpilotLongitudinalControl:
      self.mpc.set_cur_state(self.v_desired_filter.x, self.output_a_target)
    else:
      self.mpc.set_cur_state(v_ego, sm['carState'].aEgo)
    t_follow = LongitudinalPlannerSP.get_t_follow(self, sm, v_ego)  # fork: adaptive follow distance (None = stock)
    self.mpc.update(sm['radarState'], personality=sm['selfdriveState'].personality, t_follow=t_follow)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Save starting point for next iteration
    a_prev = self.output_a_target

    action_t =  self.CP.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                              action_t=action_t)
    output_should_stop_mpc = should_stop(v_ego, output_a_target_mpc)
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    # fork (cycle 1): far-lead e2e gate. Applied to the RAW e2e value, before the turn limit (raising e2e after the
    # turn limit would delete curve braking). Raise-only: never lowers e2e, never touches the MPC/cruise candidates.
    output_a_target_e2e = self.update_e2e_gate(sm['radarState'].leadOne, v_ego, output_a_target_e2e,
                                               output_a_target_mpc, sm['modelV2'].action.shouldStop,
                                               t_follow if t_follow is not None else
                                               get_T_FOLLOW(sm['selfdriveState'].personality))
    # G4: the e2e candidate owns the launch and was never turn-limited (get_cruise_accel's
    # limit_accel_in_turns only clips the cruise candidate). Limit its positive accel to the
    # leftover longitudinal budget of the same a_total_max(v) ellipse, floored at 0.3 m/s^2 so a
    # stopped car with the wheel turned can still pull away. min() never raises accel, so the
    # MPC/lead candidate's braking is untouched.
    output_a_target_e2e = min(output_a_target_e2e,
                              max(turn_accel_limit(v_ego, steer_angle_without_offset, self.CP), 0.3))
    output_should_stop_e2e = sm['modelV2'].action.shouldStop

    self.a_cruise = get_cruise_accel(v_cruise, v_ego, self.a_cruise, self.dt)
    cruise_should_stop = should_stop(v_ego, self.a_cruise)

    output_a_target_e2e = self.accel_boost.update(sm, output_a_target_e2e, output_a_target_mpc, self.a_cruise)

    candidates = [(output_a_target_mpc, self.mpc.source, output_should_stop_mpc),
                  (self.a_cruise, LongitudinalPlanSource.cruise, cruise_should_stop),
                  (output_a_target_e2e, LongitudinalPlanSource.e2e, output_should_stop_e2e)]

    output_a_target, self.mpc.source, _ = min(candidates, key=lambda c: c[0])
    self.output_should_stop = any(should_stop for _, _, should_stop in candidates)
    self.output_a_target = np.clip(output_a_target, ACCEL_MIN, ACCEL_MAX)

    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.output_a_target + a_prev) / 2.0

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks()

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.present
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.accelBoost = float(self.accel_boost.total_boost)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True

    pm.send('longitudinalPlan', plan_send)

    self.publish_longitudinal_plan_sp(sm, pm)
