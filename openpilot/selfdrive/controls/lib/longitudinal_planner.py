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
# ENTER tests the 0.3 s median of aLeadK (a 1-2 frame vision spike must not block entry) AND the raw aLeadK against the
# stay threshold, so a lagging median can never admit a frame the raw stay test would drop: ENTER stays a subset of STAY.
# STAY is unfiltered (raw aLeadK, first-frame release).
E2E_GATE_ENTER_ALEAD_MEDIAN_FRAMES = 6  # 0.3 s at 20 Hz
# closing-rate veto (cycle 2, route 169 16b cut-in at t=1647.0): the gate must never hold speed up toward a lead that is
# still closing fast. The gap and TTC floors above are TTC-scaled, so the further the lead the LARGER the closing rate
# they tolerate (at 1.96x d_des, TTC 12 s allows -7 m/s of closure), and vRel > -3.0 alone let a lead cut in at 116 m
# and still closing -2.0 m/s be gated: the model's wish -0.06 was raised to +0.67 and the car kept accelerating toward
# it until the driver braked. Veto = never enter and drop at once while, inside E2E_GATE_VREL_CLOSE_GAP x d_des (the
# region where the gate actually acts: every intended-case entry on the owner's drives was 1.27-1.86x), the lead is
# closing faster than E2E_GATE_STAY_VREL_CLOSE; ENTER additionally tests the 0.3 s median against the tighter
# E2E_GATE_ENTER_VREL_CLOSE (vRel is a noisy derivative of a noisy vision range: on 16a, raw -1.5 alone cut a 131-frame
# legitimate follow, the median cut 17) and the raw floor, so ENTER stays a subset of STAY.
E2E_GATE_ENTER_VREL_CLOSE = -1.5  # m/s, enter needs median_0.3s(vRel) > this
E2E_GATE_STAY_VREL_CLOSE = -2.0   # m/s, stay needs raw vRel > this (first-frame exit)
E2E_GATE_VREL_CLOSE_GAP = 3.0     # x d_des, the closing veto applies only inside this ratio (0.5x margin on the
#                                  incident frame at 1.96x under the replay's t_follow, 2.49x under stock 1.45 s)
# rebound latch: the gate turned ON at 1647.015 only because vRel crossed back above its -3.0 enter floor after peaking
# at -7.2. vRel crossing back is not "the approach ended" - it is noise on a lead that cut in 2 s earlier. Once a raw
# vRel <= E2E_GATE_ENTER_VREL (the gate's own idea of a fast closure, never present in the 168 trace it was built for:
# min -2.65) has been seen, no entry for E2E_GATE_REBOUND_FRAMES, and the latch only counts down on frames whose raw
# vRel is clear of the veto band (a frame inside the band freezes it: ambiguous -> fail toward OFF, stock behaviour).
E2E_GATE_REBOUND_FRAMES = 80      # 4.0 s at 20 Hz
E2E_GATE_ENTER_VREL_MEDIAN_FRAMES = 6  # 0.3 s at 20 Hz, same window as the aLeadK median
# comfort cap (cycle 3, routes 16d/16f 2026-10-26): the gate's job is to stop the model braking for a lead it can see
# far ahead -- NOT to hand the car the MPC's gap-chasing ask. Measured on the deployed gate (replayed frame-by-frame
# from the rlogs, gate decision from this very function): the gate is on for 25-32 % of long-active time and, uncapped,
# the plan asks > 0.3 m/s^2 for 3.7x as long as it would without the gate (16d 64.7 s vs 17.5 s) and > 0.8 m/s^2 for
# 30x as long (3.1 s vs 0.1 s); 11 of 18 gate episodes were followed by a brake within 8 s. The owner's words: "we
# haven't hit the max yet, let me race to it, oh but now we're too close, I need to brake". So the raise target is
# min(mpc, E2E_GATE_MAX_RAISE): the model still may not BRAKE for a far, slow-closing lead, but the gate can never
# command more than a comfortable acceleration on its own. 0.4 m/s^2 removes every gate-caused frame above 0.8 (the
# hard-accel tail falls back to the pre-gate baseline) while keeping the anti-lag behaviour the gate was built for
# (the lagging case measured +0.04 model vs +0.81 MPC -> the car now accelerates at 0.4 instead of sticking at 0.04).
E2E_GATE_MAX_RAISE = 0.4      # m/s^2, ceiling on the accel the gate itself may add


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
    self.e2e_gate_alead = []  # last E2E_GATE_ENTER_ALEAD_MEDIAN_FRAMES aLeadK values of the present lead
    self.e2e_gate_vrel = []   # last E2E_GATE_ENTER_VREL_MEDIAN_FRAMES vRel values of the present lead
    self.e2e_gate_latch = 0   # frames of rebound latch left (set by a fast closure, see E2E_GATE_REBOUND_FRAMES)

  def update_e2e_gate(self, lead, v_ego, e2e_raw, output_a_target_mpc, should_stop_e2e, t_follow):
    """fork (cycle 1): far-lead e2e gate. While a lead is present, far (> 1.2x the MPC's desired gap), slow-closing
    (TTC > 12 s) and the MPC candidate is not braking, the vision model's accel wish may not undercut the MPC: e2e is
    raised to the MPC value. RAISE ONLY. Enters after 10 consecutive qualifying frames (0.5 s at 20 Hz), exits on the
    first frame the looser stay test fails (gap < 1.1x, TTC < 8 s, vRel < -3.5, MPC < -0.2: a cut-in drops it at once).
    Lead-decel veto: enter needs median_0.3s(aLeadK) > -0.5 AND raw aLeadK > -0.8, stay needs raw aLeadK > -0.8, so the
    gate is never active while the lead is braking (a far but braking lead is exactly when the model's early ask must be
    kept). The median buffer holds the present lead's last 6 values and is cleared while no lead is present.

    Closing-rate veto (cycle 2, the 16b cut-in): the gap/TTC floors are TTC-scaled, so a far lead may close fast and
    still pass them; the gate may then hold speed toward a car that just cut in. Inside E2E_GATE_VREL_CLOSE_GAP x d_des
    the gate never enters and drops at once unless the lead's closing rate is benign: enter needs median_0.3s(vRel) >
    E2E_GATE_ENTER_VREL_CLOSE AND raw vRel > E2E_GATE_STAY_VREL_CLOSE, stay needs raw vRel > E2E_GATE_STAY_VREL_CLOSE
    (unfiltered, same-frame release). A raw vRel <= E2E_GATE_ENTER_VREL (fast closure) additionally arms
    E2E_GATE_REBOUND_FRAMES of latch during which the gate never enters, so a vRel that merely oscillates back above
    -3.0 on a lead that is still being caught up with cannot open the gate. The latch only counts down on frames clear
    of the veto band; a non-finite vRel or a frame inside the band holds it (ambiguous -> fail toward OFF). Both
    buffers and the latch are cleared while no lead is present. Every term is an AND on the existing tests, so the
    veto can only keep the gate OFF: it never raises the ask itself.

    d_des matches the MPC's steady follow distance: get_safe_obstacle_distance(v_ego, t_follow) minus the lead's
    stopped-equivalence term is STOP_DISTANCE + t_follow * v_ego at equal speeds, with the same t_follow the MPC got."""
    d_des = STOP_DISTANCE + t_follow * v_ego
    ttc = lead.dRel / max(-lead.vRel, 0.1)
    base = lead.present and v_ego > E2E_GATE_MIN_SPEED and not should_stop_e2e
    if lead.present:
      self.e2e_gate_alead.append(lead.aLeadK)
      del self.e2e_gate_alead[:-E2E_GATE_ENTER_ALEAD_MEDIAN_FRAMES]
      if math.isfinite(lead.vRel):
        self.e2e_gate_vrel.append(lead.vRel)
        del self.e2e_gate_vrel[:-E2E_GATE_ENTER_VREL_MEDIAN_FRAMES]
        vrel_med_ = float(np.median(self.e2e_gate_vrel))
        if vrel_med_ <= E2E_GATE_ENTER_VREL:
          self.e2e_gate_latch = E2E_GATE_REBOUND_FRAMES  # sustained fast closure: no entry until it has clearly ended
        elif self.e2e_gate_latch and lead.vRel > E2E_GATE_STAY_VREL_CLOSE:
          self.e2e_gate_latch -= 1  # inside the band (or ambiguous): the latch holds instead of counting down
    else:
      self.e2e_gate_alead.clear()
      self.e2e_gate_vrel.clear()
      self.e2e_gate_latch = 0
    alead_med = float(np.median(self.e2e_gate_alead)) if self.e2e_gate_alead else lead.aLeadK
    vrel_med = float(np.median(self.e2e_gate_vrel)) if self.e2e_gate_vrel else lead.vRel
    # closing-rate veto: only inside the gap band where the gate acts, only ever an AND on the existing tests
    gap_close = lead.dRel < E2E_GATE_VREL_CLOSE_GAP * d_des
    vrel_ok_enter = (not gap_close or (vrel_med > E2E_GATE_ENTER_VREL_CLOSE and lead.vRel > E2E_GATE_STAY_VREL_CLOSE))
    vrel_ok_stay = not gap_close or lead.vRel > E2E_GATE_STAY_VREL_CLOSE
    enter = (base and lead.dRel > E2E_GATE_ENTER_GAP * d_des and ttc > E2E_GATE_ENTER_TTC and
             lead.vRel > E2E_GATE_ENTER_VREL and output_a_target_mpc >= E2E_GATE_ENTER_MPC and
             alead_med > E2E_GATE_ENTER_ALEAD and lead.aLeadK > E2E_GATE_STAY_ALEAD and
             vrel_ok_enter and not self.e2e_gate_latch)
    stay = (base and lead.dRel > E2E_GATE_STAY_GAP * d_des and ttc > E2E_GATE_STAY_TTC and
            lead.vRel > E2E_GATE_STAY_VREL and output_a_target_mpc >= E2E_GATE_STAY_MPC and
            lead.aLeadK > E2E_GATE_STAY_ALEAD and
            vrel_ok_stay and not self.e2e_gate_latch)
    if self.e2e_gate:
      self.e2e_gate = bool(stay)  # exit is immediate
      self.e2e_gate_cnt = 0
    else:
      self.e2e_gate_cnt = self.e2e_gate_cnt + 1 if enter else 0
      self.e2e_gate = self.e2e_gate_cnt >= E2E_GATE_ENTER_FRAMES
    if self.e2e_gate:
      # RAISE ONLY, never lower; capped at a comfortable acceleration so the gate cannot turn the model's wish into the
      # MPC's gap-chasing ask (see E2E_GATE_MAX_RAISE).
      e2e_raw = max(e2e_raw, min(output_a_target_mpc, E2E_GATE_MAX_RAISE))
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
