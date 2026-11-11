"""
Fork (17c): the no-lead stop-approach commitment through the REAL LongitudinalPlanner.update path (MPC, cruise, e2e gate,
turn limit, AccelBoost, min()). Flag OFF vs ON in lock-step on identical inputs.
"""
import numpy as np

from openpilot.cereal import log
import openpilot.cereal.messaging as messaging
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.modeld.constants import ModelConstants
import openpilot.selfdrive.controls.lib.longitudinal_planner as lp
from openpilot.sunnypilot.fork import stop_commit as sc

T = np.array(ModelConstants.T_IDXS)


class FakeParams:
  def __init__(self, enabled):
    self.enabled = enabled

  def get_bool(self, key):
    assert key == sc.PARAM
    return self.enabled


def make_planner(enabled):
  from opendbc.car.honda.interface import CarInterface
  from opendbc.car.honda.values import CAR
  CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
  CP.openpilotLongitudinalControl = True
  CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.HONDA_CIVIC)
  p = lp.LongitudinalPlanner(CP, CP_SP, init_v=17.)
  p.stop_commit = sc.StopCommit(params=FakeParams(enabled))
  return p


def planner_sm(v_ego, desired_accel, plan_v, plan_x, lead_present=False, d_rel=60., v_lead=None, gas=False):
  N = len(T)
  msgs = {k: messaging.new_message(k) for k in ('modelV2', 'carState', 'carControl', 'controlsState', 'selfdriveState',
                                                'radarState', 'liveMapDataSP', 'vehicleParameters', 'carStateSP',
                                                'gpsLocation', 'gpsLocationExternal', 'selfdriveStateSP')}
  md = msgs['modelV2'].modelV2
  md.position.x = [float(x) for x in plan_x]
  md.position.t = [float(t) for t in T]
  md.velocity.x = [float(v) for v in plan_v]
  md.orientationRate.z = [0.] * N
  md.action.desiredAcceleration = float(desired_accel)
  md.meta.disengagePredictions.gasPressProbs = [1.] * 6
  cs = msgs['carState'].carState
  cs.vEgo = float(v_ego)
  cs.vCruise = 100.
  cs.vCruiseCluster = 100.
  cs.gasPressed = gas
  cc = msgs['carControl'].carControl
  cc.enabled = True
  cc.orientationNED = [0., 0., 0.]
  msgs['controlsState'].controlsState.longControlState = LongCtrlState.pid
  sd = msgs['selfdriveState'].selfdriveState
  sd.enabled = True
  sd.personality = log.LongitudinalPersonality.standard
  l1 = msgs['radarState'].radarState.leadOne
  l1.present = lead_present
  if lead_present:
    vl = v_ego if v_lead is None else v_lead
    l1.dRel = float(d_rel)
    l1.vLead = float(vl)
    l1.vRel = float(vl - v_ego)
    l1.vLeadK = float(vl)
    l1.aLeadTau = 1.5
    l1.modelProb = 1.0
  return {k: getattr(m, k) for k, m in msgs.items()}


def stop_plan(v0, x_stop):
  a = v0 * v0 / (2. * x_stop)
  v = np.clip(v0 - a * T, 0., None)
  x = np.where(T < v0 / a, v0 * T - 0.5 * a * T ** 2, x_stop)
  return v, x, a


def lockstep(frames):
  on, off = make_planner(True), make_planner(False)
  a_on, a_off, src_on = [], [], []
  for kw in frames:
    sm = planner_sm(**kw)
    on.update(sm)
    off.update(sm)
    a_on.append(float(on.output_a_target))
    a_off.append(float(off.output_a_target))
    src_on.append(on.mpc.source)
  return np.array(a_on), np.array(a_off), src_on


class TestStopCommitThroughPlanner(OpenpilotTestCase):
  def test_no_lead_stop_deepens_the_published_ask(self):
    # 17c window (a) shape: model plans a stop ~100 m out from 17 m/s, but its instantaneous ask is only -0.4
    v, x, a = stop_plan(17., 100.)
    a_on, a_off, src = lockstep([{'v_ego': 17., 'desired_accel': -0.4, 'plan_v': v, 'plan_x': x}] * 80)
    self.assertAlmostEqual(a_off[-1], -0.4, places=3)        # stock: the instantaneous e2e ask wins
    self.assertLess(a_on[-1], -1.3)                          # committed toward the plan's own -1.45
    self.assertGreater(a_on[-1], -a - 0.05)                  # ...and never past it
    self.assertTrue(np.all(a_on <= a_off + 1e-9))            # only ever deeper
    self.assertLessEqual(np.max(-np.diff(a_on)), sc.J_IN * 0.05 + 1e-6)  # ramped, never stepped
    self.assertEqual(src[-1], lp.LongitudinalPlanSource.e2e)

  def test_lead_present_is_byte_identical(self):
    # occupied stop / steady follow: the flag must not change a single frame
    v, x, _ = stop_plan(17., 100.)
    frames = [{'v_ego': 17., 'desired_accel': -0.4, 'plan_v': v, 'plan_x': x, 'lead_present': True, 'd_rel': 60., 'v_lead': 12.}] * 60
    frames += [{'v_ego': 20., 'desired_accel': 0.05, 'plan_v': np.full(len(T), 20.), 'plan_x': 20. * T, 'lead_present': True, 'd_rel': 40., 'v_lead': 20.}] * 60
    a_on, a_off, _ = lockstep(frames)
    np.testing.assert_array_equal(a_on, a_off)

  def test_cruise_and_launch_are_byte_identical(self):
    frames = [{'v_ego': 25., 'desired_accel': 0.0, 'plan_v': np.full(len(T), 25.), 'plan_x': 25. * T}] * 40
    frames += [{'v_ego': 2., 'desired_accel': 1.2, 'plan_v': 2. + 1.2 * T, 'plan_x': 2. * T + 0.6 * T ** 2}] * 40
    a_on, a_off, _ = lockstep(frames)
    np.testing.assert_array_equal(a_on, a_off)

  def test_gas_press_releases_at_once(self):
    v, x, _ = stop_plan(17., 100.)
    frames = [{'v_ego': 17., 'desired_accel': -0.4, 'plan_v': v, 'plan_x': x}] * 60 + \
             [{'v_ego': 17., 'desired_accel': -0.4, 'plan_v': v, 'plan_x': x, 'gas': True}]
    on = make_planner(True)
    for kw in frames:
      on.update(planner_sm(**kw))
    self.assertEqual(on.stop_commit.delta, 0.)
