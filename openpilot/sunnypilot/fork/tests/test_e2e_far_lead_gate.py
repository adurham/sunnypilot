"""
Cycle 1 far-lead e2e gate (adurham fork, longitudinal_planner.LongitudinalPlanner.update_e2e_gate).

On routes 167/168 the vision model's accel wish (modelV2.action.desiredAcceleration) won the planner min() ~20 s before
stops and on leads sitting 1.2-2.3x the desired gap while the MPC candidate was still >= 0. The gate raises the RAW
e2e value to the MPC candidate while a lead is far and slow-closing. These tests pin the approved design:
enter hysteresis (10 frames), immediate exit, raise-only, bit-exact when there is no lead / shouldStop / v < 3, a cut-in
drops the gate the same frame, and a lead at exactly 1.2x with TTC 13 s does not gate.
"""
from types import SimpleNamespace
from unittest import mock

import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib import longitudinal_planner as lp
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import STOP_DISTANCE, get_T_FOLLOW
from openpilot.selfdrive.modeld.constants import ModelConstants

T_FOLLOW = get_T_FOLLOW(log.LongitudinalPersonality.standard)  # 1.45
N_ENTER = lp.E2E_GATE_ENTER_FRAMES


def d_des(v):
  return STOP_DISTANCE + T_FOLLOW * v


def lead(present=True, d_rel=60., v_rel=-1.):
  return SimpleNamespace(present=present, dRel=d_rel, vRel=v_rel)


class GateOnly:
  """The gate state machine alone (the method only touches self.e2e_gate / self.e2e_gate_cnt)."""
  def __init__(self):
    self.e2e_gate = False
    self.e2e_gate_cnt = 0

  def step(self, ld, v=15., e2e=-0.8, mpc=0.2, should_stop=False, t_follow=T_FOLLOW):
    return lp.LongitudinalPlanner.update_e2e_gate(self, ld, v, e2e, mpc, should_stop, t_follow)


V = 15.
FAR = lead(d_rel=1.5 * d_des(V), v_rel=-1.)  # ratio 1.5, TTC ~49 s: enter and stay both true


class TestGateLogic(OpenpilotTestCase):
  def test_d_des_matches_mpc_desired_distance(self):
    from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (get_safe_obstacle_distance,
                                                                                get_stopped_equivalence_factor)
    for v in (3.5, 10., 20., 30.):
      for tf in (1.25, 1.45, 1.75, 2.2):
        # the MPC's steady-state follow distance at equal speeds
        mpc_gap = get_safe_obstacle_distance(v, tf) - get_stopped_equivalence_factor(v)
        self.assertAlmostEqual(mpc_gap, STOP_DISTANCE + tf * v)

  def test_enter_hysteresis_10_frames(self):
    g = GateOnly()
    outs = [g.step(FAR) for _ in range(N_ENTER + 5)]
    self.assertEqual(N_ENTER, 10)
    for i in range(N_ENTER - 1):
      self.assertEqual(outs[i], -0.8, i)  # first 9 qualifying frames: untouched
    self.assertEqual(outs[N_ENTER - 1], 0.2)  # 10th qualifying frame: gated
    self.assertTrue(all(o == 0.2 for o in outs[N_ENTER - 1:]))

  def test_enter_counter_resets_on_a_break(self):
    g = GateOnly()
    for _ in range(N_ENTER - 1):
      g.step(FAR)
    g.step(lead(present=False))  # one non-qualifying frame
    outs = [g.step(FAR) for _ in range(N_ENTER)]
    self.assertTrue(all(o == -0.8 for o in outs[:N_ENTER - 1]))
    self.assertEqual(outs[-1], 0.2)

  def test_stay_band_holds_once_entered(self):
    # 1.15x / TTC 10 s / MPC -0.15 fails enter but satisfies stay: an entered gate holds, a fresh one never enters
    v = V
    mid = lead(d_rel=1.15 * d_des(v), v_rel=-1.15 * d_des(v) / 10.)
    g = GateOnly()
    for _ in range(N_ENTER):
      g.step(FAR)
    self.assertTrue(g.e2e_gate)
    for _ in range(50):
      self.assertEqual(g.step(mid, mpc=-0.15), -0.15)
    self.assertTrue(g.e2e_gate)
    fresh = GateOnly()
    for _ in range(50):
      self.assertEqual(fresh.step(mid, mpc=-0.15), -0.8)
    self.assertFalse(fresh.e2e_gate)

  def test_exit_is_immediate(self):
    exits = {
      'no lead': {'ld': lead(present=False)},
      'shouldStop': {'ld': FAR, 'should_stop': True},
      'v<3': {'ld': FAR, 'v': 2.9},
      'gap<1.1x': {'ld': lead(d_rel=1.09 * d_des(V), v_rel=-0.5)},
      'ttc<8': {'ld': lead(d_rel=1.5 * d_des(V), v_rel=-1.5 * d_des(V) / 7.9)},
      'vRel<-3.5': {'ld': lead(d_rel=200., v_rel=-3.6)},
      'mpc<-0.2': {'ld': FAR, 'mpc': -0.21},
    }
    for name, kw in exits.items():
      g = GateOnly()
      for _ in range(N_ENTER):
        g.step(FAR)
      self.assertTrue(g.e2e_gate, name)
      ld = kw.pop('ld')
      out = g.step(ld, **kw)
      self.assertFalse(g.e2e_gate, name)
      self.assertEqual(out, -0.8, name)  # raw e2e passed through on the exit frame itself
      # and it must re-qualify for 10 frames before gating again
      outs = [g.step(FAR) for _ in range(N_ENTER)]
      self.assertTrue(all(o == -0.8 for o in outs[:N_ENTER - 1]), name)
      self.assertEqual(outs[-1], 0.2, name)

  def test_synthetic_cut_in_drops_gate_same_frame(self):
    # far lead at 20 m/s ego, gate entered; a car cuts in at 18 m with -2 m/s closing; MPC goes to -1.0 that frame
    v = 20.
    g = GateOnly()
    for _ in range(N_ENTER + 20):
      g.step(lead(d_rel=2.0 * d_des(v), v_rel=-1.), v=v, e2e=-0.6, mpc=0.1)
    self.assertTrue(g.e2e_gate)
    out = g.step(lead(d_rel=18., v_rel=-2.), v=v, e2e=-1.4, mpc=-1.0)
    self.assertFalse(g.e2e_gate)
    self.assertEqual(out, -1.4)
    # each cut-in signature alone also drops it on the same frame
    for ld, mpc in ((lead(d_rel=1.05 * d_des(v), v_rel=-0.2), 0.1),         # gap only
                    (lead(d_rel=1.5 * d_des(v), v_rel=-1.5 * d_des(v) / 7.5), 0.1),  # TTC only
                    (lead(d_rel=2.0 * d_des(v), v_rel=-1.), -0.25)):         # MPC only
      g = GateOnly()
      for _ in range(N_ENTER):
        g.step(lead(d_rel=2.0 * d_des(v), v_rel=-1.), v=v, e2e=-0.6, mpc=0.1)
      self.assertTrue(g.e2e_gate)
      self.assertEqual(g.step(ld, v=v, e2e=-0.6, mpc=mpc), -0.6)
      self.assertFalse(g.e2e_gate)

  def test_1p2x_13s_does_not_gate(self):
    v = V
    d = 1.2 * d_des(v)
    g = GateOnly()
    for _ in range(200):
      self.assertEqual(g.step(lead(d_rel=d, v_rel=-d / 13.), v=v, mpc=0.0), -0.8)
    self.assertFalse(g.e2e_gate)
    # boundary sanity: just over 1.2x with TTC 13 s does gate (the 1.2x edge is what blocks it, not the TTC)
    g = GateOnly()
    outs = [g.step(lead(d_rel=1.21 * d_des(v), v_rel=-1.21 * d_des(v) / 13.), v=v, mpc=0.0) for _ in range(N_ENTER)]
    self.assertEqual(outs[-1], 0.0)

  def test_raise_only_on_every_frame(self):
    # piecewise-held random scenarios (so the 10-frame enter count is reachable) with per-frame jitter, crossing every
    # enter/stay/exit condition; the invariant is checked on every frame
    rng = np.random.default_rng(167168)
    g = GateOnly()
    n_raised = n = 0
    while n < 20000:
      v0 = float(rng.choice([rng.uniform(0., 3.5), rng.uniform(3., 35.)], p=[0.1, 0.9]))
      present = bool(rng.random() > 0.1)
      ratio0 = float(rng.choice([rng.uniform(0.5, 1.3), rng.uniform(1.0, 3.)], p=[0.3, 0.7]))
      vrel0 = float(rng.uniform(-4., 2.))
      mpc0 = float(rng.uniform(-0.6, 1.0))
      ss = bool(rng.random() < 0.05)
      for _ in range(int(rng.integers(1, 80))):
        v = max(v0 + float(rng.normal(0., 0.1)), 0.)
        ld = lead(present, (ratio0 + float(rng.normal(0., 0.03))) * d_des(v), vrel0 + float(rng.normal(0., 0.2)))
        e2e = float(rng.uniform(-3., 2.))
        mpc = mpc0 + float(rng.normal(0., 0.05))
        out = g.step(ld, v=v, e2e=e2e, mpc=mpc, should_stop=ss)
        self.assertGreaterEqual(out, e2e)
        self.assertIn(out, (e2e, max(e2e, mpc)))
        if not present or ss or v <= 3.:
          self.assertEqual(out, e2e)  # never touches a no-lead / shouldStop / slow frame
        n_raised += out > e2e
        n += 1
    self.assertGreater(n_raised, 1000)  # the property was actually exercised


# ---- through the REAL LongitudinalPlanner.update(): gate vs gate-disabled, in lockstep ----

def make_planner():
  from opendbc.car.honda.interface import CarInterface
  from opendbc.car.honda.values import CAR
  CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
  CP.openpilotLongitudinalControl = True
  CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.HONDA_CIVIC)
  return lp.LongitudinalPlanner(CP, CP_SP, init_v=15.)


def planner_sm(v_ego, desired_accel, should_stop=False, lead_present=True, d_rel=60., v_lead=None, steer=0.):
  N = len(ModelConstants.T_IDXS)
  msgs = {k: messaging.new_message(k) for k in ('modelV2', 'carState', 'carControl', 'controlsState', 'selfdriveState',
                                                'radarState', 'liveMapDataSP', 'vehicleParameters', 'carStateSP',
                                                'gpsLocation', 'gpsLocationExternal', 'selfdriveStateSP')}
  md = msgs['modelV2'].modelV2
  md.position.x = [v_ego * t for t in ModelConstants.T_IDXS]
  md.velocity.x = [v_ego] * N
  md.orientationRate.z = [0.] * N
  md.action.desiredAcceleration = float(desired_accel)
  md.action.shouldStop = should_stop
  md.meta.disengagePredictions.gasPressProbs = [1.] * 6
  cs = msgs['carState'].carState
  cs.vEgo = float(v_ego)
  cs.vCruise = 100.
  cs.vCruiseCluster = 100.
  cs.steeringAngleDeg = float(steer)
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


class TestGateThroughPlanner(OpenpilotTestCase):
  def _lockstep(self, frames):
    """Run a gated and a gate-disabled planner on the same inputs; return (gated, ungated) output_a_target lists and
    the gated planner's gate flags."""
    pg, pu = make_planner(), make_planner()
    pu.update_e2e_gate = lambda lead_, v, e2e, *a: e2e  # the old code path: raw e2e straight into the turn limit
    og, ou, flags = [], [], []
    for kw in frames:
      sm = planner_sm(**kw)
      pg.update(sm)
      pu.update(sm)
      og.append(float(pg.output_a_target))
      ou.append(float(pu.output_a_target))
      flags.append(pg.e2e_gate)
      self.assertEqual(bool(pg.output_should_stop), bool(pu.output_should_stop))
    return og, ou, flags

  def test_no_lead_bit_exact(self):
    frames = [{'v_ego': 15., 'desired_accel': -0.9 + 0.01 * i, 'lead_present': False} for i in range(100)]
    og, ou, flags = self._lockstep(frames)
    self.assertEqual(og, ou)
    self.assertFalse(any(flags))

  def test_should_stop_bit_exact(self):
    frames = [{'v_ego': 12., 'desired_accel': -1.0, 'should_stop': True, 'd_rel': 80., 'v_lead': 11.5} for _ in range(100)]
    og, ou, flags = self._lockstep(frames)
    self.assertEqual(og, ou)
    self.assertFalse(any(flags))

  def test_v_below_3_bit_exact(self):
    frames = [{'v_ego': 2.9, 'desired_accel': -0.7, 'd_rel': 40., 'v_lead': 2.9} for _ in range(100)]
    og, ou, flags = self._lockstep(frames)
    self.assertEqual(og, ou)
    self.assertFalse(any(flags))

  def test_far_lead_raises_plan_and_never_lowers(self):
    # lead 2x the desired gap at matching speed: the model is neutral (+0.1) while the gate enters, then its wish
    # drops to -0.9 (the 167/168 "unexplained" decel signature). The MPC stays ~0 because the plan follows it.
    v = 15.
    frames = ([{'v_ego': v, 'desired_accel': 0.1, 'd_rel': 2.0 * d_des(v), 'v_lead': v} for _ in range(20)] +
              [{'v_ego': v, 'desired_accel': -0.9, 'd_rel': 2.0 * d_des(v), 'v_lead': v} for _ in range(40)])
    og, ou, flags = self._lockstep(frames)
    self.assertFalse(any(flags[:N_ENTER - 1]))
    self.assertTrue(all(flags[N_ENTER - 1:]))
    self.assertGreater(og[-1], -0.3)   # the e2e wish no longer wins
    self.assertLess(ou[-1], -0.85)     # control: without the gate the model ask binds

  def test_gate_never_lowers_through_planner(self):
    # inside the real update(): on every frame the e2e value leaving the gate is >= the raw model value entering it,
    # and is either the raw value or max(raw, MPC candidate). Asks above and below the MPC, lead far then near.
    v = 15.
    p = make_planner()
    rec = []
    orig = lp.LongitudinalPlanner.update_e2e_gate

    def spy(self_, lead_, v_, e2e, mpc, ss, tf):
      out = orig(self_, lead_, v_, e2e, mpc, ss, tf)
      rec.append((e2e, mpc, out))
      return out
    with mock.patch.object(lp.LongitudinalPlanner, 'update_e2e_gate', spy):
      for i in range(300):
        e2e = (-1.5, -0.5, 0.0, 0.5, 1.5)[(i // 15) % 5]
        ratio = 2.0 if i < 200 else 1.05
        p.update(planner_sm(v, e2e, d_rel=ratio * d_des(v), v_lead=v))
    self.assertEqual(len(rec), 300)
    for e2e, mpc, out in rec:
      self.assertGreaterEqual(out, e2e)
      self.assertIn(out, (e2e, max(e2e, mpc)))
    self.assertTrue(any(out > e2e for e2e, _, out in rec[:200]))   # gate did act on the far lead
    self.assertTrue(all(out == e2e for e2e, _, out in rec[200:]))  # and not on the 1.05x lead

  def test_turn_limit_still_applies_after_the_gate(self):
    # raising e2e before the G4 turn limit must not lift it past the turn budget: in a hard turn the gated e2e is
    # still clipped by turn_accel_limit (floored 0.3), so the plan can never exceed the ungated min of the other two
    pg = make_planner()
    v = 12.
    for _ in range(40):
      pg.update(planner_sm(v, desired_accel=-0.9, d_rel=2.5 * d_des(v), v_lead=v + 3., steer=90.))
    self.assertTrue(pg.e2e_gate)
    limit = max(lp.turn_accel_limit(v, 90., pg.CP), 0.3)
    self.assertLessEqual(float(pg.output_a_target), limit + 1e-6)

  def test_gate_resets_with_a_new_planner(self):
    p = make_planner()
    self.assertFalse(p.e2e_gate)
    self.assertEqual(p.e2e_gate_cnt, 0)

  def test_enter_frames_is_half_a_second(self):
    self.assertAlmostEqual(N_ENTER * DT_MDL, 0.5)

  def test_gate_uses_mpc_t_follow(self):
    # with adaptive follow returning a t_follow, d_des uses that same value (not the stock personality gap)
    p = make_planner()
    seen = []
    orig = lp.LongitudinalPlanner.update_e2e_gate

    def spy(self_, lead_, v, e2e, mpc, ss, tf):
      seen.append(tf)
      return orig(self_, lead_, v, e2e, mpc, ss, tf)
    with mock.patch.object(lp.LongitudinalPlanner, 'update_e2e_gate', spy), \
         mock.patch.object(lp.LongitudinalPlannerSP, 'get_t_follow', lambda self_, sm, v: 2.05):
      p.update(planner_sm(15., -0.5, d_rel=60., v_lead=15.))
    self.assertEqual(seen, [2.05])
    seen.clear()
    with mock.patch.object(lp.LongitudinalPlanner, 'update_e2e_gate', spy), \
         mock.patch.object(lp.LongitudinalPlannerSP, 'get_t_follow', lambda self_, sm, v: None):
      p.update(planner_sm(15., -0.5, d_rel=60., v_lead=15.))
    self.assertEqual(seen, [T_FOLLOW])
