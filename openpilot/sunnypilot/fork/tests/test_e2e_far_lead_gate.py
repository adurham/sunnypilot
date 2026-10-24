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


def lead(present=True, d_rel=60., v_rel=-1., a_lead=0.):
  return SimpleNamespace(present=present, dRel=d_rel, vRel=v_rel, aLeadK=a_lead)


class GateOnly:
  """The gate state machine alone (the method only touches self.e2e_gate / self.e2e_gate_cnt)."""
  def __init__(self):
    self.e2e_gate = False
    self.e2e_gate_cnt = 0
    self.e2e_gate_alead = []

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
      'aLeadK<=-0.8': {'ld': lead(d_rel=1.5 * d_des(V), v_rel=-1., a_lead=-0.8)},
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
      alead0 = float(rng.choice([rng.uniform(-2.5, -0.4), rng.uniform(-0.6, 1.0)], p=[0.2, 0.8]))
      ss = bool(rng.random() < 0.05)
      for _ in range(int(rng.integers(1, 80))):
        v = max(v0 + float(rng.normal(0., 0.1)), 0.)
        ld = lead(present, (ratio0 + float(rng.normal(0., 0.03))) * d_des(v), vrel0 + float(rng.normal(0., 0.2)),
                  alead0 + float(rng.normal(0., 0.1)))
        e2e = float(rng.uniform(-3., 2.))
        mpc = mpc0 + float(rng.normal(0., 0.05))
        out = g.step(ld, v=v, e2e=e2e, mpc=mpc, should_stop=ss)
        self.assertGreaterEqual(out, e2e)
        self.assertIn(out, (e2e, max(e2e, mpc)))
        if not present or ss or v <= 3. or ld.aLeadK <= lp.E2E_GATE_STAY_ALEAD:
          self.assertEqual(out, e2e)  # never touches a no-lead / shouldStop / slow / braking-lead frame
        n_raised += out > e2e
        n += 1
    self.assertGreater(n_raised, 1000)  # the property was actually exercised


# ---- lead-decel veto (cycle 1 audit, route 168 17:14:55) ----
# leadOne.aLeadK is the lead's absolute accel, negative = braking (radard Track.aLeadK = Kalman accel state of vLead;
# vision lead: modelV2.leadsV3[0].a[0]). Enter needs aLeadK > -0.5, stay needs aLeadK > -0.8.

# route 168 17:14:52.49 - 17:14:56.74 CDT, 4 Hz decimation of the logged plan frames (gate-audit
# bad_episode_context.json): (vEgo, dRel, vRel = vLead - vEgo, aLeadK, raw e2e, MPC candidate)
LOG_168_171455 = [
  (16.76, 52.5, 0.29, -0.19, -0.02, 0.68), (16.74, 55.6, 0.96, 0.01, 0.02, 0.66), (16.72, 53.6, 0.47, -0.33, -0.06, 0.66),
  (16.70, 53.9, 0.83, -0.04, -0.06, 0.63), (16.69, 54.2, 0.89, 0.04, -0.05, 0.63), (16.67, 55.9, 1.01, -0.01, -0.05, 0.66),
  (16.64, 54.6, 0.25, -0.29, -0.18, 0.69), (16.63, 53.9, 0.42, -0.34, -0.24, 0.64), (16.61, 55.3, 0.99, -0.11, -0.28, 0.61),
  (16.59, 55.7, -1.34, -0.55, -0.44, 0.54), (16.55, 56.8, -0.45, -0.62, -0.45, 0.40), (16.50, 58.6, -1.09, -0.85, -0.49, 0.32),
  (16.43, 60.2, -1.75, -0.79, -0.53, 0.22), (16.33, 58.2, -1.69, -0.78, -0.60, 0.12), (16.24, 56.3, -1.66, -0.75, -0.64, 0.02),
  (16.10, 55.1, -2.42, -0.93, -0.76, -0.08), (15.89, 56.7, -1.44, -0.78, -0.68, -0.20), (15.52, 55.1, -2.65, -1.00, -0.70, -0.23),
]
# adaptive follow moved t_follow 1.45 -> ~1.97 over the trace (logged gap 1.73x -> 1.46x); the standard 1.45 is used, the
# gap/TTC/vRel/MPC stay terms hold on every row either way, so only the aLeadK veto decides the exit here
T_FOLLOW_168 = T_FOLLOW


class TestLeadDecelVeto(OpenpilotTestCase):
  def test_sign_convention(self):
    self.assertLess(lp.E2E_GATE_ENTER_ALEAD, 0.)
    self.assertLess(lp.E2E_GATE_STAY_ALEAD, lp.E2E_GATE_ENTER_ALEAD)  # stay is the looser (hysteresis) side

  def test_hard_braking_lead_blocks_entry(self):
    for a in (-0.5, -1.0, -1.5, -3.0):
      g = GateOnly()
      for _ in range(200):
        self.assertEqual(g.step(lead(d_rel=2.0 * d_des(V), v_rel=-0.5, a_lead=a)), -0.8, a)
      self.assertFalse(g.e2e_gate, a)

  def test_braking_lead_resets_the_enter_count(self):
    g = GateOnly()
    for _ in range(N_ENTER - 1):
      g.step(FAR)
    g.step(lead(d_rel=1.5 * d_des(V), v_rel=-1., a_lead=-1.0))
    self.assertEqual(g.e2e_gate_cnt, 0)
    outs = [g.step(FAR) for _ in range(N_ENTER)]
    self.assertTrue(all(o == -0.8 for o in outs[:N_ENTER - 1]))
    self.assertEqual(outs[-1], 0.2)

  def test_hard_braking_lead_drops_stay_same_frame(self):
    for a in (-0.8, -1.0, -1.7):
      g = GateOnly()
      for _ in range(N_ENTER + 5):
        g.step(FAR)
      self.assertTrue(g.e2e_gate)
      out = g.step(lead(d_rel=1.5 * d_des(V), v_rel=-1., a_lead=a), e2e=-0.73)
      self.assertFalse(g.e2e_gate, a)
      self.assertEqual(out, -0.73, a)  # the brake ask passes through on the exit frame

  def test_stay_band_between_enter_and_stay_veto(self):
    # -0.6 fails the enter veto but satisfies stay: an entered gate holds, a fresh one never enters
    mid = lead(d_rel=1.5 * d_des(V), v_rel=-1., a_lead=-0.6)
    g = GateOnly()
    for _ in range(N_ENTER):
      g.step(FAR)
    for _ in range(50):
      self.assertEqual(g.step(mid), 0.2)
    fresh = GateOnly()
    for _ in range(50):
      self.assertEqual(fresh.step(mid), -0.8)

  def test_mildly_braking_lead_still_gates(self):
    # -0.3 m/s^2 (lift-off / engine braking) is NOT a veto: the gate enters on time and holds
    mild = lead(d_rel=1.5 * d_des(V), v_rel=-1., a_lead=-0.3)
    g = GateOnly()
    outs = [g.step(mild) for _ in range(N_ENTER + 40)]
    self.assertTrue(all(o == -0.8 for o in outs[:N_ENTER - 1]))
    self.assertTrue(all(o == 0.2 for o in outs[N_ENTER - 1:]))

  def test_route_168_171455_shape(self):
    # the audit's failing case: gate active on a far lead, then the lead brakes (aLeadK -0.55 at 17:14:54.73, falling
    # to -1.0) while the e2e ask grows to -0.76. Old gate: e2e discarded until MPC < -0.2 (~1.5-2 s late).
    # New gate: e2e passes through from the first frame aLeadK <= -0.8, and never re-enters while aLeadK <= -0.5.
    g = GateOnly()
    for _ in range(N_ENTER):  # gate already active on the steady far lead before the trace starts
      v, d, vr, a, e2e, mpc = LOG_168_171455[0]
      g.step(lead(d_rel=d, v_rel=vr, a_lead=a), v=v, e2e=e2e, mpc=mpc, t_follow=T_FOLLOW_168)
    self.assertTrue(g.e2e_gate)
    first_veto = next(i for i, r in enumerate(LOG_168_171455) if r[3] <= lp.E2E_GATE_STAY_ALEAD)
    self.assertEqual(first_veto, 11)  # 17:14:55.23, aLeadK -0.85
    for i, (v, d, vr, a, e2e, mpc) in enumerate(LOG_168_171455):
      for _ in range(5):  # 4 Hz trace -> 20 Hz frames, each row held
        out = g.step(lead(d_rel=d, v_rel=vr, a_lead=a), v=v, e2e=e2e, mpc=mpc, t_follow=T_FOLLOW_168)
      if i < first_veto:
        self.assertTrue(g.e2e_gate, i)  # far, steady or mildly braking lead: still gated
        self.assertEqual(out, max(e2e, mpc), i)
      else:
        self.assertFalse(g.e2e_gate, i)  # braking lead: every e2e ask from here is kept
        self.assertEqual(out, e2e, i)
    # the -0.73 / -0.76 asks the old gate discarded (17:14:56.0-56.3) now reach the min() untouched
    self.assertGreater(len(LOG_168_171455) - first_veto, 5)


# ---- median + raw-floor ENTER (BRIEF H): enter needs median_0.3s(aLeadK) > -0.5 AND raw aLeadK > -0.8 ----
N_MED = lp.E2E_GATE_ENTER_ALEAD_MEDIAN_FRAMES


def far(a):
  return lead(d_rel=1.5 * d_des(V), v_rel=-1., a_lead=a)


class TestMedianEnter(OpenpilotTestCase):
  def test_window_is_0p3s(self):
    self.assertEqual(N_MED, 6)
    self.assertAlmostEqual(N_MED * DT_MDL, 0.3)

  def test_noise_spike_does_not_block_entry(self):
    # one- and two-frame vision spikes in the -0.8 .. -0.5 band (the 167 16:15:16 shape, smaller than the stay floor)
    # leave the 6-frame median above -0.5: the 10-frame entry count runs straight through them
    for spike in ([-0.6], [-0.75], [-0.79, -0.7]):
      g = GateOnly()
      seq = [far(0.)] * 4 + [far(a) for a in spike] + [far(0.)] * (N_ENTER - 4 - len(spike))
      outs = [g.step(ld) for ld in seq]
      self.assertEqual(len(outs), N_ENTER)
      self.assertEqual(outs[-1], 0.2, spike)  # entered on the 10th frame, spike included
      self.assertTrue(g.e2e_gate, spike)

  def test_raw_spike_below_stay_floor_still_resets_entry(self):
    # a single raw value <= -0.8 fails the raw floor even though the median is fine: entry restarts
    g = GateOnly()
    for _ in range(N_ENTER - 1):
      g.step(far(0.))
    g.step(far(-0.8))
    self.assertEqual(g.e2e_gate_cnt, 0)
    self.assertFalse(g.e2e_gate)

  def test_genuine_hard_braking_lead_still_blocks(self):
    # a lead that really brakes (sustained, ramping to -1.0 .. -3.0) never enters, from any starting state
    for a_end in (-1.0, -1.5, -3.0):
      g = GateOnly()
      ramp = [far(a_end * k / 5.) for k in range(1, 6)] + [far(a_end)] * 200
      outs = [g.step(ld) for ld in ramp]
      self.assertFalse(any(o != -0.8 for o in outs), a_end)
      self.assertFalse(g.e2e_gate, a_end)

  def test_sustained_band_braking_blocks_via_median(self):
    # a lead holding -0.6 (above the raw floor, below the enter threshold) is blocked by the median alone
    g = GateOnly()
    for _ in range(200):
      self.assertEqual(g.step(far(-0.6)), -0.8)
    self.assertFalse(g.e2e_gate)

  def test_median_lag_cannot_admit_below_floor(self):
    # the median still reads ~0 for 3 frames after a step to -2.0; the raw floor stops entry on the first frame
    g = GateOnly()
    for _ in range(N_ENTER - 1):
      g.step(far(0.))
    out = g.step(far(-2.0))
    self.assertEqual(out, -0.8)
    self.assertFalse(g.e2e_gate)
    self.assertEqual(g.e2e_gate_cnt, 0)

  def test_enter_subset_of_stay_randomized(self):
    # ENTER ⊆ STAY on aLeadK: whenever the enter count advances, the raw aLeadK satisfies the stay veto; and an active
    # gate is always released on the first raw aLeadK <= -0.8 frame (no smoothing on STAY)
    rng = np.random.default_rng(7)
    g = GateOnly()
    for _ in range(40000):
      a = float(rng.choice([rng.normal(0., 0.25), rng.uniform(-2., -0.4)], p=[0.85, 0.15]))
      cnt0, gate0 = g.e2e_gate_cnt, g.e2e_gate
      out = g.step(far(a), e2e=-0.8)
      if g.e2e_gate_cnt > cnt0 or (g.e2e_gate and not gate0):
        self.assertGreater(a, lp.E2E_GATE_STAY_ALEAD)
      if a <= lp.E2E_GATE_STAY_ALEAD:
        self.assertFalse(g.e2e_gate)
        self.assertEqual(out, -0.8)

  def test_median_buffer_resets_without_lead(self):
    g = GateOnly()
    for _ in range(N_MED):
      g.step(far(-0.7))  # median -0.7: blocked
    g.step(lead(present=False))
    self.assertEqual(g.e2e_gate_alead, [])
    outs = [g.step(far(0.)) for _ in range(N_ENTER)]
    self.assertEqual(outs[-1], 0.2)  # the old braking history does not leak across a lead loss

  def test_stay_is_unfiltered(self):
    # once active, a single raw -0.8 drops the gate even when the median buffer is all zeros
    g = GateOnly()
    for _ in range(N_ENTER + 10):
      g.step(far(0.))
    self.assertTrue(g.e2e_gate)
    out = g.step(far(-0.8), e2e=-0.73)
    self.assertFalse(g.e2e_gate)
    self.assertEqual(out, -0.73)

  def test_raw_enter_would_have_blocked_the_spike(self):
    # control: with the median disabled (window 1) the same -0.6 spike resets the entry, so the spike test is not vacuous
    with mock.patch.object(lp, 'E2E_GATE_ENTER_ALEAD_MEDIAN_FRAMES', 1):
      g = GateOnly()
      outs = [g.step(ld) for ld in [far(0.)] * 4 + [far(-0.6)] + [far(0.)] * (N_ENTER - 5)]
      self.assertEqual(outs[-1], -0.8)
      self.assertFalse(g.e2e_gate)


# ---- through the REAL LongitudinalPlanner.update(): gate vs gate-disabled, in lockstep ----

def make_planner():
  from opendbc.car.honda.interface import CarInterface
  from opendbc.car.honda.values import CAR
  CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
  CP.openpilotLongitudinalControl = True
  CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.HONDA_CIVIC)
  return lp.LongitudinalPlanner(CP, CP_SP, init_v=15.)


def planner_sm(v_ego, desired_accel, should_stop=False, lead_present=True, d_rel=60., v_lead=None, steer=0.,
               a_lead=0.):
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
    l1.aLeadK = float(a_lead)
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

  def test_braking_far_lead_keeps_e2e_through_planner(self):
    # a lead 4x the desired gap braking at -1.0 m/s^2: far enough that the MPC candidate stays >= -0.1 (without the veto
    # the gate enters on frame 10 and the plan sits at 0.0), so only the veto keeps the -0.9 e2e ask binding
    v = 15.
    frames = [{'v_ego': v, 'desired_accel': -0.9, 'd_rel': 4.0 * d_des(v), 'v_lead': v, 'a_lead': -1.0} for _ in range(60)]
    og, ou, flags = self._lockstep(frames)
    self.assertFalse(any(flags))
    self.assertEqual(og, ou)
    self.assertLess(og[-1], -0.85)
    # control: the same frames with the veto thresholds disabled DO gate, so the test is not vacuous
    with mock.patch.object(lp, 'E2E_GATE_ENTER_ALEAD', -99.), mock.patch.object(lp, 'E2E_GATE_STAY_ALEAD', -99.):
      og2, _, flags2 = self._lockstep(frames)
    self.assertTrue(any(flags2))
    self.assertGreater(og2[-1], -0.3)

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
