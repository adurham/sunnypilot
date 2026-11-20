"""
Cycle 1 far-lead e2e gate (adurham fork, longitudinal_planner.LongitudinalPlanner.update_e2e_gate).

On routes 167/168 the vision model's accel wish (modelV2.action.desiredAcceleration) won the planner min() ~20 s before
stops and on leads sitting 1.2-2.3x the desired gap while the MPC candidate was still >= 0. The gate raises the RAW
e2e value to the MPC candidate while a lead is far and slow-closing. These tests pin the approved design:
enter hysteresis (10 frames), immediate exit, raise-only, bit-exact when there is no lead / shouldStop / v < 3, a cut-in
drops the gate the same frame, and a lead at exactly 1.2x with TTC 13 s does not gate.

Cycle 2 adds the closing-rate veto (route 169 16b cut-in, t=1647.015): no entry and a same-frame exit while the lead is
closing faster than -2.0 m/s inside 3x d_des (entry also tests the 0.3 s median against -1.5), plus a 4 s rebound latch
after any sustained closure at the -3.0 m/s enter floor, so vRel oscillating back above -3.0 on a lead that cut in
2 s earlier cannot open the gate. The incident profile is replayed frame by frame from the logged drive, the
matched-speed case that the gate was built for is pinned as unchanged, and the mutation check (veto disabled -> the
profile gates on the same frame) is a permanent test.
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
  """The gate state machine alone (the method only touches self.e2e_gate / self.e2e_gate_cnt / its buffers)."""
  def __init__(self):
    self.e2e_gate = False
    self.e2e_gate_cnt = 0
    self.e2e_gate_alead = []
    self.e2e_gate_vrel = []
    self.e2e_gate_latch = 0

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
    # 1.15x / TTC 19 s / MPC -0.15 fails enter but satisfies stay: an entered gate holds, a fresh one never enters.
    # (The old mid frame closed at 3.19 m/s, TTC 10 s: the closing-rate veto now treats that as a cut-in-class frame,
    # so the stay-band case moved to a lead still matched inside the veto band.)
    v = V
    mid = lead(d_rel=1.15 * d_des(v), v_rel=-1.7)
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
    # boundary sanity: just over 1.2x at a matched speed does gate (the 1.2x edge is what blocks it, not the TTC).
    # The same 1.21x frame closing at TTC 13 s (-2.58 m/s) is now a closing-rate veto frame: that is the point.
    g = GateOnly()
    outs = [g.step(lead(d_rel=1.21 * d_des(v), v_rel=-1.), v=v, mpc=0.0) for _ in range(N_ENTER)]
    self.assertEqual(outs[-1], 0.0)

  def test_raise_only_on_every_frame(self):
    # piecewise-held random scenarios (so the 10-frame enter count is reachable) with per-frame jitter, crossing every
    # enter/stay/exit condition; the invariant is checked on every frame
    rng = np.random.default_rng(167168)
    g = GateOnly()
    n_raised = n = 0
    while n < 40000:
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
        self.assertIn(out, (e2e, max(e2e, min(mpc, lp.E2E_GATE_MAX_RAISE))))
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
        self.assertEqual(out, max(e2e, min(mpc, lp.E2E_GATE_MAX_RAISE)), i)
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
      self.assertIn(out, (e2e, max(e2e, min(mpc, lp.E2E_GATE_MAX_RAISE))))
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


# ---- closing-rate veto (cycle 2): route 169 16b cut-in at t=1647.0 ----
# Measured harm (report ~/comma-sim/wsSTOP/drive-169-16c/sub/gate-replay/REPORT.md, cut_in.rows): a lead appears 116 m
# ahead closing -5.8..-7.2 m/s, the gate turned ON at 1647.015 the moment vRel oscillated back above its -3.0 enter
# floor, raised the model's wish (-0.26 raw) to the MPC candidate (+0.60) and kept the car accelerating while the gap
# fell to 52 m (1.27x d_des) and v_ego rose 19.25 -> 21.27 m/s; the driver braked at 1650.1 and stayed out for 31 s.
# Per-frame columns below: (t, v_ego, dRel, vRel, aLeadK, raw e2e, MPC candidate, t_follow the replay fed the gate).
ROUTE_16B_CUT_IN = [
  (1645.015, 19.38,  116.0, -5.81, -0.22, -0.449,  1.622, 1.65),
  (1645.066, 19.38,  108.5, -7.18, -0.47, -0.445,  1.559, 1.70),
  (1645.116, 19.37,  106.6, -5.32, -0.11, -0.432,  1.492, 1.75),
  (1645.166, 19.37,  101.8, -4.44, -0.11, -0.407,  1.423, 1.80),
  (1645.216, 19.36,  101.5, -2.57, -0.10, -0.392,  1.365, 1.85),
  (1645.267, 19.36,  101.3, -3.45, -0.09, -0.376,  1.305, 1.90),
  (1645.316, 19.36,  103.1, -4.00, -0.16, -0.368,  1.241, 1.95),
  (1645.367, 19.32,  100.3, -3.72, -0.20, -0.366,  1.183, 2.00),
  (1645.416, 19.31,   99.7, -1.62, -0.15, -0.346,  1.150, 1.99),
  (1645.466, 19.30,   99.1, -1.37, -0.07, -0.313,  1.110, 1.98),
  (1645.516, 19.28,   98.4, -1.20, -0.05, -0.300,  1.070, 1.97),
  (1645.566, 19.27,  102.4, -1.69, -0.10, -0.273,  1.043, 1.96),
  (1645.616, 19.26,  102.6, -2.78, -0.08, -0.250,  1.001, 2.01),
  (1645.666, 19.25,   98.3, -3.54, -0.05, -0.231,  0.956, 2.05),
  (1645.716, 19.27,  100.7, -2.73, -0.03, -0.234,  0.929, 2.04),
  (1645.766, 19.26,   99.0, -4.56, -0.02, -0.217,  0.897, 2.05),
  (1645.817, 19.27,   99.1, -1.74, -0.03, -0.226,  0.882, 2.04),
  (1645.866, 19.26,   96.8, -4.33, -0.05, -0.241,  0.852, 2.05),
  (1645.917, 19.26,  101.2, -2.84, -0.02, -0.267,  0.845, 2.04),
  (1645.966, 19.26,   97.8, -3.94, -0.03, -0.415,  0.825, 2.05),
  (1646.015, 19.27,   95.0, -6.11,  0.13, -0.418,  0.842, 2.05),
  (1646.066, 19.28,  100.2, -3.45,  0.05, -0.396,  0.797, 2.05),
  (1646.116, 19.29,   96.7, -3.31,  0.00, -0.396,  0.748, 2.05),
  (1646.166, 19.29,   93.7, -2.74, -0.01, -0.369,  0.720, 2.04),
  (1646.216, 19.29,   97.0, -1.36, -0.03, -0.349,  0.699, 2.03),
  (1646.266, 19.29,   95.7, -1.87, -0.05, -0.346,  0.680, 2.02),
  (1646.317, 19.28,   94.1, -2.09, -0.03, -0.359,  0.670, 2.01),
  (1646.366, 19.28,   93.2, -2.63, -0.02, -0.342,  0.662, 2.00),
  (1646.415, 19.30,   94.3, -1.20, -0.02, -0.369,  0.655, 1.99),
  (1646.466, 19.29,   94.5, -1.62,  0.09, -0.382,  0.671, 1.98),
  (1646.516, 19.35,   90.5, -3.14,  0.05, -0.370,  0.655, 2.03),
  (1646.566, 19.29,   88.0, -1.80, -0.03, -0.349,  0.641, 2.02),
  (1646.616, 19.32,   86.8, -1.48, -0.02, -0.341,  0.627, 2.01),
  (1646.666, 19.33,   85.6, -1.01, -0.05, -0.347,  0.628, 2.00),
  (1646.716, 19.31,   91.2, -1.72, -0.07, -0.359,  0.635, 1.99),
  (1646.766, 19.31,   92.5, -1.83, -0.04, -0.355,  0.643, 1.98),
  (1646.816, 19.30,   95.6, -2.44, -0.04, -0.336,  0.639, 1.97),
  (1646.866, 19.29,   84.2, -1.90, -0.05, -0.311,  0.623, 1.96),
  (1646.916, 19.28,   87.3, -1.73, -0.08, -0.281,  0.613, 1.95),
  (1646.967, 19.31,   85.2, -1.26, -0.05, -0.265,  0.607, 1.94),
  (1647.015, 19.25,   84.6, -2.01, -0.04, -0.255,  0.603, 1.93),
  (1647.066, 19.31,   87.7, -1.48, -0.03, -0.242,  0.217, 1.92),
  (1647.116, 19.32,   89.1, -1.30, -0.04, -0.230,  0.563, 1.91),
  (1647.166, 19.32,   83.1, -0.91, -0.02, -0.215,  0.419, 1.90),
  (1647.216, 19.33,   84.3, -1.35, -0.00, -0.186,  0.571, 1.89),
  (1647.266, 19.32,   92.2, -1.42, -0.03, -0.157,  0.532, 1.88),
  (1647.316, 19.37,   84.6, -1.11, -0.02, -0.138,  0.603, 1.87),
  (1647.367, 19.34,   86.9, -0.27, -0.02, -0.149,  0.610, 1.86),
  (1647.416, 19.35,   84.7, -1.36, -0.03, -0.146,  0.639, 1.85),
  (1647.466, 19.39,   83.0, -1.50, -0.01, -0.152,  0.647, 1.84),
  (1647.516, 19.39,   85.7, -1.80, -0.03, -0.144,  0.665, 1.83),
  (1647.566, 19.42,   85.1, -0.97, -0.01, -0.138,  0.682, 1.82),
  (1647.616, 19.45,   82.8, -1.81, -0.01, -0.124,  0.685, 1.81),
  (1647.666, 19.51,   91.2, -1.75, -0.00, -0.111,  0.704, 1.80),
  (1647.716, 19.51,   78.4, -1.59,  0.00, -0.106,  0.699, 1.79),
  (1647.766, 19.55,   85.2, -1.85, -0.01, -0.092,  0.711, 1.78),
  (1647.816, 19.56,   82.2, -1.96, -0.01, -0.074,  0.707, 1.77),
  (1647.866, 19.64,   83.4, -0.29, -0.00, -0.077,  0.729, 1.76),
  (1647.916, 19.65,   81.9, -0.42,  0.02, -0.079,  0.729, 1.75),
  (1647.966, 19.69,   77.7, -0.50,  0.03, -0.082,  0.737, 1.74),
  (1648.016, 19.69,   79.9, -0.47,  0.01, -0.067,  0.743, 1.73),
  (1648.066, 19.76,   80.8, -0.90,  0.01, -0.048,  0.745, 1.72),
  (1648.117, 19.76,   79.5, -1.98,  0.00, -0.032,  0.738, 1.73),
  (1648.166, 19.78,   77.6, -1.98, -0.01, -0.013,  0.734, 1.73),
  (1648.216, 19.82,   69.0, -1.72, -0.03,  0.001,  0.720, 1.72),
  (1648.266, 19.85,   79.5, -0.86,  0.00,  0.001,  0.731, 1.71),
  (1648.317, 19.89,   76.4, -1.03,  0.01, -0.007,  0.720, 1.70),
  (1648.367, 19.94,   78.8, -1.00,  0.01,  0.002,  0.725, 1.69),
  (1648.417, 19.98,   73.7, -0.36,  0.01, -0.003,  0.720, 1.68),
  (1648.466, 20.01,   72.8, -0.80,  0.01, -0.002,  0.716, 1.67),
  (1648.516, 20.06,   71.2, -0.34,  0.03, -0.001,  0.715, 1.66),
  (1648.566, 20.08,   72.1,  0.41,  0.05,  0.007,  0.722, 1.65),
  (1648.615, 20.13,   70.2,  0.39,  0.03,  0.014,  0.721, 1.64),
  (1648.666, 20.16,   70.1, -0.34,  0.03,  0.019,  0.718, 1.63),
  (1648.716, 20.18,   66.4, -0.70,  0.02,  0.013,  0.710, 1.62),
  (1648.766, 20.20,   66.8, -0.48,  0.04,  0.012,  0.709, 1.61),
  (1648.817, 20.24,   65.9, -0.25,  0.05,  0.001,  0.705, 1.60),
  (1648.866, 20.26,   66.8, -0.92,  0.04, -0.005,  0.697, 1.59),
  (1648.916, 20.31,   68.6, -0.08,  0.05, -0.014,  0.702, 1.58),
  (1648.967, 20.34,   66.7, -1.17,  0.04, -0.022,  0.686, 1.57),
  (1649.016, 20.37,   66.3, -0.18,  0.07, -0.023,  0.693, 1.56),
  (1649.067, 20.43,   67.9, -0.78,  0.05, -0.035,  0.680, 1.55),
  (1649.117, 20.47,   71.0, -1.18,  0.06, -0.038,  0.679, 1.55),
  (1649.167, 20.51,   67.0, -0.51,  0.05, -0.038,  0.673, 1.54),
  (1649.217, 20.55,   66.4,  0.16,  0.05, -0.040,  0.676, 1.53),
  (1649.266, 20.64,   64.4, -0.58,  0.03, -0.040,  0.664, 1.52),
  (1649.317, 20.67,   61.7, -0.11,  0.04, -0.016,  0.665, 1.51),
  (1649.367, 20.71,   61.8,  0.17,  0.06,  0.012,  0.661, 1.50),
  (1649.417, 20.77,   62.5,  0.23,  0.05,  0.014,  0.663, 1.49),
  (1649.468, 20.75,   64.1, -0.25,  0.03,  0.000,  0.660, 1.48),
  (1649.517, 20.76,   61.9, -1.08, -0.02, -0.020,  0.642, 1.53),
  (1649.566, 20.84,   63.0, -0.48,  0.02, -0.033,  0.645, 1.52),
  (1649.616, 20.87,   62.5,  0.45,  0.04, -0.034,  0.645, 1.51),
  (1649.667, 20.94,   63.0,  0.01,  0.02, -0.051,  0.642, 1.50),
  (1649.717, 21.00,   58.5, -1.06,  0.02, -0.041,  0.629, 1.53),
  (1649.766, 20.99,   60.2, -0.95, -0.01, -0.037,  0.632, 1.52),
  (1649.816, 21.07,   57.3, -1.44, -0.08, -0.023,  0.592, 1.57),
  (1649.866, 21.14,   58.9, -0.42, -0.04, -0.007,  0.603, 1.56),
  (1649.917, 21.17,   54.4, -0.76, -0.12, -0.014,  0.594, 1.55),
  (1649.967, 21.21,   51.9, -1.87, -0.24, -0.045,  0.558, 1.60),
  (1650.016, 21.27,   52.3, -3.10, -0.17, -0.065,  0.513, 1.65),
  (1650.066, 21.29,   51.7, -0.61, -0.07, -0.079,  0.506, 1.64),
]
CUT_IN_ENTRY = 40  # index of t=1647.015, the frame the gate turned ON (verified against the replay's 62 ON frames)


def cut_in_replay():
  """Replay ROUTE_16B_CUT_IN at 20 Hz through the gate alone; return the per-frame outputs and gate flags."""
  g = GateOnly()
  outs, flags = [], []
  for t, v, d_rel, v_rel, a_lead, e2e, mpc, t_follow in ROUTE_16B_CUT_IN:
    outs.append(g.step(lead(d_rel=d_rel, v_rel=v_rel, a_lead=a_lead), v=v, e2e=e2e, mpc=mpc, t_follow=t_follow))
    flags.append(g.e2e_gate)
  return outs, flags


class TestClosingRateVeto(OpenpilotTestCase):
  def test_constants(self):
    # the stay side must be the looser one (it tolerates more closing than entry), the band is negative closing,
    # the veto's gap band is wider than the enter band (the gate acts at 1.27-1.86x on these drives), and the two
    # median/latch windows are 0.3 s and 4.0 s at 20 Hz
    self.assertLess(lp.E2E_GATE_STAY_VREL_CLOSE, lp.E2E_GATE_ENTER_VREL_CLOSE)
    self.assertLess(lp.E2E_GATE_ENTER_VREL_CLOSE, 0.)
    self.assertGreater(lp.E2E_GATE_VREL_CLOSE_GAP, lp.E2E_GATE_ENTER_GAP)
    self.assertAlmostEqual(lp.E2E_GATE_ENTER_VREL_MEDIAN_FRAMES * DT_MDL, 0.3)
    self.assertAlmostEqual(lp.E2E_GATE_REBOUND_FRAMES * DT_MDL, 4.0)

  def test_16b_cut_in_never_gates(self):
    # (a) the incident profile: the gate never turns ON and the model's wish is never raised, on any of the 102 frames
    outs, flags = cut_in_replay()
    self.assertFalse(any(flags))
    for i, (row, out) in enumerate(zip(ROUTE_16B_CUT_IN, outs)):
      self.assertEqual(out, row[5], i)  # raw e2e through, bit-exact
    # the entry frame is blocked by all three terms, not by luck: raw past the stay floor, 0.3 s median past the enter
    # threshold, and the rebound latch still armed from the -6.1 m/s closure 1 s earlier
    g = GateOnly()
    for row in ROUTE_16B_CUT_IN[:CUT_IN_ENTRY]:
      g.step(lead(d_rel=row[2], v_rel=row[3], a_lead=row[4]), v=row[1], e2e=row[5], mpc=row[6], t_follow=row[7])
    t, v, d_rel, v_rel = ROUTE_16B_CUT_IN[CUT_IN_ENTRY][:4]
    self.assertEqual(t, 1647.015)
    self.assertLess(v_rel, lp.E2E_GATE_STAY_VREL_CLOSE)
    self.assertLess(float(np.median([r[3] for r in ROUTE_16B_CUT_IN[CUT_IN_ENTRY - 6:CUT_IN_ENTRY]])),
                    lp.E2E_GATE_ENTER_VREL_CLOSE)
    self.assertGreater(g.e2e_gate_latch, 0)
    out = g.step(lead(d_rel=d_rel, v_rel=v_rel), v=v, e2e=ROUTE_16B_CUT_IN[CUT_IN_ENTRY][5],
                 mpc=ROUTE_16B_CUT_IN[CUT_IN_ENTRY][6], t_follow=ROUTE_16B_CUT_IN[CUT_IN_ENTRY][7])
    self.assertEqual(g.e2e_gate_cnt, 0)
    self.assertFalse(g.e2e_gate)
    self.assertEqual(out, ROUTE_16B_CUT_IN[CUT_IN_ENTRY][5])

  def test_16b_cut_in_would_gate_without_the_veto(self):
    # mutation check: disable the veto (thresholds and latch) and the same profile gates on the same frame, 62 frames
    with mock.patch.object(lp, 'E2E_GATE_ENTER_VREL_CLOSE', -99.), \
         mock.patch.object(lp, 'E2E_GATE_STAY_VREL_CLOSE', -99.), \
         mock.patch.object(lp, 'E2E_GATE_REBOUND_FRAMES', 0):
      _, flags = cut_in_replay()
    self.assertEqual(sum(flags), 62)
    self.assertEqual(flags.index(True), CUT_IN_ENTRY)

  def test_matched_far_lead_still_enters(self):
    # (b) the case the gate was built for: far lead, matching to within 1-2 m/s -> unchanged
    for v_rel in (-0.5, -1.0):
      g = GateOnly()
      outs = [g.step(lead(d_rel=1.5 * d_des(V), v_rel=v_rel)) for _ in range(N_ENTER + 20)]
      self.assertTrue(all(o == -0.8 for o in outs[:N_ENTER - 1]), v_rel)
      self.assertTrue(all(o == 0.2 for o in outs[N_ENTER - 1:]), v_rel)
    # and inside the veto's tightest band only at entry: a sustained -1.4 m/s closure still gates
    g = GateOnly()
    outs = [g.step(lead(d_rel=2.0 * d_des(V), v_rel=-1.4)) for _ in range(N_ENTER)]
    self.assertEqual(outs[-1], 0.2)

  def test_closing_lead_at_short_gap_drops_same_frame(self):
    # (c) an entered gate and a lead that passes every pre-existing stay test but is closing -2.2 m/s at 1.6x d_des
    # (TTC 25 s, gap > 1.1x, vRel > -3.5, MPC +0.1, aLeadK 0): the gate drops on that frame and the ask passes through
    v = 20.
    d = 1.6 * d_des(v)
    ld = lead(d_rel=d, v_rel=-2.2)
    self.assertGreater(d, lp.E2E_GATE_STAY_GAP * d_des(v))
    self.assertGreater(d / -ld.vRel, lp.E2E_GATE_STAY_TTC)
    self.assertGreater(ld.vRel, lp.E2E_GATE_STAY_VREL)
    g = GateOnly()
    for _ in range(N_ENTER + 5):
      g.step(lead(d_rel=1.5 * d_des(v), v_rel=-1.), v=v, e2e=-0.6, mpc=0.1)
    self.assertTrue(g.e2e_gate)
    self.assertEqual(g.step(ld, v=v, e2e=-0.9, mpc=0.1), -0.9)
    self.assertFalse(g.e2e_gate)
    # control: with the veto disabled the same frame holds the gate (so only the veto explains the drop)
    with mock.patch.object(lp, 'E2E_GATE_ENTER_VREL_CLOSE', -99.), \
         mock.patch.object(lp, 'E2E_GATE_STAY_VREL_CLOSE', -99.):
      g2 = GateOnly()
      for _ in range(N_ENTER + 5):
        g2.step(lead(d_rel=1.5 * d_des(v), v_rel=-1.), v=v, e2e=-0.6, mpc=0.1)
      self.assertEqual(g2.step(ld, v=v, e2e=-0.9, mpc=0.1), 0.1)
      self.assertTrue(g2.e2e_gate)

  def test_veto_never_raises_when_the_gate_is_off(self):
    # (d) bit-exact: with the gate off, a closing lead never gets its ask raised, however attractive the MPC is
    for v_rel in (-2.0, -2.2, -3.0, -5.0, float('nan')):
      for e2e in (-2.0, -0.5, 0.0, 1.5):
        g = GateOnly()
        out = g.step(lead(d_rel=1.5 * d_des(V), v_rel=v_rel), e2e=e2e, mpc=e2e + 2.0)
        self.assertFalse(g.e2e_gate, (v_rel, e2e))
        self.assertEqual(out, e2e, (v_rel, e2e))
    # control: the same frames at a matched speed DO raise, so the check is not vacuous
    g = GateOnly()
    outs = [g.step(lead(d_rel=1.5 * d_des(V), v_rel=-0.5), e2e=-2.0, mpc=0.0) for _ in range(N_ENTER)]
    self.assertEqual(outs[-1], 0.0)

  def test_gap_band_limits_where_the_veto_applies(self):
    # inside 3x d_des a sustained -2.5 m/s closure never enters; beyond it the gate still serves the far lead
    inside = lead(d_rel=2.9 * d_des(V), v_rel=-2.5)
    outside = lead(d_rel=3.5 * d_des(V), v_rel=-2.5)
    g = GateOnly()
    self.assertTrue(all(g.step(inside) == -0.8 for _ in range(100)))
    self.assertFalse(g.e2e_gate)
    g = GateOnly()
    self.assertEqual([g.step(outside) for _ in range(N_ENTER)][-1], 0.2)
    self.assertTrue(g.e2e_gate)

  def test_rebound_latch_blocks_entry_after_a_fast_closure(self):
    # a 0.5 s closure at -5 m/s arms E2E_GATE_REBOUND_FRAMES of latch; a lead that is then matched cannot open the gate
    # until the latch expires, and the 10-frame enter count starts only after that
    g = GateOnly()
    for _ in range(N_ENTER):
      self.assertEqual(g.step(lead(d_rel=2.0 * d_des(V), v_rel=-5.)), -0.8)
    self.assertEqual(g.e2e_gate_latch, lp.E2E_GATE_REBOUND_FRAMES)
    matched = lead(d_rel=1.5 * d_des(V), v_rel=-0.5)
    outs = [g.step(matched) for _ in range(lp.E2E_GATE_REBOUND_FRAMES + N_ENTER + 5)]
    first_on = outs.index(0.2)
    # 80 latch frames, then a fresh 10-frame enter count: the whole 4 s is lost, nothing overlaps
    self.assertEqual(first_on, lp.E2E_GATE_REBOUND_FRAMES + N_ENTER)
    self.assertTrue(all(o == -0.8 for o in outs[:first_on]))

  def test_latch_does_not_count_down_inside_the_veto_band(self):
    # a frame at -2.5 m/s (inside the band, above the -3.0 fast-closure floor) freezes the countdown instead of
    # releasing it: ambiguous -> fail toward OFF
    g = GateOnly()
    for _ in range(N_ENTER):
      g.step(lead(d_rel=2.0 * d_des(V), v_rel=-5.))
    self.assertEqual(g.e2e_gate_latch, lp.E2E_GATE_REBOUND_FRAMES)
    for _ in range(200):
      g.step(lead(d_rel=1.5 * d_des(V), v_rel=-2.5))
    self.assertEqual(g.e2e_gate_latch, lp.E2E_GATE_REBOUND_FRAMES)
    self.assertFalse(g.e2e_gate)

  def test_no_lead_clears_the_latch_and_the_vrel_buffer(self):
    g = GateOnly()
    for _ in range(N_ENTER):
      g.step(lead(d_rel=2.0 * d_des(V), v_rel=-5.))
    g.step(lead(present=False))
    self.assertEqual(g.e2e_gate_latch, 0)
    self.assertEqual(g.e2e_gate_vrel, [])
    outs = [g.step(lead(d_rel=1.5 * d_des(V), v_rel=-0.5)) for _ in range(N_ENTER)]
    self.assertEqual(outs[-1], 0.2)  # the old closure does not leak across a lead loss

  def test_nan_vrel_is_a_veto(self):
    # fail toward OFF: an unreadable closing rate never arms the gate
    g = GateOnly()
    outs = [g.step(lead(d_rel=1.5 * d_des(V), v_rel=float('nan')), e2e=-1.0, mpc=0.5) for _ in range(100)]
    self.assertFalse(g.e2e_gate)
    self.assertTrue(all(o == -1.0 for o in outs))
    # once armed by a real fast closure, a NaN frame also holds the latch (no countdown on ambiguous input)
    g = GateOnly()
    for _ in range(N_ENTER):
      g.step(lead(d_rel=2.0 * d_des(V), v_rel=-5.))
    before = g.e2e_gate_latch
    g.step(lead(d_rel=1.5 * d_des(V), v_rel=float('nan')))
    self.assertEqual(g.e2e_gate_latch, before)


class TestClosingRateVetoThroughPlanner(OpenpilotTestCase):
  def test_16b_cut_in_bit_exact_through_update(self):
    # the same profile through the real LongitudinalPlanner.update(): gate and gate-disabled planners agree bit-exactly
    # on every frame and the gate never turns ON, i.e. the veto cannot move the plan at all in the incident
    pg, pu = make_planner(), make_planner()
    pu.update_e2e_gate = lambda lead_, v, e2e, *a: e2e
    flags = []
    for t, v, d_rel, v_rel, a_lead, e2e, mpc, t_follow in ROUTE_16B_CUT_IN:
      sm = planner_sm(v, e2e, d_rel=d_rel, v_lead=v + v_rel, a_lead=a_lead)
      pg.update(sm)
      pu.update(sm)
      self.assertEqual(float(pg.output_a_target), float(pu.output_a_target), t)
      flags.append(pg.e2e_gate)
    self.assertFalse(any(flags))


class TestComfortCap(OpenpilotTestCase):
  """Cycle 3 (routes 16d/16f, 2026-10-26): the gate may stop the model braking for a far lead, but it may not hand the
  car the MPC's gap-chasing ask. Raising to min(mpc, E2E_GATE_MAX_RAISE) keeps the anti-lag behaviour (the 167/168
  lag signature: model +0.04, MPC +0.81) and removes the race-then-brake pattern (measured: gate-caused frames above
  0.8 m/s^2 drop to zero, accel > 0.3 m/s^2 over a drive 64.7 s -> 17.5 s with the gate off)."""

  def _gated(self, mpc, e2e=-0.9, n=N_ENTER):
    g = GateOnly()
    out = None
    for _ in range(n):
      out = g.step(FAR, e2e=e2e, mpc=mpc)
    self.assertTrue(g.e2e_gate)
    return out

  def test_the_cap_is_a_comfort_acceleration(self):
    self.assertAlmostEqual(lp.E2E_GATE_MAX_RAISE, 0.4)

  def test_large_mpc_is_capped(self):
    # a lead 1.5x the desired gap at matching speed, model on the brakes: the MPC's +1.2 does NOT reach the plan
    self.assertEqual(self._gated(mpc=1.2), 0.4)
    # mutation: with the cap lifted the same frames hand the car the full MPC ask, so the cap is what does the work
    with mock.patch.object(lp, 'E2E_GATE_MAX_RAISE', 99.):
      self.assertEqual(self._gated(mpc=1.2), 1.2)

  def test_small_mpc_passes_through_unchanged(self):
    # below the cap the gate is exactly what it was before cycle 3
    self.assertEqual(self._gated(mpc=0.2), 0.2)

  def test_raise_only_still_never_lowers_the_model(self):
    # a model wish ABOVE the cap must survive the gate untouched: the cap is a ceiling on what the gate adds
    self.assertEqual(self._gated(mpc=1.2, e2e=0.9), 0.9)

  def test_anti_lag_case_still_accelerates(self):
    # the case the gate was built for: the model lags (+0.04) while the MPC wants +0.81 -- the car must still get a
    # real, if comfortable, acceleration instead of the model's near-zero
    out = self._gated(mpc=0.81, e2e=0.04)
    self.assertEqual(out, lp.E2E_GATE_MAX_RAISE)
    self.assertGreater(out, 0.04)

  def test_cap_holds_inside_the_planner_update(self):
    # in situ through the real update(): every frame's e2e leaving the gate is the raw wish or max(raw, min(mpc, cap))
    p = make_planner()
    rec = []
    orig = lp.LongitudinalPlanner.update_e2e_gate

    def spy(self_, lead_, v_, e2e, mpc, ss, tf):
      out = orig(self_, lead_, v_, e2e, mpc, ss, tf)
      rec.append((e2e, mpc, out))
      return out
    with mock.patch.object(lp.LongitudinalPlanner, 'update_e2e_gate', spy):
      for i in range(120):
        p.update(planner_sm(15., -0.9, d_rel=2.0 * d_des(15.), v_lead=17.))
    self.assertEqual(len(rec), 120)
    self.assertTrue(any(out != e2e for e2e, _, out in rec))       # the gate did act
    for e2e, mpc, out in rec:
      self.assertGreaterEqual(out, e2e)                           # never lowers
      self.assertIn(out, (e2e, max(e2e, min(mpc, lp.E2E_GATE_MAX_RAISE))))
      self.assertLessEqual(out, max(e2e, lp.E2E_GATE_MAX_RAISE))  # and never above the cap unless the model asked
