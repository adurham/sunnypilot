"""
Tests for the fork's lead closing-speed filter (sunnypilot/fork/lead_closing.py) and for the NoCloseOnLead invariant
through the REAL LongitudinalPlanner.update path (MPC, cruise, e2e gate, AccelBoost, min(), cap).

Owner complaints (2026-11-04): #3 "accelerating towards a car for no reason ... must NEVER happen" and #4 "gas flutter".
Measured mechanism (lane D, routes 189-194): the cap read the raw per-frame vEgo - vLead of a vision-only lead, which
reads "matched / opening" while the gap shrinks at range, and jitters 0.4-0.7 m/s per frame across the cap's ramp.
"""
import numpy as np

from openpilot.cereal import log
import openpilot.cereal.messaging as messaging
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.modeld.constants import ModelConstants
import openpilot.selfdrive.controls.lib.longitudinal_planner as lp
from openpilot.sunnypilot.fork import no_close as nc

T = np.array(ModelConstants.T_IDXS)
DT = 0.05


class FakeParams:
  def __init__(self, enabled=True):
    self.enabled = enabled

  def get_bool(self, key):
    return self.enabled


def make_planner(no_close_on=True, v0=17.):
  from opendbc.car.honda.interface import CarInterface
  from opendbc.car.honda.values import CAR
  CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
  CP.openpilotLongitudinalControl = True
  CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.HONDA_CIVIC)
  p = lp.LongitudinalPlanner(CP, CP_SP, init_v=v0)
  p.no_close = nc.NoCloseCap(params=FakeParams(no_close_on))
  return p


def planner_sm(v_ego, desired_accel, d_rel, v_rel, v_cruise_kph=110., lead_present=True):
  N = len(T)
  msgs = {k: messaging.new_message(k) for k in ('modelV2', 'carState', 'carControl', 'controlsState', 'selfdriveState',
                                                'radarState', 'liveMapDataSP', 'vehicleParameters', 'carStateSP',
                                                'gpsLocation', 'gpsLocationExternal', 'selfdriveStateSP')}
  md = msgs['modelV2'].modelV2
  md.position.x = [float(v_ego * t) for t in T]
  md.position.t = [float(t) for t in T]
  md.velocity.x = [float(v_ego)] * N
  md.orientationRate.z = [0.] * N
  md.action.desiredAcceleration = float(desired_accel)
  md.meta.disengagePredictions.gasPressProbs = [0.] * 6
  md.meta.disengagePredictions.brakePressProbs = [0.] * 6
  cs = msgs['carState'].carState
  cs.vEgo = float(v_ego)
  cs.vCruise = float(v_cruise_kph)
  cs.vCruiseCluster = float(v_cruise_kph)
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
    l1.dRel = float(d_rel)
    l1.vRel = float(v_rel)
    l1.vLead = float(v_ego + v_rel)
    l1.vLeadK = float(v_ego + v_rel)
    l1.aLeadTau = 0.3
    l1.modelProb = 1.0
  return {k: getattr(m, k) for k, m in msgs.items()}


def drive(planner, n, v_ego, d0, gap_rate, vrel_fn, desired=0.3, v_cruise_kph=110.):
  """Open loop: ego speed held, the gap changes at gap_rate (m/s, < 0 = closing), radar vRel from vrel_fn(i)."""
  out = []
  for i in range(n):
    sm = planner_sm(v_ego, desired, d0 + gap_rate * i * DT, vrel_fn(i), v_cruise_kph)
    planner.update(sm)
    out.append(float(planner.output_a_target))
  return np.array(out)


class TestLeadClosingFilter(OpenpilotTestCase):
  def test_first_frame_is_the_raw_closing(self):
    from openpilot.sunnypilot.fork.lead_closing import LeadClosingFilter
    f = LeadClosingFilter()
    self.assertAlmostEqual(f.update(True, 60., -1.3), 1.3)
    self.assertTrue(f.valid)

  def test_gap_shrinking_wins_over_a_biased_vrel(self):
    """The 193/194 picture: the gap shrinks 1.2 m/s at 60 m while the vision vRel reads +0.3 (opening) with 0.7 m/s
    of frame noise. Within 3 s the estimate must say 'closing' and keep saying it."""
    from openpilot.sunnypilot.fork.lead_closing import LeadClosingFilter
    rng = np.random.default_rng(0)
    f = LeadClosingFilter()
    est = [f.update(True, 60. - 1.2 * i * DT + rng.normal(0, 1.0), 0.3 + rng.normal(0, 0.7)) for i in range(200)]
    self.assertGreater(min(est[60:]), 0.4)
    self.assertLess(np.std(np.diff(est[60:])), 0.1)   # raw vRel jitters 0.7*sqrt(2) ~ 1.0 frame to frame

  def test_jump_to_another_object_is_gated_then_reinitialised(self):
    from openpilot.sunnypilot.fork.lead_closing import LeadClosingFilter, REINIT_FRAMES
    f = LeadClosingFilter()
    for i in range(40):
      f.update(True, 50. - 0.5 * i * DT, -0.5)
    before = f.closing
    f.update(True, 75., 0.)                       # a different car 25 m further: not fused
    self.assertAlmostEqual(f.closing, before, delta=0.05)
    for _ in range(REINIT_FRAMES):
      f.update(True, 75., 0.)
    self.assertAlmostEqual(f.x[0], 75., delta=0.5)  # consistently there: it is the new lead

  def test_short_dropout_is_coasted_long_one_resets(self):
    from openpilot.sunnypilot.fork.lead_closing import LeadClosingFilter, HOLD_FRAMES
    f = LeadClosingFilter()
    for i in range(60):
      f.update(True, 50. - 1.0 * i * DT, -1.0)
    for _ in range(HOLD_FRAMES):
      f.update(False, None, None)
      self.assertTrue(f.valid and f.coasting)
      self.assertGreater(f.closing, 0.5)
    f.update(False, None, None)
    self.assertFalse(f.valid)


class TestNoCloseInvariantPlanner(OpenpilotTestCase):
  """Through LongitudinalPlanner.update. These FAIL on the deployed build (cap keyed on raw per-frame vEgo - vLead with
  a 0.25/0.6 m/s band)."""

  def test_no_throttle_while_the_gap_shrinks_and_vrel_reads_matched(self):
    """Complaint 3 (194 t=329-336, 193 t=297-302): cruising below set speed, a lead far ahead, the gap shrinking ~1 m/s
    while the vision vRel reads 0..+0.3. The MPC chases the gap; the published ask must not stay positive."""
    for v_ego, d0 in ((12., 45.), (17., 80.), (25., 95.)):
      with self.subTest(v_ego=v_ego, d0=d0):
        p = make_planner()
        out = drive(p, 120, v_ego, d0, -1.0, lambda i: 0.3 if i % 2 else 0.0)
        self.assertLessEqual(out[60:].max(), 1e-6, f'positive ask while closing: max {out[60:].max():.3f}')

  def test_matched_lead_gets_no_throttle_at_speed(self):
    """'Never faster than the lead': with the gap constant and the lead matched, a gap-chasing ask is not published."""
    p = make_planner()
    out = drive(p, 80, 17., 70., 0., lambda i: 0.)
    self.assertLessEqual(out[20:].max(), 1e-6)

  def test_pulling_away_lead_is_followed_without_flutter(self):
    """Complaint 4 (194 t=429-431): a lead pulling away at 1.0 m/s, vision vRel noise 0.7 m/s. The old cap zeroed the
    ask on every noise frame that read 'closing' (factor 0 for the 0.5 s dwell); the published ask must stay positive
    and never hit the cap's exact zero."""
    rng = np.random.default_rng(1)
    noise = rng.normal(0., 0.7, 400)
    out = drive(make_planner(), 400, 17., 60., +1.0, lambda i: 1.0 + noise[i], desired=0.5)
    ref = drive(make_planner(no_close_on=False), 400, 17., 60., +1.0, lambda i: 1.0 + noise[i], desired=0.5)
    pos = np.flatnonzero(ref[40:] > 0.05) + 40
    self.assertGreater(len(pos), 300)   # the uncapped plan really does ask for throttle here (premise check)
    zeros = int(np.sum(out[pos] < 1e-9))
    self.assertEqual(zeros, 0, f'{zeros} zeroed frames while the lead pulled away')
    self.assertGreater(float(np.mean(out[pos] / ref[pos])), 0.8)

  def test_launch_band_unchanged_at_crawl(self):
    """Below V_LOW the pre-2026-11-04 band applies (the launch decision belongs to the standstill lane): a lead that
    is barely closing (0.1 m/s) does not cap a launch."""
    self.assertEqual(nc.no_close_factor(0.1, 1.0), 1.)
    self.assertEqual(nc.no_close_factor(0.6, 1.0), 0.)
    self.assertEqual(nc.band(nc.V_STRICT), (nc.ENTER, nc.EXIT))
    self.assertEqual(nc.band(0.), (nc.ENTER_LOW, nc.EXIT_LOW))

  def test_brake_requests_never_touched(self):
    p = make_planner()
    out = drive(p, 80, 20., 25., -4.0, lambda i: -4.0, desired=-1.5)
    q = make_planner(no_close_on=False)
    ref = drive(q, 80, 20., 25., -4.0, lambda i: -4.0, desired=-1.5)
    neg = ref <= 0.
    np.testing.assert_array_equal(out[neg], ref[neg])
