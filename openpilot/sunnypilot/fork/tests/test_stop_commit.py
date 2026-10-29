"""
Fork (17c): unit/behaviour tests for the no-lead stop-approach commitment (sunnypilot/fork/stop_commit.py).
"""
import numpy as np

from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.fork import stop_commit as sc

T = np.array(ModelConstants.T_IDXS)


class FakeParams:
  def __init__(self, enabled=True, unknown=False):
    self.enabled = enabled
    self.unknown = unknown

  def get_bool(self, key):
    assert key == sc.PARAM
    if self.unknown:
      raise KeyError(key)  # stands in for UnknownKeyName (stale libparams)
    return self.enabled


def stop_plan(v0, x_stop):
  """Model-like plan: uniform decel from v0 to 0 at x_stop, then standing."""
  a = v0 * v0 / (2. * x_stop)
  v = np.clip(v0 - a * T, 0., None)
  x = np.where(T < v0 / a, v0 * T - 0.5 * a * T ** 2, x_stop)
  return x, v, a


def cruise_plan(v0, accel=0.):
  v = np.clip(v0 + accel * T, 0., None)
  return v0 * T + 0.5 * accel * T ** 2, v


def run(c, n, e2e, v_ego, lead=False, gas=False, active=True, *, plan):
  out = []
  for _ in range(n):
    out.append(c.update(e2e, active, gas, v_ego, lead, plan[0], plan[1], T))
  return out


class TestCommitAccel(OpenpilotTestCase):
  def test_uniform_stop_plan_reads_its_own_decel(self):
    for v0, xs in ((17., 100.), (12., 40.), (20., 150.)):
      x, v, a = stop_plan(v0, xs)
      self.assertAlmostEqual(sc.commit_accel(x, v, T), -a, delta=0.02 * a)

  def test_steady_and_accelerating_plans_are_zero(self):
    self.assertEqual(sc.commit_accel(*cruise_plan(20.), T), 0.)
    self.assertEqual(sc.commit_accel(*cruise_plan(5., 1.0), T), 0.)

  def test_standstill_and_malformed_plans_are_zero(self):
    self.assertEqual(sc.commit_accel(np.zeros(33), np.zeros(33), T), 0.)
    self.assertEqual(sc.commit_accel([], [], []), 0.)
    x, v, _ = stop_plan(15., 60.)
    v[5] = np.nan
    self.assertEqual(sc.commit_accel(x, v, T), 0.)


class TestStopWeight(OpenpilotTestCase):
  def test_planned_stop_has_full_weight(self):
    self.assertEqual(sc.stop_weight(stop_plan(17., 100.)[1]), 1.)

  def test_curve_slowdown_is_not_a_stop(self):
    # 15 -> 6 m/s (ratio 0.4, an absolute 6 m/s floor): a turn, not a stop -> no weight at all
    v = np.clip(15. - 1.5 * T, 6., None)
    self.assertEqual(sc.stop_weight(v), 0.)

  def test_cruise_has_no_weight(self):
    self.assertEqual(sc.stop_weight(cruise_plan(20.)[1]), 0.)


class TestStopCommit(OpenpilotTestCase):
  def setUp(self):
    self.c = sc.StopCommit(params=FakeParams(True))

  def test_disabled_is_identity_on_every_input(self):
    c = sc.StopCommit(params=FakeParams(False))
    plan = stop_plan(17., 100.)
    rng = np.random.default_rng(0)
    for _ in range(400):
      e2e = float(rng.uniform(-3., 2.))
      self.assertEqual(c.update(e2e, True, False, 17., False, plan[0], plan[1], T), e2e)
    self.assertEqual(c.delta, 0.)

  def test_unknown_key_stays_off(self):
    c = sc.StopCommit(params=FakeParams(True, unknown=True))
    plan = stop_plan(17., 100.)
    self.assertEqual(run(c, 60, -0.4, 17., plan=plan)[-1], -0.4)

  def test_no_lead_stop_commits_to_the_plans_own_decel(self):
    # 17c window (a) shape: plan stops at ~100 m from 17 m/s (-1.45 m/s^2), instantaneous ask -0.4
    plan = stop_plan(17., 100.)
    out = run(self.c, 60, -0.4, 17., plan=plan)
    self.assertAlmostEqual(out[-1], -plan[2], delta=0.03)

  def test_deepening_is_rate_limited_never_a_step(self):
    plan = stop_plan(17., 100.)
    out = np.array([-0.4] + run(self.c, 60, -0.4, 17., plan=plan))
    self.assertLessEqual(np.max(-np.diff(out)), sc.J_IN * self.c.dt + 1e-9)

  def test_never_raises_the_ask(self):
    rng = np.random.default_rng(1)
    for _ in range(300):
      v0 = float(rng.uniform(5., 25.))
      plan = stop_plan(v0, float(rng.uniform(20., 200.))) if rng.random() < 0.5 else cruise_plan(v0, float(rng.uniform(-1., 1.)))
      e2e = float(rng.uniform(-3., 1.5))
      lead = bool(rng.random() < 0.3)
      self.assertLessEqual(self.c.update(e2e, True, False, v0, lead, plan[0], plan[1], T), e2e + 1e-12)

  def test_lead_on_the_approach_blocks_arming(self):
    # occupied stop: the lead candidate owns the approach -> byte-identical e2e
    plan = stop_plan(17., 100.)
    self.assertEqual(run(self.c, 100, -0.4, 17., lead=True, plan=plan), [-0.4] * 100)

  def test_flickering_lead_cannot_arm(self):
    plan = stop_plan(17., 100.)
    for _ in range(10):
      run(self.c, 1, -0.4, 17., lead=True, plan=plan)
      out = run(self.c, sc.LEAD_CLEAR_FRAMES - 1, -0.4, 17., plan=plan)
      self.assertEqual(out, [-0.4] * (sc.LEAD_CLEAR_FRAMES - 1))

  def test_lead_appearing_mid_stop_does_not_release(self):
    plan = stop_plan(17., 100.)
    run(self.c, 60, -0.4, 17., plan=plan)
    d0 = self.c.delta
    run(self.c, 10, -0.4, 17., lead=True, plan=plan)
    self.assertLessEqual(self.c.delta, d0 + 1e-9)

  def test_gas_and_long_off_reset_at_once(self):
    plan = stop_plan(17., 100.)
    run(self.c, 60, -0.4, 17., plan=plan)
    self.assertEqual(run(self.c, 1, -0.4, 17., gas=True, plan=plan), [-0.4])
    run(self.c, 60, -0.4, 17., plan=plan)
    self.assertEqual(run(self.c, 1, -0.4, 17., active=False, plan=plan), [-0.4])

  def test_launch_cruise_and_low_speed_never_arm(self):
    self.assertEqual(run(self.c, 100, 1.0, 3., plan=cruise_plan(3., 1.2)), [1.0] * 100)
    self.assertEqual(run(self.c, 100, 0.0, 25., plan=cruise_plan(25.)), [0.0] * 100)
    plan = stop_plan(4., 10.)
    self.assertEqual(run(self.c, 100, -0.2, 4., plan=plan), [-0.2] * 100)

  def test_curve_slowdown_never_arms(self):
    v = np.clip(15. - 1.5 * T, 6., None)
    x = np.concatenate([[0.], np.cumsum(np.diff(T) * (v[1:] + v[:-1]) / 2.)])
    self.assertEqual(run(self.c, 100, -0.2, 15., plan=(x, v)), [-0.2] * 100)

  def test_small_gap_does_not_arm(self):
    # instantaneous ask already within GAP_ENTER of the plan's own decel -> nothing to commit
    plan = stop_plan(17., 100.)
    e2e = -plan[2] + 0.2
    self.assertEqual(run(self.c, 100, e2e, 17., plan=plan), [e2e] * 100)

  def test_release_ramps_when_the_plan_stops_planning_a_stop(self):
    plan = stop_plan(17., 100.)
    run(self.c, 60, -0.4, 17., plan=plan)
    self.assertLess(self.c.delta, -0.5)
    out = np.array(run(self.c, 40, -0.4, 17., plan=cruise_plan(17.)))
    self.assertFalse(self.c.armed)
    self.assertLessEqual(np.max(np.diff(out)), sc.J_OUT * self.c.dt + 1e-9)
    self.assertAlmostEqual(out[-1], -0.4, places=6)

  def test_disarmed_once_the_model_catches_up(self):
    plan = stop_plan(17., 100.)
    run(self.c, 60, -0.4, 17., plan=plan)
    out = run(self.c, 60, -plan[2], 17., plan=plan)  # model's own ask now equals its plan
    self.assertFalse(self.c.armed)
    self.assertAlmostEqual(out[-1], -plan[2], places=6)


class TestPinnedConstants(OpenpilotTestCase):
  """The comfort / debounce constants define the feel and the lead gate. Every other test in this file reads them
  symbolically (``sc.J_IN``, ``sc.LEAD_CLEAR_FRAMES``, ...), so a mutated VALUE survives the whole suite (review #3:
  J_IN 1.0->5.0, J_OUT 1.5->50, LEAD_CLEAR_FRAMES 20->1, R_NONE 0.5->1.0 and T_MIN 1->0 all SURVIVED). Each test
  below pins the literal AND the behaviour that literal produces, with the numbers written out rather than read
  back from the module."""

  def setUp(self):
    self.c = sc.StopCommit(params=FakeParams(True))

  def test_the_five_constants_are_pinned(self):
    self.assertEqual(sc.J_IN, 1.0)                  # m/s^3 deepening: the single knob, inside the owner's 1.4 reference
    self.assertEqual(sc.J_OUT, 1.4)                 # m/s^3 release: the owner's documented 1.4 reference (FORK row 91)
    self.assertEqual(sc.LEAD_CLEAR_FRAMES, 20)      # 1.0 s at 20 Hz with no lead before arming
    self.assertEqual(sc.R_NONE, 0.5)                # plan v_min / v_0 >= 0.5 is not a stop
    self.assertEqual(sc.T_MIN, 1.0)                 # s, plan points nearer than this are the live ask, not a plan

  def test_deepening_rate_is_1_0_m_s3(self):
    # behaviour, not the symbol: each 20 Hz frame deepens delta by exactly J_IN*dt = 1.0*0.05 = 0.05 m/s^2
    plan = stop_plan(17., 100.)
    run(self.c, 25, -0.4, 17., plan=plan)   # 20 frames to arm + 5 ramping frames; target is -1.045, so still ramping
    self.assertTrue(self.c.armed)
    self.assertLess(self.c.delta, 0.)
    steps = []
    for _ in range(10):
      before = self.c.delta
      self.c.update(-0.4, True, False, 17., False, plan[0], plan[1], T)
      steps.append(before - self.c.delta)
    self.assertAlmostEqual(float(np.max(steps)), 0.05, places=9)   # 1.0 m/s^3 * 0.05 s
    self.assertAlmostEqual(float(np.min(steps)), 0.05, places=9)   # still ramping: every frame is the full step
    self.assertLessEqual(sc.J_IN, 1.4)                              # the owner's comfort reference is the ceiling

  def test_release_rate_is_the_1_4_reference(self):
    # behaviour: after the plan stops planning a stop, delta returns at J_OUT*dt = 1.4*0.05 = 0.07 m/s^2 per frame
    plan = stop_plan(17., 100.)
    run(self.c, 60, -0.4, 17., plan=plan)
    self.assertLess(self.c.delta, -0.5)
    out = [-0.4] + run(self.c, 40, -0.4, 17., plan=cruise_plan(17.))
    steps = np.diff(out)
    self.assertAlmostEqual(float(np.max(steps)), 0.07, places=9)   # 1.4 m/s^3 * 0.05 s
    self.assertGreater(float(np.max(steps)), 0.0)

  def test_a_lead_must_be_clear_for_exactly_twenty_frames(self):
    # 19 clear frames may NOT arm (the old 20->1 mutation would have armed on the first one); the 20th does
    plan = stop_plan(17., 100.)
    c = sc.StopCommit(params=FakeParams(True))
    for _ in range(19):
      self.assertEqual(c.update(-0.4, True, False, 17., False, plan[0], plan[1], T), -0.4)
    self.assertFalse(c.armed)
    c.update(-0.4, True, False, 17., False, plan[0], plan[1], T)
    self.assertTrue(c.armed)

  def test_r_none_is_half_the_current_speed(self):
    # the relative curve ends at exactly half speed: 10 -> 5 m/s is "no stop" (weight 0), 10 -> 4.5 is not
    v = np.full(len(T), 5.0)
    v[0] = 10.0
    self.assertEqual(sc.stop_weight(v), 0.)
    v = np.full(len(T), 4.5)
    v[0] = 10.0
    # ratio 0.45 -> (0.5-0.45)/0.25 = 0.2 ; v_min 4.5 -> (6-4.5)/3 = 0.5 ; product 0.1
    self.assertAlmostEqual(sc.stop_weight(v), 0.1, places=12)

  def test_t_min_ignores_the_first_second_of_the_plan(self):
    # a plan that sheds speed hard inside the first second and then holds: those early points are the live ask,
    # not a plan, so the commit must be the post-1 s value only (with T_MIN = 0 it reads ~-28 m/s^2 instead)
    t = T
    v = np.where(t < 0.5, 20. - 28. * t, 6.)
    x = np.concatenate([[0.], np.cumsum(np.diff(t) * (v[1:] + v[:-1]) / 2.)])
    keep = t >= 1.0
    expected = float(((v[keep] ** 2 - v[0] ** 2) / (2. * x[keep])).min())
    self.assertAlmostEqual(sc.commit_accel(x, v, t), expected, places=9)
    self.assertGreater(sc.commit_accel(x, v, t), -20.)   # the early-second value is about -28

