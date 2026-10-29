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
