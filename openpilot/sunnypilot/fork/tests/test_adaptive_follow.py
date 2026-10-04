"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.cereal import log
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import get_T_FOLLOW
from openpilot.sunnypilot.fork import adaptive_follow as af

STANDARD = log.LongitudinalPersonality.standard
PERSONALITIES = (log.LongitudinalPersonality.relaxed, STANDARD, log.LongitudinalPersonality.aggressive)


class FakeParams:
  def __init__(self, enabled=True, unknown=False):
    self.enabled = enabled
    self.unknown = unknown

  def get_bool(self, key):
    assert key == af.PARAM
    if self.unknown:
      raise KeyError(key)  # stands in for UnknownKeyName (stale libparams)
    return self.enabled


class TestRoadSpeedFactor(OpenpilotTestCase):
  def test_urban_never_changes_the_gap(self):
    # a brakeless car must not follow closer in town: urban is exactly the personality gap at every speed
    for v in (0., 10., 15., 20., 25., 30., 40.):
      self.assertEqual(af.road_speed_factor("urban", v), 1.0)

  def test_highway_ramps_up_with_speed(self):
    for rt in af.HIGHWAY_ROAD_TYPES:
      self.assertEqual(af.road_speed_factor(rt, 15.), 1.0)
      self.assertAlmostEqual(af.road_speed_factor(rt, 29.), 1.1)
      self.assertAlmostEqual(af.road_speed_factor(rt, 35.), 1.1)
      vs = [15. + i for i in range(20)]
      fs = [af.road_speed_factor(rt, v) for v in vs]
      self.assertEqual(fs, sorted(fs))

  def test_highway_factor_drive_12f(self):
    # drive 12f: 1.2 gave 1.97-2.02 s to a car at 29 m/s ("a bit far"); 1.1 -> T 1.60 s, d = T*v + 6 m = 52.3 m = 1.80 s
    self.assertEqual(af.HIGHWAY_FACTOR_V, [1.0, 1.1])
    t = af.get_adaptive_t_follow(STANDARD, 29., "interstate", True, 29., True)
    self.assertAlmostEqual(t, 1.1 * get_T_FOLLOW(STANDARD))
    self.assertAlmostEqual((t * 29. + 6.) / 29., 1.80, places=2)
    # never below stock on the highway (no-brake car keeps margin over stock 1.45 s)
    for v in (15., 20., 25., 29., 35.):
      self.assertGreaterEqual(af.road_speed_factor("highway", v), 1.0)

  def test_unknown_is_between(self):
    self.assertAlmostEqual(af.road_speed_factor("unknown", 30.), 1.1)
    self.assertEqual(af.road_speed_factor("unknown", 15.), 1.0)


class TestClosingMargin(OpenpilotTestCase):
  def test_no_margin_when_not_closing(self):
    self.assertEqual(af.closing_margin(30., 30.), 0.)
    self.assertEqual(af.closing_margin(30., 32.), 0.)
    self.assertEqual(af.closing_margin(0.5, 0.), 0.)

  def test_margin_matches_coast_physics(self):
    # 30 m/s closing on a 28 m/s lead: coasting at 0.42 vs braking at 2.5 needs vrel^2/2*(1/0.42-1/2.5) extra metres
    v, vl = 30., 28.
    expected = (2. ** 2 / 2. * (1. / 0.42 - 1. / 2.5)) / v
    self.assertAlmostEqual(af.closing_margin(v, vl), expected, places=6)

  def test_margin_is_capped(self):
    self.assertEqual(af.closing_margin(30., 20.), af.MAX_CLOSING_MARGIN)

  def test_margin_grows_with_closing_speed(self):
    ms = [af.closing_margin(30., 30. - dv) for dv in (0.5, 1., 1.5, 2., 3.)]
    self.assertEqual(ms, sorted(ms))
    self.assertGreater(ms[-1], ms[0])


class TestAdaptiveTFollow(OpenpilotTestCase):
  def test_personality_is_the_base(self):
    for p in PERSONALITIES:
      with self.subTest(personality=p):
        self.assertAlmostEqual(af.get_adaptive_t_follow(p, 15., "urban", False, 0., True), get_T_FOLLOW(p))
        self.assertAlmostEqual(af.get_adaptive_t_follow(p, 31., "highway", False, 0., True), 1.1 * get_T_FOLLOW(p))

  def test_closing_margin_only_for_throttle_only_cars_with_a_lead(self):
    with_brakes = af.get_adaptive_t_follow(STANDARD, 30., "highway", True, 28., throttle_only=False)
    no_lead = af.get_adaptive_t_follow(STANDARD, 30., "highway", False, 28., throttle_only=True)
    pedal = af.get_adaptive_t_follow(STANDARD, 30., "highway", True, 28., throttle_only=True)
    self.assertAlmostEqual(with_brakes, no_lead)
    self.assertGreater(pedal, with_brakes)


class TestAdaptiveFollowController(OpenpilotTestCase):
  def test_disabled_is_inert(self):
    a = af.AdaptiveFollow(True, params=FakeParams(enabled=False))
    for _ in range(50):
      self.assertIsNone(a.update(STANDARD, 31., "highway", True, 28.))

  def test_stale_libparams_is_inert(self):
    a = af.AdaptiveFollow(True, params=FakeParams(unknown=True))
    self.assertIsNone(a.update(STANDARD, 31., "highway", True, 28.))

  def test_param_is_reread(self):
    p = FakeParams(enabled=False)
    a = af.AdaptiveFollow(True, params=p)
    self.assertIsNone(a.update(STANDARD, 31., "highway", False, 0.))
    p.enabled = True
    out = [a.update(STANDARD, 31., "highway", False, 0.) for _ in range(int(1. / a.dt) + 1)]
    self.assertAlmostEqual(out[-1], 1.1 * get_T_FOLLOW(STANDARD))

  def test_rate_limits(self):
    a = af.AdaptiveFollow(True, params=FakeParams())
    t0 = a.update(STANDARD, 15., "urban", False, 0.)
    self.assertAlmostEqual(t0, get_T_FOLLOW(STANDARD))
    # jump to highway speed: rises at most T_FOLLOW_RATE_UP per second
    t1 = a.update(STANDARD, 31., "highway", True, 28.)
    self.assertAlmostEqual(t1 - t0, af.T_FOLLOW_RATE_UP * a.dt, places=6)
    for _ in range(100):
      t_hi = a.update(STANDARD, 31., "highway", True, 28.)
    self.assertAlmostEqual(t_hi, af.get_adaptive_t_follow(STANDARD, 31., "highway", True, 28., True), places=6)
    # back to urban: margin is given back slowly
    t2 = a.update(STANDARD, 15., "urban", False, 0.)
    self.assertAlmostEqual(t_hi - t2, af.T_FOLLOW_RATE_DOWN * a.dt, places=6)

  def test_throttle_only_detection(self):
    class P:
      def __init__(self, **kw):
        self.__dict__.update(kw)
    self.assertTrue(af.is_throttle_only(P(brand="hyundai"), P(enableGasInterceptor=True)))
    self.assertFalse(af.is_throttle_only(P(brand="hyundai"), P(enableGasInterceptor=False)))
    self.assertFalse(af.is_throttle_only(P(brand="toyota"), P(enableGasInterceptor=True)))


class TestPlannerFollowsOverride(OpenpilotTestCase):
  """End to end through the real LongitudinalPlanner + acados MPC (openpilot's longitudinal_maneuvers Plant).

  Asserts the wiring (the time gap the MPC was actually given) and the steady-state gap change vs stock.
  """

  @staticmethod
  def _run(adaptive, v_lead=30., t_end=120.):
    from openpilot.selfdrive.test.longitudinal_maneuvers.plant import Plant
    plant = Plant(lead_relevancy=True, speed=v_lead, distance_lead=100., personality=STANDARD)
    # Plant's non-essential Honda CP has openpilotLongitudinalControl False and Plant never sets selfdriveState.enabled,
    # so the planner would reset to carState every step (why upstream's test_following_distance fails 15/18 here).
    plant.planner.CP.openpilotLongitudinalControl = True
    # the Plant builds a Honda planner (has brakes) -> no closing margin; its liveMapDataSP is empty -> roadType
    # unknown -> factor 1.1 at >= 29 m/s
    plant.planner.adaptive_follow = af.AdaptiveFollow(False, params=FakeParams(enabled=adaptive))
    while plant.current_time < t_end:
      out = plant.step(v_lead=v_lead, prob_lead=1.0, v_cruise=40.)
    return plant, out['distance_lead'] - out['distance']

  def test_disabled_passes_stock_t_follow_to_mpc(self):
    plant, _ = self._run(adaptive=False, t_end=2.)
    self.assertTrue((plant.planner.mpc.params[:, 4] == get_T_FOLLOW(STANDARD)).all())

  def test_enabled_passes_adaptive_t_follow_to_mpc(self):
    plant, _ = self._run(adaptive=True, t_end=2.)
    self.assertAlmostEqual(float(plant.planner.mpc.params[0, 4]), 1.1 * get_T_FOLLOW(STANDARD), places=5)

  def test_enabled_lengthens_steady_state_gap(self):
    # Relative check only: in this Plant both stock and adaptive settle ~9 m beyond T*v + 6 at 30 m/s on this host
    # (upstream's absolute test_following_distance.py fails 15/18 on an unmodified tree here), so the oracle is the
    # stock run itself. Expected change: +10 % of T*v = 4.35 m.
    _, stock = self._run(adaptive=False)
    _, adaptive_gap = self._run(adaptive=True)
    expected = 0.1 * get_T_FOLLOW(STANDARD) * 30.
    self.assertGreater(adaptive_gap - stock, 0.5 * expected)
    self.assertLess(adaptive_gap - stock, 1.5 * expected)
