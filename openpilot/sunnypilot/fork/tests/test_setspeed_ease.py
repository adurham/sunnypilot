"""
Tests for fork/setspeed_ease.py (personality-dependent set-speed easing). Every rule in the module docstring has a test
here
car-features/setspeed-ease-mutation.py mutates each rule and checks this file fails.
"""
import inspect
from unittest import mock

import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib import longitudinal_planner as upstream_lp
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.fork import setspeed_ease as se
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP

P = log.LongitudinalPersonality
PERSONALITIES = (P.relaxed, P.standard, P.aggressive)
MPH = 0.44704


def capnp_personality(p):
  """selfdriveState.personality as the planner really sees it (a capnp _DynamicEnum, not an int)."""
  sd = messaging.new_message('selfdriveState').selfdriveState
  sd.personality = p
  return sd.personality


def run_steps(e, n, v_target, v_ego, personality=P.standard, active=True, a_plan=0.5):
  out = []
  for _ in range(n):
    out.append(e.step(v_target, v_ego, personality, active, a_plan))
  return out


def engaged(e, v_ego, personality=P.standard):
  """an engaged, settled ease at v_ego (target == v_ego)"""
  e.step(v_ego, v_ego, personality, True, 0.)
  return e


class TestRate(OpenpilotTestCase):
  def test_personality_order_at_every_speed(self):
    for v in np.arange(0., 45., 0.5):
      r, s, a = (se.get_rate(p, v) for p in PERSONALITIES)
      self.assertLess(r, s, v)
      self.assertLess(s, a, v)

  def test_relaxed_and_standard_well_under_upstream_ceiling(self):
    # the owner's complaint: 1.0-1.2 m/s^2 flat. relaxed / standard must ask well under the upstream ACC ceiling
    for v in np.arange(8., 40., 1.):
      ceiling = float(upstream_lp.get_max_accel(v))
      self.assertLessEqual(se.get_rate(P.standard, v) * se.LEASH_S, 1.15 * ceiling + 1e-6, v)
      self.assertLess(se.get_rate(P.relaxed, v), 0.6 * ceiling, v)
      self.assertLess(se.get_rate(P.standard, v), 0.8 * ceiling, v)

  def test_capnp_enum_personality_is_honoured(self):
    # regression: a capnp _DynamicEnum == int but does not hash like it; a raw dict lookup silently fell back to standard
    for p in PERSONALITIES:
      self.assertEqual(se.get_rate(capnp_personality(p), 20.), se.get_rate(p, 20.))
    self.assertNotEqual(se.get_rate(capnp_personality(P.relaxed), 20.), se.get_rate(P.standard, 20.))

  def test_unknown_personality_is_standard(self):
    self.assertEqual(se.get_rate(7, 20.), se.get_rate(P.standard, 20.))

  def test_uncastable_personality_is_standard(self):
    # a renamed / non-numeric enum must not raise inside the planner: it falls back to standard
    for bad in ('sport', None, object()):
      self.assertEqual(se.personality_id(bad), int(P.standard))
      self.assertEqual(se.get_rate(bad, 20.), se.get_rate(P.standard, 20.))


class TestStep(OpenpilotTestCase):
  def test_ramps_at_rate(self):
    for p in PERSONALITIES:
      e = engaged(se.SetSpeedEase(), 20., p)
      v0 = e.v_ease
      e.step(25., 20., p, True, 0.5)
      self.assertAlmostEqual(e.v_ease - v0, se.get_rate(p, 20.) * DT_MDL, places=6)

  def test_plus_10_mph_is_a_ramp_not_a_step(self):
    e = engaged(se.SetSpeedEase(), 20.)
    v = e.step(20. + 10 * MPH, 20., P.standard, True, 0.)
    self.assertLess(v - 20., 0.1)

  def test_leash(self):
    # car stuck (hill / pedal cap): v_ease stops at v_ego + rate * LEASH_S
    for p in PERSONALITIES:
      e = engaged(se.SetSpeedEase(), 20., p)
      vs = run_steps(e, 400, 35., 20., p)
      self.assertAlmostEqual(vs[-1], 20. + se.get_rate(p, 20.) * se.LEASH_S, places=6)
      self.assertTrue(all(v <= 20. + se.get_rate(p, 20.) * se.LEASH_S + 1e-9 for v in vs))

  def test_leash_after_constraint_released(self):
    # held below the target (lead / curve), then released: the error handed to the planner is bounded, not the full gap
    e = engaged(se.SetSpeedEase(), 18.)
    run_steps(e, 200, 30., 18.)
    self.assertLessEqual(e.v_ease - 18., se.get_rate(P.standard, 18.) * se.LEASH_S + 1e-9)

  def test_down_is_immediate(self):
    e = engaged(se.SetSpeedEase(), 20.)
    run_steps(e, 40, 30., 21.)
    v = e.step(19., 21., P.standard, True, 0.5)
    self.assertEqual(v, 19.)
    v = e.step(0., 21., P.standard, True, 0.5)  # forceDecel / curve target to 0
    self.assertEqual(v, 0.)

  def test_never_above_target(self):
    e = engaged(se.SetSpeedEase(), 20.)
    vs = run_steps(e, 500, 21., 20.9)
    self.assertTrue(all(v <= 21. for v in vs))
    self.assertEqual(vs[-1], 21.)

  def test_never_below_current_speed(self):
    # the car got faster than v_ease (downhill / driver nudge): v_ease follows, it never asks to slow down below target
    e = engaged(se.SetSpeedEase(), 20.)
    v = e.step(30., 24., P.standard, True, 0.)
    self.assertGreaterEqual(v, 24.)
    # car above the target: v_ease = target (cruise decel exactly as upstream)
    v = e.step(22., 24., P.standard, True, 0.)
    self.assertEqual(v, 22.)

  def test_reset_when_inactive(self):
    e = engaged(se.SetSpeedEase(), 20.)
    run_steps(e, 30, 30., 20.)
    self.assertGreater(e.v_ease, 20.)
    v = e.step(30., 15., P.standard, False, 0.)
    self.assertEqual(v, 30.)            # inactive: pass-through (upstream behaviour)
    self.assertEqual(e.v_ease, 15.)     # ... with the ramp re-armed at the current speed
    # re-engage at a stored higher set speed while moving: ramps from the current speed
    v = e.step(30., 15., P.standard, True, 0.)
    self.assertLess(v - 15., 0.1)

  def test_first_update_starts_at_current_speed(self):
    e = se.SetSpeedEase()
    self.assertEqual(e.step(30., 20., P.standard, True, 0.), 20.)


class TestLaunch(OpenpilotTestCase):
  def test_launch_from_stop_not_eased(self):
    e = se.SetSpeedEase()
    self.assertEqual(e.step(20., 0., P.relaxed, True, 1.5), 20.)
    # still launching while accelerating above LAUNCH_V
    for v in np.arange(0.5, 15., 0.2):
      self.assertEqual(e.step(20., float(v), P.relaxed, True, 1.0), 20.)
    self.assertTrue(e.launching)

  def test_launch_ends_near_target(self):
    e = se.SetSpeedEase()
    e.step(20., 0., P.relaxed, True, 1.5)
    e.step(20., 20. - se.LAUNCH_DONE_MARGIN + 0.01, P.relaxed, True, 1.0)
    self.assertFalse(e.launching)
    e.step(25., 19.5, P.relaxed, True, 0.3)  # a later raise is eased
    self.assertLess(e.v_ease, 20.)

  def test_launch_ends_after_low_accel(self):
    # launched behind a lead, settled at 12 m/s with the set speed at 30: the lead pulling away later is eased
    e = se.SetSpeedEase()
    e.step(30., 0., P.standard, True, 1.5)
    n = int(round(se.LAUNCH_END_S / DT_MDL))
    for _ in range(n - 1):
      e.step(30., 12., P.standard, True, 0.0)
    self.assertTrue(e.launching)
    e.step(30., 12., P.standard, True, 0.0)
    self.assertFalse(e.launching)
    v = e.step(30., 12., P.standard, True, 1.0)
    self.assertLess(v - 12., 0.1)

  def test_low_accel_timer_needs_continuity(self):
    e = se.SetSpeedEase()
    e.step(30., 0., P.standard, True, 1.5)
    n = int(round(se.LAUNCH_END_S / DT_MDL))
    for i in range(3 * n):
      e.step(30., 12., P.standard, True, 0.0 if i % n else 1.0)  # one strong sample per second resets the timer
    self.assertTrue(e.launching)

  def test_disengage_clears_launch(self):
    e = se.SetSpeedEase()
    e.step(30., 0., P.standard, True, 1.5)
    e.step(30., 10., P.standard, False, 0.)
    self.assertFalse(e.launching)
    self.assertLess(e.step(30., 10., P.standard, True, 1.0) - 10., 0.1)


class TestCruiseAccelBounds(OpenpilotTestCase):
  """Through the REAL upstream get_cruise_accel: easing only ever lowers a positive request, never adds decel."""

  def _target(self, e2e, v_cruise, v_ego):
    CP = mock.Mock(steerRatio=15., wheelbase=2.7)
    # dt large -> the jerk limit does not bind, this is the unclipped target_accel
    return upstream_lp.get_cruise_accel(e2e, v_cruise, v_ego, 0., 0., CP, 100., 0., True)

  def test_eased_request_between_min0_and_upstream(self):
    rng = np.random.default_rng(0)
    for _ in range(3000):
      p = PERSONALITIES[rng.integers(3)]
      v_ego = float(rng.uniform(3.5, 38.))
      v_target = float(rng.uniform(0., 40.))
      e = engaged(se.SetSpeedEase(), float(rng.uniform(3.5, 38.)), p)
      for _ in range(int(rng.integers(0, 60))):
        e.step(float(rng.uniform(0., 40.)), float(rng.uniform(3.5, 38.)), p, True, float(rng.uniform(-1, 1)))
      v_ease = e.step(v_target, v_ego, p, True, 0.)
      for e2e in (False, True):
        up = self._target(e2e, v_target, v_ego)
        eased = self._target(e2e, v_ease, v_ego)
        self.assertLessEqual(eased, up + 1e-9)
        self.assertGreaterEqual(eased, min(0., up) - 1e-9)


HWY = 55 * MPH  # road_type_classifier.HIGHWAY_SPEED_THRESHOLD


def ref(road_type='unknown', map_limit=0., map_valid=None, ahead=0., ahead_valid=None, ahead_dist=0., car=0., v_set=75 * MPH):
  return se.merge_reference(road_type, map_limit, map_limit > 0. if map_valid is None else map_valid, ahead,
                            ahead > 0. if ahead_valid is None else ahead_valid, ahead_dist, car, v_set)


class TestMergeReference(OpenpilotTestCase):
  """merge_reference: which signals say 'highway-class road' (merge gate) vs 'normal road' (ease) vs no data."""

  def test_threshold_is_the_road_type_classifier_one(self):
    from openpilot.sunnypilot.mapd.lib import road_type_classifier as rtc
    self.assertEqual(se.HIGHWAY_SPEED_THRESHOLD, rtc.HIGHWAY_SPEED_THRESHOLD)

  def test_highway_road_type(self):
    for rt in ('highway', 'interstate'):
      self.assertEqual(ref(rt, 70 * MPH), (70 * MPH, se.MERGE_DEFICIT))
      self.assertEqual(ref(rt), (HWY, se.MERGE_DEFICIT))                  # highway without a limit: 55 mph floor

  def test_highway_class_map_limit(self):
    self.assertEqual(ref('unknown', 65 * MPH), (65 * MPH, se.MERGE_DEFICIT))
    self.assertEqual(ref('urban', HWY), (HWY, se.MERGE_DEFICIT))           # inclusive at 55 mph, like the classifier
    self.assertEqual(ref('unknown', 70 * MPH, map_valid=False), (75 * MPH, se.NODATA_DEFICIT))  # invalid limit = no data
    self.assertEqual(ref('unknown', 70 * MPH, map_valid=False, v_set=40 * MPH), (0., 0.))

  def test_car_limit(self):
    # carStateSP.speedLimit (cluster / camera): 12f @686 and 134 @157 have no map way match but the car reads 70
    self.assertEqual(ref('unknown', car=70 * MPH), (70 * MPH, se.MERGE_DEFICIT))
    self.assertEqual(ref('urban', 35 * MPH, car=70 * MPH), (70 * MPH, se.MERGE_DEFICIT))
    self.assertEqual(ref('unknown', car=45 * MPH, v_set=75 * MPH), (0., 0.))  # a non-highway car limit: ease

  def test_highway_limit_ahead(self):
    # an on-ramp is often an unnamed OSM link without a limit: the highway's limit is the next one
    self.assertEqual(ref(ahead=70 * MPH, ahead_dist=378.), (70 * MPH, se.MERGE_DEFICIT))
    self.assertEqual(ref(ahead=70 * MPH, ahead_dist=se.MERGE_AHEAD_DIST + 1., v_set=50 * MPH), (0., 0.))
    self.assertEqual(ref(ahead=70 * MPH, ahead_valid=False, ahead_dist=100., v_set=50 * MPH), (0., 0.))
    self.assertEqual(ref(ahead=45 * MPH, ahead_dist=100., v_set=50 * MPH), (0., 0.))

  def test_reference_capped_at_set_speed(self):
    # driving 50 on a 70 mph highway, +10 to 60: reference 60, deficit 10 mph < MERGE_DEFICIT -> eased, not a merge
    self.assertEqual(ref('highway', 70 * MPH, v_set=60 * MPH), (60 * MPH, se.MERGE_DEFICIT))
    g = se.MergeGate()
    self.assertFalse(g.update(*ref('highway', 70 * MPH, v_set=60 * MPH), 50 * MPH))

  def test_highest_highway_evidence_wins(self):
    self.assertEqual(ref('highway', 55 * MPH, car=65 * MPH, ahead=70 * MPH, ahead_dist=100.)[0], 70 * MPH)

  def test_normal_road_never_merges(self):
    for kw in ({'road_type': 'urban'}, {'map_limit': 45 * MPH}, {'car': 35 * MPH}, {'road_type': 'urban', 'map_limit': 50 * MPH}):
      self.assertEqual(ref(v_set=80 * MPH, **kw), (0., 0.), kw)

  def test_no_data_fails_safe_on_highway_class_target(self):
    self.assertEqual(ref(v_set=75 * MPH), (75 * MPH, se.NODATA_DEFICIT))
    self.assertEqual(ref(v_set=HWY), (HWY, se.NODATA_DEFICIT))
    self.assertEqual(ref(v_set=50 * MPH), (0., 0.))
    # roadType is a capnp enum on the car: compared as text
    lm = messaging.new_message('liveMapDataSP').liveMapDataSP
    lm.roadType = 'interstate'
    self.assertEqual(ref(lm.roadType)[0], HWY)
    # ... and the classifier's own return value (a raw int)
    from openpilot.sunnypilot.mapd.lib.road_type_classifier import classify_road_type
    self.assertEqual(ref(classify_road_type('I-94', 0.))[0], HWY)
    self.assertEqual(ref(classify_road_type('Main Street', 0.), v_set=80 * MPH), (0., 0.))
    self.assertEqual(se.road_type_name(99), 'unknown')

  def test_deficits(self):
    # a +10 mph discretionary bump must stay eased in both the map and the no-data case
    self.assertGreater(se.MERGE_DEFICIT, 10 * MPH + se.MERGE_HYST)
    self.assertGreater(se.NODATA_DEFICIT, se.MERGE_DEFICIT)


class TestMergeGate(OpenpilotTestCase):
  def test_enter_exit_hysteresis(self):
    g = se.MergeGate()
    r = 70 * MPH
    self.assertFalse(g.update(r, se.MERGE_DEFICIT, r - se.MERGE_DEFICIT))           # exactly at the edge: not merging
    self.assertTrue(g.update(r, se.MERGE_DEFICIT, r - se.MERGE_DEFICIT - 0.01))
    self.assertTrue(g.update(r, se.MERGE_DEFICIT, r - se.MERGE_DEFICIT + se.MERGE_HYST - 0.01))  # inside the band
    self.assertFalse(g.update(r, se.MERGE_DEFICIT, r - se.MERGE_DEFICIT + se.MERGE_HYST))
    self.assertFalse(g.update(r, se.MERGE_DEFICIT, r - se.MERGE_DEFICIT + 0.5))      # no re-entry inside the band

  def test_no_reference_clears(self):
    g = se.MergeGate()
    self.assertTrue(g.update(30., se.MERGE_DEFICIT, 10.))
    self.assertFalse(g.update(0., 0., 10.))


class TestMergeStep(OpenpilotTestCase):
  def test_merging_passes_target_through(self):
    for p in PERSONALITIES:
      e = engaged(se.SetSpeedEase(), 12., p)
      self.assertEqual(e.step(31., 12., p, True, 0.5, merging=True), 31.)

  def test_merge_exit_hands_over_at_the_leash(self):
    # leaving the merge: v_ease continues from v_ego + rate * LEASH_S (no drop of the request to zero, no step to target)
    for p in PERSONALITIES:
      e = engaged(se.SetSpeedEase(), 12., p)
      e.step(31., 22., p, True, 0.5, merging=True)
      lim = 22. + se.get_rate(p, 22.) * se.LEASH_S
      v = e.step(31., 22., p, True, 0.5, merging=False)
      self.assertAlmostEqual(v, lim, places=6)
      self.assertLess(v, 31.)

  def test_merge_never_above_target(self):
    e = engaged(se.SetSpeedEase(), 20.)
    e.step(20.5, 20., P.standard, True, 0.5, merging=True)
    self.assertLessEqual(e.v_ease, 20.5)

  def test_down_still_immediate_after_merge(self):
    e = engaged(se.SetSpeedEase(), 12.)
    e.step(31., 20., P.relaxed, True, 0.5, merging=True)
    self.assertEqual(e.step(18., 20., P.relaxed, True, 0.5), 18.)


def planner_sm(v_ego, v_cruise_kph, personality=P.standard, experimental=False, lcs=LongCtrlState.pid, override=False,
               lead=None, road_type=None, map_limit=0., car_limit=0.):
  N = len(ModelConstants.T_IDXS)
  md = messaging.new_message('modelV2').modelV2
  md.position.x = [v_ego * t for t in ModelConstants.T_IDXS]
  md.velocity.x = [v_ego] * N
  md.orientationRate.z = [0.] * N
  md.action.desiredAcceleration = 2.0
  md.meta.disengagePredictions.gasPressProbs = [1.] * 6
  cs = messaging.new_message('carState').carState
  cs.vEgo = v_ego
  cs.vCruise = v_cruise_kph
  cs.vCruiseCluster = v_cruise_kph
  cc = messaging.new_message('carControl').carControl
  cc.enabled = True
  cc.cruiseControl.override = override
  cc.orientationNED = [0., 0., 0.]
  ctl = messaging.new_message('controlsState').controlsState
  ctl.longControlState = lcs
  sd = messaging.new_message('selfdriveState').selfdriveState
  sd.enabled = True
  sd.experimentalMode = experimental
  sd.personality = personality
  rs = messaging.new_message('radarState').radarState
  if lead is not None:
    rs.leadOne.present = True
    rs.leadOne.dRel = lead[0]
    rs.leadOne.vLead = lead[1]
    rs.leadOne.modelProb = 1.
    rs.leadOne.aLeadTau = 1.5
  sm = {'modelV2': md, 'carState': cs, 'carControl': cc, 'controlsState': ctl, 'selfdriveState': sd, 'radarState': rs}
  for w in ('liveMapDataSP', 'vehicleParameters', 'carStateSP', 'gpsLocation', 'gpsLocationExternal', 'selfdriveStateSP'):
    sm[w] = getattr(messaging.new_message(w), w)
  if road_type is not None:
    sm['liveMapDataSP'].roadType = road_type
  if map_limit > 0.:
    sm['liveMapDataSP'].speedLimit = map_limit
    sm['liveMapDataSP'].speedLimitValid = True
  sm['carStateSP'].speedLimit = car_limit
  return sm


def make_planner():
  from opendbc.car.honda.interface import CarInterface
  from opendbc.car.honda.values import CAR
  CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
  CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.HONDA_CIVIC)
  return upstream_lp.LongitudinalPlanner(CP, CP_SP, init_v=20., init_a=0.)


class TestUpstreamHooks(OpenpilotTestCase):
  """The fork hooks these upstream members; a rename / signature change upstream must fail here."""

  def test_pinned_signatures(self):
    self.assertEqual(list(inspect.signature(upstream_lp.get_cruise_accel).parameters),
                     ['e2e', 'v_cruise', 'v_ego', 'a_cruise_prev', 'angle_steers', 'CP', 'dt', 'accel_coast', 'allow_throttle'])
    self.assertEqual(list(inspect.signature(LongitudinalPlannerSP.update_targets).parameters),
                     ['self', 'sm', 'v_ego', 'a_ego', 'v_cruise'])
    src = inspect.getsource(upstream_lp.LongitudinalPlanner.update)
    # the planner must feed update_targets' returned v_cruise into get_cruise_accel
    self.assertIn('v_cruise, self.output_a_target = LongitudinalPlannerSP.update_targets(', src)
    self.assertIn('get_cruise_accel(is_e2e, v_cruise, v_ego', src)

  def _run(self, planner, n, v_ego, v_cruise_kph, **kw):
    for _ in range(n):
      planner.update(planner_sm(v_ego, v_cruise_kph, **kw))

  def test_planner_owns_ease(self):
    self.assertIsInstance(make_planner().setspeed_ease, se.SetSpeedEase)

  def test_end_to_end_bump_is_eased(self):
    # +10 mph at 45 mph through the REAL LongitudinalPlanner: the cruise candidate stays near the personality rate
    # (upstream: the ceiling within ~1 s). ACC and e2e.
    v = 45 * MPH
    kph = 55 * 1.609344
    for exp in (False, True):
      for p in PERSONALITIES:
        planner = make_planner()
        self._run(planner, 5, v, 45 * 1.609344, personality=p, experimental=exp)
        self._run(planner, 40, v, kph, personality=p, experimental=exp)  # 2 s after the press, car not moving (worst case)
        self.assertLessEqual(planner.a_cruise, se.get_rate(p, v) * se.LEASH_S + 1e-6, (exp, p))
        self.assertGreater(planner.a_cruise, 0.2)
        # vTarget published to longitudinalPlanSP stays the raw target
        self.assertAlmostEqual(planner.output_v_target, kph / 3.6, places=4)

  def test_eased_with_far_lead(self):
    # a lead far ahead and faster (not binding) must not switch the easing off: +10 mph at 45 mph stays eased
    v = 45 * MPH
    planner = make_planner()
    self._run(planner, 5, v, 45 * 1.609344, lead=(150., v + 3.))
    self._run(planner, 40, v, 55 * 1.609344, lead=(150., v + 3.))
    self.assertLessEqual(planner.a_cruise, se.get_rate(P.standard, v) * se.LEASH_S + 1e-6)
    self.assertLess(planner.output_a_target, float(upstream_lp.get_max_accel(v)) - 0.2)

  def test_lead_braking_identical_on_same_input(self):
    # Same input sequence to two real planners (eased / hook bypassed): cruising at 45 mph, +15 mph press, then a slow
    # lead cuts in close and the car must slow. Whenever the plan comes from the lead (MPC) or asks to slow, the eased
    # planner's output must equal the upstream one: easing never touches braking / lead-follow decel.
    v = 45 * MPH
    for exp in (False, True):
      eased, stock = make_planner(), make_planner()
      stock.setspeed_ease.update = lambda sm, v_target, *a: v_target
      seq = [planner_sm(v, 45 * 1.609344, experimental=exp) for _ in range(10)]
      seq += [planner_sm(v, 60 * 1.609344, experimental=exp) for _ in range(40)]
      seq += [planner_sm(v, 60 * 1.609344, experimental=exp, lead=(18. + 0.2 * i, 40 * MPH)) for i in range(60)]
      n_lead = 0
      for sm in seq:
        eased.update(sm)
        stock.update(sm)
        if sm['radarState'].leadOne.present and stock.output_a_target < 0.:
          n_lead += 1
          self.assertAlmostEqual(eased.output_a_target, stock.output_a_target, places=2, msg=f'exp={exp}')
          self.assertNotEqual(str(eased.mpc.source), 'cruise')
        self.assertLessEqual(eased.output_a_target, stock.output_a_target + 1e-3)
      self.assertGreater(n_lead, 30)  # the lead phase really braked

  def test_end_to_end_control_without_ease(self):
    # positive control for the test above: with the hook bypassed the same input reaches the upstream ceiling
    v = 45 * MPH
    planner = make_planner()
    planner.setspeed_ease.update = lambda sm, v_target, *a: v_target
    self._run(planner, 5, v, 45 * 1.609344, personality=P.relaxed)
    self._run(planner, 40, v, 55 * 1.609344, personality=P.relaxed)
    self.assertAlmostEqual(planner.a_cruise, float(upstream_lp.get_max_accel(v)), places=3)

  def test_end_to_end_down_immediate(self):
    v = 45 * MPH
    planner = make_planner()
    self._run(planner, 5, v, 45 * 1.609344)
    self._run(planner, 20, v, 65 * 1.609344)
    self._run(planner, 1, v, 40 * 1.609344)
    self.assertAlmostEqual(planner.setspeed_ease.v_ease, 40 * 1.609344 / 3.6, places=4)  # vCruise is float32

  def test_end_to_end_inactive_states(self):
    v = 45 * MPH
    for kw in ({'lcs': LongCtrlState.off}, {'override': True}):
      planner = make_planner()
      self._run(planner, 5, v, 45 * 1.609344)
      self._run(planner, 20, v, 65 * 1.609344, **kw)
      self.assertAlmostEqual(planner.setspeed_ease.v_ease, v, places=4, msg=str(kw))
      # while not in control the planner gets the raw target (upstream behaviour)
      v_out, _ = planner.update_targets(planner_sm(v, 65 * 1.609344, **kw), v, 0., 65 * 1.609344 / 3.6)
      self.assertAlmostEqual(v_out, 65 * 1.609344 / 3.6, places=4, msg=str(kw))

  def test_launch_end_uses_plan_accel(self):
    # the hook passes the planner's previous output accel (a_ego arg of update_targets) as a_plan
    planner = make_planner()
    with mock.patch.object(planner.setspeed_ease, 'update', wraps=planner.setspeed_ease.update) as up:
      planner.output_a_target = 0.77
      planner.update(planner_sm(20., 100.))
      self.assertAlmostEqual(up.call_args.args[4], 0.77, places=5)

  def test_diagnostics_event_on_ramp_edges(self):
    e = se.SetSpeedEase()
    with mock.patch.object(se.cloudlog, 'event') as ev:
      e.update(planner_sm(20., 72.), 20., True, False, 0.)
      ev.assert_not_called()
      e.update(planner_sm(20., 108.), 30., True, False, 0.)
      e.update(planner_sm(20., 108.), 30., True, False, 0.)
      self.assertEqual(ev.call_count, 1)
      self.assertEqual(ev.call_args.args[0], 'setspeed_ease')
      self.assertTrue(ev.call_args.kwargs['ramping'])
      self.assertEqual(ev.call_args.kwargs['personality'], int(P.standard))
      e.update(planner_sm(20., 72.), 20., True, False, 0.)
      self.assertEqual(ev.call_count, 2)
      self.assertFalse(ev.call_args.kwargs['ramping'])

  def test_end_to_end_onramp_not_eased(self):
    # on-ramp (12f @686 shape): 30 mph, set 75 mph, the car reads a 70 mph limit (no map way match): upstream ceiling
    v = 30 * MPH
    for road in ({'car_limit': 70 * MPH}, {'road_type': 'highway', 'map_limit': 70 * MPH}, {}):
      for p in PERSONALITIES:
        planner = make_planner()
        self._run(planner, 5, v, 30 * 1.609344, personality=p, **road)
        self._run(planner, 40, v, 75 * 1.609344, personality=p, **road)
        self.assertTrue(planner.setspeed_ease.merging, (road, p))
        self.assertAlmostEqual(planner.a_cruise, float(upstream_lp.get_max_accel(v)), places=3, msg=str((road, p)))

  def test_end_to_end_bump_on_normal_road_still_eased(self):
    # +10 mph at 45 mph on an urban 45 mph road, and +10 mph at 65 mph on a 70 mph highway: eased (discretionary)
    for v_mph, road in ((45, {'road_type': 'urban', 'map_limit': 45 * MPH}), (65, {'road_type': 'highway', 'map_limit': 70 * MPH}),
                        (45, {}), (50, {'road_type': 'highway', 'map_limit': 70 * MPH})):
      planner = make_planner()
      v = v_mph * MPH
      self._run(planner, 5, v, v_mph * 1.609344, personality=P.relaxed, **road)
      self._run(planner, 40, v, (v_mph + 10) * 1.609344, personality=P.relaxed, **road)
      self.assertFalse(planner.setspeed_ease.merging, road)
      self.assertLessEqual(planner.a_cruise, se.get_rate(P.relaxed, v) * se.LEASH_S + 1e-6, road)

  def test_end_to_end_override_resets_ramp(self):
    # mid-ramp the driver presses the gas to 25 m/s: the planner gets the raw target and v_ease re-arms at vEgo;
    # after the release the ramp restarts from the (new) current speed, not from where it was
    v = 45 * MPH
    planner = make_planner()
    self._run(planner, 5, v, 45 * 1.609344)
    self._run(planner, 20, v, 65 * 1.609344)
    self.assertGreater(planner.setspeed_ease.v_ease, v)
    self._run(planner, 5, 25., 65 * 1.609344, override=True)
    self.assertEqual(planner.setspeed_ease.v_ease, 25.)
    v_out, _ = planner.update_targets(planner_sm(25., 65 * 1.609344, override=True), 25., 0., 65 * 1.609344 / 3.6)
    self.assertAlmostEqual(v_out, 65 * 1.609344 / 3.6, places=4)
    self._run(planner, 1, 25., 65 * 1.609344)
    self.assertAlmostEqual(planner.setspeed_ease.v_ease, 25. + se.get_rate(P.standard, 25.) * DT_MDL, places=4)

  def test_diagnostics_event_on_merge_edges(self):
    e = se.SetSpeedEase()
    with mock.patch.object(se.cloudlog, 'event') as ev:
      e.update(planner_sm(13., 120., car_limit=70 * MPH), 33., True, False, 0.)
      self.assertTrue(ev.call_args.kwargs['merging'])
      e.update(planner_sm(13., 120., car_limit=70 * MPH), 33., True, False, 0.)
      self.assertEqual(ev.call_count, 1)
      e.update(planner_sm(13., 120., lcs=LongCtrlState.off, car_limit=70 * MPH), 33., True, False, 0.)
      self.assertFalse(e.merge_gate.merging)   # inactive clears the gate

  def test_merge_survives_a_lower_arbitrated_target(self):
    # on the ramp an SCC curve target (arbitrated v_target) dips to 15 m/s while the set speed stays 75 mph: the gate
    # keys off the SET speed, stays merging, and passes the (lower) arbitrated target through unchanged
    e = se.SetSpeedEase()
    sm = planner_sm(12., 75 * 1.609344, car_limit=70 * MPH)
    self.assertEqual(e.update(sm, 33., True, False, 0.5, 33.), 33.)
    self.assertTrue(e.merging)
    self.assertEqual(e.update(sm, 15., True, False, 0.5, 33.), 15.)
    self.assertTrue(e.merging)
    # without the set speed (default) the arbitrated target is the cap: 15 m/s is no merge -> eased
    e2 = se.SetSpeedEase()
    e2.update(sm, 33., True, False, 0.5)
    self.assertLess(e2.update(sm, 15., True, False, 0.5), 15.)
    self.assertFalse(e2.merging)

  def test_hook_passes_the_set_speed(self):
    planner = make_planner()
    with mock.patch.object(planner.setspeed_ease, 'update', wraps=planner.setspeed_ease.update) as up:
      planner.update(planner_sm(20., 100.))
      self.assertAlmostEqual(up.call_args.args[5], 100. / 3.6, places=3)
