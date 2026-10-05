"""
Fork SCC fixes + throttle-only guard (fork/scc.py). Scenarios are taken from routes 0000012e / 0000012f
(car-features/drive-12e-12f-report.md §B; an12ef/scc_episodes.py, scc_uncorr.py).
"""
import json
from unittest import mock

import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom, log
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.fork import scc as fscc
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import MIN_V
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.map_controller import R, SmartCruiseControlMap
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.vision_controller import SmartCruiseControlVision

VisionState = custom.LongitudinalPlanSP.SmartCruiseControl.VisionState
MapState = custom.LongitudinalPlanSP.SmartCruiseControl.MapState
LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource
RoadType = custom.LiveMapDataSP.RoadType
N = len(ModelConstants.T_IDXS)
LAT0, LON0 = 41.5, -87.3
M_PER_DEG_LAT = R * np.pi / 180.


def model_sm(v_ego: float, cur_lat: float, pred_lat: float, live_map=None) -> dict:
  """sm for SCC-V: predicted lat accel = |orientationRate.z| * velocity.x = pred_lat at every point; current lat accel
  = v^2 * |curvature| = cur_lat."""
  mdl = messaging.new_message('modelV2')
  mdl.modelV2.velocity.x = [1.0] * N
  mdl.modelV2.orientationRate.z = [float(pred_lat)] * N
  cs = messaging.new_message('controlsState')
  cs.controlsState.curvature = float(cur_lat / max(v_ego, 0.1) ** 2)
  sm = {'modelV2': mdl.modelV2, 'controlsState': cs.controlsState}
  sm['liveMapDataSP'] = live_map if live_map is not None else messaging.new_message('liveMapDataSP').liveMapDataSP
  return sm


def matched_map(name="Casimir Pulaski Memorial Highway"):
  m = messaging.new_message('liveMapDataSP').liveMapDataSP
  m.roadType = RoadType.highway
  m.roadName = name
  m.speedLimitValid = True
  m.speedLimit = 31.3
  return m


def put_path(params, offset_m: float, points: list[tuple[float, float]]):
  """Car at (LAT0, LON0). Path due north starting offset_m east of the car: [(ahead_m, velocity), ...]."""
  dlon = offset_m / (M_PER_DEG_LAT * np.cos(np.radians(LAT0)))
  tv = [{"latitude": LAT0 + ahead / M_PER_DEG_LAT, "longitude": LON0 + dlon, "velocity": v} for ahead, v in points]
  params.put("LastGPSPosition", json.dumps({"latitude": LAT0, "longitude": LON0}), block=True)
  params.put("MapTargetVelocities", json.dumps(tv), block=True)


# 12f @698: on the ramp, a 14.93 m/s point ~100 m ahead on the list, car at 20.7 m/s
RAMP_PATH = [(0., 30.), (40., 30.), (100., 14.93), (160., 30.)]


class TestUpstreamHooks(OpenpilotTestCase):
  """The fork overrides these upstream members; a rename upstream must fail here, not silently drop the fix."""

  def test_overridden_members_exist(self):
    for name in ("get_v_target_from_control", "_update_state_machine", "_update_calculations", "update"):
      self.assertTrue(callable(getattr(SmartCruiseControlVision, name)), name)
    for name in ("get_v_target_from_control", "update_calculations", "_update_state_machine", "update"):
      self.assertTrue(callable(getattr(SmartCruiseControlMap, name)), name)
    v = SmartCruiseControlVision()
    for attr in ("state", "v_ego", "current_lat_acc", "max_pred_lat_acc", "is_active", "output_v_target"):
      self.assertTrue(hasattr(v, attr), attr)
    m = SmartCruiseControlMap()
    for attr in ("state", "last_position", "target_velocities", "v_target", "target_lat", "target_lon", "frame"):
      self.assertTrue(hasattr(m, attr), attr)

  def test_planner_uses_fork_scc(self):
    from opendbc.car.honda.interface import CarInterface
    from opendbc.car.honda.values import CAR
    from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
    CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.HONDA_CIVIC)
    planner = LongitudinalPlanner(CP, CP_SP)
    self.assertIsInstance(planner.scc, fscc.ForkSmartCruiseControl)
    self.assertIsInstance(planner.scc.vision, fscc.ForkSCCVision)
    self.assertIsInstance(planner.scc.map, fscc.ForkSCCMap)
    self.assertFalse(planner.scc.throttle_only)  # Honda: has brakes, guard inert


class TestVision(OpenpilotTestCase):
  def setup_method(self):
    Params().put_bool("SmartCruiseControlVision", True, block=True)

  def _run(self, cls, seq):
    c = cls()
    out = []
    for v, cur, pred in seq:
      c.update(model_sm(v, cur, pred), True, False, v, 0., 30.)
      out.append((c.state, c.output_v_target, c.a_target))
    return c, out

  # 12f @1574.75-1577.0 launch into a turn (v, current lat accel, predicted lat accel), 20 Hz
  LAUNCH_12F = [(4.32, 1.2, 2.18)] * 5 + [(4.63, 1.25, 2.24)] * 5 + [(5.0, 1.3, 2.25)] * 5 + [(5.6, 0.95, 2.26)] * 10 + \
               [(5.77, 0.95, 2.32)] * 10 + [(5.53, 0.9, 2.23)] * 10

  def test_launch_into_turn_upstream_reproduces_the_bug(self):
    # positive control: upstream enters ENTERING at 5.6 m/s (> MIN_V) and outputs a target BELOW its own MIN_V floor
    _, out = self._run(SmartCruiseControlVision, self.LAUNCH_12F)
    entering = [o for o in out if o[0] == VisionState.entering]
    self.assertTrue(entering)
    self.assertLess(min(o[1] for o in entering), MIN_V - 1.)

  def test_launch_into_turn_fixed(self):
    _, out = self._run(fscc.ForkSCCVision, self.LAUNCH_12F)
    self.assertTrue(all(o[0] == VisionState.enabled for o in out[1:]), [str(o[0]) for o in out])
    self.assertTrue(all(o[1] == V_CRUISE_UNSET for o in out))

  def test_output_floored_at_min_v(self):
    c = fscc.ForkSCCVision()
    c.update(model_sm(12., 0.2, 3.0), True, False, 12., 0., 30.)  # disabled -> enabled
    c.update(model_sm(12., 0.2, 3.0), True, False, 12., 0., 30.)  # enabled -> entering
    self.assertEqual(c.state, VisionState.entering)
    c.update(model_sm(5.7, 0.2, 3.0), True, False, 5.7, 0., 30.)  # still entering, slow, max decel
    self.assertEqual(c.state, VisionState.entering)
    self.assertLess(c.a_target, -0.5)
    self.assertGreaterEqual(c.output_v_target, MIN_V)
    upstream_value = max(c.v_target, MIN_V) + c.a_target * 4.
    self.assertLess(upstream_value, MIN_V)  # i.e. the floor is what held it

  def test_no_entering_below_9_mps_but_above(self):
    for v, expect in ((6., VisionState.enabled), (8.9, VisionState.enabled), (9.1, VisionState.entering), (20., VisionState.entering)):
      c = fscc.ForkSCCVision()
      for _ in range(3):
        c.update(model_sm(v, 0.2, 2.0), True, False, v, 0., 30.)
      self.assertEqual(c.state, expect, v)

  def test_already_turning_goes_to_turning(self):
    # 12e @337-343 kind: at >= 9 m/s, ENTERING first fires while the car is already at 1.0-1.6 m/s^2 lateral
    for cur, expect in ((1.2, VisionState.turning), (1.0, VisionState.turning), (0.6, VisionState.entering)):
      c = fscc.ForkSCCVision()
      c.update(model_sm(12., cur, 2.2), True, False, 12., 0., 30.)
      c.update(model_sm(12., cur, 2.2), True, False, 12., 0., 30.)
      self.assertEqual(c.state, expect, cur)
      if expect == VisionState.turning:
        self.assertGreaterEqual(c.a_target, 0.)  # gentle accel, not a decel

  def test_other_transitions_untouched(self):
    # entering -> turning at >= 1.6 still happens, and the feature stays on
    c = fscc.ForkSCCVision()
    for cur in (0.2, 0.2, 1.7):
      c.update(model_sm(15., cur, 2.4), True, False, 15., 0., 30.)
    self.assertEqual(c.state, VisionState.turning)


class TestMap(OpenpilotTestCase):
  def setup_method(self):
    self.params = Params()
    self.params.put_bool("SmartCruiseControlMap", True, block=True)

  def _run(self, cls, n, v=20.7, live_map=None, v_cruise=30.):
    c = cls()
    if live_map is not None:
      c.live_map = live_map
    states = []
    for _ in range(n):
      c.update(True, False, v, 0., v_cruise)
      states.append((c.state, c.output_v_target))
    return c, states

  def test_positive_control_near_matched_path_turns(self):
    put_path(self.params, 3., RAMP_PATH)
    c, st = self._run(fscc.ForkSCCMap, 5, live_map=matched_map())
    self.assertEqual(c.state, MapState.turning)
    self.assertAlmostEqual(st[-1][1], 14.93, places=2)
    self.assertLess(c.nearest_dist, 5.)

  def test_far_path_ignored(self):
    put_path(self.params, 60., RAMP_PATH)
    _, st_up = self._run(SmartCruiseControlMap, 5)  # upstream: binds on a path 60 m away (min_dist never checked)
    self.assertEqual(st_up[-1][0], MapState.turning)
    c, st = self._run(fscc.ForkSCCMap, 5, live_map=matched_map())
    self.assertTrue(all(s == MapState.enabled for s, _ in st[1:]))
    self.assertEqual(c.reject_reason, "far")
    self.assertGreater(c.nearest_dist, fscc.MAP_MAX_NEAREST_DIST)

  def test_gate_boundary(self):
    for off, turning in ((20., True), (30., False)):
      put_path(self.params, off, RAMP_PATH)
      c, _ = self._run(fscc.ForkSCCMap, 5, live_map=matched_map())
      self.assertEqual(c.state == MapState.turning, turning, off)

  def test_unmatched_ignored(self):
    # 12f 682.5-705.5: roadType unknown, no name, no valid speed limit
    put_path(self.params, 3., RAMP_PATH)
    unmatched = messaging.new_message('liveMapDataSP').liveMapDataSP
    c, st = self._run(fscc.ForkSCCMap, 5, live_map=unmatched)
    self.assertTrue(all(s == MapState.enabled for s, _ in st[1:]))
    self.assertEqual(c.reject_reason, "unmatched")
    # any one of a name / road type / speed limit counts as matched
    for setup in (lambda m: setattr(m, 'roadName', 'E 181st Ave'), lambda m: setattr(m, 'roadType', RoadType.urban),
                  lambda m: setattr(m, 'speedLimitValid', True)):
      m = messaging.new_message('liveMapDataSP').liveMapDataSP
      setup(m)
      self.assertFalse(fscc.map_unmatched(m))

  def test_active_rejection_needs_hold(self):
    # mid-curve, a 1-sample match/GPS blip must not snap the target back to cruise; 1 s of rejection does
    put_path(self.params, 3., RAMP_PATH)
    c, _ = self._run(fscc.ForkSCCMap, 5, live_map=matched_map())
    self.assertEqual(c.state, MapState.turning)
    unmatched = messaging.new_message('liveMapDataSP').liveMapDataSP
    hold = int(round(fscc.MAP_REJECT_HOLD_S / DT_MDL))
    for i in range(hold + 1):
      c.live_map = unmatched
      c.update(True, False, 20.7, 0., 30.)
      if i < hold - 1:
        self.assertEqual(c.state, MapState.turning, i)
    self.assertEqual(c.state, MapState.enabled)
    self.assertEqual(c.output_v_target, V_CRUISE_UNSET)
    # a blip shorter than the hold is ignored
    c2, _ = self._run(fscc.ForkSCCMap, 5, live_map=matched_map())
    for live in [unmatched] * 3 + [matched_map()] * 30:
      c2.live_map = live
      c2.update(True, False, 20.7, 0., 30.)
      self.assertEqual(c2.state, MapState.turning)

  def test_diagnostics_logged(self):
    put_path(self.params, 60., RAMP_PATH)
    with mock.patch.object(fscc.cloudlog, "event") as ev:
      self._run(fscc.ForkSCCMap, int(3. / DT_MDL), live_map=matched_map())
    diags = [c.kwargs for c in ev.call_args_list if c.args and c.args[0] == "scc_map_diag"]
    self.assertGreaterEqual(len(diags), 3)  # >= 1 Hz while engaged
    self.assertLessEqual(len(diags), 3 + 5)  # rate-capped
    d = diags[-1]
    for k in ("points", "nearest_m", "nearest_idx", "target_lat", "target_lon", "reject", "road_type", "road_name", "state"):
      self.assertIn(k, d)
    self.assertEqual(d["points"], len(RAMP_PATH))
    self.assertEqual(d["reject"], "far")
    self.assertAlmostEqual(d["nearest_m"], 60., delta=1.)
    # not engaged -> no diagnostics
    with mock.patch.object(fscc.cloudlog, "event") as ev:
      c = fscc.ForkSCCMap()
      for _ in range(50):
        c.update(False, False, 20., 0., 30.)
    self.assertFalse([x for x in ev.call_args_list if x.args and x.args[0] == "scc_map_diag"])


  def test_map_path_logged_when_it_changes(self):
    """Route data: MapTargetVelocities is only in /dev/shm params; scc_map_path puts it in the rlog (engaged or not),
    only when its content changes, rate-capped."""
    put_path(self.params, 0., RAMP_PATH)
    with mock.patch.object(fscc.cloudlog, "event") as ev:
      c = fscc.ForkSCCMap()
      for _ in range(int(5. / DT_MDL)):
        c.update(False, False, 20., 0., 30.)
    paths = [x.kwargs for x in ev.call_args_list if x.args and x.args[0] == "scc_map_path"]
    self.assertEqual(len(paths), 1)  # unchanged path -> logged once
    self.assertEqual(paths[0]["n"], len(RAMP_PATH))
    self.assertEqual(len(paths[0]["points"]), len(RAMP_PATH))
    self.assertEqual([p[2] for p in paths[0]["points"]], [v for _, v in RAMP_PATH])
    # a new path is logged again, but not faster than MAP_PATH_MIN_GAP_S
    with mock.patch.object(fscc.cloudlog, "event") as ev:
      for i in range(int(4. / DT_MDL)):
        put_path(self.params, float(i % 7), RAMP_PATH)  # content changes every cycle
        c.update(False, False, 20., 0., 30.)
    paths = [x for x in ev.call_args_list if x.args and x.args[0] == "scc_map_path"]
    self.assertGreaterEqual(len(paths), 1)
    self.assertLessEqual(len(paths), int(4. / fscc.MAP_PATH_MIN_GAP_S) + 1)

  def test_map_path_capped(self):
    big = [{"latitude": LAT0 + i * 1e-5, "longitude": LON0, "velocity": 20.} for i in range(fscc.MAP_PATH_MAX_POINTS + 50)]
    self.params.put("MapTargetVelocities", json.dumps(big), block=True)
    with mock.patch.object(fscc.cloudlog, "event") as ev:
      c = fscc.ForkSCCMap()
      c.update(False, False, 20., 0., 30.)
    p = [x.kwargs for x in ev.call_args_list if x.args and x.args[0] == "scc_map_path"][0]
    self.assertEqual(p["n"], len(big))
    self.assertTrue(p["truncated"])
    self.assertEqual(len(p["points"]), fscc.MAP_PATH_MAX_POINTS)

class TestGuardUnit(OpenpilotTestCase):
  def _feed(self, g, n, **kw):
    a = {"long_enabled": True, "v_ego": 20.7, "a_prev": -1.2, "raw_v_targets": (V_CRUISE_UNSET, 14.93),
         "current_lat_acc": 0.3, "max_pred_lat_acc": 0.4}
    a.update(kw)
    return [g.update(**a) for _ in range(n)]

  def test_releases_after_2s_uncorroborated(self):
    g = fscc.ThrottleOnlySCCGuard()
    out = self._feed(g, int(3. / DT_MDL))
    first = out.index(True)
    self.assertAlmostEqual(first * DT_MDL, fscc.GUARD_UNCORROBORATED_S - DT_MDL, delta=DT_MDL)
    self.assertTrue(all(out[first:]))

  def test_latched_release_does_not_sawtooth(self):
    # after release the plan accelerates (a_prev > 0) and the raw SCC target is still low: must stay released
    g = fscc.ThrottleOnlySCCGuard()
    self._feed(g, int(2.5 / DT_MDL))
    self.assertTrue(all(self._feed(g, int(10. / DT_MDL), a_prev=0.8)))

  def test_corroboration_never_releases(self):
    for kw in ({"current_lat_acc": 1.0}, {"max_pred_lat_acc": 1.3}, {"current_lat_acc": 2.3, "max_pred_lat_acc": 2.5}):
      g = fscc.ThrottleOnlySCCGuard()
      self.assertFalse(any(self._feed(g, int(15. / DT_MDL), **kw)), kw)

  def test_corroboration_resets_a_latched_guard(self):
    g = fscc.ThrottleOnlySCCGuard()
    self._feed(g, int(2.5 / DT_MDL))
    self.assertFalse(self._feed(g, 1, max_pred_lat_acc=2.0)[0])
    self.assertFalse(self._feed(g, int(1.5 / DT_MDL))[-1])  # timer restarted from 0

  def test_resets(self):
    for kw in ({"raw_v_targets": (V_CRUISE_UNSET, V_CRUISE_UNSET)}, {"raw_v_targets": (21.8, V_CRUISE_UNSET)},
               {"long_enabled": False}):
      g = fscc.ThrottleOnlySCCGuard()
      self._feed(g, int(2.5 / DT_MDL))
      self.assertFalse(self._feed(g, 1, **kw)[0], kw)
      self.assertEqual(g.uncorroborated_t, 0.)
    # hysteresis: a raw target only just above vEgo (inside the reset margin) does not reset
    g = fscc.ThrottleOnlySCCGuard()
    self._feed(g, int(2.5 / DT_MDL))
    self.assertTrue(self._feed(g, 1, raw_v_targets=(21.2, V_CRUISE_UNSET))[0])

  def test_only_counts_while_coasting_and_slowing(self):
    g = fscc.ThrottleOnlySCCGuard()
    self.assertFalse(any(self._feed(g, int(5. / DT_MDL), a_prev=0.2)))  # not coasting
    g = fscc.ThrottleOnlySCCGuard()
    self.assertFalse(any(self._feed(g, int(5. / DT_MDL), raw_v_targets=(20.5, V_CRUISE_UNSET))))  # not slowing


class TestEndToEndPlanner(OpenpilotTestCase):
  """Through the real LongitudinalPlanner (MPC + LongitudinalPlannerSP arbitration) on the 12f @698 phantom: a matched,
  near map path asks for 14.93 m/s at 20.7 m/s on a straight merge (no lateral evidence)."""

  def setup_method(self):
    self.params = Params()
    self.params.put_bool("SmartCruiseControlMap", True, block=True)
    self.params.put_bool("SmartCruiseControlVision", True, block=True)

  def _planner(self, throttle_only: bool):
    from opendbc.car.honda.interface import CarInterface
    from opendbc.car.honda.values import CAR
    from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
    CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
    CP.openpilotLongitudinalControl = True
    CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.HONDA_CIVIC)
    p = LongitudinalPlanner(CP, CP_SP, init_v=20.7)
    p.scc.throttle_only = throttle_only
    return p

  def _sm(self, v, live_map, cur_lat=0.2, pred_lat=0.3, v_cruise=30.):
    sm = model_sm(v, cur_lat, pred_lat, live_map)
    msgs = {k: messaging.new_message(k) for k in ('radarState', 'selfdriveState', 'carState', 'vehicleParameters',
                                                    'carControl', 'carStateSP', 'gpsLocation')}
    msgs['carState'].carState.vEgo = float(v)
    msgs['carState'].carState.vCruise = float(v_cruise * 3.6)
    msgs['carState'].carState.vCruiseCluster = float(v_cruise * 3.6)
    msgs['carControl'].carControl.enabled = True
    msgs['carControl'].carControl.orientationNED = [0., 0., 0.]
    msgs['selfdriveState'].selfdriveState.personality = log.LongitudinalPersonality.standard
    sm['modelV2'].meta.disengagePredictions.gasPressProbs = [1.0] * 6
    sm['modelV2'].position.x = [float(x) for x in (v + 0.5) * np.array(ModelConstants.T_IDXS)]
    sm['modelV2'].action.desiredAcceleration = 0.5
    sm['controlsState'].longControlState = LongCtrlState.pid
    sm.update({k: getattr(m, k) for k, m in msgs.items()})
    return sm

  def _drive(self, throttle_only, live_map, seconds=8., **kw):
    p = self._planner(throttle_only)
    v = 20.7
    out = []
    for _ in range(int(seconds / DT_MDL)):
      p.update(self._sm(v, live_map, **kw))
      a = max(float(p.output_a_target), -0.4)  # throttle-only plant: coast decel at most ~0.4 m/s^2
      v += a * DT_MDL
      out.append((float(p.output_a_target), 'sccMap' if p.source == LongitudinalPlanSource.sccMap else str(p.source), v,
                  p.scc.guard_active))
    return out

  def _baseline(self, seconds):
    # control: same scenario with no map path at all (what the planner does without SCC)
    self.params.put("MapTargetVelocities", "[]", block=True)
    return self._drive(True, matched_map(), seconds=seconds)

  def test_phantom_without_guard_coasts_whole_time(self):
    put_path(self.params, 3., RAMP_PATH)
    out = self._drive(False, matched_map())
    late = out[int(3. / DT_MDL):]
    self.assertTrue(all(src == 'sccMap' for _, src, _, _ in late))
    self.assertLess(max(a for a, *_ in late), -0.3)  # the 12f @698 failure: held coast on the merge

  def test_phantom_with_guard_released_after_2s(self):
    put_path(self.params, 3., RAMP_PATH)
    out = self._drive(True, matched_map())
    i_rel = next(i for i, o in enumerate(out) if o[3])
    t_rel = i_rel * DT_MDL
    self.assertAlmostEqual(t_rel, fscc.GUARD_UNCORROBORATED_S, delta=0.3)
    self.assertTrue(all(o[3] for o in out[i_rel:]))  # stays released (no sawtooth)
    i_late = i_rel + int(round(1.5 / DT_MDL))
    late = out[i_late:]
    self.assertTrue(all(src != 'sccMap' for _, src, _, _ in late))
    self.assertGreater(min(a for a, *_ in late), -0.05)  # no longer coasting
    # and the plan recovered to what it is without SCC (the coast already taken only lowers v a little)
    base = self._baseline(8.)
    self.assertGreater(min(a for a, *_ in late), min(a for a, *_ in base[i_late:]) - 0.05)
    self.assertLess(out[i_rel][2], 20.7)  # it did coast for the 2 s

  def test_real_curve_not_released(self):
    # same map target, but the model sees the curve (12f @1349: predicted lat >= 1.3 throughout): keep slowing
    put_path(self.params, 3., RAMP_PATH)
    out = self._drive(True, matched_map(), pred_lat=1.8)
    self.assertFalse(any(o[3] for o in out))
    self.assertLess(max(a for a, *_ in out[int(3. / DT_MDL):]), -0.3)

  def test_unmatched_ramp_never_binds(self):
    # 12f @698 as logged: mapd unmatched -> SCC-M ignored outright, no coast at all, guard not needed
    put_path(self.params, 3., RAMP_PATH)
    out = self._drive(True, messaging.new_message('liveMapDataSP').liveMapDataSP, seconds=4.)
    self.assertTrue(all(src != 'sccMap' for _, src, _, _ in out))
    self.assertFalse(any(o[3] for o in out))
    base = self._baseline(4.)
    self.assertEqual([round(a, 6) for a, *_ in out], [round(a, 6) for a, *_ in base])  # identical to no SCC path
