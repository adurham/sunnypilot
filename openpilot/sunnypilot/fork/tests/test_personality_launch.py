"""
Personality-indexed launch feel (adurham fork): the driver's LongitudinalPersonality scales the two driver-felt planner
levers — the cruise-candidate ceiling (``A_CRUISE_MAX`` via ``get_max_accel``) — and is carried to the car controller on
``CarControlSP.personality``. The Hyundai pedal law (launch ceiling / pull gain / interceptor cap) is covered by the
opendbc patch's test_gas_interceptor_personality.py.

STANDARD reproduces upstream bit-for-bit; relaxed is gentler; aggressive pulls harder (relaxed < standard < aggressive).
"""
import inspect
from types import SimpleNamespace

import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib import longitudinal_planner as upstream_lp
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.car.helpers import convert_carControlSP
from opendbc.car import structs
from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt

P = log.LongitudinalPersonality
PERSONALITIES = (P.relaxed, P.standard, P.aggressive)
MPH = 0.44704


def capnp_personality(p):
  """selfdriveState.personality as the planner really sees it (a capnp _DynamicEnum, not an int)."""
  sd = messaging.new_message('selfdriveState').selfdriveState
  sd.personality = p
  return sd.personality


class TestCruiseCeilingScale(OpenpilotTestCase):
  def test_tiers_ordered_and_standard_is_upstream(self):
    for v in np.arange(0., 45., 0.5):
      r = upstream_lp.get_max_accel(v, P.relaxed)
      s = upstream_lp.get_max_accel(v, P.standard)
      a = upstream_lp.get_max_accel(v, P.aggressive)
      self.assertLess(r, s, v)
      self.assertLess(s, a, v)
      # standard == upstream table exactly
      self.assertEqual(s, float(np.interp(v, upstream_lp.A_CRUISE_MAX_BP, upstream_lp.A_CRUISE_MAX_VALS)))

  def test_capnp_enum_personality_is_honoured(self):
    # a capnp _DynamicEnum == int but does not hash like it; the mapping must still resolve
    for p in PERSONALITIES:
      self.assertEqual(upstream_lp.get_max_accel(25., capnp_personality(p)), upstream_lp.get_max_accel(25., p))
    self.assertNotEqual(upstream_lp.get_max_accel(25., P.relaxed), upstream_lp.get_max_accel(25., P.standard))

  def test_unknown_and_missing_fall_back_to_standard(self):
    for bad in (7, -1, None, 'bogus'):
      self.assertEqual(upstream_lp.personality_scale(bad), upstream_lp.personality_scale(P.standard))
    # a direct call with no personality (the sim / any caller that forgets) is STANDARD, never aggressive
    for v in (0., 5., 10., 25., 40.):
      self.assertEqual(upstream_lp.get_max_accel(v), upstream_lp.get_max_accel(v, P.standard))

  def test_get_cruise_accel_personality_signature_and_scale(self):
    self.assertEqual(list(inspect.signature(upstream_lp.get_cruise_accel).parameters)[-1], 'personality')
    CP = SimpleNamespace(steerRatio=15., wheelbase=2.7)
    # cruise-bound (big error), ACC mode: the ceiling is the personality-scaled A_CRUISE_MAX
    for v in (10., 20., 30.):
      std = upstream_lp.get_cruise_accel(False, v + 20., v, 0., 0., CP, 100., 0., True, P.standard)
      agg = upstream_lp.get_cruise_accel(False, v + 20., v, 0., 0., CP, 100., 0., True, P.aggressive)
      rel = upstream_lp.get_cruise_accel(False, v + 20., v, 0., 0., CP, 100., 0., True, P.relaxed)
      self.assertLess(rel, std, v)
      self.assertLess(std, agg, v)
      self.assertAlmostEqual(std, float(np.interp(v, upstream_lp.A_CRUISE_MAX_BP, upstream_lp.A_CRUISE_MAX_VALS)), places=6)


class TestCapnpPlumbing(OpenpilotTestCase):
  def test_capnp_field_round_trips(self):
    m = messaging.new_message('carControlSP')
    # capnp primitive default is 0 = aggressive, so the producer ALWAYS writes the field (controlsd_ext) and the opendbc
    # mirror defaults to standard; consumers clamp unknown to standard.
    self.assertEqual(int(m.carControlSP.personality), 0)
    for p in PERSONALITIES:
      m.carControlSP.personality = int(p)
      self.assertEqual(int(m.carControlSP.personality), int(p))

  def test_convert_carcontrolsp_carries_personality(self):
    m = messaging.new_message('carControlSP')
    m.carControlSP.personality = int(P.aggressive)
    sp = convert_carControlSP(m.carControlSP)
    self.assertEqual(sp.personality, int(P.aggressive))
    # opendbc dataclass default (no-arg construction, e.g. tests / other brands) is STANDARD, never aggressive
    self.assertEqual(structs.CarControlSP().personality, int(P.standard))

  def test_state_control_ext_copies_live_personality(self):
    class _SM(dict):
      def __init__(self, d, seen):
        super().__init__(d)
        self.seen = {'selfdriveState': seen}  # SubMaster.seen is a per-service dict

    def make_sm(personality, seen=True):
      sd = messaging.new_message('selfdriveState').selfdriveState
      sd.personality = personality
      rs = messaging.new_message('radarState').radarState
      return _SM({'selfdriveState': sd, 'radarState': rs,
                  'selfdriveStateSP': messaging.new_message('selfdriveStateSP').selfdriveStateSP}, seen)

    for p in PERSONALITIES:
      inst = ControlsExt.__new__(ControlsExt)  # get_lead_data is a staticmethod; no init state needed
      cc_sp = ControlsExt.state_control_ext(inst, make_sm(p))
      self.assertEqual(int(cc_sp.personality), int(p))
    # a not-yet-seen selfdriveState must NOT silently read as 0 (aggressive)
    inst = ControlsExt.__new__(ControlsExt)
    cc_sp = ControlsExt.state_control_ext(inst, make_sm(P.aggressive, seen=False))
    self.assertEqual(int(cc_sp.personality), int(P.standard))


# ---- end-to-end through the REAL LongitudinalPlanner: the ceiling really moves a_cruise by personality ----

def planner_sm(v_ego, v_cruise_kph, personality=P.standard, experimental=False, override=False):
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
  ctl.longControlState = LongCtrlState.pid
  sd = messaging.new_message('selfdriveState').selfdriveState
  sd.enabled = True
  sd.experimentalMode = experimental
  sd.personality = personality
  rs = messaging.new_message('radarState').radarState
  sm = {'modelV2': md, 'carState': cs, 'carControl': cc, 'controlsState': ctl, 'selfdriveState': sd, 'radarState': rs}
  for w in ('liveMapDataSP', 'vehicleParameters', 'carStateSP', 'gpsLocation', 'gpsLocationExternal', 'selfdriveStateSP'):
    sm[w] = getattr(messaging.new_message(w), w)
  return sm


def make_planner():
  from opendbc.car.honda.interface import CarInterface
  from opendbc.car.honda.values import CAR
  CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
  CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.HONDA_CIVIC)
  return upstream_lp.LongitudinalPlanner(CP, CP_SP, init_v=20., init_a=0.)


class TestEndToEndCruiseCeiling(OpenpilotTestCase):
  def _run(self, planner, n, v_ego, v_cruise_kph, **kw):
    for _ in range(n):
      planner.update(planner_sm(v_ego, v_cruise_kph, **kw))

  def test_aggressive_chases_harder_than_relaxed(self):
    # +20 mph at 25 mph (11.18 m/s) with the ease bypassed: the cruise candidate is ceiling-bound (error +9 m/s >> the
    # ceiling), so a_cruise IS the personality ceiling. Aggressive > standard > relaxed, and standard == the upstream one.
    v = 25. * MPH
    kph = 45. * 1.609344
    out = {}
    for p in PERSONALITIES:
      planner = make_planner()
      planner.setspeed_ease.update = lambda sm, v_target, *a, **k: v_target  # isolate the A_CRUISE_MAX lever
      self._run(planner, 5, v, 25. * 1.609344, personality=p)
      self._run(planner, 60, v, kph, personality=p)
      out[p] = planner.a_cruise
    self.assertLess(out[P.relaxed], out[P.standard])
    self.assertLess(out[P.standard], out[P.aggressive])
    self.assertAlmostEqual(out[P.standard], float(np.interp(v, upstream_lp.A_CRUISE_MAX_BP, upstream_lp.A_CRUISE_MAX_VALS)), places=3)
    self.assertAlmostEqual(out[P.aggressive], upstream_lp.get_max_accel(v, P.aggressive), places=3)
    self.assertAlmostEqual(out[P.relaxed], upstream_lp.get_max_accel(v, P.relaxed), places=3)
