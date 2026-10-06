"""
Tests for fork/cruise_prefs.py (fork #30: never delete the owner's cruise prefs when
openpilot longitudinal is transiently unavailable).

Simulates one ignition with longitudinal unavailable, then one with it available, and checks
the five cruise preferences (ExperimentalMode, DynamicExperimentalControl,
CustomAccIncrementsEnabled, SmartCruiseControlVision, SmartCruiseControlMap) survive every
upstream cleanup path, that the UI toggles show the stored value, and that the runtime
consumers stay inert while unavailable.

car-features/keep-cruise-prefs-mutation.py mutates each guard and checks this file fails.
"""
import importlib
import inspect
import sys
import types

from opendbc.car import structs
from openpilot.cereal import custom, messaging
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.fork import cruise_prefs as cp_mod

KEYS = ("ExperimentalMode", "DynamicExperimentalControl", "CustomAccIncrementsEnabled",
        "SmartCruiseControlVision", "SmartCruiseControlMap")
STORE = dict.fromkeys(KEYS, True)  # the owner's stored (non-default) values


def store_all(params):
  for k, v in STORE.items():
    params.put_bool(k, v, block=True)


def read_all(params):
  return {k: params.get_bool(k) for k in KEYS}


# --- fakes (no real CarParamsSP capnp object needed) -------------------------------------------------

class CPSP:
  def __init__(self, pcm_cruise_speed=True, icbm_available=False):
    self.pcmCruiseSpeed = pcm_cruise_speed
    self.intelligentCruiseButtonManagementAvailable = icbm_available


def make_cp(openpilot_long: bool):
  cp = structs.CarParams()
  cp.openpilotLongitudinalControl = openpilot_long
  cp.pcmCruise = not openpilot_long
  cp.steerControlType = structs.CarParams.SteerControlType.torque
  cp.steerRatio = 15.0
  cp.wheelbase = 2.7
  cp.longitudinalActuatorDelay = 0.2
  return cp


class UIStateShim:
  """Just the attributes UIStateSP._enforce_constraints touches (runs the real method)."""

  def __init__(self, params, cp, cp_sp, has_long, has_icbm=False):
    self.params = params
    self.CP = cp
    self.CP_SP = cp_sp
    self.has_longitudinal_control = has_long
    self.has_icbm = has_icbm


def run_interface_cleanup(params, openpilot_long: bool):
  from openpilot.sunnypilot.selfdrive.car import interfaces as sp_interfaces
  sp_interfaces._cleanup_unsupported_params(make_cp(openpilot_long), CPSP(pcm_cruise_speed=not openpilot_long), params)


def run_ui_state_constraints(params, openpilot_long: bool):
  from openpilot.selfdrive.ui.sunnypilot.ui_state import UIStateSP
  shim = UIStateShim(params, make_cp(openpilot_long), CPSP(pcm_cruise_speed=not openpilot_long), has_long=openpilot_long)
  UIStateSP._enforce_constraints(shim)


CLEANUP_PATHS = {
  "interfaces._cleanup_unsupported_params": run_interface_cleanup,
  "UIStateSP._enforce_constraints": run_ui_state_constraints,
}


class TestPredicate(OpenpilotTestCase):
  def test_all_five_are_preserved(self):
    for k in KEYS:
      self.assertTrue(cp_mod.preserved(k), k)

  def test_other_keys_are_not_preserved(self):
    for k in ("IntelligentCruiseButtonManagement", "AlphaLongitudinalEnabled", "NeuralNetworkLateralControl"):
      self.assertFalse(cp_mod.preserved(k), k)

  def test_key_list_is_exactly_the_five(self):
    self.assertEqual(tuple(cp_mod.CRUISE_PREF_KEYS), KEYS)


class TestRemoveUnlessPreserved(OpenpilotTestCase):
  def test_preserved_key_is_not_removed(self):
    from openpilot.common.params import Params
    p = Params()
    store_all(p)
    for k in KEYS:
      cp_mod.remove_unless_preserved(p, k)
    self.assertEqual({k: p.get_bool(k) for k in KEYS}, STORE)

  def test_non_preserved_key_IS_removed(self):
    # the wrapper must still clean up everything else (kills a "never removes" mutant)
    from openpilot.common.params import Params
    p = Params()
    p.put_bool("NeuralNetworkLateralControl", True, block=True)
    cp_mod.remove_unless_preserved(p, "NeuralNetworkLateralControl")
    self.assertFalse(p.get_bool("NeuralNetworkLateralControl"))

  def test_fake_params_sees_the_delegate(self):
    seen = []
    class FakeParams:
      def remove(self, key):
        seen.append(key)
    cp_mod.remove_unless_preserved(FakeParams(), "DynamicExperimentalControl")
    cp_mod.remove_unless_preserved(FakeParams(), "IntelligentCruiseButtonManagement")
    self.assertEqual(seen, ["IntelligentCruiseButtonManagement"])


class TestCleanupKeepsPrefs(OpenpilotTestCase):
  """One ignition with long unavailable, then one with long available: the five survive both."""

  def test_every_cleanup_path_keeps_all_five(self):
    from openpilot.common.params import Params
    for name, run in CLEANUP_PATHS.items():
      for openpilot_long in (False, True):
        p = Params()
        store_all(p)
        run(p, openpilot_long)
        self.assertEqual(read_all(p), STORE, f"{name} long={openpilot_long}")
        # ... and the stored increment VALUES (the 5 mph the owner had) survive too
        p.put("CustomAccShortPressIncrement", 5, block=True)
        run(p, openpilot_long)
        self.assertEqual(p.get("CustomAccShortPressIncrement", return_default=True), 5)

  def test_cleanup_still_removes_other_unsupported_params(self):
    # positive control: the cleanup paths are not made inert, only the five keys are kept
    from openpilot.common.params import Params
    from openpilot.selfdrive.ui.sunnypilot.ui_state import UIStateSP
    p = Params()
    store_all(p)
    p.put_bool("AlphaLongitudinalEnabled", True, block=True)
    cp = make_cp(False)
    cp.alphaLongitudinalAvailable = False
    shim = UIStateShim(p, cp, CPSP(False), has_long=False)
    UIStateSP._enforce_constraints(shim)
    self.assertFalse(p.get_bool("AlphaLongitudinalEnabled"))


class TestUpstreamGatePins(OpenpilotTestCase):
  """The fix is only safe because the consumers stay gated. Pin each upstream gate."""

  def _src(self, mod_path):
    mod = importlib.import_module(mod_path)
    return inspect.getsource(mod)

  def test_controlsd_long_active_excludes_unavailable_car(self):
    src = self._src("openpilot.selfdrive.controls.controlsd")
    self.assertIn("self.CP.openpilotLongitudinalControl or not self.CP_SP.pcmCruiseSpeed", src)

  def test_card_and_selfdrived_gate_experimental_mode_on_longitudinal(self):
    card = self._src("openpilot.selfdrive.car.card")
    sd = self._src("openpilot.selfdrive.selfdrived.selfdrived")
    self.assertIn('self.experimental_mode = self.params.get_bool("ExperimentalMode") and self.CP.openpilotLongitudinalControl', card)
    self.assertIn('self.experimental_mode = self.params.get_bool("ExperimentalMode") and self.CP.openpilotLongitudinalControl', sd)

  def test_cruise_custom_increments_only_in_the_non_pcm_path(self):
    src = self._src("openpilot.selfdrive.car.cruise")
    self.assertIn("if not self.CP.pcmCruise or (not self.CP_SP.pcmCruiseSpeed and _enabled):", src)

  def test_all_six_cleanup_sites_use_the_guard(self):
    sites = {
      "openpilot.sunnypilot.selfdrive.car.interfaces": ["remove_unless_preserved"],
      "openpilot.selfdrive.ui.sunnypilot.ui_state": ["remove_unless_preserved"],
      "openpilot.selfdrive.ui.sunnypilot.layouts.settings.cruise": ["sync_toggle"],
      "openpilot.selfdrive.selfdrived.selfdrived": ["remove_unless_preserved"],
      "openpilot.selfdrive.ui.layouts.settings.toggles": ["remove_unless_preserved"],
      "openpilot.selfdrive.ui.mici.layouts.settings.toggles": ["remove_unless_preserved"],
    }
    for mod_path, needles in sites.items():
      src = self._src(mod_path)
      for needle in needles:
        self.assertIn(needle, src, mod_path)
      # and no bare deletion of one of the five keys is left anywhere in the file
      for k in KEYS:
        self.assertNotIn(f'params.remove("{k}")', src, f"{mod_path}: {k}")


# --- UI: the toggles must show the STORED value while unavailable ------------------------------------

def _load_fonts():
  from openpilot.system.ui.lib.application import gui_app
  try:
    if not gui_app._fonts:
      gui_app._load_fonts()
  except Exception:
    return None
  return gui_app


def _fake_ui_state_module(params, cp, cp_sp, has_long, has_icbm=False, engaged=False, offroad=False):
  mod = types.ModuleType("openpilot.selfdrive.ui.ui_state")

  class FakeUI:
    def __init__(self):
      self.params = params
      self.CP = cp
      self.CP_SP = cp_sp
      self.has_longitudinal_control = has_long
      self.has_icbm = has_icbm
      self.engaged = engaged
      self.started = False
      self.is_release = False
      self.personality = 1

    def is_offroad(self):
      return offroad

    def update_params(self):
      pass

    def add_engaged_transition_callback(self, cb):
      pass

  mod.ui_state = FakeUI()
  return mod


class TestUiShowsStoredValue(OpenpilotTestCase):
  """With long unavailable, the toggles are disabled but display the stored value."""

  def setUp(self):
    if _load_fonts() is None:
      self.skipTest("raylib fonts unavailable headless")

  def _import_with_fake(self, mod_name, params, has_long, offroad=False):
    fake = _fake_ui_state_module(params, make_cp(has_long), CPSP(False), has_long, offroad=offroad)
    real = sys.modules.get("openpilot.selfdrive.ui.ui_state")
    sys.modules["openpilot.selfdrive.ui.ui_state"] = fake
    saved = {m: sys.modules.pop(m, None) for m in (
      mod_name, "openpilot.selfdrive.ui.sunnypilot.layouts.settings.cruise_sub_layouts.speed_limit_settings",
      "openpilot.selfdrive.ui.sunnypilot.layouts.settings.cruise_sub_layouts.speed_limit_policy",
      "openpilot.selfdrive.ui.layouts.settings.toggles",
      "openpilot.selfdrive.ui.mici.layouts.settings.toggles")}
    self.addCleanup(lambda: (sys.modules.__setitem__("openpilot.selfdrive.ui.ui_state", real) if real else None,
                             [sys.modules.__setitem__(k, v) for k, v in saved.items() if v is not None]))
    return importlib.import_module(mod_name), fake.ui_state

  def test_cruise_layout_shows_stored_values_while_greyed(self):
    from openpilot.common.params import Params
    p = Params()
    store_all(p)
    mod, ui = self._import_with_fake("openpilot.selfdrive.ui.sunnypilot.layouts.settings.cruise", p, has_long=False)
    layout = mod.CruiseLayout()
    # the fake ui_state is the object the module imported
    mod.ui_state.CP = ui.CP
    mod.ui_state.CP_SP = ui.CP_SP
    mod.ui_state.has_longitudinal_control = False
    mod.ui_state.has_icbm = False
    layout._update_state()

    for item, key in ((layout.custom_acc_toggle, "CustomAccIncrementsEnabled"),
                      (layout.dec_toggle, "DynamicExperimentalControl"),
                      (layout.scc_v_toggle, "SmartCruiseControlVision"),
                      (layout.scc_m_toggle, "SmartCruiseControlMap")):
      self.assertEqual(item.action_item.get_state(), p.get_bool(key), key)  # shown == stored
      self.assertTrue(item.action_item.get_state(), key)                    # really True, not clobbered
      self.assertFalse(item.action_item.enabled, key)                       # ... and greyed out
    # nothing got deleted
    self.assertEqual(read_all(p), STORE)

  def test_main_toggles_show_stored_experimental_mode(self):
    from openpilot.common.params import Params
    p = Params()
    store_all(p)
    mod, ui = self._import_with_fake("openpilot.selfdrive.ui.layouts.settings.toggles", p, has_long=False)
    mod.ui_state.CP = ui.CP
    mod.ui_state.has_longitudinal_control = False
    layout = mod.TogglesLayout()
    layout._update_toggles()
    self.assertTrue(layout._toggles["ExperimentalMode"].action_item.get_state())
    self.assertEqual(layout._toggles["ExperimentalMode"].action_item.get_state(), p.get_bool("ExperimentalMode"))
    self.assertFalse(layout._toggles["ExperimentalMode"].action_item.enabled)
    self.assertTrue(p.get_bool("ExperimentalMode"))

  def test_mici_toggle_shows_stored_experimental_mode(self):
    from openpilot.common.params import Params
    p = Params()
    store_all(p)
    mod, ui = self._import_with_fake("openpilot.selfdrive.ui.mici.layouts.settings.toggles", p, has_long=False)
    mod.ui_state.CP = ui.CP
    mod.ui_state.has_longitudinal_control = False
    layout = mod.TogglesLayoutMici()
    layout._update_toggles()
    self.assertTrue(layout._experimental_btn._checked)
    self.assertEqual(layout._experimental_btn._checked, p.get_bool("ExperimentalMode"))
    self.assertFalse(layout._experimental_btn.is_visible)
    self.assertTrue(p.get_bool("ExperimentalMode"))


# --- runtime consumers stay inert while long is unavailable ------------------------------------------

V_EGO = 20.0


def _planner_sm(experimental_mode: bool, long_active: bool, curve: bool = False):
  s = {}
  for svc in ("radarState", "controlsState", "vehicleParameters", "carStateSP", "liveMapDataSP",
              "gpsLocationExternal", "gpsLocation", "selfdriveStateSP"):
    s[svc] = getattr(messaging.new_message(svc), svc)
  cs = messaging.new_message('carState')
  cs.carState.vEgo = V_EGO
  cs.carState.vCruise = 100.
  cs.carState.vCruiseCluster = 100.
  s['carState'] = cs.carState.as_reader()
  sd = messaging.new_message('selfdriveState')
  sd.selfdriveState.experimentalMode = experimental_mode
  sd.selfdriveState.enabled = True
  s['selfdriveState'] = sd.selfdriveState.as_reader()
  cc = messaging.new_message('carControl')
  cc.carControl.enabled = long_active  # controlsd sets this False when long is unavailable
  s['carControl'] = cc.carControl.as_reader()
  rate = [0.5] * 33 if curve else [0.0] * 33
  m = messaging.new_message('modelV2')
  m.modelV2.orientationRate.z = rate
  m.modelV2.velocity.x = [V_EGO] * 33
  m.modelV2.position.x = [float(i) for i in range(33)]
  m.modelV2.action.desiredAcceleration = 0.0
  s['modelV2'] = m.modelV2.as_reader()
  return s


def _make_planner(openpilot_long: bool, pcm_cruise_speed: bool):
  from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
  CP_SP = custom.CarParamsSP(pcmCruiseSpeed=pcm_cruise_speed)
  planner = LongitudinalPlanner(make_cp(openpilot_long), CP_SP, init_v=V_EGO)
  return planner


class TestConsumersInert(OpenpilotTestCase):
  def setUp(self):
    from openpilot.common.params import Params
    p = Params()
    store_all(p)  # every prefs toggle ON

  def test_dec_and_e2e_inert_when_longitudinal_unavailable(self):
    planner = _make_planner(openpilot_long=False, pcm_cruise_speed=True)
    # selfdrived/card produce experimentalMode False when long is unavailable, even with the param True
    sm = _planner_sm(experimental_mode=False, long_active=False)
    for _ in range(5):
      planner.update(sm)
    self.assertFalse(planner.dec.active())
    self.assertFalse(planner.is_e2e(sm))
    self.assertNotEqual(str(planner.mpc.source), "e2e")

  def test_scc_targets_inert_when_longitudinal_unavailable(self):
    planner = _make_planner(openpilot_long=False, pcm_cruise_speed=True)
    # a strong curve that WOULD drop the target if SCC vision were live
    sm = _planner_sm(experimental_mode=False, long_active=False, curve=True)
    for _ in range(30):
      planner.update(sm)
    self.assertFalse(planner.scc.vision.is_active)
    self.assertFalse(planner.scc.map.is_active)
    self.assertEqual(planner.scc.vision.output_v_target, 255)  # V_CRUISE_UNSET
    # the arbitrated SP target is the cruise speed, not the 5.56 m/s SCC curve target
    self.assertGreater(planner.output_v_target, 20.)

  def test_consumers_live_again_when_longitudinal_returns(self):
    # phase 2 sanity: the same setup with long active does engage the SCC consumer
    planner = _make_planner(openpilot_long=True, pcm_cruise_speed=False)
    sm = _planner_sm(experimental_mode=False, long_active=True, curve=True)
    for _ in range(30):
      planner.update(sm)
    self.assertTrue(planner.scc.vision.is_active)

  def test_custom_increments_inert_on_pcm_cruise_speed_car(self):
    from openpilot.selfdrive.car.cruise import VCruiseHelper
    from opendbc.car import structs as car_structs
    from openpilot.common.constants import CV
    CP = car_structs.CarParams()
    CP.pcmCruise = True
    CP_SP = custom.CarParamsSP(pcmCruiseSpeed=True)
    helper = VCruiseHelper(CP, CP_SP)
    # a stored custom increment the owner set (5 mph), with the enable param ON
    from openpilot.common.params import Params
    p = Params()
    p.put_bool("CustomAccIncrementsEnabled", True, block=True)
    p.put("CustomAccShortPressIncrement", 5, block=True)
    helper.read_custom_set_speed_params()

    CS = car_structs.CarState(cruiseState={"available": True, "speed": 50 * CV.KPH_TO_MS})
    helper.update_v_cruise(CS, enabled=True, is_metric=False)
    # PCM-car path: set speed mirrors the stock ACC, the custom increment never applies
    self.assertAlmostEqual(helper.v_cruise_kph, 50, delta=0.2)
