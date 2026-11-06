"""fork (0044): the PURE visibility/enable rules behind the Hyundai FCA11 settings toggles (fork/fca11_settings.py).

The raylib panel cannot run headless; this pins the decision it delegates to, plus a static check that the panel
actually wires the shaping toggle to HyundaiFca11AffineGain through these functions."""
import ast
import itertools
import json
import os
import re
import subprocess
import sys
import unittest
from pathlib import Path

from openpilot.sunnypilot.fork import fca11_settings
from openpilot.sunnypilot.fork.fca11_settings import ToggleState, fca11_brake_toggle_state, fca11_shaping_toggle_state, \
  fca11_shadow_toggle_state, fca11_dynamic_release_toggle_state, pedal_delivery_toggle_state, stop_approach_toggle_state, \
  gas_rise_lag_toggle_state, fca11_stop_hold_toggle_state

REPO = Path(__file__).resolve().parents[4]
PANEL = REPO / "openpilot" / "selfdrive" / "ui" / "sunnypilot" / "layouts" / "settings" / "vehicle" / "brands" / "hyundai.py"


def _row_rule(name):
  """The settings-tidy row rules are the subject of this file. On a tree that predates them the row is listed
  unconditionally; the tests below must FAIL BY VALUE there (the row is visible) and never by ImportError."""
  return getattr(fca11_settings, name, None)


ON_VALUES = (True, "1", b"1", 1)
OFF_VALUES = (False, "0", b"0", None, "", 0, "junk")


class TestFca11SettingsRules(unittest.TestCase):

  def test_brake_toggle(self):
    for pedal, offroad in itertools.product(ON_VALUES + OFF_VALUES, (True, False)):
      on = pedal in ON_VALUES
      self.assertEqual(ToggleState(visible=on, enabled=on and offroad), fca11_brake_toggle_state(pedal, offroad), (pedal, offroad))

  def test_shaping_toggle_full_truth_table(self):
    """visible <=> pedal on AND braking on; enabled <=> visible AND offroad (never settable onroad)."""
    for pedal, brake, offroad in itertools.product(ON_VALUES + OFF_VALUES, ON_VALUES + OFF_VALUES, (True, False)):
      vis = pedal in ON_VALUES and brake in ON_VALUES
      self.assertEqual(ToggleState(visible=vis, enabled=vis and offroad), fca11_shaping_toggle_state(pedal, brake, offroad),
                       (pedal, brake, offroad))

  def test_shaping_hidden_without_braking_even_offroad(self):
    self.assertEqual(ToggleState(False, False), fca11_shaping_toggle_state(True, False, True))
    self.assertEqual(ToggleState(False, False), fca11_shaping_toggle_state(False, True, True))
    self.assertEqual(ToggleState(True, False), fca11_shaping_toggle_state(True, True, False))
    self.assertEqual(ToggleState(True, True), fca11_shaping_toggle_state(True, True, True))

  def test_shadow_toggle_full_truth_table(self):
    """0045: visible <=> pedal on (independent of braking); enabled <=> visible AND offroad (never settable onroad)."""
    for pedal, offroad in itertools.product(ON_VALUES + OFF_VALUES, (True, False)):
      vis = pedal in ON_VALUES
      self.assertEqual(ToggleState(visible=vis, enabled=vis and offroad), fca11_shadow_toggle_state(pedal, offroad),
                       (pedal, offroad))
    self.assertEqual(ToggleState(True, False), fca11_shadow_toggle_state(True, False))

  def test_pedal_delivery_toggle_full_truth_table(self):
    """17c: visible <=> pedal on (it lives in the pedal path); enabled <=> visible AND offroad."""
    for pedal, offroad in itertools.product(ON_VALUES + OFF_VALUES, (True, False)):
      vis = pedal in ON_VALUES
      self.assertEqual(ToggleState(visible=vis, enabled=vis and offroad), pedal_delivery_toggle_state(pedal, offroad),
                       (pedal, offroad))

  def test_dynamic_release_toggle_full_truth_table(self):
    """18x: visible <=> pedal on AND braking on (it is a property of the FCA11 law); enabled <=> visible AND offroad."""
    for pedal, brake, offroad in itertools.product(ON_VALUES + OFF_VALUES, ON_VALUES + OFF_VALUES, (True, False)):
      vis = pedal in ON_VALUES and brake in ON_VALUES
      self.assertEqual(ToggleState(visible=vis, enabled=vis and offroad),
                       fca11_dynamic_release_toggle_state(pedal, brake, offroad), (pedal, brake, offroad))

  def test_stop_approach_toggle(self):
    """17c lane C: always listed (planner-side, no pedal dependency); settable offroad only."""
    self.assertEqual(ToggleState(True, True), stop_approach_toggle_state(True))
    self.assertEqual(ToggleState(True, False), stop_approach_toggle_state(False))

  def test_longitudinal_tuning_toggle_full_truth_table(self):
    """SETTINGS TIDY: visible <=> pedal OFF (the row is inert with the pedal on); enabled <=> visible AND offroad."""
    rule = _row_rule("longitudinal_tuning_toggle_state")
    self.assertIsNotNone(rule, "no visibility rule for HyundaiLongitudinalTuning: the row is listed unconditionally")
    for pedal, offroad in itertools.product(ON_VALUES + OFF_VALUES, (True, False)):
      vis = pedal not in ON_VALUES
      self.assertEqual(ToggleState(visible=vis, enabled=vis and offroad), rule(pedal, offroad), (pedal, offroad))

  def test_longitudinal_tuning_row_is_hidden_on_this_car(self):
    """The car's real value is HyundaiGasInterceptor=1, so the row must NOT be listed - offroad or onroad."""
    rule = _row_rule("longitudinal_tuning_toggle_state")
    self.assertIsNotNone(rule, "no visibility rule for HyundaiLongitudinalTuning: the row is listed unconditionally")
    self.assertEqual(ToggleState(False, False), rule("1", True))
    self.assertEqual(ToggleState(False, False), rule("1", False))
    # ... and it comes back, unchanged, on a car without the pedal (the key is upstream and shared).
    self.assertEqual(ToggleState(True, True), rule("0", True))

  def test_longitudinal_tuning_key_is_never_deleted_or_re_defaulted(self):
    """It is an UPSTREAM sunnypilot key (present in the sync target): hiding the row must not touch registry or value."""
    keys_src = (REPO / "openpilot" / "common" / "params_keys.h").read_text(encoding="utf-8")
    self.assertRegex(keys_src, r'\{"HyundaiLongitudinalTuning", \{PERSISTENT \| BACKUP, INT, "0"\}\}')

  def test_the_cars_real_param_combination(self):
    """Every row decided with the car's own values (gas interceptor 1, Brake 1, Shaping 1, Dynamic 1, Shadow 0,
    PedalDeliveryLaw 1, GasRiseLag 1, StopHoldClutch 0): the inert rows are hidden, every live row is still listed."""
    pedal, brake = "1", "1"
    tuning = _row_rule("longitudinal_tuning_toggle_state")
    self.assertIsNotNone(tuning, "no visibility rule for HyundaiLongitudinalTuning: the row is listed unconditionally")
    rows = {
      "FCA11 Braking": fca11_brake_toggle_state(pedal, True),
      "FCA11 Brake Shaping": fca11_shaping_toggle_state(pedal, brake, True),
      "FCA11 Shadow": fca11_shadow_toggle_state(pedal, True),
      "Pedal Delivery Law": pedal_delivery_toggle_state(pedal, True),
      "Gas Rise Lag": gas_rise_lag_toggle_state(pedal, True),
      "Dynamic Brakes": fca11_dynamic_release_toggle_state(pedal, brake, True),
      "Stop-Approach Commitment": stop_approach_toggle_state(True),
      "Custom Longitudinal Tuning": tuning(pedal, True),
      "Hold Clutch Release (experimental)": fca11_stop_hold_toggle_state(pedal, brake, True),
    }
    hidden = {title for title, state in rows.items() if not state.visible}
    self.assertEqual({"Custom Longitudinal Tuning", "Hold Clutch Release (experimental)"}, hidden)

  def test_stop_hold_row_is_retired_hidden(self):
    """SETTINGS TIDY: the 0034 row is hidden on every car - the ESC hold has never been validated (FORK.md row 137)
    and the TCU opens the clutch on the brake pedal switch, not on 0x38D (lane-U report, summarised in row 140). The
    rule keeps the pedal/brake arguments so the row comes back by reverting it."""
    for pedal, brake, offroad in itertools.product(ON_VALUES + OFF_VALUES, ON_VALUES + OFF_VALUES, (True, False)):
      self.assertEqual(ToggleState(False, False), fca11_stop_hold_toggle_state(pedal, brake, offroad), (pedal, brake, offroad))

  def test_stop_hold_key_is_never_deleted_or_re_defaulted(self):
    """Hidden, not removed: the registry entry, its default and the car's stored value (0) stay exactly as they are."""
    keys_src = (REPO / "openpilot" / "common" / "params_keys.h").read_text(encoding="utf-8")
    self.assertRegex(keys_src, r'\{"HyundaiFca11StopHoldClutch", \{PERSISTENT \| BACKUP, BOOL, "0"\}\}')


class TestPanelWiring(unittest.TestCase):
  """Static: the panel builds the shaping toggle on the right key, lists it in items, and drives it from the pure rule."""

  @classmethod
  def setUpClass(cls):
    cls.src = PANEL.read_text(encoding="utf-8")

  def test_shaping_toggle_on_the_param(self):
    self.assertRegex(self.src, r'toggle_item_sp\(tr\("FCA11 Brake Shaping \(Beta\)"\),\s*"",\s*param="HyundaiFca11AffineGain"')

  def test_shaping_toggle_in_items(self):
    m = re.search(r"self\.items = \[(.*?)\]", self.src, re.S)
    self.assertIsNotNone(m)
    self.assertIn("self.fca11_shaping_toggle", m.group(1))

  def test_panel_uses_the_pure_rules(self):
    self.assertIn("fca11_shaping_toggle_state(pedal_raw, brake_raw, offroad)", self.src)
    self.assertIn("self.fca11_shaping_toggle.set_visible(shaping_state.visible)", self.src)
    self.assertIn("self.fca11_shaping_toggle.action_item.set_enabled(shaping_state.enabled)", self.src)

  def test_descriptions(self):
    self.assertIn("Takes effect", self.src)
    self.assertIn("at the next ignition", self.src)
    self.assertIn("Requires FCA11 Braking", self.src)
    # SETTINGS TIDY (D2): 0037 deleted the continuous law's fast-step bypass (FORK row 135), so the old
    # "Hard stops are NOT slowed ... full rate" claim is FALSE on the law the car runs. It must not come back.
    self.assertNotIn("Hard stops are NOT", self.src)
    for var in ("shaping_desc", "dynamic_desc", "shadow_desc"):
      self.assertNotIn("full rate", self._description(var), var)
    # 0044: the stale pre-0040 claim is gone (both speed floors were deleted in 0040)
    self.assertNotIn("below 12 km/h", self.src)

  def test_shadow_toggle_wiring(self):
    """0045: the Shadow toggle is on HyundaiFca11Shadow, in items, driven by the pure rule, honest description."""
    self.assertRegex(self.src, r'toggle_item_sp\(tr\("FCA11 Shadow \(Log-Only\)"\),\s*"",\s*param="HyundaiFca11Shadow"')
    m = re.search(r"self\.items = \[(.*?)\]", self.src, re.S)
    self.assertIn("self.fca11_shadow_toggle", m.group(1))
    self.assertIn("fca11_shadow_toggle_state(pedal_raw, offroad)", self.src)
    self.assertIn("self.fca11_shadow_toggle.set_visible(shadow_state.visible)", self.src)
    self.assertIn("self.fca11_shadow_toggle.action_item.set_enabled(shadow_state.enabled)", self.src)
    self.assertIn("never sent to the car", self._description("shadow_desc"))
    self.assertIn("ON switches braking back to the plain OLD law", self._description("shadow_desc"))
    # SETTINGS TIDY (D3): the logged core is the 0043 SHAPED law, not "the new law (V3)".
    self.assertIn("logs the 0043 shaped law", self._description("shadow_desc"))
    self.assertNotIn("(V3)", self.src)
    self.assertNotIn("you brake for every stop", self.src)   # the first build's FCA11-DISABLED reading is gone

  def _description(self, var):
    """The whole tr(...) string of a panel description (`shaping_desc`, `dynamic_desc`, `shadow_desc`)."""
    blocks = re.findall(var + r" = tr\((.*?)\)\n", self.src, re.S)
    self.assertTrue(blocks, f"no {var} description in the panel")

    def join(node):
      if isinstance(node, ast.Constant):
        return node.value
      if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Add):
        return join(node.left) + join(node.right)
      raise AssertionError(f"unexpected description node: {type(node).__name__}")

    return join(ast.parse("(" + blocks[0] + ")", mode="eval").body)

  def test_brake_law_descriptions_state_the_four_states(self):
    """SETTINGS TIDY (D1): the brake law has FOUR states, not two (measured with the real setup_interfaces +
    Fca11LongBrake): Shaping ON + Dynamic ON + Shadow OFF is the continuous law; Shaping ON + Dynamic OFF is the 0043
    shaped law; Shaping OFF + Dynamic ON is the OLD law + the 18x ramp; Shadow ON (or both OFF) is the plain OLD law.
    Each of the three rows must state all four and keep the next-ignition sentence. (The value check - the text
    against the REAL controller's flags for every combination - is TestBrakeLawMatrix below.)"""
    for var in ("shaping_desc", "dynamic_desc", "shadow_desc"):
      block = self._description(var)
      self.assertIn("continuous linear law", block, var)
      # R1 (N2): the 0043 and 18x sentences say the Shadow condition they need - Shadow ON overrides both, so
      # "Shaping ON with Dynamic Brakes OFF gives 0043" is only true with Shadow OFF (101/011 really run the OLD law).
      self.assertIn("Shaping ON with Dynamic Brakes OFF and Shadow OFF gives the 0043 shaped law", block, var)
      self.assertIn("Shaping OFF with Dynamic Brakes ON and Shadow OFF gives the OLD law", block, var)
      self.assertIn("Shadow ON, or Shaping and Dynamic Brakes both OFF, gives the plain OLD law", block, var)
      self.assertIn("at the next ignition", block, var)
    # D2 / nit: no bypass on the continuous law, and the OLD law's measured number, not "a tenth of a second".
    for var in ("shaping_desc", "dynamic_desc"):
      self.assertIn("no fast-step bypass", self._description(var), var)
      self.assertIn("at full strength in 140 ms", self._description(var), var)
      self.assertNotIn("a tenth of a second", self._description(var), var)
    # D3: Shadow names the LOGGED law correctly.
    self.assertIn("logs the 0043 shaped law", self._description("shadow_desc"))

  def test_new_lane_toggles_wiring(self):
    """17c/18x: the three param-only lane switches are real UI toggles now, on the right keys and the right rules."""
    for title, param, attr, rule in (
      ("Pedal Delivery Law", "HyundaiPedalDeliveryLaw", "pedal_delivery_toggle", "pedal_delivery_toggle_state(pedal_raw, offroad)"),
      ("Gas Rise Lag", "HyundaiGasRiseLag", "gas_rise_lag_toggle", "gas_rise_lag_toggle_state(pedal_raw, offroad)"),
      ("Dynamic Brakes", "HyundaiFca11DynamicBrakes", "fca11_dynamic_release_toggle",
       "fca11_dynamic_release_toggle_state(pedal_raw, brake_raw, offroad)"),
      ("Stop-Approach Commitment", "StopApproachCommit", "stop_approach_toggle", "stop_approach_toggle_state(offroad)"),
    ):
      self.assertRegex(self.src, r'toggle_item_sp\(tr\("' + re.escape(title) + r'"\),\s*"",\s*param="' + param + r'"')
      m = re.search(r"self\.items = \[(.*?)\]", self.src, re.S)
      self.assertIn(f"self.{attr}", m.group(1))
      self.assertIn(rule, self.src)
      self.assertIn(f"self.{attr}.set_visible(", self.src)
      self.assertIn(f"self.{attr}.action_item.set_enabled(", self.src)

  def test_new_lane_descriptions_are_honest(self):
    self.assertIn("Requires the comma pedal", self.src)          # pedal delivery
    self.assertIn("Requires FCA11 Braking", self.src)            # dynamic release
    self.assertIn("never raises it", self.src)                   # stop-approach commitment
    self.assertNotIn("you must enable it with a param", self.src)

  def test_longitudinal_tuning_row_wiring(self):
    """SETTINGS TIDY: the upstream row stays wired to its key but its visibility goes through the pure rule."""
    self.assertRegex(self.src, r'multiple_button_item_sp\(tr\("Custom Longitudinal Tuning"\),\s*"",\s*tuning_texts,')
    self.assertIn('param="HyundaiLongitudinalTuning"', self.src)
    self.assertIn("longitudinal_tuning_toggle_state(pedal_raw, offroad)", self.src)
    self.assertIn("self.longitudinal_tuning_item.set_visible(self.alpha_long_available and tuning_state.visible)", self.src)

  def test_stop_hold_row_wiring(self):
    """SETTINGS TIDY: the retired row keeps title -> param -> attribute wiring; only its visibility is forced off."""
    self.assertRegex(self.src, r'toggle_item_sp\(tr\("Hold Clutch Release \(experimental\)"\),\s*"",\s*'
                               r'param="HyundaiFca11StopHoldClutch"')
    m = re.search(r"self\.items = \[(.*?)\]", self.src, re.S)
    self.assertIsNotNone(m)
    self.assertIn("self.fca11_stop_hold_toggle", m.group(1))
    self.assertIn("fca11_stop_hold_toggle_state(pedal_raw, brake_raw, offroad)", self.src)
    self.assertIn("self.fca11_stop_hold_toggle.set_visible(hold_state.visible)", self.src)
    self.assertIn("FORK.md row 137", self.src)   # how to bring it back is written next to the row
    self.assertIn("(row 140)", self.src)         # ... and the TCU evidence it may not cite otherwise


# --- D4: the brake-law text, checked against the REAL controller for every (Shaping, Dynamic, Shadow) combination ---
#
# The three brake descriptions are not pinned as strings here. Each one states the FOUR real states ("condition gives
# law"), that table is PARSED out of the panel's own description, and it is evaluated at each of the 8 combinations
# against the law the REAL car layer selects for the same combination (`setup_interfaces` + `Fca11LongBrake`, reading
# cont_law / shape_track / feel / dynamic_brakes). A description with the wrong condition - or a panel line that
# flips a row after `update_settings` set it - therefore fails BY VALUE, on the combination it got wrong.
LAW_CONT, LAW_SHAPED, LAW_RAMP, LAW_OLD = "continuous", "0043", "18x", "OLD"
BRAKE_ROWS = ("FCA11 Brake Shaping (Beta)", "Dynamic Brakes", "FCA11 Shadow (Log-Only)")
HIDDEN_ROWS = {"Custom Longitudinal Tuning", "Hold Clutch Release (experimental)"}

# The REAL panel (this tree's hyundai.py + fca11_settings.py) executed headless: the widget/ui_state/multilang modules
# are recording stubs, the panel file is the tree's own. Prints one JSON object: every row's visible/enabled/description
# for all 8 Shaping/Dynamic/Shadow combinations, the same onroad, and every param write seen.
_PANEL_PROBE = r'''
import importlib.util, json, os, sys, types

tree, odbc = sys.argv[1], sys.argv[2]
sys.path[:0] = [tree, odbc]

PARAMS = {"HyundaiGasInterceptor": True, "HyundaiFca11Brake": True, "HyundaiPedalDeliveryLaw": True,
          "HyundaiGasRiseLag": True, "HyundaiFca11StopHoldClutch": False, "HyundaiLongitudinalTuning": 0,
          "AdaptiveFollowDistance": True, "NoCloseOnLead": True, "StopApproachCommit": True,
          "CarPlatformBundle": {"platform": "HYUNDAI_ELANTRA_2021"}}
WRITES = []


class Params:
  def __init__(self, d): self.d = dict(d)
  def get(self, k, *a, **kw): return self.d.get(k)
  def get_bool(self, k, *a, **kw): return bool(self.d.get(k))
  def put(self, k, v, *a, **kw):
    WRITES.append([k, str(v)])
    self.d[k] = v
  put_bool = put_nonblocking = put


class UIState:
  def __init__(self):
    self.params = Params(PARAMS)
    self.CP = None
    self.has_longitudinal_control = True
    self.off = True
  def is_offroad(self): return self.off


class Action:
  def __init__(self): self.enabled = None; self.selected = None
  def set_enabled(self, e): self.enabled = e
  def set_selected_button(self, i): self.selected = i


class Item:
  def __init__(self, title, param, callback):
    self.title, self.param, self.callback = title, param, callback
    self.action_item = Action(); self.visible = True; self.desc = ""
  def set_visible(self, v): self.visible = v
  def set_description(self, d): self.desc = d
  def show_description(self, s): pass


def mod(name, **attrs):
  m = types.ModuleType(name); m.__dict__.update(attrs); sys.modules[name] = m


ui_state = UIState()
mod("openpilot.selfdrive.ui.ui_state", ui_state=ui_state)
mod("openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.base", BrandSettings=type("BrandSettings", (), {}))
mod("openpilot.system.ui.lib.multilang", tr=lambda s: s)
mod("openpilot.system.ui.sunnypilot.widgets.list_view",
    toggle_item_sp=lambda title, desc="", param=None, callback=None, **kw: Item(title, param, callback),
    multiple_button_item_sp=lambda title, desc, buttons, param=None, callback=None, **kw: Item(title, param, callback))

spec = importlib.util.spec_from_file_location(
  "panel_under_test", os.path.join(tree, "openpilot/selfdrive/ui/sunnypilot/layouts/settings/vehicle/brands/hyundai.py"))
hy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(hy)

panel = hy.HyundaiSettings()
out = {}
for s in (True, False):
  for d in (True, False):
    for h in (True, False):
      ui_state.params.d["HyundaiFca11AffineGain"] = s
      ui_state.params.d["HyundaiFca11DynamicBrakes"] = d
      ui_state.params.d["HyundaiFca11Shadow"] = h
      panel.update_settings()
      out["%d%d%d" % (s, d, h)] = {it.title: {"visible": bool(it.visible), "enabled": it.action_item.enabled,
                                              "desc": it.desc} for it in panel.items}
ui_state.off = False
panel.update_settings()
onroad = {it.title: {"visible": bool(it.visible), "enabled": it.action_item.enabled, "desc": it.desc}
          for it in panel.items}
for it in panel.items:
  if it.callback is not None and it.param != "HyundaiLongitudinalTuning":
    it.callback(True)
print(json.dumps({"combos": out, "onroad": onroad, "writes": WRITES}))
'''


def _panel_matrix():
  """Run the real panel headless in its own process and return its row matrix (see `_PANEL_PROBE`).

  A separate process on purpose: importing the panel pulls raylib in, which the rest of this suite must never load,
  and the stub modules must not leak into `sys.modules` for the other tests. It also means a panel that raises
  (e.g. a NameError from a deleted read) fails the test with its own traceback instead of poisoning this process."""
  try:
    import opendbc
  except Exception as e:   # no opendbc: the panel cannot be imported, and neither can the car layer
    raise unittest.SkipTest(f"opendbc is not importable: {e}")
  odbc = Path(opendbc.__file__).resolve().parents[1]
  env = {k: v for k, v in os.environ.items() if k != "PYTHONHOME"}
  env["PYTHONPATH"] = os.pathsep.join([str(REPO), str(odbc)])
  r = subprocess.run([sys.executable, "-B", "-c", _PANEL_PROBE, str(REPO), str(odbc)],
                     capture_output=True, text=True, env=env, timeout=300)
  if r.returncode != 0:
    raise AssertionError("the panel did not survive update_settings headless:\n" + (r.stderr or r.stdout)[-3000:])
  return json.loads(r.stdout)


def _real_law(shaping, dynamic, shadow):
  """The law the REAL car layer selects for this combination, as a token.

  Built through the real `setup_interfaces` (the one the car boots with - it is where Shadow forces Shaping and
  Dynamic Brakes off) and read off the controller's own switches, never re-derived from the UI rule under test."""
  try:
    from opendbc.car import structs
    from opendbc.car.hyundai.interface import CarInterface
    from opendbc.car.hyundai.values import CAR
    from opendbc.sunnypilot.car.interfaces import setup_interfaces
    from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP
    from opendbc.sunnypilot.car.hyundai import fca11_long as fl
  except Exception as e:
    raise unittest.SkipTest(f"the fork's fca11_long is not in this tree: {e}")

  saved = {k: os.environ.get(k) for k in ("FCA11_LOG_DIR", "FCA11_CAL_DIR")}
  os.environ["FCA11_LOG_DIR"] = "0"    # never touch /data/fca11-log from a test host
  os.environ["FCA11_CAL_DIR"] = "0"
  try:
    cp = structs.CarParams()
    cp.brand = "hyundai"
    cp.carFingerprint = CAR.HYUNDAI_ELANTRA_2021
    cp_sp = structs.CarParamsSP()
    cp_sp.flags |= HyundaiFlagsSP.GAS_INTERCEPTOR_DETECTED.value
    setup_interfaces(CarInterface, cp, cp_sp, [
      {"HyundaiGasInterceptor": "1"}, {"HyundaiFca11Brake": True},
      {"HyundaiFca11AffineGain": bool(shaping)}, {"HyundaiFca11DynamicBrakes": bool(dynamic)},
      {"HyundaiFca11Shadow": bool(shadow)}, {"HyundaiLongitudinalTuning": 0},
    ])
    brake = fl.Fca11LongBrake(cp, cp_sp)
  finally:
    for k, v in saved.items():
      if v is None:
        os.environ.pop(k, None)
      else:
        os.environ[k] = v

  if brake.cont_law:
    return LAW_CONT
  if brake.affine_gain:
    return LAW_SHAPED
  if brake.feel:
    return LAW_RAMP if brake.dynamic_brakes else LAW_OLD
  raise AssertionError(f"no law flag is set for Shaping={shaping} Dynamic={dynamic} Shadow={shadow}: "
                       f"cont_law={brake.cont_law} affine_gain={brake.affine_gain} feel={brake.feel}")


def _law_table(desc):
  """{condition: law token} parsed out of a description's own four-state sentences.

  R1 (Opus re-review N2): the Shadow condition is read FROM EACH SENTENCE, never assumed. A sentence that claims a
  law for a Shaping/Dynamic combination but names no Shadow state is an ERROR here - that is exactly the mutant
  "S1D0 gives the 0043 shaped law whatever Shadow is set to", which must die by value on 101/011, where Shadow ON
  really overrides. The law token is whatever the sentence itself names - the parser never assumes which law a
  condition gets."""
  table = {}
  for sent in re.split(r"(?<=\.)\s+", desc):
    if not sent.startswith("Shaping ") or "gives the" not in sent:
      continue
    m = re.search(r"gives the ([^:.]*)", sent)
    law = ("continuous" if "continuous" in m.group(1) else
           "0043" if "0043" in m.group(1) else
           "18x" if "18x" in m.group(1) else
           "OLD" if "OLD law" in m.group(1) else None)
    if law is None:
      continue
    s = 1 if re.search(r"Shaping ON", sent) else 0
    d = 1 if re.search(r"Dynamic Brakes ON", sent) else 0
    if "Shadow ON" in sent:
      h = 1
    elif "Shadow OFF" in sent:
      h = 0
    else:
      raise AssertionError(f"this sentence claims {law} without naming the Shadow state it needs: {sent!r}")
    if (s, d, h) in table:
      raise AssertionError(f"two sentences claim Shaping={s} Dynamic={d} Shadow={h}: {sent!r}")
    table[(s, d, h)] = law
  # the catch-all: "Shadow ON, or Shaping and Dynamic Brakes both OFF, gives the plain OLD law."
  for sent in re.split(r"(?<=\.)\s+", desc):
    if sent.startswith("Shadow ON") and "gives the" in sent:
      m = re.search(r"gives the ([^:.]*)", sent)
      table["else"] = "OLD" if "OLD law" in m.group(1) else None
  return table


def _table_law(table, shaping, dynamic, shadow):
  """What the TEXT says the law is for this combination - keyed on the sentence's OWN Shadow condition.

  There is no "shadow means else" shortcut any more: a sentence only speaks for the Shadow state it names, so
  Shaping=1 Dynamic=0 Shadow=1 falls through to the catch-all (the plain OLD law), which is what the car does."""
  key = (shaping, dynamic, shadow)
  if key in table:
    return table[key]
  return table.get("else")


class TestBrakeLawMatrix(unittest.TestCase):
  """D4 (Opus): the descriptions must survive the mutations the string checks could not see. The rows are read out of
  the panel AFTER its own `update_settings`, and the law the text claims is compared with the law the REAL controller
  selects - for every one of the 8 Shaping / Dynamic Brakes / Shadow combinations."""

  @classmethod
  def setUpClass(cls):
    cls.matrix = _panel_matrix()

  def combos(self):
    return list(itertools.product((1, 0), (1, 0), (1, 0)))

  def test_every_row_states_the_law_the_real_controller_picks(self):
    for shaping, dynamic, shadow in self.combos():
      expected = _real_law(shaping, dynamic, shadow)
      combo = f"{shaping}{dynamic}{shadow}"
      self.assertIn(combo, self.matrix["combos"])
      for row in BRAKE_ROWS:
        desc = self.matrix["combos"][combo][row]["desc"]
        table = _law_table(desc)
        self.assertEqual(expected, _table_law(table, shaping, dynamic, shadow),
                         f"{row} at Shaping={shaping} DynamicBrakes={dynamic} Shadow={shadow}: the text says "
                         f"{_table_law(table, shaping, dynamic, shadow)}, the car layer says {expected}\n{desc}")

  def test_every_row_states_all_four_states(self):
    for row in BRAKE_ROWS:
      table = _law_table(self.matrix["combos"]["110"][row]["desc"])
      self.assertEqual({"else", (1, 1, 0), (1, 0, 0), (0, 1, 0)}, set(table), row)

  def test_the_continuous_row_is_the_one_the_car_runs(self):
    """The car's own combination (Shaping 1, Dynamic 1, Shadow 0) must be the continuous law, and the text must say
    the car is on it - the point of describing the car's state first."""
    self.assertEqual(LAW_CONT, _real_law(1, 1, 0))
    for row in BRAKE_ROWS:
      self.assertIn("This car runs the continuous linear law", self.matrix["combos"]["110"][row]["desc"], row)

  def test_the_two_hidden_rows_are_hidden_in_every_combination(self):
    """A rule that flips a retired row back on after the fact (a late `set_visible(brake_state.visible)` or a late
    tuning re-show) lands here: the rows are read after update_settings, not inferred from the source."""
    for combo, rows in self.matrix["combos"].items():
      self.assertEqual(HIDDEN_ROWS, {t for t, r in rows.items() if not r["visible"]}, combo)

  def test_the_brake_rows_are_listed_and_settable_offroad_in_every_combination(self):
    for combo, rows in self.matrix["combos"].items():
      for row in BRAKE_ROWS:
        self.assertTrue(rows[row]["visible"], f"{row} hidden at {combo}")
        self.assertTrue(rows[row]["enabled"], f"{row} not settable offroad at {combo}")

  def test_onroad_hides_nothing_extra_and_disables_the_editable_rows(self):
    rows = self.matrix["onroad"]
    self.assertEqual(HIDDEN_ROWS, {t for t, r in rows.items() if not r["visible"]})
    for title, row in rows.items():
      if title in HIDDEN_ROWS or title == "Drive Mode Personality":
        continue
      self.assertFalse(row["enabled"], f"{title} is settable onroad")

  def test_update_settings_and_every_row_callback_write_no_param(self):
    """The tidy is a visibility/text change: 8 combinations plus every row callback = 0 param writes."""
    self.assertEqual([], self.matrix["writes"])


class TestEveryRowWiring(unittest.TestCase):
  """Every row must resolve title -> param -> attribute, and every param must be a registered key. This is the guard
  the two hidden rows are expected to keep passing: they are hidden by a rule, not unwired."""

  @classmethod
  def setUpClass(cls):
    cls.src = PANEL.read_text(encoding="utf-8")
    cls.keys_src = (REPO / "openpilot" / "common" / "params_keys.h").read_text(encoding="utf-8")
    m = re.search(r"self\.items = \[(.*?)\]", cls.src, re.S)
    cls.items = m.group(1) if m else ""

  def rows(self):
    """(attribute, title, param) for every toggle / multi-button row the panel builds."""
    out = []
    for m in re.finditer(r"self\.(\w+) = (?:toggle_item_sp|multiple_button_item_sp)\(\s*tr\(\"([^\"]+)\"\)", self.src):
      tail = self.src[m.end():m.end() + 400]
      hits = re.findall(r'param="(\w+)"', tail)
      self.assertTrue(hits, f"{m.group(1)} row has no param= (title -> param broken)")
      out.append((m.group(1), m.group(2), hits[0]))
    return out

  # The one row the panel never hides (it has no visibility write at all, and never had one).
  ALWAYS_VISIBLE = {"Drive Mode Personality"}

  def test_every_row_resolves_title_param_attribute(self):
    rows = self.rows()
    self.assertEqual(12, len(rows), [r[1] for r in rows])   # 11 toggles + the 3-way tuning row
    for attr, title, param in rows:
      self.assertIn(f"self.{attr}", self.items, f"{title}: not in self.items")
      self.assertRegex(self.keys_src, r'\{"' + param + r'", ', f"{title}: {param} is not a registered key")
      if title not in self.ALWAYS_VISIBLE:
        self.assertIn(f"self.{attr}.set_visible(", self.src, f"{title}: no visibility write")

  def test_the_two_hidden_rows_are_still_wired(self):
    """Hiding must not be done by deleting the row: both retired/inert rows stay in the panel and keep their key."""
    by_param = {param: (attr, title) for attr, title, param in self.rows()}
    self.assertIn("HyundaiLongitudinalTuning", by_param)
    self.assertIn("HyundaiFca11StopHoldClutch", by_param)


if __name__ == "__main__":
  unittest.main()
