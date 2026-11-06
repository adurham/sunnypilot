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
      "Hold Clutch Release (experimental)": fca11_stop_hold_toggle_state(pedal, brake, "0", True),
    }
    hidden = {title for title, state in rows.items() if not state.visible}
    self.assertEqual({"Custom Longitudinal Tuning", "Hold Clutch Release (experimental)"}, hidden)

  def test_brake_state_rule_and_sentence(self):
    """R2: the law token mirrors the car layer from the VALUES (Braking off => no law; Shadow => the plain OLD law;
    Shaping+Dynamic => continuous; Shaping => 0043; Dynamic => 18x; neither => OLD), and the sentence names all four."""
    want = {  # (Braking, Shaping, Dynamic, Shadow) -> law token
      (0, 1, 1, 0): fca11_settings.LAW_NONE, (0, 0, 0, 1): fca11_settings.LAW_NONE,
      (1, 1, 1, 0): fca11_settings.LAW_CONTINUOUS, (1, 1, 0, 0): fca11_settings.LAW_SHAPED,
      (1, 0, 1, 0): fca11_settings.LAW_RAMP, (1, 0, 0, 0): fca11_settings.LAW_OLD,
      (1, 1, 1, 1): fca11_settings.LAW_OLD, (1, 1, 0, 1): fca11_settings.LAW_OLD,
      (1, 0, 1, 1): fca11_settings.LAW_OLD,
    }
    for (b, s, d, h), law in want.items():
      self.assertEqual(law, fca11_settings.fca11_brake_state(b, s, d, h).law, (b, s, d, h))
      self.assertEqual(law, fca11_settings.fca11_brake_state(bool(b), bool(s), bool(d), bool(h)).law)
    # the string forms the params actually hold ("1" is what Params.get() answers for a BOOL key)
    self.assertEqual(fca11_settings.LAW_CONTINUOUS, fca11_settings.fca11_brake_state("1", "1", "1", "0").law)
    self.assertEqual(fca11_settings.LAW_OLD, fca11_settings.fca11_brake_state("1", "1", "1", "1").law)
    # the sentence follows the values, and the Braking-OFF one keeps the old "nothing changes" meaning
    cont = fca11_settings.fca11_brake_state_sentence("1", "1", "1", "0")
    self.assertTrue(cont.startswith("This car runs the continuous linear law"), cont)
    for pat in (r"FCA11 Braking ON", r"Shaping ON", r"Dynamic Brakes ON", r"Shadow OFF"):
      self.assertRegex(cont, pat)
    off = fca11_settings.fca11_brake_state_sentence("0", "1", "1", "0")
    self.assertIn("no FCA11 brake law", off)
    self.assertIn("nothing changes while Braking is off", off)
    for pat in (r"FCA11 Braking OFF", r"Shaping ON", r"Dynamic Brakes ON", r"Shadow OFF"):
      self.assertRegex(off, pat)

  def test_stop_hold_row_is_listed_only_while_its_value_is_on(self):
    """R4 (Opus re-review (f)): the 0034 row is hidden at its value 0 and SHOWN at value 1, in every pedal/brake/onroad
    state. The key is BACKUP-flagged and was 1 on route 195, so a device backup restore can write 1 and the reader
    honours it with no switch in sight — the row is that switch. The panel never writes the param (the write test
    below pins update_settings at zero writes)."""
    for pedal, brake, hold, offroad in itertools.product(ON_VALUES + OFF_VALUES, ON_VALUES + OFF_VALUES,
                                                         ON_VALUES + OFF_VALUES, (True, False)):
      vis = pedal in ON_VALUES and brake in ON_VALUES and hold in ON_VALUES
      self.assertEqual(ToggleState(visible=vis, enabled=vis and offroad),
                       fca11_stop_hold_toggle_state(pedal, brake, hold, offroad), (pedal, brake, hold, offroad))
    # the four corners the review named, spelled out
    self.assertEqual(ToggleState(False, False), fca11_stop_hold_toggle_state("1", "1", "0", True))   # OFF => hidden
    self.assertEqual(ToggleState(True, True), fca11_stop_hold_toggle_state("1", "1", "1", True))     # ON  => shown
    self.assertEqual(ToggleState(True, False), fca11_stop_hold_toggle_state("1", "1", "1", False))   # offroad only
    self.assertEqual(ToggleState(False, False), fca11_stop_hold_toggle_state("0", "1", "1", True))   # still needs the
    self.assertEqual(ToggleState(False, False), fca11_stop_hold_toggle_state("1", "0", "1", True))   # pedal + braking

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
    """The whole rendered text of a panel description (`shaping_desc`, `dynamic_desc`, `shadow_desc`).

    R2: the description is now assembled (`tr(state_sent) + " " + tr("...")`), so this joins every string constant of
    the assignment's expression; the generated car-state sentence is the `state_sent` name and renders as <STATE>
    here. Both branches of the Shadow row's `shadow_law` conditional are joined, so the static checks below see the
    whole vocabulary (the RENDERED text per combination is what TestBrakeLawMatrix checks)."""
    module = ast.parse(self.src)
    assign = None
    for node in ast.walk(module):
      if isinstance(node, ast.Assign) and any(getattr(t, "id", "") == var for t in node.targets):
        assign = node.value   # the first (main) assignment; a later `var = offroad_hint + var` is skipped
        break
    self.assertIsNotNone(assign, f"no {var} description in the panel")

    def named(name, seen=()):
      """Every assignment of a local name (e.g. `shadow_law`, which has a Braking-OFF variant), joined."""
      if name in seen:
        raise AssertionError(f"recursive description name: {name}")
      values = [n.value for n in ast.walk(module)
                if isinstance(n, ast.Assign) and any(getattr(t, "id", "") == name for t in n.targets)]
      self.assertTrue(values, f"{name} is used in a description but never assigned")
      return " ".join(join(v, seen + (name,)) for v in values)

    def join(node, seen=()):
      if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
      if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Add):
        return join(node.left, seen) + join(node.right, seen)
      if isinstance(node, ast.Call) and getattr(node.func, "id", "") == "tr":
        return join(node.args[0], seen)
      if isinstance(node, ast.Name):
        return "<STATE>" if node.id == "state_sent" else named(node.id, seen)
      if isinstance(node, ast.IfExp):
        return join(node.body, seen) + " " + join(node.orelse, seen)
      raise AssertionError(f"unexpected description node: {type(node).__name__}")

    return join(assign)

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
    """SETTINGS TIDY + R4: the row keeps title -> param -> attribute wiring, and its visibility comes from the pure rule
    WITH the row's own value (listed only while ON, pedal and braking on)."""
    self.assertRegex(self.src, r'toggle_item_sp\(tr\("Hold Clutch Release \(experimental\)"\),\s*"",\s*'
                               r'param="HyundaiFca11StopHoldClutch"')
    m = re.search(r"self\.items = \[(.*?)\]", self.src, re.S)
    self.assertIsNotNone(m)
    self.assertIn("self.fca11_stop_hold_toggle", m.group(1))
    self.assertIn('hold_raw = ui_state.params.get("HyundaiFca11StopHoldClutch")', self.src)
    self.assertIn("fca11_stop_hold_toggle_state(pedal_raw, brake_raw, hold_raw, offroad)", self.src)
    self.assertIn("self.fca11_stop_hold_toggle.set_visible(hold_state.visible)", self.src)
    self.assertIn("FORK.md row 137", self.src)   # why the row is out of the way is written next to the row
    self.assertIn("(row 141)", self.src)         # ... and the TCU evidence it may not cite otherwise
    # R5 nits 2-3 (Opus): the row must not claim the TCU opens its clutch on THIS request (row 137 / lane-U: the DCT
    # opens on the brake pedal switch), and it must not guess where a stored ON came from - it can only say what the
    # panel can see.
    self.assertNotIn("releases the clutch", self.src)
    hold_desc = self._description("hold_desc")
    self.assertNotIn("backup restore", hold_desc)
    self.assertIn("(the stored value is ON)", hold_desc)


# --- D4: the brake-law text, checked against the REAL controller for every (Shaping, Dynamic, Shadow) combination ---
#
# The three brake descriptions are not pinned as strings here. Each one states the FOUR real states ("condition gives
# law"), that table is PARSED out of the panel's own description, and it is evaluated at each of the 8 combinations
# against the law the REAL car layer selects for the same combination (`setup_interfaces` + `Fca11LongBrake`, reading
# cont_law / shape_track / feel / dynamic_brakes). A description with the wrong condition - or a panel line that
# flips a row after `update_settings` set it - therefore fails BY VALUE, on the combination it got wrong.
LAW_CONT, LAW_SHAPED, LAW_RAMP, LAW_OLD, LAW_NONE = "continuous", "0043", "18x", "OLD", "none"
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
for b in (True, False):
  for s in (True, False):
    for d in (True, False):
      for h in (True, False):
        ui_state.params.d.update({"HyundaiFca11Brake": b, "HyundaiFca11AffineGain": s,
                                  "HyundaiFca11DynamicBrakes": d, "HyundaiFca11Shadow": h})
        panel.update_settings()
        out["%d%d%d%d" % (b, s, d, h)] = {it.title: {"visible": bool(it.visible), "enabled": it.action_item.enabled,
                                                     "desc": it.desc} for it in panel.items}
update_writes = list(WRITES)     # R3 (F4): update_settings alone, 16 combinations
# R4: the stop-hold row's own value ON (what a route-195-era backup restore writes) - the row must be SHOWN then.
hold_on = {}
ui_state.params.d["HyundaiFca11StopHoldClutch"] = True
for b in (True, False):
  for s in (True, False):
    for d in (True, False):
      for h in (True, False):
        ui_state.params.d.update({"HyundaiFca11Brake": b, "HyundaiFca11AffineGain": s,
                                  "HyundaiFca11DynamicBrakes": d, "HyundaiFca11Shadow": h})
        panel.update_settings()
        hold_on["%d%d%d%d" % (b, s, d, h)] = {"visible": bool(panel.fca11_stop_hold_toggle.visible),
                                              "enabled": panel.fca11_stop_hold_toggle.action_item.enabled}
ui_state.params.d["HyundaiFca11StopHoldClutch"] = False
ui_state.params.d.update({"HyundaiFca11Brake": True, "HyundaiFca11AffineGain": True,
                          "HyundaiFca11DynamicBrakes": True, "HyundaiFca11Shadow": False})
# N6: the row at value ON while ONROAD - it must be LISTED (its rule is visibility, not offroad) and NOT settable.
ui_state.params.d["HyundaiFca11StopHoldClutch"] = True
ui_state.off = False
panel.update_settings()
hold_on_onroad = {"visible": bool(panel.fca11_stop_hold_toggle.visible),
                  "enabled": panel.fca11_stop_hold_toggle.action_item.enabled}
ui_state.off = True
ui_state.params.d["HyundaiFca11StopHoldClutch"] = False
ui_state.off = False
panel.update_settings()
onroad = {it.title: {"visible": bool(it.visible), "enabled": it.action_item.enabled, "desc": it.desc}
          for it in panel.items}
ui_state.off = True
panel.update_settings()
# R3 (F4): EVERY row callback runs, including the Custom Longitudinal Tuning button handler the old probe skipped.
# The multi-button handler takes an index (the 3-way Off/Dynamic/Predictive choice), the toggles take the new value.
WRITES.clear()
for it in panel.items:
  if it.callback is None:
    continue
  if it.param == "HyundaiLongitudinalTuning":
    it.callback(2)
  else:
    it.callback(True)
print(json.dumps({"combos": out, "hold_on": hold_on, "hold_on_onroad": hold_on_onroad, "onroad": onroad,
                  "update_writes": update_writes, "callback_writes": list(WRITES)}))
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


def _real_law(brake, shaping, dynamic, shadow):
  """The law the REAL car layer selects for this combination, as a token.

  Built through the real `setup_interfaces` (the one the car boots with - it is where Shadow forces Shaping and
  Dynamic Brakes off, and where a Braking-off car gets no FCA11 at all) and read off the controller's own switches,
  never re-derived from the UI rule under test. R2 adds the Braking dimension: `enabled` is the controller's own
  answer to "does any of this reach the car", so Braking off is LAW_NONE by value, not by assumption."""
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
      {"HyundaiGasInterceptor": "1"}, {"HyundaiFca11Brake": bool(brake)},
      {"HyundaiFca11AffineGain": bool(shaping)}, {"HyundaiFca11DynamicBrakes": bool(dynamic)},
      {"HyundaiFca11Shadow": bool(shadow)}, {"HyundaiLongitudinalTuning": 0},
    ])
    brake_obj = fl.Fca11LongBrake(cp, cp_sp)
  finally:
    for k, v in saved.items():
      if v is None:
        os.environ.pop(k, None)
      else:
        os.environ[k] = v

  if not brake:
    assert brake_obj.enabled is False, "a Braking-OFF controller must be disabled"
    assert not (brake_obj.cont_law or brake_obj.affine_gain or brake_obj.feel or brake_obj.dynamic_brakes), \
      "a Braking-OFF controller must not select any law"
    return LAW_NONE
  assert brake_obj.enabled is True, "a Braking-ON controller must be enabled"
  if brake_obj.cont_law:
    return LAW_CONT
  if brake_obj.affine_gain:
    return LAW_SHAPED
  if brake_obj.feel:
    return LAW_RAMP if brake_obj.dynamic_brakes else LAW_OLD
  raise AssertionError(f"no law flag is set for Braking={brake} Shaping={shaping} Dynamic={dynamic} Shadow={shadow}: "
                       f"cont_law={brake_obj.cont_law} affine_gain={brake_obj.affine_gain} feel={brake_obj.feel}")


def _camera_frame(alive=3):
  """A 0x38D camera frame in the LOGGED layout, so the law's own camera checks see a live, non-braking camera
  (status/DriverSetStatus/alive set, byte4 bit7 always 1, real CRC)."""
  try:
    from opendbc.car.hyundai.hyundaican import hyundai_checksum
  except Exception as e:
    raise unittest.SkipTest(f"the fork's hyundai can helpers are not in this tree: {e}")
  b = bytearray(8)

  def setb(start, length, val):
    for i in range(length):
      b[(start + i) // 8] |= ((int(val) >> i) & 1) << ((start + i) % 8)

  setb(18, 2, 2)        # FCA_Status
  setb(22, 3, 1)        # FCA_DrvSetStatus
  setb(35, 4, alive)
  setb(39, 9, 255)      # FCA_RelativeVelocity
  setb(48, 8, 254)      # FCA_TimetoCollision
  b[4] |= 0x80
  b[7] = hyundai_checksum(bytes(b[:7]))
  return bytes(b)


def _sent_levels(shaping, dynamic, shadow, accel, v_ego=15.0, pre=100, frames=250):
  """[(level, dat)] for every 0x38D frame the REAL controller sends after `accel` becomes the ask.

  `pre` frames of a 0.0 ask first (a settled cruise), then the ask held for `frames` frames at 50 Hz, so the levels are
  the ones that would reach the car on the wire. The controller is the tree's own, built through `setup_interfaces`, so
  a change in the car layer shows up here as a changed level list, not as a stale comment."""
  try:
    from opendbc.car import structs
    from opendbc.car.hyundai.interface import CarInterface
    from opendbc.car.hyundai.values import CAR
    from opendbc.sunnypilot.car.interfaces import setup_interfaces
    from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP
    from opendbc.sunnypilot.car.hyundai import fca11_long as fl
  except Exception as e:
    raise unittest.SkipTest(f"the fork's fca11_long is not in this tree: {e}")

  class _CS:
    """The same minimal CarState double the car layer's own controller tests use (it reads nothing else)."""

    def __init__(self):
      self.out = structs.CarState()
      self.out.vEgo = v_ego
      self.out.gearShifter = structs.CarState.GearShifter.drive
      self.out.brakePressed = False
      self.out.gasPressed = False
      self.out.aEgo = 0.0
      self.out.standstill = False
      self.interceptor_state = 0
      self.interceptor_fault_seen = False
      self.fca11_cam_frame = (0, _camera_frame())
      self.fca11_now_nanos = 0
      self.fca11_lead_closing = False
      self.fca11_wheel_min_kph = v_ego * 3.6
      self.fca11_echoes = []

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
    brake_obj = fl.Fca11LongBrake(cp, cp_sp)
    brake_obj._dyn_log = None      # no shadow record from a test host
    brake_obj._mark_log = None
    out = []
    for f in range(pre + frames):
      cc = structs.CarControl()
      cc.enabled = True
      cc.longActive = True
      cc.actuators.accel = 0.0 if f < pre else accel
      for s in brake_obj.update(cc, _CS(), f):
        if s.address == fl.FCA11_ADDR and f >= pre:
          out.append((fl._get_bits(s.dat, 8, 8), s.dat))
  finally:
    for k, v in saved.items():
      if v is None:
        os.environ.pop(k, None)
      else:
        os.environ[k] = v
  return out


def _eighteen_clause(desc):
  """The 18x sentence of a brake row, exactly as that row renders it."""
  sent = next((s for s in _sentences(desc)
               if s.startswith("Shaping OFF with Dynamic Brakes ON and Shadow OFF gives")), None)
  if sent is None:
    raise AssertionError(f"no 18x sentence in this description: {desc[:240]!r}")
  return sent


def _sentences(desc):
  return [s for s in re.split(r"(?<=\.)\s+", desc) if s.strip()]


def _state_claim(desc):
  """The car-state sentence a row opens with (R2: `fca11_brake_state_sentence`, rendered by the panel).

  Returns (law token, (Braking, Shaping, Dynamic, Shadow) as the SENTENCE names them). Every one of the four must be
  named: the sentence is the reader's summary, so a sentence that names the wrong combination (or drops a switch)
  fails here by value - that is the X4 mutant."""
  sent = next((s for s in _sentences(desc) if s.startswith("This car runs")), None)
  if sent is None:
    raise AssertionError(f"no car-state sentence in this description: {desc[:200]!r}")
  head = re.search(r"This car runs (.*?):", sent)
  if head is None:
    raise AssertionError(f"unparsable car-state sentence: {sent!r}")
  law = ("none" if "no FCA11 brake law" in head.group(1) else
         "continuous" if "continuous" in head.group(1) else
         "0043" if "0043" in head.group(1) else
         "18x" if "18x" in head.group(1) else
         "OLD" if "OLD law" in head.group(1) else None)
  if law is None:
    raise AssertionError(f"the car-state sentence names no known law: {sent!r}")
  conds = []
  for pat in (r"FCA11 Braking (ON|OFF)", r"Shaping (ON|OFF)", r"Dynamic Brakes (ON|OFF)", r"Shadow (ON|OFF)"):
    m = re.search(pat, sent)
    if m is None:
      raise AssertionError(f"the car-state sentence does not name {pat}: {sent!r}")
    conds.append(int(m.group(1) == "ON"))
  return law, tuple(conds), sent


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
  """D4 + R1 + R2 (Opus): the descriptions must survive the mutations the string checks could not see. The rows are
  read out of the real panel AFTER its own `update_settings`, and BOTH what the text claims is compared with what the
  REAL controller does - for all 16 Braking / Shaping / Dynamic Brakes / Shadow combinations:

    * the car-state sentence must name the ACTUAL four values and the law the controller selects (R2/N1/N3);
    * the four-state table must give the law the controller selects, keyed on each sentence's OWN Shadow condition
      (R1/N2);
    * Braking OFF must say that no FCA11 brake law runs and that nothing changes (N3)."""

  @classmethod
  def setUpClass(cls):
    cls.matrix = _panel_matrix()

  def combos(self):
    return list(itertools.product((1, 0), (1, 0), (1, 0), (1, 0)))

  def test_the_state_sentence_names_the_actual_params_and_the_real_law(self):
    """R2: the sentence is built from the values, so it names the combination the car is really in and the law the
    real controller selects - in all 16 states, including Braking OFF (where no law runs at all)."""
    for brake, shaping, dynamic, shadow in self.combos():
      combo = f"{brake}{shaping}{dynamic}{shadow}"
      expected = _real_law(brake, shaping, dynamic, shadow)
      for row in BRAKE_ROWS:
        desc = self.matrix["combos"][combo][row]["desc"]
        law, conds, sent = _state_claim(desc)
        self.assertEqual((brake, shaping, dynamic, shadow), conds,
                         f"{row} at {combo}: the state sentence names {conds}, the params are "
                         f"{(brake, shaping, dynamic, shadow)}\n{sent}")
        self.assertEqual(expected, law,
                         f"{row} at {combo}: the state sentence says {law}, the car layer says {expected}\n{sent}")

  def test_braking_off_says_no_law_runs_and_nothing_changes(self):
    """N3: Braking OFF leaves the Shadow row listed while the controller is `enabled=False`; 'this car runs the
    continuous linear law' and 'ON switches braking back' were false there. The old meaning is back."""
    for shaping, dynamic, shadow in itertools.product((1, 0), (1, 0), (1, 0)):
      combo = f"0{shaping}{dynamic}{shadow}"
      for row in BRAKE_ROWS:
        desc = self.matrix["combos"][combo][row]["desc"]
        self.assertIn("no FCA11 brake law", desc, row)
        self.assertIn("nothing changes while Braking is off", desc, row)
        self.assertNotIn("This car runs the continuous linear law", desc, row)
    # the Shadow row, where it matters most, says what Shadow still does with nothing to brake
    self.assertIn("only logs", self.matrix["combos"]["0110"]["FCA11 Shadow (Log-Only)"]["desc"])

  def test_every_row_states_the_law_the_real_controller_picks(self):
    """R1: the four-state table of every row, against the real controller, for every Braking-ON combination."""
    for brake, shaping, dynamic, shadow in self.combos():
      if not brake:
        continue
      expected = _real_law(brake, shaping, dynamic, shadow)
      combo = f"{brake}{shaping}{dynamic}{shadow}"
      self.assertIn(combo, self.matrix["combos"])
      for row in BRAKE_ROWS:
        desc = self.matrix["combos"][combo][row]["desc"]
        table = _law_table(desc)
        self.assertEqual(expected, _table_law(table, shaping, dynamic, shadow),
                         f"{row} at Shaping={shaping} DynamicBrakes={dynamic} Shadow={shadow}: the text says "
                         f"{_table_law(table, shaping, dynamic, shadow)}, the car layer says {expected}\n{desc}")

  def test_every_row_states_all_four_states(self):
    for row in BRAKE_ROWS:
      table = _law_table(self.matrix["combos"]["1110"][row]["desc"])
      self.assertEqual({"else", (1, 1, 0), (1, 0, 0), (0, 1, 0)}, set(table), row)

  def test_the_continuous_row_is_the_one_the_car_runs(self):
    """The car's own combination (Braking 1, Shaping 1, Dynamic 1, Shadow 0) is the continuous law, and every brake
    row's car-state sentence says so - the point of describing the car's own state first."""
    self.assertEqual(LAW_CONT, _real_law(1, 1, 1, 0))
    for row in BRAKE_ROWS:
      law, conds, sent = _state_claim(self.matrix["combos"]["1110"][row]["desc"])
      self.assertEqual(LAW_CONT, law, row)
      self.assertEqual((1, 1, 1, 0), conds, f"{row}: {sent}")

  def test_the_18x_clause_states_what_the_real_controller_does(self):
    """R6 (Opus re-review #4): the 18x sentence must be TRUE for every ask the car can send, not just up to -3.5.

    R5 quoted "(-2.8 m/s^2: 20 LSB at 780 ms) ... full strength within 140 ms"; the second is FALSE past -3.6 (the
    wire caps at 46 LSB, values.py l.62 = safety/hyundai.h l.136: -4.0 reaches 36 LSB at 160 ms, -5.0 the cap at
    220 ms, and -5.0 is reachable with Braking ON). A precise time on screen cannot hold for every ask, so the row
    now states the clause QUALITATIVELY - 1 LSB opening and a ramp for a gentle ask, the OLD law's 4 LSB per sent
    frame up to the ask's own level (capped at the wire limit) for an emergency one - and this test pins the
    measurement underneath it: -2.0/-2.8/-2.99 open at 1 LSB and climb, and -3.0/-3.5/-3.62/-4.0/-4.5/-5.0/-6.0
    are 4,8,12,16... and BYTE-IDENTICAL to the OLD law, through `setup_interfaces` + `Fca11LongBrake`, not copied
    from a comment. The numbers themselves are pinned in FORK row 141 by
    `test_the_fork_row_quotes_the_measured_table`."""
    from opendbc.sunnypilot.car.hyundai import fca11_long as fl
    from opendbc.sunnypilot.car.hyundai.values import HYUNDAI_FCA11_LONG_MAX_DEC
    self.assertEqual(-3.0, fl.FCA11_DYN_EMERGENCY_ACCEL, "the sentence quotes -3.0 m/s^2: the boundary moved")
    self.assertEqual(4, fl.HYUNDAI_FCA11_LONG_RATE_STEP, "the sentence quotes 4 LSB per sent frame")
    # 0x38D leaves the controller on SENT frames only; the module's own period is what turns frames into ms.
    dt_ms = int(round(fl.FCA11_SHAPE_SENT_DT * 1000))
    self.assertEqual(20, dt_ms)   # 50 Hz on the wire; the times in FORK row 141 are SENT-frame times

    ramp = {ask: _sent_levels(0, 1, 0, ask) for ask in (-2.0, -2.8, -2.99)}   # S0D1H0, the 18x law
    emerg = {ask: (_sent_levels(0, 1, 0, ask), _sent_levels(0, 0, 0, ask))
             for ask in (-3.0, -3.5, -3.62, -4.0, -4.5, -5.0, -6.0)}
    for ask, frames in ramp.items():
      self.assertEqual(1, frames[0][0], f"a sub-emergency ask ({ask}) must open at 1 LSB, not step")
      self.assertLess(frames[0][0], frames[-1][0], f"a sub-emergency ask ({ask}) must ramp, not jump")
    t20 = next((i * dt_ms for i, (lv, _) in enumerate(ramp[-2.8]) if lv >= 20), None)
    self.assertIsNotNone(t20, "the -2.8 ask must reach 20 LSB on the ramp")
    for ask, (frames, old) in emerg.items():
      self.assertEqual([4, 8, 12, 16], [lv for lv, _ in frames[:4]], f"an emergency ask ({ask}) must keep the step")
      self.assertEqual([d for _, d in old], [d for _, d in frames],
                       f"an emergency ask ({ask}) must be BYTE-IDENTICAL to the OLD law")
    # "up to the ask's own level (capped at the wire limit)": a deeper ask climbs deeper, until the wire cap.
    finals = [emerg[ask][0][-1][0] for ask in (-3.0, -3.5, -3.62, -4.0, -4.5)]
    self.assertEqual(sorted(set(finals)), finals, f"a deeper emergency ask must reach a deeper level: {finals}")
    self.assertEqual(HYUNDAI_FCA11_LONG_MAX_DEC, emerg[-6.0][0][-1][0],
                     "an ask past the cap must stop at the wire's own limit, not at its own level")
    self.assertLess(finals[-1], HYUNDAI_FCA11_LONG_MAX_DEC, "the -4.5 ask must still be under the cap")
    self.assertGreater(finals[-1], emerg[-3.5][0][-1][0], "the -4.5 ask must be past the -3.5 level")

    for row in BRAKE_ROWS:
      clause = _eighteen_clause(self.matrix["combos"]["1010"][row]["desc"])
      self.assertIn("a gentle ask starts at 1 LSB", clause, row)
      self.assertIn(f"{fl.FCA11_DYN_EMERGENCY_ACCEL:.1f} m/s^2 or harder", clause, row)
      self.assertIn("steps like the OLD law", clause, row)
      self.assertIn(f"{fl.HYUNDAI_FCA11_LONG_RATE_STEP} LSB per sent frame", clause, row)
      self.assertIn("capped at the wire limit", clause, row)
      self.assertNotIn("even a hard ask is ramped in rather than stepped", clause, f"{row}: the false clause is back")

  def test_the_on_screen_clause_names_no_time(self):
    """R6 (Opus re-review #4): no millisecond figure on screen - no single one holds for every ask.

    M4 ("within 140 ms" -> "within 60 ms") SURVIVED R5 because nothing read the sentence's number; and the true
    numbers run 120 ms (-3.0) to 220 ms (-5.0 and harder), so a screen that prints one is wrong for the rest. The
    rendered 18x sentence of every brake row, in every one of the 16 Braking/Shaping/Dynamic/Shadow states, must
    carry NO digit followed by "ms"; the measured table lives in FORK row 141, pinned there by
    `test_the_fork_row_quotes_the_measured_table`."""
    for combo, rows in self.matrix["combos"].items():
      for row in BRAKE_ROWS:
        clause = _eighteen_clause(rows[row]["desc"])
        self.assertIsNone(re.search(r"\d\s*ms\b", clause),
                          f"{row} at {combo}: a millisecond figure on screen must hold for EVERY ask, and none does "
                          f"- the table belongs in FORK row 141\n{clause}")

  def test_the_braking_off_sentence_never_appears_with_braking_on(self):
    """N7 (Opus): the Shadow row's Braking-OFF branch must be unreachable while Braking is ON. The `if True:` form
    told a Braking-ON owner "nothing is sent ... only logs", the exact opposite of what the car does."""
    for shaping, dynamic, shadow in itertools.product((1, 0), (1, 0), (1, 0)):
      combo = f"1{shaping}{dynamic}{shadow}"
      for row in BRAKE_ROWS:
        desc = self.matrix["combos"][combo][row]["desc"]
        self.assertNotIn("With FCA11 Braking OFF", desc, f"{row} at {combo}")
        self.assertNotIn("nothing is sent", desc, f"{row} at {combo}")
        self.assertNotIn("assume FCA11 Braking ON", desc, f"{row} at {combo}")

  def test_the_braking_off_table_says_which_car_it_describes(self):
    """Nit 1 (Opus): with Braking OFF the Shadow row is still listed, so its four-state table must say it assumes
    Braking ON - and the table must still be the law the controller picks in the state it names (Braking ON)."""
    for shaping, dynamic, shadow in itertools.product((1, 0), (1, 0), (1, 0)):
      combo = f"0{shaping}{dynamic}{shadow}"
      desc = self.matrix["combos"][combo]["FCA11 Shadow (Log-Only)"]["desc"]
      self.assertIn("The four combinations below assume FCA11 Braking ON", desc, combo)
      table = _law_table(desc)
      self.assertEqual(_real_law(1, shaping, dynamic, shadow), _table_law(table, shaping, dynamic, shadow),
                       f"{combo}: the table must still be the law Braking ON selects\n{desc}")

  def test_hidden_rows_are_the_two_retired_rows_plus_braking_dependents(self):
    """A rule that flips a retired row back on after the fact (a late `set_visible(brake_state.visible)` or a late
    tuning re-show) lands here: the rows are read after update_settings, not inferred from the source. The Shaping and
    Dynamic Brakes rows exist only with FCA11 Braking on, which is their own rule, not a tidy."""
    for combo, rows in self.matrix["combos"].items():
      brake = int(combo[0])
      expected = set(HIDDEN_ROWS)
      if not brake:
        expected |= {"FCA11 Brake Shaping (Beta)", "Dynamic Brakes"}
      self.assertEqual(expected, {t for t, r in rows.items() if not r["visible"]}, combo)

  def test_the_stop_hold_row_is_shown_only_while_its_value_is_on(self):
    """R4: at its default 0 the 0034 row stays out of the way in all 16 states; with the value ON (a backup restore
    from the route-195 era) it is LISTED and settable offroad, so the owner has the switch the review asked for."""
    for combo, rows in self.matrix["combos"].items():
      self.assertFalse(rows["Hold Clutch Release (experimental)"]["visible"], combo)
    for combo, row in self.matrix["hold_on"].items():
      brake = bool(int(combo[0]))
      self.assertEqual(brake, row["visible"], f"hold value ON at {combo}: the row must be listed iff Braking is on")
      self.assertEqual(brake, row["enabled"], f"hold value ON at {combo}: settable offroad")
    # N6 (Opus): the row at value ON while ONROAD - visible (its rule is visibility) and NOT settable. Enabling it on
    # visibility alone (`set_enabled(hold_state.visible)`) would let the owner flip a CarParams key while driving.
    self.assertTrue(self.matrix["hold_on_onroad"]["visible"], "the hold row must stay LISTED onroad while its value is ON")
    self.assertFalse(self.matrix["hold_on_onroad"]["enabled"], "the hold row must NOT be settable onroad")

  def test_the_brake_rows_are_listed_and_settable_offroad_in_every_combination(self):
    for combo, rows in self.matrix["combos"].items():
      brake = int(combo[0])
      for row in BRAKE_ROWS:
        want = bool(brake) or row == "FCA11 Shadow (Log-Only)"
        self.assertEqual(want, rows[row]["visible"], f"{row} at {combo}")
        self.assertEqual(want, rows[row]["enabled"], f"{row} not settable offroad at {combo}")

  def test_onroad_hides_nothing_extra_and_disables_the_editable_rows(self):
    rows = self.matrix["onroad"]
    self.assertEqual(HIDDEN_ROWS, {t for t, r in rows.items() if not r["visible"]})
    for title, row in rows.items():
      if title in HIDDEN_ROWS or title == "Drive Mode Personality":
        continue
      self.assertFalse(row["enabled"], f"{title} is settable onroad")

  def test_update_settings_writes_no_param_and_only_the_tuning_handler_does(self):
    """R3 (F4): `update_settings` writes NOTHING - all 16 combinations plus onroad. Exactly ONE row callback writes:
    the Custom Longitudinal Tuning row's own button handler, which puts the chosen index into
    HyundaiLongitudinalTuning - its documented job, unchanged by this patch. The probe calls it now instead of
    skipping it (Opus: 'every row callback writes 0 params' was wrong)."""
    self.assertEqual([], self.matrix["update_writes"])
    self.assertEqual([["HyundaiLongitudinalTuning", "2"]], self.matrix["callback_writes"])


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
