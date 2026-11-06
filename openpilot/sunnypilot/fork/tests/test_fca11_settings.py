"""fork (0044): the PURE visibility/enable rules behind the Hyundai FCA11 settings toggles (fork/fca11_settings.py).

The raylib panel cannot run headless; this pins the decision it delegates to, plus a static check that the panel
actually wires the shaping toggle to HyundaiFca11AffineGain through these functions."""
import itertools
import re
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
    """SETTINGS TIDY: the 0034 row is hidden on every car - the pair was never carried on the wire and the TCU does not
    act on 0x38D (FORK.md rows 130/137). The rule keeps the pedal/brake arguments so the row comes back by reverting it."""
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
    self.assertIn("Hard stops are NOT", self.src)
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
    self.assertIn("V3 NEVER acts on the car", self.src)
    self.assertIn("Data collection only", self.src)
    self.assertIn("The car brakes exactly as today", self.src)
    self.assertIn("(OLD-LAW)", self.src)
    self.assertNotIn("you brake for every stop", self.src)   # the first build's FCA11-DISABLED reading is gone

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
    self.assertIn("FORK.md rows 130/137", self.src)   # how to bring it back is written next to the row


if __name__ == "__main__":
  unittest.main()
