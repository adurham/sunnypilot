"""fork (0044): the PURE visibility/enable rules behind the Hyundai FCA11 settings toggles (fork/fca11_settings.py).

The raylib panel cannot run headless; this pins the decision it delegates to, plus a static check that the panel
actually wires the shaping toggle to HyundaiFca11AffineGain through these functions."""
import itertools
import re
import unittest
from pathlib import Path

from openpilot.sunnypilot.fork.fca11_settings import ToggleState, fca11_brake_toggle_state, fca11_shaping_toggle_state, \
  fca11_shadow_toggle_state

REPO = Path(__file__).resolve().parents[4]
PANEL = REPO / "openpilot" / "selfdrive" / "ui" / "sunnypilot" / "layouts" / "settings" / "vehicle" / "brands" / "hyundai.py"

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


if __name__ == "__main__":
  unittest.main()
