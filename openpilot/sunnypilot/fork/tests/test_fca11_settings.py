"""Tests for the consolidated Hyundai page: pure row rules + static panel wiring.

The old suite pinned the pre-0046 page (eleven peer switches, the four-law prose, the old key
names).  That page is gone by design; this suite pins the replacement: one mode, grouped rows,
one plain-English sentence, experimental behind a reveal, and nothing that can act on the car while
its path is not selected or while the car is onroad.
"""
import ast
import re
import unittest
from pathlib import Path

from openpilot.sunnypilot.fork import fca11_settings as fs
from openpilot.sunnypilot.fork import hyundai_settings as hs

REPO = Path(__file__).resolve().parents[4]
PANEL = REPO / "openpilot" / "selfdrive" / "ui" / "sunnypilot" / "layouts" / "settings" / "vehicle" / "brands" / "hyundai.py"
KEYS_SRC = REPO / "openpilot" / "common" / "params_keys.h"


class TestRowRules(unittest.TestCase):

  def test_mode_selector_is_offroad_only(self):
    self.assertEqual(fs.ToggleState(True, True), fs.mode_toggle_state(True))
    self.assertEqual(fs.ToggleState(True, False), fs.mode_toggle_state(False))

  def test_path_rows_are_visible_but_greyed_when_the_path_is_off(self):
    self.assertEqual(fs.ToggleState(True, True), fs.path_toggle_state(True, True))
    self.assertEqual(fs.ToggleState(True, False), fs.path_toggle_state(False, True))
    self.assertEqual(fs.ToggleState(True, False), fs.path_toggle_state(True, False))

  def test_tuning_only_in_stock_mode(self):
    self.assertEqual(fs.ToggleState(True, True), fs.tuning_toggle_state(hs.MODE_STOCK, True, True, True))
    self.assertEqual(fs.ToggleState(False, False), fs.tuning_toggle_state(hs.MODE_PEDAL, True, True, True))
    self.assertEqual(fs.ToggleState(False, False), fs.tuning_toggle_state(hs.MODE_PEDAL_FCA11, True, True, True))
    self.assertEqual(fs.ToggleState(True, False), fs.tuning_toggle_state(hs.MODE_STOCK, True, True, False))
    self.assertEqual(fs.ToggleState(True, False), fs.tuning_toggle_state(hs.MODE_STOCK, True, False, True))
    self.assertEqual(fs.ToggleState(False, False), fs.tuning_toggle_state(hs.MODE_STOCK, False, True, True))

  def test_experimental_section_needs_the_pedal(self):
    self.assertEqual(fs.ToggleState(False, False), fs.experimental_header_state(False))
    self.assertEqual(fs.ToggleState(True, True), fs.experimental_header_state(True))
    self.assertEqual(fs.ToggleState(False, False), fs.show_experimental_toggle_state(False))
    self.assertEqual(fs.ToggleState(True, True), fs.show_experimental_toggle_state(True))

  def test_experimental_row_reveal_and_stored_on(self):
    # Hidden at its default with the reveal closed.
    self.assertEqual(fs.ToggleState(False, False), fs.experimental_row_state(True, True, False, "0", True))
    # The reveal shows it.
    self.assertEqual(fs.ToggleState(True, True), fs.experimental_row_state(True, True, True, "0", True))
    # A stored ON is always shown, even with the reveal closed (backup restore case).
    self.assertEqual(fs.ToggleState(True, True), fs.experimental_row_state(True, True, False, "1", True))
    # Prerequisite (brake for stop-hold) still applies.
    self.assertEqual(fs.ToggleState(False, False), fs.experimental_row_state(True, False, True, "1", True))
    # Offroad only.
    self.assertEqual(fs.ToggleState(True, False), fs.experimental_row_state(True, True, True, "0", False))
    # No pedal, no section.
    self.assertEqual(fs.ToggleState(False, False), fs.experimental_row_state(False, True, True, "1", True))

  def test_no_row_is_enabled_onroad(self):
    for state in (fs.mode_toggle_state(False), fs.path_toggle_state(True, False),
                  fs.tuning_toggle_state(hs.MODE_STOCK, True, True, False),
                  fs.experimental_row_state(True, True, True, "0", False)):
      self.assertFalse(state.enabled)


class TestPanel(unittest.TestCase):

  @classmethod
  def setUpClass(cls):
    cls.src = PANEL.read_text(encoding="utf-8")
    m = re.search(r"self\.items = \[(.*?)\]", cls.src, re.S)
    cls.items = m.group(1) if m else ""
    cls.keys_src = KEYS_SRC.read_text(encoding="utf-8")

  def test_mode_selector_is_the_master_and_writes_the_new_key(self):
    self.assertIn('tr("Longitudinal Mode")', self.src)
    self.assertIn("callback=self._on_mode_selected", self.src)
    self.assertIn("ui_state.params.put(MODE_KEY, int(index))", self.src)
    for legacy in ('param="HyundaiGasInterceptor"', 'param="HyundaiFca11Brake"'):
      self.assertNotIn(legacy, self.src)

  def test_shaping_is_the_renamed_key_and_old_key_has_no_row(self):
    self.assertIn('tr("Smooth FCA11 Braking")', self.src)
    self.assertIn("ui_state.params.put(SHAPING_KEY, bool(state))", self.src)
    self.assertNotIn('param="HyundaiFca11AffineGain"', self.src)

  def test_groups_are_present(self):
    for title in ("Longitudinal", "Following and stopping", "FCA11 braking", "Pedal", "Experimental"):
      self.assertIn(f'tr("{title}")', self.src)

  def test_experimental_rows_are_behind_the_reveal(self):
    self.assertIn('tr("Show experimental")', self.src)
    self.assertIn("experimental_row_state(pedal, brake, self.show_experimental", self.src)
    self.assertIn("experimental_row_state(pedal, True, self.show_experimental", self.src)

  def test_panel_uses_the_pure_rules(self):
    self.assertIn("mode_toggle_state(offroad).enabled", self.src)
    self.assertIn("path_toggle_state(brake, offroad).enabled", self.src)
    self.assertIn("path_toggle_state(pedal, offroad).enabled", self.src)
    self.assertIn("tuning_toggle_state(state.mode, self.alpha_long_available", self.src)

  def test_one_plain_sentence_per_row_no_jargon_or_patch_numbers(self):
    banned = ("LSB", "capnp", "CarParams", "scc_long", "0040", "0042", "0043", "0044", "0045", "(Beta)")
    for word in banned:
      self.assertNotIn(word, self.src, word)
    # No old four-law table / "continuous linear law" prose.
    for phrase in ("continuous linear law", "OLD law", "shaped law", "release ramp"):
      self.assertNotIn(phrase, self.src, phrase)

  def test_every_wired_param_is_registered(self):
    for param in re.findall(r'param="([A-Za-z0-9_]+)"', self.src):
      self.assertRegex(self.keys_src, r'\{"' + param + r'", ', param)

  def test_every_row_is_in_items(self):
    rows = re.findall(r"self\.(\w+) = (?:toggle_item_sp|multiple_button_item_sp)\(", self.src)
    self.assertGreaterEqual(len(rows), 10)
    for attr in rows:
      self.assertIn(f"self.{attr}", self.items, attr)

  def test_no_peer_master_switches_left(self):
    self.assertNotIn('tr("FCA11 Braking (Beta)")', self.src)
    self.assertNotIn('tr("Custom Longitudinal Tuning")', self.src)

  def test_panel_parses(self):
    ast.parse(self.src)


if __name__ == "__main__":
  unittest.main()
