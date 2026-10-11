"""fork (0045): the superproject's last Shadow gate (fork/fca11_shadow_guard.py) + its wiring in card.py.

In FCA11 SHADOW (CP_SP.fca11Shadow) no host 0x38D may ever reach sendcan. The car layer already returns none; card.py
filters every sendcan list through fca11_shadow_filter right before publishing (belt and braces). With Shadow OFF the
filter returns the SAME list object (identity: the non-Shadow path is untouched)."""
import re
import unittest
from pathlib import Path
from types import SimpleNamespace

from opendbc.car.can_definitions import CanData
from openpilot.sunnypilot.fork.fca11_shadow_guard import Fca11ShadowStats, fca11_shadow_filter

REPO = Path(__file__).resolve().parents[4]
CARD = REPO / "openpilot" / "selfdrive" / "car" / "card.py"


def _sends():
  return [CanData(0x340, b"\x01" * 8, 0), CanData(0x38D, b"\x02" * 8, 0), (0x200, b"\x03" * 6, 0), (0x38D, b"\x04" * 8, 0)]


class TestShadowFilter(unittest.TestCase):
  def test_shadow_on_drops_every_0x38d_and_counts(self):
    st = Fca11ShadowStats()
    out = fca11_shadow_filter(SimpleNamespace(fca11Shadow=True), _sends(), st)
    self.assertEqual([0x340, 0x200], [m[0] for m in out])
    self.assertEqual(2, st.dropped)
    self.assertTrue(st.warned)

  def test_shadow_off_is_identity(self):
    s = _sends()
    st = Fca11ShadowStats()
    self.assertIs(s, fca11_shadow_filter(SimpleNamespace(fca11Shadow=False), s, st))
    self.assertIs(s, fca11_shadow_filter(SimpleNamespace(), s, st))          # older struct without the field
    self.assertEqual(0, st.dropped)

  def test_shadow_on_clean_stream_unchanged(self):
    s = [m for m in _sends() if m[0] != 0x38D]
    st = Fca11ShadowStats()
    self.assertEqual(s, fca11_shadow_filter(SimpleNamespace(fca11Shadow=True), s, st))
    self.assertEqual(0, st.dropped)


class TestCardWiring(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.src = CARD.read_text(encoding="utf-8")

  def test_filter_sits_between_apply_and_sendcan(self):
    m = re.search(r"self\.CI\.apply\(CC, convert_carControlSP\(CC_SP\), now_nanos\)\n(.*?)self\.pm\.send\('sendcan', "
                  r"can_list_to_can_capnp\(can_sends", self.src, re.S)
    self.assertIsNotNone(m, "the sendcan publish must follow CI.apply")
    self.assertIn("can_sends = fca11_shadow_filter(self.CP_SP, can_sends, self._fca11_shadow_stats)", m.group(1))

  def test_stats_constructed(self):
    self.assertIn("self._fca11_shadow_stats = Fca11ShadowStats()", self.src)


if __name__ == "__main__":
  unittest.main()
