"""fork (0045 v2): the FCA11 Shadow post-send hook (fork/fca11_shadow_hook.py) + its wiring in card.py.

The hook runs V3 strictly AFTER sendcan is published (V3-fault isolation) and there is NO 0x38D send filter any more (Shadow
keeps the OLD-LAW frames on the wire, exactly as today)."""
import re
import unittest
from pathlib import Path
from types import SimpleNamespace

from openpilot.sunnypilot.fork.fca11_shadow_hook import fca11_shadow_post_send

REPO = Path(__file__).resolve().parents[4]
CARD = REPO / "openpilot" / "selfdrive" / "car" / "card.py"


class TestShadowHook(unittest.TestCase):
  def test_calls_the_car_layer_post_send(self):
    calls = []
    fca11_shadow_post_send(SimpleNamespace(CC=SimpleNamespace(fca11_post_send=lambda: calls.append(1))))
    self.assertEqual([1], calls)

  def test_noop_without_the_method(self):
    fca11_shadow_post_send(SimpleNamespace(CC=SimpleNamespace()))
    fca11_shadow_post_send(SimpleNamespace())

  def test_never_raises(self):
    def boom():
      raise RuntimeError("V3 exploded")
    fca11_shadow_post_send(SimpleNamespace(CC=SimpleNamespace(fca11_post_send=boom)))


class TestCardWiring(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.src = CARD.read_text(encoding="utf-8")

  def test_hook_runs_after_the_sendcan_publish(self):
    m = re.search(r"self\.CI\.apply\(CC, convert_carControlSP\(CC_SP\), now_nanos\)\n(.*?)\n\n", self.src, re.S)
    self.assertIsNotNone(m)
    body = m.group(1)
    i_send = body.find("self.pm.send('sendcan', can_list_to_can_capnp(can_sends")
    i_hook = body.find("fca11_shadow_post_send(self.CI)")
    self.assertGreaterEqual(i_send, 0)
    self.assertGreater(i_hook, i_send, "V3 must run AFTER the OLD-LAW frames are published")

  def test_no_send_filter_between_apply_and_sendcan(self):
    """Shadow keeps OLD-LAW on the wire: nothing may rewrite can_sends between CI.apply and the publish."""
    m = re.search(r"self\.CI\.apply\(CC, convert_carControlSP\(CC_SP\), now_nanos\)\n(.*?)self\.pm\.send\('sendcan'", self.src, re.S)
    self.assertIsNotNone(m)
    self.assertNotIn("can_sends =", m.group(1))
    self.assertNotIn("fca11_shadow_filter", self.src)


if __name__ == "__main__":
  unittest.main()
