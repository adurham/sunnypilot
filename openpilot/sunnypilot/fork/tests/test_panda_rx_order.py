"""
Guard test for ``check_panda_rx_order.py`` (FORK.md row 83).

The panda RX path must hand each frame to ``safety_rx_hook`` BEFORE ``safety_fwd_hook`` decides whether to forward it,
otherwise the first braking-class camera 0x38D of every FCA11 hand-back is dropped. This asserts that:
  * the shipped panda patch, applied to the pinned upstream ``fdcan.h`` text, PASSES;
  * the upstream (pre-fix) order FAILS;
  * the checker cannot be fooled by a call in a comment, a skippable branch, an ``#ifdef``, a duplicated rx call, or
    a source with no forwarding path at all.
The panda submodule may not be checked out in a test environment, so the real-file cases skip when it is absent.
"""
import subprocess
import tempfile
import unittest
from pathlib import Path

from openpilot.sunnypilot.fork.tests import check_panda_rx_order as G

REPO = Path(__file__).resolve().parents[4]
PANDA = REPO / "panda"
PANDA_PATCH_DIR = Path(__file__).resolve().parents[1] / "patches" / "panda"

FIXED = """
static void can_rx(uint8_t can_number) {
  while (x) {
    can_set_checksum(&to_push);
    safety_rx_invalid += safety_rx_hook(&to_push) ? 0U : 1U;
    int bus_fwd_num = safety_fwd_hook(bus_number, to_push.addr);
    if (bus_fwd_num >= 0) { can_send(&to_send, bus_fwd_num, true); }
    #ifdef PANDA_BODY
    body_can_rx(&to_push);
    #endif
    ignition_can_hook(&to_push);
  }
}
"""

UPSTREAM = """
static void can_rx(uint8_t can_number) {
  while (x) {
    // forwarding
    int bus_fwd_num = safety_fwd_hook(bus_number, to_push.addr);
    if (bus_fwd_num >= 0) { can_send(&to_send, bus_fwd_num, true); }
    safety_rx_invalid += safety_rx_hook(&to_push) ? 0U : 1U;
  }
}
"""


def _check(src):
  return G.check_source(src, "t.h")


class TestPandaRxOrderGuard(unittest.TestCase):

  def test_fixed_order_passes(self):
    paths, errors = _check(FIXED)
    self.assertEqual(len(paths), 1)
    self.assertEqual(errors, [])

  def test_upstream_order_fails(self):
    _, errors = _check(UPSTREAM)
    self.assertTrue(any("runs BEFORE" in e for e in errors), errors)

  def test_rx_call_only_in_a_comment_does_not_count(self):
    src = UPSTREAM.replace("// forwarding", "// safety_rx_hook(&to_push); /* safety_rx_hook(x) */")
    _, errors = _check(src)
    self.assertTrue(any("runs BEFORE" in e for e in errors), errors)

  def test_rx_inside_a_branch_fails(self):
    src = FIXED.replace("    safety_rx_invalid += safety_rx_hook(&to_push) ? 0U : 1U;\n",
                        "    if (y) { safety_rx_invalid += safety_rx_hook(&to_push) ? 0U : 1U; }\n")
    _, errors = _check(src)
    self.assertTrue(any("same block" in e for e in errors), errors)

  def test_rx_under_ifdef_fails(self):
    src = FIXED.replace("    safety_rx_invalid += safety_rx_hook(&to_push) ? 0U : 1U;\n",
                        "    #ifdef FOO\n    safety_rx_invalid += safety_rx_hook(&to_push) ? 0U : 1U;\n    #endif\n")
    _, errors = _check(src)
    self.assertTrue(any("#if" in e for e in errors), errors)

  def test_rx_called_twice_fails(self):
    # two safety passes per frame would double-count safety_rx_invalid and double-step every rx state machine
    src = FIXED.replace("    ignition_can_hook(&to_push);\n",
                        "    safety_rx_invalid += safety_rx_hook(&to_push) ? 0U : 1U;\n    ignition_can_hook(&to_push);\n")
    _, errors = _check(src)
    self.assertTrue(any("exactly one" in e for e in errors), errors)

  def test_no_forwarding_path_fails_cli(self):
    with tempfile.TemporaryDirectory() as d:
      f = Path(d) / "x.h"
      f.write_text("static void f(void) { safety_rx_hook(&p); }\n")
      self.assertEqual(G.main(["--file", str(f)]), 1)

  @unittest.skipUnless((PANDA / "board" / "drivers" / "fdcan.h").is_file(), "panda submodule not checked out")
  def test_pinned_upstream_fails_and_shipped_patch_passes(self):
    pin = subprocess.run(["git", "-C", str(REPO), "rev-parse", "HEAD:panda"], capture_output=True, text=True, check=True).stdout.strip()
    upstream = subprocess.run(["git", "-C", str(PANDA), "show", f"{pin}:board/drivers/fdcan.h"],
                              capture_output=True, text=True, check=True).stdout
    _, errors = _check(upstream)
    self.assertTrue(any("runs BEFORE" in e for e in errors), "upstream pin no longer has the defect: drop the panda patch")
    with tempfile.TemporaryDirectory() as d:
      (Path(d) / "board" / "drivers").mkdir(parents=True)
      (Path(d) / "board" / "drivers" / "fdcan.h").write_text(upstream)
      subprocess.run(["git", "init", "-q", d], check=True)
      for p in sorted(PANDA_PATCH_DIR.glob("*.patch")):
        subprocess.run(["git", "-C", d, "apply", str(p)], check=True)
      self.assertEqual(G.main(["--panda", d]), 0)


if __name__ == "__main__":
  unittest.main()
