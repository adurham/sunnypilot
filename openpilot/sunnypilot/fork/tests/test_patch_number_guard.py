"""
Guard test for the duplicate-patch-number check (``check_patch_numbers.py``).

Regression for a real collision: a DEPLOYED ``0029-...cal-command-mode`` and a HELD
``0029-...brake-shaping`` coexisted in the brake story and caused a briefing error.
This asserts the guard PASSes on the shipped series and FAILs on a deliberately
duplicated entry (and on a series patch that lost its ``NNNN-`` prefix).
"""
import tempfile
import unittest
from pathlib import Path

from openpilot.sunnypilot.fork.tests import check_patch_numbers as G


def _mk(dirpath, *names):
  d = Path(dirpath)
  for n in names:
    (d / n).write_text("x\n")
  return d


class TestPatchNumberGuard(unittest.TestCase):

  def test_real_series_is_clean(self):
    series = G._default_series()
    self.assertTrue(series.is_dir(), series)
    nums = G.find_patch_numbers([series], require_numbered=True)
    prefixes = [p for p, _ in nums]
    self.assertEqual(len(prefixes), len(set(prefixes)), "a duplicate prefix slipped through")
    self.assertGreater(len(prefixes), 0)

  def test_duplicate_prefix_fails(self):
    with tempfile.TemporaryDirectory() as t:
      d = _mk(t, "0029-hyundai-fca11-cal-command-mode.patch",
                 "0029-hyundai-fca11-brake-shaping.patch")
      with self.assertRaises(ValueError) as cm:
        G.find_patch_numbers([d], require_numbered=True)
      self.assertIn("DUPLICATE", str(cm.exception))

  def test_missing_prefix_in_series_fails(self):
    with tempfile.TemporaryDirectory() as t:
      d = _mk(t, "0001-ok.patch", "brake-shaping-HELD.patch")
      with self.assertRaises(ValueError):
        G.find_patch_numbers([d], require_numbered=True)

  def test_held_dir_allows_unprefixed(self):
    with tempfile.TemporaryDirectory() as t:
      d = _mk(t, "brake-shaping-v1-HELD.patch")
      self.assertEqual(G.find_patch_numbers([d], require_numbered=False), [])

  def test_held_dir_still_catches_internal_dup(self):
    with tempfile.TemporaryDirectory() as t:
      d = _mk(t, "0077-a.patch", "0077-b.patch")
      with self.assertRaises(ValueError):
        G.find_patch_numbers([d], require_numbered=False)

  def test_cli_exit_codes(self):
    with tempfile.TemporaryDirectory() as t:
      d = _mk(t, "0012-a.patch", "0012-b.patch")
      self.assertEqual(G.main(["--series", str(d)]), 1)
    self.assertEqual(G.main(["--series", str(G._default_series())]), 0)


if __name__ == "__main__":
  unittest.main()
