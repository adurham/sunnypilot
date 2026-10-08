"""
0035: the FCA11 cal runner's tool-side hold cap.

The panda no longer bounds an FCA11-long actuation episode (the 2.5 s budget / 3 s cooldown were deleted in opendbc
patch 0035), so the cal TOOL bounds its own holds. The car layer (opendbc cal_mode.CAL_MAX_HOLD_S) refuses to arm an
over-cap rep and runs a watchdog; the plan writer here refuses to WRITE one, so a bad plan is caught off-car first.
"""
import unittest

from opendbc.sunnypilot.car.hyundai import cal_mode
from openpilot.sunnypilot.fork import fca11_cal_runner as R


class TestCalRunnerHoldCap(unittest.TestCase):

  def test_cap_matches_the_car_layer(self):
    self.assertEqual(R.MAX_HOLD_S, cal_mode.CAL_MAX_HOLD_S)

  def test_shipped_matrices_are_inside_the_cap(self):
    for matrix in ("rev1", "rev2"):
      doc = R.build_matrix(matrix, ["steps", "staircase", "releases"])
      self.assertTrue(doc["reps"], matrix)
      for r in doc["reps"]:
        self.assertTrue(0.0 < r["hold_s"] <= R.MAX_HOLD_S, (matrix, r))
        self.assertTrue(cal_mode.hold_ok(r["hold_s"])[0], (matrix, r))

  def test_hold_at_the_cap_is_written(self):
    self.assertEqual(R.MAX_HOLD_S, R._mk_rep("x", "steps", 20, 60.0, hold=R.MAX_HOLD_S)["hold_s"])

  def test_over_cap_or_non_positive_hold_is_refused(self):
    for bad in (R.MAX_HOLD_S + 0.01, 30.0, 0.0, -1.0):
      with self.assertRaises(ValueError, msg=str(bad)):
        R._mk_rep("x", "steps", 20, 60.0, hold=bad)

  def test_dry_run_no_longer_mentions_a_panda_cooldown(self):
    import contextlib
    import io
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
      R.dry_run(["steps"])
    self.assertNotIn("cooldown", buf.getvalue())


if __name__ == "__main__":
  unittest.main()
