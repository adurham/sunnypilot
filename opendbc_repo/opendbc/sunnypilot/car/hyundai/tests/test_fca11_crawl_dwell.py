"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

cycle 1: the SLOW-CRAWL dwell escape (route 167 seg 9 under-brake). Pins:
  * below FCA11_FEEL_DWELL_SLOW_MPS a sustained ask <= -0.5 opens after FCA11_FEEL_DWELL_SLOW_FRAMES (not one frame sooner);
  * at/above the limit the deployed 1.0 s dwell is unchanged (no opening moves at speed);
  * the slow counter is STRICT: frames accumulated at speed never count, a lift above -0.5 or any `ok` veto resets it;
  * the seg-9 ask shape (74 frames <= -0.5, a 53-frame shallow lift to -0.46, 68 frames <= -0.5) now opens while crawling;
  * FEEL off: no crawl escape at all (pre-0049 law untouched); disengage clears the counter.
"""
import unittest

from opendbc.sunnypilot.car.hyundai import fca11_long as fl
from opendbc.sunnypilot.car.hyundai.tests.test_fca11_feel import _brake, _cc
from opendbc.sunnypilot.car.hyundai.tests.test_fca11_long import _CS, _cam

SLOW = fl.FCA11_FEEL_DWELL_SLOW_FRAMES
DWELL = fl.FCA11_FEEL_DWELL_FRAMES
CRAWL = 1.3     # m/s: seg 9 crept 1.13 -> 1.49 (4.1-5.4 km/h, above the 3 km/h hold band entry)


def _first_open(b, asks, v_fn, start=0, gas_fn=None):
  """Frame index of the first frame the controller is braking, else None."""
  for i, a in enumerate(asks):
    f = start + i
    b.update(_cc(a), _CS(v_ego=v_fn(f), cam=_cam(), gas=bool(gas_fn and gas_fn(f))), f)
    if b.episode_open:
      return f
  return None


class TestCrawlDwell(unittest.TestCase):

  def test_constants(self):
    self.assertEqual(70, SLOW)
    self.assertEqual(100, DWELL, "the at-speed dwell is unchanged")
    self.assertEqual(2.0, fl.FCA11_FEEL_DWELL_SLOW_MPS)

  def test_sustained_small_ask_opens_after_the_crawl_dwell(self):
    b = _brake(True)
    self.assertEqual(SLOW - 1, _first_open(b, [-0.55] * 150, lambda f: CRAWL))

  def test_at_and_above_the_limit_the_deployed_dwell_is_unchanged(self):
    for v in (fl.FCA11_FEEL_DWELL_SLOW_MPS, 2.5, 5.0, 12.0, 25.0):
      with self.subTest(v=v):
        b = _brake(True)
        self.assertEqual(DWELL - 1, _first_open(b, [-0.55] * 150, lambda f, v=v: v))

  def test_counter_is_strict_frames_at_speed_never_count(self):
    # 50 frames at 3 m/s then crawl: the slow counter starts at frame 50 -> it would reach 70 at 119, the 1.0 s dwell
    # reaches 100 first (frame 99). A non-strict scoped dwell (speed checked only at the open) would open at frame 69.
    b = _brake(True)
    self.assertEqual(DWELL - 1, _first_open(b, [-0.55] * 150, lambda f: 3.0 if f < 50 else CRAWL))

  def test_crossing_the_limit_resets(self):
    # 60 slow frames, one at 2.0 m/s, then slow again: reopens only after a fresh 70 (or the 1.0 s dwell at 99).
    b = _brake(True)
    self.assertEqual(DWELL - 1, _first_open(b, [-0.55] * 150, lambda f: 2.0 if f == 60 else CRAWL))
    b = _brake(True)
    self.assertEqual(61 + SLOW - 1, _first_open(b, [-0.55] * 60 + [-0.40] + [-0.55] * 150, lambda f: CRAWL))

  def test_lift_above_onset_resets(self):
    b = _brake(True)
    asks = ([-0.55] * (SLOW - 1) + [-0.49]) * 4
    self.assertIsNone(_first_open(b, asks, lambda f: CRAWL))
    self.assertEqual(0, b._feel_dwell_slow)

  def test_ok_veto_resets(self):
    b = _brake(True)
    self.assertIsNone(_first_open(b, [-0.55] * 150, lambda f: CRAWL, gas_fn=lambda f: True))
    self.assertEqual(0, b._feel_dwell_slow)
    # a gas tap mid-count: hold-off (GAS_HOLDOFF_FRAMES) then a fresh 70
    b = _brake(True)
    self.assertEqual(41 + fl.GAS_HOLDOFF_FRAMES + SLOW - 1,
                     _first_open(b, [-0.55] * 300, lambda f: CRAWL, gas_fn=lambda f: f == 40))

  def test_seg9_shape_opens_while_crawling(self):
    # the logged 167 seg-9 ask: 74 frames -0.56..-0.50, 53 frames -0.50..-0.46, 68 frames -0.51..-0.62 (14 <= -0.60)
    asks = [-0.52] * 74 + [-0.48] * 53 + [-0.55] * 54 + [-0.61] * 14

    def v(f):
      return 1.25 + 0.25 * f / len(asks)
    b = _brake(True)
    self.assertEqual(SLOW - 1, _first_open(b, asks, v), "opens inside the first 74-frame run (log: 15:50:37.99)")
    self.assertTrue(b.episode_open)
    # the deployed gate never opened here (74 < 100, 68 < 100, 14 < 20 at <= -0.60)
    prev = fl.FCA11_FEEL_DWELL_SLOW_FRAMES
    fl.FCA11_FEEL_DWELL_SLOW_FRAMES = 10 ** 9
    try:
      self.assertIsNone(_first_open(_brake(True), asks, v))
    finally:
      fl.FCA11_FEEL_DWELL_SLOW_FRAMES = prev

  def test_episode_holds_through_the_shallow_lift(self):
    # once open, the deployed release rule (-0.30 + min-on) keeps it open across the -0.48 lift
    asks = [-0.52] * 74 + [-0.48] * 53 + [-0.55] * 68
    b = _brake(True)
    _first_open(b, asks[:SLOW], lambda f: CRAWL)
    for i, a in enumerate(asks[SLOW:], start=SLOW):
      b.update(_cc(a), _CS(v_ego=CRAWL, cam=_cam()), i)
      self.assertTrue(b.episode_open, i)

  def test_feel_off_has_no_crawl_escape(self):
    b = _brake(False)
    _first_open(b, [-0.40] * 150, lambda f: CRAWL)    # above -0.5: the pre-0049 law never opens, neither does the escape
    self.assertFalse(b.episode_open)
    self.assertEqual(0, b._feel_dwell_slow)
    b = _brake(False)
    _first_open(b, [-0.55] * 150, lambda f: CRAWL)     # the pre-0049 law opens at 0.2 s; the crawl counter never runs
    self.assertTrue(b.episode_open)
    self.assertEqual(0, b._feel_dwell_slow)

  def test_disengage_clears_the_counter(self):
    b = _brake(True)
    _first_open(b, [-0.55] * (SLOW - 5), lambda f: CRAWL)
    self.assertEqual(SLOW - 5, b._feel_dwell_slow)
    b.update(_cc(-0.55, long_active=False), _CS(v_ego=CRAWL, cam=_cam()), SLOW)
    self.assertEqual(0, b._feel_dwell_slow)
    self.assertIsNone(_first_open(b, [-0.55] * (SLOW - 1), lambda f: CRAWL, start=SLOW + 1))
    self.assertFalse(b.episode_open)
    self.assertEqual(2 * SLOW, _first_open(b, [-0.55], lambda f: CRAWL, start=2 * SLOW), "a fresh 70 after re-engage")

  def test_feature_disable_clears_the_counter(self):
    b = _brake(True)
    _first_open(b, [-0.55] * (SLOW - 5), lambda f: CRAWL)
    b.enabled = False                                     # the toggle goes off (as test_fca11_long does)
    b.update(_cc(-0.55), _CS(v_ego=CRAWL, cam=_cam()), SLOW)
    self.assertEqual(0, b._feel_dwell_slow)
    b.enabled = True
    self.assertIsNone(_first_open(b, [-0.55] * (SLOW - 1), lambda f: CRAWL, start=SLOW + 1))


if __name__ == "__main__":
  unittest.main()
