"""2026-11-03 (routes 190/191): the MEASURED FCA11 plant curve is the level the dynamic-brakes (ON) path requests.

Sustained DecCmd plateaus on this car: 4 LSB -> 1.14, 8 -> 1.27, 13 -> 1.49, 20 -> 1.57 m/s^2. ON, the law's target is the
inverse of that curve (saturating at 20 LSB), so the level it RAMPS TO is bounded by what the ask measurably buys - not a
post-hoc clamp at the send site. The one deliberate override is the existing urgent lane (planner ask <=
FCA11_URGENT_ASK_MSS2, the owner's 0.5 g ruling, route 190): it may spend the analytic map up to the raised gate and only
ever ADDS to the curve level. OFF is the deployed law byte-for-byte (pinned in test_fca11_long.py).
"""
import unittest

from opendbc.car import structs
from opendbc.sunnypilot.car.hyundai import fca11_long as fl
from opendbc.sunnypilot.car.hyundai.tests.test_fca11_long import _CS, _cam, _cc, _cp_sp, _decs


def _run(dyn, ask, v=15.0, frames=400):
  """Drive the REAL controller (Fca11LongBrake.update -> the FCA11 send path) with a constant ask; return sent levels."""
  b = fl.Fca11LongBrake(structs.CarParams(), _cp_sp(dyn=dyn))
  out = []
  for f in range(frames):
    out += _decs(b.update(_cc(ask), _CS(v_ego=v, cam=_cam()), f))
  return b, out


class TestPlantCurveInverse(unittest.TestCase):
  def test_measured_points(self):
    self.assertEqual({1.14: 4, 1.27: 8, 1.49: 13, 1.57: 20},
                     {a: fl.dec_cmd_from_plant_curve(a) for a in (1.14, 1.27, 1.49, 1.57)})

  def test_monotone_and_saturated(self):
    xs = [i * 0.01 for i in range(0, 501)]
    ys = [fl.dec_cmd_from_plant_curve(x) for x in xs]
    self.assertTrue(all(b >= a for a, b in zip(ys, ys[1:])), "non-decreasing in the ask")
    self.assertEqual(fl.FCA11_PLANT_LSB[0], ys[0])
    self.assertEqual(20, max(ys), "saturates at the top measured point")
    # the accel-domain wrapper the ON law uses: coast above onset, speed floor, monotone, saturated
    self.assertEqual(0, fl.dec_cmd_from_accel_plant(-0.3))
    asks = [-0.5 - i * 0.01 for i in range(400)]
    lv = [fl.dec_cmd_from_accel_plant(a) for a in asks]
    self.assertTrue(all(b >= a for a, b in zip(lv, lv[1:])))
    self.assertEqual(20, max(lv))
    self.assertEqual(fl.FCA11_NORESPONSE_MIN_LSB, fl.dec_cmd_from_accel_plant(-0.6, 0.2), "stop-band floor kept")


class TestOnPathRequestsTheCurve(unittest.TestCase):
  """Through Fca11LongBrake.update: the level the ON path converges to IS the curve level for the ask."""

  def test_mid_ask_converges_to_the_curve_level(self):
    want = fl.dec_cmd_from_plant_curve(1.3)
    self.assertIn(want, (8, 9))
    for v in (15.0, 5.0):
      _, out = _run(True, -1.3, v)
      self.assertEqual(want, out[-1], f"v={v}: steady state is the curve level")
      self.assertLessEqual(max(out), want, f"v={v}: the ramp never overshoots the curve level")

  def test_hard_ask_below_the_urgent_lane_saturates_at_20(self):
    for ask in (-1.6, -2.0, -2.3):
      for v in (15.0, 5.0):
        b, out = _run(True, ask, v)
        self.assertFalse(b._feel_in_hold_band)
        self.assertEqual(20, out[-1], f"ask={ask} v={v}: saturates at the curve top")
        self.assertLessEqual(max(out), 20, f"ask={ask} v={v}: never above the curve-saturated level")

  def test_target_itself_is_curve_bounded(self):
    """Root cause, not a clamp: the law's own target (what the ramp climbs toward) is the curve level."""
    b = fl.Fca11LongBrake(structs.CarParams(), _cp_sp(dyn=True))
    b._planner_ask = -2.0
    for ask in (-0.7, -1.0, -1.3, -1.5, -2.0):
      self.assertEqual(min(fl.personality_max_dec(fl.PERSONALITY_STANDARD), fl.dec_cmd_from_accel_plant(ask, 15.0)),
                       b._feel_target(ask, ask, fl.PERSONALITY_STANDARD, 1.0, 15.0), ask)

  def test_urgent_lane_is_the_deliberate_override(self):
    """-3.0 is at/below FCA11_URGENT_ASK_MSS2 (-2.4): the owner-ruled brake-authority lane keeps the deployed level (26),
    so ON never brakes LESS than OFF on a genuine hard stop (route 190: 20 LSB let a lead close to 2.9 m)."""
    self.assertLessEqual(-3.0, fl.FCA11_URGENT_ASK_MSS2)
    for v in (15.0, 5.0):
      _, on = _run(True, -3.0, v)
      _, off = _run(False, -3.0, v)
      self.assertEqual(off[-1], on[-1])
      self.assertGreaterEqual(on[-1], 20)
      self.assertLessEqual(max(on), fl.HYUNDAI_FCA11_LONG_MAX_DEC)

  def test_relaxed_tier_still_caps(self):
    b = fl.Fca11LongBrake(structs.CarParams(), _cp_sp(dyn=True))
    out = []
    for f in range(400):
      out += _decs(b.update(_cc(-2.0), _CS(v_ego=15.0, cam=_cam()), f, fl.PERSONALITY_RELAXED))
    self.assertEqual(fl.personality_max_dec(fl.PERSONALITY_RELAXED), out[-1])

  def test_sweep_on_is_monotone_and_never_below_the_curve_or_above_the_gate(self):
    prev = 0
    for i in range(0, 36):
      ask = -0.7 - i * 0.1
      _, on = _run(True, ask, frames=300)
      lvl = on[-1]
      self.assertGreaterEqual(lvl, prev, f"steady level monotone in the ask at {ask:.2f}")
      prev = lvl
      if ask > fl.FCA11_URGENT_ASK_MSS2:
        self.assertEqual(fl.dec_cmd_from_accel_plant(ask), lvl, f"{ask:.2f}: the curve level")
      else:
        self.assertGreaterEqual(lvl, 20)
      self.assertLessEqual(max(on), fl.HYUNDAI_FCA11_LONG_MAX_DEC)

  def test_off_path_untouched(self):
    for ask, want in ((-1.3, 9), (-2.0, 16), (-3.0, 26)):
      _, off = _run(False, ask)
      self.assertEqual(want, off[-1], f"OFF {ask}: the deployed level")


class TestFlatSegmentNoChatter(unittest.TestCase):
  """Fable review: 13 -> 20 LSB buys only 1.49 -> 1.57 m/s^2, so the raw inverse of an ask hovering at -1.5 walks the level
  with every planner wiggle (pre-fix, through update(): +/-0.05 flipped 17/18 EVERY sent frame, a 0.1 m/s^2 1 Hz wobble swept
  12..20). ON now runs the curve level through an ask-domain backlash (FCA11_PLANT_HYST_MSS2)."""

  @staticmethod
  def _hover(dyn, ask_fn, frames=800, v=15.0):
    b = fl.Fca11LongBrake(structs.CarParams(), _cp_sp(dyn=dyn))
    out = []
    for f in range(frames):
      out += _decs(b.update(_cc(ask_fn(f)), _CS(v_ego=v, cam=_cam()), f))
    return out

  def test_backlash_pure_function(self):
    h = fl.FCA11_PLANT_HYST_MSS2
    self.assertEqual(fl.dec_cmd_from_plant_curve(1.5), fl.dec_cmd_from_plant_curve_hyst(1.5, None), "no history = plain inverse")
    for ref in range(4, 21):
      for i in range(0, 200):
        w = 1.0 + i * 0.005
        y = fl.dec_cmd_from_plant_curve_hyst(w, ref)
        self.assertLessEqual(fl.dec_cmd_from_plant_curve(w - h), y)
        self.assertGreaterEqual(fl.dec_cmd_from_plant_curve(w + h), y)
    # a reversal inside the window never moves the level
    lvl = fl.dec_cmd_from_plant_curve(1.5)
    for w in (1.45, 1.55, 1.46, 1.54, 1.5):
      lvl2 = fl.dec_cmd_from_plant_curve_hyst(w, lvl)
      self.assertEqual(lvl, lvl2, w)

  def test_hover_around_minus_1p5_is_bounded(self):
    import math
    import random
    rng = random.Random(7)
    noise = [rng.uniform(-0.08, 0.08) for _ in range(800)]
    cases = {
      "alt +/-0.05 every sent frame": lambda f: -1.5 + (0.05 if (f // 2) % 2 else -0.05),
      "alt +/-0.08 every sent frame": lambda f: -1.5 + (0.08 if (f // 2) % 2 else -0.08),
      "sine 0.10 @ 1 Hz": lambda f: -1.5 + 0.10 * math.sin(2 * math.pi * f / 100),
      "sine 0.06 @ 2 Hz": lambda f: -1.5 + 0.06 * math.sin(2 * math.pi * f / 50),
      "uniform noise +/-0.08": lambda f: -1.5 + noise[f],
    }
    for name, fn in cases.items():
      tail = self._hover(True, fn)[200:]   # past the onset ramp
      steps = [abs(b - a) for a, b in zip(tail, tail[1:])]
      self.assertLessEqual(max(steps), 1, f"{name}: per-sent-frame step")
      self.assertLessEqual(max(tail) - min(tail), 5, f"{name}: hover span (pre-fix up to 8)")
    # within the play the level is CONSTANT
    for name in ("alt +/-0.05 every sent frame", "sine 0.06 @ 2 Hz"):
      tail = self._hover(True, cases[name])[200:]
      self.assertEqual(1, len(set(tail)), f"{name}: a wiggle inside +/-HYST never moves the level")

  def test_a_real_trend_is_still_followed(self):
    """The play lags a trend by HYST, it does not stop it: a slow -1.3 -> -1.7 ramp still reaches the curve top."""
    out = self._hover(True, lambda f: -1.3 - 0.4 * min(1.0, f / 400), frames=700)
    self.assertEqual(20, out[-1])
    out = self._hover(True, lambda f: -1.7 + 0.4 * min(1.0, f / 400), frames=900)
    self.assertEqual(fl.dec_cmd_from_plant_curve(1.3 + fl.FCA11_PLANT_HYST_MSS2), out[-1], "an easing settles at the window edge")

  def test_off_path_never_touches_the_backlash(self):
    import unittest.mock as um
    with um.patch.object(fl, "dec_cmd_from_plant_curve_hyst", side_effect=AssertionError("OFF used the ON backlash")):
      out = self._hover(False, lambda f: -1.5 + (0.05 if (f // 2) % 2 else -0.05), frames=300)
    self.assertTrue(out)


if __name__ == "__main__":
  unittest.main()
