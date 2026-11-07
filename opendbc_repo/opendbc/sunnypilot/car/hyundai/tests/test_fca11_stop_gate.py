"""WI3: the shaped law's jump to the hold cap waits for CONFIRMED STANDSTILL (FCA11_SHAPE_BAND_* block in fca11_long.py).

Every test drives the real controller update path (Fca11LongBrake.update with a CarState double), on the shaped (ON,
HyundaiFca11AffineGain) instance - the law the logged stops on routes 18a/190 ran.
"""
import os
import unittest

from opendbc.car import structs
import opendbc.sunnypilot.car.hyundai.fca11_long as fl
from opendbc.sunnypilot.car.hyundai.tests.test_fca11_long import _CS, _cam

AGG = fl.PERSONALITY_AGGRESSIVE
STD = fl.PERSONALITY_STANDARD


def _brake():
  cp_sp = structs.CarParamsSP()
  cp_sp.fca11Brake = True
  cp_sp.enableGasInterceptor = True
  cp_sp.fca11AffineGain = True
  return fl.Fca11LongBrake(structs.CarParams(), cp_sp)


def _cc(accel):
  cc = structs.CarControl()
  cc.enabled = True
  cc.longActive = True
  cc.actuators.accel = accel
  return cc


class _Drive:
  """Feeds frames one at a time and records every SENT 0x38D level as (frame, dec)."""
  def __init__(self, personality=AGG):
    self.b = _brake()
    self.f = 0
    self.p = personality
    self.sent = []

  def run(self, n, accel, kph, standstill=False):
    for _ in range(n):
      for s in self.b.update(_cc(accel), _CS(v_ego=kph / 3.6, cam=_cam(), standstill=standstill), self.f, self.p):
        self.sent.append((self.f, fl._get_bits(s.dat, 8, 8)))
      self.f += 1
    return self

  def levels(self, since=0):
    return [d for f, d in self.sent if f >= since]


class TestStopGate(unittest.TestCase):

  def setUp(self):
    self._env_prev = os.environ.pop("FCA11_AFFINE", None)

  def tearDown(self):
    if self._env_prev is not None:
      os.environ["FCA11_AFFINE"] = self._env_prev

  def test_constants_pinned(self):
    self.assertEqual(3.0, fl.FCA11_SHAPE_HOLD_BAND_KPH, "the 3 km/h band edge is NOT moved (reviewer ruling)")
    self.assertEqual(1.5, fl.FCA11_SHAPE_BAND_SS_TIMEOUT_S)
    self.assertEqual(150, fl.FCA11_SHAPE_BAND_SS_TIMEOUT_FRAMES)
    self.assertEqual(0.5, fl.FCA11_SHAPE_BAND_TIMEOUT_RAMP_S)
    self.assertEqual(50, fl.FCA11_SHAPE_BAND_TIMEOUT_RAMP_FRAMES)

  def test_no_jump_to_the_cap_while_rolling_below_3kph_within_the_timeout(self):
    """A stop approaching at 6 km/h then rolling at <= 3 km/h without standstill for just under 1.5 s: the command never
    reaches the cap and never steps by more than the creep-floor take-up."""
    for pers in (AGG, STD):
      cap = fl.personality_max_dec(pers)
      d = _Drive(pers).run(200, -0.73, 6.0)
      pre = d.levels()[-1]
      t0 = d.f
      for kph in (2.9, 2.5, 2.0, 1.5, 1.0, 0.8):
        d.run(24, -0.73, kph)                                      # 6 x 24 = 144 frames < 150
      band = d.levels(t0)
      self.assertTrue(band)
      self.assertLess(max(band), cap, (pers, band))
      self.assertEqual(max(pre, fl.FCA11_CRAWL_MIN_LSB), max(band), (pers, pre, band))
      for a, c in zip([pre] + band, band):
        self.assertLessEqual(c - a, fl.HYUNDAI_FCA11_LONG_RATE_STEP, (pers, band))

  def test_cap_at_standstill(self):
    """At CarState.standstill the cap is taken up at the panda's +RATE_STEP (the deployed take-up) and latched: a
    standstill flag dropping False while still in the band does not pull the hold down."""
    for pers in (AGG, STD):
      cap = fl.personality_max_dec(pers)
      d = _Drive(pers).run(200, -0.73, 6.0).run(40, -0.73, 2.0)
      before = d.levels()[-1]
      self.assertLess(before, cap)
      t0 = d.f
      d.run(40, -0.73, 0.0, standstill=True)
      held = d.levels(t0)
      self.assertEqual(cap, held[-1], (pers, held))
      n = -(-(cap - before) // fl.HYUNDAI_FCA11_LONG_RATE_STEP)
      self.assertEqual(cap, held[n - 1], (pers, held))                 # +RATE_STEP per sent frame, no slower
      for a, c in zip([before] + held, held):
        self.assertLessEqual(c - a, fl.HYUNDAI_FCA11_LONG_RATE_STEP)
      # 17c: standstill flips False for 0.8 s mid-creep (still in the band) -> the cap holds
      t1 = d.f
      d.run(80, -0.73, 0.3, standstill=False)
      self.assertEqual({cap}, set(d.levels(t1)), pers)

  def test_cap_at_standstill_from_rest(self):
    """An episode that opens AT standstill (held at a light) still ramps straight to the cap."""
    d = _Drive(AGG).run(60, -0.73, 0.0, standstill=True)
    self.assertEqual(fl.HYUNDAI_FCA11_LONG_RATE_STEP, d.levels()[0])
    self.assertEqual(30, d.levels()[-1])

  def test_timeout_ramp_starts_after_1p5s_and_reaches_the_cap_in_0p5s(self):
    for pers in (AGG, STD):
      cap = fl.personality_max_dec(pers)
      d = _Drive(pers).run(200, -0.73, 6.0)
      t_in = d.f                                                     # first frame at/below 3 km/h
      d.run(400, -0.73, 2.0)
      sent = [(f - t_in, x) for f, x in d.sent if f >= t_in]
      creep = sent[0][1]
      # nothing above the creep level up to 1.5 s
      self.assertEqual({creep}, {x for t, x in sent if t <= fl.FCA11_SHAPE_BAND_SS_TIMEOUT_FRAMES}, pers)
      first_up = next(t for t, x in sent if x > creep)
      at_cap = next(t for t, x in sent if x >= cap)
      self.assertGreater(first_up, fl.FCA11_SHAPE_BAND_SS_TIMEOUT_FRAMES, pers)
      self.assertLessEqual(first_up, fl.FCA11_SHAPE_BAND_SS_TIMEOUT_FRAMES + 6, pers)
      ramp_frames = at_cap - fl.FCA11_SHAPE_BAND_SS_TIMEOUT_FRAMES
      self.assertGreaterEqual(ramp_frames, fl.FCA11_SHAPE_BAND_TIMEOUT_RAMP_FRAMES - 4, (pers, ramp_frames))
      self.assertLessEqual(ramp_frames, fl.FCA11_SHAPE_BAND_TIMEOUT_RAMP_FRAMES + 4, (pers, ramp_frames))
      # linear, no step: per SENT frame (2 controller frames) <= ceil(2 * (cap - creep) / 50) + 1 LSB
      bound = -(-2 * (cap - creep) // fl.FCA11_SHAPE_BAND_TIMEOUT_RAMP_FRAMES) + 1
      levels = [x for _, x in sent]
      steps = [c - a for a, c in zip(levels, levels[1:])]
      self.assertLessEqual(max(steps), bound, (pers, levels))
      self.assertGreaterEqual(min(steps), 0, (pers, levels))
      self.assertEqual(cap, levels[-1])

  def test_timeout_counts_continuous_time_below_3kph(self):
    """Creeping back above 3 km/h (inside the 3-5 km/h exit hysteresis) restarts the 1.5 s count."""
    d = _Drive(AGG).run(200, -0.73, 6.0)
    d.run(120, -0.73, 2.0).run(20, -0.73, 3.5).run(120, -0.73, 2.0)   # 1.2 + 1.2 s below, split by 0.2 s at 3.5
    self.assertLess(max(d.levels(200)), 30)
    d.run(60, -0.73, 2.0)                                            # now 1.8 s continuous -> ramping
    self.assertGreater(d.levels()[-1], fl.FCA11_CRAWL_MIN_LSB)

  def test_creep_floor_applies_in_the_band(self):
    """The 3 km/h band still applies the creep floor while rolling: a soft ask whose moving level is 4-6 LSB commands
    at least FCA11_CRAWL_MIN_LSB (the clutch-open level) inside the band, and the plain moving level just above it."""
    for ask in (-0.5, -0.6, -0.73):
      above = _Drive(AGG).run(400, ask, 3.5)                       # 3.5 km/h: not in the band (entry is <= 3)
      lvl_above = above.levels()[-1]
      self.assertLess(lvl_above, fl.FCA11_CRAWL_MIN_LSB, ask)
      inb = _Drive(AGG).run(140, ask, 2.5)
      self.assertEqual(fl.FCA11_CRAWL_MIN_LSB, inb.levels()[-1], ask)
      self.assertEqual(fl.FCA11_CRAWL_MIN_LSB, max(inb.levels()), ask)

  def test_hard_ask_in_the_band_still_gets_the_cap(self):
    d = _Drive(AGG).run(200, -0.73, 6.0).run(20, -0.73, 2.0)
    t0 = d.f
    d.run(30, -2.5, 2.0)
    self.assertEqual(30, d.levels(t0)[-1])

  def test_band_exit_clears_the_latch(self):
    d = _Drive(AGG).run(200, -0.73, 6.0).run(40, -0.73, 0.0, standstill=True)
    self.assertEqual(30, d.levels()[-1])
    d.run(20, -0.73, 6.0)                                            # >= 5 km/h exit
    t0 = d.f
    d.run(20, -0.73, 2.0)                                            # re-entry while rolling: creep floor again
    self.assertLess(max(d.levels(t0)), 30)


if __name__ == "__main__":
  unittest.main()
