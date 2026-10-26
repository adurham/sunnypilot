"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

0051: FCA11 feel FIX round on the 0049 feel layer (drive-feedback 15b-15f). Pins:
  * (a) wobble damping: a gentle ask with planner ripple after an easing no longer dips-and-recovers; a genuine
    re-deepening (gentle by >= RERISE_LSB, a harder ask, or a fast-building one) still climbs; onsets are untouched;
  * (b) gentle-ask gain: pure map bounds (in (0, 1], 1.0 for a HARD ask, monotone), the steady level for gentle asks,
    and a HARD ask byte-identical to the deployed law;
  * (b2) low-speed restore: the gain fades back to 1.0 ahead of the stop/hold band (which runs the DEPLOYED law), so a
    gentle ask arrives at the deployed level and band entry cannot step it up (independent review M1);
  * (c) lead-loss hold: a lead drop-out mid-brake does not let the level fall to the floor; the hold is bounded by the
    level on the wire AND the PRE-0049 deployed request, never arms without an open episode, tapers (no step), and never
    delays an exit or a veto;
  * the lead-loss hold bounds/timings and the deadband/gain boundaries the review found unpinned (its M2b);
  * the kill switch (FCA11_FEEL_FIX_0051 False == the 0049 feel layer) and the scope (feel path only).
"""
import random
import unittest

from opendbc.car import structs
from opendbc.sunnypilot.car.hyundai import fca11_long as fl
from opendbc.sunnypilot.car.hyundai.tests.test_fca11_long import _CS, _cam
from opendbc.sunnypilot.car.hyundai.tests.test_fca11_feel import _FeelSwitch, _cc, _cp_sp

STEP = fl.HYUNDAI_FCA11_LONG_RATE_STEP


class _FixSwitch:
  def __init__(self, on: bool):
    self.on = on

  def __enter__(self):
    self.prev = fl.FCA11_FEEL_FIX_0051
    fl.FCA11_FEEL_FIX_0051 = self.on
    return self

  def __exit__(self, *a):
    fl.FCA11_FEEL_FIX_0051 = self.prev


def _brake(feel=True, fix=True, affine=False):
  with _FeelSwitch(feel), _FixSwitch(fix):
    b = fl.Fca11LongBrake(structs.CarParams(), _cp_sp(affine))
  b.log.enabled = False
  b.cal_seq = None
  return b


class _Lead:
  status = True
  dRel = 30.0
  vRel = -1.0


def _cs(lead=True, v=20.0, **kw):
  cs = _CS(v_ego=v, cam=_cam(), **kw)
  cs.fca11_lead_one = _Lead() if lead else None
  return cs


def _run(b, asks, leads=None, v=20.0, personality=fl.PERSONALITY_STANDARD, start=0, cs_fn=None):
  """Step frames; return [(frame, dec)] of every 0x38D sent. leads[i] False = no lead on that frame (default: lead)."""
  out = []
  for i, a in enumerate(asks):
    f = start + i
    cs = cs_fn(f) if cs_fn else _cs(lead=(True if leads is None else leads[i]), v=v)
    for s in b.update(_cc(a), cs, f, personality):
      out.append((f, fl._get_bits(s.dat, 8, 8)))
  return out


def _dips(levels):
  """The stops-analysis wire counter: fall >= 2 below the running max, then rise >= 2 above the trough."""
  cyc = 0
  rmax = -1
  tr = 99
  dip = False
  for x in levels:
    rmax = max(rmax, x)
    if not dip and x <= rmax - 2:
      dip = True
      tr = x
    if dip:
      tr = min(tr, x)
      if x >= tr + 2:
        cyc += 1
        dip = False
        rmax = x
  return cyc


def _ripple_trace(seed, n=600):
  """Build an episode at -1.0, ease to a gentle -0.62 plateau carrying planner-like ripple (+-0.06, ~0.1-0.3 s)."""
  rng = random.Random(seed)
  asks = [-1.0] * 150 + [-1.0 + 0.38 * i / 50 for i in range(50)]
  a = -0.62
  d = 0.0
  for _ in range(n):
    d = 0.8 * d + rng.gauss(0, 0.012)
    asks.append(max(-0.74, min(-0.50, a + d + 0.03 * rng.choice((-1, 1)) * (rng.random() < 0.1))))
  return asks


class TestGentleGainPure(unittest.TestCase):

  def test_gain_bounds_and_shape(self):
    self.assertEqual(1.0, fl.feel_gentle_gain(fl.FCA11_ONSET_BYPASS_ACCEL))
    self.assertEqual(1.0, fl.feel_gentle_gain(-3.0))
    self.assertEqual(fl.FCA11_FEEL_GENTLE_GAIN, fl.feel_gentle_gain(-0.5))
    self.assertEqual(fl.FCA11_FEEL_GENTLE_GAIN, fl.feel_gentle_gain(fl.FCA11_FEEL_GAIN_FADE_ACCEL))
    prev = None
    a = 0.0
    while a >= -3.0:
      g = fl.feel_gentle_gain(a)
      self.assertGreater(g, 0.0)
      self.assertLessEqual(g, 1.0)
      if prev is not None:
        self.assertGreaterEqual(g, prev - 1e-12, f"gain never decreases as the ask hardens ({a})")
      prev = g
      a -= 0.01

  def test_gentle_target_never_above_0049_target_and_monotone(self):
    for p in (fl.PERSONALITY_RELAXED, fl.PERSONALITY_STANDARD, fl.PERSONALITY_AGGRESSIVE):
      prev = None
      a = 0.0
      while a >= -3.5:
        g = fl.feel_gentle_target_lsb(a, p)
        self.assertLessEqual(g, fl.feel_target_lsb(a, p), (a, p))
        self.assertGreaterEqual(g, fl.FCA11_FEEL_FLOOR_LSB)
        if a <= fl.FCA11_ONSET_BYPASS_ACCEL:
          self.assertEqual(fl.feel_target_lsb(a, p), g, f"HARD ask keeps the deployed level ({a})")
        if prev is not None:
          self.assertGreaterEqual(g, prev, f"monotone ({a}, {p})")
        prev = g
        a -= 0.01

  def test_documented_partial_correction(self):
    """The gain is the documented partial correction: sqrt(1/2.03) of the (-1.0,-0.7] band's measured 2.03x (rounded)."""
    self.assertAlmostEqual((1 / 2.03) ** 0.5, fl.FCA11_FEEL_GENTLE_GAIN, delta=0.005)
    self.assertEqual(8, fl.feel_target_lsb(-0.5))
    self.assertEqual(5, fl.feel_gentle_target_lsb(-0.5))
    self.assertEqual(12, fl.feel_target_lsb(-0.8))
    self.assertEqual(9, fl.feel_gentle_target_lsb(-0.8))


class TestGentleGainController(unittest.TestCase):

  def test_steady_gentle_levels(self):
    for ask in (-0.62, -0.7, -0.8, -0.95):
      got = _run(_brake(), [ask] * 300)
      self.assertEqual(fl.feel_gentle_target_lsb(ask), got[-1][1], ask)

  def test_hard_ask_from_idle_byte_identical_to_deployed(self):
    for p in (fl.PERSONALITY_RELAXED, fl.PERSONALITY_STANDARD, fl.PERSONALITY_AGGRESSIVE):
      for ask in (-1.2, -1.6, -2.0, -3.5):
        new = _run(_brake(True, True), [ask] * 120, personality=p)
        old = _run(_brake(False), [ask] * 120, personality=p)
        self.assertEqual(old, new, (p, ask))


class TestWobbleDamping(unittest.TestCase):

  def test_ripple_after_easing_no_longer_dips(self):
    """15c seg 15 class: an eased gentle ask with planner ripple. 0049 (fix off) dips-and-recovers; 0051 does not."""
    on = off = 0
    for seed in range(12):
      asks = _ripple_trace(seed)
      on += _dips([d for f, d in _run(_brake(True, True), asks) if f >= 200 and d > 0])
      off += _dips([d for f, d in _run(_brake(True, False), asks) if f >= 200 and d > 0])
    self.assertGreater(off, 0, "the trace reproduces the 0049 dither (else this test proves nothing)")
    self.assertLessEqual(on, off // 4, f"0051 dips {on} vs 0049 {off}")

  def test_eased_plateau_with_ripple_holds_still(self):
    """Anticipation reads the SLOW slope EMA: on the eased, rippling plateau the level does not step down at all (the
    0049 fast EMA steps it down-and-up by ripple alone)."""
    downs = 0
    for seed in range(12):
      lv = [d for f, d in _run(_brake(), _ripple_trace(seed)) if f >= 260 and d > 0]
      downs += sum(1 for x, y in zip(lv, lv[1:], strict=False) if y < x)
    self.assertLessEqual(downs, 2, f"level steps down on ripple: {downs}")

  def test_fast_building_ask_bypasses_the_deadband_by_one_lsb(self):
    """White-box: below the peak, a gentle ask whose target is only +1 LSB is HELD by the deadband - unless the ask is
    building faster than FCA11_FEEL_DEEPEN_SLOPE (then it climbs on the same sent frame)."""
    for fast in (False, True):
      b = _brake()
      _run(b, [-1.0] * 150 + [-0.55] * 100)
      a = -0.7
      L0 = fl.feel_gentle_target_lsb(a) - 1           # place the episode exactly 1 LSB below the gentle target for `a`
      b._feel_acc = float(L0)
      b._dec_last_sent = L0
      b._feel_slope_ant = 0.0
      self.assertLess(L0, b._feel_peak - 0.5)
      if fast:
        b._feel_slope = -3.0
      got = _run(b, [a] * 2, start=250)
      self.assertEqual(L0 + 1 if fast else L0, got[0][1], f"fast={fast}")

  def test_genuine_gentle_redeepening_still_climbs(self):
    """After an easing, a gentle ask that calls for >= RERISE_LSB more climbs to its target (the deadband only HOLDS)."""
    b = _brake()
    _run(b, [-1.0] * 150 + [-0.6] * 100)
    low = b._dec_last_sent
    got = _run(b, [-0.95] * 150, start=250)
    self.assertEqual(fl.feel_gentle_target_lsb(-0.95), got[-1][1])
    self.assertGreaterEqual(got[-1][1], low + 2)

  def test_harder_or_fast_building_ask_bypasses_the_deadband(self):
    b = _brake()
    _run(b, [-1.0] * 150 + [-0.6] * 100)
    got = _run(b, [-1.4] * 60, start=250)
    self.assertEqual(fl.feel_target_lsb(-1.4), got[-1][1], "a harder ask reaches the deployed level")
    b2 = _brake()
    _run(b2, [-1.0] * 150 + [-0.6] * 100)
    asks = [-0.6 - 0.04 * i for i in range(10)] + [-0.98] * 40    # -4 m/s^3 inside the soft band
    lv = [d for _, d in _run(b2, asks, start=250)]
    self.assertGreater(max(y - x for x, y in zip(lv, lv[1:], strict=False)), 1, "fast-building demand: not throttled")

  def test_onset_ramp_untouched_by_the_deadband(self):
    """The deadband needs the level to have come DOWN from the episode peak: an onset (level == peak) is the 0049 ramp."""
    for ask in (-0.65, -0.9):
      lv = [d for _, d in _run(_brake(), [ask] * 200)]
      self.assertEqual(STEP, lv[0])
      self.assertEqual(lv, sorted(lv), "onset is monotone")
      self.assertEqual(fl.feel_gentle_target_lsb(ask), lv[-1])

  def test_deepening_bypass_reads_the_unchanged_fast_slope(self):
    self.assertEqual(0.2, fl.FCA11_FEEL_SLOPE_ALPHA, "the 0049 EMA (deepening bypass) is unchanged")
    self.assertLess(fl.FCA11_FEEL_SLOPE_ALPHA_0051, fl.FCA11_FEEL_SLOPE_ALPHA)


class TestLeadLossHold(unittest.TestCase):

  def _scene(self, fix):
    """15c seg 45 shape: braking at -0.95 with a lead; the lead drops for 0.5 s and the ask eases to -0.49 on the same frame,
    stays eased ~1 s after the lead returns, then rebuilds."""
    asks = [-0.95] * 200 + [-0.49] * 50 + [-0.52] * 100 + [-0.52 - 0.6 * i / 150 for i in range(150)]
    leads = [True] * 200 + [False] * 50 + [True] * 250
    return _run(_brake(True, fix), asks, leads=leads)

  def test_lead_flicker_does_not_drop_the_level_to_the_floor(self):
    on = [d for f, d in self._scene(True) if 200 <= f < 350]
    off = [d for f, d in self._scene(False) if 200 <= f < 350]
    self.assertEqual(fl.FCA11_FEEL_FLOOR_LSB, min(off), "0049 falls to the floor (the 15c seg45 scene)")
    req = fl.clamp_dec_cmd(fl.dec_cmd_from_accel(-0.5))      # the PRE-0049 request for an eased (-0.49) ask: 8
    self.assertGreaterEqual(min(on[:50]), min(req, max(off[:1])), "held while the lead is absent")
    self.assertGreater(min(on), min(off) + 1)

  def test_hold_never_above_the_wire_or_the_pre_0049_request(self):
    rng = random.Random(51)
    for trial in range(30):
      asks, leads, a, lead = [], [], -0.9, True
      for _ in range(800):
        a = max(-3.0, min(0.3, a + rng.gauss(0, 0.05)))
        if rng.random() < 0.01:
          a = rng.uniform(-2.5, 0.0)
        if rng.random() < 0.02:
          lead = not lead
        asks.append(a)
        leads.append(lead)
      p = rng.choice([0, 1, 2])
      b = _brake()
      last = 0
      for f, d in _run(b, asks, leads=leads, personality=p):
        if d > 0:
          req = fl.clamp_dec_cmd(fl.dec_cmd_from_accel(min(asks[f], fl.FCA11_BRAKE_ONSET_ACCEL)), p)
          self.assertLessEqual(d, req, f"trial {trial} f={f}: above the PRE-0049 request")
          self.assertLessEqual(d, last + STEP)
        last = d

  def test_hold_tapers_without_a_step_and_ends(self):
    asks = [-0.95] * 200 + [-0.55] * 400
    leads = [True] * 200 + [False] * 30 + [True] * 370
    lv = [d for f, d in _run(_brake(), asks, leads=leads) if f >= 200]
    for x, y in zip(lv, lv[1:], strict=False):
      self.assertGreaterEqual(y, x - 1, f"taper: at most 1 LSB per sent frame down ({lv})")
    self.assertEqual(fl.feel_gentle_target_lsb(-0.55), lv[-1], "the hold has run out: back to the feel target")

  def test_lead_lost_before_the_episode_never_arms(self):
    asks = [0.0] * 50 + [-0.9] * 200
    leads = [True] * 20 + [False] * 230
    b = _brake()
    _run(b, asks, leads=leads)
    self.assertIsNone(b._feel_ll_frame)

  def test_exit_and_vetoes_are_not_delayed(self):
    """The episode exit rule (-0.30 + min-on) and every `ok` veto end braking on the SAME frame with or without a hold."""
    for fix in (True, False):
      b = _brake(True, fix)
      _run(b, [-0.95] * 200)
      got = _run(b, [-0.1] * 6, leads=[False] * 6, start=200)
      self.assertFalse(b.braking, fix)
      self.assertEqual(0, got[0][1], f"release: passive on the first send slot (fix={fix})")
    b = _brake()
    _run(b, [-0.95] * 200)
    _run(b, [-0.95] * 4, leads=[False] * 4, start=200)
    self.assertIsNotNone(b._feel_ll_frame)
    got = _run(b, [-0.95] * 6, start=204, cs_fn=lambda f: _cs(lead=False, brake=True))
    self.assertFalse(b.braking)
    self.assertEqual([0], [d for _, d in got][:1], "driver brake during a hold: immediate passive frame")

  def test_new_episode_clears_the_hold(self):
    b = _brake()
    _run(b, [-0.95] * 200)
    _run(b, [-0.95] * 4, leads=[False] * 4, start=200)
    _run(b, [-0.1] * 80, start=204)
    self.assertFalse(b.braking)
    got = _run(b, [-0.9] * 30, start=284)
    self.assertEqual(STEP, [d for _, d in got if d > 0][0], "a re-open is a fresh soft onset")
    self.assertIsNone(b._feel_ll_frame)


class TestKillSwitchAndScope(unittest.TestCase):

  def test_fix_off_is_the_0049_feel_layer(self):
    b = _brake(True, False)
    self.assertTrue(b.feel)
    self.assertFalse(b.feel_fix)
    for ask in (-0.62, -0.8, -1.0):
      got = _run(_brake(True, False), [ask] * 300)
      self.assertEqual(fl.feel_target_lsb(ask), got[-1][1], "fix off: the 0049 steady level")

  def test_fix_off_ignores_lead_and_matches_without_lead(self):
    rng = random.Random(5)
    asks = [max(-3.0, min(0.2, -0.8 + rng.gauss(0, 0.4))) for _ in range(1500)]
    leads = [rng.random() < 0.7 for _ in range(1500)]
    a = _run(_brake(True, False), asks, leads=leads)
    b = _run(_brake(True, False), asks, leads=[True] * 1500)
    self.assertEqual(a, b)

  def test_feel_off_or_affine_never_runs_the_fix(self):
    self.assertFalse(_brake(False, True).feel_fix)
    self.assertFalse(_brake(True, True, affine=True).feel_fix)


class TestLowSpeedRestore(unittest.TestCase):
  """0051 (b2), the review's M1: the stop/hold band (<= 3 km/h, standstill) runs the DEPLOYED law, NOT the feel law. With
  the gentle gain in force the level sat 2-3 LSB below the band's request and band entry re-stepped it (a +4 LSB step in
  the last ~0.6 s before standstill on 18 of the 25 scored stops; 0/25 on 0049). The gain is restored to 1.0 as the car
  slows, so the level ARRIVES at the deployed request and the band cannot step it."""

  def test_curve_thresholds_are_documented_and_the_range_is_1_to_0(self):
    """Two thresholds in the stop band's own idiom (FULL 5.0 at the band's EXIT 5.0, FADE 8.0 above its ENTER 3.0), a
    continuous linear fade between them. Literal pins: a changed threshold fails here."""
    self.assertEqual(8.0, fl.FCA11_FEEL_GAIN_FADE_KPH)
    self.assertEqual(5.0, fl.FCA11_FEEL_GAIN_FULL_KPH)
    self.assertGreater(fl.FCA11_FEEL_GAIN_FADE_KPH, fl.FCA11_FEEL_GAIN_FULL_KPH)
    self.assertLessEqual(fl.FCA11_FEEL_GAIN_FULL_KPH, fl.FCA11_FEEL_HOLD_EXIT_KPH)
    self.assertEqual(1.0, fl.feel_gentle_gain_speed_frac(fl.FCA11_FEEL_GAIN_FADE_KPH))
    self.assertEqual(1.0, fl.feel_gentle_gain_speed_frac(40.0))
    self.assertEqual(0.0, fl.feel_gentle_gain_speed_frac(fl.FCA11_FEEL_GAIN_FULL_KPH))
    self.assertEqual(0.0, fl.feel_gentle_gain_speed_frac(0.0))
    self.assertAlmostEqual(0.5, fl.feel_gentle_gain_speed_frac(6.5), places=6)
    v = 0.0
    while v <= 40.0:      # monotone in v, always in [0, 1]
      g = fl.feel_gentle_gain_speed_frac(v)
      self.assertGreaterEqual(g, 0.0)
      self.assertLessEqual(g, 1.0)
      self.assertLessEqual(fl.feel_gentle_gain_speed_frac(v - 0.5), g + 1e-12, v)
      v += 0.5

  def test_gain_speed_zero_is_the_deployed_0049_level(self):
    """The restore endpoint: gain_speed 0 == feel_target_lsb (the deployed request); 1 == the (b) gentle target."""
    for p in (fl.PERSONALITY_RELAXED, fl.PERSONALITY_STANDARD, fl.PERSONALITY_AGGRESSIVE):
      for ask in (-0.4, -0.55, -0.62, -0.8, -1.0, -1.1):
        self.assertEqual(fl.feel_target_lsb(ask, p), fl.feel_gentle_target_lsb(ask, p, gain_speed=0.0), (ask, p))
        self.assertEqual(fl.feel_gentle_target_lsb(ask, p), fl.feel_gentle_target_lsb(ask, p, gain_speed=1.0), (ask, p))
        self.assertLessEqual(fl.feel_gentle_target_lsb(ask, p, gain_speed=0.5), fl.feel_target_lsb(ask, p), (ask, p))

  def test_stop_band_entry_does_not_step_the_level_up(self):
    """Drive a stop (20 m/s -> 0.2 m/s at 1 m/s^2) on a constant gentle ask. The level must be at the DEPLOYED request
    before the band (v <= 3 km/h) and band entry must not step it up (the review measured a +4 step at band entry)."""
    ask = -0.6
    v_of = lambda f: max(0.0, 20.0 - 0.01 * f)
    self.assertLess(fl.feel_gentle_target_lsb(ask), fl.feel_target_lsb(ask), "the ask must be in the gain's range")
    b = _brake()
    got = [(f, d) for f, d, in _run(b, [ask] * 1990, cs_fn=lambda f: _cs(v=v_of(f)))]
    lv = [(f, d, v_of(f) * 3.6) for f, d in got if d > 0]
    above = [x for x in lv if x[2] > fl.FCA11_FEEL_HOLD_BAND_KPH]
    below = [x for x in lv if x[2] <= fl.FCA11_FEEL_HOLD_BAND_KPH]
    self.assertTrue(above and below, "the scene must reach the stop band")
    pre, first = above[-1][1], below[0][1]
    self.assertEqual(fl.feel_target_lsb(ask), pre, "the restore has returned the deployed level before the band")
    self.assertLessEqual(first - pre, 1, f"no step up at band entry ({pre} -> {first})")
    self.assertLessEqual(max(d for _, d in got), fl.feel_target_lsb(ask), "and never above the deployed request")

  def test_restore_is_inert_at_speed(self):
    """Above FADE_KPH nothing changes: a steady gentle ask at 20 m/s is the (b) gentle level, byte-identical to the
    same scene with the restore disabled by hand (gain_speed 1.0 == the shipped curve above the fade threshold)."""
    for v_kph in (15.0, 20.0, 60.0, 100.0):
      self.assertEqual(1.0, fl.feel_gentle_gain_speed_frac(v_kph))
      got = _run(_brake(), [-0.62] * 300, v=v_kph / 3.6)
      self.assertEqual(fl.feel_gentle_target_lsb(-0.62), got[-1][1], v_kph)

  def test_kill_switch_off_ignores_the_restore(self):
    """Off the fix the target is the 0049 deployed level at every speed: the restore only ever moves the level back
    toward 0049, so switching it off cannot leave the level lower."""
    for v_kph in (2.0, 6.0, 20.0):
      got = _run(_brake(True, False), [-0.62] * 300, v=v_kph / 3.6)
      self.assertEqual(fl.feel_target_lsb(-0.62), got[-1][1], v_kph)


class TestReviewPins(unittest.TestCase):
  """The review's M2b: bounds and constants that no test pinned (mutants holding the lead-loss floor at 30 LSB, or the
  hold/taper/absent times at 3/5/10 s, survived the whole suite). Pinned by LITERAL value where the number IS the safety
  bound, and by behaviour otherwise."""

  def test_gain_is_chosen_by_the_raw_ask_not_the_anticipated_one(self):
    """(b) + (c): the gain is the RAW ask's; the anticipated ask only moves the target, never the gain."""
    a_raw, a_ant = -1.05, -0.75
    self.assertNotEqual(fl.feel_gentle_gain(a_raw), fl.feel_gentle_gain(a_ant))
    b = _brake()
    self.assertEqual(fl.feel_gentle_target_lsb(a_ant, gain_accel=a_raw), b._feel_target(a_ant, a_raw, None))
    self.assertNotEqual(fl.feel_gentle_target_lsb(a_ant, gain_accel=a_ant), b._feel_target(a_ant, a_raw, None))

  def test_gain_value_in_the_soft_band_is_pinned(self):
    """The partial correction is a documented design number (GENTLE_GAIN at/above -0.8, linear to 1.0 at the HARD
    bypass): pinned as literals so a moved fade anchor fails."""
    self.assertEqual(0.70, fl.FCA11_FEEL_GENTLE_GAIN)
    self.assertEqual(-0.8, fl.FCA11_FEEL_GAIN_FADE_ACCEL)
    self.assertAlmostEqual(0.775, fl.feel_gentle_gain(-0.9), places=9)
    self.assertAlmostEqual(0.925, fl.feel_gentle_gain(-1.1), places=9)

  def test_mid_ask_is_not_held_by_the_gentle_deadband(self):
    """The (a) deadband covers GENTLE asks only (above FCA11_FEEL_RATE_SOFT_ACCEL): an ask in (-1.2, -1.0] climbs.
    The ask is held constant across the step so the fast slope stays mild and the deadband is the only thing that could
    hold it (a fast-DEEPENING ask bypasses the deadband by design, so a step from an eased ask would prove nothing)."""
    a = -1.05
    self.assertLess(a, fl.FCA11_FEEL_RATE_SOFT_ACCEL)
    b = _brake()
    _run(b, [-1.3] * 150 + [a] * 100)      # the episode peak is high; the level has followed down to a's target
    L0 = fl.feel_gentle_target_lsb(a) - 1
    b._feel_acc = float(L0)
    b._dec_last_sent = L0
    b._feel_slope = -0.5
    b._feel_slope_ant = 0.0
    self.assertLess(L0, b._feel_peak - 0.5, "the level must be below the episode peak for the deadband to matter")
    got = _run(b, [a] * 2, start=250)
    self.assertEqual(L0 + 1, got[0][1], "a mid ask (in (-1.2, -1.0]) is not held by the gentle deadband")

  def test_anticipation_is_suspended_while_a_hold_is_active(self):
    """(c): an ask easing because the track vanished is not a trend to run ahead of. White-box: with the hold active the
    level climbs to the RAW ask's target, not to the anticipated one (they differ by construction here)."""
    a = -0.62
    ant = fl.feel_anticipated_accel(a, 2.0)
    self.assertNotEqual(ant, a)
    self.assertNotEqual(fl.feel_gentle_target_lsb(ant), fl.feel_gentle_target_lsb(a))
    b = _brake()
    _run(b, [-0.62] * 50)
    b._feel_ll_frame = b._feel_ll_hold_from = 50    # a hold armed at the last SENT level
    b._feel_ll_lsb = 5                              # low enough that the floor cannot mask the target
    b._feel_slope_ant = 2.0                          # an easing slope: the anticipation would run 0.25 m/s^2 ahead
    b._feel_acc = 5.0
    b._dec_last_sent = 5
    b._feel_peak = 5.0                               # the episode peak IS the level, so the (a) deadband is inert here
    got = _run(b, [a] * 40, start=50)
    self.assertEqual(fl.feel_gentle_target_lsb(a), _lvl_at(got, 50 + 30), "the hold target is the RAW ask's")

  def test_hold_never_goes_above_the_level_on_the_wire(self):
    """The (c) bound: the floor IS the last SENT level - a lead drop-out holds that level, it never commands more (the
    floor is not a free number). A mutant freezing e.g. 30 LSB climbs the level here."""
    asks = [-0.95] * 600
    leads = [True] * 200 + [False] * 400
    b = _brake()
    got = _run(b, asks, leads=leads)
    frozen = b._feel_ll_lsb
    self.assertGreater(frozen, 4, "the hold must arm at a real level")
    lv = [d for f, d in got if f >= 200]
    self.assertGreaterEqual(min(lv[:20]), 4)
    self.assertLessEqual(max(lv), frozen, f"the hold climbs above the level on the wire ({frozen} -> {max(lv)})")
    self.assertLessEqual(frozen, fl.feel_target_lsb(-0.95) + 4, "and the frozen level is a level, not a free number")

  def test_lead_loss_timing_constants_are_pinned(self):
    """Safety bounds: the hold follows a drop-out up to MAX_ABSENT, holds HOLD_S, then tapers over TAPER_S. A typo here
    holds braking for seconds after a track loss. Literal pins (NOT read back from the module, so a changed constant
    fails this test)."""
    self.assertEqual(1.0, fl.FCA11_FEEL_LEAD_LOSS_MAX_ABSENT_S)
    self.assertEqual(0.6, fl.FCA11_FEEL_LEAD_LOSS_HOLD_S)
    self.assertEqual(1.0, fl.FCA11_FEEL_LEAD_LOSS_TAPER_S)

  def test_lead_loss_floor_decay_is_hold_then_taper_then_zero(self):
    """Frame-exact shape of the floor: frozen for 0.6 s, linear to 0 over 1.0 s, then cleared (literal times)."""
    b = _brake()
    b._feel_ll_frame = b._feel_ll_hold_from = 0
    b._feel_ll_lsb = 10
    self.assertEqual(10.0, b._feel_lead_loss_floor(0))
    self.assertEqual(10.0, b._feel_lead_loss_floor(59))
    self.assertAlmostEqual(5.0, b._feel_lead_loss_floor(110), places=6)     # 0.50 s into the 1.0 s taper
    self.assertEqual(0.0, b._feel_lead_loss_floor(160))                     # 0.6 + 1.0 s: over
    self.assertIsNone(b._feel_ll_frame, "the hold clears itself when it has run out")

  def test_hold_follows_a_dropout_up_to_max_absent(self):
    """The HOLD phase is counted from the lead's RETURN (or from MAX_ABSENT), not from the edge: a 0.9 s drop-out
    (< MAX_ABSENT 1.0) still holds the floor 0.9 s after the edge. The pre-edge ask is gentle so the frozen level is low
    enough that a decayed floor (the edge-counted mutant) lands below it."""
    asks = [-0.7] * 200 + [-0.4] * 400
    leads = [True] * 200 + [False] * 45 + [True] * 355
    b = _brake()
    got = _run(b, asks, leads=leads)
    target = fl.feel_gentle_target_lsb(-0.4)
    held = min(b._feel_ll_lsb, fl.clamp_dec_cmd(fl.dec_cmd_from_accel(fl.FCA11_BRAKE_ONSET_ACCEL)))
    self.assertGreater(b._feel_ll_lsb, 4)
    self.assertGreater(held, target, "the scene only proves anything if the hold is above the eased target")
    self.assertEqual(held, _lvl_at(got, 200 + 20), "the eased ask is held, not followed down")
    self.assertEqual(held, _lvl_at(got, 200 + 90), "still held 0.9 s after the edge")

  def test_hold_has_ended_within_max_absent_plus_hold_plus_taper(self):
    """A 3 s drop-out (> MAX_ABSENT): the hold runs 1.0 + 0.6 + 1.0 s from the edge and is over by 3.0 s."""
    asks = [-0.95] * 200 + [-0.45] * 500
    leads = [True] * 200 + [False] * 300 + [True] * 200
    b = _brake()
    got = _run(b, asks, leads=leads)
    target = fl.feel_gentle_target_lsb(-0.45)
    self.assertGreater(b._feel_ll_lsb, 4)
    self.assertLessEqual(_lvl_at(got, 200 + 300), target + 1, "the hold must be over 3.0 s after the edge")


def _lvl_at(got, f):
  """The last sent level at/before frame ``f`` (0 if none)."""
  x = 0
  for ff, d in got:
    if ff <= f:
      x = d
  return x


if __name__ == "__main__":
  unittest.main()
