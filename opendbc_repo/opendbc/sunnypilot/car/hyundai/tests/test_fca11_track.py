"""0035: the SHAPED law keeps TRACKING the need inside an episode (FCA11_TRACK_* block in fca11_long.py).

Owner complaint #1 (2026-11-04): "braking does whatever the first request was and never adjusts". The car runs with
HyundaiFca11AffineGain AND HyundaiFca11DynamicBrakes ON, i.e. the 0043 SHAPED law (feel/dynamic_brakes are False there).
Every test drives Fca11LongBrake.update() with that exact CP_SP. Behaviour tests use LITERAL numbers (not the new
constants) so they fail on the pre-0035 law by behaviour, not by a missing name.
"""
import unittest

from opendbc.car import structs
import opendbc.sunnypilot.car.hyundai.fca11_long as fl
from opendbc.sunnypilot.car.hyundai.tests.test_fca11_long import _CS, _cam

STD = fl.PERSONALITY_STANDARD


def setup_module(module):
  """0037: this file pins the 0035 TRACKING layer, which is now the OFF path of FCA11_CONT_ENABLED (the continuous law
  that ships on the car is conformance-tested in tests/test_fca11_continuous.py). Without the pin these assertions would
  be asserting the pre-0037 law against the post-0037 one - e.g. the tier caps and the light-ask threshold that the
  continuous law deliberately deletes. `_Drive` builds its controller per test, so the pin here covers the file."""
  module._cont_enabled_prev = fl.FCA11_CONT_ENABLED
  fl.FCA11_CONT_ENABLED = False


def teardown_module(module):
  fl.FCA11_CONT_ENABLED = module._cont_enabled_prev


def _cp_sp(aff=True, dyn=True):
  return structs.CarParamsSP(fca11Brake=True, enableGasInterceptor=True, fca11AffineGain=aff, fca11DynamicBrakes=dyn)


def _cc(accel):
  cc = structs.CarControl()
  cc.enabled = True
  cc.longActive = True
  cc.actuators.accel = accel
  return cc


class _Drive:
  """Feeds frames and records every SENT 0x38D level as (frame, dec)."""

  def __init__(self, aff=True, dyn=True, personality=STD, brake=None):
    self.b = brake or fl.Fca11LongBrake(structs.CarParams(), _cp_sp(aff, dyn))
    self.f = 0
    self.p = personality
    self.sent = []

  def run(self, n, accel, v=15.0, a_ego=None, lead=None):
    """lead = (dRel, vRel) or None. a_ego None = the car delivers exactly the last sent level's 'nominal' 1.2 m/s^2."""
    for _ in range(n):
      cs = _CS(v_ego=v, cam=_cam(), a_ego=(-1.2 if a_ego is None else a_ego))
      if lead is not None:
        cs.fca11_lead_drel_m, cs.fca11_lead_vrel_mps = lead
        cs.fca11_lead_one = object()
      else:
        cs.fca11_lead_drel_m = cs.fca11_lead_vrel_mps = cs.fca11_lead_one = None
      for s in self.b.update(_cc(accel), cs, self.f, self.p):
        self.sent.append((self.f, fl._get_bits(s.dat, 8, 8)))
      self.f += 1
    return self

  def levels(self, since=0):
    return [d for f, d in self.sent if f >= since and d > 0]


class TestScope(unittest.TestCase):

  def test_the_cars_config_runs_the_shaped_law_with_tracking(self):
    """The root cause: AffineGain ON switches the feel layer (and with it dynamic_brakes) OFF - the car's config."""
    b = fl.Fca11LongBrake(structs.CarParams(), _cp_sp(True, True))
    self.assertTrue(b.affine_gain)
    self.assertFalse(b.feel)
    self.assertFalse(b.dynamic_brakes)
    self.assertTrue(getattr(b, "shape_track", False))

  def test_scope_off_without_dynamic_brakes_or_affine(self):
    self.assertFalse(fl.Fca11LongBrake(structs.CarParams(), _cp_sp(True, False)).shape_track)
    self.assertFalse(fl.Fca11LongBrake(structs.CarParams(), _cp_sp(False, True)).shape_track)

  def test_dynamic_brakes_off_is_the_shaped_law_byte_for_byte(self):
    """AffineGain ON + DynamicBrakes OFF: identical frames to the kill switch (the pre-0035 shaped law)."""
    def script(d):
      d.run(150, -1.4, 15.0, a_ego=-1.0, lead=(25.0, -6.0)).run(100, -0.7, 15.0).run(80, -2.2, 12.0, a_ego=-1.5)
      d.run(60, -0.2, 12.0).run(100, -0.9, 20.0)
      return d.sent
    off = script(_Drive(True, False))
    prev = fl.FCA11_TRACK_ENABLED
    try:
      fl.FCA11_TRACK_ENABLED = False
      killed = script(_Drive(True, True))
    finally:
      fl.FCA11_TRACK_ENABLED = prev
    self.assertEqual(off, killed)


class TestMeasuredInverse(unittest.TestCase):

  def test_plant_table_pins(self):
    self.assertEqual((4, 6, 8, 13, 20, 30), fl.FCA11_TRACK_PLANT_LSB)
    self.assertEqual((1.00, 1.14, 1.20, 1.49, 1.57, 2.56), fl.FCA11_TRACK_PLANT_MSS2)
    self.assertEqual(4.0, fl.fca11_track_lsb(0.5))
    self.assertEqual(4.0, fl.fca11_track_lsb(1.00))
    self.assertAlmostEqual(8.0, fl.fca11_track_lsb(1.20))
    self.assertAlmostEqual(20.0, fl.fca11_track_lsb(1.57))
    self.assertEqual(30.0, fl.fca11_track_lsb(5.0))

  def test_inverse_is_monotone_and_inverts_the_plant(self):
    last = 0.0
    for k in range(0, 300):
      a = k * 0.01
      lsb = fl.fca11_track_lsb(a)
      self.assertGreaterEqual(lsb, last)
      last = lsb
      if 1.0 <= a <= 2.56:
        self.assertAlmostEqual(a, fl.fca11_track_plant_decel(lsb), places=6)


class TestTracking(unittest.TestCase):
  """Complaint #1: inside ONE episode the level follows the need up and down."""

  def test_level_rises_while_the_car_under_delivers_a_steady_ask(self):
    """Steady ask -1.4, the car delivers only 1.0: the level must keep CLIMBING (the pre-0035 law holds one level)."""
    d = _Drive().run(100, -1.4, 15.0, a_ego=-1.0)
    at1s = d.levels()[-1]
    d.run(200, -1.4, 15.0, a_ego=-1.0)
    at3s = d.levels()[-1]
    self.assertGreaterEqual(at3s, at1s + 4, (at1s, at3s))

  def test_level_falls_while_the_car_over_delivers(self):
    """Steady ask -1.4, the car delivers 2.0: the level must come DOWN within the episode."""
    d = _Drive().run(100, -1.4, 15.0, a_ego=-1.4)
    at1s = d.levels()[-1]
    d.run(200, -1.4, 15.0, a_ego=-2.0)
    at3s = d.levels()[-1]
    self.assertLessEqual(at3s, at1s - 2, (at1s, at3s))
    self.assertGreaterEqual(at3s, 4)

  def test_closing_lead_raises_the_level_above_the_ask(self):
    """Ask -1.0 but a lead 20 m ahead closing at 8 m/s needs 64 / (2 * 16) = 2.0 m/s^2: the level must exceed the tier
    cap's worth of the ask alone (the shaped law never saw the lead)."""
    plain = _Drive().run(150, -1.0, 15.0, a_ego=-1.0).levels()[-1]
    lead = _Drive().run(150, -1.0, 15.0, a_ego=-1.0, lead=(20.0, -8.0)).levels()[-1]
    self.assertGreaterEqual(lead, plain + 8, (plain, lead))

  def test_easing_need_lowers_the_level(self):
    d = _Drive().run(150, -2.0, 15.0, a_ego=-1.5)
    high = d.levels()[-1]
    d.run(150, -1.1, 15.0, a_ego=-1.3)
    low = d.levels()[-1]
    self.assertLess(low, high - 3, (high, low))


class TestFirmAskCeiling(unittest.TestCase):

  def test_firm_ask_beyond_the_tier_cap_reaches_the_law_ceiling_not_the_gate(self):
    """Standard tier (20 LSB ~1.57 measured): a sustained -2.2 ask the car is not meeting climbs past 20 to 30 - and
    never into the 31-46 wire region (the ask is above the -2.4 urgent lane)."""
    lv = _Drive().run(400, -2.2, 15.0, a_ego=-1.6).levels()
    self.assertGreater(max(lv), 20)
    self.assertLessEqual(max(lv), 30)

  def test_normal_ask_the_tier_cap_can_serve_stays_under_it(self):
    lv = _Drive().run(400, -1.4, 15.0, a_ego=-1.4).levels()
    self.assertLessEqual(max(lv), 20)

  def test_panda_contract_rate_and_gate(self):
    d = _Drive().run(100, -0.9, 15.0).run(100, -3.0, 15.0, a_ego=-1.0).run(100, -1.0, 15.0, a_ego=-2.5)
    prev = 0
    for _, x in d.sent:
      self.assertLessEqual(x, prev + fl.HYUNDAI_FCA11_LONG_RATE_STEP)
      self.assertLessEqual(x, fl.HYUNDAI_FCA11_LONG_MAX_DEC)
      prev = x


class TestLightAsks(unittest.TestCase):

  def test_light_ask_at_speed_does_not_open_a_brake_episode(self):
    """193 #5 / 194 #2: ask -0.5..-0.7 at 19 m/s with no closing lead. 4 LSB (the weakest level) delivers ~1.0 m/s^2,
    i.e. 2x the ask; the pedal's coast handles it. Pre-0035 opened at -0.5 (+0.2 s debounce / 1 s dwell)."""
    self.assertEqual([], _Drive().run(300, -0.65, 19.0).levels())

  def test_light_ask_with_a_closing_lead_still_brakes(self):
    """The closing-lead demand counts as the ask: 25 m, closing 7 m/s -> 49 / 42 = 1.17 m/s^2 -> opens."""
    self.assertTrue(_Drive().run(150, -0.6, 19.0, lead=(25.0, -7.0)).levels())

  def test_stop_approach_below_10_mps_keeps_the_deployed_onset(self):
    """Below 10 m/s (every stop) a -0.6 ask still opens: the release-below-the-floor rejection was a low-speed failure."""
    self.assertTrue(_Drive().run(150, -0.6, 8.0).levels())

  def test_open_episode_at_speed_releases_when_the_ask_goes_light(self):
    d = _Drive().run(150, -1.3, 19.0)
    self.assertTrue(d.levels())
    t = d.f
    d.run(100, -0.4, 19.0)
    self.assertEqual([], d.levels(t + 20))


class TestPhantomLeadCannotOpen(unittest.TestCase):
  """Fable F1: the law-internal closing demand may RAISE an open episode, but it may not OPEN one on its own.

  Before: the demand fed `hard = accel <= -1.2`, which bypassed the 0.2 s onset debounce, so one 20 Hz model frame of a
  false vision object (30 m, closing 25 m/s) opened a 12 LSB / 0.58 s episode. The bypass is now keyed on the PLANNER's
  own ask; the demand still counts as the ask, so it opens once it has persisted the same debounce every mild ask serves.
  """

  def _phantom(self, model_frames):
    return _Drive().run(50, -0.1, 25.0, a_ego=0.0) \
                   .run(5 * model_frames, -0.1, 25.0, a_ego=0.0, lead=(30.0, -25.0)) \
                   .run(150, -0.1, 25.0, a_ego=0.0)

  def test_one_two_three_model_frame_phantoms_send_nothing(self):
    for mf in (1, 2, 3):
      self.assertEqual([], self._phantom(mf).levels(), f"{mf} model frame phantom")

  def test_one_controller_frame_phantom_sends_nothing(self):
    d = _Drive().run(50, 0.0, 15.0, a_ego=0.0).run(1, 0.0, 15.0, a_ego=0.0, lead=(20.0, -8.0)).run(100, 0.0, 15.0, a_ego=0.0)
    self.assertEqual([], d.levels())

  def test_a_closing_lead_persisting_past_the_debounce_still_opens(self):
    """0.3 s of a real closing lead - the demand is not silenced, it just has to outlive the onset debounce."""
    d = _Drive().run(50, -0.1, 25.0, a_ego=0.0).run(30, -0.1, 25.0, a_ego=0.0, lead=(30.0, -25.0))
    self.assertTrue(d.levels())

  def test_the_demand_still_raises_the_level_inside_an_open_episode(self):
    """Opened by the PLANNER (-1.3): the same closing lead must still deepen the episode (only the OPEN is gated)."""
    plain = _Drive().run(60, -1.3, 15.0, a_ego=-1.0).levels()[-1]
    lead = _Drive().run(60, -1.3, 15.0, a_ego=-1.0, lead=(20.0, -8.0)).levels()[-1]
    self.assertGreater(lead, plain, (plain, lead))

  def test_a_hard_planner_ask_still_opens_without_the_debounce(self):
    """The bypass is keyed on the planner ask: a genuine -1.3 at speed still opens immediately (not after 0.2 s)."""
    self.assertTrue(_Drive().run(10, -1.3, 15.0, a_ego=-1.0).levels())


class TestCeilingIsATargetClamp(unittest.TestCase):
  """Fable F2: the ceiling bounds the TARGET before the jerk slew, with hysteresis, and rides the down slew on a drop."""

  def test_hover_across_the_ceiling_switch_does_not_sawtooth(self):
    """Ask alternating -1.72 / -1.62 every 1 s at 1.6 delivered: levels 27-28 -> 20 in one frame, once a second."""
    d = _Drive().run(200, -1.9, 15.0, a_ego=-1.6)
    for k in range(6):
      d.run(100, -1.72 if k % 2 == 0 else -1.62, 15.0, a_ego=-1.6)
    seq = [(f, x) for f, x in d.sent if f >= 200]
    self.assertEqual([], [(f, a, b) for (_, a), (f, b) in zip(seq, seq[1:]) if a - b >= 4])
    # the raised ceiling is LATCHED: with the need held above the 1.33 m/s^2 exit the level never returns to the cap
    self.assertGreater(min(x for _, x in seq), 20)

  def test_crossing_the_urgent_gate_does_not_step(self):
    """Ask -2.5 -> -2.3: the level used to go 41 -> 30 in one 20 ms frame (a 1.4 m/s^2 release step)."""
    d = _Drive().run(300, -2.5, 15.0, a_ego=-1.6)
    t = d.f
    d.run(30, -2.3, 15.0, a_ego=-1.6)
    seq = [x for f, x in d.sent if f >= t - 4]
    self.assertLess(max(a - b for a, b in zip(seq, seq[1:])), 4, seq)

  def test_no_single_frame_step_over_the_panda_step_in_a_harness_sweep(self):
    """A sweep of asks x speeds x personalities: no level delta above the panda's own +RATE_STEP anywhere."""
    worst = 0
    for personality in (fl.PERSONALITY_RELAXED, STD, fl.PERSONALITY_AGGRESSIVE):
      for v in (6.0, 15.0, 25.0):
        d = _Drive(personality=personality)
        d.run(60, -1.4, v, a_ego=-1.0).run(80, -2.6, v, a_ego=-1.2).run(80, -1.5, v, a_ego=-2.0)
        d.run(80, -2.9, v, a_ego=-1.1).run(80, -0.9, v, a_ego=-1.4)
        prev = 0
        for _, x in d.sent:
          if prev and x:                      # a release frame (0) is an episode end, not a level step
            worst = max(worst, abs(x - prev))
          prev = x
    self.assertLessEqual(worst, fl.HYUNDAI_FCA11_LONG_RATE_STEP)

  def test_max_level_on_this_path_is_30(self):
    """"For this build cap the law at 30 LSB (0.30 g) on this path" - the 41-46 lane is behind a DEFAULT-OFF constant."""
    for ask in (-2.2, -2.5, -3.0, -3.4):
      lv = _Drive().run(400, ask, 15.0, a_ego=-1.0).levels()
      self.assertLessEqual(max(lv), 30, ask)

  def test_the_41_46_lane_is_off_by_default(self):
    self.assertFalse(fl.FCA11_TRACK_URGENT46_ENABLED)
    self.assertEqual(30, fl.FCA11_TRACK_CEIL_MAX_LSB)


class TestTierCapBound(unittest.TestCase):
  """Fable F2 (the leak): the tier cap bounds the level in EVERY personality - relaxed reached 26 LSB before this."""

  def test_relaxed_tier_cap_is_never_exceeded(self):
    for ask, aego in ((-1.6, -1.3), (-2.2, -1.6), (-3.0, -1.0)):
      lv = _Drive(personality=fl.PERSONALITY_RELAXED).run(400, ask, 15.0, a_ego=aego).levels()
      self.assertLessEqual(max(lv), 12, (ask, aego))

  def test_each_personality_is_bounded_by_its_cap_or_the_documented_30(self):
    """relaxed 12; standard 20 raised to the documented 30 lane; aggressive 30 (its own cap)."""
    for personality, allowed in ((fl.PERSONALITY_RELAXED, 12), (STD, 30), (fl.PERSONALITY_AGGRESSIVE, 30)):
      lv = _Drive(personality=personality).run(400, -2.5, 15.0, a_ego=-1.3).levels()
      self.assertLessEqual(max(lv), allowed, personality)
      self.assertGreaterEqual(max(lv), min(allowed, 12), personality)   # and it does brake, not silently nothing


class TestHoldBandSeam(unittest.TestCase):
  """Fable F4: the band may inherit a tracked level above its own target; the hand-over must not lurch."""

  def test_band_entry_walks_down_instead_of_stepping(self):
    d = _Drive().run(300, -2.0, 6.0, a_ego=-1.5)
    entry = d.levels()[-1]
    self.assertGreaterEqual(entry, 27, entry)          # the tracked moving level really is above the band target
    t = d.f
    d.run(40, -2.0, 0.8, a_ego=-1.5)                  # 0.8 m/s = 2.9 km/h -> into the <= 3 km/h hold band
    seq = [x for f, x in d.sent if f >= t - 4]
    self.assertLess(max(a - b for a, b in zip(seq, seq[1:])), 4, seq)
    self.assertLessEqual(seq[-1], 20, seq)            # and it does reach the band's own target

  def test_a_band_pass_clears_the_latched_ceiling(self):
    """Fable R2: a latched 30 LSB ceiling must not survive the hold band. Latched at 6 m/s (need 2.0), into the band at
    0.8 m/s, then a creep-out at 5.8 km/h with need 1.5 (inside the 1.33..1.67 hysteresis): the episode starts again
    from the tier cap 20, not from the carried-over 30 lane (it climbed 19, 20, 21 ... 24 before)."""
    d = _Drive().run(300, -2.0, 6.0, a_ego=-1.5).run(60, -2.0, 0.8, a_ego=-1.5)
    t = d.f
    d.run(100, -1.5, 1.6, a_ego=-1.0)                 # 1.6 m/s = 5.8 km/h -> out of the band, tracking resumes
    seq = [x for f, x in d.sent if f >= t]
    self.assertTrue(seq and min(seq) > 0, seq)        # still braking (no release) - the episode never closed
    self.assertLessEqual(max(seq), 20, seq)


class TestPhantomInsideAnOpenEpisode(unittest.TestCase):
  """Fable R1: the +RATE_STEP fast ramp belongs to the PLANNER's hard ask. The law's own closing demand may still raise an
  open episode, but at the comfort jerk slew - a one-frame phantom mid-follow ramped 4 -> 30 LSB in ~0.16 s before."""

  def _run(self):
    d = _Drive().run(50, -0.9, 25.0, a_ego=-0.8)       # planner-opened soft episode, past the 0.2 s onset hold
    t = d.f
    d.run(15, -0.9, 25.0, a_ego=-0.8, lead=(30.0, -25.0)).run(60, -0.9, 25.0, a_ego=-0.8)   # 3 model frames phantom
    return d, t

  def test_a_phantom_does_not_fast_ramp_an_open_episode(self):
    d, t = self._run()
    seq = [x for f, x in d.sent if f >= t - 2]
    self.assertTrue(seq and min(seq) > 0, seq)        # the episode stays open
    rises = [b - a for a, b in zip(seq, seq[1:])]
    self.assertLess(max(rises), 4, seq)                # no +RATE_STEP fast-lane frame
    self.assertLess(max(seq), 20, seq)                 # and nowhere near the 30 LSB lane

  def test_the_demand_still_raises_a_soft_open_episode_at_the_slew(self):
    d, t = self._run()
    plain = _Drive().run(125, -0.9, 25.0, a_ego=-0.8)
    lv = [x for f, x in d.sent if t <= f < t + 15]
    pl = [x for f, x in plain.sent if t <= f < t + 15]
    self.assertGreater(max(lv), max(pl), (lv, pl))


if __name__ == "__main__":
  unittest.main()
