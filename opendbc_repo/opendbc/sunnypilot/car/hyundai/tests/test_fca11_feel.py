"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

0049: FCA11 brake FEEL on the DEPLOYED (OLD-LAW) path - coast-first open, lag-aware onset ramp through the intermediate
LSBs, anticipatory release. Pins:
  * the pure maps (target == the deployed request at/below -0.5, never above it, intermediate levels in the band; the ramp
    never exceeds +RATE_STEP and is +RATE_STEP for a HARD ask; anticipation only ever eases);
  * the controller behaviour (coast-first suppression + hysteresis, onset ramp 4 -> 5 -> 6 -> 7 -> 8, follow-down in the
    release band, the hold band and HARD asks byte-identical to the pre-0049 law);
  * the AUTHORITY invariant (sent level <= the pre-0049 law's on every slot, randomized);
  * the untouched panda contract with FEEL on (Warn 3, <= +RATE_STEP over the last SENT frame, one passive frame on every
    exit, no frames while blocked, camera hand-back latch);
  * the scope (OLD-LAW instance only: the shaping/V3 core and cal never run the feel layer).
"""
import random
import unittest

from opendbc.car import structs
from opendbc.sunnypilot.car.hyundai import fca11_long as fl
from opendbc.sunnypilot.car.hyundai.tests.test_fca11_long import _CS, _cam, _real_camera_frame

STEP = fl.HYUNDAI_FCA11_LONG_RATE_STEP


def _cc(accel=0.0, engaged=True, long_active=True):
  cc = structs.CarControl()
  cc.enabled = engaged
  cc.longActive = long_active
  cc.actuators.accel = accel
  return cc


def _cp_sp(affine=False):
  return structs.CarParamsSP(fca11Brake=True, enableGasInterceptor=True, fca11AffineGain=affine)


class _FeelSwitch:
  """Build an instance with FCA11_FEEL_ENABLED forced (read once at construction), restoring the module value."""
  def __init__(self, on: bool):
    self.on = on

  def __enter__(self):
    self.prev = fl.FCA11_FEEL_ENABLED
    fl.FCA11_FEEL_ENABLED = self.on
    return self

  def __exit__(self, *a):
    fl.FCA11_FEEL_ENABLED = self.prev


def _brake(feel=True, affine=False):
  with _FeelSwitch(feel):
    b = fl.Fca11LongBrake(structs.CarParams(), _cp_sp(affine))
  b.log.enabled = False
  b.cal_seq = None
  return b


def _run(b, asks, v=20.0, personality=fl.PERSONALITY_STANDARD, start=0, cs_fn=None):
  """Step frames; return [(frame, dec, warn)] of every 0x38D sent."""
  out = []
  for i, a in enumerate(asks):
    f = start + i
    cs = cs_fn(f) if cs_fn else _CS(v_ego=v, cam=_cam())
    for s in b.update(_cc(a), cs, f, personality):
      out.append((f, fl._get_bits(s.dat, 8, 8), fl._get_bits(s.dat, 3, 2)))
  return out


class TestFeelPure(unittest.TestCase):

  def test_target_equals_deployed_at_and_below_onset(self):
    for p in (fl.PERSONALITY_RELAXED, fl.PERSONALITY_STANDARD, fl.PERSONALITY_AGGRESSIVE):
      a = fl.FCA11_BRAKE_ONSET_ACCEL
      while a >= -3.5:
        self.assertEqual(fl.clamp_dec_cmd(fl.dec_cmd_from_accel(a), p), fl.feel_target_lsb(a, p), (a, p))
        a -= 0.01

  def test_target_band_uses_intermediate_levels_monotone_and_never_above_deployed(self):
    prev = None
    seen = set()
    a = 0.0
    while a >= -3.0:
      t = fl.feel_target_lsb(a)
      deployed = fl.clamp_dec_cmd(fl.dec_cmd_from_accel(min(a, fl.FCA11_BRAKE_ONSET_ACCEL)))
      self.assertLessEqual(t, deployed, a)
      self.assertGreaterEqual(t, fl.FCA11_FEEL_FLOOR_LSB, a)
      if prev is not None:
        self.assertGreaterEqual(t, prev, f"monotone at {a}")
      prev = t
      seen.add(t)
      a -= 0.005
    for lvl in (4, 5, 6, 7):
      self.assertIn(lvl, seen, "the release band must reach the intermediate LSBs the old law never emitted")

  def test_rate_hard_is_panda_step_soft_is_jerk_limited_never_above_step(self):
    self.assertEqual(float(STEP), fl.feel_rate_lsb_per_sent(fl.FCA11_ONSET_BYPASS_ACCEL))
    self.assertEqual(float(STEP), fl.feel_rate_lsb_per_sent(-3.0, 4.0))
    lo = fl.feel_rate_lsb_per_sent(-0.6, 4.0)
    hi = fl.feel_rate_lsb_per_sent(-0.6, 12.0)
    self.assertAlmostEqual(fl.FCA11_FEEL_JERK_TARGET / fl.FCA11_FEEL_SLOPE_LOW * 0.02, lo)
    self.assertAlmostEqual(fl.FCA11_FEEL_JERK_TARGET / fl.FCA11_FEEL_SLOPE_HIGH * 0.02, hi)
    self.assertLess(lo, 1.0)
    self.assertLess(hi, 1.0)
    a = 0.0
    while a >= -3.0:
      for lsb in range(1, 31):
        r = fl.feel_rate_lsb_per_sent(a, float(lsb))
        self.assertGreater(r, 0.0)
        self.assertLessEqual(r, float(STEP))
      a -= 0.05

  def test_anticipation_only_eases_and_never_for_hard(self):
    for a in (-0.4, -0.6, -1.0, -1.19):
      for s in (-5.0, -0.5, 0.0, 0.3, 1.0, 50.0):
        got = fl.feel_anticipated_accel(a, s)
        self.assertGreaterEqual(got, a)
        self.assertLessEqual(got - a, fl.FCA11_FEEL_ANTICIPATE_MAX + 1e-9)
        if s <= 0:
          self.assertEqual(a, got)
    for a in (fl.FCA11_ONSET_BYPASS_ACCEL, -2.0):
      self.assertEqual(a, fl.feel_anticipated_accel(a, 10.0))
    # and it DOES look ahead by the plant lag while easing (capped)
    self.assertAlmostEqual(-0.8 + 0.5 * fl.FCA11_FEEL_ANTICIPATE_S, fl.feel_anticipated_accel(-0.8, 0.5))
    self.assertAlmostEqual(-0.8 + fl.FCA11_FEEL_ANTICIPATE_MAX, fl.feel_anticipated_accel(-0.8, 50.0))

  def test_open_threshold_is_coast_plus_margin(self):
    self.assertAlmostEqual(-0.60, fl.FCA11_FEEL_OPEN_ACCEL)
    self.assertLess(fl.FCA11_FEEL_OPEN_ACCEL, fl.FCA11_BRAKE_ONSET_ACCEL)
    self.assertGreater(fl.FCA11_FEEL_OPEN_ACCEL, fl.FCA11_ONSET_BYPASS_ACCEL)


class TestFeelScope(unittest.TestCase):

  def test_feel_on_old_law_only(self):
    self.assertTrue(_brake(True).feel)
    self.assertFalse(_brake(False).feel)
    self.assertFalse(_brake(True, affine=True).feel, "the shaping/V3 law never runs the feel layer")
    with _FeelSwitch(True):
      off = fl.Fca11LongBrake(structs.CarParams(), structs.CarParamsSP(fca11Brake=False, enableGasInterceptor=True))
    self.assertFalse(off.feel)

  def test_shadow_v3_core_never_runs_feel_and_old_law_does(self):
    import os
    import tempfile
    from unittest import mock
    with tempfile.TemporaryDirectory() as d, mock.patch.dict(os.environ, {"FCA11_LOG_DIR": d}):
      with _FeelSwitch(True):
        cp_sp = structs.CarParamsSP(fca11Brake=True, enableGasInterceptor=True, fca11Shadow=True)
        b = fl.Fca11LongBrake(structs.CarParams(), cp_sp)
      self.assertTrue(b.shadow)
      self.assertTrue(b.feel, "the actuating OLD-LAW instance runs the feel layer")
      core = b._shadow_v3
      assert core is not None
      self.assertFalse(core.feel, "the V3 core is the frozen shaping law")

  def test_feel_disabled_is_byte_identical_to_pre_0049(self):
    """FCA11_FEEL_ENABLED False must reproduce the pre-0049 law - pinned against the deployed sequence shape."""
    b = _brake(False)
    sent = _run(b, [-0.55] * 40 + [-0.35] * 60 + [-0.2] * 10)
    decs = [d for _, d, _ in sent]
    self.assertEqual([4, 8], decs[:2], "pre-0049: 0 -> 4 -> 8")
    self.assertNotIn(5, decs)
    self.assertNotIn(6, decs)
    self.assertNotIn(7, decs)


class TestCoastFirst(unittest.TestCase):

  def test_transient_small_ask_below_coast_plus_margin_never_opens(self):
    b = _brake(True)
    asks = ([-0.55] * 80 + [-0.2] * 20) * 3   # 0.8 s small asks with gaps: transients, never a sustained 1 s
    self.assertEqual([], _run(b, asks))
    self.assertFalse(b.braking)
    old = _brake(False)
    self.assertTrue(any(d > 0 for _, d, _ in _run(old, asks)), "the pre-0049 law opens on -0.55")

  def test_dwell_escape_opens_a_sustained_small_ask(self):
    b = _brake(True)
    for f in range(fl.FCA11_FEEL_DWELL_FRAMES - 1):
      b.update(_cc(-0.55), _CS(cam=_cam()), f)
      self.assertFalse(b.braking, f)
    b.update(_cc(-0.55), _CS(cam=_cam()), fl.FCA11_FEEL_DWELL_FRAMES - 1)
    self.assertTrue(b.braking, "a sustained -0.55 opens after the dwell")
    sent = _run(b, [-0.55] * 100, start=fl.FCA11_FEEL_DWELL_FRAMES)
    for x, y in zip(sent, sent[1:], strict=False):
      self.assertLessEqual(y[1] - x[1], 1, "and ramps softly")

  def test_dwell_counts_only_ok_frames(self):
    b = _brake(True)
    _run(b, [-0.55] * 150, cs_fn=lambda f: _CS(cam=_cam(), gas=True))
    self.assertFalse(b.braking)
    self.assertEqual(0, b._feel_dwell)

  def test_ask_past_threshold_opens_after_the_same_debounce(self):
    b = _brake(True)
    for f in range(fl.FCA11_ONSET_HOLD_FRAMES - 1):
      b.update(_cc(-0.62), _CS(cam=_cam()), f)
      self.assertFalse(b.braking, f)
    b.update(_cc(-0.62), _CS(cam=_cam()), fl.FCA11_ONSET_HOLD_FRAMES - 1)
    self.assertTrue(b.braking)

  def test_hysteresis_release_rule_unchanged(self):
    """Once open, the deployed release rule (-0.30 + min-on) governs: an ask easing to -0.45 STAYS braking."""
    b = _brake(True)
    _run(b, [-0.8] * 30)
    self.assertTrue(b.braking)
    _run(b, [-0.45] * 200, start=30)
    self.assertTrue(b.braking, "inside the hysteresis band the episode stays open")
    _run(b, [-0.2] * 2, start=230)
    self.assertFalse(b.braking)

  def test_hard_ask_opens_on_first_frame(self):
    b = _brake(True)
    sent = _run(b, [-1.5])
    self.assertTrue(b.braking)
    self.assertEqual([(0, STEP, 3)], sent)

  def test_hold_band_runs_the_deployed_law_byte_identical(self):
    # 17c: the feel band IS the deployed band, except the deliberate standstill-hold ceiling (FCA11_STANDSTILL_HOLD_*,
    # pinned in test_fca11_long.py::Test17cStandstillHold). With that switch off the 0049 identity holds at every band
    # speed; with it on the identity still holds wherever the car is NOT at standstill and wherever the ask never asks
    # past the tier cap.
    prev = fl.FCA11_STANDSTILL_HOLD_ENABLED
    for hold_on in (False, True):
      fl.FCA11_STANDSTILL_HOLD_ENABLED = hold_on
      try:
        for v in (0.0, 0.5, 0.8):
          for asks in ([-0.55] * 200, [-0.9] * 50 + [-0.4] * 100 + [-0.2] * 50, [-2.0] * 100):
            if hold_on and v == 0.0 and min(asks) < -1.3:
              continue   # standstill AND an ask past the standard cap: the 17c hold raises it (pinned elsewhere)
            def cs_fn(f, v=v):
              return _CS(v_ego=v, cam=_cam(), standstill=(v == 0.0))
            self.assertEqual(_run(_brake(False), asks, cs_fn=cs_fn), _run(_brake(True), asks, cs_fn=cs_fn), (hold_on, v))
      finally:
        fl.FCA11_STANDSTILL_HOLD_ENABLED = prev

  def test_hold_band_uses_deployed_open(self):
    for v in (0.5, 0.8):   # m/s (<= 3 km/h)
      b = _brake(True)
      self.assertTrue(any(d > 0 for _, d, _ in _run(b, [-0.55] * 60, v=v)), v)


class TestOnsetRamp(unittest.TestCase):

  def test_soft_onset_walks_intermediate_lsbs_with_small_steps(self):
    b = _brake(True)
    sent = [(f, d) for f, d, _ in _run(b, [-0.9] * 200)]
    decs = [d for _, d in sent]
    self.assertEqual(STEP, decs[0], "first frame is the panda onset step")
    for lvl in (5, 6, 7):
      self.assertIn(lvl, decs)
    for x, y in zip(decs, decs[1:], strict=False):
      self.assertLessEqual(y - x, 1, "a soft onset never steps more than 1 LSB per sent frame")
    # 0051 (b): the steady level is the GENTLE-GAIN target (<= the deployed level); fix off -> the deployed level (0049)
    target = fl.feel_gentle_target_lsb(-0.9) if b.feel_fix else fl.feel_target_lsb(-0.9)
    self.assertEqual(target, decs[-1], "and reaches the feel steady level")
    self.assertLessEqual(target, fl.feel_target_lsb(-0.9))
    t8 = next(i for i, d in enumerate(decs) if d >= 8) * 0.02
    self.assertGreaterEqual(t8, 0.25, "4 -> 8 spans at least the plant lag (was 0.02 s)")

  def test_hard_ask_from_idle_is_byte_identical_to_pre_0049(self):
    for p in (fl.PERSONALITY_RELAXED, fl.PERSONALITY_STANDARD, fl.PERSONALITY_AGGRESSIVE):
      for ask in (-1.2, -1.6, -2.0, -3.5):
        new = _run(_brake(True), [ask] * 120, personality=p)
        old = _run(_brake(False), [ask] * 120, personality=p)
        self.assertEqual(old, new, (p, ask))

  def test_steady_level_unchanged_for_onset_asks(self):
    """0049 pinned steady == deployed for every onset ask. 0051 (b) deliberately lowers GENTLE asks (gentle-gain target),
    so: steady == feel_gentle_target_lsb(ask) <= deployed, and == deployed for a HARD ask (<= the bypass). With the 0051
    kill switch OFF (feel_fix False) the steady level is the 0049 deployed level again - the test follows the instance
    (the review's M2a: this test used to fail with FCA11_FEEL_FIX_0051 False)."""
    fix = _brake(True).feel_fix
    for ask in (-0.6, -0.8, -1.0, -1.4):
      new = _run(_brake(True), [ask] * 300)
      old = _run(_brake(False), [ask] * 300)
      want = fl.feel_gentle_target_lsb(ask) if fix else fl.feel_target_lsb(ask)
      self.assertEqual(want, new[-1][1], ask)
      self.assertLessEqual(new[-1][1], old[-1][1], ask)
      if ask <= fl.FCA11_ONSET_BYPASS_ACCEL or not fix:
        self.assertEqual(old[-1][1], new[-1][1], ask)


class TestAnticipatoryRelease(unittest.TestCase):

  def test_band_follows_ask_down_instead_of_clamping_at_8(self):
    b = _brake(True)
    _run(b, [-0.9] * 200)
    sent = _run(b, [-0.36] * 40, start=200)
    self.assertTrue(b.braking)
    self.assertLessEqual(sent[-1][1], 6, "inside the release band the level follows the 0.67 inverse (5-6), not 8")
    old = _brake(False)
    _run(old, [-0.9] * 200)
    self.assertEqual(8, _run(old, [-0.36] * 40, start=200)[-1][1])

  def test_anticipation_lowers_the_level_while_the_ask_eases_below_the_onset(self):
    """Strictly in the ask <= -0.5 region (where the target map == the deployed map), an easing ask must already be
    commanded LOWER than the deployed law - that is the lag look-ahead, not the band follow-down."""
    ease = [-1.6] * 150 + [-1.6 + 1.0 * i / 100 for i in range(100)]    # -1.6 -> -0.6 at +1 m/s^3 (all <= -0.5)
    new = {f: d for f, d, _ in _run(_brake(True), ease)}
    old = {f: d for f, d, _ in _run(_brake(False), ease)}
    lower = [f for f in new if 180 <= f < 250 and new[f] < old[f]]
    self.assertGreater(len(lower), 10)
    self.assertTrue(all(new[f] <= old[f] for f in new))

  def test_easing_ask_drops_level_ahead_of_the_deployed_law_and_exit_timing_is_unchanged(self):
    ease = [-1.0] * 150 + [-1.0 + 0.7 * i / 100 for i in range(100)] + [-0.25] * 10
    new_b, old_b = _brake(True), _brake(False)
    new = _run(new_b, ease)
    old = _run(old_b, ease)
    by_f_new = {f: d for f, d, _ in new}
    by_f_old = {f: d for f, d, _ in old}
    lower = [f for f in by_f_new if 150 <= f < 250 and by_f_new[f] < by_f_old.get(f, 0)]
    self.assertGreater(len(lower), 5, "the anticipated (eased) ask lowers the level during the ease")
    for f in by_f_new:
      self.assertLessEqual(by_f_new[f], by_f_old.get(f, 99), f)
    pass_new = [f for f, d, w in new if d == 0]
    pass_old = [f for f, d, w in old if d == 0]
    self.assertEqual(pass_old, pass_new, "the passive close frame goes out on the same frame")
    self.assertEqual(1, len(pass_new))


class TestAuthorityInvariant(unittest.TestCase):

  def test_randomized_feel_never_exceeds_the_pre_0049_law(self):
    """Two invariants over random ask traces (speeds incl. the hold band, every personality):
      (1) REQUEST domain - on EVERY actuating slot the 0049 level is <= what the deployed law REQUESTS for the same ask
          (clamp_dec_cmd(dec_cmd_from_accel(min(ask, -0.5)))) - feel never asks for more braking than the deployed map;
      (2) TRAJECTORY domain - the 0049 level is <= the deployed law's SENT level on the same slot, except where the two
          episodes are misaligned in time: coast-first opens later, so the UNCHANGED min-on / hysteresis window ends later
          and can overlap the deployed law's next re-open ramp or its gap. Those slots are disclosed and must be rare."""
    rng = random.Random(49)
    misaligned = total = 0
    for trial in range(40):
      asks, a = [], 0.0
      for _ in range(600):
        a += rng.gauss(0, 0.06)
        if rng.random() < 0.01:
          a = rng.uniform(-3.0, 0.3)
        a = max(-3.5, min(0.5, a))
        asks.append(a)
      v = rng.choice([1.0, 8.0, 25.0])
      p = rng.choice([0, 1, 2])
      new = {f: d for f, d, _ in _run(_brake(True), asks, v=v, personality=p)}
      old = {f: d for f, d, _ in _run(_brake(False), asks, v=v, personality=p)}
      for f, d in new.items():
        if d <= 0:
          continue
        total += 1
        deployed_req = fl.clamp_dec_cmd(fl.dec_cmd_from_accel(min(asks[f], fl.FCA11_BRAKE_ONSET_ACCEL)), p)
        self.assertLessEqual(d, deployed_req, f"trial {trial} f={f}: above the deployed request for this ask")
        if d > old.get(f, 0):
          misaligned += 1
    self.assertLess(misaligned, 0.01 * total, f"misaligned slots must be rare: {misaligned}/{total}")


class TestContractWithFeel(unittest.TestCase):

  def test_warn3_and_rate_step_on_every_actuating_frame(self):
    rng = random.Random(7)
    asks = [max(-3.5, min(0.2, -0.9 + rng.gauss(0, 0.6))) for _ in range(2000)]
    b = _brake(True)
    last = 0
    for f, d, w in _run(b, asks, personality=fl.PERSONALITY_AGGRESSIVE):
      if d > 0:
        self.assertEqual(3, w)
        self.assertLessEqual(d, last + STEP, f)
        self.assertLessEqual(d, fl.HYUNDAI_FCA11_LONG_MAX_DEC)
      else:
        self.assertEqual(0, w, "passive frame: Warn 0")
      last = d

  def test_one_passive_frame_on_every_exit_from_a_soft_episode(self):
    # 0053: the camera-BRAKING exit sends nothing while it holds (its owed close frame is pinned in
    # test_camera_handback_releases_and_resumes below), so it is not one of the "one passive frame within 6" exits.
    exits = {
      "brake": lambda f: _CS(cam=_cam(), brake=True),
      "gas": lambda f: _CS(cam=_cam(), gas=True),
      "gear": lambda f: _CS(cam=_cam(), gear=structs.CarState.GearShifter.sport),
      "stale": lambda f: _CS(cam=(0, _real_camera_frame()), now=int(0.2 * 1e9)),
      "release": lambda f: _CS(cam=_cam()),
    }
    for name, cs_fn in exits.items():
      for odd in (0, 1):
        b = _brake(True)
        _run(b, [-0.8] * (100 + odd))
        self.assertTrue(b.braking, name)
        start = 100 + odd
        ask = -0.1 if name == "release" else -0.8
        got = _run(b, [ask] * 6, start=start, cs_fn=cs_fn)
        self.assertFalse(b.braking, name)
        self.assertEqual(1, len([g for g in got if g[1] == 0]), f"{name}@{odd}: {got}")
        self.assertEqual([], [g for g in got if g[1] > 0], f"{name}: no actuation after the exit")
    # longActive-off: owed and delivered on the first frame back, before any actuation
    b = _brake(True)
    _run(b, [-0.8] * 100)
    b.update(_cc(-0.8, long_active=False), _CS(cam=_cam()), 100)
    self.assertTrue(b._release_pending)
    got = _run(b, [-0.8] * 4, start=101)
    self.assertEqual(0, got[0][1])

  def test_camera_handback_releases_and_resumes(self):
    """0053 (was test_camera_handback_latches_and_no_auto_resume): a camera BRAKE hands back (nothing of ours on 0x38D
    while it holds), a camera WARNING does not, and openpilot RESUMES on its own after FCA11_CAM_RESUME_FRAMES quiet
    camera frames - first the owed passive close frame, then a fresh soft onset."""
    b = _brake(True)
    _run(b, [-0.8] * 100)
    self.assertTrue(b.braking)
    self.assertNotEqual([], _run(b, [-0.8] * 10, start=100, cs_fn=lambda f: _CS(cam=(0, _real_camera_frame(warn=2)))),
                        "a warning keeps openpilot braking")
    self.assertTrue(b.braking)
    self.assertEqual([], _run(b, [-0.8] * 4, start=110, cs_fn=lambda f: _CS(cam=(0, _real_camera_frame(dec=7)))))
    self.assertTrue(b.cam_brake)
    self.assertFalse(b.blocked_until_resume)
    quiet = _run(b, [-2.0] * (2 * fl.FCA11_CAM_RESUME_FRAMES - 2), start=114)
    self.assertEqual([], quiet, "handed back until the quiet count")
    after = _run(b, [-2.0] * 20, start=114 + 2 * fl.FCA11_CAM_RESUME_FRAMES - 2)
    self.assertEqual(0, after[0][1], f"the owed close frame first: {after}")
    self.assertTrue([g for g in after if g[1] > 0], "openpilot braking resumes")
    self.assertLessEqual(next(g[1] for g in after if g[1] > 0), STEP, "a fresh onset (panda ref restarted)")

  def test_refused_echo_restarts_the_ramp_within_the_panda_onset_step(self):
    """0035 contract: a REFUSED actuating echo while braking drops our reference to 0 -> the next SENT frame is <= +RATE_STEP
    (the panda's onset rule). The feel accumulator must not run ahead of it."""
    b = _brake(True)
    _run(b, [-1.0] * 300)
    lvl = b._dec_last_sent
    self.assertGreater(lvl, STEP)
    b.observe_echo([(192, bytes([0, lvl, 0, 0, 0, 0, 0, 0]))])
    sent = _run(b, [-1.0] * 6, start=300)
    self.assertLessEqual(sent[0][1], STEP)
    for x, y in zip(sent, sent[1:], strict=False):
      self.assertLessEqual(y[1] - x[1], STEP)
    # and a SOFT re-climb after the reset is a fresh soft onset (the ESC has been back on the camera's idle frame): the
    # accumulator is held to the wire ceiling, so it cannot release banked growth as a +4 jump (4 -> 8).
    b2 = _brake(True)
    _run(b2, [-0.9] * 300)
    lvl2 = b2._dec_last_sent
    self.assertGreaterEqual(lvl2, 8)
    b2.observe_echo([(192, bytes([0, lvl2, 0, 0, 0, 0, 0, 0]))])
    re = [d for _, d, _ in _run(b2, [-0.9] * 40, start=300)]
    self.assertEqual(STEP, re[0])
    for x, y in zip(re, re[1:], strict=False):
      self.assertLessEqual(y - x, 1, f"soft re-climb must stay on the comfort ramp: {re}")

  def test_reopen_restarts_at_the_onset_step(self):
    b = _brake(True)
    _run(b, [-1.0] * 200)
    _run(b, [-0.1] * 70, start=200)
    self.assertFalse(b.braking)
    sent = _run(b, [-1.0] * 30, start=270)
    self.assertEqual(STEP, [d for _, d, _ in sent if d > 0][0])

  def test_fast_deepening_ask_uses_the_full_panda_rate(self):
    """An ask deepening faster than FCA11_FEEL_DEEPEN_SLOPE inside the SOFT band (never reaching -1.0) still grows by more
    than the comfort ramp's <= 1 LSB per sent frame."""
    asks = [-0.62] * 60 + [-0.62 - 0.04 * i for i in range(10)] + [-1.0] * 40     # -4 m/s^3, -0.62 -> -0.98
    sent = [(f, d) for f, d, _ in _run(_brake(True), asks)]
    steps = [y[1] - x[1] for x, y in zip(sent, sent[1:], strict=False) if asks[y[0]] >= -1.0]
    self.assertGreater(max(steps), 1, "a fast-building demand is not throttled by the comfort ramp")
    self.assertLessEqual(max(steps), STEP)

  def test_mid_ask_turning_hard_catches_up_at_the_panda_rate(self):
    """Disclosed trade: a soft ramp that turns HARD starts its +RATE_STEP climb from a lower level, then climbs at the
    panda's full rate - it must reach the deployed level within a bounded number of sent frames."""
    asks = [-0.9] * 30 + [-2.0] * 60
    new = {f: d for f, d, _ in _run(_brake(True), asks, personality=fl.PERSONALITY_AGGRESSIVE)}
    old = {f: d for f, d, _ in _run(_brake(False), asks, personality=fl.PERSONALITY_AGGRESSIVE)}
    cap = fl.PERSONALITY_MAX_DEC[fl.PERSONALITY_AGGRESSIVE]
    t_new = min(f for f, d in new.items() if d >= cap)
    t_old = min(f for f, d in old.items() if d >= cap)
    self.assertLessEqual(t_new - t_old, 6, "at most ~3 sent frames (60 ms) behind")


if __name__ == "__main__":
  unittest.main()
