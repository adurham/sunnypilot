"""0040 (lane L, brake slice 1a): the WIRE quantiser rounds with hysteresis; the aEgo feedback trim STAYS ON.
Owner (2026-11-05), routes 195/196: "Braking is properly linear now. Brake fluttering is bad though ... We
don't just tap the brakes 10 times in a second ... tune it based on my typical braking behavior; smoothness for
acceleration and deceleration."

Lane J shipped the same band WITH the trim removed; Fable's review said split the two, because they carry different
risk. Replayed on the real class over the same 23 windows (lane-K/replay_k_flags.py, 50 Hz sent frames): trim ON + no
band (what ships today) 2.13 reversals/s and 59 isolated zero frames; trim ON + the 1.0 LSB band 0.47/s and 11; trim
OFF only 0.95/s and 6; both 0.27/s and 1. The band is ~78% of the win on its own; the trim removal costs 4.2 m of
minimum gap on a 6% downhill (lane-K/grade_k.py, model-only) and is held for a separate slice. So here the shipped
constant is `FCA11_CONT_TRIM_ENABLED = True` (the 0037 trim, byte-identical to what ships) and the ONLY change to the
law is the quantiser.

Every assertion here is behavioural - none of these tests needs a name that only exists after the change, so the file
runs on the 0038/0039 tree and fails there for the reason the owner reported:

  (a) the trim is LIVE (flag True) and its flag-off branch is still exactly zero - so the split is a real split and
      the trim path was not quietly deleted;
  (b) the wire quantiser: < 0.5 LSB of noise never moves the level, a monotone ramp never reverses, a static target
      still lands on its nearest integer (<= 0.5 LSB), and the band is symmetric;
  (c) the pipeline order as ONE unit: continuous level -> hysteresis round -> down-step clamp -> panda onset gate
      -> sub-bite exclusion (0038, unchanged) -> wire.

Run from the opendbc root:  PYTHONPATH=. python -m pytest -q opendbc/sunnypilot/car/hyundai/tests/test_fca11_brake_commit.py
"""
import contextlib
import math
import random
import unittest

from opendbc.car import structs
import opendbc.sunnypilot.car.hyundai.fca11_long as fl
from opendbc.sunnypilot.car.hyundai.tests.test_fca11_long import _CS, _cam

STD = fl.PERSONALITY_STANDARD


def _trim_live() -> bool:
  """True = the feedback trim contributes. A PRE-0040 tree has no flag and the trim is ALWAYS live, so the tests below
  fail there by behaviour rather than by AttributeError."""
  return bool(getattr(fl, "FCA11_CONT_TRIM_ENABLED", True))


@contextlib.contextmanager
def _trim(flag: bool):
  """Force the trim flag for one test. The quantiser tests use `False` because the trim is the ONE other input to
  `_shape_acc`: a property of the WIRE band must be measured with the demand path frozen, or the trim's own walk
  (which is what the band deliberately does not touch) is read as a quantiser defect."""
  old = _trim_live()
  fl.FCA11_CONT_TRIM_ENABLED = bool(flag)
  try:
    yield
  finally:
    fl.FCA11_CONT_TRIM_ENABLED = old



def _qhyst(target, last):
  """The shipped wire quantiser, or plain rounding on a tree that has none (so (b) fails by behaviour there)."""
  fn = getattr(fl, "fca11_cont_qhyst", None)
  return fn(float(target), int(last)) if fn is not None else int(round(float(target)))


def _band() -> float:
  return float(getattr(fl, "FCA11_CONT_QHYST_LSB", 0.0))


def _cp_sp():
  return structs.CarParamsSP(fca11Brake=True, enableGasInterceptor=True,
                              fca11AffineGain=True, fca11DynamicBrakes=True)


def _cc(accel):
  cc = structs.CarControl()
  cc.enabled = cc.latActive = True
  cc.longActive = True
  cc.actuators.accel = float(accel)
  return cc


class _Drive:
  """Feeds (ask, v, aEgo) frames through the REAL controller and records every SENT 0x38D level."""

  def __init__(self, personality=STD):
    self.b = fl.Fca11LongBrake(structs.CarParams(), _cp_sp())
    self.b._dyn_log = None
    self.b._mark_log = None
    assert self.b.cont_law, "this file tests the 0037 continuous law - the car's own path"
    self.f = 0
    self.p = personality
    self.sent = []      # (frame, level) for every frame the law produced
    self.levels = []

  def run(self, n, ask, v=15.0, a_ego=-1.2, gas=False, brake=False, standstill=False, keep=(), engaged=True):
    for _ in range(n):
      cs = _CS(v_ego=v, cam=_cam(), a_ego=a_ego)
      cs.out.gasPressed = gas
      cs.out.brakePressed = brake
      cs.out.standstill = standstill
      for attr in keep:
        setattr(cs, attr, keep[attr])
      cc = _cc(ask)
      if not engaged:
        cc.enabled = False
      self.last_sends = list(self.b.update(cc, cs, self.f, self.p))
      self.last_frame = self.f
      for s in self.last_sends:
        self.sent.append((self.f, fl._get_bits(s.dat, 8, 8)))
        self.levels.append(fl._get_bits(s.dat, 8, 8))
      self.f += 1
    return self

  def wire(self):
    """The level as the ESC sees it, one sample per 50 Hz send slot."""
    return [lv for f, lv in self.sent if f % fl.FCA11_SEND_EVERY == 0]


def _revs(lv):
  d = [b - a for a, b in zip(lv, lv[1:]) if b != a]
  return sum(1 for a, b in zip(d, d[1:]) if (a > 0) != (b > 0))


# ---------------------------------------------------------------------------------------------------------------------
class Test0040TrimKept(unittest.TestCase):
  """(a) The trim is LIVE in the shipped constant (this slice did NOT remove it), and its flag-off branch - kept for the
  next slice - still contributes exactly zero."""

  # a full command range: coast, light, medium, deep, release, and the stop band - 200 frames each so the 0.5 s
  # feedback delay and the 0.3 s lag line are both long past
  SCRIPT = ((-0.10, 200), (-0.60, 200), (-1.20, 200), (-2.00, 200), (-3.50, 200), (0.00, 200), (-1.50, 300))

  def _seq(self, a_ego):
    d = _Drive()
    for ask, n in self.SCRIPT:
      d.run(n, ask, v=15.0, a_ego=a_ego)
    return d

  def test_the_shipped_constant_has_the_trim_on(self):
    self.assertTrue(_trim_live(), "0040 slice 1a ships with the aEgo feedback trim ON (FCA11_CONT_TRIM_ENABLED = True)")

  def test_the_trim_integrates_and_reaches_the_target(self):
    d = self._seq(a_ego=-1.2)          # a delivered term of 1.2 m/s^2 drives the integrator hard
    self.assertNotEqual(d.b._track_trim, 0.0, "the trim never integrated with the flag ON")
    # ... and it is ADDED to the target: at a fixed ask the level with aEgo -1.2 differs from the aEgo 0 level.
    for ask, n in ((-2.00, 200), (-3.50, 200)):
      d.run(n, ask, v=15.0, a_ego=-1.2)
      self.assertNotEqual(d.b._track_trim, 0.0)

  def test_a_ego_moves_the_level_when_the_trim_is_on(self):
    """THE PROPERTY of slice 1a: the trim is IN the path, so aEgo is an input to the level again (that is the risk
    Fable asked to keep in a separate slice, not the thing this one changes)."""
    base = self._seq(a_ego=0.0).wire()
    self.assertGreater(max(base), 5, "the script must actually brake")
    moved = [a_ego for a_ego in (-1.2, 1.2) if self._seq(a_ego=a_ego).wire() != base]
    self.assertTrue(moved, "aEgo changed nothing - the trim is not in the path with the flag ON")

  def test_with_the_flag_off_the_trim_contributes_exactly_zero(self):
    """The flag-off branch (the OTHER slice's shipped constant) is still there and is still exactly zero: this is the
    proof that slicing the patch did not delete the trim path."""
    with _trim(False):
      d = self._seq(a_ego=-1.2)
      self.assertEqual(d.b._track_trim, 0.0, "the trim integrated with the flag off")
      for ask, n in ((-2.00, 200), (-3.50, 200)):
        d.run(n, ask, v=15.0, a_ego=-1.2)
        self.assertEqual(d.b._track_trim, 0.0)

  def test_with_the_flag_off_the_level_is_identical_across_a_ego(self):
    with _trim(False):
      base = self._seq(a_ego=0.0).wire()
      self.assertGreater(max(base), 5, "the script must actually brake")
      for a_ego in (-1.2, -0.6, 0.6, 1.2):
        self.assertEqual(self._seq(a_ego=a_ego).wire(), base,
                         f"aEgo = {a_ego:+.1f} changed the sent level with the flag off")



# ---------------------------------------------------------------------------------------------------------------------
class Test0040WireQuantiser(unittest.TestCase):
  """(b) The quantiser: one LSB of hysteresis around the level last sent, and nothing else."""

  def test_the_band_is_one_lsb(self):
    self.assertAlmostEqual(_band(), 1.0, places=6)

  def test_noise_below_half_an_lsb_never_moves_the_level(self):
    """A dithered target that stays < 0.5 LSB from an integer must not move it, at any level in the range."""
    for last in (1, 2, 3, 7, 13, 20, 29):
      for k in range(400):
        t = last + 0.49 * math.sin(k * 0.7) + 0.01 * math.sin(k * 3.1)
        self.assertEqual(_qhyst(t, last), last, (last, t))

  def test_noise_above_the_band_does_move_the_level(self):
    """The band must not be a lock: a full LSB past the held level changes it, both ways."""
    self.assertEqual(_qhyst(6.0, 5), 6)
    self.assertEqual(_qhyst(4.0, 5), 4)
    self.assertEqual(_qhyst(6.4, 5), 6)

  def test_a_monotone_ramp_produces_a_monotone_level(self):
    lv = 0
    seq = []
    for i in range(0, 3001):
      lv = _qhyst(i * 0.01, lv)
      seq.append(lv)
    self.assertEqual(seq, sorted(seq), "a monotone ramp reversed the level")
    self.assertEqual(_revs(seq), 0)
    self.assertEqual(seq[-1], 30)

  def test_a_static_target_lands_on_its_nearest_integer(self):
    """Steady state after ONE change: |level - target| <= 0.5 LSB."""
    for target in (3.2, 5.4, 7.5, 12.6, 12.9, 19.7):
      lv = 0
      for _ in range(50):
        lv = _qhyst(target, lv)
      self.assertLessEqual(abs(lv - target), 0.5, (target, lv))

  def test_the_band_is_symmetric(self):
    self.assertEqual(_qhyst(10 + 1.2, 10), 11)
    self.assertEqual(_qhyst(10 - 1.2, 10), 9)
    self.assertEqual(_qhyst(10 + 0.9, 10), 10)
    self.assertEqual(_qhyst(10 - 0.9, 10), 10)

  def test_hysteresis_never_holds_a_level_against_a_release(self):
    """A release to a true zero must not wait for the band: target 0 is a full LSB past anything >= 1."""
    for last in (1, 2, 7, 30):
      self.assertEqual(_qhyst(0.0, last), 0)

  def test_the_law_holds_the_level_through_a_dithering_demand(self):
    """Behavioural, through the controller: the ask is put where the FRACTIONAL target sits on a rounding boundary
    (~20.5 LSB at v = 12 m/s) and then dithered by +-0.2 LSB. Plain rounding re-rounds every 20 ms and flaps 20<->21;
    with the one-LSB band the level does not move at all. The trim is forced OFF: it is the one other input to
    `_shape_acc`, and with a synthetic aEgo it walks on its own, which is exactly the demand-side motion this band
    does not claim to remove (see the module docstring: that is the OTHER slice)."""
    d = _Drive()
    with _trim(False):
      d.run(200, -1.582, v=12.0, a_ego=0.0)                 # settle: the fractional target is ~20.5 LSB
      settled = d.wire()[-1]
      self.assertIn(settled, (20, 21), f"the scenario must sit on the boundary, got {settled}")
      n0 = len(d.wire())
      for k in range(200):                                 # +-0.02 m/s^2 of ask dither = ~0.2 LSB either way
        d.run(1, -1.582 + 0.02 * math.sin(k * 0.9), v=12.0, a_ego=0.0)
      tail = d.wire()[n0:]
    self.assertEqual(len(set(tail)), 1, f"the level flapped on a sub-0.5-LSB dither: {sorted(set(tail))}")


# ---------------------------------------------------------------------------------------------------------------------
class Test0040Pipeline(unittest.TestCase):
  """(c) ONE unit: continuous level -> hysteresis round -> sub-bite exclusion (0038) -> wire."""

  def test_the_wire_never_moves_faster_than_the_law_allows(self):
    """Up <= +4 (the panda's onset gate) over the level the panda LAST ACCEPTED, and down <= 3 LSB per sent frame, under
    a random command stream - measured on the frame that actually reaches the WIRE (through `sub_bite_gate`, which
    replaces a 1-3 LSB frame with the passive 0 when the pedal commands gas) against the panda's own reference
    `_dec_last_sent`, not against our wire deltas (review: the earlier version never called the gate)."""
    rnd = random.Random(195)
    d = _Drive()
    panda_ref = 0
    accepted = muted = 0
    ask = -0.5
    for k in range(1200):
      if k % 50 == 0:
        ask = rnd.uniform(-3.5, 0.4)          # a piecewise-constant ask, so the level has time to build
      d.run(1, ask, v=rnd.uniform(4.0, 22.0), a_ego=rnd.uniform(-1.2, 1.2))
      sends = list(d.last_sends)
      if not sends:
        continue
      wire = d.b.sub_bite_gate(sends, _cs15(), d.last_frame, 0.3 if k % 3 == 0 else 0.0)
      if not wire:
        continue
      lvl = fl._get_bits(wire[-1].dat, 8, 8)
      self.assertLessEqual(lvl, panda_ref + fl.HYUNDAI_FCA11_LONG_RATE_STEP,
                           f"frame {k}: wire {lvl} over the panda's last accepted {panda_ref} + RATE_STEP")
      if lvl == 0 and fl._get_bits(sends[-1].dat, 8, 8) > 0:
        muted += 1
      else:
        accepted += 1
      panda_ref = lvl
    self.assertGreater(accepted, 100, "the scenario never produced an accepted frame, so it proves nothing")
    self.assertGreater(muted, 0, "the scenario never exercised the sub-bite withhold")

  def test_no_sub_bite_level_is_ever_sent_under_throttle(self):
    """0038 unchanged, and DOWNSTREAM of the quantiser: with the pedal commanding gas, a 1-3 LSB frame never reaches
    the wire; with the pedal at true zero the same frame goes out (light braking over coast is untouched)."""
    rnd = random.Random(196)
    d = _Drive()
    muted = passed = 0
    for _ in range(300):
      d.run(3, rnd.uniform(-0.85, -0.55), v=rnd.uniform(12.0, 20.0), a_ego=0.0)
      sends = d.last_sends
      if not sends:
        continue
      lvl = fl._get_bits(sends[-1].dat, 8, 8)
      if not 0 < lvl <= 3:
        continue
      wire_gas = d.b.sub_bite_gate(list(sends), _CS(v_ego=15.0, cam=_cam(), a_ego=0.0), d.last_frame, 0.3)
      wire_zero = d.b.sub_bite_gate(list(sends), _CS(v_ego=15.0, cam=_cam(), a_ego=0.0), d.last_frame, 0.0)
      self.assertFalse(0 < fl._get_bits(wire_gas[-1].dat, 8, 8) <= 3,
                       f"a {lvl} LSB frame rode alongside throttle")
      muted += int(fl._get_bits(wire_gas[-1].dat, 8, 8) == 0)
      passed += int(0 < fl._get_bits(wire_zero[-1].dat, 8, 8) <= 3)
    self.assertGreater(muted, 0, "the scenario never produced a sub-bite level, so it proves nothing")
    self.assertGreater(passed, 0, "with the pedal at zero a sub-bite level must still go out")

  def test_the_law_level_is_unchanged_by_the_sub_bite_gate(self):
    """Order check: the gate changes the WIRE frame only; the law's own level (and so the next frame's hysteresis
    reference and the +RATE_STEP clamp) is a property of the continuous path."""
    d = _Drive().run(120, -0.7, v=15.0, a_ego=0.0)
    level = int(d.b._dec_last_sent)
    self.assertTrue(0 < level <= 3, f"expected a sub-bite episode, got {level}")
    d.run(1, -0.7, v=15.0, a_ego=0.0)
    sends = d.last_sends
    self.assertTrue(sends, "a sub-bite level must produce a frame")
    before = fl._get_bits(sends[-1].dat, 8, 8)
    wire = d.b.sub_bite_gate(list(sends), _CS(v_ego=15.0, cam=_cam(), a_ego=0.0), d.last_frame, 0.4)
    self.assertEqual(fl._get_bits(wire[-1].dat, 8, 8), 0, "the sub-bite frame must not reach the wire under throttle")
    self.assertEqual(before, level, "the gate must be downstream of the level computation")


# ---------------------------------------------------------------------------------------------------------------------
def _cs15(**kw):
  return _CS(v_ego=15.0, cam=_cam(), a_ego=0.0, **kw)


class Test0040Corrections(unittest.TestCase):
  """Review K1-K5 on 0040 v1, each one a property of the real class:

  K1  a NON-SEND frame answers with the level last SENT, not a fresh quantisation of the accumulator;
  K2  the quantiser ZERO-SNAPS below half an LSB, so a 1 LSB application still releases;
  K3  the panda's +RATE_STEP gate is the OUTERMOST bound (nothing can raise the frame above it), and the hysteresis
      reference tracks the BUS;
  K4  an episode that closes with a non-zero wire level leaves no stale reference behind (the re-open is rate-limited
      from the close frame, never from the level the episode died at);
  K5  the two equivalence pins: band 0 == plain rounding, and TRIM_ENABLED + band 0 == the 0037 law's own levels.
  """

  # ---- K1 ------------------------------------------------------------------------------------------------------------
  def _close_then_resume(self, ask=-1.5, v=15.0):
    """Reach a high level, close the episode on a driver-brake veto (ok == False -> the passive close frame) and
    re-arm through a resume edge (the veto latches `blocked_until_resume`, so a plain release cannot re-open)."""
    d = _Drive().run(200, ask, v=v, a_ego=0.0)
    self.assertGreaterEqual(d.wire()[-1], 8, "the script must reach a level the re-open can be clamped from")
    d.run(1, ask, v=v, a_ego=0.0, brake=True)
    self.assertEqual(d.wire()[-1], 0, "the passive close frame must carry level 0")
    self.assertTrue(d.b.blocked_until_resume)
    d.run(2, ask, v=v, a_ego=0.0, engaged=False)
    return d

  def test_a_non_send_frame_answers_with_the_level_last_sent(self):
    """After a rate-limited ONSET (the panda gate binding from the close frame's 0) the accumulator sits ABOVE the
    wire. v1 re-quantised it and answered a level the wire does not carry; that value feeds
    `want_brake = cont_dec > 0` on every frame."""
    d = self._close_then_resume()
    for _ in range(4):                                    # the first clean frame clears the latch, then the re-open
      d.run(1, -1.5, v=15.0, a_ego=0.0)
      if int(d.b._cont_q_last) > 0:
        break
    on_wire = int(d.b._cont_q_last)
    self.assertGreaterEqual(on_wire, 1, "the re-open did not produce a level at all")
    self.assertLessEqual(on_wire, fl.HYUNDAI_FCA11_LONG_RATE_STEP, f"the re-open is not rate-limited: {on_wire}")
    self.assertGreater(d.b._shape_acc, on_wire + 1,
                       "the onset must be clamped below the accumulator, or this proves nothing")
    odd = d.f + 1 if d.f % fl.FCA11_SEND_EVERY == 0 else d.f
    self.assertNotEqual(odd % fl.FCA11_SEND_EVERY, 0)
    self.assertEqual(d.b._cont_level(-1.5, 15.0, _cs15(), odd, STD), on_wire,
                     "a non-send frame answered a level the wire does not carry")

  def test_a_non_send_frame_answers_the_SAME_level_over_a_whole_release(self):
    """The same property on the way down: while the release ramp walks the wire from 13 to 0, every non-send frame
    must answer the level the ramp last put on the wire, not the pre-release one."""
    d = _Drive().run(200, -1.5, v=15.0, a_ego=0.0)
    self.assertGreaterEqual(d.wire()[-1], 8)
    seen = []
    for _ in range(12):
      d.run(1, 0.0, v=15.0, a_ego=0.0)
      if not d.b._actuating:
        break
      ref = int(d.b._cont_q_last)
      self.assertEqual(ref, int(d.b._dec_last_sent),
                       "the law's wire reference and the panda's reference diverged during a release")
      odd = d.f + 1 if d.f % fl.FCA11_SEND_EVERY == 0 else d.f
      self.assertEqual(d.b._cont_level(0.0, 15.0, _cs15(), odd, STD), ref)
      seen.append(ref)
    self.assertGreater(len(seen), 3, "the release ended before it could be sampled")

  # ---- K2 ------------------------------------------------------------------------------------------------------------
  def test_a_target_below_half_an_lsb_releases_a_held_level(self):
    """ZERO-SNAP: (0, 0.5) LSB is below anything the pads can deliver, so the level is 0 whatever was held - the band
    must never hold a 1-3 LSB level against a demand that has already fallen into that window."""
    for last in (1, 2, 3, 8, 13, 30):
      for t in (0.0, 0.1, 0.25, 0.499):
        self.assertEqual(_qhyst(t, last), 0, (t, last))
    self.assertEqual(_qhyst(0.5, 30), 0, "0.5 LSB is still plain rounding's 0")

  def test_a_one_lsb_application_releases_to_zero_and_the_episode_closes(self):
    d = _Drive().run(240, -0.55, v=15.0, a_ego=0.0)      # a sub-bite application (the live trim lifts it to 1-3 LSB)
    self.assertTrue(1 <= d.wire()[-1] <= 3, f"expected a 1-3 LSB application, got {d.wire()[-1]}")
    for _ in range(80):
      d.run(2, 0.0, v=15.0, a_ego=0.0)
      if d.wire()[-1] == 0:
        break
    self.assertEqual(d.wire()[-1], 0, "a 1-3 LSB application never released")
    self.assertFalse(d.b._actuating, "the episode is still open at level 0")

  # ---- K3 / K4 -------------------------------------------------------------------------------------------------------
  def test_a_reopen_after_a_close_never_exceeds_the_panda_gate(self):
    """K3: the panda's reference is `_dec_last_sent` (the last frame it ACCEPTED), so no frame of the re-open may be
    above `reference + RATE_STEP`, and none may fall more than 3 LSB below the previous wire level."""
    d = self._close_then_resume()
    over, step_downs = [], []
    prev = 0
    for _ in range(40):
      ref = int(d.b._dec_last_sent)
      f0 = d.f
      d.run(1, -1.5, v=15.0, a_ego=0.0)
      for f, lv in d.sent:
        if f != f0:
          continue
        if lv > ref + fl.HYUNDAI_FCA11_LONG_RATE_STEP:
          over.append((f, ref, lv))
        if prev > 0 and lv > 0 and prev - lv > int(fl.FCA11_CONT_DOWN_STEP_LSB):
          step_downs.append((f, prev, lv))
        prev = lv
    self.assertEqual(over, [], f"frames above the panda's onset gate (frame, reference, level): {over}")
    self.assertEqual(step_downs, [], f"frames more than 3 LSB below the previous level: {step_downs}")

  def test_the_hysteresis_reference_follows_the_bus_not_the_withheld_level(self):
    """K3, the sub-bite case: a 1-3 LSB frame withheld by sub_bite_gate puts 0 on the wire, so the quantiser must
    compare against 0 next time - while the law's own level (which answers a non-send frame) stays where it is, or the
    episode would close on the very next odd frame."""
    d = _Drive().run(120, -0.70, v=15.0, a_ego=0.0)
    level = int(d.b._dec_last_sent)
    self.assertTrue(0 < level <= 3, f"expected a sub-bite episode, got {level}")
    d.run(1, -0.70, v=15.0, a_ego=0.0)
    wire = d.b.sub_bite_gate(list(d.last_sends), _cs15(), d.last_frame, 0.4)
    self.assertEqual(fl._get_bits(wire[-1].dat, 8, 8), 0, "the sub-bite frame must not reach the wire under throttle")
    self.assertEqual(getattr(d.b, "_cont_q_bus", d.b._cont_q_last), 0,
                     "the hysteresis reference did not follow the bus to 0")
    self.assertEqual(d.b._cont_q_last, level, "the law's own level must survive the withhold")
    self.assertGreater(d.b._cont_level(-0.70, 15.0, _cs15(), d.f + 1, STD), 0,
                       "the next non-send frame closed the episode over a withheld sub-bite frame")

  # ---- K5 ------------------------------------------------------------------------------------------------------------
  def test_a_monotone_ramp_at_any_slope_from_0_1_to_3_lsb_a_frame_never_reverses(self):
    for step in (0.1, 0.25, 0.5, 0.77, 1.0, 1.5, 2.0, 3.0):
      lv, seq = 0, []
      for i in range(int(30.0 / step) + 3):
        lv = _qhyst(min(30.0, i * step), lv)
        seq.append(lv)
      self.assertEqual(_revs(seq), 0, f"a {step} LSB/frame ramp reversed the level: {seq}")
      self.assertEqual(seq, sorted(seq), f"a {step} LSB/frame ramp is not monotone")
      self.assertEqual(seq[-1], 30)

  def test_a_flat_demand_within_plus_minus_0_49_lsb_never_changes_the_level(self):
    for base in (0.0, 1.0, 7.4, 20.5, 29.3):
      lv = 0
      for _ in range(40):                                # settle first (0 -> the nearest integer)
        lv = _qhyst(base, lv)
      seq = [lv]
      for k in range(600):
        seq.append(_qhyst(base + 0.49 * math.sin(k * 0.7) + 0.01, seq[-1]))
      self.assertEqual(len(set(seq)), 1, f"a +-0.49 LSB dither around {base} moved the level: {sorted(set(seq))}")

  def test_a_zero_band_reproduces_plain_rounding(self):
    old = fl.FCA11_CONT_QHYST_LSB
    try:
      fl.FCA11_CONT_QHYST_LSB = 0.0
      for last in (0, 1, 7, 19, 30):
        for k in range(0, 3001):
          t = k * 0.01
          self.assertEqual(fl.fca11_cont_qhyst(t, last), int(round(t)), (t, last))
    finally:
      fl.FCA11_CONT_QHYST_LSB = old

  # the 0038/0039 law's own levels for ONE episode (opendbc patch series without 0040, aEgo -1.2 m/s^2, the trim live),
  # run-length encoded: (level, count). Recorded on the 0038 tree with the same script this test runs.
  GOLDEN_0037 = ((1, 4), (2, 4), (3, 18), (2, 15), (1, 8), (2, 5), (3, 4), (4, 3), (5, 2), (6, 2), (8, 1), (9, 1),
                 (10, 1), (11, 2), (12, 1), (13, 2), (15, 1), (17, 1), (18, 7), (19, 16), (21, 1), (22, 2), (23, 3),
                 (24, 2), (25, 3), (26, 2), (27, 3), (28, 2), (29, 3), (30, 29), (29, 1), (28, 1), (27, 1), (26, 2),
                 (25, 1), (24, 1), (23, 1), (22, 2), (21, 1), (20, 1), (17, 1), (14, 1), (11, 36))

  def test_the_shipped_trim_with_a_zero_band_reproduces_the_0037_levels(self):
    """SLICE 1a's PIN: `FCA11_CONT_TRIM_ENABLED = True` (the shipped constant) + a ZERO band must give the shipped
    law's level sequence EXACTLY on one episode, every SENT frame - the proof that this slice changed the WIRE only and
    that the demand path is bit-for-bit the one that ships. On a tree without the band the same sequence is what the
    law produces natively, so this test passes on the base tree as well BY DESIGN (it is an equivalence pin, not a
    fail-before behavioural test)."""
    self.assertTrue(_trim_live(), "the shipped constant must be the trim-ON branch for this pin to mean anything")
    golden = [lv for lv, n in self.GOLDEN_0037 for _ in range(n)]
    had = hasattr(fl, "FCA11_CONT_QHYST_LSB")
    old = getattr(fl, "FCA11_CONT_QHYST_LSB", None)
    try:
      if had:
        fl.FCA11_CONT_QHYST_LSB = 0.0                 # a pre-0040 tree has no band at all; that IS the zero band
      d = _Drive()
      for ask, n in ((-0.80, 100), (-1.60, 100), (-2.40, 100), (-1.20, 100)):
        d.run(n, ask, v=15.0, a_ego=-1.2)
      self.assertEqual(d.levels, golden, "TRIM_ENABLED=True + band 0 does not reproduce the shipped law")
      self.assertEqual(len(d.levels), sum(n for _, n in self.GOLDEN_0037), "every SENT frame, not a subset")
    finally:
      if had:
        fl.FCA11_CONT_QHYST_LSB = old

  def test_with_the_trim_flag_off_the_wire_is_a_ego_invariant(self):
    """...and the flag-off branch (the next slice's constant) is what makes aEgo leave the level alone."""
    with _trim(False):
      base = _Drive()
      for ask, n in ((-0.80, 100), (-1.60, 100), (-2.40, 100), (-1.20, 100)):
        base.run(n, ask, v=15.0, a_ego=0.0)
      other = _Drive()
      for ask, n in ((-0.80, 100), (-1.60, 100), (-2.40, 100), (-1.20, 100)):
        other.run(n, ask, v=15.0, a_ego=-1.2)
      self.assertEqual(base.levels, other.levels)


# ---------------------------------------------------------------------------------------------------------------------
class Test0040Slice1aOrdering(unittest.TestCase):
  """The three properties lane L's brief names explicitly: the withheld frame leaves NOTHING behind, the wire can never
  be raised by the band against a zero target, and the release latency is a measured number, not the clamp's bound."""

  def test_a_withheld_sub_bite_frame_is_followed_by_nothing_on_the_wire(self):
    """sub_bite_gate withholds a 1-3 LSB frame on the 50 Hz slot; the NEXT frame is a non-send frame and must put
    NOTHING on the wire. Confirmed in the code, not assumed: `update()` appends a brake frame ONLY inside
    `if frame % FCA11_SEND_EVERY == 0:` (fca11_long.py l.4020), which is the sole `build_fca11_frame` call on this path
    (l.4026); on the odd frame `_cont_level` returns the last SENT level (l.2719-2724) and builds nothing. So there is
    no off-slot frame for the gate's zeroed reference to leak into."""
    d = _Drive().run(240, -0.55, v=15.0, a_ego=0.0)
    self.assertTrue(1 <= d.wire()[-1] <= 3, f"expected a sub-bite level, got {d.wire()[-1]}")
    d.run(1, -0.55, v=15.0, a_ego=0.0)                    # a SEND frame carrying a 1-3 LSB level
    self.assertEqual(d.last_frame % fl.FCA11_SEND_EVERY, 0, "this must be a send frame")
    sends = list(d.last_sends)
    self.assertTrue(sends, "the send frame must build a frame for the gate to withhold")
    wire = d.b.sub_bite_gate(sends, _cs15(), d.last_frame, 0.4)
    self.assertEqual(fl._get_bits(wire[-1].dat, 8, 8), 0, "the gate did not withhold the sub-bite frame")
    self.assertEqual(getattr(d.b, "_cont_q_bus", None), 0, "the gate must zero the hysteresis reference it withheld")
    d.run(1, -0.55, v=15.0, a_ego=0.0)                    # the next frame: NOT a 50 Hz slot
    self.assertNotEqual(d.last_frame % fl.FCA11_SEND_EVERY, 0, "this must be a non-send frame")
    self.assertEqual(d.last_sends, [], "a non-send frame built a brake frame - the withhold would have leaked")

  def test_a_bus_level_above_the_wire_with_a_zero_target_never_raises_the_level(self):
    """The review's ordering case: the hysteresis reference is the BUS (20 LSB accepted), the wire only reached 13
    (the down-step clamp bound it), and the target drops to 0. The emitted level must never be ABOVE the wire
    (<= 13) and must fall by no more than the 3 LSB down step."""
    d = _Drive().run(200, -1.5, v=15.0, a_ego=0.0)
    d.b._cont_q_bus, d.b._cont_q_last, d.b._dec_last_sent = 20, 13, 13
    d.b._shape_acc = 13.0
    frame = d.f + (d.f % fl.FCA11_SEND_EVERY)
    self.assertEqual(frame % fl.FCA11_SEND_EVERY, 0)
    lvl = d.b._cont_level(0.0, 15.0, _cs15(), frame, STD)
    self.assertLessEqual(lvl, 13, f"the zero target raised the level above the wire: {lvl}")
    self.assertGreaterEqual(lvl, 13 - int(fl.FCA11_CONT_DOWN_STEP_LSB),
                            f"the drop exceeded the {int(fl.FCA11_CONT_DOWN_STEP_LSB)} LSB down step: {lvl}")

  def test_release_latency_from_a_held_level(self):
    """MEASURED release latency, sent frames at 50 Hz, from target 0 to wire 0. The 3 LSB/sent-frame down clamp is NOT
    what sets it: the jerk-limited slew is. Measured on the shipped tree (no band) the frame counts are the same
    10 / 12 / 19, so the ONE-LSB band does not lengthen a release by a single frame - it only changes one intermediate
    level in some sequences. Pinned so a change to the slew is visible."""
    self.assertEqual(fl.FCA11_SEND_EVERY * 0.01, 0.02)
    expected = {8: 10, 16: 12, 24: 19}
    for start, frames in expected.items():
      d = _Drive().run(200, -1.5, v=15.0, a_ego=0.0)
      d.b._cont_q_bus = d.b._cont_q_last = d.b._dec_last_sent = start
      d.b._shape_acc = float(start)
      seq = []
      for k in range(30):
        lvl = d.b._cont_level(0.0, 15.0, _cs15(), 2 * k, STD)
        d.b._dec_last_sent = lvl          # the send site does this on every frame it sends (l.4032)
        seq.append(lvl)
        if lvl == 0:
          break
      self.assertEqual(seq[-1], 0, f"{start} LSB never released: {seq}")
      self.assertEqual(len(seq), frames, f"{start} LSB released in {len(seq)} frames ({(len(seq)) * 20} ms): {seq}")
      for a, b in zip(seq, seq[1:]):
        self.assertLessEqual(a - b, int(fl.FCA11_CONT_DOWN_STEP_LSB), f"a frame dropped more than 3 LSB: {seq}")


# ---------------------------------------------------------------------------------------------------------------------
class Test0040NoTrimClose(unittest.TestCase):
  """Lane N: the aEgo trim may lower a level to the 1 LSB floor, never to 0, while the UNTRIMMED demand still asks.

  Measured on drive 195/196 (lane-K/replay_k_flags.py, 23 windows): with slice 1a every one of the 11 remaining isolated
  zeros is the same chain - the trim walks its down-rail (-0.2 .. -6.0 LSB), a demand still at -0.45 .. -1.07 m/s^2 has
  a 0.37..0.50 LSB target left, the zero-snap releases the last LSB, the episode closes, `_track_reset()` zeroes the
  trim and the same ask re-opens at 1 LSB one frame later (a 20 ms CmdAct 0->1 edge per blip). The law now decides the
  snap on the UNTRIMMED demand and floors the sum at 1 LSB on that same condition.
  """

  # 35 m/s, standard personality, gain 1.0: untrimmed LSB target for each ask (fca11_cont_lsb is the plant inverse; the
  # coast term is ~0.364 m/s^2 there, so a -0.45 ask leaves only 0.047 m/s^2 above it). 0.71 .. 1.81 LSB spans the
  # brief's 0.6..3.0 band; -0.45 gives 0.28 LSB, i.e. under the half-LSB snap.
  ASKS = (-0.52, -0.55, -0.60, -0.70)
  V = 35.0
  OPEN = -0.60           # ff 1.20 LSB: the level slice 1a can open from (its 1 LSB band holds 0 until the target is a
                         # full LSB away, so an untrimmed demand under 1.0 LSB never opens an episode on that tree)
  BELOW = -0.45          # untrimmed 0.28 LSB: under the half-LSB snap

  @staticmethod
  def _ff(ask, v=V):
    """The untrimmed demand in LSB, exactly as `_cont_level` derives it (``v`` is m/s, as `_Drive.run` takes it)."""
    bn = fl.fca11_cont_brake_need(max(0.0, -float(ask)), float(v))
    return fl.fca11_cont_lsb(fl.fca11_cont_gain(STD) * bn)

  @staticmethod
  def _pinned(d, n, ask, trim, v=V, a_ego=0.0):
    """Run ``n`` frames with the feedback trim PINNED at ``trim`` LSB before every frame (its own integrator still runs
    inside the frame, so this is the rail, not a frozen law)."""
    for _ in range(n):
      d.b._track_trim = float(trim)
      d.run(1, ask, v=v, a_ego=a_ego)
    return d

  # ---- (a) -----------------------------------------------------------------------------------------------------------
  def test_a_positive_untrimmed_demand_never_releases_under_a_down_rail_trim(self):
    """(a) untrimmed 0.6..3.0 LSB + trim forced to -6 LSB (its down-rail): the level is the 1 LSB FLOOR from the first
    actuating frame on, it is never 0, and the episode stays open. On the af4cd540 tree the same frames emit 0 and the
    episode closes (the trim eats the whole target)."""
    for ask in self.ASKS:
      ff = self._ff(ask)
      self.assertTrue(0.6 <= ff <= 3.0, f"ask {ask}: untrimmed demand {ff:.2f} LSB is outside the brief's 0.6..3.0")
      self.assertLess(ff - 6.0, 0.5, f"ask {ask}: the trim must be able to push the sum under the snap, else vacuous")
      d = _Drive().run(120, self.OPEN, v=self.V, a_ego=0.0)   # the episode is open at ~1.4 LSB
      self.assertGreaterEqual(d.wire()[-1], 1, f"ask {ask}: the episode never opened")
      self._pinned(d, 120, ask, -6.0)
      lv = d.wire()
      tail = lv[-40:]
      self.assertEqual(tail[-1], 1, f"ask {ask}: expected the 1 LSB floor, got {tail[-1]}")
      self.assertEqual(min(tail), 1, f"ask {ask}: the trim released the level: {tail}")
      self.assertEqual([b for a, b in zip(lv, lv[1:]) if a and not b], [], f"ask {ask}: a 1 -> 0 release inside the "
                                                                          f"episode: {lv}")
      self.assertTrue(d.b._actuating, f"ask {ask}: the episode closed on the trim")

  # ---- (b) -----------------------------------------------------------------------------------------------------------
  def test_b_below_half_an_lsb_the_release_is_unchanged(self):
    """(b) untrimmed < 0.5 LSB: nothing is asking for the pads, so the floor is NOT applied and the level walks to 0 and
    the episode closes - exactly the pre-change behaviour (this test is a guard, it passes on af4cd540 too)."""
    ff = self._ff(self.BELOW)
    self.assertLess(ff, 0.5, f"ask {self.BELOW}: untrimmed demand {ff:.2f} LSB is not below the snap")
    d = _Drive().run(120, self.OPEN, v=self.V, a_ego=0.0)    # open at a level the release has to walk down
    self.assertGreaterEqual(d.wire()[-1], 1)
    self._pinned(d, 80, self.BELOW, -6.0)                    # same trim rail, but no demand for the pads
    lv = d.wire()
    self.assertEqual(lv[-1], 0, f"a sub-half-LSB demand did not release: {lv[-20:]}")
    self.assertFalse(d.b._actuating, "the episode is still open at level 0")
    for a, b in zip(lv, lv[1:]):
      self.assertLessEqual(a - b, int(fl.FCA11_CONT_DOWN_STEP_LSB), f"a frame dropped more than 3 LSB: {lv}")

  # ---- (c) -----------------------------------------------------------------------------------------------------------
  def test_c_release_latency_from_a_held_level_is_unchanged(self):
    """(c) the pinned release latency 8/16/24 -> 10/12/19 sent frames, measured with the trim at its down-rail AND THE
    TRIM IN THE SUM: the demand is 0.28 LSB at 126 km/h, i.e. under the half-LSB snap, so `bn > 0`, the trim IS added
    to the target (and takes it negative), the floor is NOT applied (untrimmed < 0.5) and the level still walks to 0.
    With ask 0 the whole trim term is skipped (bn == 0 -> target 0), which is what made the earlier version of this pin
    unable to fail on its own claim (review: `Test0040NoTrimClose.test_c`)."""
    expected = {8: 10, 16: 12, 24: 19}
    ask, v_kph = self.BELOW, 126.0        # untrimmed 0.28 LSB at 35 m/s: the trim is live, the floor is not
    self.assertLess(self._ff(ask, 35.0), 0.5)
    for start, frames in expected.items():
      d = _Drive().run(200, -1.5, v=15.0, a_ego=0.0)
      d.b._cont_q_bus = d.b._cont_q_last = d.b._dec_last_sent = start
      d.b._shape_acc = float(start)
      seq = []
      for k in range(30):
        d.b._track_trim = -6.0
        before = d.b._track_trim
        lvl = d.b._cont_level(ask, v_kph, _cs15(), 2 * k, STD)
        self.assertNotEqual(d.b._track_trim, before, "the trim must be IN the target, else this pin is the old vacuous one")
        d.b._dec_last_sent = lvl        # as the send site does
        seq.append(lvl)
        if lvl == 0:
          break
      self.assertEqual(seq[-1], 0, f"{start} LSB never released: {seq}")
      self.assertEqual(len(seq), frames, f"{start} LSB released in {len(seq)} frames, pinned {frames}: {seq}")

  # ---- (d) -----------------------------------------------------------------------------------------------------------
  def test_d_the_trim_up_rail_never_raises_past_the_panda_gate_and_the_down_rail_stops_at_one_lsb(self):
    """(d) sign symmetry. +rail: a trim forced to +12 LSB cannot put a frame above the panda's +RATE_STEP over the level
    it last accepted (the outermost bound). -rail: the same forced trim cannot put the level below the 1 LSB floor."""
    d = _Drive().run(200, -1.5, v=15.0, a_ego=0.0)
    self.assertGreaterEqual(d.wire()[-1], 8)
    d.run(1, -1.5, v=15.0, a_ego=0.0, brake=True)            # driver-brake veto: the passive close frame carries 0
    self.assertEqual(d.wire()[-1], 0)
    self.assertTrue(d.b.blocked_until_resume)
    d.run(2, -1.5, v=15.0, a_ego=0.0, engaged=False)         # resume edge re-arms
    over, prev = [], 0
    for _ in range(30):
      ref = int(d.b._dec_last_sent)
      f0 = d.f
      self._pinned(d, 1, -1.5, 12.0)
      for f, lvl in d.sent:
        if f != f0:
          continue
        if lvl > ref + fl.HYUNDAI_FCA11_LONG_RATE_STEP:
          over.append((f, ref, lvl))
        prev = lvl
    self.assertEqual(over, [], f"frames above the panda's onset gate with trim +12 (frame, ref, level): {over}")
    self.assertGreater(prev, 0, "the re-open never produced a level")
    # the mirror: the same episode re-opened with the trim at -6 must sit on the floor, not on 0.
    e = _Drive().run(200, -1.5, v=15.0, a_ego=0.0)
    e.run(1, -1.5, v=15.0, a_ego=0.0, brake=True)
    e.run(2, -1.5, v=15.0, a_ego=0.0, engaged=False)
    self._pinned(e, 40, -0.55, -6.0)                         # untrimmed ~2.05 LSB while the trim is at -6
    self.assertEqual(e.wire()[-1], 1, f"the down rail did not stop at the floor: {e.wire()[-8:]}")

  # ---- (e) -----------------------------------------------------------------------------------------------------------
  def test_e_band_zero_with_the_trim_on_still_reproduces_the_0037_levels(self):
    """(e) the slice-1a equivalence pin still holds: band 0 + TRIM ON == the 0038/0039 law's level sequence. The floor is
    part of the banded quantiser's zero-snap, so the band-0 path stays plain rounding - this is the pin's own golden."""
    golden = [lv for lv, n in Test0040Corrections.GOLDEN_0037 for _ in range(n)]
    had = hasattr(fl, "FCA11_CONT_QHYST_LSB")
    old = getattr(fl, "FCA11_CONT_QHYST_LSB", None)
    try:
      if had:
        fl.FCA11_CONT_QHYST_LSB = 0.0
      self.assertTrue(_trim_live())
      d = _Drive()
      for ask, n in ((-0.80, 100), (-1.60, 100), (-2.40, 100), (-1.20, 100)):
        d.run(n, ask, v=15.0, a_ego=-1.2)
      self.assertEqual(d.levels, golden, "band 0 + TRIM ON no longer reproduces the shipped law's levels")
    finally:
      if had:
        fl.FCA11_CONT_QHYST_LSB = old


class Test0040FloorPins(unittest.TestCase):
  """Lane P: the 1 LSB floor is applied to `target`, BEFORE the hysteresis comparator and its zero-snap.

  Ordering, `fca11_long.py` `_cont_level` (shipped slice 1b): the untrimmed LSB demand is l.2756, `target` l.2757, the
  stop-band blend l.2769 and the `[0, CEIL]` clip l.2770; the floor is l.2784-2785 (`if FCA11_CONT_TRIM_ENABLED and
  untrimmed >= 0.5 and int(self._cont_q_last) > 0: target = max(target, 1.0)`), the jerk slew l.2786-2793, and the wire
  quantiser l.2805. The quantiser's first statement is the zero-snap (`target < 0.5 -> 0`, l.1207-1208), so the floor
  must have decided the frame BEFORE that line: a floor applied after the decision is a patch on the emitted integer,
  not a level, and it leaves the continuous accumulator at 0. That is what (1) pins.
  """

  V = 35.0
  # Asks that put the UNTRIMMED demand at 0.70 / 0.49 LSB at 35 m/s, standard personality. Both are asserted below, so a
  # plant-curve change cannot silently move the pin. (0.5 itself is not reachable from an ask: the inverse of the plant
  # table gives 0.4999999999999997 / 0.5000000000000001 for the two bracketing asks, 1 ulp either side - see (4).)
  AT_070 = -0.5179109776862992
  AT_049 = -0.4835759776862993

  @staticmethod
  def _ff(ask, v=V):
    """The untrimmed demand in LSB, exactly as `_cont_level` derives it (``v`` m/s, as `_Drive.run` takes it)."""
    bn = fl.fca11_cont_brake_need(max(0.0, -float(ask)), float(v))
    return fl.fca11_cont_lsb(fl.fca11_cont_gain(STD) * bn)

  @staticmethod
  def _pin(d, n, ask, trim, v=V):
    """Run ``n`` frames with the feedback trim PINNED at ``trim`` LSB before every frame (the rail, not a frozen law)."""
    for _ in range(n):
      d.b._track_trim = float(trim)
      d.run(1, ask, v=v, a_ego=0.0)
    return d

  def _open(self):
    """An open episode at >= 1 LSB, then the 0035 dynamic release ramp switched OFF.

    The ramp is a separate downstream path (`_dyn_release_level`, l.4062) that re-emits decaying levels after the law
    has already released; these pins are about `_cont_level`'s own ordering, so the observation is kept on the law.
    """
    d = _Drive().run(120, -0.60, v=self.V, a_ego=0.0)
    d.b.dynamic_brakes = False
    self.assertGreaterEqual(max(d.wire()), 1, "the episode never opened")
    return d

  # ---- (1) floor -> target -> slew -> comparator ---------------------------------------------------------------------
  def test_the_floor_reaches_the_target_before_the_quantiser_so_the_accumulator_itself_is_pinned(self):
    """(1) a level held (`_cont_q_last > 0`), untrimmed 0.70 LSB, trim at its -6 LSB down rail: the target is floored to
    1.0 BEFORE the slew and the comparator, so the WIRE is 1 for the whole rail with no 1 -> 0 edge, the episode stays
    open, AND the law's accumulator sits exactly on the floor (`_shape_acc == 1.0`). The last assertion is the one a
    floor applied after the hysteresis decision fails: that variant emits the same integer 1 but leaves `_shape_acc`
    at 0, i.e. it patches the wire while the law has already released."""
    ff = self._ff(self.AT_070)
    self.assertAlmostEqual(ff, 0.70, places=12, msg=f"untrimmed {ff!r} is not the pinned 0.70 LSB")
    self.assertLess(ff - 6.0, 0.5, "the rail must be able to push the sum under the snap, else the pin is vacuous")
    d = self._open()
    self._pin(d, 60, self.AT_070, -6.0)
    lv = d.wire()
    tail = lv[-20:]   # the first sent frames under the rail are the 2 -> 1 walk-down, the tail is the floor itself
    self.assertEqual(set(tail), {1}, f"the floored level moved off 1: {tail}")
    self.assertEqual([b for a, b in zip(lv, lv[1:]) if a and not b], [], f"a 1 -> 0 release under the floor: {lv}")
    self.assertTrue(d.b._actuating, "the episode closed under the floor")
    self.assertEqual(d.b._cont_q_last, 1)
    self.assertEqual(d.b._shape_acc, 1.0,
                     f"the floor did not reach the target/slew (accumulator {d.b._shape_acc}), it was applied after the "
                     f"quantiser")

  # ---- (2) under the boundary the zero-snap still releases -----------------------------------------------------------
  def test_under_half_an_lsb_the_floor_is_absent_and_the_zero_snap_releases(self):
    """(2) the same held level and the same -6 rail, untrimmed 0.49 LSB: the floor condition is false, the target is
    `0.49 - 6 -> 0`, and the quantiser's zero-snap releases to 0 and closes the episode within a few sent frames. The
    floor may hold a level, never create one."""
    ff = self._ff(self.AT_049)
    self.assertAlmostEqual(ff, 0.49, places=12, msg=f"untrimmed {ff!r} is not the pinned 0.49 LSB")
    self.assertLess(ff, 0.5, "the pin demands a demand below the snap")
    d = self._open()
    first_zero = None
    for k in range(30):
      self._pin(d, 1, self.AT_049, -6.0)
      if d.b._cont_q_last == 0:
        first_zero = k
        break
    lv = d.wire()
    self.assertIsNotNone(first_zero, f"a sub-half-LSB demand never released: {lv[-12:]}")
    self.assertLessEqual(first_zero, 6, f"release took {first_zero} sent frames (pinned <= 6): {lv[-12:]}")
    self.assertEqual(lv[-1], 0, f"the level did not reach 0: {lv[-12:]}")
    self.assertFalse(d.b._actuating, "the episode is still open at level 0")
    self.assertLess(d.b._shape_acc, 0.5, f"the accumulator was floored below the snap: {d.b._shape_acc}")
    for a, b in zip(lv, lv[1:]):
      self.assertLessEqual(a - b, int(fl.FCA11_CONT_DOWN_STEP_LSB), f"a frame dropped more than 3 LSB: {lv}")

  # ---- (3) no held level -> no floor, so the OPENING threshold is unchanged -------------------------------------------
  def test_without_a_held_level_there_is_no_floor_and_no_episode_opens(self):
    """(3) `_cont_q_last == 0` (nothing on the wire) + untrimmed 0.70 LSB + the same -6 rail: NO floor, nothing is sent,
    the level stays 0 and no episode opens. The `int(self._cont_q_last) > 0` half of the condition is what keeps the
    floor from lowering the opening threshold from the band's full 1.0 LSB to 0.5 - an unconditional floor opens an
    episode here, which is the slice-1a regression lane N measured (5 isolated zeros instead of 1)."""
    ff = self._ff(self.AT_070)
    self.assertAlmostEqual(ff, 0.70, places=12)
    d = _Drive()
    d.b.dynamic_brakes = False
    self._pin(d, 60, self.AT_070, -6.0)
    self.assertEqual(d.levels, [], f"the unfloored 0.5..1.0 LSB demand reached the wire: {d.levels[:8]}")
    self.assertEqual(d.b._cont_q_last, 0)
    self.assertEqual(d.b._shape_acc, 0.0, f"the accumulator was floored: {d.b._shape_acc}")
    self.assertFalse(d.b._actuating, "an episode opened below a full LSB")
    # the same frames WITH a level held do floor (1), so the difference is the gate and nothing else
    self.assertEqual(d.b._track_trim, 0.0, f"the trim was applied to a closed episode: {d.b._track_trim}")

  # ---- (4) the boundary: exactly 0.5 LSB is INCLUSIVE ----------------------------------------------------------------
  def test_at_exactly_half_an_lsb_the_floor_is_inclusive_where_plain_rounding_would_release(self):
    """(4) untrimmed EXACTLY 0.5 LSB, held level, -6 rail: the floor's `>= 0.5` is INCLUSIVE, so the target is 1.0 and
    the level is held at 1 (`round(0.5) == 0` in Python, so the pre-0040 plain-rounding quantiser - band 0, and the
    trim-off path on it - releases on this same frame). This is documented, not changed: at exactly half an LSB the
    demand is the weakest level the quantiser refuses to snap, and the trim is NOT allowed to end an episode the
    demand still asks for. NOTE the shipped 1.0 LSB band also holds 1 at exactly 0.5 when a level is already on the
    wire (|0.5 - 1| = 0.5 < 1), so the inclusive edge agrees with the band it guards and differs from plain rounding
    only on the trim-off/band-0 path."""
    real = fl.fca11_cont_lsb
    d = self._open()                       # opened on the real plant inverse, then the demand is pinned at exactly 0.5
    fl.fca11_cont_lsb = lambda need, _real=real: 0.5 if float(need) > 0.0 else 0.0
    try:
      self._pin(d, 40, self.AT_070, -6.0)   # the ask is moot: the stub pins the derived demand at exactly 0.5 LSB
    finally:
      fl.fca11_cont_lsb = real
    lv = d.wire()
    self.assertEqual(set(lv[-10:]), {1}, f"exactly 0.5 LSB was not floored: {lv[-10:]}")
    self.assertEqual(d.b._shape_acc, 1.0, f"exactly 0.5 LSB did not reach the target: {d.b._shape_acc}")
    self.assertTrue(d.b._actuating, "exactly 0.5 LSB closed the episode")
    # the comparison this pin documents: plain rounding releases at exactly 0.5, the floor and the band do not
    self.assertEqual(int(round(0.5)), 0)
    had, old = hasattr(fl, "FCA11_CONT_QHYST_LSB"), getattr(fl, "FCA11_CONT_QHYST_LSB", None)
    try:
      if had:
        fl.FCA11_CONT_QHYST_LSB = 0.0
      self.assertEqual(fl.fca11_cont_qhyst(0.5, 1), 0, "band 0 must release at exactly 0.5 (plain rounding)")
    finally:
      if had:
        fl.FCA11_CONT_QHYST_LSB = old
    self.assertEqual(fl.fca11_cont_qhyst(0.5, 1), 1, "band 1 must hold 1 at exactly 0.5")


# ----------------------------------------------------------------------------------------------------------------------
class Test0040FloorWindup(unittest.TestCase):
  """Lane W (review D1/D3): the 1 LSB FLOOR must not wind the aEgo trim up against itself, and a veto must not carry the
  wound trim across the gap.

  The floor holds `_shape_acc` at 1.0, so the trim's own anti-windup guard in `_cont_level` (`err < 0 and prev <= 0.5`)
  can never fire: with the pads delivering more than the model says at 1 LSB - the low-level gain is the least-known
  part of this plant, which is why the trim exists - the integrator walks to its -6 rail while the level sits on the
  floor, and the next REAL demand starts 4-8 LSB short of slice 1a for 1-3 s (review, 195 t=11022-11025: 1,1,2,4,8,11,15
  where 1a sends 5,6,8,10,12,16,20). Both drives are replayed through the real class from the RECORDED plan ask and
  speed of that onset (knots every 0.25 s), against the same demand with the trim PINNED AT ZERO - the demand-only arm,
  which is the trajectory slice 1a tracks.
  """

  # RECORDED onsets, plan aTarget / vEgo, one knot per 0.25 s (segment and absolute time in the header of each table).
  K195 = [  # 196--9f739d721d--3, t=11022..11026   (drive 195/196)
    (0.00, -1.087, 14.512), (0.25, -1.206, 14.273), (0.50, -1.176, 14.036), (0.75, -1.180, 13.783),
    (1.00, -1.263, 13.545), (1.25, -1.297, 13.237), (1.50, -1.365, 12.932), (1.75, -1.385, 12.643),
    (2.00, -1.409, 12.332), (2.25, -1.494, 12.015), (2.50, -1.573, 11.688), (2.75, -1.536, 11.275),
    (3.00, -1.560, 10.838), (3.25, -1.585, 10.396), (3.50, -1.568, 9.927), (3.75, -1.599, 9.477),
    (4.00, -1.618, 9.040),
  ]
  K197 = [  # 197--d3d743112e--8, t=23263..23267
    (0.00, -1.058, 8.776), (0.25, -1.060, 8.593), (0.50, -0.974, 8.425), (0.75, -0.876, 8.317),
    (1.00, -0.871, 8.101), (1.25, -0.943, 7.941), (1.50, -0.993, 7.783), (1.75, -0.996, 7.613),
    (2.00, -0.963, 7.411), (2.25, -1.080, 7.246), (2.50, -1.039, 7.043), (2.75, -0.991, 6.824),
    (3.00, -0.959, 6.589), (3.25, -1.009, 6.385), (3.50, -1.031, 6.181), (3.75, -1.068, 5.975),
    (4.00, -1.033, 5.744),
  ]
  FLOORED = -0.52          # 0.71 LSB of untrimmed demand at 35 m/s: the floor, not the demand, is what holds the level
  MISMATCH = 3.0           # the pads deliver 3x the documented model at 1 LSB: the plant error the trim exists to take out

  @staticmethod
  def _plant(lsb: float) -> float:
    """The documented plant, LSB -> m/s^2 (FCA11_CONT_PLANT_LSB/MSS2), linearly between its knots."""
    L, A = fl.FCA11_CONT_PLANT_LSB, fl.FCA11_CONT_PLANT_MSS2
    lsb = max(0.0, min(float(A[-1]), float(lsb)))
    for k in range(len(L) - 1):
      if L[k] <= lsb <= L[k + 1]:
        f = (lsb - L[k]) / (L[k + 1] - L[k])
        return A[k] + f * (A[k + 1] - A[k])
    return A[-1]

  @staticmethod
  def _knot(knots, t: float):
    if t <= knots[0][0]:
      return knots[0][1:]
    for (t0, *a), (t1, *b) in zip(knots, knots[1:]):
      if t0 <= t <= t1:
        f = (t - t0) / (t1 - t0)
        return tuple(x + f * (y - x) for x, y in zip(a, b))
    return knots[-1][1:]

  def _wound(self, seconds: int, pin: bool = False):
    """A deep demand, then the sag to the floored one held ``seconds`` 100 Hz frames with the pads over-delivering.
    ``pin`` keeps the trim at zero (the demand-only reference arm)."""
    d = _Drive()
    coast = fl.fca11_cont_coast(35.0)
    for _ in range(120):
      if pin:
        d.b._track_trim = 0.0
      d.run(1, -1.5, v=35.0, a_ego=-(coast + self._plant(d.b._cont_q_last)))
    for _ in range(seconds):
      if pin:
        d.b._track_trim = 0.0
      d.run(1, self.FLOORED, v=35.0, a_ego=-(coast + self.MISMATCH * self._plant(d.b._cont_q_last)))
    return d

  def _ramp(self, d, knots, pin: bool = False):
    """The recorded onset, 4 s at 100 Hz, aEgo from the DOCUMENTED plant (so the comparison measures the trim state, not
    a plant artefact). Returns the sent levels of the ramp."""
    out = []
    for i in range(400):
      ask, v = self._knot(knots, i * 0.01)
      if pin:
        d.b._track_trim = 0.0
      before = len(d.sent)
      d.run(1, ask, v=v, a_ego=-(fl.fca11_cont_coast(v) + self._plant(d.b._cont_q_last)))
      for f, lv in d.sent[before:]:
        if f % fl.FCA11_SEND_EVERY == 0:
          out.append(lv)
    return out

  # ---- (a) onset tracking ---------------------------------------------------------------------------------------------
  def test_a_the_recorded_onset_tracks_the_demand_within_one_lsb_after_a_floored_hold(self):
    """(a) a 5 s floored hold, then the REAL onset of 195 t=11022 and 197 t=23263: every SENT frame of the ramp is within
    1 LSB of the demand-only arm. On 3269e8fd the trim is at -5 LSB at the ramp start, the floor holds the level at 1
    while the demand asks 4-5, and the deficit reaches 4-5 LSB for ~0.4 s of sent frames (review D1)."""
    for knots, tag in ((self.K195, "195 t=11022"), (self.K197, "197 t=23263")):
      d = self._wound(250)
      self.assertEqual(int(d.b._cont_q_last), 1, f"{tag}: the hold did not sit on the floor")
      self.assertEqual(d.b._shape_acc, 1.0, f"{tag}: the floor is not what held the level")
      got = self._ramp(d, knots)
      ref = self._ramp(self._wound(250, pin=True), knots)
      self.assertGreaterEqual(max(ref), 5, f"{tag}: the reference arm never asked for a real level: {ref[:20]}")
      self.assertEqual(len(got), len(ref), f"{tag}: the two arms did not send the same frames")
      deficit = [ref[i] - got[i] for i in range(len(got))]
      self.assertLessEqual(max(deficit), 1,
                           f"{tag}: level {max(deficit)} LSB below the demand at the onset\n"
                           f"    this {got[:30]}\n    demand {ref[:30]}")

  # ---- (b) the wind-up itself -----------------------------------------------------------------------------------------
  def test_b_three_seconds_on_the_floor_leaves_the_trim_above_minus_two_lsb(self):
    """(b) 3 s floored at 1 LSB with the pads over-delivering: the trim stays above -2 LSB. It integrates at
    KI*err*dt = 8 * -0.33 * 0.02 = -0.05 LSB per sent frame if nothing stops it, i.e. -2.4 LSB in the 3 s the brief
    names, and on to the -6 rail - which is exactly the review's state at the next onset."""
    d = self._wound(300)
    self.assertEqual(int(d.b._cont_q_last), 1, "the hold did not sit on the floor")
    self.assertGreater(d.b._track_trim, -2.0,
                       f"3 s on the floor wound the trim to {d.b._track_trim:+.2f} LSB")
    # and it is the BACK-CALCULATION that bounds it: the applied correction is 1 - untrimmed
    untrimmed = Test0040NoTrimClose._ff(self.FLOORED, 35.0)
    self.assertGreaterEqual(d.b._track_trim, 1.0 - untrimmed - 1e-9,
                            f"the trim is below the correction actually applied ({1.0 - untrimmed:+.2f})")

  # ---- (c) the veto gap ----------------------------------------------------------------------------------------------
  def test_c_a_gas_veto_does_not_carry_the_wound_trim_across_the_gap(self):
    """(c) review D3: a gas press closes the episode WITHOUT calling `_cont_level`, so a -5 LSB trim used to survive the
    gap (`_track_reset` only fires once the accumulator decays) and the demand after the lift started 4-5 LSB short.
    Measured: the trim is exactly 0 on the first frame after the gap, and every sent frame of the resumed ramp is
    within 1 LSB of the demand-only arm."""
    d = self._wound(250)
    self.assertEqual(int(d.b._cont_q_last), 1, "the scenario did not sit on the floor, so it proves nothing")
    self.assertEqual(d.b._shape_acc, 1.0, "the scenario's level is not held by the floor")
    d.run(20, -0.7, v=35.0, a_ego=-0.9, gas=True)      # the driver presses the gas: ok == False, the episode closes
    self.assertFalse(d.b._actuating, "the gas press did not close the episode")
    self.assertEqual(d.b._track_trim, 0.0,
                     f"the wound trim ({d.b._track_trim:+.2f} LSB) crossed the gas gap")
    got = self._ramp(d, self.K195)
    ref = self._ramp(self._wound(250, pin=True), self.K195)
    n = min(len(got), len(ref))
    self.assertGreater(n, 50, "the resumed ramp never ran")
    deficit = [ref[i] - got[i] for i in range(n)]
    self.assertLessEqual(max(deficit), 1,
                         f"after the gas gap the level is {max(deficit)} LSB below the demand\n"
                         f"    this {got[:30]}\n    demand {ref[:30]}")


if __name__ == "__main__":
  unittest.main()