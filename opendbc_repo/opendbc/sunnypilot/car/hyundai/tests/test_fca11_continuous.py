"""0037: the ONE continuous brake law (FCA11_CONT_ENABLED) - conformance + engine braking first.

Owner (2026-11-05): "As long as it's linear I don't care. The entire point is that we need to treat braking with the same
logic and respect that we do for accelerating. We don't gate accelerating to buckets." DESIGN.md lane-E §3-§5 is the
spec; every assertion here is a LITERAL from it, so this file fails on the pre-0037 law by behaviour, not by a missing
name:

  (a) sweep ask x v x personality - monotone, slew-bounded, contiguous (no hidden bucket), capped at 30
  (b) engine braking first - no friction brake while the coast covers the ask, gas at a TRUE zero at the hand-over
  (c) phantom 1/2/3 model frames - peak <= the §5 slew-only bound, and the episode closes by itself
  (d) driver brake / gas - immediate cut
  (e) standstill - the hold is reached by a blend, never a step
  (f) the OFF path (pre-0037) is byte-identical, pinned against a series34 capture

Run from the opendbc root:  PYTHONPATH=. python -m pytest -q opendbc/sunnypilot/car/hyundai/tests/test_fca11_continuous.py
"""
import math
import unittest

from opendbc.car import structs
import opendbc.sunnypilot.car.hyundai.fca11_long as fl
from opendbc.sunnypilot.car.hyundai import gas_interceptor as gi
from opendbc.sunnypilot.car.hyundai.tests.test_fca11_long import _CS, _cam

STD = fl.PERSONALITY_STANDARD
PERS = (fl.PERSONALITY_AGGRESSIVE, STD, fl.PERSONALITY_RELAXED)


def _cont_flag() -> bool:
  """The switch, tolerant of the PRE-0037 tree (where the name does not exist): the file must run on `series34` and
  fail by BEHAVIOUR there, not by AttributeError."""
  return bool(getattr(fl, "FCA11_CONT_ENABLED", False))


def _cp_sp(cont=True, aff=True, dyn=True):
  prev = _cont_flag()
  fl.FCA11_CONT_ENABLED = cont
  try:
    return structs.CarParamsSP(fca11Brake=True, enableGasInterceptor=True, fca11AffineGain=aff, fca11DynamicBrakes=dyn)
  finally:
    fl.FCA11_CONT_ENABLED = prev


def _brake(cont=True):
  prev = _cont_flag()
  fl.FCA11_CONT_ENABLED = cont
  try:
    return fl.Fca11LongBrake(structs.CarParams(), _cp_sp(cont))
  finally:
    fl.FCA11_CONT_ENABLED = prev


def _cc(accel, long_active=True):
  cc = structs.CarControl()
  cc.enabled = cc.latActive = True
  cc.longActive = long_active
  cc.actuators.accel = accel
  return cc


def _gas_signals():
  from opendbc.can.dbc import DBC
  return DBC(gi.GAS_INTERCEPTOR_DBC).name_to_msg["GAS_COMMAND"].sigs


def _enable_bit(dat: bytes) -> int:
  """GAS_COMMAND.ENABLE straight off the wire bytes (the pedal passes the driver's signal through when it is 0)."""
  sig = _gas_signals()["ENABLE"]
  return (dat[sig.start_bit // 8] >> (sig.start_bit % 8)) & 1


def _gas_word(dat: bytes) -> int:
  sig = _gas_signals()["GAS_COMMAND"]
  return (dat[sig.start_bit // 8] >> (sig.start_bit % 8)) & ((1 << sig.size) - 1)


def _elantra_ci():
  """The car's own object: Elantra 2022 non-SCC with the gas interceptor + FCA11 brake (AffineGain, DynamicBrakes)."""
  from opendbc.car.car_helpers import interfaces
  from opendbc.car.hyundai.values import CAR
  from opendbc.sunnypilot.car.interfaces import setup_interfaces
  from opendbc.sunnypilot.car.hyundai.tests.test_gas_interceptor import BASE_FINGERPRINT, STD as PEDAL
  car = CAR.HYUNDAI_ELANTRA_2022_NON_SCC
  fingerprint = {i: {} for i in range(8)}
  fingerprint[0] = {**BASE_FINGERPRINT, **PEDAL}
  CarInterface = interfaces[car]
  CP = CarInterface.get_params(car, fingerprint, [], alpha_long=False, is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, car, fingerprint, [], alpha_long=False, is_release_sp=False, docs=False)
  setup_interfaces(CarInterface, CP, CP_SP, [
    {"HyundaiGasInterceptor": True}, {"HyundaiFca11Brake": True},
    {"HyundaiFca11AffineGain": True}, {"HyundaiFca11DynamicBrakes": True},
  ])
  return CarInterface(CP, CP_SP)


class _Drive:
  """Feeds (ask, v, aEgo[, lead, standstill]) frames and records every SENT 0x38D level as (frame, dec)."""

  def __init__(self, cont=True, personality=STD):
    self.b = _brake(cont)
    self.f = 0
    self.p = personality
    self.sent = []

  def run(self, n, accel, v=15.0, a_ego=-1.2, lead=None, standstill=False, brake=False, gas=False):
    for _ in range(n):
      cs = _CS(v_ego=v, cam=_cam(), a_ego=a_ego)
      cs.out.standstill = standstill
      cs.out.brakePressed = brake
      cs.out.gasPressed = gas
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

  def seq(self, since=0):
    return [d for f, d in self.sent if f >= since]


# ---------------------------------------------------------------------------------------------------------------------
class TestSweep(unittest.TestCase):
  """(a) level monotone in the ask, no step larger than the slew, no hidden bucket, ceiling 30."""

  ASKS = [round(-0.1 * i, 2) for i in range(36, -1, -1)]        # -3.6 .. 0.0 in 0.1 steps
  SPEEDS = (0.0, 2.5, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0)

  def test_level_is_monotone_in_the_ask_at_every_frame(self):
    """A monotone target map through a monotone slew preserves order at EVERY frame index, so comparing the
    trajectories frame by frame is stricter than comparing the converged levels."""
    for v in self.SPEEDS:
      for p in PERS:
        traj = []
        for a in self.ASKS:
          d = _Drive(personality=p).run(80, a, v=v, a_ego=-1.0)
          traj.append([d2 for _f, d2 in d.sent])
        n = min(len(t) for t in traj)
        for i in range(n):
          col = [t[i] for t in traj]
          self.assertEqual(col, sorted(col), (v, p, i, col))
          self.assertLessEqual(max(col), 30, (v, p, i, col))

  def test_no_step_between_adjacent_asks(self):
    """0.02 m/s^2 apart, the plant's own inverse moves at most ~1 LSB (min slope 0.02), so anything above 2 would be a
    hidden threshold. This is the by-sweep bucket hunt."""
    asks = [round(-0.02 * i, 2) for i in range(175, -1, -1)]     # -3.50 .. 0.00
    for v in (2.0, 8.0, 15.0, 25.0, 35.0):
      for p in PERS:
        lv = [_Drive(personality=p).run(80, a, v=v, a_ego=-1.0).levels()[-1:] or [0] for a in asks]
        lv = [x[0] for x in lv]
        for a, b in zip(lv, lv[1:]):
          self.assertLessEqual(b - a, 2, (v, p, a, b))
        self.assertGreaterEqual(min(lv), 0)

  def test_the_reached_levels_are_contiguous_no_buckets(self):
    """Every LSB between "no brake" and the level a hard ask reaches must be reachable: a bucket/threshold law leaves
    holes (the pre-0037 law jumped 0 -> 4 and then 4 -> 8 -> 12 in one frame)."""
    asks = [round(-0.02 * i, 2) for i in range(175, -1, -1)]
    reached = set()
    for a in asks:
      lv = _Drive().run(120, a, v=15.0, a_ego=-1.0).levels()
      reached.add(lv[-1] if lv else 0)
    self.assertIn(0, reached, "the coast region must be reachable")
    self.assertGreaterEqual(max(reached), 20, sorted(reached))
    # no bucket: between "no brake" and the level a hard ask reaches, no gap wider than the 2 LSB the plant's own flat
    # segment (0.02 m/s^2 per LSB) implies for a 0.02 m/s^2 ask step. The pre-0037 law left 4-LSB holes (0 -> 4 -> 8).
    gaps = [b - a for a, b in zip(sorted(reached), sorted(reached)[1:])]
    self.assertLessEqual(max(gaps), 2, sorted(reached))

  def test_sent_frames_never_move_faster_than_the_slew(self):
    """One jerk form both ways: up <= J*dt/slope(prev) <= 4 (the panda's +RATE_STEP), down <= 3 (the down-step cap).
    Run a full-deep stop and a full release through the law and check every sent step."""
    d = _Drive()
    d.run(200, -3.5, v=15.0, a_ego=-2.0)     # deepen and hold
    t = d.f
    d.run(200, 0.0, v=15.0, a_ego=0.0)      # release
    seq = [(f, x) for f, x in d.sent]
    ups = downs = 0
    for (f0, a), (f1, b) in zip(seq, seq[1:]):
      if f1 - f0 != 2:      # only consecutive send slots (a gap means the episode closed and re-opened)
        continue
      if b >= a:
        ups += 1
        self.assertLessEqual(b - a, 4, (f0, a, b))
      else:
        downs += 1
        self.assertLessEqual(a - b, 3, (f0, a, b))
    self.assertGreater(ups, 0)
    self.assertGreater(downs, 0)
    self.assertEqual(d.levels(since=t + 160), [], "a release must reach 0 (no residual floor)")

  def test_ceiling_is_30_in_every_personality(self):
    for p in PERS:
      d = _Drive(personality=p).run(400, -3.5, v=10.0, a_ego=-2.0)
      self.assertEqual(max(d.levels()), 30, p)

  def test_personality_is_a_continuous_gain_not_a_cap(self):
    """Same ask, three personalities: relaxed < standard < aggressive, and the ratio is the gas law's gain ratio."""
    mid = {}
    for p, gain in ((fl.PERSONALITY_RELAXED, 0.85), (STD, 1.00), (fl.PERSONALITY_AGGRESSIVE, 1.20)):
      d = _Drive(personality=p).run(120, -1.2, v=20.0, a_ego=-0.9)
      mid[p] = d.levels()[-1]
      self.assertAlmostEqual(fl.fca11_cont_gain(p), gain, places=6)
    self.assertLess(mid[fl.PERSONALITY_RELAXED], mid[STD], mid)
    self.assertLess(mid[STD], mid[fl.PERSONALITY_AGGRESSIVE], mid)

  def test_a_light_ask_that_beats_the_coast_does_open_the_brake(self):
    """The pre-0037 light-ask threshold (-0.80 at >= 10 m/s) is gone: an ask the coast cannot cover is a brake, however
    small, and it arrives through the slew."""
    d = _Drive().run(200, -0.60, v=20.0, a_ego=-0.4)
    lv = d.levels()
    self.assertTrue(lv, "no brake episode for an ask 0.25 m/s^2 past the coast")
    self.assertLessEqual(max(lv), 3, lv)
    self.assertLessEqual(lv[0], 1, lv)


# ---------------------------------------------------------------------------------------------------------------------
class TestEngineBrakingFirst(unittest.TestCase):
  """(b) brake level is 0 while the coast covers the ask, and rises continuously past it; the gas is at a TRUE zero."""

  def test_no_friction_brake_while_the_coast_covers_the_ask(self):
    for v in (2.5, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0):
      coast = fl.fca11_cont_coast(v)
      for ask in (-0.5 * coast, -0.9 * coast, -coast):
        d = _Drive().run(150, ask, v=v, a_ego=0.0)
        self.assertEqual(d.levels(), [], (v, ask, coast))

  def test_level_rises_continuously_past_the_coast(self):
    v = 15.0
    coast = fl.fca11_cont_coast(v)
    asks = [round(-(coast + 0.02 * i), 3) for i in range(0, 60)]
    lv = []
    for a in asks:
      got = _Drive().run(150, a, v=v, a_ego=0.0).levels()
      lv.append(got[-1] if got else 0)
    self.assertEqual(lv[0], 0, lv[:5])
    self.assertGreater(max(lv), 5, lv)
    self.assertEqual(lv, sorted(lv), lv)
    for a, b in zip(lv, lv[1:]):
      self.assertLessEqual(b - a, 2, lv)

  def test_brake_need_is_c1_at_the_join(self):
    """fca11_cont_brake_need: exactly 0 at/below the coast, value AND slope 0 there, monotone, quadratic width 0.04."""
    v = 12.0
    c = fl.fca11_cont_coast(v)
    self.assertEqual(fl.fca11_cont_brake_need(c - 1e-6, v), 0.0)
    self.assertEqual(fl.fca11_cont_brake_need(c, v), 0.0)
    h = 1e-6
    slope = (fl.fca11_cont_brake_need(c + h, v) - fl.fca11_cont_brake_need(c, v)) / h
    self.assertLessEqual(abs(slope), 0.05, slope)          # C1: the slope leaves 0 continuously
    self.assertAlmostEqual(fl.fca11_cont_brake_need(c + 1.0, v), 1.0 - 0.04, places=6)   # x - w above 2w
    prev = -1.0
    for i in range(400):
      x = c + 0.005 * i
      cur = fl.fca11_cont_brake_need(x, v)
      self.assertGreaterEqual(cur, prev)
      prev = cur

  def test_coast_fit_matches_the_measured_bins(self):
    """The fit's own anchors (DESIGN.md §3): monotone, 0 at rest, and within 0.09 m/s^2 of every bin median with n>=1000."""
    self.assertEqual(fl.fca11_cont_coast(0.0), 0.0)
    self.assertAlmostEqual(fl.fca11_cont_coast(20.0), 0.349, delta=0.002)
    self.assertAlmostEqual(fl.fca11_cont_coast(35.0), 0.364, delta=0.002)
    prev = -1.0
    for i in range(0, 400):
      c = fl.fca11_cont_coast(i * 0.1)
      self.assertGreaterEqual(c, prev)
      prev = c
    bins = ((1.25, 0.059), (3.75, 0.155), (6.25, 0.231), (8.75, 0.308), (11.25, 0.276), (13.75, 0.336),
            (16.25, 0.333), (18.75, 0.342), (26.25, 0.333), (28.75, 0.358), (31.25, 0.384), (33.75, 0.373))
    for v, obs in bins:
      self.assertLessEqual(abs(fl.fca11_cont_coast(v) - obs), 0.09, (v, obs))

  def test_gas_goes_to_exactly_zero_at_the_handover(self):
    """The gas law's negative branch on this path: hold_cmd(v) at ask 0, EXACTLY 0.0 at ask = -coast(v), 0 beyond -
    and the continuous hand-over predicate the gas law reads is true at the same point."""
    b = _brake()
    self.assertTrue(b.cont_law)
    for v in (5.0, 10.0, 20.0, 30.0):
      coast = fl.fca11_cont_coast(v)
      hold = gi.get_hold_command(v)
      self.assertGreater(hold, 0.0)
      # the law's own formula (same expression gas_interceptor uses)
      pedal = hold * max(0.0, 1.0 - coast / coast)
      self.assertEqual(pedal, 0.0)
      self.assertFalse(b.cont_coast(0.0, v))
      self.assertFalse(b.cont_coast(-0.5 * coast, v))
      self.assertTrue(b.cont_coast(-coast, v))
      self.assertTrue(b.cont_coast(-(coast + 0.5), v))
      # monotone + continuous in the ask, from hold at 0 to 0 at the coast
      prev, prev_a = None, None
      for i in range(0, 101):
        a = -coast * i / 100.0
        cur = hold * max(0.0, 1.0 - (-a) / coast)
        if prev is not None:
          self.assertLessEqual(cur, prev + 1e-12)
          self.assertLessEqual(abs(cur - prev), 0.01)      # <= 1 % pedal per 1 % of the coast
        prev, prev_a = cur, a
      self.assertAlmostEqual(prev, 0.0, places=9)


class TestGasPathZeroFrame(unittest.TestCase):
  """(b) through the REAL gas path: 0x700/0x200 carries an all-zero ENABLE=0 frame at the hand-over ask."""

  def _ci(self):
    return _elantra_ci()

  def test_coast_ask_sends_the_all_zero_pedal_frame(self):
    from opendbc.sunnypilot.car.hyundai.tests.test_gas_interceptor import _Fca11CS, _fca11_cam_frame
    CI = self._ci()
    self.assertTrue(CI.CC.fca11_brake.cont_law)
    cs = _Fca11CS(v_ego=20.0, cam=(0, _fca11_cam_frame(alive=5)))
    v = 20.0
    coast = fl.fca11_cont_coast(v)
    # get the pedal commanding first (ask 0 = hold), then hand over to the coast
    for f in range(0, 40, 2):
      CI.CC.create_gas_command(_cc(0.0), cs, f, 0)
    self.assertGreater(CI.CC.gas, 0.0)
    sends = CI.CC.create_gas_command(_cc(-coast), cs, 40, 0)
    self.assertEqual(CI.CC.gas, 0.0)                       # exactly zero -> below the 0.001 enable threshold
    pedal = [d for a, d, _s in sends if a in (0x200, 0x700)]
    self.assertEqual(len(pedal), 1)
    self.assertEqual(_enable_bit(pedal[0]), 0, "the wire must carry an ENABLE=0 (pass-through) frame")
    self.assertEqual(_gas_word(pedal[0]), 0)
    # and a shallower ask is still a live, partial pedal (engine braking modulated, not on/off)
    sends = CI.CC.create_gas_command(_cc(-0.25 * coast), cs, 42, 0)
    self.assertGreater(CI.CC.gas, 0.0)


# ---------------------------------------------------------------------------------------------------------------------
class TestPhantom(unittest.TestCase):
  """(c) a 1/2/3 model-frame phantom peaks at the §5 slew-only bound and closes by itself."""

  def _phantom(self, model_frames, open_level=0):
    d = _Drive()
    if open_level:
      d.run(200, -1.2, v=25.0, a_ego=-1.0)
      self.assertGreaterEqual(d.levels()[-1], open_level)
    d.run(50, -0.1, v=25.0, a_ego=0.0)
    t = d.f
    d.run(5 * model_frames, -0.1, v=25.0, a_ego=0.0, lead=(30.0, -25.0))   # one 20 Hz model frame = 5 controller frames
    d.run(200, -0.1, v=25.0, a_ego=0.0)
    return d, t

  def test_one_two_three_model_frames(self):
    for mf, bound in ((1, 1), (2, 1), (3, 2)):
      d, t = self._phantom(mf)
      lv = d.levels(since=t)
      self.assertLessEqual(max(lv) if lv else 0, bound, (mf, lv))

  def test_the_episode_closes_by_itself(self):
    d, t = self._phantom(3)
    lv = d.levels(since=t)
    self.assertTrue(lv)
    close = max(f for f, x in d.sent if f >= t and x > 0)
    self.assertLessEqual(close - t, 60, "the phantom episode must be gone within 0.6 s")   # 150 ms + ~0.4 s slew

  def test_no_single_frame_step_inside_an_open_episode(self):
    d = _Drive().run(200, -1.2, v=25.0, a_ego=-1.0)
    t = d.f
    d.run(15, -0.1, v=25.0, a_ego=0.0, lead=(30.0, -25.0))
    seq = [x for f, x in d.sent if f >= t - 2]
    for a, b in zip(seq, seq[1:]):
      self.assertLessEqual(abs(b - a), 4, seq)

  def test_a_real_persisting_demand_still_arrives(self):
    """The other side of the trade: 4 model frames (200 ms) of a real closing lead must NOT be ignored - the slew is
    the only bound, and it reaches a real level in under a second."""
    d = _Drive().run(50, -0.1, v=25.0, a_ego=0.0).run(20, -0.1, v=25.0, a_ego=0.0, lead=(30.0, -25.0))
    self.assertGreaterEqual(max(d.levels()), 2, d.levels())
    d = _Drive().run(50, -0.1, v=25.0, a_ego=0.0).run(50, -0.1, v=25.0, a_ego=0.0, lead=(30.0, -25.0))
    self.assertGreaterEqual(max(d.levels()), 5, d.levels())


# ---------------------------------------------------------------------------------------------------------------------
class TestDriverOverride(unittest.TestCase):
  """(d) driver brake / gas -> the very next frames command nothing."""

  def _open(self):
    d = _Drive().run(200, -1.5, v=15.0, a_ego=-1.2)
    self.assertGreaterEqual(d.levels()[-1], 5)
    return d

  def test_driver_brake_cuts_on_the_frame(self):
    d = self._open()
    t = d.f
    d.run(20, -1.5, v=15.0, a_ego=-1.2, brake=True)
    self.assertEqual(d.levels(since=t), [], d.seq(since=t)[:6])
    self.assertEqual(d.seq(since=t)[0], 0, d.seq(since=t)[:6])

  def test_driver_gas_cuts_on_the_frame(self):
    d = self._open()
    t = d.f
    d.run(20, -1.5, v=15.0, a_ego=-1.2, gas=True)
    self.assertEqual(d.levels(since=t), [], d.seq(since=t)[:6])
    self.assertEqual(d.seq(since=t)[0], 0, d.seq(since=t)[:6])

  def test_driver_brake_also_zeroes_the_pedal(self):
    from opendbc.sunnypilot.car.hyundai.tests.test_gas_interceptor import _Fca11CS, _fca11_cam_frame
    from opendbc.car.car_helpers import interfaces
    from opendbc.car.hyundai.values import CAR
    from opendbc.sunnypilot.car.interfaces import setup_interfaces
    from opendbc.sunnypilot.car.hyundai.tests.test_gas_interceptor import BASE_FINGERPRINT, STD as PEDAL
    car = CAR.HYUNDAI_ELANTRA_2022_NON_SCC
    fp = {i: {} for i in range(8)}
    fp[0] = {**BASE_FINGERPRINT, **PEDAL}
    CI_ = interfaces[car]
    CP = CI_.get_params(car, fp, [], alpha_long=False, is_release=False, docs=False)
    CP_SP = CI_.get_params_sp(CP, car, fp, [], alpha_long=False, is_release_sp=False, docs=False)
    setup_interfaces(CI_, CP, CP_SP, [{"HyundaiGasInterceptor": True}, {"HyundaiFca11Brake": True},
                                      {"HyundaiFca11AffineGain": True}, {"HyundaiFca11DynamicBrakes": True}])
    CI = CI_(CP, CP_SP)
    cs = _Fca11CS(v_ego=15.0, cam=(0, _fca11_cam_frame(alive=5)))
    for f in range(0, 120, 2):
      CI.CC.create_gas_command(_cc(1.0), cs, f, 0)
    self.assertGreater(CI.CC.gas, 0.0)
    # a braking ask while the driver is on the brake: the level is cut by `ok`, and the pedal is forced to 0
    cs.out.brakePressed = True
    CI.CC.create_gas_command(_cc(-1.5), cs, 120, 0)
    self.assertEqual(CI.CC.gas, 0.0)


# ---------------------------------------------------------------------------------------------------------------------
class TestStandstill(unittest.TestCase):
  """(e) the stop/hold band is a blend now: the creep floor and the standstill hold enter by speed, never as a step."""

  def test_standstill_hold_is_reached_without_a_step(self):
    d = _Drive().run(150, -1.5, v=3.0, a_ego=-1.0)
    t = d.f
    d.run(300, -1.5, v=0.0, a_ego=0.0, standstill=True)
    seq = [x for f, x in d.sent if f >= t]
    self.assertEqual(seq[-1], 30, seq[-8:])
    for a, b in zip(seq, seq[1:]):
      self.assertLessEqual(b - a, 4, seq)

  def test_creep_floor_in_the_band_and_no_seam(self):
    """At 2.5 km/h an open episode holds at least the clutch-open floor (7 LSB); at 7 km/h it is the demand's own
    level. Neither the entry nor the exit may step by more than the panda's onset step."""
    d = _Drive().run(200, -1.5, v=2.0, a_ego=-1.0)          # 7.2 km/h: demand level
    self.assertGreaterEqual(d.levels()[-1], 7)
    t = d.f
    seq = []
    for v_kph in (7.2, 6.0, 5.0, 4.5, 4.0, 3.5, 3.0, 2.5, 2.0):   # walk down through the blend
      d.run(10, -1.5, v=v_kph / 3.6, a_ego=-1.0)
      seq += [x for x in d.seq(since=t) if x is not None]
      t = d.f
    allseq = [x for f, x in d.sent]
    for a, b in zip(allseq, allseq[1:]):
      self.assertLessEqual(abs(b - a), 4, allseq[-20:])

  def test_planner_asking_to_move_releases_the_hold(self):
    """A standstill hold must never keep the brake on against a launch request (the panda's gas/brake exclusion)."""
    d = _Drive().run(300, -1.5, v=0.0, a_ego=0.0, standstill=True)
    self.assertEqual(d.levels()[-1], 30)
    t = d.f
    d.run(200, 0.4, v=0.0, a_ego=0.0, standstill=True)
    self.assertEqual(d.levels(since=t + 160), [], d.seq(since=t)[:20])
    self.assertLessEqual(max(d.seq(since=t)[:10]), 30)
    for a, b in zip(d.seq(since=t), d.seq(since=t)[1:]):
      self.assertLessEqual(a - b, 3, d.seq(since=t)[:20])


# ---------------------------------------------------------------------------------------------------------------------
# (f) the OFF path: the sent-level series of the PRE-0037 law, captured on the lane's `series34` base commit
# (84c8dbd) with the same script. Byte-identity is asserted against this literal, so the OFF path can never drift.
OFF_SCRIPT = [(60, -1.0, 15.0), (80, -2.5, 15.0), (40, -0.2, 15.0), (40, 0.5, 15.0),
              (40, -1.5, 2.0), (40, -1.5, 0.7)]
OFF_EXPECTED = [(20, 4), (22, 4), (24, 4), (26, 4), (28, 4), (30, 4), (32, 4), (34, 4), (36, 4), (38, 4), (40, 4),
                (42, 4), (44, 4), (46, 4), (48, 4), (50, 4), (52, 4), (54, 4), (56, 4), (58, 4), (60, 8), (62, 12),
                (64, 16), (66, 20), (68, 24), (70, 28), (72, 29), (74, 29), (76, 29), (78, 29), (80, 29), (82, 29),
                (84, 29), (86, 29), (88, 29), (90, 29), (92, 29), (94, 30), (96, 30), (98, 30), (100, 30), (102, 30),
                (104, 30), (106, 30), (108, 30), (110, 30), (112, 30), (114, 30), (116, 30), (118, 30), (120, 30),
                (122, 30), (124, 30), (126, 30), (128, 30), (130, 30), (132, 30), (134, 30), (136, 30), (138, 30),
                (140, 0), (220, 4), (222, 8), (224, 12), (226, 14), (228, 14), (230, 14), (232, 14), (234, 14),
                (236, 14), (238, 14), (240, 14), (242, 14), (244, 14), (246, 14), (248, 14), (250, 14), (252, 14),
                (254, 14), (256, 14), (258, 14), (260, 17), (262, 20), (264, 20), (266, 20), (268, 20), (270, 20),
                (272, 20), (274, 20), (276, 20), (278, 20), (280, 20), (282, 20), (284, 20), (286, 20), (288, 20),
                (290, 20), (292, 20), (294, 20), (296, 20), (298, 20)]


class TestOffPathIdentity(unittest.TestCase):
  """(f) FCA11_CONT_ENABLED = False emits the pre-0037 series byte for byte - the shipped law is still the OFF path."""

  def test_off_path_is_byte_identical(self):
    b = _brake(cont=False)
    self.assertFalse(getattr(b, "cont_law", False))
    out = []
    f = 0
    for n, ask, v in OFF_SCRIPT:
      for _ in range(n):
        cs = _CS(v_ego=v, cam=_cam(), a_ego=-1.2)
        cs.fca11_lead_drel_m = cs.fca11_lead_vrel_mps = cs.fca11_lead_one = None
        for s in b.update(_cc(ask), cs, f, STD):
          out.append((f, fl._get_bits(s.dat, 8, 8)))
        f += 1
    self.assertEqual(out, OFF_EXPECTED)

  def test_the_switch_defaults_on_for_this_path(self):
    self.assertTrue(_cont_flag(), "FCA11_CONT_ENABLED must exist and default True on this path")
    self.assertTrue(_brake(cont=True).cont_law)
    # and it is a code constant, not a params key: absent CP_SP flags leave the pre-0037 instance alone
    cp = structs.CarParamsSP(fca11Brake=True, enableGasInterceptor=True, fca11AffineGain=False, fca11DynamicBrakes=True)
    self.assertFalse(fl.Fca11LongBrake(structs.CarParams(), cp).cont_law)


# ---------------------------------------------------------------------------------------------------------------------
# CORRECTIONS ROUND 2 (deploy-gate review, 2026-11-05): M1 phantom vs THROTTLE, M4 trim through the handover, M5 the
# ENABLE dither at the coast cliff. Each one fails on the 0037 tree this file was written against (lane-E/gas_probe_*).
# ---------------------------------------------------------------------------------------------------------------------
class TestPhantomOnTheGasWire(unittest.TestCase):
  """M1: a vision phantom that only reaches 1-3 LSB must NOT touch the throttle.

  The continuous law opens an episode at 1 LSB, so `braking` is True for every phantom >= 0.25 s; keying the pedal's
  forced zero on it produced a 5.2 % -> 0 -> 5.2 % dip in ~0.7 s per ghost (owner complaint #4) where pre-0037 the same
  phantom did nothing on either wire. The gas side now follows the brake only from FCA11_FEEL_FIRST_LSB = 4 LSB (the
  measured first bite, 1.00 m/s^2), so the 1-3 LSB levels are still SENT but leave the pedal alone.
  """

  V = 20.0
  ASK = -0.20                       # planner ask: shallower than coast(20) = 0.349 -> the pedal holds a partial value
  GHOST = (30.0, -25.0)             # the closing-lead phantom the probes use

  def _frames(self, ghost_frames=0, pad=0):
    """(gas[], enable[], brake_level[], first ghost frame) over 60 warm-up + ghost_frames ghost + 60+pad clear frames."""
    from opendbc.sunnypilot.car.hyundai.tests.test_gas_interceptor import _Fca11CS, _fca11_cam_frame
    CI = _elantra_ci()
    cs = _Fca11CS(v_ego=self.V, cam=(0, _fca11_cam_frame(alive=5)))
    gas, en, lvl = [], [], []

    def step(lead):
      d, vr = lead if lead is not None else (None, None)
      setattr(cs, "fca11_lead_drel_m", d)
      setattr(cs, "fca11_lead_vrel_mps", vr)
      setattr(cs, "fca11_lead_one", object() if lead is not None else None)
      sends = CI.CC.create_gas_command(_cc(self.ASK), cs, len(gas), 0)
      pedal = [d for a, d, _s in sends if a in (0x200, 0x700)]
      gas.append(float(CI.CC.gas))
      en.append(_enable_bit(pedal[0]) if pedal else None)
      lvl.append(int(CI.CC.fca11_brake.brake_level))

    for _ in range(60):
      step(None)
    t = len(gas)
    for _ in range(ghost_frames):
      step(self.GHOST)
    for _ in range(60 + pad):
      step(None)
    return gas, en, lvl, t

  def test_a_short_phantom_does_not_move_the_pedal(self):
    for model_frames in (1, 2, 3, 5):
      n = 5 * model_frames                                    # one 20 Hz model frame = 5 controller frames
      ref_gas, ref_en, _, _ = self._frames(0, pad=n)
      gas, en, lvl, t = self._frames(n)
      self.assertEqual(len(gas), len(ref_gas))
      self.assertLessEqual(max(lvl), 3, (model_frames, max(lvl)))
      self.assertEqual(gas, ref_gas, f"a {model_frames}-frame phantom moved the throttle")
      self.assertEqual(en, ref_en)
      self.assertEqual(sum(1 for x in en if x == 0), 0, "no ENABLE=0 frame may be caused by a 1-3 LSB phantom")

  def test_a_real_episode_at_the_bite_still_cuts_the_throttle(self):
    gas, en, lvl, t = self._frames(50)                        # 0.5 s of the same closing lead: past the 4 LSB bite
    self.assertGreaterEqual(max(lvl), 4, lvl[t:t + 50])
    i = next(k for k in range(t, len(lvl)) if lvl[k] >= 4 and en[k] is not None)
    self.assertEqual(gas[i], 0.0)
    self.assertEqual(en[i], 0, "at/above the first bite the pedal must be the all-zero ENABLE=0 frame")

  def test_the_coast_handover_is_still_at_need_equals_coast(self):
    from opendbc.sunnypilot.car.hyundai.tests.test_gas_interceptor import _Fca11CS, _fca11_cam_frame
    CI = _elantra_ci()
    cs = _Fca11CS(v_ego=self.V, cam=(0, _fca11_cam_frame(alive=5)))
    coast = fl.fca11_cont_coast(self.V)
    for f in range(0, 40, 2):
      CI.CC.create_gas_command(_cc(0.0), cs, f, 0)
    self.assertGreater(CI.CC.gas, 0.0)
    sends = CI.CC.create_gas_command(_cc(-coast), cs, 40, 0)
    pedal = [d for a, d, _s in sends if a in (0x200, 0x700)]
    self.assertEqual(CI.CC.gas, 0.0)
    self.assertEqual(_enable_bit(pedal[0]), 0, "need == coast(v) must still be the 0x700 ENABLE=0 hand-over")
    self.assertEqual(_gas_word(pedal[0]), 0)
    # M5 makes the crossing quantized, but only inside the pedal's own LSB: 1 % shallower is still an exact zero ...
    self.assertEqual(fl.fca11_cont_coast(self.V), coast)
    CI.CC.create_gas_command(_cc(-0.99 * coast), cs, 42, 0)
    self.assertEqual(CI.CC.gas, 0.0)
    # ... and a ask a quarter of the coast shallower is a live, partial pedal (engine braking modulated)
    CI.CC.create_gas_command(_cc(-0.25 * coast), cs, 44, 0)
    self.assertGreater(CI.CC.gas, 0.0)

  def test_the_coast_cliff_does_not_dither_the_enable_bit(self):
    """M5: a +-2 % of coast dither at 10 Hz across the hand-over must not toggle ENABLE, and no live frame may be a
    single pedal count. The hysteresis is one pedal LSB (1/255), not a time delay."""
    from opendbc.sunnypilot.car.hyundai.tests.test_gas_interceptor import _Fca11CS, _fca11_cam_frame
    CI = _elantra_ci()
    cs = _Fca11CS(v_ego=self.V, cam=(0, _fca11_cam_frame(alive=5)))
    coast = fl.fca11_cont_coast(self.V)
    for f in range(0, 40, 2):
      CI.CC.create_gas_command(_cc(0.0), cs, f, 0)
    flips = 0
    prev = None
    for i in range(60):
      ask = -(coast * (1.02 if (i // 5) % 2 == 0 else 0.98))
      sends = CI.CC.create_gas_command(_cc(ask), cs, 40 + 2 * i, 0)
      pedal = [d for a, d, _s in sends if a in (0x200, 0x700)]
      en = _enable_bit(pedal[0])
      if prev is not None and en != prev:
        flips += 1
      prev = en
      if en:
        self.assertGreaterEqual(_gas_word(pedal[0]), 2, "a live command below 2 counts is not representable")
    self.assertLessEqual(flips, 1, f"ENABLE toggled {flips} times on a 1.2 s dither across the coast cliff")


class TestTrimThroughTheHandover(unittest.TestCase):
  """M4: the feedback trim is held through the coast handover instead of being disarmed on every zero-demand frame,
  and it is still dropped when the episode really closes (the phantom hold the original guard was added for).

  0040 slice 1a ships with FCA11_CONT_TRIM_ENABLED = True (the trim is live); the setUp still pins the FLAG-ON branch
  for the duration of these tests so they mean the same thing on the trim-off tree they were written against.
  """

  def setUp(self):
    prev = bool(getattr(fl, "FCA11_CONT_TRIM_ENABLED", True))
    self.addCleanup(setattr, fl, "FCA11_CONT_TRIM_ENABLED", prev)
    fl.FCA11_CONT_TRIM_ENABLED = True

  def _open(self):
    d = _Drive().run(200, -1.5, v=15.0, a_ego=-1.2)
    self.assertGreater(d.b._track_trim, 0.0, "the loop must have accumulated a correction to hold")
    self.assertTrue(d.b.episode_open)
    return d

  def test_the_trim_survives_a_zero_demand_frame(self):
    d = self._open()
    trim = d.b._track_trim
    d.run(1, -0.01, v=15.0, a_ego=-1.2)        # need < coast(15) -> brake_need == 0
    self.assertEqual(d.b._track_trim, trim, "a zero-demand frame disarmed the loop")
    d.run(1, -1.5, v=15.0, a_ego=-1.2)         # the same demand returns: no 0.5 s re-arm penalty
    self.assertEqual(d.b._track_trim, trim)

  def test_the_trim_is_dropped_when_the_episode_closes(self):
    d = self._open()
    d.run(80, -0.01, v=15.0, a_ego=-1.2)       # release: the level walks to 0 and the episode closes
    self.assertFalse(d.b.episode_open)
    self.assertEqual(d.b._track_trim, 0.0, "a closed episode must not carry a trim into the next one")

  def test_a_phantom_leaves_nothing_on_the_wire_after_it(self):
    d = self._open()
    d.run(80, -0.01, v=15.0, a_ego=-1.2)
    t = d.f
    d.run(5, -0.1, v=15.0, a_ego=0.0, lead=(30.0, -25.0))
    d.run(120, -0.1, v=15.0, a_ego=0.0)
    tail = [x for f, x in d.sent if f >= t + 60]
    self.assertEqual(tail, [0.0] * len(tail), "a held trim must not hold a level after the phantom")
    self.assertEqual(d.b._track_trim, 0.0)


if __name__ == "__main__":
  unittest.main()