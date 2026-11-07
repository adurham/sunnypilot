"""0034: the FCA11 STOP-HOLD CLUTCH lane (HyundaiFca11StopHoldClutch -> CP_SP.fca11StopHoldClutch, DEFAULT OFF).

At a standstill hold the law's actuating 0x38D also carries CF_VSM_HBACmd (1|2) = 1 + FCA_StopReq (21|1) = 1, asking
the ESC for its own stop hold so the DCT releases the clutch (route 190: +26 Nm under our hold vs 0.0 Nm under the foot).

Every stream test drives the REAL controller update path (Fca11LongBrake.update with a CarState double). The panda
acceptance test runs the emitted frames through the REAL production TX hook (libsafety, safety/modes/hyundai.h with the
0033 window), not a Python re-implementation of it.
"""
import unittest

from opendbc.car import structs
from opendbc.car.hyundai.hyundaican import hyundai_checksum
from opendbc.sunnypilot.car.hyundai import fca11_long as fl
from opendbc.sunnypilot.car.interfaces import setup_interfaces
from opendbc.sunnypilot.car.hyundai.tests.test_fca11_long import _CS, _cc, _get_ci, _real_camera_frame, \
  _stop_and_hold_scenario

CAM = _real_camera_frame(alive=3)
HBA_MASK0 = 0x06     # byte 0 bits 1-2 = CF_VSM_HBACmd
STOP_MASK2 = 0x20    # byte 2 bit 5 (= DBC bit 21) = FCA_StopReq


def _cp_sp(stop_hold=None, dyn=None, fca11=True):
  cp_sp = structs.CarParamsSP()
  cp_sp.fca11Brake = fca11
  cp_sp.enableGasInterceptor = True
  if dyn is not None:
    cp_sp.fca11DynamicBrakes = dyn
  if stop_hold is not None:
    cp_sp.fca11StopHoldClutch = stop_hold
  return cp_sp


def _bits(d):
  return fl._get_bits(d, 1, 2), fl._get_bits(d, 21, 1)


def _pair(d):
  return _bits(d) == (1, 1)


def _run(cp_sp, scenario, n, wheel_max=lambda v: v * 3.6, esc=lambda ss: (ss, ss)):
  """[(frame, dat)] for every 0x38D the law sends through `scenario(f) -> (ask, vEgo, standstill)`. The fastest wheel
  is published like carstate.py does (km/h), from vEgo by default; 0036: so are TCS13 FCA_ACK / StandStill, which the ESC
  reports while it holds our brake at a stop (`esc(standstill) -> (ack, standstill)`, both = standstill by default)."""
  b = fl.Fca11LongBrake(structs.CarParams(), cp_sp)
  out = []
  for f in range(n):
    ask, v, ss = scenario(f)
    cs = _CS(v_ego=v, cam=(0, CAM), standstill=ss, a_ego=-1.5 if ask < 0 else 0.3)
    setattr(cs, "fca11_wheel_max_kph", wheel_max(v))
    cs.fca11_esc_ack, cs.fca11_esc_standstill = (bool(x) for x in esc(ss))
    for s in b.update(_cc(ask), cs, f):
      if s.address == fl.FCA11_ADDR:
        out.append((f, bytes(s.dat)))
  return b, out


def _release_at_standstill(f):
  """Stop, hold, then the ask RELEASES while the car is still at a standstill (the dynamic release ramp runs at 0 km/h)."""
  if f < 950:
    return _stop_and_hold_scenario(f)
  return 0.5, 0.0, True


class TestWiring(unittest.TestCase):

  def test_default_off_everywhere(self):
    self.assertFalse(structs.CarParamsSP().fca11StopHoldClutch)
    self.assertFalse(fl.stop_hold_enabled(structs.CarParamsSP()))
    self.assertFalse(fl.stop_hold_enabled(object()))     # an un-wired CP_SP (no field at all) is OFF
    for dyn in (None, False, True):
      self.assertFalse(fl.Fca11LongBrake(structs.CarParams(), _cp_sp(dyn=dyn)).stop_hold_clutch)

  def test_requires_fca11_braking_and_is_never_on_the_shadow_core(self):
    self.assertFalse(fl.Fca11LongBrake(structs.CarParams(), _cp_sp(stop_hold=True, fca11=False)).stop_hold_clutch)
    self.assertTrue(fl.Fca11LongBrake(structs.CarParams(), _cp_sp(stop_hold=True)).stop_hold_clutch)
    self.assertFalse(fl.Fca11LongBrake(structs.CarParams(), _cp_sp(stop_hold=True), core=True).stop_hold_clutch)

  def test_setup_interfaces_param(self):
    """The param reaches CP_SP only with HyundaiFca11Brake on and Shadow off; missing/junk = OFF."""
    def build(extra):
      CI, CP, CP_SP = _get_ci(True, None)      # fresh CP_SP: no FCA11 param applied yet
      setup_interfaces(CI, CP, CP_SP, [{"HyundaiGasInterceptor": True}] + extra)
      return CP_SP
    self.assertFalse(build([{"HyundaiFca11Brake": True}]).fca11StopHoldClutch)
    for raw in (True, "1", b"1"):
      self.assertTrue(build([{"HyundaiFca11Brake": True}, {"HyundaiFca11StopHoldClutch": raw}]).fca11StopHoldClutch, raw)
    for raw in (False, "0", None, "junk"):
      self.assertFalse(build([{"HyundaiFca11Brake": True}, {"HyundaiFca11StopHoldClutch": raw}]).fca11StopHoldClutch)
    self.assertFalse(build([{"HyundaiFca11Brake": False}, {"HyundaiFca11StopHoldClutch": True}]).fca11StopHoldClutch)
    self.assertFalse(build([{"HyundaiFca11Brake": True}, {"HyundaiFca11Shadow": True},
                            {"HyundaiFca11StopHoldClutch": True}]).fca11StopHoldClutch)

  def test_constants_match_the_panda_window(self):
    self.assertEqual(7, fl.FCA11_STOP_HOLD_MIN_LSB)
    self.assertEqual(fl.FCA11_CRAWL_MIN_LSB, fl.FCA11_STOP_HOLD_MIN_LSB)
    self.assertEqual(1, fl.FCA11_STOP_HOLD_HBA)
    from opendbc.sunnypilot.car.hyundai.values import HYUNDAI_FCA11_LONG_STOP_HOLD_MAX_SPEED_KPH as PANDA_KPH
    self.assertEqual(1.0, PANDA_KPH)                            # 0036: the panda's fastest-wheel bound
    self.assertLessEqual(fl.FCA11_STOP_HOLD_V_MAX_MPS, 0.2)
    self.assertLess(fl.FCA11_STOP_HOLD_V_MAX_MPS * 3.6, PANDA_KPH)   # margin under the panda's wheel gate
    self.assertLess(fl.FCA11_STOP_HOLD_WHEEL_MAX_KPH, PANDA_KPH)


class TestBuilder(unittest.TestCase):

  def test_default_builder_is_unchanged(self):
    m = fl.Fca11MirrorFrame(dec_cmd=20, prefill=True, warn=3, cmd_act=True, dec_cmd_act=False)
    self.assertFalse(m.stop_hold)
    for hba, stop in ((0, 0), (2, 1), (3, 0)):
      cam = _real_camera_frame(alive=3, hba=hba, stop_req=stop)
      self.assertEqual((hba, stop), _bits(fl.build_fca11_frame(cam, 4, m)))

  def test_stop_hold_sets_exactly_the_pair_at_20_lsb(self):
    base = fl.Fca11MirrorFrame(dec_cmd=20, prefill=True, warn=3, cmd_act=True, dec_cmd_act=False)
    held = fl.Fca11MirrorFrame(dec_cmd=20, prefill=True, warn=3, cmd_act=True, dec_cmd_act=False, stop_hold=True)
    for cam in (CAM, _real_camera_frame(alive=3, hba=3), _real_camera_frame(alive=3, hba=2, stop_req=1)):
      a, b = fl.build_fca11_frame(cam, 4, base), fl.build_fca11_frame(cam, 4, held)
      self.assertEqual((1, 1), _bits(b))                       # the exact pair, whatever the camera carried
      self.assertEqual(20, fl._get_bits(b, 8, 8))
      self.assertEqual(b[7], hyundai_checksum(b[:7]))
    a, b = fl.build_fca11_frame(CAM, 4, base), fl.build_fca11_frame(CAM, 4, held)
    self.assertEqual(bytes([HBA_MASK0 & 0x02, 0, STOP_MASK2, 0, 0, 0, 0]), bytes(x ^ y for x, y in zip(a[:7], b[:7])))


class TestGate(unittest.TestCase):
  """Fca11LongBrake._stop_hold_ok, term by term (the predicate the single call site uses)."""

  def setUp(self):
    self.b = fl.Fca11LongBrake(structs.CarParams(), _cp_sp(stop_hold=True))

  def _cs(self, v=0.0, standstill=True, wheel_max: float | None = 0.0, esc=(True, True)):
    cs = _CS(v_ego=v, cam=(0, CAM), standstill=standstill)
    if wheel_max is not None:
      setattr(cs, "fca11_wheel_max_kph", wheel_max)
    if esc is not None:
      cs.fca11_esc_ack, cs.fca11_esc_standstill = esc
    return cs

  @staticmethod
  def _m(dec=20, **kw):
    a = dict(dec_cmd=dec, prefill=True, warn=3, cmd_act=True, dec_cmd_act=False)
    a.update(kw)
    return fl.Fca11MirrorFrame(**a)

  def test_standstill_20_lsb_hold(self):
    self.assertTrue(self.b._stop_hold_ok(self._cs(), self._m(20), False))
    # the boundary (vEgo is float32 in CarState: 0.2 reads as 0.2000000030 and is refused - the fail-safe direction)
    self.assertTrue(self.b._stop_hold_ok(self._cs(v=0.19, wheel_max=0.75), self._m(7), False))

  def test_never_above_0_2_mps(self):
    for v in (0.21, 0.5, 1.2, 1.5, 3.0, 20.0):     # 0036: 0.21-1.2 m/s were legal under 0034
      self.assertFalse(self.b._stop_hold_ok(self._cs(v=v), self._m(), False), v)

  def test_never_without_the_esc_reporting_the_hold(self):
    """0036: TCS13 FCA_ACK (the ESC executes our brake) AND StandStill, both required; unpublished = refused."""
    for esc in ((False, False), (False, True), (True, False), None):
      self.assertFalse(self.b._stop_hold_ok(self._cs(esc=esc), self._m(), False), esc)

  def test_never_on_a_fast_or_unknown_wheel(self):
    for w in (0.76, 1.0, 4.0, 5.0, 30.0, None, float("nan")):
      self.assertFalse(self.b._stop_hold_ok(self._cs(wheel_max=w), self._m(), False), w)

  def test_never_without_confirmed_standstill(self):
    self.assertFalse(self.b._stop_hold_ok(self._cs(standstill=False), self._m(), False))

  def test_never_below_level_7(self):
    for dec in range(0, 7):
      self.assertFalse(self.b._stop_hold_ok(self._cs(), self._m(dec), False), dec)

  def test_never_with_dec_cmd_act_or_a_bad_shape(self):
    for kw in ({"dec_cmd_act": True}, {"warn": 0}, {"warn": 2}, {"cmd_act": False}, {"prefill": False},
               {"dec_cmd": 0, "prefill": False, "warn": 0, "cmd_act": False}):
      self.assertFalse(self.b._stop_hold_ok(self._cs(), self._m(**kw), False), kw)

  def test_never_under_cal_or_toggle_off(self):
    self.assertFalse(self.b._stop_hold_ok(self._cs(), self._m(), True))
    off = fl.Fca11LongBrake(structs.CarParams(), _cp_sp(stop_hold=False))
    self.assertFalse(off._stop_hold_ok(self._cs(), self._m(), False))


class TestStream(unittest.TestCase):
  """The real controller over a full stop-and-hold: OFF is byte-identical, ON differs ONLY in the pair."""

  def _streams(self, dyn, scenario=_stop_and_hold_scenario, n=1100, **kw):
    off = _run(_cp_sp(stop_hold=False, dyn=dyn), scenario, n, **kw)[1]
    absent = _run(_cp_sp(dyn=dyn), scenario, n, **kw)[1]
    b_on, on = _run(_cp_sp(stop_hold=True, dyn=dyn), scenario, n, **kw)
    return off, absent, on, b_on

  def test_toggle_off_is_byte_identical(self):
    """OFF (explicit False) == field absent (today's CP_SP) == the stream the 0024 tests were written against."""
    from opendbc.sunnypilot.car.hyundai.tests.test_fca11_long import stop_and_hold_frames
    for dyn in (False, True):
      with self.subTest(dyn=dyn):
        off, absent, _, _ = self._streams(dyn)
        self.assertEqual(absent, off)
        self.assertEqual(stop_and_hold_frames(dyn, CAM), off)
        self.assertFalse(any(_bits(d) != (0, 0) for _, d in off))

  def test_toggle_on_differs_only_in_the_pair(self):
    for dyn in (False, True):
      with self.subTest(dyn=dyn):
        off, _, on, b_on = self._streams(dyn)
        self.assertEqual([f for f, _ in off], [f for f, _ in on], "same frames, same slots")
        n_pair = 0
        for (f, a), (_, b) in zip(off, on):
          x = bytes(p ^ q for p, q in zip(a[:7], b[:7]))
          self.assertEqual(0, x[0] & ~HBA_MASK0 & 0xFF, f)
          self.assertEqual(b"\x00", x[1:2], f"DecCmd changed @{f}")
          self.assertEqual(0, x[2] & ~STOP_MASK2 & 0xFF, f)
          self.assertEqual(b"\x00" * 4, x[3:7], f)
          self.assertEqual(b[7], hyundai_checksum(b[:7]))
          if a != b:
            self.assertTrue(_pair(b), f"never one bit alone @{f}")
            self.assertEqual((0, 0), _bits(a))
            n_pair += 1
        self.assertGreater(n_pair, 200, "the hold at the light must actually carry the pair")
        self.assertEqual(n_pair, b_on.stop_hold_frames)

  def test_on_bits_only_on_standstill_hold_frames(self):
    for dyn in (False, True):
      with self.subTest(dyn=dyn):
        _, _, on, _ = self._streams(dyn)
        for f, d in on:
          ask, v, ss = _stop_and_hold_scenario(f)
          lvl = fl._get_bits(d, 8, 8)
          if _pair(d):
            self.assertTrue(ss and v <= 1.2 and lvl >= 7, f"pair @{f}: v={v} ss={ss} lvl={lvl}")
            self.assertEqual((1, 1, 3, 0), (fl._get_bits(d, 0, 1), fl._get_bits(d, 20, 1), fl._get_bits(d, 3, 2),
                                            fl._get_bits(d, 31, 1)))
          else:
            self.assertEqual((0, 0), _bits(d), f)
        # every moving frame and every frame of the launch is clean, and the launch closes with a clean passive frame
        self.assertFalse(any(_pair(d) for f, d in on if _stop_and_hold_scenario(f)[1] > 0.0))
        self.assertFalse(any(_pair(d) for f, d in on if f >= 950))
        self.assertTrue(any(fl._get_bits(d, 8, 8) == 0 for f, d in on if f >= 950))

  def test_never_above_0_2_mps_even_if_standstill_says_so(self):
    """A CarState that (wrongly) claims standstill at 1.5 m/s with slow wheels still never gets the pair."""
    def sc(f):
      a, v, ss = _stop_and_hold_scenario(f)
      return (a, 1.5, True) if 350 <= f < 950 else (a, v, ss)
    _, _, on, b = self._streams(False, scenario=sc, wheel_max=lambda v: 0.0)
    self.assertEqual(0, b.stop_hold_frames)
    self.assertFalse(any(_pair(d) for _, d in on))

  def test_never_on_a_fast_wheel(self):
    _, _, on, b = self._streams(False, wheel_max=lambda v: 0.9)
    self.assertEqual(0, b.stop_hold_frames)
    _, _, on, b = self._streams(False, wheel_max=lambda v: None)
    self.assertEqual(0, b.stop_hold_frames)

  def test_never_without_the_esc_ack_in_the_stream(self):
    """0036: the whole stop-and-hold with the ESC NOT acknowledging (or not at its own standstill) never carries the pair."""
    for esc in (lambda ss: (False, ss), lambda ss: (ss, False), lambda ss: (False, False)):
      _, _, on, b = self._streams(False, esc=esc)
      self.assertEqual(0, b.stop_hold_frames)
      self.assertFalse(any(_pair(d) for _, d in on))

  def test_pair_drops_once_and_stays_off_for_that_stop(self):
    """0036: the ESC ack falls mid-hold -> the pair drops on that frame and does NOT come back at the same stop, even
    when the ack returns (no on/off flapping at the light). A new stop (after moving) may carry it again."""
    def esc_flap(f):
      return (False, True) if 600 <= f < 650 else (True, True)
    b = fl.Fca11LongBrake(structs.CarParams(), _cp_sp(stop_hold=True))
    pair_frames = []
    for f in range(1100):
      ask, v, ss = _stop_and_hold_scenario(f)
      cs = _CS(v_ego=v, cam=(0, CAM), standstill=ss, a_ego=-1.5 if ask < 0 else 0.3)
      cs.fca11_wheel_max_kph = v * 3.6
      cs.fca11_esc_ack, cs.fca11_esc_standstill = esc_flap(f) if ss else (False, False)
      for snd in b.update(_cc(ask), cs, f):
        if snd.address == fl.FCA11_ADDR and _pair(bytes(snd.dat)):
          pair_frames.append(f)
    self.assertTrue(pair_frames and min(pair_frames) < 600)
    self.assertFalse(any(f >= 600 for f in pair_frames), "came back at the same stop after the ack fell")

  def test_never_on_the_release_ramp_or_passive_frame(self):
    """Dynamic Brakes ON, ask released while STILL at a standstill: the ramp-down and the close frame carry no pair -
    the pair drops on the first frame the law stops holding."""
    off, _, on, _ = self._streams(True, scenario=_release_at_standstill, n=1100)
    self.assertEqual([f for f, _ in off], [f for f, _ in on])
    after = [(f, d) for f, d in on if f >= 950]
    lv = [fl._get_bits(d, 8, 8) for _, d in after]
    self.assertGreater(len(after), 3)
    self.assertTrue(any(0 < x < lv[0] for x in lv), "a release ramp actually ran at standstill")
    self.assertIn(0, lv, "and closed with a passive frame")
    for f, d in after:
      self.assertEqual((0, 0), _bits(d), f"frame {f} lvl {fl._get_bits(d, 8, 8)}")
    self.assertTrue(any(_pair(d) for f, d in on if f < 950))
    # the passive frame in every stream is clean
    for f, d in on:
      if fl._get_bits(d, 8, 8) == 0:
        self.assertEqual((0, 0), _bits(d))

  def test_never_below_level_7(self):
    """Frames below 7 LSB at a standstill (the onset ramp 4 -> 8 at a stop) never carry the pair."""
    def sc(f):
      return (-2.0, 0.0, True)
    _, _, on, _ = self._streams(False, scenario=sc, n=200)
    for f, d in on:
      if fl._get_bits(d, 8, 8) < 7:
        self.assertEqual((0, 0), _bits(d), f)
    self.assertTrue(any(fl._get_bits(d, 8, 8) < 7 for _, d in on), "the onset frames below 7 exist")
    self.assertTrue(any(_pair(d) for _, d in on))


class TestPandaAcceptance(unittest.TestCase):
  """The ON stream through the REAL production TX hook (libsafety, FCA11_LONG on the pedal car, window open, engaged).
  The C hook IS callable from Python through the existing libsafety cffi harness (opendbc/safety/tests)."""

  def _fixture(self):
    import opendbc.safety.tests.test_hyundai as th
    # the same parameterized fixture TestNoStopHoldBitsPandaTx (test_fca11_long.py) uses
    tc = getattr(th, "TestHyundaiFca11LongSafety_0")("test_needs_controls_and_heartbeat")
    tc.setUp()
    return tc

  def _verdicts(self, tc, sent, speed_raw, esc=lambda f: (1, 1) if f >= 350 else (0, 0)):
    from opendbc.safety.tests.libsafety import libsafety_py
    tc._rearm()
    out = []
    for f, d in sent:
      tc.safety.set_timer(1_000_000 + f * 10_000)
      tc.esc_hold = esc(f)      # 0036: TCS13 FCA_ACK / StandStill, as the ESC reports them while it holds our brake
      tc._open_window(speed_raw=speed_raw(f))
      tc._rx(tc._cam_fca11())
      out.append(tc._tx(libsafety_py.make_CANPacket(fl.FCA11_ADDR, 0, d)))
    return out

  def test_on_stream_accepted_like_the_off_stream(self):
    tc = self._fixture()
    spd = lambda f: 0 if f >= 350 else 640     # noqa: E731 - WHL_SPD11 raw (1/32 km/h): 0 at the stop, 20 km/h before
    for dyn in (False, True):
      with self.subTest(dyn=dyn):
        off = _run(_cp_sp(stop_hold=False, dyn=dyn), _stop_and_hold_scenario, 1100)[1]
        on = _run(_cp_sp(stop_hold=True, dyn=dyn), _stop_and_hold_scenario, 1100)[1]
        v_off, v_on = self._verdicts(tc, off, spd), self._verdicts(tc, on, spd)
        self.assertEqual(v_off, v_on, "the pair never costs the law a frame")
        pair = [i for i, (_, d) in enumerate(on) if _pair(d)]
        self.assertGreater(len(pair), 200)
        self.assertTrue(all(v_on[i] for i in pair), "every frame carrying the pair is ACCEPTED by the panda")

  def test_contrast_same_frames_refused_above_walking_pace(self):
    """Falsifiable: the panda refuses the very same pair frames once the fastest wheel is above 1 km/h (0036; 5 km/h
    under 0033), and whenever the ESC does not report the hold. The WHOLE stream is replayed (the ramp to the hold level
    must pass the panda's rise gate first); only the verdicts of the frames carrying the pair are judged."""
    tc = self._fixture()
    on = _run(_cp_sp(stop_hold=True), _stop_and_hold_scenario, 1100)[1]
    pair = [i for i, (_, d) in enumerate(on) if _pair(d)]
    self.assertGreater(len(pair), 200)
    stop = lambda f: f >= 350    # noqa: E731 - the scenario's standstill
    ok = self._verdicts(tc, on, lambda f: 0 if stop(f) else 640)
    self.assertTrue(all(ok[i] for i in pair), "control: accepted at 0 km/h with the ESC reporting the hold")
    for raw in (33, 5 * 32 + 1):
      v = self._verdicts(tc, on, lambda f, r=raw: r if stop(f) else 640)
      self.assertFalse(any(v[i] for i in pair), f"raw {raw}: a pair frame was accepted above 1 km/h")
    for esc in ((0, 1), (1, 0), (0, 0)):
      v = self._verdicts(tc, on, lambda f: 0 if stop(f) else 640, esc=lambda f, e=esc: e if stop(f) else (0, 0))
      self.assertFalse(any(v[i] for i in pair), f"esc {esc}: a pair frame was accepted without the ESC hold")


class TestCarStatePublishes(unittest.TestCase):
  """0036: the lane's inputs through the REAL CarInterface.update -> carstate.py path. FAIL-BEFORE: nothing published
  fca11_wheel_max_kph (or the ESC flags), so _stop_hold_ok's fail-closed read refused every frame on the car and the
  0034 toggle could never put a single pair frame on the wire."""

  def test_fastest_wheel_and_esc_flags_reach_the_carstate(self):
    from opendbc.can import CANPacker
    from opendbc.sunnypilot.car.hyundai.tests.test_fca11_clutch import _ci
    pk = CANPacker("hyundai_can_generated")
    CI, _ = _ci()
    whl = pk.make_can_msg("WHL_SPD11", 0, {"WHL_SPD_FL": 0.0, "WHL_SPD_FR": 0.5, "WHL_SPD_RL": 0.25, "WHL_SPD_RR": 0.0})
    for ack, ss in ((1, 1), (0, 1), (1, 0)):
      tcs = pk.make_can_msg("TCS13", 0, {"FCA_ACK": ack, "StandStill": ss})
      for i in range(3):
        CI.update([((i + 1) * int(0.02e9), [(whl[0], whl[1], 0), (tcs[0], tcs[1], 0)])])
      self.assertAlmostEqual(0.5, CI.CS.fca11_wheel_max_kph, places=3)
      self.assertEqual((bool(ack), bool(ss)), (CI.CS.fca11_esc_ack, CI.CS.fca11_esc_standstill))


if __name__ == "__main__":
  unittest.main()
