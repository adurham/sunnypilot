"""0056: the FCA11 STANDSTILL CLAMP (param HyundaiFca11StandstillClamp -> CP_SP.fca11StandstillClamp, DEFAULT OFF).

At a stop the continuous law's standstill hold reaches its 30 steps at the panda's +4 per sent frame instead of the
2 m/s^3 slew. Every assertion below is stated in INPUT terms (what the car and the planner did), not in the clamp's own
state, and the property test is run against two deliberately broken variants that must FAIL it (positive controls).

  (a) wiring: setup_interfaces writes the field only with FCA11 braking on and Shadow off; the car layer reads it only on
      the continuous-law instance; a CP_SP without the field is OFF
  (b) OFF (field absent / False) is byte-identical to the law without the clamp code path
  (c) ON: identical before the stop, +4 per sent frame to 30 after it, never above 30, never above the last sent + 4
  (d) the "never" table: gas, brake, gear, planner asking to move, not armed, standstill flag without the speed, the
      speed without the standstill flag, a non-finite speed
  (e) the planner asking to move hands the wire back on the SAME frame: the launch is byte-identical from there on
  (f) a slip (standstill flag dropping) while engaged does not lower the floor; the law catching up retires it without
      a step; the clamp is once per stop and re-arms only after the car has moved
  (g) fail-safe: an exception inside the clamp latches it off and returns the law's own level
  (h) through the REAL gas path: the pedal frames are identical ON vs OFF (exactly 0 while held)
  (i) randomised two-mode lockstep property test + two broken variants that must fail it

Run from the opendbc root:  PYTHONPATH=. python -m pytest -q opendbc/sunnypilot/car/hyundai/tests/test_fca11_standstill_clamp.py
"""
import inspect
import math
import random
import unittest

from opendbc.car import structs
import opendbc.sunnypilot.car.hyundai.fca11_long as fl
from opendbc.sunnypilot.car.hyundai.tests.test_fca11_long import _CS, _cam

STD = fl.PERSONALITY_STANDARD
D = structs.CarState.GearShifter.drive
P = structs.CarState.GearShifter.park


def _cp_sp(clamp=None, aff=True, dyn=True):
  kw = dict(fca11Brake=True, enableGasInterceptor=True, fca11AffineGain=aff, fca11DynamicBrakes=dyn)
  if clamp is not None:
    kw["fca11StandstillClamp"] = clamp
  return structs.CarParamsSP(**kw)


def _brake(clamp=None, aff=True, dyn=True, cls=fl.Fca11LongBrake):
  return cls(structs.CarParams(), _cp_sp(clamp, aff, dyn))


def _cc(accel, long_active=True):
  cc = structs.CarControl()
  cc.enabled = cc.latActive = True
  cc.longActive = long_active
  cc.actuators.accel = accel
  return cc


class _Drive:
  """Feeds one input frame at a time; records the SENT 0x38D level per controller frame (None on a non-send frame)."""

  def __init__(self, clamp=None, cls=fl.Fca11LongBrake):
    self.b = _brake(clamp, cls=cls)
    self.f = 0
    self.wire = []          # (frame, level) for every SENT 0x38D

  def step(self, accel, v, a_ego=0.0, standstill=False, gas=False, brake=False, gear=D, long_active=True,
           cam_ok=True):
    cs = _CS(v_ego=v, cam=_cam() if cam_ok else None, a_ego=a_ego, gear=gear, brake=brake, gas=gas)
    cs.out.standstill = standstill
    cs.fca11_lead_drel_m = cs.fca11_lead_vrel_mps = cs.fca11_lead_one = None
    out = None
    for s in self.b.update(_cc(accel, long_active), cs, self.f, STD):
      out = fl._get_bits(s.dat, 8, 8)
      self.wire.append((self.f, out))
    self.f += 1
    return out

  def run(self, n, accel, v, **kw):
    for _ in range(n):
      self.step(accel, v, **kw)
    return self

  def approach(self):
    """2 s braking at 3 m/s (arms the clamp), then down the creep floor to v = 0.09 m/s (flag still False)."""
    self.run(100, -1.0, 3.0, a_ego=-0.9)
    for v in (2.0, 1.5, 1.0, 0.6, 0.3, 0.15, 0.09):
      self.run(10, -0.6, v, a_ego=-0.8)
    return self

  def levels(self, since=0, until=10 ** 9):
    return [lv for f, lv in self.wire if since <= f < until]


def _pair(script):
  """Run the same input script through an OFF and an ON instance; return both drives."""
  off, on = _Drive(clamp=False), _Drive(clamp=True)
  for d in (off, on):
    script(d)
  return off, on


# ---------------------------------------------------------------------------------------------------------------------
class TestWiring(unittest.TestCase):
  """(a) who may switch it on."""

  def test_default_is_off_and_needs_the_continuous_law(self):
    self.assertFalse(_brake().ss_clamp)                       # field absent -> OFF
    self.assertFalse(_brake(clamp=False).ss_clamp)
    self.assertTrue(_brake(clamp=True).ss_clamp)
    self.assertFalse(_brake(clamp=True, aff=False).ss_clamp)  # no shaping -> feel/old law -> inert
    self.assertFalse(_brake(clamp=True, dyn=False).ss_clamp)  # no DynamicBrakes -> no continuous law -> inert
    cp = structs.CarParamsSP(fca11Brake=False, enableGasInterceptor=True, fca11AffineGain=True, fca11DynamicBrakes=True,
                             fca11StandstillClamp=True)
    self.assertFalse(fl.Fca11LongBrake(structs.CarParams(), cp).ss_clamp)   # FCA11 braking off -> inert

  def test_the_shadow_core_never_carries_it(self):
    self.assertFalse(fl.Fca11LongBrake(structs.CarParams(), fl._shadow_core_params(), core=True).ss_clamp)

  def test_setup_interfaces_writer(self):
    from opendbc.car.car_helpers import interfaces
    from opendbc.car.hyundai.values import CAR
    from opendbc.sunnypilot.car.interfaces import setup_interfaces
    from opendbc.sunnypilot.car.hyundai.tests.test_gas_interceptor import BASE_FINGERPRINT, STD as PEDAL
    car = CAR.HYUNDAI_ELANTRA_2022_NON_SCC
    fp = {i: {} for i in range(8)}
    fp[0] = {**BASE_FINGERPRINT, **PEDAL}
    CarInterface = interfaces[car]

    def written(extra):
      CP = CarInterface.get_params(car, fp, [], alpha_long=False, is_release=False, docs=False)
      CP_SP = CarInterface.get_params_sp(CP, car, fp, [], alpha_long=False, is_release_sp=False, docs=False)
      setup_interfaces(CarInterface, CP, CP_SP, [{"HyundaiGasInterceptor": True}, {"HyundaiFca11AffineGain": True},
                                                 {"HyundaiFca11DynamicBrakes": True}] + extra)
      return CP_SP.fca11StandstillClamp

    self.assertFalse(written([{"HyundaiFca11Brake": True}]))                                         # key absent
    self.assertTrue(written([{"HyundaiFca11Brake": True}, {"HyundaiFca11StandstillClamp": True}]))
    self.assertTrue(written([{"HyundaiFca11Brake": True}, {"HyundaiFca11StandstillClamp": "1"}]))
    self.assertFalse(written([{"HyundaiFca11Brake": True}, {"HyundaiFca11StandstillClamp": False}]))
    self.assertFalse(written([{"HyundaiFca11Brake": True}, {"HyundaiFca11StandstillClamp": None}]))   # stale libparams
    self.assertFalse(written([{"HyundaiFca11Brake": True}, {"HyundaiFca11StandstillClamp": "junk"}]))
    self.assertFalse(written([{"HyundaiFca11Brake": False}, {"HyundaiFca11StandstillClamp": True}]))
    self.assertFalse(written([{"HyundaiFca11Brake": True}, {"HyundaiFca11Shadow": True},
                              {"HyundaiFca11StandstillClamp": True}]))


# ---------------------------------------------------------------------------------------------------------------------
def _stop_hold_launch(d):
  d.approach()
  d.run(300, -2.0, 0.0, standstill=True)                  # 3 s held
  d.run(150, 0.4, 0.0, standstill=True)                   # the planner asks to move
  d.run(100, 0.6, 1.0, a_ego=0.8)                         # moving off


class TestOffIdentity(unittest.TestCase):
  """(b) the OFF instance (field absent) and an explicit False are the same law, frame for frame."""

  def test_absent_and_false_are_identical(self):
    a, b = _Drive(clamp=None), _Drive(clamp=False)
    for d in (a, b):
      _stop_hold_launch(d)
    self.assertEqual(a.wire, b.wire)
    self.assertEqual(b.b.ss_clamp_engagements, 0)


class TestOnArrival(unittest.TestCase):
  """(c) identical before the stop, then +4 per sent frame to 30."""

  def test_identical_before_the_stop_and_fast_after_it(self):
    off, on = _pair(_stop_hold_launch)
    t_stop = 100 + 70                                     # first frame with v <= 0.08 and the standstill flag
    self.assertEqual(off.levels(until=t_stop), on.levels(until=t_stop))
    seq_on = on.levels(since=t_stop, until=t_stop + 40)
    seq_off = off.levels(since=t_stop, until=t_stop + 80)
    self.assertEqual(seq_on[:7], [11, 15, 19, 23, 27, 30, 30], seq_on)     # 7 -> 30 in 6 sent frames (0.12 s)
    self.assertGreater(seq_off.index(30), 30, seq_off)                     # today's law: ~0.7 s (> 30 sent frames)
    self.assertEqual(on.b.ss_clamp_engagements, 1)

  def test_bounds_on_every_sent_frame(self):
    off, on = _pair(_stop_hold_launch)
    o = dict(off.wire)
    prev = 0
    for f, lv in on.wire:
      self.assertLessEqual(lv, 30)
      self.assertGreaterEqual(lv, o[f], f)                 # never below today's law
      self.assertLessEqual(lv, prev + 4, f)                # never faster than the panda's onset step
      prev = lv


# ---------------------------------------------------------------------------------------------------------------------
class TestNever(unittest.TestCase):
  """(d) the clamp must not act in any of these; each case is the whole stop, frame for frame, against OFF."""

  def _case(self, **hold_kw):
    def script(d):
      d.approach()
      d.run(150, hold_kw.get("accel", -2.0),
            hold_kw.get("v", 0.0), standstill=hold_kw.get("standstill", True), gas=hold_kw.get("gas", False),
            brake=hold_kw.get("brake", False), gear=hold_kw.get("gear", D))
    off, on = _pair(script)
    self.assertEqual(off.wire, on.wire)
    self.assertEqual(on.b.ss_clamp_engagements, 0)

  def test_gas(self):
    self._case(gas=True)

  def test_brake(self):
    self._case(brake=True)

  def test_gear(self):
    self._case(gear=P)

  def test_planner_asking_to_move(self):
    self._case(accel=0.3)

  def test_standstill_flag_but_still_rolling_at_0p09(self):
    self._case(v=0.09)

  def test_slow_but_no_standstill_flag(self):
    self._case(v=0.05, standstill=False)

  def test_not_armed_without_movement(self):
    # engaged at a standstill from the start: the car never moved under openpilot, so there is no stop to serve
    off, on = _pair(lambda d: d.run(300, -2.0, 0.0, standstill=True))
    self.assertEqual(off.wire, on.wire)

  def test_non_finite_speed(self):
    def script(d):
      d.approach()
      d.run(100, -2.0, float("nan"), standstill=True)
    off, on = _pair(script)
    self.assertEqual(off.wire, on.wire)


# ---------------------------------------------------------------------------------------------------------------------
class TestHandBack(unittest.TestCase):
  """(e) + (f)"""

  def test_planner_go_inside_the_floor_is_byte_identical_from_that_frame(self):
    def script(d):
      d.approach()
      d.run(12, -2.0, 0.0, standstill=True)               # floor engaged and at 30, the law still ~12
      d.run(150, 0.4, 0.0, standstill=True)                # launch request
      d.run(100, -2.0, 0.0, standstill=True)               # and a re-stop request while still stopped
    off, on = _pair(script)
    t_go = 170 + 12
    on_l, off_l = on.levels(until=t_go), off.levels(until=t_go)
    self.assertGreater(max(a - b for a, b in zip(on_l, off_l)), 10)        # the floor was well above the law
    self.assertEqual(on.levels(since=t_go), off.levels(since=t_go))        # identical from the go frame on

  def test_a_slip_does_not_lower_the_floor_and_catch_up_has_no_step(self):
    def script(d):
      d.approach()
      d.run(14, -0.85, 0.0, standstill=True)               # 7 sent frames: the floor reaches 30 (the law ~10)
      d.run(40, -0.85, 0.15, a_ego=0.3, standstill=False)  # 0.4 s slip: the flag drops
      d.run(200, -0.85, 0.0, standstill=True)
    off, on = _pair(script)
    t_slip = 170 + 14
    slip_on = on.levels(since=t_slip, until=t_slip + 40)
    self.assertEqual(min(slip_on), 30, slip_on)            # held through the roll
    self.assertLess(min(off.levels(since=t_slip, until=t_slip + 40)), 20)  # today's law sits on the demand
    seq = on.levels(since=170)
    for a, b in zip(seq, seq[1:]):
      self.assertGreaterEqual(b, a - 3, seq)               # no step down anywhere, incl. the catch-up retirement
    self.assertEqual(on.levels()[-50:], off.levels()[-50:])

  def test_once_per_stop_and_rearm_after_moving(self):
    def script(d):
      d.approach()
      d.run(150, -2.0, 0.0, standstill=True)
      d.run(30, -0.6, 0.12, standstill=False)              # a slip after catch-up: no second clamp
      d.run(100, -2.0, 0.0, standstill=True)
      d.approach()                                         # drove off and stopped again
      d.run(100, -2.0, 0.0, standstill=True)
    _off, on = _pair(script)
    self.assertEqual(on.b.ss_clamp_engagements, 2)


class TestFailSafe(unittest.TestCase):
  """(g)"""

  @staticmethod
  def _faulty(after_engaged_sends):
    class Faulty(fl.Fca11LongBrake):
      n_eng = 0

      def _ss_clamp_step_checked(self, *a, **k):
        if self._ss_engaged:
          Faulty.n_eng += 1
          if Faulty.n_eng > after_engaged_sends:
            raise RuntimeError("injected")
        return super()._ss_clamp_step_checked(*a, **k)
    return Faulty

  def test_exception_while_engaged_returns_the_law_on_that_frame_and_latches(self):
    Faulty = self._faulty(2)
    off, on = _Drive(clamp=False), _Drive(clamp=True, cls=Faulty)
    for d in (off, on):
      _stop_hold_launch(d)
    self.assertEqual(on.b.ss_clamp_faults, 1)
    self.assertTrue(on.b._ss_fault)
    self.assertEqual(on.b.ss_clamp_engagements, 1)
    o = dict(off.wire)
    diff = [f for f, lv in on.wire if lv != o[f]]
    self.assertEqual(len(diff), 3, diff)                   # engage frame + 2 sends, then the fault: the law's level
    t_fault = diff[-1] + 2
    self.assertEqual([lv for f, lv in on.wire if f >= t_fault], [lv for f, lv in off.wire if f >= t_fault])

  def test_exception_in_the_watch_latches_off(self):
    class FaultyWatch(fl.Fca11LongBrake):
      def _ss_watch(self, CS, accel):
        if self._ss_engaged:
          CS = None                                        # any attribute access now raises inside the guarded body
        return super()._ss_watch(CS, accel)
    off, on = _Drive(clamp=False), _Drive(clamp=True, cls=FaultyWatch)
    for d in (off, on):
      _stop_hold_launch(d)
    self.assertTrue(on.b._ss_fault)
    o = dict(off.wire)
    diff = [f for f, lv in on.wire if lv != o[f]]
    self.assertEqual(len(diff), 1, diff)                   # only the engagement frame differs, then today's law


# ---------------------------------------------------------------------------------------------------------------------
class TestRealGasPath(unittest.TestCase):
  """(h) the pedal: through GasInterceptorCarController.create_gas_command on the car's own CarInterface."""

  def _ci(self, clamp):
    from opendbc.car.car_helpers import interfaces
    from opendbc.car.hyundai.values import CAR
    from opendbc.sunnypilot.car.interfaces import setup_interfaces
    from opendbc.sunnypilot.car.hyundai.tests.test_gas_interceptor import BASE_FINGERPRINT, STD as PEDAL
    car = CAR.HYUNDAI_ELANTRA_2022_NON_SCC
    fp = {i: {} for i in range(8)}
    fp[0] = {**BASE_FINGERPRINT, **PEDAL}
    CarInterface = interfaces[car]
    CP = CarInterface.get_params(car, fp, [], alpha_long=False, is_release=False, docs=False)
    CP_SP = CarInterface.get_params_sp(CP, car, fp, [], alpha_long=False, is_release_sp=False, docs=False)
    setup_interfaces(CarInterface, CP, CP_SP, [{"HyundaiGasInterceptor": True}, {"HyundaiFca11Brake": True},
                                               {"HyundaiFca11AffineGain": True}, {"HyundaiFca11DynamicBrakes": True},
                                               {"HyundaiFca11StandstillClamp": clamp}])
    return CarInterface(CP, CP_SP)

  def test_pedal_frames_identical_and_zero_while_held(self):
    from opendbc.sunnypilot.car.hyundai.tests.test_gas_interceptor import _Fca11CS, _fca11_cam_frame
    script = [(150, 0.3, 4.0, False), (100, -1.0, 3.0, False)]
    script += [(10, -0.6, v, False) for v in (2.0, 1.5, 1.0, 0.6, 0.3, 0.15, 0.09)]
    script += [(300, -2.0, 0.0, True), (200, 0.5, 0.0, True), (100, 0.6, 1.0, False)]
    out = {}
    for clamp in (False, True):
      CI = self._ci(clamp)
      self.assertEqual(CI.CC.fca11_brake.ss_clamp, clamp)
      pedal, lv, f = [], [], 0
      for n, accel, v, ss in script:
        for _ in range(n):
          cs = _Fca11CS(v_ego=v, cam=(0, _fca11_cam_frame(alive=f % 16)))
          cs.out.standstill = ss
          cs.out.aEgo = 0.0
          sends = CI.CC.create_gas_command(_cc(accel), cs, f, STD)
          pedal += [(f, bytes(d)) for a, d, _s in sends if a in (0x200, 0x700)]
          lv += [(f, d[1]) for a, d, _s in sends if a == 0x38D]
          f += 1
      out[clamp] = (pedal, lv)
    self.assertEqual(out[False][0], out[True][0])         # every pedal frame byte-identical
    self.assertNotEqual(out[False][1], out[True][1])      # while the brake stream did change (the clamp ran)
    held = [d for f, d in out[True][0] if 500 <= f < 800]
    self.assertTrue(held)
    from opendbc.sunnypilot.car.hyundai.tests.test_fca11_continuous import _gas_word
    self.assertEqual({_gas_word(d) for d in held}, {0})


# ---------------------------------------------------------------------------------------------------------------------
# (i) randomised two-mode lockstep property test, stated in INPUT terms, with positive controls
def _gen_inputs(rng, n=4000):
  """A random but car-shaped drive: approaches, stops of random length, slips (flag drops at low speed), planner
  launch / dither / mild asks, rolling at any speed. Each episode is CLEAN (no driver input) or NOISY (per-frame driver
  gas/brake taps, gear changes, camera dropouts, longActive drops), so the clamp is both exercised and interrupted."""
  seq = []
  v = rng.uniform(0.0, 6.0)
  while len(seq) < n:
    ep = []
    kind = rng.random()
    if kind < 0.45:                                       # approach to a stop, then a hold
      v = rng.uniform(1.0, 6.0)
      while v > 0.0:
        v = max(0.0, v - rng.uniform(0.005, 0.03))
        ep.append(dict(accel=rng.uniform(-1.5, -0.3), v=v, ss=v < 0.104 and rng.random() < 0.9))
      for _ in range(rng.randint(20, 400)):
        slip = rng.random() < 0.02
        vv = rng.uniform(0.09, 0.3) if slip else rng.choice((0.0, 0.0, 0.0, rng.uniform(0.0, 0.09)))
        ep.append(dict(accel=rng.choice((-2.0, -2.0, -2.0, -0.85, rng.uniform(-2.0, 0.2))), v=vv,
                       ss=(not slip) and vv < 0.104 and rng.random() < 0.97))
      v = 0.0
    elif kind < 0.6:                                      # a hold without an approach (mild asks, dither, slips)
      for _ in range(rng.randint(20, 300)):
        slip = rng.random() < 0.05
        vv = rng.uniform(0.09, 0.3) if slip else rng.uniform(0.0, 0.09)
        ep.append(dict(accel=rng.uniform(-2.0, 0.3), v=vv, ss=(not slip) and rng.random() < 0.95))
    elif kind < 0.8:                                      # launch
      for _ in range(rng.randint(5, 150)):
        v = min(8.0, v + rng.uniform(0.0, 0.05))
        ep.append(dict(accel=rng.uniform(0.0, 1.0), v=v, ss=v < 0.104))
    else:                                                 # rolling at any speed with any ask
      for _ in range(rng.randint(5, 100)):
        v = rng.uniform(0.0, 3.0)
        ep.append(dict(accel=rng.uniform(-2.0, 1.0), v=v, ss=v < 0.104 and rng.random() < 0.8))
    noisy = rng.random() < 0.4
    for s in ep:
      s["gas"] = noisy and rng.random() < 0.01
      s["brake"] = noisy and rng.random() < 0.005
      s["gear"] = P if (noisy and rng.random() < 0.003) else D
      s["la"] = not (noisy and rng.random() < 0.003)
      s["cam"] = not (noisy and rng.random() < 0.005)
    seq += ep
  return seq[:n]


def _check(seq, cls=fl.Fca11LongBrake):
  """Run seq through OFF and ON (cls) in lockstep; return (violations, engagements). Invariants in INPUT terms:
    I1 ON >= OFF   I2 ON <= 30   I3 ON <= previous ON + 4 (the panda's onset step)
    I4 ON == OFF before the first frame with the standstill flag AND v <= 0.08 since the car last moved >= 0.5 m/s
    I5 ON == OFF after any standstill frame whose ask asks to move (or any driver pedal at a standstill), until the car
       has moved >= 0.5 m/s again
    I6 ON == OFF on any frame with driver gas/brake, gear != D or a move ask
    I7 a drop of > 3 steps between two sent ON frames only if OFF dropped > 3 too, or a move ask / driver pedal / gear /
       longActive drop / camera dropout / moving >= 0.5 m/s happened since the previous sent frame
    I8 ON only rises above OFF's own rise on a frame with the standstill flag and v <= 0.08 (the trigger)
    I9 a run of sent frames with ON != OFF lasts at most FCA11_SS_CLAMP_MAX_S + 1 s, and there is at most one such run
       between two moving (>= 0.5 m/s) frames
    I10 the pedal side (braking, gas_hold) is identical"""
  off, on = _Drive(clamp=False), _Drive(clamp=True, cls=cls)
  bad = []
  prev_on = prev_off = 0
  seen_stop = False
  go_lock = False
  event = False            # an I7 excuse since the previous sent frame
  run_len = 0
  runs_since_move = 0
  limit = int((fl.FCA11_SS_CLAMP_MAX_S + 1.0) * 100) // fl.FCA11_SEND_EVERY
  for i, s in enumerate(seq):
    kw = dict(standstill=s["ss"], gas=s["gas"], brake=s["brake"], gear=s["gear"], long_active=s["la"], cam_ok=s["cam"])
    a = off.step(s["accel"], s["v"], **kw)
    b = on.step(s["accel"], s["v"], **kw)
    move_ask = s["accel"] > -fl.fca11_cont_coast(max(0.0, s["v"]))
    moving = s["v"] >= 0.5 and not s["ss"]
    if moving:
      seen_stop = False
      go_lock = False
      runs_since_move = 0
    if s["ss"] and s["v"] <= 0.08:
      seen_stop = True
    if s["ss"] and (move_ask or s["gas"] or s["brake"]):
      go_lock = True
    if move_ask or s["gas"] or s["brake"] or s["gear"] != D or not s["la"] or not s["cam"] or moving:
      event = True
    if off.b.braking != on.b.braking or off.b.gas_hold != on.b.gas_hold:
      bad.append((i, "I10 pedal-side flags differ", off.b.gas_hold, on.b.gas_hold))
    if (a is None) != (b is None):
      bad.append((i, "send slot differs", a, b)); continue
    if a is None:
      continue
    if b > 30:
      bad.append((i, "I2 above 30", a, b))
    if b < a:
      bad.append((i, "I1 ON below OFF", a, b))
    if b > prev_on + 4:
      bad.append((i, "I3 faster than +4", prev_on, b))
    if b < prev_on - 3 and not (a < prev_off - 3 or event):
      bad.append((i, "I7 unexcused drop", prev_on, b))
    trig = s["ss"] and s["v"] <= 0.08
    if (not trig) and b > prev_on and b != a and (b - prev_on) > (a - prev_off):
      bad.append((i, "I8 rose without the trigger", prev_on, b))
    if b != a:
      if not seen_stop:
        bad.append((i, "I4 differs before the stop", a, b))
      if go_lock:
        bad.append((i, "I5 differs after a move ask / pedal at the stop", a, b))
      if s["gas"] or s["brake"] or s["gear"] != D or move_ask:
        bad.append((i, "I6 differs under driver input / gear / a move ask", a, b))
      if run_len == 0:
        runs_since_move += 1
        if runs_since_move > 1:
          bad.append((i, "I9 second clamp run without moving", a, b))
      run_len += 1
      if run_len > limit:
        bad.append((i, "I9 clamp run too long", run_len, limit))
    else:
      run_len = 0
    prev_on, prev_off = b, a
    event = False
  return bad, on.b.ss_clamp_engagements


def _long_creep(d):
  """Deterministic: a stop that engages the clamp, then 6 s of creeping with the flag down under a stop ask."""
  d.approach()
  d.run(8, -0.5, 0.0, standstill=True)
  d.run(600, -0.5, 0.2, a_ego=0.0, standstill=False)
  d.run(100, -0.5, 0.0, standstill=True)


class _Recorder:
  """Records the inputs a _Drive script would feed, as a _check() sequence."""
  def __init__(self):
    self.seq = []

  def step(self, accel, v, a_ego=0.0, standstill=False, gas=False, brake=False, gear=D, long_active=True, cam_ok=True):
    self.seq.append(dict(accel=accel, v=v, ss=standstill, gas=gas, brake=brake, gear=gear, la=long_active, cam=cam_ok))

  run = _Drive.run
  approach = _Drive.approach


def _seq_of(script):
  r = _Recorder()
  script(r)
  return r.seq


def _broken(old, new):
  """A copy of the clamp step with one line changed (the positive controls)."""
  src = inspect.getsource(fl.Fca11LongBrake._ss_clamp_step_checked)
  assert src.count(old) == 1, old
  src = "\n".join(line[2:] if line.startswith("  ") else line for line in src.replace(old, new).splitlines())
  ns = dict(fl.__dict__)
  exec(compile(src, "<broken>", "exec"), ns)

  class Broken(fl.Fca11LongBrake):
    pass
  Broken._ss_clamp_step_checked = ns["_ss_clamp_step_checked"]
  return Broken


class TestPropertyTwoMode(unittest.TestCase):

  def test_random_drives(self):
    total = 0
    for seed in range(40):
      bad, n = _check(_gen_inputs(random.Random(seed), n=6000))
      self.assertEqual(bad[:5], [], seed)
      total += n
    self.assertGreater(total, 30, "the generator must actually exercise the clamp")

  def test_positive_control_planner_go_ignored(self):
    Broken = _broken("    elif not (clean and contiguous) or self._ss_handback:", "    elif not contiguous:")
    hits = sum(len(_check(_gen_inputs(random.Random(s)), cls=Broken)[0]) for s in range(12))
    self.assertGreater(hits, 0, "the property test must catch a clamp that holds through a launch request")

  def test_the_long_creep_terminates(self):
    bad, n = _check(_seq_of(_long_creep))
    self.assertEqual((bad[:5], n), ([], 1))

  def test_positive_control_never_decays(self):
    Broken = _broken("    if (frame - self._ss_since) < FCA11_SS_CLAMP_MAX_FRAMES:", "    if True:")
    bad, _n = _check(_seq_of(_long_creep), cls=Broken)
    self.assertTrue(any("I9" in b[1] for b in bad), bad[:5])

  def test_positive_control_raises_without_the_trigger(self):
    Broken = _broken("      if trigger:\n        lvl = min(hold,", "      if True:\n        lvl = min(hold,")
    hits = [b for s_ in range(40) for b in _check(_gen_inputs(random.Random(s_), n=6000), cls=Broken)[0]]
    self.assertTrue(any("I8" in b[1] for b in hits), hits[:5])

  def test_positive_control_acts_before_the_stop(self):
    Broken = _broken("    trigger = bool(standstill and v <= FCA11_SS_CLAMP_V_MPS)", "    trigger = bool(v <= 0.3)")
    hits = sum(len(_check(_gen_inputs(random.Random(s)), cls=Broken)[0]) for s in range(12))
    self.assertGreater(hits, 0, "the property test must catch a clamp that acts while the car still rolls")


if __name__ == "__main__":
  unittest.main()
