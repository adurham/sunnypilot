"""0057: the STRONG STANDSTILL HOLD (param HyundaiFca11StrongHold -> CP_SP.fca11StrongHold, INT, DEFAULT 0 = OFF).

Once the continuous law holds its own 30-step standstill level at a REAL standstill, the wire may rise above it to 40
(mode 1) or through the parked staircase 32/36/40/44/46 (mode 2), at +2 per sent frame (rev 2: half the panda's +4
gate, so one lost frame in the climb can never make the next one a refused +8). Every assertion is
stated in INPUT terms (what the car and the planner did), and the property test is run against deliberately broken
variants that must FAIL it (positive controls).

  (a) wiring: the INT parse, the setup_interfaces writer (FCA11 on, Shadow off), the continuous law only, absent = OFF
  (b) OFF (field absent / 0 / junk) is byte-identical, and no strong-hold state moves
  (c) mode 1: identical until the law itself holds 30 at a real standstill, then 32 34 36 38 40 - never above 40, never
      above the last sent + 2, never below the law, and the 0056 clamp below it runs untouched; one lost frame anywhere
      in the climb leaves the next frame inside the panda's +4 gate (and a +3 or +4 climb would not)
  (d) the "never" table: gas, brake, gear, the planner asking to move or easing its ask, not armed (no prior motion),
      the standstill flag without the speed, the speed without the flag, a non-finite speed, no FCA_ACK
  (e) a launch request hands the wire back on the SAME frame: the launch is byte-identical from there on
  (f) a slip keeps the level (no rise while moving, no drop); really moving hands back; once per stop; re-arms after
      motion; an FCA_ACK drop latches it off for the ignition
  (g) fail-safe: an exception latches it off and returns the law's own level
  (h) mode 2: the parked staircase, its stop rule (motion, ESC standstill, ack, alerts), once per ignition, never after
      driving
  (i) through the REAL gas path: the pedal frames are identical ON vs OFF (exactly 0 while held)
  (j) randomised two-mode lockstep property test + broken variants that must fail it

Run from the opendbc root:  PYTHONPATH=. python -m pytest -q opendbc/sunnypilot/car/hyundai/tests/test_fca11_strong_hold.py
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
MAX_DEC = fl.HYUNDAI_FCA11_LONG_MAX_DEC
# rev 2: the climb rule as a LITERAL (half the panda's +4), so a positive control that patches the module constant back
# to +4 is still judged against +2
SH_STEP = 2


def _cp_sp(sh=None, clamp=None, aff=True, dyn=True, brake=True):
  kw = dict(fca11Brake=brake, enableGasInterceptor=True, fca11AffineGain=aff, fca11DynamicBrakes=dyn)
  if sh is not None:
    kw["fca11StrongHold"] = sh
  if clamp is not None:
    kw["fca11StandstillClamp"] = clamp
  return structs.CarParamsSP(**kw)


def _brake(sh=None, clamp=None, aff=True, dyn=True, cls=fl.Fca11LongBrake):
  return cls(structs.CarParams(), _cp_sp(sh, clamp, aff, dyn))


def _cc(accel, long_active=True):
  cc = structs.CarControl()
  cc.enabled = cc.latActive = True
  cc.longActive = long_active
  cc.actuators.accel = accel
  return cc


class _Drive:
  """Feeds one input frame at a time; records the SENT 0x38D level per controller frame."""

  def __init__(self, sh=None, clamp=None, cls=fl.Fca11LongBrake):
    self.b = _brake(sh, clamp, cls=cls)
    self.f = 0
    self.wire = []          # (frame, level) for every SENT 0x38D
    self.inputs = {}        # frame -> the inputs of that frame (for the property checks)

  def step(self, accel, v, a_ego=0.0, standstill=False, gas=False, brake=False, gear=D, long_active=True,
           cam_ok=True, ack=True, esc_ss=None):
    cs = _CS(v_ego=v, cam=_cam() if cam_ok else None, a_ego=a_ego, gear=gear, brake=brake, gas=gas)
    cs.out.standstill = standstill
    cs.fca11_lead_drel_m = cs.fca11_lead_vrel_mps = cs.fca11_lead_one = None
    cs.fca11_esc_ack = ack
    cs.fca11_esc_standstill = standstill if esc_ss is None else esc_ss
    self.inputs[self.f] = dict(accel=accel, v=v, standstill=standstill, gas=gas, brake=brake, gear=gear,
                               long_active=long_active, cam_ok=cam_ok, ack=ack)
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
    """2 s braking at 3 m/s (arms mode 1), then down the creep floor to v = 0.09 m/s (flag still False)."""
    self.run(100, -1.0, 3.0, a_ego=-0.9)
    for v in (2.0, 1.5, 1.0, 0.6, 0.3, 0.15, 0.09):
      self.run(10, -0.6, v, a_ego=-0.8)
    return self

  def levels(self, since=0, until=10 ** 9):
    return [lv for f, lv in self.wire if since <= f < until]


def _pair(script, sh_on=1, clamp=None, cls=fl.Fca11LongBrake):
  off, on = _Drive(sh=0, clamp=clamp), _Drive(sh=sh_on, clamp=clamp, cls=cls)
  for d in (off, on):
    script(d)
  return off, on


def _stop_hold_launch(d):
  d.approach()
  d.run(300, -2.0, 0.0, standstill=True)                  # 3 s held at the planner's stop ask
  d.run(150, 0.4, 0.0, standstill=True)                   # the planner asks to move
  d.run(100, 0.6, 1.0, a_ego=0.8)                         # moving off


T_STOP = 100 + 70                                          # first frame with v <= 0.08 and the standstill flag


# ---------------------------------------------------------------------------------------------------------------------
class TestWiring(unittest.TestCase):
  """(a) who may switch it on."""

  def test_mode_parse_fails_closed(self):
    for raw, want in ((None, 0), (0, 0), (1, 1), (2, 2), (3, 0), (-1, 0), ("1", 0), (True, 0), (False, 0), (1.0, 0)):
      self.assertEqual(fl.fca11_strong_hold_mode(raw), want, raw)

  def test_default_is_off_and_needs_the_continuous_law(self):
    self.assertEqual(_brake().sh_mode, 0)                    # field absent -> OFF
    self.assertEqual(_brake(sh=0).sh_mode, 0)
    self.assertEqual(_brake(sh=1).sh_mode, 1)
    self.assertEqual(_brake(sh=2).sh_mode, 2)
    self.assertEqual(_brake(sh=1, aff=False).sh_mode, 0)    # no shaping -> feel/old law -> inert
    self.assertEqual(_brake(sh=1, dyn=False).sh_mode, 0)    # no DynamicBrakes -> no continuous law -> inert
    cp = _cp_sp(sh=1, brake=False)
    self.assertEqual(fl.Fca11LongBrake(structs.CarParams(), cp).sh_mode, 0)   # FCA11 braking off -> inert

  def test_the_shadow_core_never_carries_it(self):
    self.assertEqual(fl.Fca11LongBrake(structs.CarParams(), fl._shadow_core_params(), core=True).sh_mode, 0)

  def test_constants_inside_the_panda_box(self):
    self.assertEqual(MAX_DEC, 46)
    self.assertEqual(fl.FCA11_SH_CEIL_LSB, 40)
    self.assertLessEqual(max(fl.FCA11_SH_PROBE_STEPS), MAX_DEC)
    # rev 2: +2 per sent frame = HALF the panda's +4 gate, so one lost climb frame leaves the next at exactly +4 over the
    # panda's last accepted level (accepted) instead of +8 (refused)
    self.assertEqual(fl.FCA11_SH_STEP_LSB, 2)
    self.assertEqual(fl.FCA11_SH_STEP_LSB, fl.HYUNDAI_FCA11_LONG_RATE_STEP // 2)
    self.assertLessEqual(2 * fl.FCA11_SH_STEP_LSB, fl.HYUNDAI_FCA11_LONG_RATE_STEP)
    self.assertEqual(fl.FCA11_SH_V_MPS, fl.FCA11_SS_CLAMP_V_MPS)
    self.assertEqual(fl.FCA11_SH_REARM_V_MPS, fl.FCA11_SS_CLAMP_REARM_V_MPS)
    steps = (fl.FCA11_CONT_CEIL_LSB,) + tuple(fl.FCA11_SH_PROBE_STEPS)
    self.assertTrue(all(0 < b - a <= fl.HYUNDAI_FCA11_LONG_RATE_STEP for a, b in zip(steps, steps[1:])), steps)

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
      return CP_SP.fca11StrongHold

    on = {"HyundaiFca11Brake": True}
    self.assertEqual(written([on]), 0)                                          # key absent
    self.assertEqual(written([on, {"HyundaiFca11StrongHold": 1}]), 1)
    self.assertEqual(written([on, {"HyundaiFca11StrongHold": 2}]), 2)
    self.assertEqual(written([on, {"HyundaiFca11StrongHold": "1"}]), 1)        # raw libparams string
    self.assertEqual(written([on, {"HyundaiFca11StrongHold": b"2"}]), 2)
    self.assertEqual(written([on, {"HyundaiFca11StrongHold": 0}]), 0)
    self.assertEqual(written([on, {"HyundaiFca11StrongHold": 3}]), 0)          # out of range -> OFF
    self.assertEqual(written([on, {"HyundaiFca11StrongHold": None}]), 0)       # stale libparams
    self.assertEqual(written([on, {"HyundaiFca11StrongHold": "junk"}]), 0)
    self.assertEqual(written([{"HyundaiFca11Brake": False}, {"HyundaiFca11StrongHold": 1}]), 0)
    self.assertEqual(written([on, {"HyundaiFca11Shadow": True}, {"HyundaiFca11StrongHold": 1}]), 0)


# ---------------------------------------------------------------------------------------------------------------------
class TestOffIdentity(unittest.TestCase):
  """(b) absent, 0 and junk are the same law, frame for frame, and no strong-hold state ever moves."""

  def test_absent_zero_and_junk_are_identical(self):
    runs = []
    for sh in (None, 0, 7):
      d = _Drive(sh=sh, clamp=True)
      _stop_hold_launch(d)
      runs.append(d)
      self.assertEqual(d.b.sh_engagements, 0)
      self.assertEqual(d.b.sh_frames, 0)
    self.assertEqual(runs[0].wire, runs[1].wire)
    self.assertEqual(runs[0].wire, runs[2].wire)


# ---------------------------------------------------------------------------------------------------------------------
class TestMode1(unittest.TestCase):
  """(c) mode 1: identical until the law holds its 30 at a real standstill, then 32 34 36 38 40 (rev 2: +2 per frame)."""

  def _check(self, clamp, cls=fl.Fca11LongBrake):
    off, on = _pair(_stop_hold_launch, clamp=clamp, cls=cls)
    o = dict(off.wire)
    first_diff = next(f for f, lv in on.wire if lv != o[f])
    # the first raised frame is the one on which the LAW ITSELF (not the clamp floor) first sends its 30, at a real
    # standstill; from there the wire climbs at +2 per sent frame to 40 (five sent frames, ~0.1 s, with the clamp:
    # 30 -> 32 34 36 38 40; with the law alone, which arrives from 29: 31 33 35 37 39 40)
    self.assertGreaterEqual(first_diff, T_STOP)
    self.assertEqual(o[first_diff], 30, (first_diff, off.levels(since=T_STOP, until=first_diff + 2)))
    if not clamp:
      # law alone: the sent frame before the first raise is still the law's own climb (29), i.e. the raise starts ON the
      # first frame the law commands its 30 - the wire goes 29 -> 33, never 29 -> 30 -> 34
      self.assertLess(o[first_diff - 2], 30, (first_diff, off.levels(since=T_STOP, until=first_diff + 2)))
    prev_wire = dict(on.wire)[first_diff - 2]
    seq = on.levels(since=first_diff, until=first_diff + 20)
    want = []
    for _ in seq:
      prev_wire = min(fl.FCA11_SH_CEIL_LSB, prev_wire + fl.FCA11_SH_STEP_LSB)
      want.append(prev_wire)
    self.assertEqual(seq, want, seq)
    climb = [32, 34, 36, 38] if clamp else [31, 33, 35, 37, 39]     # the literal rev-2 staircase, pinned
    self.assertEqual(seq[:len(climb)], climb, seq)
    self.assertEqual(seq[len(climb):], [40] * (len(seq) - len(climb)), seq)
    self.assertEqual(on.b.sh_engagements, 1)
    prev = 0
    for f, lv in on.wire:
      self.assertLessEqual(lv, fl.FCA11_SH_CEIL_LSB, f)
      self.assertGreaterEqual(lv, o[f], f)                    # never below today's law
      self.assertLessEqual(lv, prev + fl.HYUNDAI_FCA11_LONG_RATE_STEP, f)
      if lv > o[f]:
        inp = on.inputs[f]
        self.assertTrue(inp["standstill"] and inp["v"] <= fl.FCA11_SH_V_MPS and inp["accel"] <= -1.0, (f, inp))
        self.assertLessEqual(lv, prev + fl.FCA11_SH_STEP_LSB, f)   # a raised frame is never more than +2 over the last
      prev = lv
    return off, on

  def test_on_top_of_the_law_alone(self):
    self._check(clamp=False)

  def test_on_top_of_the_clamp(self):
    off, on = self._check(clamp=True)
    # the clamp below it ran exactly as without the strong hold (same engagements, same fast climb to 30)
    self.assertEqual(off.b.ss_clamp_engagements, on.b.ss_clamp_engagements)
    self.assertEqual(off.levels(until=T_STOP + 30), on.levels(until=T_STOP + 30))

  def test_one_lost_climb_frame_stays_inside_the_panda_gate(self):
    # rev 2 (F-OPUS-REVIEW-strong-hold-2026-11-22 §2): ONE host frame lost anywhere in the climb must not make the panda
    # refuse the next one. A lost frame never reaches the panda (no echo), so the car layer sends on unchanged and the
    # panda judges the next frame against the frame BEFORE the lost one. Two lost in a row are NOT covered (+6).
    for clamp in (True, False):
      off, on = _pair(_stop_hold_launch, clamp=clamp)
      self.assertGreater(on.b.sh_frames, 0)
      self.assertEqual(_single_loss_refusals(off, on), [], clamp)
    # positive controls: the rev-1 +4 climb (a lost 34 -> the next 38 is +8 over the panda's 30) and a +3 climb (a lost
    # 33 -> the next 36 is +6) are both caught by the same check
    from unittest import mock
    for step in (3, 4):
      with mock.patch.object(fl, "FCA11_SH_STEP_LSB", step):
        off, on = _pair(_stop_hold_launch, clamp=True)
      self.assertTrue(_single_loss_refusals(off, on), step)


def _single_loss_refusals(off, on):
  """The panda's onset gate (hyundai.h: accept an actuating level only if <= HYUNDAI_FCA11_LONG_MAX_DEC and <= its last
  ACCEPTED level + HYUNDAI_FCA11_LONG_RATE_STEP) applied to the ON wire with ONE sent frame k dropped, for every k from
  the frame before the strong hold's first raise through the frame after its last. Returns (frame, ref, lost, next) for
  every k whose loss would get the NEXT frame refused."""
  o = dict(off.wire)
  frames = [f for f, _ in on.wire]
  lv = [x for _, x in on.wire]
  raised = [i for i, f in enumerate(frames) if lv[i] > o[f]]
  bad = []
  for k in range(max(1, raised[0] - 1), min(len(lv) - 1, raised[-1] + 2)):
    ref, nxt = lv[k - 1], lv[k + 1]
    if nxt > min(MAX_DEC, ref + fl.HYUNDAI_FCA11_LONG_RATE_STEP):
      bad.append((frames[k], ref, lv[k], nxt))
  return bad


class TestNever(unittest.TestCase):
  """(d) each case is the whole stop, frame for frame, against OFF: nothing above the law."""

  def _case(self, approach=True, **hold_kw):
    def script(d):
      if approach:
        d.approach()
      d.run(250, hold_kw.get("accel", -2.0), hold_kw.get("v", 0.0), standstill=hold_kw.get("standstill", True),
            gas=hold_kw.get("gas", False), brake=hold_kw.get("brake", False), gear=hold_kw.get("gear", D),
            ack=hold_kw.get("ack", True))
    off, on = _pair(script, clamp=True)
    self.assertEqual(off.wire, on.wire)
    self.assertEqual(on.b.sh_engagements, 0)

  def test_gas(self):
    self._case(gas=True)

  def test_brake(self):
    self._case(brake=True)

  def test_gear(self):
    self._case(gear=P)

  def test_planner_asks_to_move(self):
    self._case(accel=0.3)

  def test_planner_eases_its_ask(self):
    self._case(accel=-0.8)                                    # still a stop demand (bn > 0) but above -1.0

  def test_not_armed_without_prior_motion(self):
    self._case(approach=False)

  def test_standstill_flag_without_the_speed(self):
    self._case(v=0.09)

  def test_speed_without_the_standstill_flag(self):
    self._case(standstill=False)

  def test_non_finite_speed(self):
    self._case(v=float("nan"))

  def test_no_fca_ack(self):
    self._case(ack=False)


class TestLaunchAndSlip(unittest.TestCase):
  """(e) + (f)"""

  def test_launch_is_byte_identical_from_the_move_request(self):
    off, on = _pair(_stop_hold_launch, clamp=True)
    t_go = T_STOP + 300
    self.assertEqual(off.levels(since=t_go), on.levels(since=t_go))
    self.assertEqual(on.b._sh_end_reason, "handback")

  def test_odd_frame_move_request_is_seen(self):
    def script(d):
      d.approach()
      d.run(301, -2.0, 0.0, standstill=True)                  # ends on an odd frame
      d.step(0.4, 0.0, standstill=True)                       # ONE move-ask frame, on a non-send slot
      d.run(60, -2.0, 0.0, standstill=True)                   # the planner flickers back to its stop ask
    off, on = _pair(script, clamp=True)
    t_go = T_STOP + 301
    self.assertEqual(off.levels(since=t_go), on.levels(since=t_go))
    self.assertEqual(on.b.sh_engagements, 1)                  # once per stop: it does not come back at this stop

  def test_slip_keeps_the_level_and_really_moving_hands_back(self):
    def script(d):
      d.approach()
      d.run(200, -2.0, 0.0, standstill=True)                  # at 40
      d.run(60, -2.0, 0.15, standstill=False)                 # a creep slip under the stop ask
      d.run(60, -2.0, 0.0, standstill=True)                   # caught again
      d.run(20, -2.0, 0.6, standstill=False)                  # really moving (>= 0.5 m/s)
      d.run(60, -2.0, 0.0, standstill=True)                   # stopped again with no new arming drive
    off, on = _pair(script, clamp=True)
    t_slip = T_STOP + 200
    slip = on.levels(since=t_slip, until=t_slip + 60)
    self.assertTrue(all(lv == fl.FCA11_SH_CEIL_LSB for lv in slip), slip)       # neither rises nor falls in the slip
    self.assertGreater(min(slip), max(off.levels(since=t_slip, until=t_slip + 60)))
    t_move = t_slip + 120
    self.assertEqual(off.levels(since=t_move, until=t_move + 20), on.levels(since=t_move, until=t_move + 20))
    self.assertEqual(on.b.sh_engagements, 2)                  # the >= 0.5 m/s roll re-armed it for the next stop

  def test_once_per_stop_and_rearm_after_motion(self):
    def script(d):
      d.approach()
      d.run(200, -2.0, 0.0, standstill=True)
      d.run(30, -2.0, 0.0, standstill=True, gas=True)        # the driver touches the gas: hands back, disarmed
      d.run(100, -2.0, 0.0, standstill=True)                  # same stop: stays off
      d.approach()                                            # drives again (re-arms) and stops
      d.run(200, -2.0, 0.0, standstill=True)
    off, on = _pair(script, clamp=True)
    self.assertEqual(on.b.sh_engagements, 2)
    o = dict(off.wire)
    t_same = T_STOP + 230
    self.assertEqual([lv for f, lv in on.wire if t_same <= f < t_same + 100],
                     [o[f] for f, _ in on.wire if t_same <= f < t_same + 100])

  def test_ack_drop_latches_off_for_the_ignition(self):
    def script(d):
      d.approach()
      d.run(150, -2.0, 0.0, standstill=True)
      d.run(10, -2.0, 0.0, standstill=True, ack=False)       # the ESC stops acknowledging at 40
      d.run(100, -2.0, 0.0, standstill=True)
      d.approach()
      d.run(200, -2.0, 0.0, standstill=True)
    off, on = _pair(script, clamp=True)
    self.assertTrue(on.b._sh_ack_latch)
    self.assertEqual(on.b.sh_engagements, 1)
    t_drop = T_STOP + 150
    self.assertEqual(off.levels(since=t_drop), on.levels(since=t_drop))   # the very next sent frame is the law's


class TestFailSafe(unittest.TestCase):
  """(g) an exception inside the strong hold latches it off and returns the law's level."""

  def test_exception_while_engaged(self):
    def script(d, boom_at=None):
      d.approach()
      for i in range(300):
        if boom_at is not None and i == boom_at:
          d.b._sh_target = None                               # the next call raises TypeError inside the guard
        d.step(-2.0, 0.0, standstill=True)
    off = _Drive(sh=0, clamp=True)
    script(off)
    on = _Drive(sh=1, clamp=True)
    script(on, boom_at=150)
    self.assertTrue(on.b._sh_fault)
    self.assertEqual(on.b.sh_faults, 1)
    t = T_STOP + 151
    self.assertEqual(off.levels(since=t), on.levels(since=t))

  def test_exception_in_the_watcher(self):
    on = _Drive(sh=1, clamp=True)
    on.approach()
    on.run(100, -2.0, 0.0, standstill=True)
    on.b._planner_ask = None
    on.b._sh_watch(_CS(v_ego=0.0), -2.0)
    self.assertTrue(on.b._sh_fault)


class TestParkedStaircase(unittest.TestCase):
  """(h) mode 2: one staircase per ignition, parked, with its stop rule."""

  @staticmethod
  def _parked(d, n=2600, **kw):
    d.run(n, -2.0, 0.0, standstill=True, **kw)               # engaged parked, the planner holding the stop
    return d

  def test_the_staircase(self):
    off, on = _Drive(sh=0, clamp=True), _Drive(sh=2, clamp=True)
    for d in (off, on):
      self._parked(d, n=4800)                                 # ~0.6 s law ramp + 2 s settle + 5 x 8 s + margin
    o = dict(off.wire)
    self.assertTrue(all(lv <= 30 for lv in o.values()))
    runs = []
    for _, lv in on.wire:
      if runs and runs[-1][0] == lv:
        runs[-1][1] += 1
      else:
        runs.append([lv, 1])
    above = [(lv, n) for lv, n in runs if lv > 30]
    # rev 2: each step's dwell counts from its first sent frame, and a 4-step rise climbs at +2, so it spends ONE of its
    # FCA11_SH_PROBE_DWELL_SENT frames (20 ms) on the level between: 32 | 34 36 | 38 40 | 42 44 | 46
    dw = fl.FCA11_SH_PROBE_DWELL_SENT
    self.assertEqual([lv for lv, _ in above], [32, 34, 36, 38, 40, 42, 44, 46], runs)
    self.assertEqual(above, [(32, dw), (34, 1), (36, dw - 1), (38, 1), (40, dw - 1), (42, 1), (44, dw - 1), (46, dw)])
    self.assertEqual([lv for lv, n in above if n > 1], list(fl.FCA11_SH_PROBE_STEPS))
    self.assertEqual(on.levels()[-1], 30)                     # back to the law's own hold
    self.assertEqual(on.b._sh_probe_state, "done")
    self.assertEqual(on.b.sh_probe_end_reason, "complete")
    self.assertEqual(on.b.sh_max_lvl, 46)
    prev = 0
    for f, lv in on.wire:
      self.assertLessEqual(lv, prev + 4, f)
      self.assertGreaterEqual(lv, o[f], f)
      if lv > o[f]:
        self.assertLessEqual(lv, prev + fl.FCA11_SH_STEP_LSB, f)
      prev = lv

  def _aborted(self, v=0.0, standstill=True, resend=True, **kw):
    on = _Drive(sh=2, clamp=True)
    self._parked(on, n=800)                                   # into step 2 (settle 100 sent + 8 s steps)
    self.assertEqual(on.b._sh_probe_state, "running")
    on.run(20, -2.0, v, standstill=standstill, **kw)
    self.assertEqual(on.b._sh_probe_state, "done")
    self.assertLessEqual(on.levels()[-1], 30)                 # back to the law's own level, never above it
    f_end = on.f
    on.run(1000, -2.0, 0.0, standstill=True)
    after = on.levels(since=f_end)
    self.assertTrue(all(lv <= 30 for lv in after), after[:10])   # never re-run this ignition
    self.assertEqual(bool(after), resend)                    # (a driver brake latches the cut until re-engage)
    return on

  def test_abort_on_motion(self):
    self.assertEqual(self._aborted(v=0.06).b.sh_probe_end_reason, "probe_motion")

  def test_abort_on_the_standstill_flag(self):
    self.assertEqual(self._aborted(standstill=False).b.sh_probe_end_reason, "probe_motion")

  def test_abort_on_esc_standstill(self):
    self.assertEqual(self._aborted(esc_ss=False).b.sh_probe_end_reason, "probe_esc_standstill")

  def test_abort_on_ack(self):
    self.assertEqual(self._aborted(ack=False).b.sh_probe_end_reason, "ack")

  def test_abort_on_gas(self):
    self.assertEqual(self._aborted(gas=True).b.sh_probe_end_reason, "pedal")

  def test_abort_on_brake(self):
    self.assertEqual(self._aborted(brake=True, resend=False).b.sh_probe_end_reason, "pedal")

  def test_abort_on_a_move_ask(self):
    on = _Drive(sh=2, clamp=True)
    self._parked(on, n=800)
    on.run(20, 0.3, 0.0, standstill=True)
    self.assertEqual(on.b.sh_probe_end_reason, "move_ask")
    f_end = on.f
    on.run(1000, -2.0, 0.0, standstill=True)
    after = on.levels(since=f_end)
    self.assertTrue(after and all(lv <= 30 for lv in after), after[:10])

  def test_abort_on_an_alert(self):
    on = _Drive(sh=2, clamp=True)
    self._parked(on, n=800)
    on.b.brake_now = True                                     # stands for every FCA11 alert state
    on.step(-2.0, 0.0, standstill=True)
    on.step(-2.0, 0.0, standstill=True)
    self.assertEqual(on.b._sh_probe_state, "done")
    self.assertEqual(on.b.sh_probe_end_reason, "probe_alert")

  def test_never_after_driving(self):
    d = _Drive(sh=2, clamp=True)
    d.run(50, 0.5, 3.0)                                       # driven at 3 m/s (> 2) with openpilot
    self._parked(d, n=1500)
    self.assertTrue(all(lv <= 30 for lv in d.levels()))
    self.assertEqual(d.b._sh_probe_state, "done")
    self.assertEqual(d.b.sh_probe_end_reason, "driven")

  def test_never_after_driving_with_openpilot_off(self):
    d = _Drive(sh=2, clamp=True)
    d.run(50, 0.0, 6.0, long_active=False)                   # driven manually
    self._parked(d, n=1500)
    self.assertTrue(all(lv <= 30 for lv in d.levels()))

  def test_disengage_during_settle_allows_a_retry(self):
    d = _Drive(sh=2, clamp=True)
    d.run(100, -2.0, 0.0, standstill=True)
    d.run(20, -2.0, 0.0, standstill=True, long_active=False)
    self._parked(d, n=4800)
    self.assertEqual(max(d.levels()), 46)


class TestRealGasPath(unittest.TestCase):
  """(i) the pedal: through GasInterceptorCarController.create_gas_command on the car's own CarInterface."""

  def _ci(self, sh):
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
                                               {"HyundaiFca11StandstillClamp": True}, {"HyundaiFca11StrongHold": sh}])
    return CarInterface(CP, CP_SP)

  def test_pedal_frames_identical_and_zero_while_held(self):
    from opendbc.sunnypilot.car.hyundai.tests.test_gas_interceptor import _Fca11CS, _fca11_cam_frame
    from opendbc.sunnypilot.car.hyundai.tests.test_fca11_continuous import _gas_word
    script = [(150, 0.3, 4.0, False), (100, -1.0, 3.0, False)]
    script += [(10, -0.6, v, False) for v in (2.0, 1.5, 1.0, 0.6, 0.3, 0.15, 0.09)]
    script += [(300, -2.0, 0.0, True), (200, 0.5, 0.0, True), (100, 0.6, 1.0, False)]
    out = {}
    for sh in (0, 1):
      CI = self._ci(sh)
      self.assertEqual(CI.CC.fca11_brake.sh_mode, sh)
      pedal, lv, f = [], [], 0
      for n, accel, v, ss in script:
        for _ in range(n):
          cs = _Fca11CS(v_ego=v, cam=(0, _fca11_cam_frame(alive=f % 16)))
          cs.out.standstill = ss
          cs.out.aEgo = 0.0
          cs.fca11_esc_ack = True
          cs.fca11_esc_standstill = ss
          sends = CI.CC.create_gas_command(_cc(accel), cs, f, STD)
          pedal += [(f, bytes(d)) for a, d, _s in sends if a in (0x200, 0x700)]
          lv += [(f, d[1]) for a, d, _s in sends if a == 0x38D]
          f += 1
      out[sh] = (pedal, lv)
    self.assertEqual(out[0][0], out[1][0])                    # every pedal frame byte-identical
    self.assertNotEqual(out[0][1], out[1][1])                 # while the brake stream did change (it ran)
    self.assertEqual(max(v for _, v in out[1][1]), fl.FCA11_SH_CEIL_LSB)
    held = [d for f, d in out[1][0] if 500 <= f < 800]
    self.assertTrue(held)
    self.assertEqual({_gas_word(d) for d in held}, {0})


class TestLog(unittest.TestCase):
  """the k="sh" trace: every sent frame above the law carries what the next drive needs to measure the ESC; OFF opens
  no file at all."""

  def _run(self, sh):
    import json
    import os
    import tempfile
    from unittest import mock
    d = tempfile.mkdtemp(prefix="sh-log-")
    with mock.patch.dict(os.environ, {"FCA11_LOG_DIR": d}):
      drv = _Drive(sh=sh, clamp=True)
      _stop_hold_launch(drv)
    if drv.b._sh_log_w:
      drv.b._sh_log_w.flush()                                 # the logger buffers in memory between flushes
    path = os.path.join(d, "fca11-shadow.jsonl")
    recs = [json.loads(line) for line in open(path)] if os.path.exists(path) else None
    return drv, recs

  def test_off_writes_nothing(self):
    _, recs = self._run(0)
    self.assertIsNone(recs)

  def test_on_records_every_raised_frame(self):
    drv, recs = self._run(1)
    sh = [r for r in recs if r.get("k") == "sh"]
    self.assertEqual(len(sh), drv.b.sh_frames)
    self.assertGreater(len(sh), 100)
    for r in sh:
      for key in ("lvl", "law", "ack", "esc_standstill", "a_basis", "clutch_tq_nm", "wheel_max_kph", "whl", "v_ego",
                  "a_ego", "ask", "standstill", "frame", "t_wall", "mode"):
        self.assertIn(key, r)
      self.assertGreater(r["lvl"], r["law"])
      self.assertEqual(len(r["whl"]), 4)
    self.assertEqual({r["lvl"] for r in sh}, {32, 34, 36, 38, 40})   # rev 2: the +2 climb, then 40
    ends = [r for r in recs if r.get("k") == "sh_end"]
    self.assertEqual([r["reason"] for r in ends], ["handback"])


# ---------------------------------------------------------------------------------------------------------------------
def _random_script(rng):
  """A car-shaped random sequence: drives, stops (some long), slips, flickers, launches, pedals, ack drops, vetoes."""
  segs = []
  for _ in range(rng.randint(3, 7)):
    segs.append(("drive", rng.randint(60, 200), rng.uniform(1.0, 8.0)))
    segs.append(("approach", 0, 0))
    if rng.random() < 0.3:
      # a slip that starts INSIDE the strong hold's 30 -> 40 climb (the law reaches its own 30 ~66 frames after the
      # stop with the clamp ON), so a variant that keeps rising while the car rolls is exercised
      segs.append(("hold", rng.randint(60, 76), -2.0))
      segs.append(("slip", rng.randint(4, 30), 0))
    segs.append(("hold", rng.randint(50, 900), rng.choice([-2.0, -2.0, -2.0, -1.5, -0.9, -0.5])))
    for _ in range(rng.randint(0, 4)):
      segs.append((rng.choice(["slip", "flick_ask", "gas", "brake", "ackdrop", "nostand", "speedonly", "gear", "veto",
                               "ease"]), rng.randint(1, 40), 0))
      segs.append(("hold", rng.randint(10, 300), -2.0))
    segs.append(("launch", rng.randint(20, 120), 0))
  return segs


def _play(d, segs):
  for kind, n, x in segs:
    if kind == "drive":
      d.run(n, -0.3, x, a_ego=-0.2)
    elif kind == "approach":
      for v in (2.0, 1.2, 0.6, 0.3, 0.12, 0.07):
        d.run(8, -0.8, v, a_ego=-0.8)
    elif kind == "hold":
      d.run(n, x, 0.0, standstill=True)
    elif kind == "slip":
      d.run(n, -2.0, 0.14, standstill=False)
    elif kind == "flick_ask":
      d.run(n, 0.2, 0.0, standstill=True)
    elif kind == "ease":
      d.run(n, -0.7, 0.0, standstill=True)
    elif kind == "gas":
      d.run(n, -2.0, 0.0, standstill=True, gas=True)
    elif kind == "brake":
      d.run(n, -2.0, 0.0, standstill=True, brake=True)
    elif kind == "ackdrop":
      d.run(n, -2.0, 0.0, standstill=True, ack=False)
    elif kind == "nostand":
      d.run(n, -2.0, 0.0, standstill=False)
    elif kind == "speedonly":
      d.run(n, -2.0, 0.09, standstill=True)
    elif kind == "gear":
      d.run(n, -2.0, 0.0, standstill=True, gear=P)
    elif kind == "veto":
      d.run(n, -2.0, 0.0, standstill=True, cam_ok=False)
    elif kind == "launch":
      d.run(n, 0.6, 0.0, standstill=True)
      d.run(n, 0.6, 1.5, a_ego=0.8)


def _violations(off, on, ceil=fl.FCA11_SH_CEIL_LSB):
  """The rules, in INPUT terms. Returns a list of (frame, rule) violations."""
  o = dict(off.wire)
  bad = []
  prev = 0
  prev_above = False
  for f, lv in on.wire:
    inp = on.inputs[f]
    if lv < o[f]:
      bad.append((f, "below_off"))
    if lv > min(MAX_DEC, ceil):
      bad.append((f, "over_ceiling"))
    if lv > prev + fl.HYUNDAI_FCA11_LONG_RATE_STEP:
      bad.append((f, "over_rate"))
    if lv > o[f] and lv > prev + SH_STEP:
      # rev 2: a frame the strong hold raised is never more than +2 over the last sent frame - also through slips,
      # eased asks, ack drops and vetoes mid-climb (a lost frame then still leaves the next one inside the panda gate)
      bad.append((f, "over_sh_step"))
    if lv > max(prev, o[f]) and not (inp["standstill"] and inp["v"] <= fl.FCA11_SH_V_MPS):
      bad.append((f, "rise_while_moving"))
    if lv > o[f]:
      if inp["gas"] or inp["brake"] or inp["gear"] != D or not inp["ack"] or not inp["cam_ok"]:
        bad.append((f, "above_off_with_driver_or_veto"))
      if inp["v"] >= fl.FCA11_SH_REARM_V_MPS or not math.isfinite(inp["v"]):
        bad.append((f, "above_off_while_really_moving"))
      # the raw ask on THIS frame, and - once it is already above the law - on the non-send frame before it as well (the
      # watcher must catch an eased / move ask on any frame; a first raise only needs this frame's ask)
      if any(on.inputs.get(g, {}).get("accel", -2.0) > fl.FCA11_SH_HOLD_ASK_MPS2 for g in ((f, f - 1) if prev_above else (f,))):
        bad.append((f, "above_off_without_a_hold_ask"))
    prev = lv
    prev_above = lv > o[f]
  return bad


def _property(module_cls=fl.Fca11LongBrake, seeds=range(60)):
  total = []
  eng = 0
  for seed in seeds:
    rng = random.Random(seed)
    segs = _random_script(rng)
    off, on = _Drive(sh=0, clamp=True, cls=module_cls), _Drive(sh=1, clamp=True, cls=module_cls)
    _play(off, segs)
    _play(on, segs)
    total += [(seed,) + v for v in _violations(off, on)]
    eng += on.b.sh_engagements
  return total, eng


def _broken(*pairs):
  """A subclass whose _sh_step_checked / _sh_watch carry the given source edits, each hitting EXACTLY one place."""
  ns = {}
  srcs = {name: inspect.getsource(getattr(fl.Fca11LongBrake, name)) for name in ("_sh_step_checked", "_sh_watch")}
  for replace_from, replace_to in pairs:
    hits = [name for name, src in srcs.items() if src.count(replace_from) == 1]
    assert len(hits) == 1 and sum(src.count(replace_from) for src in srcs.values()) == 1, (replace_from, hits)
    srcs[hits[0]] = srcs[hits[0]].replace(replace_from, replace_to)
  for name, src in srcs.items():
    src = "\n".join(line[2:] if line.startswith("  ") else line for line in src.splitlines())
    exec(compile(src, f"<broken {name}>", "exec"), fl.__dict__, ns)
  return type("Broken", (fl.Fca11LongBrake,), {"_sh_step_checked": ns["_sh_step_checked"],
                                                "_sh_watch": ns["_sh_watch"]})


class TestProperty(unittest.TestCase):
  """(j) randomised two-mode lockstep, then broken copies that MUST fail the same rules."""

  def test_property(self):
    bad, eng = _property()
    self.assertEqual(bad, [], bad[:20])
    self.assertGreater(eng, 40)                               # it really engaged

  def test_control_climbs_at_the_panda_rate(self):
    # rev 2 positive control: the rev-1 +4 climb fails the +2 rule (the rest of the rules still hold for it)
    from unittest import mock
    with mock.patch.object(fl, "FCA11_SH_STEP_LSB", fl.HYUNDAI_FCA11_LONG_RATE_STEP):
      bad, _ = _property(seeds=range(5))
    self.assertTrue(bad and all(v[2] == "over_sh_step" for v in bad), bad[:5])

  def test_control_rises_while_moving(self):
    bad, _ = _property(_broken(("    if trigger:\n      lvl = min(", "    if True:\n      lvl = min(")), seeds=range(30))
    self.assertTrue(any(v[2] == "rise_while_moving" for v in bad), bad[:5])

  def test_control_ignores_the_ack(self):
    bad, _ = _property(_broken(("    if self._sh_engaged and not acked:", "    if False:")), seeds=range(30))
    self.assertTrue(any(v[2] == "above_off_with_driver_or_veto" for v in bad), bad[:5])

  def test_control_ignores_an_eased_ask(self):
    bad, _ = _property(_broken(("float(self._planner_ask) <= FCA11_SH_HOLD_ASK_MPS2 and", "True and"), ("      soft_ask = float(self._planner_ask) > FCA11_SH_HOLD_ASK_MPS2", "      soft_ask = False")), seeds=range(30))
    self.assertTrue(any(v[2] == "above_off_without_a_hold_ask" for v in bad), bad[:5])

  def test_control_rises_without_the_standstill_flag(self):
    bad, _ = _property(_broken(("    trigger = bool(standstill and v <= FCA11_SH_V_MPS)", "    trigger = bool(v <= 0.3)")),
                       seeds=range(30))
    self.assertTrue(any(v[2] == "rise_while_moving" for v in bad), bad[:5])

  def test_the_pedal_is_also_a_production_veto(self):
    # NOT a positive control, a documented redundancy: with BOTH of the strong hold's own pedal gates removed nothing
    # changes, because a driver pedal is a production veto (fca11_ok) - the law is not even evaluated on such a frame,
    # so nothing above it can be sent. The strong hold's gates are a second line, never the only one.
    bad, _ = _property(_broken(("not bool(CS.out.gasPressed) and not bool(CS.out.brakePressed) and", ""),
                               ("      pedal = bool(CS.out.gasPressed) or bool(CS.out.brakePressed)", "      pedal = False")),
                       seeds=range(10))
    self.assertEqual(bad, [], bad[:5])

  def test_control_ignores_really_moving(self):
    # deterministic positive control for the >= 0.5 m/s hand-back: the real class passes the scenario test, a copy
    # without the hand-back (in both the watcher and the sent-frame step) keeps 40 on the wire while the car rolls
    broken = _broken(('        self._sh_retire("moving")\n        return dec_in', "        pass"),
                     ("        if self._sh_engaged:\n          self._sh_handback = True\n      move_ask", "        if False:\n          pass\n      move_ask"))

    def script(d):
      d.approach()
      d.run(200, -2.0, 0.0, standstill=True)
      d.run(20, -2.0, 0.6, standstill=False)                  # really moving under a stop ask
    for cls, must_hand_back in ((fl.Fca11LongBrake, True), (broken, False)):
      off, on = _Drive(sh=0, clamp=True), _Drive(sh=1, clamp=True, cls=cls)
      for d in (off, on):
        script(d)
      t_move = T_STOP + 200
      handed_back = off.levels(since=t_move + 2) == on.levels(since=t_move + 2)
      self.assertEqual(handed_back, must_hand_back, (cls.__name__, on.levels(since=t_move, until=t_move + 20)))

if __name__ == "__main__":
  unittest.main()
