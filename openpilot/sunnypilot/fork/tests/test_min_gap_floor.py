"""
Tests for the fork's low-speed minimum-gap brake floor, v2.1 (sunnypilot/fork/min_gap_floor.py, param MinGapBrakeFloor).

The fixture frames are the REAL ones from route 000001ac, segment 59 (2026-11-19, I-80/94 stop-and-go behind a flat-rear
RV trailer): the gap collapse at 59:40.32-59:40.96 and the range triplet 4.6 -> 8.9 -> 3.1 m inside 0.3 s.

Where each requirement is pinned:
  R1 release dwell / no spike release / no short re-arm         TestReleaseDwell, TestPostReleaseHold
  R2/R3 the post-release hold, v < 1 m/s, no 1.0/1.5 cycle      TestReleaseDwell, TestPostReleaseHold
  R4 dropout bridge                                            TestDropoutBridge
  R5 launch grace                                              TestArming
  review item 2  a missing modelProb = untrusted (fail closed)  TestContainmentOnRealMessages
  review item 3  contained hook (OFF reads nothing, ON latches) TestContainmentOnRealMessages, TestRealPlanner
  review item 4  BRAKE duration cap, stale radarState, samples  TestDurationCap, TestStaleRadarState
  review item 5  dependency gate on the continuous law          TestDependencyGate, TestHoldOnTheCarsOwnLaw
  review item 6  what the hold ask does at the car layer         TestHoldOnTheCarsOwnLaw
  review item 7  flicker bound                                  TestFlicker
  review item 8  gas / disengage drop BRAKE within one frame    TestOffAndFailsafe, TestContainmentOnRealMessages
  Opus review finding 1  a gas press arrives as a planner RESET  TestOverrideVsDisengage, TestRealPlanner
                 (override vs disengage; real signal orders from route 000001ac and the 390-segment corpus)
  Opus re-check, open items (2026-11-21)  the post-cap coast is   TestCoastBoundInWallTime, TestEnableMidDrive
                 bounded in WALL time; the launch grace after a mid-drive enable (real dropout train, real crawl launch)
The real-message replay of the REAL planner over rlogs (red on v1 @ 1604d4a250, green here) is the sim harness
~/comma-sim/wsSTOP/drive-2026-11-19/sim/mingap/v2fix/scripts/replay_real.py; TestRealPlanner is its in-tree miniature.
Note: TestOffAndFailsafe.test_driver_gas_drops_an_active_brake_in_one_frame_and_starts_the_cooldown feeds gas with the
planner still ACTIVE on the first press frame - an order this car produces in only ~22 % of engaged presses; the orders
it does produce are in TestOverrideVsDisengage.
"""
import os

import numpy as np

from opendbc.car import structs
from openpilot.cereal import custom, log
import openpilot.cereal.messaging as messaging
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.fork import min_gap_floor as mg
from opendbc.sunnypilot.car.hyundai import fca11_long as fl
from opendbc.sunnypilot.car.hyundai import gas_interceptor as gi

DT = 0.05
COAST = fl.fca11_cont_coast


def HOLD(v):
  return mg.hold_ask(v, COAST)


def law_cp_sp(brake=True, pedal=True, affine=True, dynamic=True, shadow=False):
  """The car's own configuration (2026-11-19 carParamsSP: all four True, Shadow False) unless told otherwise."""
  return structs.CarParamsSP(fca11Brake=brake, enableGasInterceptor=pedal, fca11AffineGain=affine,
                             fca11DynamicBrakes=dynamic, fca11Shadow=shadow)


def capnp_cp_sp(**kw):
  """The same flags as the capnp reader plannerd hands the planner (messaging.log_from_bytes of the CarParamsSP param)."""
  return messaging.log_from_bytes(capnp_cp_sp_bytes(**kw), custom.CarParamsSP)


def capnp_cp_sp_bytes(brake=True, pedal=True, affine=True, dynamic=True, shadow=False):
  m = custom.CarParamsSP.new_message()
  m.fca11Brake, m.enableGasInterceptor, m.fca11AffineGain = brake, pedal, affine
  m.fca11DynamicBrakes, m.fca11Shadow = dynamic, shadow
  return m.to_bytes()


class FakeParams:
  def __init__(self, enabled=True, unknown=False, cp_sp_raw=None):
    self.enabled = enabled
    self.unknown = unknown
    self.cp_sp_raw = cp_sp_raw
    self.reads = []

  def get_bool(self, key):
    self.reads.append(key)
    if key != mg.PARAM:
      raise AssertionError(f"the floor read an unexpected bool param: {key}")
    if self.unknown:
      raise KeyError(key)
    return self.enabled

  def get(self, key):
    self.reads.append(key)
    if key != mg.CP_SP_PARAM:
      raise AssertionError(f"the floor read an unexpected param: {key}")
    return self.cp_sp_raw


# --- real frames, route 000001ac seg 59 (elapsed t, radarState.leadOne.dRel)
GAP_COLLAPSE_59_40 = [(3580.319, 2.965), (3580.362, 2.681), (3580.414, 2.539), (3580.474, 2.806), (3580.521, 2.642),
                      (3580.570, 2.130), (3580.613, 1.908), (3580.670, 1.836), (3580.719, 1.854), (3580.763, 1.722)]
# the range triplet: 4.6 -> 8.9 -> 3.1 m inside 0.3 s (real range noise at these ranges)
RANGE_TRIPLET = [4.638, 7.469, 6.386, 8.895, 8.505, 9.190, 8.395, 6.625, 5.098, 4.867, 4.371, 3.855, 3.100]


def make(enabled=True, cp_sp=None, params=None, **kw):
  return mg.MinGapFloor(params=params if params is not None else FakeParams(enabled=enabled), dt=DT,
                        cp_sp=cp_sp if cp_sp is not None else law_cp_sp(), **kw)


class Ctx:
  """One-frame stepper with the previous call's inputs as defaults. timed=True feeds real message timing: the planner
  frame advances DT and a fresh radarState (one frame old) arrives each frame unless frozen=True."""

  def __init__(self, f, timed=False, **_kw):
    self.f = f
    self.timed = timed
    self.now = 100.0
    self.radar_t = self.now - DT
    self.kw = {"ask": 0.0, "v": 5.0, "active": True, "gas": False, "present": True, "d": 2.0, "prob": 1.0,
               "stand": False, "frozen": False}
    self.kw.update(_kw)
    self.out = 0.0

  def push(self, n=1, **kw):
    self.kw.update(kw)
    k = self.kw
    for _ in range(n):
      if self.timed:
        self.now += DT
        if not k["frozen"]:
          self.radar_t = self.now - DT
        rt, mt, nt = self.radar_t, self.radar_t, self.now
      else:
        rt = mt = nt = None
      self.out = self.f.update(k["ask"], k["v"], k["active"], k["gas"], k["present"], k["d"], k["prob"], k["stand"],
                               radar_t=rt, radar_md_t=mt, now_t=nt)
    return self.out

  def arm(self, n=30, d=2.0, v=5.0, ask=0.0, **kw):
    """Feed n frames of a close lead and return the last ask (the floor arms inside this)."""
    return self.push(n, d=d, v=v, ask=ask, **kw)


class TestAsksAndGeometry(OpenpilotTestCase):
  def test_entry_exit_recover_bands_are_ordered(self):
    self.assertAlmostEqual(mg.enter_gap(5.0), 3.0)
    self.assertAlmostEqual(mg.enter_gap(8.0), 4.0)
    self.assertAlmostEqual(mg.exit_gap(5.0), 5.0)
    self.assertAlmostEqual(mg.exit_gap(8.0), 6.4)
    self.assertAlmostEqual(mg.recover_gap(3.0), 6.0)
    self.assertAlmostEqual(mg.recover_gap(8.0), 8.0)
    for v in np.arange(0.0, 15.0, 0.25):
      self.assertGreater(mg.exit_gap(v), mg.enter_gap(v))
      self.assertGreaterEqual(mg.recover_gap(v), mg.exit_gap(v))

  def test_hold_ask_is_the_car_layers_own_coast_point(self):
    """The hold ask is computed from the IMPORTED car-layer coast curve, sits HOLD_COAST_MARGIN on the pedal side of it
    and is never positive."""
    for v in np.arange(0.0, 15.01, 0.05):
      self.assertLessEqual(HOLD(v), 0.0)
      if COAST(v) > mg.HOLD_COAST_MARGIN:
        self.assertAlmostEqual(HOLD(v), -(COAST(v) - mg.HOLD_COAST_MARGIN), places=12)

  def test_floor_ask_never_raises_in_any_state(self):
    for v in (0.6, 2.0, 5.0, 9.5):
      for a in np.arange(-3.5, 2.01, 0.05):
        self.assertEqual(mg.floor_ask(float(a), mg.OFF, v, COAST), float(a))
        self.assertEqual(mg.floor_ask(float(a), mg.BRAKE, v, COAST), min(float(a), mg.A_FLOOR))
        for s in (mg.HOLD, mg.BRIDGE):
          self.assertEqual(mg.floor_ask(float(a), s, v, COAST), min(float(a), HOLD(v)))
          self.assertLessEqual(mg.floor_ask(float(a), s, v, COAST), float(a))


class TestHoldOnTheCarsOwnLaw(OpenpilotTestCase):
  """Review items 5/6 and R7: what the hold ask does in the car layer the car runs (the 0037 continuous law), and what it
  would do with that law off. actuators.accel == longitudinalPlan.aTarget on this car (kiV = [0], kp = 0, measured: 99.97 %
  of 14 072 pid frames equal one of the last two published plans), so the planner ask IS the car layer's input."""

  def test_hold_is_zero_brake_and_sub_count_pedal_for_any_nearby_car_layer_speed(self):
    for v in np.arange(0.5, 10.01, 0.05):
      need = -HOLD(v)
      for dv in (-0.1, 0.0, 0.1):                        # planner vEgo vs the 100 Hz car-layer vEgo
        vc = max(0.0, v + dv)
        self.assertEqual(fl.fca11_cont_brake_need(need, vc), 0.0, msg=f"v={v:.2f} dv={dv}")
        pedal = gi.get_hold_command(vc) * max(0.0, 1.0 - need / COAST(vc))
        self.assertLessEqual(pedal, gi.COAST_ZERO_EXIT_PEDAL, msg=f"v={v:.2f} dv={dv}")   # latched zero (from BRAKE)
      pedal0 = gi.get_hold_command(v) * max(0.0, 1.0 - need / COAST(v))
      self.assertLessEqual(pedal0, gi.COAST_ZERO_ENTER_PEDAL, msg=f"v={v:.2f}")         # even an unlatched zero

  def test_v2_minus_0_45_is_a_brake_on_the_continuous_law(self):
    """v2's A_HOLD = -0.45 assumed the OLD law's -0.5 onset. The continuous law has no onset: it brakes for any demand
    beyond coast(v), so -0.45 is a friction-brake demand across the whole band."""
    for v in np.arange(1.5, 10.01, 0.25):
      bn = fl.fca11_cont_brake_need(0.45, v)
      self.assertGreater(bn, 0.08, msg=f"v={v}")
      self.assertGreaterEqual(fl.fca11_cont_lsb(bn), 0.5, msg=f"v={v}")    # >= 1 LSB once a level is held

  def test_with_the_continuous_law_off_the_hold_is_a_live_pedal(self):
    """Why the dependency gate exists: without the continuous law the gas law is scale(v)*ask + hold(v), which is a live
    pedal at the hold ask (and at v2's -0.45) above a few m/s."""
    for v in np.arange(2.0, 10.01, 0.5):
      self.assertGreater(gi.get_pedal_command(HOLD(v), v), 0.0, msg=f"v={v}")
    for v in np.arange(4.0, 10.01, 0.5):
      self.assertGreater(gi.get_pedal_command(-0.45, v), 0.0, msg=f"v={v}")

  CAR_FLAGS = {"HyundaiGasInterceptor": True, "HyundaiFca11Brake": True, "HyundaiFca11AffineGain": True,
               "HyundaiFca11DynamicBrakes": True}          # the car's configuration (its carParamsSP, 2026-11-19)
  LAW_OFF_FLAGS = ({"HyundaiGasInterceptor": True, "HyundaiFca11Brake": True, "HyundaiFca11AffineGain": True},
                   {"HyundaiGasInterceptor": True, "HyundaiFca11Brake": True, "HyundaiFca11DynamicBrakes": True},
                   {"HyundaiGasInterceptor": True})

  @staticmethod
  def _car_ci(flags):
    """The real opendbc CarInterface for the Elantra 2022 non-SCC with the pedal, built the way setup_interfaces builds
    it from the params (opendbc's own test fingerprint; same recipe as its test_fca11_continuous._elantra_ci)."""
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
    setup_interfaces(CarInterface, CP, CP_SP, [{k: v} for k, v in flags.items()])
    prev_env = os.environ.get("FCA11_LOG_DIR")
    os.environ["FCA11_LOG_DIR"] = "0"
    try:
      return CarInterface(CP, CP_SP)
    finally:
      if prev_env is None:
        os.environ.pop("FCA11_LOG_DIR", None)
      else:
        os.environ["FCA11_LOG_DIR"] = prev_env

  def _drive_real_car_layer(self, v, asks, a_ego=-0.6, flags=None):
    """Drive the real car layer at 100 Hz; returns the interface and per-frame (gas, pedal ENABLE bit, brake level)."""
    from opendbc.sunnypilot.car.hyundai.tests.test_fca11_continuous import _cc, _enable_bit
    from opendbc.sunnypilot.car.hyundai.tests.test_gas_interceptor import _Fca11CS, _fca11_cam_frame
    CI = self._car_ci(self.CAR_FLAGS if flags is None else flags)
    cs = _Fca11CS(v_ego=v, cam=(0, _fca11_cam_frame(alive=5)))
    cs.out.aEgo = a_ego
    out = []
    for f, ask in enumerate(asks):
      sends = CI.CC.create_gas_command(_cc(ask), cs, f, 0)
      pedal = [d for a, d, _s in sends if a in (0x200, 0x700)]
      out.append((float(CI.CC.gas), _enable_bit(pedal[0]) if pedal else None, int(CI.CC.fca11_brake.brake_level)))
    return CI, out

  def test_law_status_matches_the_real_car_interface_for_each_configuration(self):
    for flags in (self.CAR_FLAGS,) + self.LAW_OFF_FLAGS:
      CI = self._car_ci(flags)
      b = CI.CC.fca11_brake
      want = mg.DEP_OK if (b.enabled and b.cont_law) else mg.DEP_LAW_OFF
      self.assertEqual(mg.law_status(CI.CP_SP, fl), want, msg=str(flags))
      self.assertEqual(want == mg.DEP_OK, flags is self.CAR_FLAGS, msg=str(flags))

  def test_on_the_real_car_layer_the_hold_releases_the_brake_and_zeroes_the_pedal(self):
    """2 s of the floor's BRAKE (-1.0), then 3 s of the hold, at 5 m/s: the brake level walks to 0 and stays there, and
    every pedal frame of the last 2 s is the all-zero ENABLE=0 frame."""
    for v in (2.0, 5.0, 9.0):
      CI, out = self._drive_real_car_layer(v, [-1.0] * 200 + [HOLD(v)] * 300)
      self.assertTrue(CI.CC.fca11_brake.cont_law)
      self.assertGreater(max(lvl for _g, _e, lvl in out[:200]), 0, msg=f"v={v}: the BRAKE ask must brake")
      tail = out[300:]
      self.assertEqual({lvl for _g, _e, lvl in tail}, {0}, msg=f"v={v}: the hold must not brake")
      self.assertEqual({g for g, _e, _l in tail}, {0.0}, msg=f"v={v}: the hold must command no pedal")
      self.assertEqual({e for _g, e, _l in tail if e is not None}, {0}, msg=f"v={v}: ENABLE must be 0")

  def test_on_the_real_car_layer_minus_0_45_keeps_braking(self):
    """The v2 defect, on the real class: after the same 2 s brake, -0.45 holds a brake level >= 1 at 5 m/s, and in the
    stop band (< 5 km/h) the law's creep floor (7 LSB) takes over."""
    _CI, out = self._drive_real_car_layer(5.0, [-1.0] * 200 + [-0.45] * 300)
    self.assertGreaterEqual(min(lvl for _g, _e, lvl in out[300:]), 1)
    _CI, out = self._drive_real_car_layer(0.8, [-1.0] * 200 + [-0.45] * 300)
    self.assertGreaterEqual(min(lvl for _g, _e, lvl in out[300:]), 5)
    _CI, out = self._drive_real_car_layer(0.8, [-1.0] * 200 + [HOLD(0.8)] * 300)
    self.assertEqual({lvl for _g, _e, lvl in out[300:]}, {0})

  def test_with_the_law_off_the_real_gas_path_is_live_at_the_hold(self):
    """Every other FCA11 configuration (no DynamicBrakes / no Shaping / FCA11 off): the hold ask is a LIVE pedal, i.e.
    not zero throttle - which is why the floor refuses to arm there."""
    for flags in self.LAW_OFF_FLAGS:
      _CI, out = self._drive_real_car_layer(5.0, [0.0] * 100 + [HOLD(5.0)] * 200, flags=flags)
      self.assertGreater(min(g for g, _e, _l in out[150:]), 0.0, msg=str(flags))
      self.assertEqual({e for _g, e, _l in out[150:] if e is not None}, {1}, msg=str(flags))


class TestDependencyGate(OpenpilotTestCase):
  FLAGS = ("brake", "pedal", "affine", "dynamic", "shadow")

  def _combos(self):
    for i in range(32):
      yield {k: bool(i >> j & 1) for j, k in enumerate(self.FLAGS)}

  def test_law_status_matches_the_real_fca11_class_for_every_flag_combination(self):
    """law_status reads the CarParamsSP fields the car layer reads with the car layer's own helpers; pinned here against
    the REAL Fca11LongBrake (enabled and cont_law), for both the dataclass and the capnp reader plannerd uses."""
    prev = os.environ.get("FCA11_LOG_DIR")
    os.environ["FCA11_LOG_DIR"] = "0"
    try:
      n_ok = 0
      for kw in self._combos():
        b = fl.Fca11LongBrake(structs.CarParams(), law_cp_sp(**kw))
        want = mg.DEP_OK if (b.enabled and b.cont_law) else mg.DEP_LAW_OFF
        self.assertEqual(mg.law_status(law_cp_sp(**kw), fl), want, msg=str(kw))
        self.assertEqual(mg.law_status(capnp_cp_sp(**kw), fl), want, msg=str(kw))
        n_ok += want == mg.DEP_OK
      self.assertEqual(n_ok, 1)                    # only brake + pedal + affine + dynamic, shadow off
    finally:
      if prev is None:
        os.environ.pop("FCA11_LOG_DIR", None)
      else:
        os.environ["FCA11_LOG_DIR"] = prev

  def test_law_off_never_arms(self):
    for kw in ({"affine": False}, {"dynamic": False}, {"brake": False}, {"pedal": False}, {"shadow": True}):
      f = make(cp_sp=law_cp_sp(**kw))
      c = Ctx(f)
      self.assertFalse(f.enabled, msg=str(kw))
      self.assertEqual(f.dependency, mg.DEP_LAW_OFF, msg=str(kw))
      self.assertEqual(c.arm(60, d=1.5, ask=0.3), 0.3, msg=str(kw))
      self.assertEqual(f.state, mg.OFF)

  def test_missing_inputs_fail_closed(self):
    self.assertEqual(mg.MinGapFloor(params=FakeParams(True), dt=DT, cp_sp=None).dependency, mg.DEP_NO_CP_SP)
    f = mg.MinGapFloor(params=FakeParams(True), dt=DT, cp_sp=law_cp_sp(), car_layer=None)
    self.assertEqual(f.dependency, mg.DEP_NO_CAR_LAYER)
    self.assertFalse(f.enabled)

    class OldSchema:           # a CarParamsSP that predates a field the law reads: a real pycapnp-style AttributeError
      fca11Brake = enableGasInterceptor = fca11AffineGain = fca11DynamicBrakes = True

      def __getattr__(self, k):
        raise AttributeError(f"struct has no such member; name = {k}")
    f = make(cp_sp=OldSchema())
    self.assertEqual(f.dependency, mg.DEP_CP_SP_UNREADABLE)
    self.assertFalse(f.enabled)

  def test_the_published_carparams_sp_is_rechecked_and_a_change_disables_the_floor(self):
    """card.py rewrites CarParamsSP when it (re)starts; the floor re-reads it once a second and follows the car layer's
    CURRENT state (a card restart with the law off must not leave the floor armed on a stale CP_SP)."""
    p = FakeParams(True, cp_sp_raw=capnp_cp_sp_bytes())
    f = make(params=p)
    c = Ctx(f)
    self.assertEqual(f.dependency, mg.DEP_OK)
    self.assertEqual(c.arm(30, d=2.0), mg.A_FLOOR)
    p.cp_sp_raw = capnp_cp_sp_bytes(dynamic=False)          # the car layer now runs without the continuous law
    out = c.push(int(round(mg.POLL_S / DT)) + 1, d=2.0, ask=0.4)
    self.assertEqual(out, 0.4)
    self.assertFalse(f.enabled)
    self.assertEqual(f.dependency, mg.DEP_LAW_OFF)
    self.assertEqual(f.state, mg.OFF)
    p.cp_sp_raw = b"\x00junk"                               # unreadable: refused, not assumed
    c.push(int(round(mg.POLL_S / DT)) + 1)
    self.assertEqual(f.dependency, mg.DEP_CP_SP_UNREADABLE)
    p.cp_sp_raw = capnp_cp_sp_bytes()                       # back: re-armed from a COLD start (no stale BRAKE)
    c.push(int(round(mg.POLL_S / DT)) + 1, d=9.0)
    self.assertEqual(f.dependency, mg.DEP_OK)
    self.assertEqual(f.state, mg.OFF)


class TestOffAndFailsafe(OpenpilotTestCase):
  def test_off_is_byte_identical(self):
    f = make(enabled=False)
    rng = np.random.default_rng(7)
    ask = 0.0
    for _ in range(500):
      ask = float(np.clip(ask + rng.normal(0.0, 0.2), -3.5, 2.0))
      v = float(np.clip(5.0 + rng.normal(0.0, 1.0), 0.0, 12.0))
      d = float(np.clip(2.5 + rng.normal(0.0, 1.5), 0.3, 20.0))
      self.assertEqual(f.update(ask, v, True, False, True, d, 1.0, False), ask)
    self.assertEqual(f.state, mg.OFF)
    self.assertEqual(f.dependency, mg.DEP_PARAM_OFF)

  def test_unknown_key_stays_off(self):
    f = make(params=FakeParams(enabled=True, unknown=True))
    c = Ctx(f)
    self.assertEqual(c.arm(30, d=1.5, ask=0.4), 0.4)
    self.assertEqual(f.state, mg.OFF)

  def test_the_rule_defaults_to_an_untrusted_lead(self):
    """update() with no probability given must not arm (v2 defaulted lead_prob to a trusted 1.0)."""
    f = make()
    out = 0.0
    for _ in range(40):
      out = f.update(0.0, 5.0, True, False, True, 1.5)
    self.assertEqual(out, 0.0)
    self.assertEqual(f.state, mg.OFF)

  def test_nan_or_low_probability_is_untrusted(self):
    for prob in (float("nan"), mg.MIN_LEAD_PROB - 1e-6):
      f = make()
      c = Ctx(f)
      self.assertEqual(c.arm(40, d=1.5, prob=prob), 0.0, msg=f"prob={prob}")
      self.assertEqual(f.state, mg.OFF)

  def test_driver_gas_drops_an_active_brake_in_one_frame_and_starts_the_cooldown(self):
    f = make()
    c = Ctx(f)
    self.assertEqual(c.arm(30, d=2.0), mg.A_FLOOR)
    self.assertEqual(c.push(1, gas=True, ask=0.4), 0.4)       # the same frame passes the driver's ask through
    self.assertEqual(f.state, mg.OFF)
    self.assertGreater(f.rearm_t, mg.REARM_COOLDOWN_S - 2 * DT)
    self.assertEqual(c.push(20, gas=False, ask=0.1), 0.1)     # foot off, still close: no instant re-brake
    self.assertEqual(f.state, mg.OFF)

  def test_planner_reset_drops_an_active_brake_in_one_frame(self):
    f = make()
    c = Ctx(f)
    self.assertEqual(c.arm(30, d=1.5, active=False), 0.0)
    self.assertEqual(f.state, mg.OFF)
    self.assertEqual(c.arm(30, d=1.5, active=True), mg.A_FLOOR)
    self.assertEqual(c.push(1, active=False, ask=0.2), 0.2)
    self.assertEqual(f.state, mg.OFF)

  def test_standstill_is_off(self):
    f = make()
    c = Ctx(f)
    self.assertEqual(c.arm(30, d=1.2, v=1.8, stand=True), 0.0)
    self.assertEqual(f.state, mg.OFF)

  def test_above_v_max_never_frees_the_ask(self):
    f = make()
    c = Ctx(f)
    self.assertEqual(c.arm(30, d=2.0, v=9.5, ask=0.0), mg.A_FLOOR)
    self.assertEqual(c.push(1, v=10.5, ask=0.8), HOLD(10.5))
    self.assertEqual(f.state, mg.HOLD)
    self.assertEqual(c.push(20, v=10.5, ask=0.8), HOLD(10.5))

  def test_hold_ends_at_a_standstill(self):
    f = make()
    c = Ctx(f)
    c.arm(30, d=2.0, v=5.0)
    c.push(1, v=0.9)
    self.assertEqual(f.state, mg.HOLD)
    self.assertEqual(c.push(1, v=0.4, d=2.0, ask=0.5), 0.5)
    self.assertEqual(f.state, mg.OFF)
    self.assertAlmostEqual(f.launch_hold, mg.LAUNCH_GRACE_S)

  def test_turning_the_param_off_mid_brake_frees_the_ask_and_resets(self):
    p = FakeParams(True)
    f = make(params=p)
    c = Ctx(f)
    self.assertEqual(c.arm(30, d=2.0), mg.A_FLOOR)
    p.enabled = False
    out = c.push(int(round(mg.POLL_S / DT)) + 1, ask=0.3)
    self.assertEqual(out, 0.3)
    self.assertEqual(f.state, mg.OFF)
    self.assertEqual(len(f._buf), 0)


class TestArming(OpenpilotTestCase):
  def test_no_entry_below_v_min(self):
    for v in (0.0, 0.4, 1.0, 1.49):
      f = make()
      c = Ctx(f)
      self.assertEqual(c.arm(40, d=1.0, v=v), 0.0, msg=f"v={v}")
      self.assertEqual(f.state, mg.OFF)

  def test_launch_from_a_short_queue_stop_is_not_braked(self):
    f = make()
    c = Ctx(f)
    self.assertEqual(c.push(20, d=2.5, v=0.0, stand=True), 0.0)
    self.assertAlmostEqual(f.launch_hold, mg.LAUNCH_GRACE_S)
    for i in range(39):
      v = min(3.0, 0.15 * (i + 1))
      self.assertEqual(c.push(1, d=2.5, v=v), 0.0, msg=f"frame {i} v={v:.2f}")

  def test_crawl_below_one_then_launch_is_not_braked(self):
    f = make()
    c = Ctx(f)
    self.assertEqual(c.push(40, d=2.6, v=0.8), 0.0)
    self.assertAlmostEqual(f.launch_hold, mg.LAUNCH_GRACE_S)
    for i in range(32):
      v = 0.8 + 2.2 * (i + 1) / 32.0
      d = 2.6 + 1.4 * (i + 1) / 32.0
      self.assertEqual(c.push(1, d=d, v=v), 0.0, msg=f"frame {i} v={v:.2f} d={d:.2f}")

  def test_the_grace_does_not_mask_a_real_close_approach(self):
    f = make()
    c = Ctx(f)
    self.assertEqual(c.push(60, d=5.5, v=3.3), 0.0)
    self.assertAlmostEqual(f.launch_hold, 0.0)
    out = c.push(10, d=1.77, v=3.3)                            # the 49:14 shape
    self.assertEqual(f.state, mg.BRAKE)
    self.assertEqual(out, mg.A_FLOOR)

  def test_one_noisy_first_frame_cannot_arm(self):
    f = make()
    c = Ctx(f)
    self.assertEqual(c.push(1, d=1.0, v=5.0), 0.0)
    self.assertEqual(f.state, mg.OFF)
    self.assertEqual(c.push(mg.MIN_SAMPLES - 2, d=4.5, v=5.0), 0.0)
    self.assertEqual(c.push(5, d=1.2, v=5.0), mg.A_FLOOR)

  def test_the_real_59_40_collapse_arms_in_a_quarter_second(self):
    f = make()
    c = Ctx(f, timed=True)
    c.push(40, d=6.0, v=5.5)
    outs = [c.push(1, d=d, v=5.5) for _t, d in GAP_COLLAPSE_59_40]
    self.assertEqual(outs[-1], mg.A_FLOOR)
    self.assertEqual(f.state, mg.BRAKE)


class TestReleaseDwell(OpenpilotTestCase):
  def _armed(self):
    f = make()
    c = Ctx(f)
    self.assertEqual(c.arm(30, d=2.0, v=5.0), mg.A_FLOOR)
    self.assertEqual(f.state, mg.BRAKE)
    return f, c

  def _to_exit(self, f, c, target=5.6, n=30):
    """Ramp the gap up until the MEDIAN is above the exit band (5.0 m at 5 m/s). 1.5 s, so the whole test stays inside
    the 5 s BRAKE budget (the duration cap has its own tests)."""
    for i in range(n):
      c.push(1, d=float(np.interp(i, [0, n - 1], [2.0, target])))
      if f.gap_med >= mg.exit_gap(5.0):
        return
    self.fail("the median never crossed the exit band")

  def test_release_requires_one_second_above_the_exit_band(self):
    f, c = self._armed()
    self._to_exit(f, c)
    self.assertEqual(f.state, mg.BRAKE)
    e0 = f.exit_t
    k = int(round((mg.EXIT_DWELL_S - e0) / DT)) - 1
    self.assertGreaterEqual(k, 15)
    for n in range(k):
      self.assertEqual(c.push(1, d=5.6), mg.A_FLOOR, msg=f"frame {n}")
    self.assertEqual(c.push(1, d=5.6), HOLD(5.0))              # releases into the hold, not into the ask
    self.assertEqual(f.state, mg.HOLD)

  def test_the_real_range_triplet_cannot_release(self):
    f, c = self._armed()
    for d in RANGE_TRIPLET:
      self.assertEqual(c.push(1, d=d), mg.A_FLOOR)
    self.assertEqual(c.push(20, d=3.2), mg.A_FLOOR)
    self.assertEqual(f.state, mg.BRAKE)

  def test_a_dip_back_inside_restarts_the_dwell(self):
    f, c = self._armed()
    self._to_exit(f, c)
    c.push(10, d=5.6)
    self.assertGreater(f.exit_t, 0.2)
    c.push(10, d=3.5)
    self.assertEqual(f.exit_t, 0.0)
    self.assertEqual(c.push(1, d=5.6), mg.A_FLOOR)

  def test_v_below_one_enters_the_hold_not_the_ask(self):
    f, c = self._armed()
    self.assertEqual(c.push(1, v=0.9, d=2.0, ask=0.6), HOLD(0.9))
    self.assertEqual(f.state, mg.HOLD)

  def test_the_hold_never_returns_a_positive_ask(self):
    f, c = self._armed()
    self._to_exit(f, c)
    n = 0
    while f.state == mg.BRAKE:
      c.push(1, d=5.6, ask=1.2)
      n += 1
      self.assertLess(n, 40)
    for _ in range(20):
      self.assertLessEqual(c.push(1, d=5.6, ask=1.2), 0.0)
      self.assertEqual(f.state, mg.HOLD)

  def test_hold_ends_only_after_the_recovery_dwell(self):
    f, c = self._armed()
    self._to_exit(f, c)
    while f.state == mg.BRAKE:
      c.push(1, d=5.6, ask=1.2)
    for i in range(60):
      c.push(1, d=float(np.interp(i, [0, 59], [5.6, 6.3])), ask=1.2)
      if f.gap_med >= mg.recover_gap(5.0):
        break
    self.assertEqual(f.state, mg.HOLD)
    outs = []
    while f.state == mg.HOLD:
      outs.append(c.push(1, d=6.3, ask=1.2))
      self.assertLess(len(outs), 60)
    self.assertGreaterEqual(len(outs) - 1, 15)
    for o in outs[:-1]:
      self.assertEqual(o, HOLD(5.0))
    self.assertEqual(outs[-1], 1.2)
    self.assertEqual(f.state, mg.OFF)
    self.assertEqual(f.brake_t, 0.0)                           # the approach is over: the budget is reset


class TestPostReleaseHold(OpenpilotTestCase):
  def _released(self):
    f = make()
    c = Ctx(f)
    c.arm(30, d=2.0, v=5.0)
    for i in range(80):
      c.push(1, d=float(np.interp(i, [0, 39], [2.0, 5.6])))
      if f.state != mg.BRAKE:
        break
    self.assertEqual(f.state, mg.HOLD)
    return f, c

  def test_rearm_is_blocked_for_five_seconds(self):
    f, c = self._released()
    t_release = f.rearm_t
    out = c.push(40, d=1.6, ask=1.0)
    self.assertEqual(f.state, mg.HOLD)
    self.assertEqual(out, HOLD(5.0))
    out = c.push(int(round(f.rearm_t / DT)) + 2, d=1.6, ask=1.0)
    self.assertEqual(f.state, mg.BRAKE)
    self.assertEqual(out, mg.A_FLOOR)
    self.assertAlmostEqual(t_release, mg.REARM_COOLDOWN_S, delta=1.5)

  def test_a_hold_below_v_min_never_re_brakes(self):
    """v2 re-entered BRAKE from HOLD without the speed band or the launch grace: at 0.8 m/s every 5 s a one-frame brake
    blip. The re-entry now needs the same conditions as an arm from OFF."""
    f, c = self._released()
    for _ in range(int(round(12.0 / DT))):
      self.assertLessEqual(c.push(1, d=1.5, v=0.8, ask=0.5), 0.0)
      self.assertNotEqual(f.state, mg.BRAKE)

  def test_no_limit_cycle_between_1_and_1_5(self):
    f = make()
    c = Ctx(f)
    c.push(40, d=3.0, v=1.4)
    arms, releases, prev = [], [], f.state
    for i in range(200):
      v = 1.6 if (i // 10) % 2 == 0 else 0.8
      c.push(1, d=2.0, v=v)
      if f.state == mg.BRAKE and prev != mg.BRAKE:
        arms.append(i)
      if f.state != mg.BRAKE and prev == mg.BRAKE:
        releases.append(i)
      prev = f.state
      self.assertLessEqual(c.out, 0.0)
    for a in arms[1:]:
      self.assertTrue(any(r <= a and (a - r) * DT >= mg.REARM_COOLDOWN_S - 1e-9 for r in releases),
                      msg=f"re-arm at frame {a} inside the cooldown")


class TestDropoutBridge(OpenpilotTestCase):
  def test_dropout_drops_the_brake_holds_zero_pedal_and_keeps_the_buffer(self):
    f = make()
    c = Ctx(f)
    c.arm(30, d=2.0, v=5.0)
    self.assertEqual(len(f._buf), mg.N_MED)
    out = c.push(10, present=False, ask=0.7)
    self.assertEqual(out, HOLD(5.0))                            # no brake on a vanished lead (zero pedal, zero brake)
    self.assertEqual(f.state, mg.BRIDGE)
    self.assertEqual(len(f._buf), mg.N_MED)
    out = c.push(1, present=True, d=2.1, ask=0.7)
    self.assertEqual(f.state, mg.BRAKE)
    self.assertEqual(out, mg.A_FLOOR)

  def test_bridge_expires_into_off(self):
    f = make()
    c = Ctx(f)
    c.arm(30, d=2.0, v=5.0)
    c.push(int(round(mg.LEAD_BRIDGE_S / DT)) - 1, present=False, ask=0.7)
    self.assertEqual(f.state, mg.BRIDGE)
    self.assertEqual(c.push(2, present=False, ask=0.7), 0.7)
    self.assertEqual(f.state, mg.OFF)
    self.assertEqual(len(f._buf), 0)
    self.assertEqual(c.push(1, present=True, d=2.0, ask=0.0), 0.0)
    self.assertEqual(f.state, mg.OFF)

  def test_untrusted_lead_bridges_instead_of_braking(self):
    f = make()
    c = Ctx(f)
    c.arm(30, d=2.0, v=5.0)
    self.assertEqual(c.push(4, prob=mg.MIN_LEAD_PROB - 0.1, ask=0.7), HOLD(5.0))
    self.assertEqual(f.state, mg.BRIDGE)
    self.assertEqual(c.push(1, prob=1.0, d=2.0, ask=0.7), mg.A_FLOOR)

  def test_a_dropout_in_the_hold_cannot_re_brake(self):
    f = make()
    c = Ctx(f)
    c.arm(30, d=2.0, v=5.0)
    c.push(1, v=0.9)
    self.assertEqual(f.state, mg.HOLD)
    self.assertEqual(c.push(6, present=False), HOLD(0.9))
    self.assertEqual(f.state, mg.BRIDGE)
    self.assertEqual(c.push(1, present=True, d=2.0, ask=0.5), HOLD(0.9))
    self.assertEqual(f.state, mg.HOLD)
    for _ in range(40):
      self.assertLessEqual(c.push(1, d=2.0, ask=1.0), 0.0)

  def test_a_lead_that_returns_far_starts_the_cooldown(self):
    f = make()
    c = Ctx(f)
    c.arm(30, d=2.0, v=5.0)
    for i in range(40):
      c.push(1, d=float(np.interp(i, [0, 39], [2.0, 5.6])))
    self.assertEqual(f.state, mg.BRAKE)
    self.assertGreater(f.exit_t, 0.0)
    c.push(6, present=False)
    self.assertEqual(f.state, mg.BRIDGE)
    self.assertEqual(c.push(1, present=True, d=5.8, ask=1.0), HOLD(5.0))
    self.assertEqual(f.state, mg.HOLD)
    self.assertEqual(c.push(10, d=2.0, ask=1.0), HOLD(5.0))
    self.assertEqual(f.state, mg.HOLD)


class TestDurationCap(OpenpilotTestCase):
  """Review item 4: a SUSTAINED false brake. BRAKE is capped at BRAKE_MAX_S per approach, then the zero-pedal hold for at
  most CAP_HOLD_MAX_S, then OFF and latched while the phantom persists."""

  def _phantom(self, d_fn, n, v=9.0, present_fn=lambda i: True):
    f = make()
    c = Ctx(f, timed=True)
    outs, states = [], []
    for i in range(n):
      outs.append(c.push(1, d=d_fn(i), v=v, present=present_fn(i), ask=0.0))
      states.append(f.state)
    return f, outs, states

  def test_a_constant_2_5_m_phantom_at_9_mps_on_a_clear_road(self):
    f, outs, states = self._phantom(lambda i: 2.5, int(round(60.0 / DT)))
    brake = sum(1 for o in outs if o == mg.A_FLOOR)
    self.assertEqual(brake, int(round(mg.BRAKE_MAX_S / DT)))     # exactly the budget, once
    first = outs.index(mg.A_FLOOR)
    last = len(outs) - 1 - outs[::-1].index(mg.A_FLOOR)
    self.assertEqual(last - first + 1, brake)                     # one contiguous brake
    hold = [o for o, s in zip(outs, states, strict=True) if s == mg.HOLD]
    self.assertEqual(len(hold), int(round(mg.CAP_HOLD_MAX_S / DT)))
    self.assertTrue(all(o == HOLD(9.0) for o in hold))            # zero pedal, zero brake
    tail = outs[last + 1 + len(hold):]
    self.assertGreater(len(tail), 600)
    self.assertTrue(all(o == 0.0 for o in tail))                  # and it STAYS off while the phantom persists
    self.assertTrue(f.cap_latched)

  def test_a_noisy_persistent_phantom_is_capped_too(self):
    rng = np.random.default_rng(3)
    noise = rng.normal(0.0, 0.4, 2000)
    _f, outs, _s = self._phantom(lambda i: 2.4 + float(noise[i]), 1200)
    self.assertEqual(sum(1 for o in outs if o == mg.A_FLOOR), int(round(mg.BRAKE_MAX_S / DT)))
    self.assertTrue(all(o == 0.0 for o in outs[-400:]))

  def test_a_blinking_phantom_cannot_buy_a_fresh_budget(self):
    """Present 4.0 s, absent 1.2 s (> the bridge), repeated: the budget is cumulative across dropouts."""
    period = int(round(5.2 / DT))
    _f, outs, _s = self._phantom(lambda i: 2.5, int(round(60.0 / DT)), present_fn=lambda i: (i % period) < 80)
    self.assertLessEqual(sum(1 for o in outs if o == mg.A_FLOOR), int(round(mg.BRAKE_MAX_S / DT)))
    self.assertTrue(all(o >= 0.0 for o in outs[-300:]))

  def test_the_latch_clears_on_a_real_recovery_and_a_new_approach_brakes(self):
    f, outs, _s = self._phantom(lambda i: 2.5, int(round(15.0 / DT)))
    self.assertTrue(f.cap_latched)
    c = Ctx(f, timed=True)
    c.now = 1000.0
    c.push(int(round(2.0 / DT)), d=12.0, v=9.0)                  # the gap really opens beyond the recovery band
    self.assertFalse(f.cap_latched)
    self.assertEqual(c.push(int(round(1.0 / DT)), d=2.0, v=9.0), mg.A_FLOOR)


class TestStaleRadarState(OpenpilotTestCase):
  def test_a_frozen_radar_state_ends_the_brake_and_stays_off(self):
    f = make()
    c = Ctx(f, timed=True)
    self.assertEqual(c.arm(30, d=2.0), mg.A_FLOOR)
    outs = [c.push(1, frozen=True, ask=0.1) for _ in range(int(round(20.0 / DT)))]
    n_trusted = int(STALE_FRAMES)                               # still fresh: <= STALE_RADAR_S old
    self.assertTrue(all(o == mg.A_FLOOR for o in outs[:n_trusted - 2]))
    bridge_end = next(i for i, o in enumerate(outs) if o == 0.1)
    self.assertLessEqual(bridge_end, n_trusted + int(round(mg.LEAD_BRIDGE_S / DT)) + 1)
    self.assertTrue(all(o == HOLD(5.0) for o in outs[n_trusted + 1:bridge_end]))   # no brake once stale
    self.assertTrue(all(o == 0.1 for o in outs[bridge_end:]))   # off, and stays off while frozen
    self.assertEqual(len(f._buf), 0)

  def test_samples_are_counted_per_new_message_not_per_planner_frame(self):
    f = make()
    now = 50.0
    for i in range(16):                                        # a new radarState only every second planner frame
      now += DT
      rt = 50.0 + DT * (i // 2) * 2
      f.update(0.0, 5.0, True, False, True, 9.0, 1.0, False, radar_t=rt, radar_md_t=rt, now_t=now)
    self.assertEqual(len(f._buf), 8)

  def test_a_radar_state_computed_from_an_old_model_frame_is_stale(self):
    f = make()
    now = 10.0
    for _ in range(40):
      now += DT
      out = f.update(0.0, 5.0, True, False, True, 1.5, 1.0, False, radar_t=now - DT, radar_md_t=now - 1.0, now_t=now)
    self.assertEqual(out, 0.0)
    self.assertEqual(f.state, mg.OFF)


STALE_FRAMES = round(mg.STALE_RADAR_S / DT)


class TestFlicker(OpenpilotTestCase):
  def test_1_2_s_out_0_5_s_back_repeated(self):
    """Review item 7: two dropout-expiry re-arms inside FLICKER_WINDOW_S make the next expiry start the cooldown, and the
    cumulative budget bounds the train's total brake."""
    f = make()
    c = Ctx(f, timed=True)
    c.arm(30, d=2.0, v=5.0)
    self.assertEqual(f.state, mg.BRAKE)
    arms, prev, brake = [f.t], f.state, 0
    n_out, n_back = int(round(1.2 / DT)), int(round(0.5 / DT))
    for _ in range(int(round(60.0 / 1.7))):
      for present, n in ((False, n_out), (True, n_back)):
        for _i in range(n):
          out = c.push(1, present=present, d=2.0, ask=0.3)
          brake += out == mg.A_FLOOR
          if f.state == mg.BRAKE and prev != mg.BRAKE and present:
            arms.append(f.t)
          prev = f.state
    gaps = [b - a for a, b in zip(arms, arms[1:], strict=False)]
    self.assertLessEqual(sum(1 for g in gaps if g < mg.REARM_COOLDOWN_S - 1e-9), mg.FLICKER_REARMS)
    self.assertLessEqual(len(arms), 1 + mg.FLICKER_REARMS + int(60.0 / mg.REARM_COOLDOWN_S) + 1)
    self.assertLessEqual(brake * DT, mg.BRAKE_MAX_S + 1e-9)


# --- real capnp messages (the v1 bug class) -----------------------------------------------------------------------------
class LogSM(dict):
  """A SubMaster stand-in over REAL capnp readers: sm[service] and sm.logMonoTime[service]."""

  def __init__(self, **readers):
    super().__init__(**readers)
    self.logMonoTime = {}


def real_sm(t, d=2.0, prob=1.0, present=True, gas=False, stand=False, v=5.0, radar_t=None, enabled=True):
  rs = messaging.new_message('radarState')
  rs.logMonoTime = int(round((radar_t if radar_t is not None else t - DT) * 1e9))
  rs.radarState.mdMonoTime = rs.logMonoTime
  rs.radarState.leadOne.present = present
  rs.radarState.leadOne.dRel = d
  rs.radarState.leadOne.modelProb = prob
  cs = messaging.new_message('carState')
  cs.carState.vEgo, cs.carState.gasPressed, cs.carState.standstill = v, gas, stand
  ss = messaging.new_message('selfdriveState')
  ss.selfdriveState.enabled = enabled
  sm = LogSM(radarState=rs.radarState.as_reader(), carState=cs.carState.as_reader(),
             selfdriveState=ss.selfdriveState.as_reader())
  sm.logMonoTime = {'radarState': rs.logMonoTime, 'modelV2': int(round(t * 1e9)), 'carState': int(round(t * 1e9)),
                    'selfdriveState': int(round(t * 1e9))}
  return sm


class TestContainmentOnRealMessages(OpenpilotTestCase):
  def test_off_reads_no_message_field_at_all(self):
    """Review item 3: with the param OFF the hook must not touch the message (v1 read sm['radarState'].modelProb on every
    frame), and it must hand back the ask OBJECT itself (byte-identical by construction)."""
    f = make(enabled=False)

    class Tripwire(dict):
      def __getitem__(self, k):
        raise AssertionError("the floor read a message with the param OFF")

      def __getattr__(self, k):
        raise AssertionError("the floor read an SM attribute with the param OFF")
    ask = np.float64(0.37)
    for _ in range(50):
      self.assertIs(f.apply(Tripwire(), ask, 5.0, True), ask)
    self.assertEqual(f.state, mg.OFF)

  def test_dependency_unmet_reads_no_message_either(self):
    f = make(cp_sp=law_cp_sp(dynamic=False))

    class Tripwire(dict):
      def __getitem__(self, k):
        raise AssertionError("the floor read a message with the dependency unmet")
    self.assertEqual(f.apply(Tripwire(), 0.2, 5.0, True), 0.2)

  def test_the_hook_arms_on_real_capnp_readers_and_gas_drops_it_in_one_frame(self):
    f = make()
    t = 20.0
    out = 0.0
    for _ in range(30):
      t += DT
      out = f.apply(real_sm(t, d=2.0), 0.0, 5.0, True)
    self.assertEqual(out, mg.A_FLOOR)
    self.assertEqual(f.state, mg.BRAKE)
    t += DT
    self.assertEqual(f.apply(real_sm(t, d=2.0, gas=True), 0.6, 5.0, True), 0.6)
    self.assertEqual(f.state, mg.OFF)
    self.assertFalse(f.faulted)

  def test_a_struct_without_the_field_latches_the_floor_off_and_passes_the_ask(self):
    """Review item 2/3 with the REAL pycapnp error: hand the hook a radarState that is really a carState reader, so
    `.leadOne` raises `AttributeError: struct has no such member` (v1's failure class). The ask passes through unmodified
    on that frame and on every later frame: the floor is latched off for the drive."""
    f = make()
    sm = real_sm(30.0)
    sm['radarState'] = messaging.new_message('carState').carState.as_reader()
    ask = np.float64(0.21)
    self.assertIs(f.apply(sm, ask, 5.0, True), ask)
    self.assertTrue(f.faulted)
    self.assertFalse(f.enabled)
    self.assertEqual(f.dependency, mg.DEP_FAULTED)
    t = 31.0
    for _ in range(60):
      t += DT
      self.assertEqual(f.apply(real_sm(t, d=1.0), 0.42, 5.0, True), 0.42)
    self.assertEqual(f.state, mg.OFF)

  def test_a_lead_without_model_prob_is_never_trusted(self):
    """A LeadData without modelProb (a schema mismatch) must raise inside the hook and latch off - never read 1.0."""
    class LeadNoProb:
      present, dRel = True, 1.0

      @property
      def modelProb(self):
        raise AttributeError("struct has no such member; name = modelProb")

    class Radar:
      leadOne = LeadNoProb()
      mdMonoTime = int(29.95e9)
    f = make()
    sm = real_sm(30.0)
    sm['radarState'] = Radar()
    self.assertEqual(f.apply(sm, 0.1, 5.0, True), 0.1)
    self.assertTrue(f.faulted)

  def test_a_frozen_real_radar_state_bridges_then_frees_the_ask(self):
    f = make()
    t = 40.0
    for _ in range(30):
      t += DT
      f.apply(real_sm(t, d=2.0), 0.0, 5.0, True)
    self.assertEqual(f.state, mg.BRAKE)
    frozen_t = t - DT
    outs = []
    for _ in range(80):
      t += DT
      outs.append(f.apply(real_sm(t, d=2.0, radar_t=frozen_t), 0.3, 5.0, True))
    self.assertEqual(outs[-1], 0.3)
    self.assertEqual(f.state, mg.OFF)
    self.assertNotIn(mg.A_FLOOR, outs[STALE_FRAMES + 1:])


# --- REAL driver gas presses, one code per planner frame exactly as plannerd's SubMaster saw them (gasPressed from carState,
# planner reset = controlsState.longControlState == off, engaged = selfdriveState.enabled; scan: wsSTOP/drive-2026-11-19/
# sim/mingap/fix2/scripts/press_catalog.py). Codes:  '.' engaged, planner active, no gas   'g' engaged, planner ACTIVE, gas
#   'G' engaged, planner RESET, gas   'r' engaged, planner RESET, no gas   'D' NOT engaged (a real disengage)
REAL_PRESS_RESET_FIRST = 'G' * 44 + 'r'     # 000001ac--0fd72a8378--65 @3919.76: reset on the 1st gas frame, 1-frame lag
REAL_PRESS_ACTIVE_FIRST = 'g' + 'G' * 8 + 'r'  # 000001ac--0fd72a8378--36 @2214.75: planner still active on frame 1, lag 1
REAL_RESET_BEFORE_GAS = 'r' + 'G' * 95      # 000001ac--0fd72a8378--65 @3924.46: the reset arrives a frame BEFORE the gas
REAL_ISOLATED_RESET = 'r'                    # 000001ac--0fd72a8378--67 @4078.96: a reset with no gas frame sampled at all
REAL_LONG_PRESS = 'G' * 118 + 'r'            # 000001ac--0fd72a8378--100 @6012.97: a 5.9 s press, 1-frame lag
REAL_LAG_2 = 'G' * 18 + 'rr'                 # 00000199--a5acf5c6e9--1 @91.21: a two-frame release lag
_SIG = {'.': (False, True, True), 'g': (True, True, True), 'G': (True, False, True), 'r': (False, False, True),
        'D': (False, False, False)}          # code -> (gasPressed, planner active = not reset_state, selfdriveState.enabled)
COOLDOWN_FRAMES = int(round(mg.REARM_COOLDOWN_S / DT))


class HookCtx:
  """Drives the REAL hook ``apply()`` with REAL capnp readers, one planner frame per code: ``active`` is what the planner
  passes (``not reset_state``); gasPressed and selfdriveState.enabled are in the messages. Fresh radarState every frame."""

  def __init__(self, f, t0=200.0):
    self.f, self.t = f, t0

  def run(self, codes, d=2.0, v=5.0, ask=0.0, present=True):
    outs = []
    for code in codes:
      gas, active, enabled = _SIG[code]
      self.t += DT
      outs.append(self.f.apply(real_sm(self.t, d=d, present=present, gas=gas, v=v, enabled=enabled), ask, v, active))
    return outs


class TestOverrideVsDisengage(OpenpilotTestCase):
  """Opus review finding 1: on this car a driver gas press reaches the floor as a PLANNER RESET (gasPressedOverride ->
  longActive False -> longControlState off), usually on the first gas frame, sometimes a frame before the gas, and the
  release can lag 1-2 frames. v2.1 read every reset as the end of the approach: no cooldown, budget and cap latch wiped.
  Every test here feeds a REAL signal order through the real hook; the ones marked RED-v2.1 fail on ac1e439c03."""

  def _armed(self, d=2.0, v=5.0):
    f = make()
    c = HookCtx(f)
    self.assertEqual(c.run('.' * 30, d=d, v=v)[-1], mg.A_FLOOR)
    return f, c

  def test_every_real_order_drops_the_brake_on_its_first_frame_and_the_cooldown_runs_from_the_release(self):
    """RED-v2.1: re-braked on the first frame after the release (reset-first / reset-before-gas / isolated reset / lag 2)
    or 4.55 s after it (active-first: its cooldown started at the press, not at the release)."""
    for seq in (REAL_PRESS_RESET_FIRST, REAL_PRESS_ACTIVE_FIRST, REAL_RESET_BEFORE_GAS, REAL_ISOLATED_RESET, REAL_LAG_2,
                REAL_LONG_PRESS):
      with self.subTest(seq=f"{seq[:3]}..{len(seq)}"):
        f, c = self._armed()
        press = c.run(seq, d=2.0, ask=0.6)                    # the lead stays 2 m away the whole time
        self.assertTrue(all(o == 0.6 for o in press))         # the driver's ask from the FIRST press frame, never -1.0
        self.assertEqual(f.state, mg.OFF)
        after = c.run('.' * (COOLDOWN_FRAMES + 40), d=2.0, ask=0.1)
        self.assertIn(mg.A_FLOOR, after)                      # the floor still works afterwards (budget left) ...
        first = after.index(mg.A_FLOOR)
        self.assertGreaterEqual(first, COOLDOWN_FRAMES - 1)   # ... but not within 5 s of the last override frame
        self.assertLessEqual(first, COOLDOWN_FRAMES + 1)

  def test_a_6_s_press_after_a_capped_phantom_keeps_the_cap_latch(self):
    """RED-v2.1: the press cleared the latch and the budget; on release the floor braked a fresh 5 s at -1.0."""
    f = make()
    c = HookCtx(f)
    outs = c.run('.' * int(round(12.0 / DT)), d=2.5, v=9.0)    # a constant 2.5 m phantom at 9 m/s on a clear road
    self.assertEqual(sum(o == mg.A_FLOOR for o in outs), int(round(mg.BRAKE_MAX_S / DT)))
    self.assertTrue(f.cap_latched)
    self.assertEqual(f.state, mg.OFF)
    press = c.run(REAL_LONG_PRESS, d=2.5, v=9.0, ask=0.6)      # the driver presses the gas for 5.9 s; the phantom stays
    self.assertTrue(all(o == 0.6 for o in press))
    self.assertTrue(f.cap_latched)
    self.assertAlmostEqual(f.brake_t, mg.BRAKE_MAX_S)
    after = c.run('.' * int(round(30.0 / DT)), d=2.5, v=9.0, ask=0.1)
    self.assertTrue(all(o == 0.1 for o in after))              # still latched 30 s later: not one brake frame
    self.assertTrue(f.cap_latched)

  def test_a_press_during_the_cap_hold_keeps_the_latch(self):
    """RED-v2.1: as above, with the press inside the 5 s zero-pedal hold that follows the cap."""
    f = make()
    c = HookCtx(f)
    c.run('.' * int(round(7.0 / DT)), d=2.5, v=9.0)
    self.assertEqual(f.state, mg.HOLD)
    self.assertTrue(f.cap_latched)
    self.assertTrue(all(o == 0.6 for o in c.run(REAL_PRESS_RESET_FIRST, d=2.5, v=9.0, ask=0.6)))
    self.assertTrue(f.cap_latched)
    self.assertNotIn(mg.A_FLOOR, c.run('.' * int(round(30.0 / DT)), d=2.5, v=9.0, ask=0.1))

  def test_an_override_on_the_frame_the_budget_runs_out_still_latches(self):
    """RED-v2.1. The cap latches on the BRAKE frame after the budget is spent; an override landing exactly there must latch
    too (otherwise the 10 s budget-forget would hand a persisting phantom a fresh 5 s)."""
    f = make()
    c = HookCtx(f)
    outs = []
    while sum(o == mg.A_FLOOR for o in outs) < int(round(mg.BRAKE_MAX_S / DT)):
      outs += c.run('.', d=2.5, v=9.0)
    self.assertEqual(f.state, mg.BRAKE)
    self.assertFalse(f.cap_latched)
    c.run(REAL_PRESS_RESET_FIRST, d=2.5, v=9.0, ask=0.6)
    self.assertTrue(f.cap_latched)
    self.assertNotIn(mg.A_FLOOR, c.run('.' * int(round(30.0 / DT)), d=2.5, v=9.0, ask=0.1))

  def test_a_press_during_the_hold_runs_the_cooldown_from_the_release(self):
    """RED-v2.1: the hold's own cooldown (started at the release into HOLD) ran out during the press, so the floor braked
    on the first frame after it."""
    f, c = self._armed()
    c.run('.' * 25, d=5.6)                                     # the gap opens past the exit band for 1 s: release, HOLD
    self.assertEqual(f.state, mg.HOLD)
    c.run('.' * 60, d=2.0)                                     # 3 s of HOLD while the gap closes again
    self.assertEqual(f.state, mg.HOLD)
    self.assertTrue(all(o == 0.6 for o in c.run(REAL_PRESS_RESET_FIRST, d=2.0, ask=0.6)))
    after = c.run('.' * (COOLDOWN_FRAMES + 40), d=2.0, ask=0.1)
    self.assertGreaterEqual(after.index(mg.A_FLOOR), COOLDOWN_FRAMES - 1)

  def test_a_press_during_the_bridge_keeps_the_brake_budget(self):
    """RED-v2.1 (a fresh 5 s after the press). One approach gets ONE 5 s budget, a gas press in the middle included."""
    f = make()
    c = HookCtx(f)
    braked = sum(o == mg.A_FLOOR for o in c.run('.' * 64, d=2.0))          # ~3 s of brake
    c.run('.' * 6, d=2.0, present=False)                                   # the lead drops out: BRIDGE
    self.assertEqual(f.state, mg.BRIDGE)
    budget = f.brake_t
    self.assertTrue(all(o == 0.6 for o in c.run(REAL_PRESS_ACTIVE_FIRST, d=2.0, ask=0.6)))
    self.assertAlmostEqual(f.brake_t, budget)
    braked += sum(o == mg.A_FLOOR for o in c.run('.' * int(round(20.0 / DT)), d=2.0, ask=0.1))
    self.assertEqual(braked, int(round(mg.BRAKE_MAX_S / DT)))
    self.assertTrue(f.cap_latched)

  def test_gas_never_brakes_while_pressed(self):
    f, c = self._armed()
    for seq in (REAL_LONG_PRESS, 'g' * 40, 'G' * 40):
      self.assertTrue(all(o == 0.7 for o in c.run(seq, d=1.5, ask=0.7)))

  def test_a_real_disengage_ends_the_approach(self):
    """A real disengage may reset everything: after re-engaging, a new approach has a fresh budget (green on v2.1 too)."""
    f = make()
    c = HookCtx(f)
    c.run('.' * int(round(12.0 / DT)), d=2.5, v=9.0)
    self.assertTrue(f.cap_latched)
    self.assertTrue(all(o == 0.3 for o in c.run('D' * 20, d=2.5, v=9.0, ask=0.3)))
    self.assertFalse(f.cap_latched)
    self.assertEqual(f.brake_t, 0.0)
    again = c.run('.' * int(round(20.0 / DT)), d=2.5, v=9.0)    # re-engaged (any cooldown left from the cap hold runs out)
    self.assertEqual(sum(o == mg.A_FLOOR for o in again), int(round(mg.BRAKE_MAX_S / DT)))   # a fresh, full budget

  def test_a_disengage_during_a_press_ends_the_approach(self):
    """Real corpus order (00000196--9f739d721d--2 @148.96: G r G86 then disengaged): the disengage wins."""
    f = make()
    c = HookCtx(f)
    c.run('.' * int(round(12.0 / DT)), d=2.5, v=9.0)
    self.assertTrue(f.cap_latched)
    c.run('G' + 'r' + 'G' * 86 + 'D' * 84, d=2.5, v=9.0, ask=0.3)
    self.assertFalse(f.cap_latched)
    self.assertEqual(f.brake_t, 0.0)

  def test_engagement_start_up_frames_are_not_an_override(self):
    """Engaged with the planner not yet in control (2 such single frames in the 390-segment corpus): no cooldown, as
    before - the floor may arm as soon as the planner is in control (green on v2.1 too)."""
    f = make()
    c = HookCtx(f)
    c.run('D' * 20, d=2.0)
    c.run('rr', d=2.0)
    self.assertEqual(f.rearm_t, 0.0)
    self.assertEqual(c.run('.' * 10, d=2.0)[-1], mg.A_FLOOR)

  def test_override_diagnostic_and_the_update_api(self):
    """``override`` marks the frames read as a driver override; update(engaged=...) gives the same answers as the hook."""
    f, c = self._armed()
    c.run('G')
    self.assertTrue(f.override)
    c.run('.')
    self.assertFalse(f.override)
    g = make()
    for _ in range(30):
      g.update(0.0, 5.0, True, False, True, 2.0, 1.0, False)
    self.assertEqual(g.state, mg.BRAKE)
    self.assertEqual(g.update(0.6, 5.0, False, True, True, 2.0, 1.0, False, engaged=True), 0.6)
    self.assertTrue(g.override)
    self.assertGreater(g.rearm_t, mg.REARM_COOLDOWN_S - DT)
    self.assertEqual(g.update(0.6, 5.0, False, False, True, 2.0, 1.0, False, engaged=False), 0.6)
    self.assertFalse(g.override)

  def test_a_missing_self_drive_state_fails_closed(self):
    """The new read is inside the contained hook: no selfdriveState -> the floor latches off, the ask passes."""
    f = make()
    sm = real_sm(30.0)
    del sm['selfdriveState']
    self.assertEqual(f.apply(sm, 0.25, 5.0, True), 0.25)
    self.assertTrue(f.faulted)


class TestRealPlanner(OpenpilotTestCase):
  """The REAL LongitudinalPlanner on real capnp messages, floor OFF and ON (the in-tree miniature of the rlog replay)."""

  def _run(self, floor, n=120, d0=6.0, closing=0.6):
    from openpilot.sunnypilot.fork.tests.test_lead_closing import make_planner, planner_sm
    p = make_planner(v0=5.0)
    p.min_gap_floor = mg.MinGapFloor(params=FakeParams(enabled=floor), dt=DT, cp_sp=capnp_cp_sp())
    outs, states = [], []
    t = 0.0
    for i in range(n):
      t += DT
      msgs = planner_sm(5.0, 0.3, max(1.0, d0 - closing * i * DT), +3.0, v_cruise_kph=40.)  # vRel says PULLING AWAY
      sm = LogSM(**msgs)
      sm.logMonoTime = {k: int(round(t * 1e9)) for k in msgs}
      sm.logMonoTime['radarState'] = int(round((t - DT) * 1e9))
      msgs['radarState'].mdMonoTime = sm.logMonoTime['radarState']
      p.update(sm)
      outs.append(float(p.output_a_target))
      states.append(p.min_gap_floor.state)
    return p, outs, states

  def test_floor_off_and_on_through_the_real_planner(self):
    p_off, off, st_off = self._run(False)
    self.assertFalse(p_off.min_gap_floor.faulted)
    self.assertEqual(p_off.min_gap_floor.dependency, mg.DEP_PARAM_OFF)
    self.assertEqual(set(st_off), {mg.OFF})
    p_on, on, st_on = self._run(True)
    self.assertFalse(p_on.min_gap_floor.faulted)
    self.assertEqual(p_on.min_gap_floor.dependency, mg.DEP_OK)
    self.assertIn(mg.BRAKE, st_on)
    self.assertLessEqual(min(on), mg.A_FLOOR + 1e-9)            # the floor's brake reaches the published ask
    first = st_on.index(mg.BRAKE)
    self.assertEqual(on[:first], off[:first])                    # identical until the floor acts

  def test_a_gas_press_that_arrives_as_a_planner_reset_through_the_real_planner(self):
    """RED-v2.1. The real planner computes ``reset_state`` from controlsState.longControlState itself; feed the real order
    of REAL_PRESS_RESET_FIRST (gas + longControlState off, then one frame of longControlState off without gas) while the
    floor brakes, the lead staying close. The published ask must leave -1.0 on the first press frame and must not return
    to -1.0 within 5 s of the release."""
    from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
    from openpilot.sunnypilot.fork.tests.test_lead_closing import make_planner, planner_sm
    p = make_planner(v0=5.0)
    p.min_gap_floor = mg.MinGapFloor(params=FakeParams(enabled=True), dt=DT, cp_sp=capnp_cp_sp())
    codes = '.' * 40 + REAL_PRESS_RESET_FIRST + '.' * (COOLDOWN_FRAMES + 30)
    outs, states, t = [], [], 0.0
    for code in codes:
      gas, active, enabled = _SIG[code]
      t += DT
      msgs = planner_sm(5.0, 0.3, 2.0, +3.0, v_cruise_kph=40.)    # a 2 m gap that vision says is pulling away
      msgs['carState'].gasPressed = gas
      msgs['controlsState'].longControlState = LongCtrlState.pid if active else LongCtrlState.off
      msgs['selfdriveState'].enabled = enabled
      sm = LogSM(**msgs)
      sm.logMonoTime = {k: int(round(t * 1e9)) for k in msgs}
      sm.logMonoTime['radarState'] = int(round((t - DT) * 1e9))
      msgs['radarState'].mdMonoTime = sm.logMonoTime['radarState']
      p.update(sm)
      outs.append(float(p.output_a_target))
      states.append(p.min_gap_floor.state)
    self.assertFalse(p.min_gap_floor.faulted)
    i0 = 40
    i1 = i0 + len(REAL_PRESS_RESET_FIRST)
    self.assertEqual(states[i0 - 1], mg.BRAKE)
    self.assertLessEqual(outs[i0 - 1], mg.A_FLOOR + 1e-9)
    self.assertTrue(all(o > mg.A_FLOOR + 1e-6 for o in outs[i0:i1]))       # dropped on the first press frame
    after = states[i1:]
    first = after.index(mg.BRAKE)
    self.assertGreaterEqual(first, COOLDOWN_FRAMES - 1)                   # not within 5 s of the release
    self.assertTrue(all(o > mg.A_FLOOR + 1e-6 for o in outs[i1:i1 + first]))

  def test_real_lead_data_has_model_prob_and_radar_state_does_not(self):
    """The schema fact v1 got wrong, asserted on the real cereal."""
    self.assertIn('modelProb', log.RadarState.LeadData.schema.fieldnames)
    self.assertNotIn('modelProb', log.RadarState.schema.fieldnames)
    self.assertIn('mdMonoTime', log.RadarState.schema.fieldnames)


# --- 2026-11-21 follow-up (the Opus re-check's open items): the coast after the duration cap is bounded in WALL time, and
# the launch grace holds after a mid-drive enable. Every test from here to TestWiring is RED on 66ac205755, GREEN here.
# A REAL close-lead dropout train as the floor saw it, one code per planner frame ('u' usable, 'x' not: absent, untrusted
# or stale): 000001b0--0bfbcacdc4--52 @3155.19, a 1-frame and a 9-frame dropout 0.6 s apart (the longest close-lead train
# in that drive and route 000001ac; sim/mingap/coast-bound/scripts/scan_flicker.py).
REAL_DROPOUT_TRAIN = 'x' + 'u' * 11 + 'x' * 9 + 'uu'
# A REAL launch from a crawl, vEgo per planner frame: 000001ac--0fd72a8378--5, its last frame under 1 m/s (@334.48) is index
# 15 here (driven by hand there; replayed engaged behind a 2.5 m lead; sim/mingap/coast-bound/scripts/scan_launch.py).
REAL_CRAWL_LAUNCH_V = [0.405, 0.45, 0.493, 0.538, 0.575, 0.61, 0.641, 0.676, 0.708, 0.745, 0.781, 0.821, 0.857, 0.898, 0.945,
                       0.996, 1.057, 1.13, 1.206, 1.288, 1.366, 1.472, 1.514, 1.584, 1.649, 1.718, 1.794, 1.875, 1.951,
                       2.026, 2.098, 2.166, 2.225, 2.282, 2.336, 2.388, 2.472, 2.482, 2.527, 2.57, 2.614, 2.649, 2.678,
                       2.697, 2.708, 2.713, 2.712, 2.709, 2.706, 2.701, 2.698, 2.693, 2.688, 2.68, 2.678, 2.675, 2.671,
                       2.667, 2.665, 2.661, 2.659, 2.652, 2.65, 2.647, 2.645, 2.642, 2.64, 2.632, 2.626, 2.619, 2.615,
                       2.615, 2.615, 2.613, 2.607, 2.596, 2.585, 2.582, 2.581, 2.58]
REAL_CRAWL_LAST_SLOW = 15
BUDGET_FRAMES = int(round(mg.BRAKE_MAX_S / DT))
CAP_COAST_FRAMES = int(round(mg.CAP_HOLD_MAX_S / DT))
GRACE_FRAMES = int(round(mg.LAUNCH_GRACE_S / DT))
POLL_FRAMES = int(round(mg.POLL_S / DT))


def run_lead(c, pattern, d=2.5, v=9.0, ask=0.0):
  """The REAL hook, one planner frame per code: 'u' a usable lead at d, 'x' the lead gone (radarState present False)."""
  outs = []
  for ch in pattern:
    outs += c.run('.', d=d, v=v, ask=ask, present=(ch == 'u'))
  return outs


class TestCoastBoundInWallTime(OpenpilotTestCase):
  """After the 5 s brake cap the zero-pedal coast lasts CAP_HOLD_MAX_S of WALL time from the cap (a lead-dropout BRIDGE
  frame counts like a HOLD frame), then the ask passes through and the latch holds. RED on 66ac205755: only HOLD frames
  counted, so every dropout stretched the coast (19 frames out / 1 in: towards ~100 s), and a dropout on the very frame
  the budget ran out skipped the cap - an UNLATCHED hold with no time bound and, once its bridge expired, a second 5 s
  brake 10 s later on the same phantom."""

  def _coast(self, outs, v=9.0):
    """The frames whose ask the floor replaced by the coast ask (HOLD and BRIDGE publish the same zero-pedal ask)."""
    return [i for i, o in enumerate(outs) if o == HOLD(v)]

  def _assert_one_capped_approach(self, f, outs, ask=0.0):
    coast = self._coast(outs)
    last_brake = len(outs) - 1 - outs[::-1].index(mg.A_FLOOR)
    self.assertEqual(sum(o == mg.A_FLOOR for o in outs), BUDGET_FRAMES)  # ONE 5 s brake for the whole approach
    self.assertEqual(coast[0], last_brake + 1)                          # the coast starts on the frame after it ...
    self.assertEqual(len(coast), CAP_COAST_FRAMES)                      # ... lasts 5.0 s ...
    self.assertEqual(coast[-1] - coast[0] + 1, CAP_COAST_FRAMES)        # ... of WALL time (one contiguous stretch)
    self.assertTrue(all(o == ask for o in outs[coast[-1] + 1:]))        # then the ask passes through, untouched
    self.assertTrue(f.cap_latched)                                      # and the latch holds while the phantom stays

  def test_a_real_dropout_train_cannot_stretch_the_capped_coast(self):
    f = make()
    c = HookCtx(f)
    outs = c.run('.' * 130, d=2.5, v=9.0)                  # a constant 2.5 m phantom at 9 m/s: 5 s brake, cap, coast
    self.assertTrue(f.cap_latched)
    self.assertEqual(f.state, mg.HOLD)
    outs += run_lead(c, REAL_DROPOUT_TRAIN)                 # the real dropouts, 1.3 s into the coast
    outs += c.run('.' * int(round(30.0 / DT)), d=2.5, v=9.0)
    self._assert_one_capped_approach(f, outs)               # 66ac205755: 110 coast frames (5.5 s)

  def test_a_19_out_1_in_flicker_cannot_stretch_it_towards_100_s(self):
    f = make()
    c = HookCtx(f)
    outs = c.run('.' * 110, d=2.5, v=9.0)
    outs += run_lead(c, ('x' * 19 + 'u') * int(round(120.0 / DT / 20)))   # each dropout just inside the 1 s bridge
    self._assert_one_capped_approach(f, outs)               # 66ac205755: ~95 s of coast

  def test_a_dropout_on_the_frame_the_budget_runs_out_is_the_cap(self):
    f = make()
    c = HookCtx(f)
    outs = c.run('.' * 104, d=2.5, v=9.0)                  # arms on frame 4, brakes frames 4..103: the budget is spent
    self.assertEqual(sum(o == mg.A_FLOOR for o in outs), BUDGET_FRAMES)
    self.assertEqual(f.state, mg.BRAKE)
    outs += run_lead(c, REAL_DROPOUT_TRAIN[0])              # the lead is gone on exactly the frame the cap is due
    self.assertTrue(f.cap_latched)                          # that IS the cap (66ac205755: unlatched, no time bound)
    outs += c.run('.' * int(round(60.0 / DT)), d=2.5, v=9.0)   # the phantom then stays for a minute
    self._assert_one_capped_approach(f, outs)

  def test_a_long_dropout_on_that_frame_cannot_buy_a_second_5_s_brake(self):
    f = make()
    c = HookCtx(f)
    outs = c.run('.' * 104, d=2.5, v=9.0)
    outs += c.run('.' * 25, d=2.5, v=9.0, present=False)   # gone 1.25 s from that frame: the bridge expires -> OFF
    outs += c.run('.' * int(round(40.0 / DT)), d=2.5, v=9.0)   # back for 40 s (66ac205755: a fresh 5 s after 10 s OFF)
    self.assertEqual(sum(o == mg.A_FLOOR for o in outs), BUDGET_FRAMES)
    self.assertTrue(f.cap_latched)
    self.assertLessEqual(len(self._coast(outs)), CAP_COAST_FRAMES)


class TestEnableMidDrive(OpenpilotTestCase):
  """RED on 66ac205755 (the re-check fuzz's only two hits): the rule's clocks do not run while the floor is off, so the 2 s
  launch grace never saw a crawl that happened then; read ON by the 1 Hz poll just after the crawl, the floor armed
  0.45 s after the car crossed 1 m/s. A mid-drive enable now assumes the car was crawling up to the last frame the floor
  did not see: no arm within LAUNCH_GRACE_S of it."""

  def _replay(self, p, speeds, flip_at, flip, d=2.5):
    f = make(params=p)
    outs, t = [], 300.0
    for i, v in enumerate(speeds):
      if i == flip_at:
        flip()                    # switched on mid-drive; the hook only reads it at its next 1 Hz poll
      t += DT
      outs.append(f.apply(real_sm(t, d=d, v=v), 0.0, v, True))
    return f, outs

  def test_switched_on_during_a_real_crawl_the_launch_grace_still_holds(self):
    cases = (('the param', FakeParams(enabled=False), lambda p: setattr(p, 'enabled', True)),
             ('the car layer law', FakeParams(enabled=True, cp_sp_raw=capnp_cp_sp_bytes(dynamic=False)),
              lambda p: setattr(p, 'cp_sp_raw', capnp_cp_sp_bytes())))
    for name, p, on in cases:
      with self.subTest(enable=name):
        f, outs = self._replay(p, REAL_CRAWL_LAUNCH_V, REAL_CRAWL_LAST_SLOW - 5, flip=lambda p=p, on=on: on(p))
        self.assertTrue(f.enabled)
        first = outs.index(mg.A_FLOOR)                     # it does arm once the grace is over (the lead stays 2.5 m)
        self.assertGreaterEqual(first - REAL_CRAWL_LAST_SLOW, GRACE_FRAMES)   # 66ac205755: 9 frames after the crawl
        self.assertGreaterEqual(first - (POLL_FRAMES - 1), GRACE_FRAMES)      # 2 s after the last frame it did not see

  def test_switched_on_at_cruise_the_first_arm_waits_the_grace_too(self):
    """Conservative by design: the floor cannot know what happened while it was off, so ANY mid-drive enable starts the
    grace - here a steady 8 m/s behind a 3 m lead: the first brake 2 s after the enable, not 0.25 s (66ac205755)."""
    p = FakeParams(enabled=False)
    f, outs = self._replay(p, [8.0] * 120, 1, d=3.0, flip=lambda: setattr(p, 'enabled', True))
    self.assertGreaterEqual(outs.index(mg.A_FLOOR) - (POLL_FRAMES - 1), GRACE_FRAMES)


class TestWiring(OpenpilotTestCase):
  def test_module_constants(self):
    self.assertAlmostEqual(mg.A_FLOOR, -1.0)
    self.assertGreaterEqual(mg.REARM_COOLDOWN_S, 5.0)
    self.assertGreaterEqual(mg.EXIT_DWELL_S, 1.0)
    self.assertGreaterEqual(mg.RECOVER_DWELL_S, 1.0)
    self.assertGreaterEqual(mg.LEAD_BRIDGE_S, 1.0)
    self.assertGreaterEqual(mg.BRAKE_MAX_S, 4.0)              # the real brakes were 2.55 s and 3.75 s
    self.assertLessEqual(mg.BRAKE_MAX_S, 6.0)
    self.assertLessEqual(mg.STALE_RADAR_S, 0.5)
    self.assertGreater(mg.FLICKER_WINDOW_S, mg.REARM_COOLDOWN_S)
    self.assertGreater(mg.HOLD_COAST_MARGIN, 0.0)
    self.assertLessEqual(mg.HOLD_COAST_MARGIN, 0.0095)        # < one pedal count at every hold(v)/coast(v) in the band

  def test_the_constructor_never_raises(self):
    class Boom:
      def get_bool(self, k):
        raise RuntimeError("params exploded")

      def get(self, k):
        raise RuntimeError("params exploded")
    f = mg.MinGapFloor(params=Boom(), dt=DT, cp_sp=law_cp_sp())
    self.assertFalse(f.enabled)
    self.assertEqual(f.apply({}, 0.5, 5.0, True), 0.5)
