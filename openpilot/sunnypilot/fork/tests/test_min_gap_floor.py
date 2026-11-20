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
The real-message replay of the REAL planner over rlogs (red on v1 @ 1604d4a250, green here) is the sim harness
~/comma-sim/wsSTOP/drive-2026-11-19/sim/mingap/v2fix/scripts/replay_real.py; TestRealPlanner is its in-tree miniature.
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


def real_sm(t, d=2.0, prob=1.0, present=True, gas=False, stand=False, v=5.0, radar_t=None):
  rs = messaging.new_message('radarState')
  rs.logMonoTime = int(round((radar_t if radar_t is not None else t - DT) * 1e9))
  rs.radarState.mdMonoTime = rs.logMonoTime
  rs.radarState.leadOne.present = present
  rs.radarState.leadOne.dRel = d
  rs.radarState.leadOne.modelProb = prob
  cs = messaging.new_message('carState')
  cs.carState.vEgo, cs.carState.gasPressed, cs.carState.standstill = v, gas, stand
  sm = LogSM(radarState=rs.radarState.as_reader(), carState=cs.carState.as_reader())
  sm.logMonoTime = {'radarState': rs.logMonoTime, 'modelV2': int(round(t * 1e9)), 'carState': int(round(t * 1e9))}
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

  def test_real_lead_data_has_model_prob_and_radar_state_does_not(self):
    """The schema fact v1 got wrong, asserted on the real cereal."""
    self.assertIn('modelProb', log.RadarState.LeadData.schema.fieldnames)
    self.assertNotIn('modelProb', log.RadarState.schema.fieldnames)
    self.assertIn('mdMonoTime', log.RadarState.schema.fieldnames)


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
