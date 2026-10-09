"""
Personality-indexed Hyundai pedal law (adurham fork): the driver's LongitudinalPersonality scales the launch ceiling,
the pull gain, the hold feedforward and the interceptor cap. STANDARD reproduces the shipped law bit-for-bit; relaxed
is gentler; aggressive bites harder but is clamped below the pedal firmware ceiling.

Companion to test_gas_interceptor.py (which pins the personality-independent law). This file is the patch-candidate
regression for the personality feature.
"""
import numpy as np
import pytest

from opendbc.can import CANPacker
from opendbc.car import DT_CTRL, structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.hyundai.values import CAR
from opendbc.sunnypilot.car import crc8_pedal, create_gas_interceptor_command
from opendbc.sunnypilot.car.hyundai import gas_interceptor as gi
from opendbc.sunnypilot.car.hyundai.values import HyundaiSafetyFlagsSP
from opendbc.sunnypilot.car.interfaces import setup_interfaces

AG, ST, RE = gi.PERSONALITY_AGGRESSIVE, gi.PERSONALITY_STANDARD, gi.PERSONALITY_RELAXED
PERSONALITIES = (AG, ST, RE)
CAR_UNDER_TEST = CAR.HYUNDAI_ELANTRA_2022_NON_SCC
BASE_FINGERPRINT = {0x260: 8, 0x371: 8, 0x386: 8, 0x394: 8, 0x251: 8, 0x4F1: 4, 0x340: 8}
STD = {0x201: 6}
STD_ADDR = 0x201  # STANDARD_IDS.sensor_addr: the plain comma pedal
MPH = 0.44704


def _cmd_fields(dat: bytes) -> tuple[int, int, int, int]:
  """-> (track A, track B, ENABLE, counter)"""
  return (dat[0] << 8) | dat[1], (dat[2] << 8) | dat[3], dat[4] >> 7, dat[4] & 0xF


class TestPersonalityScale:
  def test_tiers_ordered_standard_is_one(self):
    assert gi.PERSONALITY_SCALE_V == [1.20, 1.00, 0.85]
    assert gi.PERSONALITY_SCALE_BP == [AG, ST, RE]
    assert gi.personality_scale(RE) < gi.personality_scale(ST) == 1.0 < gi.personality_scale(AG)

  def test_aggressive_clamped_below_pedal_ceiling(self):
    # the aggressive feel factor is 1.20 but is clamped so a raw command can never exceed the 0.35 firmware ceiling
    assert gi.personality_scale(AG) == pytest.approx(gi.MAX_PERSONALITY_SCALE)
    assert gi.personality_scale(AG) * 0.30 <= gi.MAX_INTERCEPTOR_GAS + 1e-9

  def test_unknown_and_missing_fall_back_to_standard(self):
    for bad in (7, -1, 99, None, 'bogus'):
      assert gi.personality_scale(bad) == gi.personality_scale(ST), bad
    # a capnp _DynamicEnum-like object carrying .raw is accepted
    class _E:
      raw = AG
    assert gi.personality_scale(_E()) == gi.personality_scale(AG)


class TestPersonalityCeiling:
  def test_ceiling_table(self):
    # standard keeps the shipped 0.12/0.20/0.35; relaxed and aggressive offset the first two anchors only
    assert gi.get_personality_ceiling(0., ST) == pytest.approx(0.12)
    assert gi.get_personality_ceiling(5., ST) == pytest.approx(0.20)
    assert gi.get_personality_ceiling(0., RE) == pytest.approx(0.10)
    assert gi.get_personality_ceiling(5., RE) == pytest.approx(0.17)
    assert gi.get_personality_ceiling(0., AG) == pytest.approx(0.16)
    assert gi.get_personality_ceiling(5., AG) == pytest.approx(0.26)

  def test_driver_intent_brackets(self):
    # owner's older backlog intent (relaxed ~10 % / standard ~12 % / aggressive ~15-18 % at a standstill) and the
    # drive-149 "next lever" (aggressive 0.16/0.26) both landed inside these brackets
    assert 0.09 <= gi.get_personality_ceiling(0., RE) <= 0.12
    assert gi.get_personality_ceiling(0., ST) == pytest.approx(0.12)
    assert 0.15 <= gi.get_personality_ceiling(0., AG) <= 0.18

  def test_full_cap_by_25mph_every_personality(self):
    # the 0.35 cap by 25 mph must STAY for all tiers (only the sub-25 mph anchors move)
    for p in PERSONALITIES:
      assert gi.get_personality_ceiling(25. * MPH, p) == pytest.approx(gi.MAX_INTERCEPTOR_GAS)
      assert gi.get_personality_ceiling(25., p) == pytest.approx(gi.MAX_INTERCEPTOR_GAS)

  def test_monotonic_and_never_exceeds_cap(self):
    for v in np.linspace(0., 40., 161):
      c = [gi.get_personality_ceiling(float(v), p) for p in (RE, ST, AG)]
      assert c[0] <= c[1] <= c[2] + 1e-12, v
      assert all(x <= gi.MAX_INTERCEPTOR_GAS + 1e-12 for x in c), v


class TestStandardIsShippedLaw:
  """A drive that never touches the personality button (standard) must be byte-copied from before this feature."""
  @staticmethod
  def _shipped_ceiling(v):
    return min(gi.MAX_INTERCEPTOR_GAS, float(np.interp(v, gi.LOW_SPEED_MAX_GAS_BP, gi.LOW_SPEED_MAX_GAS_V)))

  def test_ceiling_identical(self):
    for v in np.linspace(0., 40., 81):
      assert gi.get_personality_ceiling(float(v), ST) == pytest.approx(self._shipped_ceiling(float(v)), abs=1e-12)

  def test_scale_hold_command_identical(self):
    for v in np.linspace(0., 40., 81):
      assert gi.get_pedal_scale(float(v), ST) == pytest.approx(float(np.interp(v, gi.PEDAL_SCALE_BP, gi.PEDAL_SCALE_V)))
      assert gi.get_hold_command(float(v), ST) == pytest.approx(float(np.interp(v, gi.HOLD_CMD_BP, gi.HOLD_CMD_V)))

  def test_pedal_command_identical(self):
    for v in np.linspace(0., 40., 81):
      for a in np.linspace(-2., 3., 21):
        shipped = float(np.clip(gi.personality_scale(ST) * (float(np.interp(v, gi.PEDAL_SCALE_BP, gi.PEDAL_SCALE_V)) * a +
                                                            float(np.interp(v, gi.HOLD_CMD_BP, gi.HOLD_CMD_V))),
                               0., self._shipped_ceiling(float(v))))
        assert gi.get_pedal_command(float(a), float(v), ST) == pytest.approx(shipped, abs=1e-12)

  def test_default_personality_is_standard(self):
    # a direct call with no personality (sim / tests / any caller that forgets) is STANDARD, never aggressive
    for v in (0., 5., 11.176, 20., 30.):
      assert gi.get_pedal_command(1.0, v) == pytest.approx(gi.get_pedal_command(1.0, v, ST))
      assert gi.get_personality_ceiling(v) == pytest.approx(gi.get_personality_ceiling(v, ST))

  def test_rate_limit_is_not_personality_scaled(self):
    # the rise limit is the same comfort jerk for every tier: get_pedal_rate_up takes no personality and is unchanged
    for v in np.linspace(0., 40., 41):
      assert gi.get_pedal_rate_up(float(v)) / float(np.interp(v, gi.PEDAL_SCALE_BP, gi.PEDAL_SCALE_V)) == pytest.approx(gi.PEDAL_JERK_UP)


class TestPersonalityPedalCommand:
  def test_order_and_cap(self):
    for v in np.linspace(0., 40., 81):
      for a in (0.3, 0.5, 1.0, 2.0, 5.0):
        c = [gi.get_pedal_command(a, float(v), p) for p in (RE, ST, AG)]
        assert c[0] <= c[1] + 1e-12 <= c[2] + 2e-12, (v, a)
        assert all(0. <= x <= gi.MAX_INTERCEPTOR_GAS + 1e-12 for x in c), (v, a)

  def test_aggressive_bites_harder_on_a_launch(self):
    # the drive-149 launch case: a strong request at low speed is ceiling-bound; aggressive delivers more pedal
    assert gi.get_pedal_command(5., 0., AG) == pytest.approx(0.16)
    assert gi.get_pedal_command(5., 0., RE) == pytest.approx(0.10)
    assert gi.get_pedal_command(5., 0., ST) == pytest.approx(0.12)
    assert gi.get_pedal_command(5., 5., AG) == pytest.approx(0.26)

  def test_accel_only_every_personality(self):
    for p in PERSONALITIES:
      for v in (0., 5., 11.2, 20., 35.):
        for a in (-4., -1., -0.5, 0., 0.5, 2., 10.):
          assert 0. <= gi.get_pedal_command(a, v, p) <= gi.MAX_INTERCEPTOR_GAS
        assert gi.get_pedal_command(-2., v, p) == 0.

  def test_gain_scaled_on_highway_pull(self):
    # a sub-ceiling highway request scales by the personality gain: aggressive > standard > relaxed
    v, a = 25., 0.4
    assert gi.get_pedal_command(a, v, AG) > gi.get_pedal_command(a, v, ST) > gi.get_pedal_command(a, v, RE)


def _get_ci(fingerprint_bus0: dict):
  fingerprint = {i: {} for i in range(8)}
  fingerprint[0] = dict(fingerprint_bus0)
  CarInterface = interfaces[CAR_UNDER_TEST]
  CP = CarInterface.get_params(CAR_UNDER_TEST, fingerprint, [], alpha_long=False, is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, CAR_UNDER_TEST, fingerprint, [], alpha_long=False, is_release_sp=False, docs=False)
  setup_interfaces(CarInterface, CP, CP_SP, [{"HyundaiGasInterceptor": True}])
  return CarInterface, CP, CP_SP


class TestPersonalityPlumbing:
  """CC_SP.personality must reach the pedal law through the real CarController -> CarInterface.apply path."""

  def _wire_command(self, personality: int, v_ego: float, accel: float, n: int = 200) -> int:
    CarInterface, CP, CP_SP = _get_ci({**BASE_FINGERPRINT, **STD})
    CI = CarInterface(CP, CP_SP)
    CC = structs.CarControl()
    CC.enabled = CC.latActive = CC.longActive = True
    CC.actuators.accel = accel
    CC_SP = structs.CarControlSP()
    CC_SP.personality = personality
    t, last_a = 0, None
    for i in range(n):
      t += int(DT_CTRL * 1e9)
      dat = bytearray(6)
      dat[4] = i & 0xF  # STATE 0, counter
      dat[5] = crc8_pedal(dat[:5])
      CI.update([(t, [(STD_ADDR, bytes(dat), 0)])])
      CI.CS.out.vEgo = v_ego
      _, sends = CI.apply(CC.as_reader(), CC_SP, t)
      cmds = [d for a, d, _ in sends if a == gi.STANDARD_IDS.command_addr]
      if cmds:
        last_a = _cmd_fields(cmds[-1])[0]
    return last_a

  def test_personality_reaches_the_law(self):
    # at 0 m/s the launch ceiling binds a strong request: the wire command must differ by personality
    c = {p: self._wire_command(p, 0., 5.0) for p in PERSONALITIES}
    assert c[RE] < c[ST] < c[AG], c
    # and match the law's ceiling packed to raw counts
    for p in PERSONALITIES:
      _, dc, _ = create_gas_interceptor_command(CANPacker(gi.GAS_INTERCEPTOR_DBC), gi.get_personality_ceiling(0., p), 0)
      assert abs(c[p] - ((dc[0] << 8) | dc[1])) <= 2, (p, c)
