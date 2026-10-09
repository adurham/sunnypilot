"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Hyundai comma pedal (gas interceptor): DBC/packer lock, threshold trio, and the detect + opt-in enablement contract.
"""
import copy
import os
import re

import numpy as np
import pytest

from opendbc.can import CANDefine, CANPacker
from opendbc.can.dbc import DBC
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.car_helpers import interfaces
from opendbc.car.hyundai.values import CAR
from opendbc.sunnypilot.car import crc8_pedal, create_gas_interceptor_command
from opendbc.sunnypilot.car.hyundai import gas_interceptor as gi
from opendbc.sunnypilot.car.hyundai import pause_resume as pr
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP, HyundaiSafetyFlagsSP
from opendbc.sunnypilot.car.interfaces import setup_interfaces

CAR_UNDER_TEST = CAR.HYUNDAI_ELANTRA_2022_NON_SCC
HYUNDAI_H = os.path.join(os.path.dirname(__file__), "../../../../safety/modes/hyundai.h")
HYUNDAI_COMMON_H = os.path.join(os.path.dirname(__file__), "../../../../safety/modes/hyundai_common.h")

# A frame set roughly like the real car's bus 0, enough for the NON_SCC platform; pedal adds 0x201
BASE_FINGERPRINT = {0x260: 8, 0x371: 8, 0x386: 8, 0x394: 8, 0x251: 8, 0x4F1: 4, 0x340: 8}
# pedal sensor frames as seen in the fingerprint: standard comma pedal / remapped custom firmware (+0x500)
STD = {0x201: 6}
REM = {0x701: 6}
DETECT_FLAGS = HyundaiFlagsSP.GAS_INTERCEPTOR_DETECTED.value | HyundaiFlagsSP.GAS_INTERCEPTOR_REMAPPED_DETECTED.value
PEDAL_SP = HyundaiSafetyFlagsSP.GAS_INTERCEPTOR | HyundaiSafetyFlagsSP.GAS_INTERCEPTOR_REMAPPED


def _c_define(path: str, name: str) -> int:
  with open(path) as f:
    m = re.search(rf"#define\s+{name}\s+(\d+)", f.read())
  assert m is not None, f"{name} not found in {path}"
  return int(m.group(1))


def _sig_layout(msg):
  return {n: (s.start_bit, s.size, s.is_signed, s.factor, s.offset, s.is_little_endian) for n, s in msg.sigs.items()}


class TestDBC:
  def test_pedal_dbc_addresses(self):
    dbc = DBC(gi.GAS_INTERCEPTOR_DBC)
    assert dbc.addr_to_msg[gi.GAS_COMMAND_ADDR].name == "GAS_COMMAND"
    assert dbc.addr_to_msg[gi.GAS_SENSOR_ADDR].name == "GAS_SENSOR"
    # remapped pedal firmware IDs (+0x500)
    assert dbc.addr_to_msg[1792].name == "GAS_COMMAND_R" and dbc.addr_to_msg[1793].name == "GAS_SENSOR_R"
    assert set(dbc.name_to_msg) == {"GAS_COMMAND", "GAS_SENSOR", "GAS_COMMAND_R", "GAS_SENSOR_R"}
    for ids in (gi.STANDARD_IDS, gi.REMAPPED_IDS):
      assert dbc.name_to_msg[ids.command_msg].address == ids.command_addr
      assert dbc.name_to_msg[ids.sensor_msg].address == ids.sensor_addr

  def test_remapped_signals_identical(self):
    # the remapped messages must be byte-for-byte the same layout as the standard ones
    dbc = DBC(gi.GAS_INTERCEPTOR_DBC)
    for std, rem in (("GAS_COMMAND", "GAS_COMMAND_R"), ("GAS_SENSOR", "GAS_SENSOR_R")):
      a, b = dbc.name_to_msg[std], dbc.name_to_msg[rem]
      assert a.size == b.size == 6
      assert _sig_layout(a) == _sig_layout(b)
    assert CANDefine(gi.GAS_INTERCEPTOR_DBC).dv["GAS_SENSOR_R"]["STATE"] == CANDefine(gi.GAS_INTERCEPTOR_DBC).dv["GAS_SENSOR"]["STATE"]

  def test_hyundai_can_dbc_has_no_remapped_ids(self):
    # 0x700/0x701 must stay free in the main Hyundai DBC (otherwise the separate-DBC rationale would apply again)
    dbc = DBC("hyundai_can_generated")
    assert 0x700 not in dbc.addr_to_msg and 0x701 not in dbc.addr_to_msg

  def test_hyundai_can_dbc_untouched(self):
    # the pedal messages must NOT be in hyundai_can: BO_ 512 is EMS20 there, and CANDefine binds VAL_ lines by address,
    # so a second BO_ 512 would silently break FCEV gear parsing (dv["EMS20"]) and/or the pedal packer
    dbc = DBC("hyundai_can_generated")
    assert dbc.addr_to_msg[0x200].name == "EMS20"
    assert "GAS_COMMAND" not in dbc.name_to_msg and "GAS_SENSOR" not in dbc.name_to_msg
    assert "HYDROGEN_GEAR_SHIFTER" in CANDefine("hyundai_can_generated").dv["EMS20"]

  def test_gas_command_packs(self):
    packer = CANPacker(gi.GAS_INTERCEPTOR_DBC)
    addr, dat, bus = create_gas_interceptor_command(packer, 0.1, 5)
    assert (addr, bus) == (0x200, 0)
    assert len(dat) == 6 and any(dat)
    assert dat[4] & 0xF == 5 and dat[4] >> 7 == 1  # counter, ENABLE
    assert dat[5] == crc8_pedal(dat[:5])           # pedal accepts only crc8(0xD5) frames

  def test_remapped_gas_command_packs(self):
    packer = CANPacker(gi.GAS_INTERCEPTOR_DBC)
    addr, dat, bus = create_gas_interceptor_command(packer, 0.1, 5, gi.REMAPPED_IDS.command_msg)
    assert (addr, bus) == (0x700, 0)
    assert len(dat) == 6 and any(dat)
    assert dat[4] & 0xF == 5 and dat[4] >> 7 == 1
    assert dat[5] == crc8_pedal(dat[:5])
    # identical payload to the standard dialect
    assert dat == create_gas_interceptor_command(packer, 0.1, 5)[1]

  def test_zero_command_disables(self):
    addr, dat, _ = create_gas_interceptor_command(CANPacker(gi.GAS_INTERCEPTOR_DBC), 0., 0)
    assert dat[:4] == b"\x00\x00\x00\x00" and dat[4] >> 7 == 0


def _sensor_frame(ids, gas: int, counter: int):
  dat = bytearray(6)
  dat[0:2] = gas.to_bytes(2, "big")
  dat[2:4] = gas.to_bytes(2, "big")
  dat[4] = counter & 0xF  # STATE 0
  dat[5] = crc8_pedal(dat[:5])
  return ids.sensor_addr, bytes(dat)


class TestCarStateDialect:
  @pytest.mark.parametrize("pedal, ids", [(STD, gi.STANDARD_IDS), (REM, gi.REMAPPED_IDS)], ids=["standard", "remapped"])
  def test_gas_pressed_from_active_sensor(self, pedal, ids):
    # end to end through CarInterface.update -> CarState -> CarStateExt (the real gasPressed path)
    CarInterface, CP, CP_SP = _get_ci({**BASE_FINGERPRINT, **pedal}, True)
    CI = CarInterface(CP, CP_SP)
    other = gi.REMAPPED_IDS if ids is gi.STANDARD_IDS else gi.STANDARD_IDS

    t = 0

    def feed(frames):
      nonlocal t
      t += int(0.02e9)
      return CI.update([(t, [(addr, dat, 0) for addr, dat in frames])])[0]

    # the other dialect's sensor reporting "pressed" is ignored; the active one says released
    for i in range(10):
      ret = feed([_sensor_frame(other, 0x1000, i), _sensor_frame(ids, 0, i)])
    assert not ret.gasPressed
    for i in range(10):
      ret = feed([_sensor_frame(ids, 0x1000, i)])
    assert ret.gasPressed
    # and the reverse: active released, other pressed -> released
    for i in range(10):
      ret = feed([_sensor_frame(ids, 0, i), _sensor_frame(other, 0x1000, i)])
    assert not ret.gasPressed


class TestThresholds:
  def test_c_and_python_threshold_match(self):
    assert _c_define(HYUNDAI_H, "HYUNDAI_GAS_INTERCEPTOR_THRESHOLD") == gi.HYUNDAI_GAS_INTERCEPTOR_THRESHOLD

  def test_c_and_python_safety_bit_match(self):
    with open(HYUNDAI_COMMON_H) as f:
      m = re.search(r"HYUNDAI_PARAM_SP_GAS_INTERCEPTOR\s*=\s*(\d+)", f.read())
    assert m is not None and int(m.group(1)) == HyundaiSafetyFlagsSP.GAS_INTERCEPTOR
    with open(HYUNDAI_COMMON_H) as f:
      m = re.search(r"HYUNDAI_PARAM_SP_GAS_INTERCEPTOR_REMAPPED\s*=\s*(\d+)", f.read())
    assert m is not None and int(m.group(1)) == HyundaiSafetyFlagsSP.GAS_INTERCEPTOR_REMAPPED
    # free in the SP space (and a power of two), and within the Int16 safetyParam field
    for name in ("GAS_INTERCEPTOR", "GAS_INTERCEPTOR_REMAPPED"):
      bit = getattr(HyundaiSafetyFlagsSP, name)
      others = [v for k, v in vars(HyundaiSafetyFlagsSP).items() if k.isupper() and k != name]
      assert bit not in others and bin(bit).count("1") == 1 and bit < 2 ** 15

  def test_capability_flags_unique(self):
    vals = [f.value for f in HyundaiFlagsSP]
    assert len(vals) == len(set(vals))
    assert HyundaiFlagsSP.GAS_INTERCEPTOR_REMAPPED_DETECTED.value == 2 ** 12

  def test_max_gas_raw_ceiling(self):
    # Pins the command scaling in raw DAC counts at the cap so it can't drift silently through a DBC/scale change.
    # Bench-measured scaling: 0.35 -> track A 1218, track B 612 (0.30 was A 1110 / B 559, 0.15 was A 788 / B 400).
    _, dat, _ = create_gas_interceptor_command(CANPacker(gi.GAS_INTERCEPTOR_DBC), gi.MAX_INTERCEPTOR_GAS, 0)
    track1, track2 = (dat[0] << 8) | dat[1], (dat[2] << 8) | dat[3]
    assert abs(track1 - 1218) <= 2 and abs(track2 - 612) <= 3
    assert abs(track2 - (0.49413 * track1 + 10.45)) <= 3  # the cap pair sits on the car's measured A/B track line
    assert abs((track1 - 465) / (2616 - 465) - 0.35) < 0.005  # 35 % of pedal travel on track A
    assert 0. < gi.MAX_INTERCEPTOR_GAS <= 0.35

  @pytest.mark.parametrize("pedal_ids", [gi.STANDARD_IDS, gi.REMAPPED_IDS], ids=["standard", "remapped"])
  def test_c_max_gas_matches_python_cap(self, pedal_ids):
    # panda's raw ceiling must be exactly the packed command at openpilot's cap: lower would cut throttle exactly when
    # it is pinned at the cap (e.g. on a climb), higher would let panda pass more than openpilot ever sends
    _, dat, _ = create_gas_interceptor_command(CANPacker(gi.GAS_INTERCEPTOR_DBC), gi.MAX_INTERCEPTOR_GAS, 0,
                                               pedal_ids.command_msg)
    assert _c_define(HYUNDAI_H, "HYUNDAI_GAS_INTERCEPTOR_MAX_GAS_A") == (dat[0] << 8) | dat[1]
    assert _c_define(HYUNDAI_H, "HYUNDAI_GAS_INTERCEPTOR_MAX_GAS_B") == (dat[2] << 8) | dat[3]

  @pytest.mark.parametrize("pedal_ids", [gi.STANDARD_IDS, gi.REMAPPED_IDS], ids=["standard", "remapped"])
  def test_dbc_mapping_matches_bench_measurement(self, pedal_ids):
    # Bench-measured on the owner's car (2026-10-03): track B = 0.49413 * track A + 10.45 (fit over 342,290 driving pairs);
    # foot-off rest A=465 B=241, full press A=2616 B=1284. Physical 0..255 = pedal travel fraction. Every commanded pair must
    # sit on the car's own curve or the ECU can flag an APS correlation fault. This is the test that catches a wrong
    # (e.g. Toyota-copied, B = A + 475) per-track scaling.
    packer = CANPacker(gi.GAS_INTERCEPTOR_DBC)

    def raw(frac: float) -> tuple[int, int]:
      # pack directly: create_gas_interceptor_command zeroes both tracks when frac <= 0.001 (ENABLE off)
      dat = packer.make_can_msg(pedal_ids.command_msg, 0, {"GAS_COMMAND": frac * 255., "GAS_COMMAND2": frac * 255.})[1]
      return (dat[0] << 8) | dat[1], (dat[2] << 8) | dat[3]

    a0, b0 = raw(0.)
    assert abs(a0 - 465) <= 2 and abs(b0 - 240) <= 3  # rest
    a1, b1 = raw(1.)
    assert abs(a1 - 2616) <= 2 and abs(b1 - 1303) <= 3  # full press (B from the fitted relation; measured 1284)
    expected_a = {0.05: 573, 0.15: 788, 0.25: 1003}  # linear rest..full on track A
    for frac, a_exp in expected_a.items():
      assert abs(raw(frac)[0] - a_exp) <= 2, frac
    prev_a = prev_b = -1
    for frac in np.linspace(0., 1., 41):
      a, b = raw(float(frac))
      assert abs(b - (0.49413 * a + 10.45)) <= 3, (frac, a, b)
      assert a > prev_a and b > prev_b  # monotonic in both tracks
      prev_a, prev_b = a, b
    # the real sender agrees with the direct pack whenever it enables (and goes to the active dialect's address)
    for frac in (0.05, 0.15, 0.25, 1.0):
      addr, dat, _ = create_gas_interceptor_command(packer, frac, 0, pedal_ids.command_msg)
      assert addr == pedal_ids.command_addr
      assert ((dat[0] << 8) | dat[1], (dat[2] << 8) | dat[3]) == raw(frac)

  def test_hold_speed_matches_road_fit(self):
    # routes 00000127+00000128 steady hold (1 s mean |aEgo| < 0.06, pedal state 0, no pedals), speed axis vEgo*1.0125:
    # measured median hold command 0.089 / 0.118 / 0.155 / 0.177 / 0.192 at 15 / 20 / 25 / 30 / 33 m/s
    # (drive-128 planner report §1.4; independent refit in the drive-128 buttons report §6)
    for v_ego, measured_hold in ((15., 0.089), (20., 0.118), (25., 0.155), (30., 0.177), (33., 0.192)):
      assert gi.get_pedal_command(0., v_ego) == pytest.approx(measured_hold, abs=0.012), v_ego

  def test_hold_table_pinned(self):
    # 0 m/s anchors at 0 (no uncommanded motion at a standstill; gas-only authority); from 20 m/s: 0.36 x the
    # route-127/128 m/s^2 table (unchanged law)
    assert gi.HOLD_CMD_BP == [0., 5., 10., 15., 20., 25., 30., 35.]
    assert gi.HOLD_CMD_V == pytest.approx([0., 0.08, 0.085, 0.095, 0.1224, 0.1548, 0.1764, 0.2016])
    for v in (25., 28., 31., 34.):
      assert gi.get_pedal_command(0., v) < 0.36 * (0.55 if v < 26 else 0.70), v  # well below the old v^2 table's hold
    # 12e/12f steady-hold fit (drive-12e-12f report §A.3), within +/-0.02 pedal, from 4.5 m/s up (0-5 ramps off the 0 anchor)
    for v, hold in ((4.5, 0.08), (7.5, 0.09), (10.5, 0.08), (13.5, 0.09), (17.5, 0.115), (22.5, 0.12)):
      assert gi.get_pedal_command(0., v) == pytest.approx(hold, abs=0.02), v

  def test_standstill_hold_is_zero(self):
    # The car has gas-only authority: with no brake actuator, any throttle at a standstill against a zero/negative
    # request is uncommanded motion. The zero/negative-request command at 0 m/s must be EXACTLY 0 pedal, so the send
    # path emits the ENABLE=0 all-zero frame (create_gas_interceptor_command enables only above 0.001).
    assert gi.HOLD_CMD_V[0] == 0.
    assert gi.get_hold_command(0.) == 0.
    for accel in (-4., -1., -0.5, 0.):
      assert gi.get_pedal_command(accel, 0.) == 0., accel
    # and the law stays 0 through the ramp until the request is positive (the 0-5 m/s hold ramps 0 -> 0.08)
    assert gi.get_hold_command(2.5) == pytest.approx(0.04)
    # pull-away path untouched: a positive request at 0 m/s still uses the 0.10 gain (ceiling 0.12 binds above ~1.2 m/s^2)
    assert gi.get_pedal_command(0.5, 0.) == pytest.approx(0.05)
    assert gi.get_pedal_command(1.0, 0.) == pytest.approx(0.10)

  def test_pedal_scale_table_pinned(self):
    assert gi.PEDAL_SCALE_BP == [0., 3., 6., 9., 12., 15., 18., 22.]
    assert gi.PEDAL_SCALE_V == [0.10, 0.10, 0.12, 0.14, 0.165, 0.20, 0.25, 0.36]
    assert all(b > a for a, b in zip(gi.PEDAL_SCALE_V[1:], gi.PEDAL_SCALE_V[2:], strict=False))  # monotonic rising

  @staticmethod
  def _old_law(a, v):
    # the law shipped up to buttons-v3 (0010): scalar 0.36 gain + m/s^2 hold table, same ceiling
    off = float(np.interp(v, [0., 10., 15., 20., 25., 30., 35.], [0., 0.10, 0.24, 0.34, 0.43, 0.49, 0.56]))
    ceil = min(gi.MAX_INTERCEPTOR_GAS, float(np.interp(v, gi.LOW_SPEED_MAX_GAS_BP, gi.LOW_SPEED_MAX_GAS_V)))
    return float(np.clip(0.36 * (a + off), 0., ceil))

  def test_highway_law_unchanged_from_22_mps(self):
    # following / set-speed hold / highway merges must not change: identical to the previous law at >= 22 m/s
    for v in np.linspace(22., 40., 37):
      for a in np.linspace(-1.5, 2.0, 15):
        assert gi.get_pedal_command(float(a), float(v)) == pytest.approx(self._old_law(float(a), float(v)), abs=1e-9), (a, v)

  # 12e/12f gain fit (aEgo(t+0.5 s) per unit pedal, steady pedal; driver <15 m/s, openpilot >=15 m/s), report §A.3
  FIT_V = [2., 4.5, 7.5, 10.5, 13.5, 17.5, 22.5, 27.5, 33.]
  FIT_G = [13.0, 9.7, 7.5, 6.0, 5.3, 3.9, 3.2, 2.9, 2.1]

  def test_launch_no_surge_on_fitted_plant(self):
    # 12e/12f surge band: a 1.15-1.3 m/s^2 request at 8-12 m/s gave 2.1-2.5 m/s^2. On the fitted gain the new law must
    # deliver no more than the request (+0.2 m/s^2 slack) wherever the launch ceiling no longer binds, 3-22 m/s ...
    #
    # Why the ceiling check below: at ~3 m/s a >= 1.2 m/s^2 request drives cmd straight into the launch ceiling
    # (ceiling ~0.168 at 3 m/s, reached by ~1.2 m/s^2 * 0.14/unit + hold), and it sits exactly on that ceiling under
    # BOTH the old and the new law, so the fitted plant would deliver request + ~0.20 m/s^2 from the ceiling's own
    # headroom regardless of which gain scaled the request. The surge bound is only meaningful where the law, not the
    # ceiling, sets the command, so the low-speed/ceiling-bound samples (approx v <= 4 m/s for a >= 1.2) are skipped.
    for v in np.linspace(3., 22., 39):
      g = float(np.interp(v, self.FIT_V, self.FIT_G))
      ceiling = min(gi.MAX_INTERCEPTOR_GAS, float(np.interp(v, gi.LOW_SPEED_MAX_GAS_BP, gi.LOW_SPEED_MAX_GAS_V)))
      for a in (0.5, 0.8, 1.0, 1.2, 1.5):
        cmd = gi.get_pedal_command(a, float(v))
        delivered = g * (cmd - gi.get_hold_command(float(v)))
        if cmd < ceiling - 1e-6:  # the law, not the launch ceiling, sets the command
          assert delivered <= a + 0.2, (v, a, delivered)
          # ... and still delivers most of it (an on-ramp keeps its pull) wherever the unchanged 0.35 cap does not bind
          if v >= 6. and cmd < gi.MAX_INTERCEPTOR_GAS - 1e-6:
            assert delivered >= 0.7 * a, (v, a, delivered)
    # the old law on the same (linear, steady-state) plant over-delivers a 1.2 request at 10-12 m/s by >= +0.2 m/s^2
    # (the road showed +0.8..+1.2; the linear fit under-predicts it, drive-12e-12f report §A.4), the new one does not
    for v in (10., 11., 12.):
      g = float(np.interp(v, self.FIT_V, self.FIT_G))
      old = g * (self._old_law(1.2, v) - gi.get_hold_command(v))
      new = g * (gi.get_pedal_command(1.2, v) - gi.get_hold_command(v))
      assert old - 1.2 > 0.2 and new - 1.2 < 0.0, (v, old, new)

  def test_onramp_authority_kept(self):
    # on-ramps (route 128 at 12-20 m/s, 12f first ramp to 33 m/s): the cap is still reachable when the planner asks
    # for it, at every speed from 20 mph up, and a +1.0 request on a 4 % climb at 18-20 m/s still exceeds the old
    # 0.15 cap (route 123 hill)
    for v in (12., 15., 18., 20., 25., 30.):
      assert gi.get_pedal_command(2.0, v) == pytest.approx(gi.MAX_INTERCEPTOR_GAS), v  # ACCEL_MAX reaches the cap
    for v in (17.5, 18.9, 19.9):
      assert gi.get_pedal_command(1.0, v) > 0.30, v  # route 123 climb: needed ~0.37, old cap 0.15

  def test_low_speed_launch_ceiling(self):
    # pause/resume may engage from a standstill: the launch is limited (pedal ~4-5x stronger per unit at <6 m/s)
    assert gi.get_pedal_command(5., 0.) == pytest.approx(0.12)
    assert gi.get_pedal_command(5., 5.) == pytest.approx(0.20)
    assert gi.get_pedal_command(5., 11.2) == pytest.approx(gi.MAX_INTERCEPTOR_GAS)
    # the ramp reaches the full cap by 25 mph and never exceeds it
    assert gi.LOW_SPEED_MAX_GAS_BP[-1] <= 25. * CV.MPH_TO_MS
    assert gi.LOW_SPEED_MAX_GAS_V[-1] == gi.MAX_INTERCEPTOR_GAS
    assert gi.get_pedal_command(5., 25. * CV.MPH_TO_MS) == pytest.approx(gi.MAX_INTERCEPTOR_GAS)
    assert all(gi.get_pedal_command(5., v) <= gi.MAX_INTERCEPTOR_GAS for v in np.linspace(0., 40., 81))
    assert gi.get_pedal_command(0.3, 0.) > 0.  # it does move off from a stop when the planner asks

  def test_cap_has_authority_for_route_123_hill(self):
    # route 00000123 t=362-382 s: ~3.5-4% climb at 17-20 m/s, planner asked +0.93..1.01 m/s^2, the command sat pinned
    # at the old 0.15 cap for the whole climb and speed fell 19.9 -> 17.1 m/s (EMS16 TQI flat at 15 %)
    for a_target, v_ego in ((0.94, 19.9), (0.96, 18.9), (1.0, 17.5)):
      assert gi.get_pedal_command(a_target, v_ego) > 0.15  # the old cap would clip here
    # fitted plant (aEgo = 2.76*cmd + 0.032 - 0.000935 v^2 - g*grade): at the cap the car must at least hold speed on
    # that hill's grade at its speed, and on a 2.5 % grade at 25 m/s (fit data 14-24 m/s; 25 m/s mildly extrapolated)

    def a_at_cap(v_ego, grade):
      return 2.76 * gi.MAX_INTERCEPTOR_GAS + 0.032 - 0.000935 * v_ego ** 2 - 9.81 * grade
    for v_ego, grade in ((15., 0.04), (20., 0.04), (25., 0.025)):
      assert a_at_cap(v_ego, grade) > 0., (v_ego, grade)

  def test_rate_limit(self):
    # jerk-based: rate_up(v) = 2.0 m/s^3 * pedal_scale(v) pedal/s
    assert gi.PEDAL_JERK_UP == 2.0
    for v, rate in ((0., 0.20), (12., 0.33), (18., 0.50), (22., 0.72), (35., 0.72)):
      assert gi.get_pedal_rate_up(v) == pytest.approx(rate), v
    for v in (0., 6., 12., 20., 30.):
      step = gi.get_pedal_rate_up(v) * DT_CTRL
      assert gi.rate_limit_pedal_command(0.3, 0., v) == pytest.approx(step)  # rise limited
      assert gi.rate_limit_pedal_command(0.1, 0.09, v) == pytest.approx(min(0.1, 0.09 + step))
      assert gi.rate_limit_pedal_command(0.1, 0.1 - step / 2, v) == 0.1      # within one step: exact
      assert gi.rate_limit_pedal_command(0., 0.3, v) == 0.                   # lift is immediate
    # equivalent accel jerk is 2.0 m/s^3 at every speed (the old fixed 0.75/s was 7.5 m/s^3 at a standstill)
    for v in np.linspace(0., 40., 41):
      assert gi.get_pedal_rate_up(float(v)) / gi.get_pedal_scale(float(v)) == pytest.approx(2.0)

  def test_launch_ramp_rate_gated_by_speed_and_flag(self):
    # 0039: the launch branch is a flat rate, only below LAUNCH_RAMP_V_MAX and only when `launching`.
    assert gi.LAUNCH_RAMP_V_MAX == 3.0 and gi.LAUNCH_RAMP_RATE_UP == 0.6
    # it is faster than the old standstill jerk rate, and gentler than the removed 0.75/s fixed ramp
    assert gi.get_pedal_rate_up(0., launching=False) < gi.LAUNCH_RAMP_RATE_UP < 0.75
    for v in np.linspace(0., gi.LAUNCH_RAMP_V_MAX, 7):
      assert gi.get_pedal_rate_up(float(v), launching=True) == gi.LAUNCH_RAMP_RATE_UP, v
    # the boundary is INCLUSIVE: at exactly LAUNCH_RAMP_V_MAX a launch still uses the fast rate
    assert gi.get_pedal_rate_up(gi.LAUNCH_RAMP_V_MAX, launching=True) == gi.LAUNCH_RAMP_RATE_UP
    # above the launch speed cap the launching flag is inert: the jerk rate is returned unchanged
    for v in np.linspace(gi.LAUNCH_RAMP_V_MAX + 0.1, 40., 40):
      assert gi.get_pedal_rate_up(float(v), launching=True) == pytest.approx(gi.get_pedal_rate_up(float(v))), v
      assert gi.get_pedal_rate_up(float(v), launching=True) == pytest.approx(2.0 * gi.get_pedal_scale(float(v))), v
    # the limiter honours the flag, the lift is still immediate even while launching, and no step exceeds the rate
    assert gi.rate_limit_pedal_command(0.3, 0., 0., launching=True) == pytest.approx(gi.LAUNCH_RAMP_RATE_UP * DT_CTRL)
    assert gi.rate_limit_pedal_command(0., 0.3, 0., launching=True) == 0.
    assert gi.rate_limit_pedal_command(0.1, 0.1 - gi.LAUNCH_RAMP_RATE_UP * DT_CTRL, 0., launching=True) == 0.1
    # default (no flag) is byte-identical to the pre-0039 limiter at every speed
    for v in np.linspace(0., 40., 81):
      assert gi.get_pedal_rate_up(float(v)) == pytest.approx(2.0 * gi.get_pedal_scale(float(v)))
      assert gi.rate_limit_pedal_command(0.3, 0.1, float(v)) == gi.rate_limit_pedal_command(0.3, 0.1, float(v), launching=False)

  def test_launch_reaches_rest_ceiling_quickly(self):
    # Owner complaint: a green-light launch sits at "2 %, then slowly ramps to 10, then 15, then 20". Closed-loop on
    # the fork's own fitted steady-pedal gain (drive-12e/12f FIT_V/FIT_G, see test_launch_no_surge_on_fitted_plant):
    # aEgo = G(v) * (cmd - hold(v)), integrated at 100 Hz. The real law + rate limiter drive it.
    fit_v = self.FIT_V
    fit_g = self.FIT_G

    def trajectory(launching):
      v, g = 0., 0.
      t10 = t12 = None
      for k in range(int(6.0 / DT_CTRL)):
        t = k * DT_CTRL
        target = gi.get_pedal_command(1.2, v)
        g = gi.rate_limit_pedal_command(target, g, v, launching and v <= gi.LAUNCH_RAMP_V_MAX)
        v = max(0., v + float(np.interp(v, fit_v, fit_g)) * (g - gi.get_hold_command(v)) * DT_CTRL)
        if t10 is None and g >= 0.10:
          t10 = t
        if t12 is None and g >= 0.12:
          t12 = t
      return t10, t12

    t10_before, t12_before = trajectory(False)
    t10_after, t12_after = trajectory(True)
    # after: 0 -> 0.10 in ~0.17 s, 0 -> the 0.12 rest ceiling in ~0.20 s
    assert t10_after <= 0.25 and t12_after <= 0.30, (t10_after, t12_after)
    # before was ~0.49 s / ~0.6 s: the launch is at least 2x quicker to real pedal authority
    assert t10_before > 0.4 and t10_after < t10_before / 2.0, (t10_before, t10_after)
    assert t12_after < t12_before / 2.0, (t12_before, t12_after)

  def test_rolling_engage_ramp_time(self):
    # 12e @112 / 12f @137 / @686: rolling engages at 11-19 m/s jumped 0 -> 0.35 in 0.47 s (+1.0 m/s^2 in ~0.5 s).
    # Now 0 -> cap takes >= 1.0 s at 12 m/s, ~0.7 s at 18 m/s, and ~0.49 s on the highway (about as before)
    def ramp_s(v, launching=False):
      g, n = 0., 0
      while g < gi.MAX_INTERCEPTOR_GAS:
        g = gi.rate_limit_pedal_command(gi.MAX_INTERCEPTOR_GAS, g, v, launching)
        n += 1
      return n * DT_CTRL
    assert ramp_s(12.) == pytest.approx(gi.MAX_INTERCEPTOR_GAS / 0.33, abs=2 * DT_CTRL)
    assert ramp_s(12.) > 1.0
    assert ramp_s(18.) == pytest.approx(0.70, abs=0.02)
    assert ramp_s(30.) == pytest.approx(gi.MAX_INTERCEPTOR_GAS / 0.72, abs=2 * DT_CTRL)
    # 0039 regression: even if the launch latch were somehow still set at rolling speed, the ramp is UNCHANGED - the
    # fast branch cannot fire above LAUNCH_RAMP_V_MAX, so the 11-19 m/s "jump to max" surge cannot come back. Exact
    # equality with the pre-0039 ramp (the strongest form of "no faster"), and no rolling ramp is shorter than 0.6 s.
    for v in (11., 12., 15., 18., 19.):
      assert ramp_s(v, launching=True) == ramp_s(v, False), v
      assert ramp_s(v, launching=True) > 0.6, v

  @pytest.mark.parametrize("v_ego", [0., 5., 11.2, 20., 35.])
  def test_pedal_command_accel_only(self, v_ego):
    for accel in (-4., -1., -0.5, 0., 0.5, 2., 10.):
      cmd = gi.get_pedal_command(accel, v_ego)
      assert 0. <= cmd <= gi.MAX_INTERCEPTOR_GAS
    # a hard decel request is always a full lift (never a negative/brake command)
    assert gi.get_pedal_command(-2., v_ego) == 0.


def _cp_bytes(CP) -> bytes:
  # CarParams is a capnp builder (no ==); compare the serialized message, i.e. literally byte-identical
  return CP.to_bytes()


def _get_ci(fingerprint_bus0: dict, param: bool | None, id_set: str | None = None):
  fingerprint = {i: {} for i in range(8)}
  fingerprint[0] = dict(fingerprint_bus0)
  CarInterface = interfaces[CAR_UNDER_TEST]
  CP = CarInterface.get_params(CAR_UNDER_TEST, fingerprint, [], alpha_long=False, is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, CAR_UNDER_TEST, fingerprint, [], alpha_long=False, is_release_sp=False, docs=False)
  params_list: list = [] if param is None else [{"HyundaiGasInterceptor": param}]
  if id_set is not None:
    params_list.append({"HyundaiGasInterceptorIDSet": id_set})
  setup_interfaces(CarInterface, CP, CP_SP, params_list)
  return CarInterface, CP, CP_SP


class TestEnablement:
  def test_no_pedal_is_identical(self):
    _, CP_ref, CP_SP_ref = _get_ci(BASE_FINGERPRINT, None)
    # param alone (incl. any ID-set override) must not change anything when no pedal is on the bus
    for id_set in (None, "auto", "standard", "remapped"):
      _, CP, CP_SP = _get_ci(BASE_FINGERPRINT, True, id_set)
      assert _cp_bytes(CP) == _cp_bytes(CP_ref) and CP_SP == CP_SP_ref, id_set
    assert not CP_SP.enableGasInterceptor and not CP.openpilotLongitudinalControl and CP.pcmCruise
    assert not CP_SP.safetyParam & HyundaiSafetyFlagsSP.GAS_INTERCEPTOR

  @pytest.mark.parametrize("pedal", [STD, REM, {**STD, **REM}], ids=["standard", "remapped", "both"])
  @pytest.mark.parametrize("id_set", [None, "auto", "remapped"])
  @pytest.mark.parametrize("param", [None, False])
  def test_pedal_without_opt_in_is_inert(self, param, id_set, pedal):
    _, CP_ref, CP_SP_ref = _get_ci(BASE_FINGERPRINT, None)
    _, CP, CP_SP = _get_ci({**BASE_FINGERPRINT, **pedal}, param, id_set)
    # only the capability flag(s) differ
    assert bool(CP_SP.flags & HyundaiFlagsSP.GAS_INTERCEPTOR_DETECTED) == (0x201 in pedal)
    assert bool(CP_SP.flags & HyundaiFlagsSP.GAS_INTERCEPTOR_REMAPPED_DETECTED) == (0x701 in pedal)
    CP_SP_cmp = copy.deepcopy(CP_SP)
    CP_SP_cmp.flags &= ~DETECT_FLAGS
    assert _cp_bytes(CP) == _cp_bytes(CP_ref) and CP_SP_cmp == CP_SP_ref
    assert not CP_SP.enableGasInterceptor and not CP_SP.safetyParam & PEDAL_SP

  @pytest.mark.parametrize("pedal, id_set, expected", [
    (STD, None, gi.STANDARD_IDS),
    (STD, "auto", gi.STANDARD_IDS),
    (REM, None, gi.REMAPPED_IDS),
    (REM, "auto", gi.REMAPPED_IDS),
    (REM, b"auto", gi.REMAPPED_IDS),
    (REM, "bogus", gi.REMAPPED_IDS),
    ({**STD, **REM}, "auto", gi.STANDARD_IDS),  # both seen: prefer standard
    ({**STD, **REM}, "standard", gi.STANDARD_IDS),
    ({**STD, **REM}, "remapped", gi.REMAPPED_IDS),
    (STD, "standard", gi.STANDARD_IDS),
    (REM, "remapped", gi.REMAPPED_IDS),
    (STD, "remapped", None),  # forced dialect not seen on the bus: stay off
    (REM, "standard", None),
  ])
  def test_dialect_selection(self, pedal, id_set, expected):
    _, CP, CP_SP = _get_ci({**BASE_FINGERPRINT, **pedal}, True, id_set)
    if expected is None:
      _, CP_ref, _ = _get_ci(BASE_FINGERPRINT, None)
      assert not CP_SP.enableGasInterceptor and not CP_SP.safetyParam & PEDAL_SP
      assert _cp_bytes(CP) == _cp_bytes(CP_ref)
      return
    assert CP_SP.enableGasInterceptor and CP.openpilotLongitudinalControl and not CP.pcmCruise
    assert CP_SP.safetyParam & HyundaiSafetyFlagsSP.GAS_INTERCEPTOR
    assert bool(CP_SP.safetyParam & HyundaiSafetyFlagsSP.GAS_INTERCEPTOR_REMAPPED) == (expected is gi.REMAPPED_IDS)
    assert gi.get_interceptor_ids(CP_SP) is expected

  def test_pedal_and_opt_in_enables(self):
    _, CP_ref, _ = _get_ci(BASE_FINGERPRINT, None)
    _, CP, CP_SP = _get_ci({**BASE_FINGERPRINT, 0x201: 6}, True)
    assert CP_SP.enableGasInterceptor
    assert CP.openpilotLongitudinalControl and not CP.pcmCruise
    assert CP_SP.safetyParam & HyundaiSafetyFlagsSP.GAS_INTERCEPTOR and CP_SP.safetyParam & HyundaiSafetyFlagsSP.NON_SCC
    # buttons-v3: no engage floor (as upstream Toyota with an interceptor); the launch limit is a throttle limit only
    assert CP.minEnableSpeed == -1.
    assert CP.autoResumeSng == CP_ref.autoResumeSng  # untouched
    assert not CP.alphaLongitudinalAvailable
    # stock HKG long flag must stay off: it would select the SCC TX allowlist in panda
    assert CP.safetyConfigs[0].safetyParam == CP_ref.safetyConfigs[0].safetyParam

  @pytest.mark.parametrize("pedal, ids", [(STD, gi.STANDARD_IDS), (REM, gi.REMAPPED_IDS)], ids=["standard", "remapped"])
  def test_runs_and_sends_only_pedal_long(self, pedal, ids):
    CarInterface, CP, CP_SP = _get_ci({**BASE_FINGERPRINT, **pedal}, True)
    CI = CarInterface(CP, CP_SP)
    assert Bus.alt in CI.can_parsers and CI.can_parsers[Bus.alt].dbc_name == gi.GAS_INTERCEPTOR_DBC
    other = gi.REMAPPED_IDS if ids is gi.STANDARD_IDS else gi.STANDARD_IDS

    CC = structs.CarControl()
    CC.enabled = CC.latActive = CC.longActive = True
    CC.actuators.accel = 1.0
    CC_SP = structs.CarControlSP()
    sent = set()
    now = 0
    for _ in range(20):
      CI.update([])
      CI.CS.out.vEgo = 25.
      _, sends = CI.apply(CC.as_reader(), CC_SP, now)
      sent |= {addr for addr, _, _ in sends}
      now += int(DT_CTRL * 1e9)
    assert ids.command_addr in sent and other.command_addr not in sent
    # no SCC replacement / radar disable traffic
    assert not sent & {0x420, 0x421, 0x50A, 0x389, 0x4A2, 0x38D, 0x483, 0x7D0}


# *** pedal fault latch recovery: clear frame + escalation (FORK.md #14, route 11c) ***

def _sensor_frame_state(ids, gas: int, counter: int, state: int):
  dat = bytearray(_sensor_frame(ids, gas, counter)[1])
  dat[4] = ((state & 0xF) << 4) | (counter & 0xF)
  dat[5] = crc8_pedal(dat[:5])
  return ids.sensor_addr, bytes(dat)


def _cmd_fields(dat: bytes) -> tuple[int, int, int, int]:
  """-> (track A, track B, ENABLE, counter)"""
  return (dat[0] << 8) | dat[1], (dat[2] << 8) | dat[3], dat[4] >> 7, dat[4] & 0xF


class TestPedalFaultMonitor:
  """Policy unit: 100 Hz frames, send slot on even frames, gas > 0 while active."""
  def _run(self, states, active=True, gas=0.1):
    mon = gi.PedalFaultMonitor()
    clears, escalated = [], []
    for frame, st in enumerate(states):
      if mon.update(frame, active, st, frame % 2 == 0, gas):
        clears.append(frame)
      escalated.append(mon.escalated)
    return clears, escalated

  def test_no_fault_no_clear(self):
    clears, esc = self._run([0] * 1000)
    assert clears == [] and not any(esc)

  def test_fault_clears_rate_limited(self):
    # fault visible for 40 frames (state stale/sticky): clears only on send slots, >= holdoff apart
    clears, _ = self._run([0] * 10 + [3] * 40 + [0] * 50)
    assert clears == [10, 16, 22, 28, 34, 40, 46]
    assert all(c % 2 == 0 for c in clears)
    assert all(b - a >= gi.PEDAL_CLEAR_HOLDOFF_FRAMES for a, b in zip(clears, clears[1:], strict=False))
    # exactly one clear per 60 ms window
    for start in range(100):
      assert sum(start <= c < start + gi.PEDAL_CLEAR_HOLDOFF_FRAMES for c in clears) <= 1

  def test_fault_on_odd_frame_waits_for_send_slot(self):
    clears, _ = self._run([0] * 11 + [3] + [0] * 10)
    assert clears == []  # one-tick blip on a non-send frame, gone before the slot: nothing to clear
    clears, _ = self._run([0] * 11 + [3, 3] + [0] * 10)
    assert clears == [12]

  @pytest.mark.parametrize("state", [1, 2, 3, 4, 5, 6])
  def test_any_fault_state_is_cleared(self, state):
    clears, _ = self._run([0] * 4 + [state] + [0] * 10)
    assert clears == [4]

  def test_inactive_never_clears_or_escalates(self):
    clears, esc = self._run([3] * 500, active=False)
    assert clears == [] and not any(esc)

  def test_zero_gas_never_clears(self):
    # commanding gas 0 already sends the (clearing) zero frame; no substitution needed
    clears, _ = self._run([3] * 30, gas=0.)
    assert clears == []

  def test_escalates_on_persistent_fault_timeout(self):
    # time path in isolation: gas 0 -> no clear substitutions (the stream is already zero), fault stays anyway
    clears, esc = self._run([0] * 10 + [3] * 100, gas=0.)
    assert clears == []
    first = esc.index(True)
    assert first == 10 + gi.PEDAL_FAULT_TIMEOUT_FRAMES + 1  # strictly more than 0.5 s of continuous fault
    assert all(esc[first:])
    # 0.5 s exactly does not escalate
    _, esc = self._run([0] * 10 + [3] * (gi.PEDAL_FAULT_TIMEOUT_FRAMES + 1) + [0] * 100, gas=0.)
    assert not any(esc)

  def test_escalates_on_persistent_fault_while_clearing(self):
    # a fault the clears can't recover: one clear per 60 ms, the 9th lands 480 ms in -> count path fires (<= 0.5 s)
    clears, esc = self._run([0] * 10 + [3] * 100)
    first = esc.index(True)
    assert first == clears[gi.PEDAL_MAX_CLEARS] == 10 + gi.PEDAL_MAX_CLEARS * gi.PEDAL_CLEAR_HOLDOFF_FRAMES
    assert first - 10 <= gi.PEDAL_FAULT_TIMEOUT_FRAMES and all(esc[first:])

  def test_escalates_on_too_many_clears(self):
    # short blips (each cleared, never 0.5 s long) every 200 ms: 10 clears in 2 s -> 9th clear escalates
    states = ([3, 3] + [0] * 18) * 15
    clears, esc = self._run(states)
    first = esc.index(True)
    assert first == clears[gi.PEDAL_MAX_CLEARS]  # the (MAX+1)th clear
    assert len([c for c in clears if c <= first]) == gi.PEDAL_MAX_CLEARS + 1
    assert clears[gi.PEDAL_MAX_CLEARS] - clears[0] < gi.PEDAL_CLEAR_WINDOW_FRAMES

  def test_slow_blips_do_not_escalate(self):
    # 8 clears per 2 s is tolerated indefinitely (blip every 250 ms)
    clears, esc = self._run(([3, 3] + [0] * 23) * 40)
    assert len(clears) == 40 and not any(esc)

  def test_escalation_held_until_disengage_then_rearms(self):
    mon = gi.PedalFaultMonitor()
    for f in range(100):
      mon.update(f, True, 3, f % 2 == 0, 0.1)
    assert mon.escalated
    mon.update(100, True, 0, True, 0.1)
    assert mon.escalated  # recovery while still engaged doesn't silently re-arm
    mon.update(101, False, 0, False, 0.)
    assert not mon.escalated and mon.fault_start_frame is None and not mon.clear_frames


class TestPedalFaultClearEndToEnd:
  """Through the real call path: CarInterface.update (CarState parses 0x701 STATE) -> CarInterface.apply
  (CarController.create_gas_command) -> the 0x700 frames on the wire, and accFaulted back out of CarState."""
  def _setup(self, ids=gi.REMAPPED_IDS, pedal=REM):
    CarInterface, CP, CP_SP = _get_ci({**BASE_FINGERPRINT, **pedal}, True)
    CI = CarInterface(CP, CP_SP)
    CC = structs.CarControl()
    CC.enabled = CC.latActive = CC.longActive = True
    CC.actuators.accel = 1.0
    return CI, CC.as_reader(), structs.CarControlSP(), ids

  def _drive(self, CI, CC, CC_SP, ids, states, v_ego=25.):
    t = 0
    out = []
    for i, st in enumerate(states):
      t += int(DT_CTRL * 1e9)
      ret, _ = CI.update([(t, [(*_sensor_frame_state(ids, 0, i, st), 0)])])
      CI.CS.out.vEgo = v_ego
      _, sends = CI.apply(CC, CC_SP, t)
      cmd = [dat for addr, dat, _ in sends if addr == ids.command_addr]
      out.append((cmd[0] if cmd else None, ret.accFaulted))
    return out

  @pytest.mark.parametrize("v_ego", [0., 2., 6.])
  def test_low_speed_commands_through_real_path(self, v_ego):
    # pause/resume may engage at a standstill: the CarController must command throttle there (no low-speed cut), but
    # never above the launch ceiling. Through CarInterface.apply -> 0x700 on the wire.
    CI, CC, CC_SP, ids = self._setup()
    out = self._drive(CI, CC, CC_SP, ids, [0] * 200, v_ego=v_ego)
    a = [_cmd_fields(d) for d, _ in out if d is not None]
    ceiling = float(np.interp(v_ego, gi.LOW_SPEED_MAX_GAS_BP, gi.LOW_SPEED_MAX_GAS_V))
    # accel 1.0 at 0-6 m/s: the speed-scheduled law (0.10-0.12 gain + hold) stays below the launch ceiling at every
    # speed here (with the 0 m/s hold anchor at 0), so a +1.0 request commands throttle and only stronger requests bind
    target = min(ceiling, gi.get_pedal_scale(v_ego) * 1.0 + gi.get_hold_command(v_ego))
    assert target < ceiling
    _, dc, _ = create_gas_interceptor_command(CANPacker(gi.GAS_INTERCEPTOR_DBC), target, 0)
    assert a[-1][2] == 1 and abs(a[-1][0] - ((dc[0] << 8) | dc[1])) <= 2, (v_ego, a[-1])
    assert all(x[0] <= ((dc[0] << 8) | dc[1]) + 2 for x in a)

  @pytest.mark.parametrize("accel", [0., -1., -2.])
  def test_standstill_zero_request_sends_zero_frame(self, accel):
    # Gas-only authority (no brake actuator): engaged at 0 m/s with a zero/negative request, the wire must carry the
    # all-zero ENABLE=0 frame, never the old 0.04 hold (which would be uncommanded motion at a standstill).
    CI, _, CC_SP, ids = self._setup()
    CC = structs.CarControl()
    CC.enabled = CC.latActive = CC.longActive = True
    CC.actuators.accel = accel
    out = self._drive(CI, CC.as_reader(), CC_SP, ids, [0] * 60, v_ego=0.)
    cmds = [_cmd_fields(d) for d, _ in out if d is not None]
    assert cmds and all(c[:3] == (0, 0, 0) for c in cmds)

  @pytest.mark.parametrize("pedal, ids", [(STD, gi.STANDARD_IDS), (REM, gi.REMAPPED_IDS)], ids=["standard", "remapped"])
  def test_clear_frame_on_fault(self, pedal, ids):
    CI, CC, CC_SP, ids = self._setup(ids, pedal)
    # 60 frames healthy (the engage ramp reaches the cap after 49 at 25 m/s), 20 faulted, 20 healthy
    out = self._drive(CI, CC, CC_SP, ids, [0] * 60 + [3] * 20 + [0] * 20)
    cmds = [(f, _cmd_fields(d)) for f, (d, _) in enumerate(out) if d is not None]
    assert all(f % 2 == 0 for f, _ in cmds) and len(cmds) == 50
    # counter continuous across clear frames
    assert all((b[1][3] - a[1][3]) % 16 == 1 for a, b in zip(cmds, cmds[1:], strict=False))
    zero = [f for f, c in cmds if c[2] == 0]
    assert zero == [60, 66, 72, 78]
    for f, (a, b, en, _) in cmds:
      if f in zero:
        assert (a, b, en) == (0, 0, 0)  # the firmware's clear frame: ENABLE=0, both tracks 0
      elif f >= 48:
        # commanding at the cap, also right after each clear frame (a clear doesn't restart the engage ramp)
        assert en == 1 and abs(a - 1218) <= 2
    assert not any(acc for _, acc in out)
    # every frame (incl. clears) has a valid pedal checksum
    assert all(d[5] == crc8_pedal(d[:5]) for d, _ in out if d is not None)

  def test_engage_ramps_then_lift_is_immediate(self):
    CI, CC, CC_SP, ids = self._setup()
    out = self._drive(CI, CC, CC_SP, ids, [0] * 60)
    a = [_cmd_fields(d)[0] for d, _ in out if d is not None]
    # first frame is one rate step, not a step to the cap (old behaviour: 0 -> cap in one frame)
    # v_ego 25 m/s (from _drive): jerk limit 2.0 m/s^3 * 0.36 = 0.72/s, 0 -> 0.35 in 0.49 s
    _, d1, _ = create_gas_interceptor_command(CANPacker(gi.GAS_INTERCEPTOR_DBC), gi.get_pedal_rate_up(25.) * DT_CTRL, 0)
    assert a[0] == (d1[0] << 8) | d1[1]
    assert all(y >= x for x, y in zip(a, a[1:], strict=False)) and abs(a[-1] - 1218) <= 2
    assert sum(abs(x - 1218) > 2 for x in a) == 24  # 0.49 s ramp: 24 send slots below the cap
    # accel request drops to coast -> the very next frame is the zero frame
    CC2 = structs.CarControl()
    CC2.enabled = CC2.latActive = CC2.longActive = True
    CC2.actuators.accel = -2.0
    out = self._drive(CI, CC2.as_reader(), CC_SP, ids, [0] * 4)
    assert all(_cmd_fields(d)[:3] == (0, 0, 0) for d, _ in out if d is not None)

  def test_no_fault_unchanged(self):
    CI, CC, CC_SP, ids = self._setup()
    out = self._drive(CI, CC, CC_SP, ids, [0] * 300)
    cmds = [_cmd_fields(d) for d, _ in out if d is not None]
    assert len(cmds) == 150 and all(en == 1 for _, _, en, _ in cmds)
    assert not any(acc for _, acc in out)

  def test_persistent_fault_sets_acc_faulted(self):
    CI, CC, CC_SP, ids = self._setup()
    out = self._drive(CI, CC, CC_SP, ids, [0] * 10 + [3] * 80)
    acc = [a for _, a in out]
    first = acc.index(True)
    # the 9th clear (frame 10 + 8*6) latches escalation in the controller; the next CarState update reports it
    assert first == 10 + gi.PEDAL_MAX_CLEARS * gi.PEDAL_CLEAR_HOLDOFF_FRAMES + 1
    assert first - 10 <= gi.PEDAL_FAULT_TIMEOUT_FRAMES + 1
    assert all(acc[first:])

  def test_disengaged_never_clears(self):
    CI, _, CC_SP, ids = self._setup()
    CC = structs.CarControl()
    CC.enabled = CC.latActive = True  # lateral only
    out = self._drive(CI, CC.as_reader(), CC_SP, ids, [3] * 200)
    assert all(_cmd_fields(d)[:3] == (0, 0, 0) for d, _ in out if d is not None)
    assert not any(a for _, a in out)

  def test_no_interceptor_car_inert(self):
    CarInterface, CP, CP_SP = _get_ci(BASE_FINGERPRINT, None)
    CI = CarInterface(CP, CP_SP)
    assert not CP_SP.enableGasInterceptor
    CC = structs.CarControl()
    CC.enabled = CC.latActive = CC.longActive = True
    for i in range(50):
      CI.update([])
      CI.CS.interceptor_state = 3  # even a bogus state can't produce anything
      _, sends = CI.apply(CC.as_reader(), structs.CarControlSP(), i)
      assert not {a for a, _, _ in sends} & {0x200, 0x700}
    assert not CI.CS.pedal_fault_monitor.escalated and not CI.CS.pedal_fault_monitor.clear_frames


class TestLaunchRampIntegration:
  """0039: through the real CarInterface.apply path, the launch latch selects the fast ramp only from (near) rest."""
  def _make(self, accel=1.2):
    CarInterface, CP, CP_SP = _get_ci({**BASE_FINGERPRINT, **REM}, True)
    CI = CarInterface(CP, CP_SP)
    CC = structs.CarControl()
    CC.enabled = CC.latActive = CC.longActive = True
    CC.actuators.accel = accel
    return CI, CC.as_reader(), structs.CarControlSP()

  def test_latch_sets_from_rest_and_clears_above_launch_speed(self):
    assert gi.LAUNCH_RAMP_V_MAX == 3.0
    CI, CC, CC_SP = self._make()
    t = 0
    seen = []
    for v in [0., 0., 1., 2., 3., 3.01, 5., 12., 18., 25., 2.9, 0.]:
      for _ in range(3):
        t += int(DT_CTRL * 1e9)
        CI.update([])
        CI.CS.out.vEgo = v
        CI.apply(CC, CC_SP, t)
        seen.append((v, CI.CC.gas_launching))
    # a positive request at v <= 3 m/s latches; above it (rolling re-engagement) it never does
    assert all(l for v, l in seen if v <= gi.LAUNCH_RAMP_V_MAX), seen
    assert not any(l for v, l in seen if v > gi.LAUNCH_RAMP_V_MAX), seen

  def test_latch_clears_on_non_positive_request(self):
    CI, CC, CC_SP = self._make()
    t = 0
    for _ in range(6):                       # positive at rest -> latched
      t += int(DT_CTRL * 1e9)
      CI.update([]); CI.CS.out.vEgo = 0.; CI.apply(CC, CC_SP, t)
    assert CI.CC.gas_launching
    cc_coast = structs.CarControl()
    cc_coast.enabled = cc_coast.latActive = cc_coast.longActive = True
    cc_coast.actuators.accel = -2.0
    t += int(DT_CTRL * 1e9)
    CI.update([]); CI.CS.out.vEgo = 0.; CI.apply(cc_coast.as_reader(), CC_SP, t)
    assert not CI.CC.gas_launching

  def test_standstill_launch_reaches_authority_fast(self):
    # real path at v=0: the command must reach 0.05 pedal within the first 5 send slots and 0.10 within 9, and never
    # exceed the unchanged 0.12 rest ceiling. Pre-0039 the same law needed 3x as long (0.05 @ 0.26 s, 0.10 @ 0.52 s);
    # here 0.05 @ ~0.10 s, 0.10 @ ~0.18 s. Raw mapping (DBC, bench-pinned): raw = 465 + frac * 2151.
    CI, CC, CC_SP = self._make(accel=1.2)
    t, slots = 0, []
    for i in range(80):
      t += int(DT_CTRL * 1e9)
      CI.update([])
      CI.CS.out.vEgo = 0.
      _, sends = CI.apply(CC, CC_SP, t)
      cmd = [d for a, d, _ in sends if a == gi.REMAPPED_IDS.command_addr]
      if cmd:
        slots.append((cmd[0][0] << 8) | cmd[0][1])
    assert len(slots) >= 11
    assert slots[4] >= 573 - 6, slots[:6]       # 0.05 pedal by the 5th send slot (~0.10 s)
    assert slots[8] >= 680 - 6, slots[:10]      # 0.10 pedal by the 9th send slot (~0.18 s)
    assert max(slots) <= 465 + 0.12 * 2151 + 3, max(slots)  # never above the 0.12 rest ceiling


class TestNoClu11InPedalMode:
  """Root-cause fix for the pedal SCE faults: openpilot never transmits CLU11 (0x4F1) in gas-interceptor mode. Our CLU11
  shares the ID with the cluster's own 50 Hz CLU11; on routes 11c and 123 every commanded SCE onset followed one of our
  CLU11 frames by 3-17 ms. The factory cruise is locked out through its MAIN lamp instead (TestFactoryMainLockout)."""
  def _sends(self, pedal: dict | None, n: int = 400) -> list[tuple[int, list]]:
    fp = BASE_FINGERPRINT if pedal is None else {**BASE_FINGERPRINT, **pedal}
    CarInterface, CP, CP_SP = _get_ci(fp, None if pedal is None else True)
    CI = CarInterface(CP, CP_SP)
    out = []
    for i in range(n):
      CC = structs.CarControl()
      # every combination openpilot could ask for, incl. cancel + resume requests and both engagement states
      CC.cruiseControl.cancel = (i // 50) % 2 == 0
      CC.cruiseControl.resume = (i // 25) % 2 == 0
      CC.enabled = CC.latActive = CC.longActive = (i // 100) % 2 == 0
      CI.update([])
      CI.CS.out.vEgo = 25.
      _, sends = CI.apply(CC.as_reader(), structs.CarControlSP(), i * int(DT_CTRL * 1e9))
      out.append((i, sends))
    return out

  @pytest.mark.parametrize("pedal", [STD, REM], ids=["standard", "remapped"])
  def test_zero_clu11_frames(self, pedal):
    sent = [i for i, sends in self._sends(pedal) if any(a == 0x4F1 for a, _, _ in sends)]
    assert sent == []

  def test_non_interceptor_cancel_unchanged(self):
    # stock (pcmCruise) cars keep sending CANCEL after the 10-frame delay
    CarInterface, CP, CP_SP = _get_ci(BASE_FINGERPRINT, None)
    CI = CarInterface(CP, CP_SP)
    CC = structs.CarControl()
    CC.cruiseControl.cancel = True
    frames = []
    for i in range(100):
      CI.update([])
      _, sends = CI.apply(CC.as_reader(), structs.CarControlSP(), i)
      if any(a == 0x4F1 for a, _, _ in sends):
        frames.append(i)
    assert frames == list(range(10, 100))


# *** CarState through the real CarInterface.update path: factory-cruise lockout + pause/resume button ***

PK = CANPacker("hyundai_can_generated")


class _Car:
  """Drives CarInterface.update with EMS16 / TCS13 / CLU11 / pedal sensor frames. One step = one 50 Hz CLU11 sample
  = two 100 Hz CarState updates (CLU11 rides on the first)."""
  def __init__(self):
    CarInterface, self.CP, self.CP_SP = _get_ci({**BASE_FINGERPRINT, **REM}, True)
    self.CI = CarInterface(self.CP, self.CP_SP)
    self.t = 0
    self.i = 0
    self.engaged = False

  def step(self, btn=0, brake=False, gas=False, main=False, active=False):
    rets = []
    for sub in range(2):
      self.t += int(DT_CTRL * 1e9)
      frames = [PK.make_can_msg("EMS16", 0, {"CRUISE_LAMP_M": main, "CRUISE_LAMP_S": active, "AliveCounter": self.i % 4}),
                PK.make_can_msg("TCS13", 0, {"DriverOverride": 2 if brake else 0, "AliveCounterTCS": self.i % 8}),
                _sensor_frame(gi.REMAPPED_IDS, 2000 if gas else 0, self.i) + (0,)]
      if sub == 0:
        frames.append(PK.make_can_msg("CLU11", 0, {"CF_Clu_CruiseSwState": btn}))
      self.i += 1
      # what the CarController stored on the previous frame (CarControl.enabled): the state before this sample
      self.CI.CS.pedal_long_engaged = self.engaged
      ret, _ = self.CI.update([(self.t, [(a, d, b) for a, d, b in frames])])
      rets.append(ret)
    return rets

  def run(self, n, **kw):
    out = []
    for _ in range(n):
      out += self.step(**kw)
    return out


def _types(rets, attr="buttonEvents"):
  return [(str(b.type), b.pressed) for r in rets for b in getattr(r, attr)]


def _enables(rets):
  return sum(r.buttonEnable for r in rets)


class TestFactoryMainLockout:
  def test_main_armed_is_non_adaptive_lockout(self):
    # available stays True (MADS lateral never follows the factory MAIN); MAIN armed -> nonAdaptive -> wrongCruiseMode
    car = _Car()
    rets = car.run(5)[2:]
    assert all(r.cruiseState.available and not r.cruiseState.nonAdaptive for r in rets)
    rets = car.run(5, main=True)[1:]
    assert all(r.cruiseState.available and r.cruiseState.nonAdaptive for r in rets)
    rets = car.run(5, main=True, active=True)[1:]
    assert all(r.cruiseState.nonAdaptive and r.cruiseState.enabled for r in rets)
    rets = car.run(5)[1:]
    assert all(r.cruiseState.available and not r.cruiseState.nonAdaptive for r in rets)

  def test_no_engage_while_main_armed(self):
    car = _Car()
    car.run(5, main=True)
    for btn in (1, 2, 4):
      rets = car.run(5, btn=btn, main=True) + car.run(10, main=True)
      assert _enables(rets) == 0, btn

  def test_no_pedal_car_keeps_main_semantics(self):
    # feature off: cruiseState.available stays = CRUISE_LAMP_M (stock non-SCC behaviour)
    CarInterface, CP, CP_SP = _get_ci(BASE_FINGERPRINT, None)
    CI = CarInterface(CP, CP_SP)
    t = 0
    for main in (True, False, True):
      for _ in range(3):
        t += 10_000_000
        ret, _ = CI.update([(t, [PK.make_can_msg("EMS16", 0, {"CRUISE_LAMP_M": main})])])
      assert ret.cruiseState.available == main


class TestFactoryCancelGiveUp:
  """openpilot side of panda's timed factory-cruise CANCEL: if CRUISE_LAMP_S rose while long was engaged and is still on
  after panda's 2 s window (+0.5 s), CarState.accFaulted -> audible immediate disable; never otherwise."""
  def test_window_matches_panda(self):
    from opendbc.sunnypilot.car.hyundai.carstate_ext import FACTORY_CANCEL_GIVE_UP_FRAMES
    window_us = _c_define(HYUNDAI_COMMON_H, "HYUNDAI_FC_CANCEL_WINDOW_US")
    assert FACTORY_CANCEL_GIVE_UP_FRAMES * DT_CTRL * 1e6 > window_us
    assert FACTORY_CANCEL_GIVE_UP_FRAMES * DT_CTRL * 1e6 <= window_us + 1e6  # alert within 1 s of panda giving up

  def _rise_while_engaged(self):
    car = _Car()
    car.run(10)
    car.engaged = True
    return car

  def test_cancel_lands_no_fault(self):
    car = self._rise_while_engaged()
    rets = car.run(50, main=True, active=True)   # 1 s on, then the cancel lands
    car.engaged = False
    rets += car.run(200, main=True)
    assert not any(r.accFaulted for r in rets)

  def test_cancel_fails_faults_after_window(self):
    car = self._rise_while_engaged()
    rets = car.run(120, main=True, active=True)  # 2.4 s
    assert not any(r.accFaulted for r in rets)
    car.engaged = False                          # (the lockout disengaged openpilot right away)
    rets = car.run(20, main=True, active=True)   # -> 2.8 s
    assert rets[-1].accFaulted
    # held while the factory cruise stays on, cleared once it is off
    assert all(r.accFaulted for r in car.run(50, main=True, active=True))
    rets = car.run(5, main=True)
    assert not rets[-1].accFaulted

  def test_not_engaged_never_faults(self):
    # the driver using the factory cruise himself (openpilot long off): panda doesn't cancel, openpilot never faults
    car = _Car()
    rets = car.run(10) + car.run(300, main=True, active=True)
    assert not any(r.accFaulted for r in rets)

  def test_main_only_never_faults(self):
    car = self._rise_while_engaged()
    rets = car.run(300, main=True)
    assert not any(r.accFaulted for r in rets)


class TestPauseResumeButton:
  def _disengaged_after_brake(self):
    car = _Car()
    car.run(10)
    car.engaged = True
    car.run(5, brake=True)  # brake: openpilot disengages (pedalPressed); panda clears controls_allowed
    car.engaged = False
    return car

  def test_constant_matches_panda(self):
    assert pr.PAUSE_RELEASE_SAMPLES == _c_define(HYUNDAI_COMMON_H, "HYUNDAI_PAUSE_RELEASE_SAMPLES")

  def test_no_auto_resume_after_brake(self):
    # brake released, then 20 s of normal driving without a button press: never an engage request
    car = self._disengaged_after_brake()
    rets = car.run(1000)
    assert _enables(rets) == 0
    assert not any(t[0] == "resumeCruise" for t in _types(rets))

  def test_resume_after_brake(self):
    car = self._disengaged_after_brake()
    car.run(5)
    held = car.run(5, btn=4)
    assert _enables(held) == 0 and _types(held) == []  # no cancel, nothing while held
    rel = car.run(5)
    # exactly one engage request, on the 3rd released CLU11 sample, carried by resumeCruise
    assert _enables(rel) == 1 and rel[2 * (pr.PAUSE_RELEASE_SAMPLES - 1)].buttonEnable
    assert _types(rel) == [("resumeCruise", True), ("resumeCruise", False)]

  def test_press_while_engaged_disengages(self):
    car = _Car()
    car.run(10)
    car.engaged = True
    rets = car.run(5, btn=4)
    assert _types(rets) == [("cancel", True)]
    car.engaged = False  # openpilot disengaged on buttonCancel
    rets = car.run(10)
    assert _types(rets) == [("cancel", False)] and _enables(rets) == 0

  @pytest.mark.parametrize("bounce", [1, 2])
  def test_bounce_in_disengage_press_never_resumes(self, bounce):
    car = _Car()
    car.run(10)
    car.engaged = True
    rets = car.run(3, btn=4)
    car.engaged = False
    rets += car.run(bounce) + car.run(3, btn=4) + car.run(20)
    assert _enables(rets) == 0 and not any(t[0] == "resumeCruise" for t in _types(rets))

  def test_press_while_brake_held(self):
    car = self._disengaged_after_brake()
    rets = car.run(5, btn=4, brake=True) + car.run(20, brake=True)
    assert _enables(rets) == 0

  def test_press_held_across_brake_release(self):
    car = self._disengaged_after_brake()
    rets = car.run(5, btn=4, brake=True) + car.run(5, btn=4) + car.run(20)
    assert _enables(rets) == 0

  def test_brake_tap_during_press(self):
    car = self._disengaged_after_brake()
    car.run(5)
    rets = car.run(3, btn=4) + car.run(1, btn=4, brake=True) + car.run(3, btn=4) + car.run(20)
    assert _enables(rets) == 0

  def test_gas_pressed(self):
    # route 00000128 @2152.9 / 2154.2 / 2155.7: three pause/resume presses with the foot on the gas, all refused.
    # Gas no longer blocks the resume: it engages into override (same as SET with gas)
    car = self._disengaged_after_brake()
    car.run(5)
    rets = car.run(5, btn=4, gas=True) + car.run(5, gas=True)
    assert _enables(rets) == 1
    car = self._disengaged_after_brake()
    car.run(5)
    rets = car.run(3, btn=4) + car.run(1, btn=4, gas=True) + car.run(5)
    assert _enables(rets) == 1

  def test_resume_after_down_press(self):
    # route 00000128 @138.3 / 139.2 shape: a down (SET) press, then pause/resume. The down press no longer engages (nor
    # leaves a panda grant); the pause press is a resume and engages (panda grants it too: lockstep test)
    car = _Car()
    car.run(10)
    rets = car.run(5, btn=2) + car.run(40)
    assert _enables(rets) == 0
    rets = car.run(5, btn=4) + car.run(5)
    assert _enables(rets) == 1

  @pytest.mark.parametrize("btn", [1, 2], ids=["RES", "SET"])
  def test_up_down_never_engage(self, btn):
    # buttons-v3: the up/down arrows only change the set speed. No press of them ever produces buttonEnable: short, long
    # (> 0.5 s), held across a brake release, with gas, before or after a set speed exists
    car = self._disengaged_after_brake()
    rets = car.run(5, btn=btn, brake=True) + car.run(5, btn=btn) + car.run(20)
    rets += car.run(5, btn=btn) + car.run(10)            # short
    rets += car.run(60, btn=btn) + car.run(10)           # long (1.2 s)
    rets += car.run(5, btn=btn, gas=True) + car.run(10, gas=True)
    assert _enables(rets) == 0
    # they still produce their set-speed button events
    assert ("accelCruise" if btn == 1 else "decelCruise") in {t for t, _ in _types(rets)}
    # pause/resume afterwards engages
    rets = car.run(5, btn=4) + car.run(5)
    assert _enables(rets) == 1

  @pytest.mark.parametrize("btn", [1, 2], ids=["RES", "SET"])
  def test_up_down_while_engaged_no_cancel(self, btn):
    car = _Car()
    car.run(10)
    car.engaged = True
    rets = car.run(60, btn=btn) + car.run(10)
    assert not any(t == "cancel" for t, _ in _types(rets))

  def test_inert_without_interceptor(self):
    # feature off: CF_Clu_CruiseSwState 4 stays a plain cancel, never resumeCruise
    CarInterface, CP, CP_SP = _get_ci(BASE_FINGERPRINT, None)
    CI = CarInterface(CP, CP_SP)
    t, types = 0, []
    for btn in [0] * 5 + [4] * 5 + [0] * 10:
      t += 10_000_000
      ret, _ = CI.update([(t, [PK.make_can_msg("CLU11", 0, {"CF_Clu_CruiseSwState": btn})])])
      types += [str(b.type) for b in ret.buttonEvents]
      assert not ret.buttonEnable
    assert types == ["cancel", "cancel"]


class TestPandaLockstep:
  """Randomized: the car-side tracker (CarState buttonEnable) and panda safety (controls_allowed grants) must agree on
  every engagement, sample for sample. Either one granting alone would be a divergence. buttons-v3: up/down (1/2) are in
  the stimulus and must never produce a grant on either side; every grant is a pause/resume (4) release."""
  @pytest.mark.parametrize("seed", range(6))
  def test_grants_agree(self, seed):
    from opendbc.car.structs import CarParams
    from opendbc.safety.tests.libsafety import libsafety_py
    from opendbc.safety.tests.common import CANPackerSafety
    rng = np.random.default_rng(seed)
    safety = libsafety_py.libsafety
    safety.set_current_safety_param_sp(int(HyundaiSafetyFlagsSP.NON_SCC | PEDAL_SP))
    safety.set_safety_hooks(CarParams.SafetyModel.hyundai, 0)
    safety.init_tests()
    spk = CANPackerSafety("hyundai_can_generated")
    ppk = CANPackerSafety(gi.GAS_INTERCEPTOR_DBC)

    def prx(name, vals, packer=spk, fix=None):
      assert safety.safety_rx_hook(packer.make_can_msg_safety(name, 0, vals, fix_checksum=fix))

    car = _Car()
    btn, brake, gas, main = 0, False, False, False
    grants_panda, grants_car, trace = [], [], []
    pause_rel, pause_started_engaged = pr.PAUSE_RELEASE_SAMPLES, False  # shadow of the press boundaries
    for k in range(3000):
      if rng.random() < 0.15:
        btn = int(rng.choice([0, 0, 0, 1, 2, 4, 4, 4]))
      brake = bool(rng.random() < 0.06) if rng.random() < 0.3 else brake
      gas = bool(rng.random() < 0.04) if rng.random() < 0.3 else gas
      main = bool(rng.random() < 0.3) if rng.random() < 0.01 else main
      # panda: same signals, pedal/brake/EMS16 before the CLU11 sample (as CarState sees them in the same update)
      before = safety.get_controls_allowed()
      safety.set_controls_allowed(car.engaged)
      prx("WHL_SPD11", {"WHL_SPD_FL": 15., "WHL_SPD_RR": 15., "WHL_SPD_AliveCounter_LSB": k % 4,
                        "WHL_SPD_AliveCounter_MSB": (k % 16) >> 2}, fix=_hyundai_checksum)
      prx("TCS13", {"DriverOverride": 2 if brake else 0, "AliveCounterTCS": k % 8}, fix=_hyundai_checksum)
      prx("EMS16", {"CRUISE_LAMP_M": main, "AliveCounter": k % 4}, fix=_hyundai_checksum)
      prx(gi.REMAPPED_IDS.sensor_msg, {"INTERCEPTOR_GAS": 2000 if gas else 0, "INTERCEPTOR_GAS2": 2000 if gas else 0,
                                       "PEDAL_COUNTER": k & 0xF}, packer=ppk, fix=_pedal_crc)
      mid = safety.get_controls_allowed()
      safety.safety_rx_hook(spk.make_can_msg_safety("CLU11", 0, {"CF_Clu_CruiseSwState": btn, "CF_Clu_AliveCnt1": k % 16}))
      after = safety.get_controls_allowed()
      if btn == 4:
        if pause_rel >= pr.PAUSE_RELEASE_SAMPLES:
          pause_started_engaged = car.engaged
        pause_rel = 0
      elif btn != 0:
        pause_rel = pr.PAUSE_RELEASE_SAMPLES
      elif pause_rel < pr.PAUSE_RELEASE_SAMPLES:
        pause_rel += 1
      if after and not mid:
        # a panda-only grant is allowed ONLY on the debounced release of a press that began while openpilot was engaged
        # (a disengage press: openpilot stays off, the unused permission is cleared by the heartbeat)
        grants_panda.append((k, btn == 0 and pause_rel == pr.PAUSE_RELEASE_SAMPLES and pause_started_engaged))
        # every panda grant, taken or not, is a pause/resume release (never an up/down edge)
        assert btn == 0 and pause_rel == pr.PAUSE_RELEASE_SAMPLES, trace[-25:]
      was_engaged = car.engaged
      rets = car.step(btn=btn, brake=brake, gas=gas, main=main)
      # an engage request only counts while disengaged (re-pressing SET while engaged is a set-speed change)
      if any(r.buttonEnable for r in rets) and not was_engaged:
        grants_car.append(k)
        # every openpilot engage request is carried by resumeCruise (never accelCruise / decelCruise)
        assert any(t == "resumeCruise" for t, _ in _types(rets)), trace[-25:]
      trace.append((k, btn, brake, gas, main, was_engaged, mid, after))
      # openpilot follows its own engage requests; brake disengages (pedalPressed), cancel/MAIN disengage
      if any(r.buttonEnable for r in rets) and not brake and not main:
        car.engaged = True
      if brake or main or any(str(b.type) == "cancel" and b.pressed for r in rets for b in r.buttonEvents):
        car.engaged = False
      del before
    assert len(grants_car) > 10, "fuzz never exercised an engagement"
    panda_k = {k for k, _ in grants_panda}
    benign = {k for k, ok in grants_panda if ok}
    # 1. every openpilot engagement coincides with a panda grant on the same sample (never engaged-but-refused)
    # 2. every panda grant openpilot didn't take is the release of a disengage press
    bad = sorted((set(grants_car) - panda_k) | (panda_k - set(grants_car) - benign))
    first = bad[0] if bad else None
    assert not bad, "\n".join(str(t) for t in trace if first - 25 <= t[0] <= first)
    assert benign, "fuzz never exercised a disengage-press release"


def _hyundai_checksum(msg):
  from opendbc.safety.tests.test_hyundai import checksum
  return checksum(msg)


def _pedal_crc(msg):
  addr, dat, bus = msg
  dat = bytearray(dat)
  dat[5] = crc8_pedal(dat[:5])
  return addr, dat, bus


# *** fork: FCA11 long braking (HyundaiFca11Brake) through the real CarController path ***

def _get_ci_fca11(fca11: str | None, pedal=STD):
  """CarInterface with the pedal armed and the FCA11 toggle as given (str or None)."""
  fingerprint = {i: {} for i in range(8)}
  fingerprint[0] = {**BASE_FINGERPRINT, **pedal}
  CarInterface = interfaces[CAR_UNDER_TEST]
  CP = CarInterface.get_params(CAR_UNDER_TEST, fingerprint, [], alpha_long=False, is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, CAR_UNDER_TEST, fingerprint, [], alpha_long=False, is_release_sp=False, docs=False)
  params_list: list = [{"HyundaiGasInterceptor": True}]
  if fca11 is not None:
    params_list.append({"HyundaiFca11Brake": fca11})
  setup_interfaces(CarInterface, CP, CP_SP, params_list)
  return CarInterface, CP, CP_SP


def _fca11_cam_frame(alive=5, warn=0):
  from opendbc.sunnypilot.car.hyundai.fca11_long import build_fca11_frame, Fca11MirrorFrame  # noqa: F401
  from opendbc.car.hyundai.hyundaican import hyundai_checksum
  b = bytearray(8)
  def setb(start, length, val):
    for i in range(length):
      pos = start + i
      b[pos // 8] |= ((int(val) >> i) & 1) << (pos % 8)
  setb(3, 2, warn)
  setb(18, 2, 2)      # FCA_Status
  setb(22, 3, 1)      # FCA_DrvSetStatus
  setb(35, 4, alive)
  setb(39, 9, 255)
  setb(48, 8, 254)
  b[4] |= 0x80        # undefined byte4 bit7 (always 1 on this car)
  b[7] = hyundai_checksum(bytes(b[:7]))
  return bytes(b)


class _Fca11CS:
  """Minimal CarState double for GasInterceptorCarController.create_gas_command."""
  def __init__(self, v_ego=20., cam=None, now=0, state=0, gear=None, brake=False, gas=False):
    self.out = structs.CarState()
    self.out.vEgo = v_ego
    self.out.gearShifter = structs.CarState.GearShifter.drive if gear is None else gear
    self.out.brakePressed = brake
    self.out.gasPressed = gas
    self.interceptor_state = state
    self.fca11_cam_frame = cam
    self.fca11_now_nanos = now
    self.fca11_lead_closing = False
    self.pedal_long_engaged = True
    self.pedal_fault_monitor = gi.PedalFaultMonitor()

  def __getattr__(self, name):
    # anything the controller reads that isn't set above (is_metric, etc.)
    if name == "is_metric":
      return False
    raise AttributeError(name)


class TestFca11LongControllerIntegration:
  """Hard-zeroed gas while braking + the 0x38D mirror frame on the wire, through create_gas_command."""

  def _cc(self, accel, long_active=True):
    cc = structs.CarControl()
    cc.enabled = cc.latActive = cc.longActive = long_active
    cc.actuators.accel = accel
    return cc.as_reader()

  def _ccr(self):
    CarInterface, CP, CP_SP = _get_ci_fca11("1")
    return CarInterface(CP, CP_SP), CP_SP

  def test_toggle_off_never_sends_fca11_and_keeps_pedal(self):
    CarInterface, CP, CP_SP = _get_ci_fca11("0")
    CI = CarInterface(CP, CP_SP)
    cs = _Fca11CS(cam=(0, _fca11_cam_frame()))
    sends = CI.CC.create_gas_command(self._cc(-2.0), cs, 0, 0)
    assert not any(addr == 0x38D for addr, _dat, _src in sends)
    # pedal still commanded at a positive accel (even frame = a send slot)
    sends = CI.CC.create_gas_command(self._cc(1.0), cs, 2, 0)
    assert any(addr == 0x200 for addr, _dat, _src in sends)

  def test_braking_zeroes_gas_and_sends_mirror(self):
    CI, CP_SP = self._ccr()
    cs = _Fca11CS(cam=(0, _fca11_cam_frame(alive=5)))
    # first get the pedal commanding
    for f in range(2, 120, 2):
      CI.CC.create_gas_command(self._cc(1.5), cs, f, 0)
    assert CI.CC.gas > 0.
    # now brake: gas must be exactly zero THIS frame and a 0x38D must appear
    sends = CI.CC.create_gas_command(self._cc(-2.0), cs, 120, 0)
    assert CI.CC.gas == 0.
    fca = [(a, d) for a, d, _ in sends if a == 0x38D]
    assert len(fca) == 1
    # pedal stream keeps flowing (a zero frame), no 0x38D duplicate
    assert sum(1 for a, _ in [(a, d) for a, d, _ in sends] if a == 0x200) == 1

  def test_no_mirror_without_fresh_camera(self):
    CI, CP_SP = self._ccr()
    cs = _Fca11CS(cam=None)
    sends = CI.CC.create_gas_command(self._cc(-2.0), cs, 0, 0)
    assert not any(addr == 0x38D for addr, _dat, _src in sends)
    # fresh-at-capture frame but the controller's "now" is >100 ms later -> stale (now_nanos is the freshness reference)
    cs = _Fca11CS(cam=(0, _fca11_cam_frame()))
    sends = CI.CC.create_gas_command(self._cc(-2.0), cs, 0, int(0.3 * 1e9))
    assert not any(addr == 0x38D for addr, _dat, _src in sends)
    # right at the freshness edge (100 ms) it still sends
    cs = _Fca11CS(cam=(0, _fca11_cam_frame()))
    sends = CI.CC.create_gas_command(self._cc(-2.0), cs, 0, int(0.100 * 1e9))
    assert any(addr == 0x38D for addr, _dat, _src in sends)

  def test_camera_request_cuts_and_handback(self):
    CI, CP_SP = self._ccr()
    cs = _Fca11CS(cam=(0, _fca11_cam_frame(warn=2)))  # the camera itself is braking
    sends = CI.CC.create_gas_command(self._cc(-2.0), cs, 0, 0)
    assert not any(addr == 0x38D for addr, _dat, _src in sends)
    assert CI.CC.fca11_brake.blocked_until_resume

  def test_mirror_frame_bytes(self):
    CI, CP_SP = self._ccr()
    cam = _fca11_cam_frame(alive=7)
    cs = _Fca11CS(cam=(0, cam))
    # drive send slots; the rate limiter ramps 4 -> 30 over ~7 frames
    dats = []
    for f in range(0, 32, 2):
      sends = CI.CC.create_gas_command(self._cc(-2.0), cs, f, 0)
      dats += [d for a, d, _ in sends if a == 0x38D]
    dat = dats[-1]
    from opendbc.sunnypilot.car.hyundai.fca11_long import _get_bits
    from opendbc.car.hyundai.hyundaican import hyundai_checksum
    assert _get_bits(dat, 0, 1) == 1          # Prefill
    assert _get_bits(dat, 3, 2) == 3          # Warn 3
    assert _get_bits(dat, 20, 1) == 1         # CmdAct
    assert _get_bits(dat, 31, 1) == 0         # DecCmdAct stays 0
    assert _get_bits(dat, 35, 4) == (7 + 1) & 0xF  # alive + 1, byte4 bits 3-6
    assert dat[7] == hyundai_checksum(dat[:7])
    assert _get_bits(dat, 8, 8) == 20         # -2.0 -> clamped 0.20 g raw (STANDARD default) once ramped
    assert _get_bits(dats[0], 8, 8) == 4      # first frame is rate-limited to +0.04 g
    assert dat[4] & 0x80                       # undefined byte4 bit7 preserved

  def test_personality_reaches_gas_command(self):
    # the create_gas_command personality arg must reach Fca11LongBrake: the emitted cap is the tier cap
    from opendbc.sunnypilot.car.hyundai.fca11_long import _get_bits, PERSONALITY_MAX_DEC
    for p, cap in ((2, 12), (1, 20), (0, 30)):
      CI, CP_SP = self._ccr()
      cs = _Fca11CS(cam=(0, _fca11_cam_frame()))
      dats = []
      for f in range(0, 32, 2):
        sends = CI.CC.create_gas_command(self._cc(-2.0), cs, f, 0, p)
        dats += [d for a, d, _ in sends if a == 0x38D]
      assert _get_bits(dats[-1], 8, 8) == cap == PERSONALITY_MAX_DEC[p], (p, cap)
