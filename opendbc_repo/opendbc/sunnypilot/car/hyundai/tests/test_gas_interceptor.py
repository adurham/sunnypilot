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
from opendbc.car.car_helpers import interfaces
from opendbc.car.hyundai.values import CAR
from opendbc.sunnypilot.car import crc8_pedal, create_gas_interceptor_command
from opendbc.sunnypilot.car.hyundai import gas_interceptor as gi
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
    # Bench-measured scaling: 0.15 -> track A 788, track B 400 (the old Toyota-copied scaling sent B=1188 here).
    _, dat, _ = create_gas_interceptor_command(CANPacker(gi.GAS_INTERCEPTOR_DBC), gi.MAX_INTERCEPTOR_GAS, 0)
    track1, track2 = (dat[0] << 8) | dat[1], (dat[2] << 8) | dat[3]
    assert abs(track1 - 788) <= 2 and abs(track2 - 400) <= 3
    assert 0. < gi.MAX_INTERCEPTOR_GAS <= 0.3

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
    assert CP.minEnableSpeed == pytest.approx(gi.HYUNDAI_GAS_INTERCEPTOR_MIN_ENABLE_SPEED, abs=1e-4)  # capnp float32
    assert CP.minEnableSpeed > 10.
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

  @pytest.mark.parametrize("pedal, ids", [(STD, gi.STANDARD_IDS), (REM, gi.REMAPPED_IDS)], ids=["standard", "remapped"])
  def test_clear_frame_on_fault(self, pedal, ids):
    CI, CC, CC_SP, ids = self._setup(ids, pedal)
    out = self._drive(CI, CC, CC_SP, ids, [0] * 20 + [3] * 20 + [0] * 20)
    cmds = [(f, _cmd_fields(d)) for f, (d, _) in enumerate(out) if d is not None]
    assert all(f % 2 == 0 for f, _ in cmds) and len(cmds) == 30
    # counter continuous across clear frames
    assert all((b[1][3] - a[1][3]) % 16 == 1 for a, b in zip(cmds, cmds[1:], strict=False))
    zero = [f for f, c in cmds if c[2] == 0]
    assert zero == [20, 26, 32, 38]
    for f, (a, b, en, _) in cmds:
      if f in zero:
        assert (a, b, en) == (0, 0, 0)  # the firmware's clear frame: ENABLE=0, both tracks 0
      else:
        assert en == 1 and abs(a - 788) <= 2  # commanding at the cap
    assert not any(acc for _, acc in out)
    # every frame (incl. clears) has a valid pedal checksum
    assert all(d[5] == crc8_pedal(d[:5]) for d, _ in out if d is not None)

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


class TestCancelBursts:
  def _cancel_frames(self, pedal: dict | None, n: int = 400) -> list[int]:
    fp = BASE_FINGERPRINT if pedal is None else {**BASE_FINGERPRINT, **pedal}
    CarInterface, CP, CP_SP = _get_ci(fp, None if pedal is None else True)
    CI = CarInterface(CP, CP_SP)
    CC = structs.CarControl()
    CC.cruiseControl.cancel = True
    frames = []
    for i in range(n):
      CI.update([])
      _, sends = CI.apply(CC.as_reader(), structs.CarControlSP(), i)
      if any(a == 0x4F1 for a, _, _ in sends):
        frames.append(i)
    return frames

  def test_burst_pattern(self):
    pat = [gi.cancel_burst_active(n) for n in range(gi.CANCEL_CYCLE_FRAMES * 2)]
    assert sum(pat) == 2 * gi.CANCEL_BURST_FRAMES * len(gi.CANCEL_PAUSE_FRAMES)
    assert pat[:5] == [True] * 5 and not any(pat[5:20]) and all(pat[20:25]) and not any(pat[25:45]) and all(pat[45:50])
    assert gi.CANCEL_CYCLE_FRAMES == 75 and not any(pat[50:75]) and all(pat[75:80])
    # pauses within the specified 150-250 ms
    assert all(15 <= p <= 25 for p in gi.CANCEL_PAUSE_FRAMES) and 3 <= gi.CANCEL_BURST_FRAMES <= 5

  def test_interceptor_cancel_bursted(self):
    frames = self._cancel_frames(REM)
    # same 10-frame delay as before, then bursts of 5 consecutive frames with 150/200/250 ms pauses
    assert frames[0] == 10
    bursts = []
    for f in frames:
      if bursts and f == bursts[-1][-1] + 1:
        bursts[-1].append(f)
      else:
        bursts.append([f])
    assert all(len(b) == gi.CANCEL_BURST_FRAMES for b in bursts[:-1])
    gaps = [b[0] - a[-1] - 1 for a, b in zip(bursts, bursts[1:], strict=False)]
    assert gaps[:6] == [15, 20, 25, 15, 20, 25]
    assert len(frames) < 0.25 * 390  # duty 20 % vs 100 % before

  def test_cancel_stops_immediately(self):
    CarInterface, CP, CP_SP = _get_ci({**BASE_FINGERPRINT, **REM}, True)
    CI = CarInterface(CP, CP_SP)
    CC = structs.CarControl()
    for i in range(60):
      CC.cruiseControl.cancel = i < 30
      CI.update([])
      _, sends = CI.apply(CC.as_reader(), structs.CarControlSP(), i)
      if i >= 30:
        assert not any(a == 0x4F1 for a, _, _ in sends)

  def test_non_interceptor_cancel_unchanged(self):
    # stock (pcmCruise) cars keep the continuous stream after the delay
    frames = self._cancel_frames(None, 100)
    assert frames == list(range(10, 100))
