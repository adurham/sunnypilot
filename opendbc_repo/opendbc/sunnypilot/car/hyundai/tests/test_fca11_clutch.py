"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

18x (b): the CLUTCH-TORQUE channel. `CS.fca11_clutch_tq_nm` is TCU_DCT13 0x162 `Clutch_Driving_Tq` (10-bit raw, offset
-512 Nm) published by carstate_ext.py so the k="b" level trace of the dynamic-release build can show, from ordinary
drives, that the clutch is OPEN through a crawl hold (+26 Nm = creep torque live, -20 Nm = open). The checks here:
  * the DBC layout the "raw10 - 512" statement rests on;
  * the REAL CarInterface.update -> CarState -> CarStateExt path publishing it (and None when the frame is missing);
  * an implausible value (a silent bus leaves the parser at raw 0 = -512 Nm) reads as MISSING, not as a torque;
  * the frame perturbs NO existing CarState field (byte-identical CarState with and without it).
"""
from opendbc.can import CANPacker
from opendbc.can.dbc import DBC
from opendbc.car.car_helpers import interfaces
from opendbc.car.hyundai.values import CAR
from opendbc.sunnypilot.car.interfaces import setup_interfaces

CAR_UNDER_TEST = CAR.HYUNDAI_ELANTRA_2022_NON_SCC
BASE_FINGERPRINT = {0x260: 8, 0x371: 8, 0x386: 8, 0x394: 8, 0x251: 8, 0x4F1: 4, 0x340: 8}
CLUTCH_ADDR = 0x162
PACKER = CANPacker("hyundai_can_generated")


def _ci(fca11=True):
  fingerprint = {i: {} for i in range(8)}
  fingerprint[0] = {**BASE_FINGERPRINT, 0x201: 6}
  CarInterface = interfaces[CAR_UNDER_TEST]
  CP = CarInterface.get_params(CAR_UNDER_TEST, fingerprint, [], alpha_long=False, is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, CAR_UNDER_TEST, fingerprint, [], alpha_long=False, is_release_sp=False, docs=False)
  params_list: list = [{"HyundaiGasInterceptor": True}]
  if fca11:
    params_list.append({"HyundaiFca11Brake": "1"})
  setup_interfaces(CarInterface, CP, CP_SP, params_list)
  return CarInterface(CP, CP_SP), CP_SP


def _drive(frames=()):
  """Three 100 Hz ticks of the real CarInterface.update; ``frames`` are appended to every bus-0 packet.

  Returns the interface (whose ``CS`` is the CarState object the car layer publishes the extra channel on - reading
  ``ret`` instead is the capnp struct, which has no such member), the published CarState struct, and CarParamsSP."""
  CI, CP_SP = _ci()
  ret = None
  for i in range(3):
    t = (i + 1) * int(0.02e9)
    ret = CI.update([(t, [(addr, dat, 0) for addr, dat in frames])])[0]
  return CI, ret, CP_SP


class TestDbcLayout:
  def test_clutch_channel_is_the_measured_10bit_raw_minus_512(self):
    msg = DBC("hyundai_can_generated").name_to_msg["TCU_DCT13"]
    assert msg.address == CLUTCH_ADDR
    sig = msg.sigs["Clutch_Driving_Tq"]
    assert (sig.start_bit, sig.size) == (0, 10)
    assert (sig.factor, sig.offset) == (1.0, -512.0)
    assert sig.is_signed is False


class TestCarStateChannel:
  def test_clutch_torque_is_published_from_the_real_carstate_path(self):
    CI, _, CP_SP = _drive([(CLUTCH_ADDR, PACKER.pack(CLUTCH_ADDR, {"Clutch_Driving_Tq": -20.0}))])
    assert CP_SP.fca11Brake
    assert CI.CS.fca11_clutch_tq_nm == -20.0, CI.CS.fca11_clutch_tq_nm
    # the creep-torque direction is just as visible (the measurement's +26 Nm case)
    CI, _, _ = _drive([(CLUTCH_ADDR, PACKER.pack(CLUTCH_ADDR, {"Clutch_Driving_Tq": 26.0}))])
    assert CI.CS.fca11_clutch_tq_nm == 26.0

  def test_a_missing_frame_reads_none_not_a_torque(self):
    CI, _, _ = _drive()
    assert CI.CS.fca11_clutch_tq_nm is None, "no 0x162 this tick -> None, never a stale or zero torque"

  def test_an_implausible_value_reads_none(self):
    # a silent bus leaves the parser at raw 0, i.e. -512 Nm: out of any driveline range, so it must NOT be logged as one
    CI, _, _ = _drive([(CLUTCH_ADDR, PACKER.pack(CLUTCH_ADDR, {"Clutch_Driving_Tq": -512.0}))])
    assert CI.CS.fca11_clutch_tq_nm is None

  def test_the_clutch_frame_perturbs_no_existing_carstate_field(self):
    CI, with_frame, _ = _drive([(CLUTCH_ADDR, PACKER.pack(CLUTCH_ADDR, {"Clutch_Driving_Tq": -20.0}))])
    _, without, _ = _drive()
    assert CI.CS.fca11_clutch_tq_nm == -20.0, "the frame under test really was decoded"
    assert with_frame.to_bytes() == without.to_bytes()
