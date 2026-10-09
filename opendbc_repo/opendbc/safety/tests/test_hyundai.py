#!/usr/bin/env python3
from opendbc.testing import parameterized_class
import os
import random
import re
import unittest

from opendbc.car.hyundai.values import HyundaiSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.safety.tests.gas_interceptor_common import GasInterceptorSafetyTest
from opendbc.safety.tests.hyundai_common import Buttons, HyundaiButtonBase, HyundaiLongitudinalBase

import numpy as np

from opendbc.can import CANPacker
from opendbc.sunnypilot.car import crc8_pedal, create_gas_interceptor_command
from opendbc.sunnypilot.car.hyundai.gas_interceptor import GAS_INTERCEPTOR_DBC, HYUNDAI_GAS_INTERCEPTOR_THRESHOLD, \
                                                         MAX_INTERCEPTOR_GAS, REMAPPED_IDS, STANDARD_IDS

from opendbc.sunnypilot.car.hyundai.values import HyundaiSafetyFlagsSP, HYUNDAI_FCA11_TEST_MAX_DEC, HYUNDAI_FCA11_ROLL_MAX_DEC, \
  HYUNDAI_FCA11_ROLL_MIN_SPEED_KPH, HYUNDAI_FCA11_ROLL_MAX_SPEED_KPH, HYUNDAI_FCA11_LONG_MAX_DEC, \
  HYUNDAI_FCA11_LONG_RATE_STEP, HYUNDAI_LKAS_PARK_MAX_TORQUE, HYUNDAI_LKAS_PARK_RATE_UP, \
  HYUNDAI_LKAS_PARK_RATE_DOWN, HYUNDAI_LKAS_PARK_MAX_ARM_S, HYUNDAI_LKAS_PARK_WHEEL_MAX_RAW, HYUNDAI_LKAS_PARK_MAX_ANGLE_DEG, \
  HYUNDAI_LKAS_PARK_MAX_DRIVER_TQ_NM
from opendbc.sunnypilot.car.hyundai.pause_resume import PAUSE_RELEASE_SAMPLES as HYUNDAI_PAUSE_RELEASE_SAMPLES

# a nonzero pedal command inside openpilot's cap (fraction of travel), for tests about *whether* gas may be sent
GAS_ON = 0.1

HYUNDAI_COMMON_H = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "modes", "hyundai_common.h")
HYUNDAI_H = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "modes", "hyundai.h")

# C-only FCA11-long constants (seconds), read straight from the header so the safety tests pin them
HYUNDAI_FCA11_LONG_TX_STALE_US = 100000


def _c_define(path: str, name: str) -> int:
  with open(path) as f:
    m = re.search(rf"#define\s+{name}\s+(\d+)U?", f.read())
  assert m is not None, f"{name} not found in {path}"
  return int(m.group(1))


# 0038: the gas hold-off and the host-liveness watchdog, pinned to the header values
HYUNDAI_FCA11_LONG_GAS_HOLDOFF_US = _c_define(HYUNDAI_H, "HYUNDAI_FCA11_LONG_GAS_HOLDOFF_US")
HYUNDAI_FCA11_LONG_HOST_HB_TIMEOUT_US = _c_define(HYUNDAI_H, "HYUNDAI_FCA11_LONG_HOST_HB_TIMEOUT_US")
HYUNDAI_FCA11_LONG_BRAKE_THROTTLE_EXCL_US = _c_define(HYUNDAI_H, "HYUNDAI_FCA11_LONG_BRAKE_THROTTLE_EXCL_US")


def fc_cancel_episode(safety, rx, ems16, clu_raw, brake_msg, period_us=19800, n=150):
  """Engaged long (panda grant + heartbeat), then the factory cruise comes on and stays on for n cluster frames.
  Returns the number of CLU11 frames panda transmitted by itself. Shared by the non-pedal / test-mode inertness tests."""
  safety.set_timer(1_000_000)
  rx(brake_msg(False))
  safety.set_controls_allowed(True)
  safety.set_heartbeat_engaged(True)
  safety.clear_self_tx()
  for k in range(n):
    t = 2_000_000 + k * period_us
    safety.set_timer(t)
    rx(ems16(True, True))
    safety.set_timer(t + 10_000)
    rx(clu_raw(k & 0xF))
  return safety.get_self_tx_count()

# LDA button availability
LDA_BUTTON = [
  {"SAFETY_PARAM_SP": HyundaiSafetyFlagsSP.DEFAULT},
  {"SAFETY_PARAM_SP": HyundaiSafetyFlagsSP.HAS_LDA_BUTTON},
]

# All combinations of non-SCC HEV/PHEV/EV cars
_ALL_NON_SCC_HEV_EV_COMBOS = [
  # Hybrid
  {"PCM_STATUS_MSG": ("E_CRUISE_CONTROL", "CRUISE_LAMP_S"),
   "ACC_STATE_MSG": ("E_CRUISE_CONTROL", "CRUISE_LAMP_M"),
   "GAS_MSG": ("E_EMS11", "CR_Vcu_AccPedDep_Pos"),
   "SAFETY_PARAM": HyundaiSafetyFlags.HYBRID_GAS},
  # EV
  {"PCM_STATUS_MSG": ("LABEL11", "CC_ACT"),
   "ACC_STATE_MSG": ("LABEL11", "CC_React"),
   "GAS_MSG": ("E_EMS11", "Accel_Pedal_Pos"),
   "SAFETY_PARAM": HyundaiSafetyFlags.EV_GAS},
]
ALL_NON_SCC_HEV_EV_COMBOS = [{**p, **lda} for lda in LDA_BUTTON for p in _ALL_NON_SCC_HEV_EV_COMBOS]


# 4 bit checkusm used in some hyundai messages
# lives outside the can packer because we never send this msg
def checksum(msg):
  addr, dat, bus = msg

  chksum = 0
  if addr == 0x386:
    for i, b in enumerate(dat):
      for j in range(8):
        # exclude checksum and counter bits
        if (i != 1 or j < 6) and (i != 3 or j < 6) and (i != 5 or j < 6) and (i != 7 or j < 6):
          bit = (b >> j) & 1
        else:
          bit = 0
        chksum += bit
    chksum = (chksum ^ 9) & 0xF
    ret = bytearray(dat)
    ret[5] |= (chksum & 0x3) << 6
    ret[7] |= (chksum & 0xc) << 4
  else:
    for i, b in enumerate(dat):
      if addr in [0x260, 0x421] and i == 7:
        b &= 0x0F if addr == 0x421 else 0xF0
      elif addr == 0x394 and i == 6:
        b &= 0xF0
      elif addr == 0x394 and i == 7:
        continue
      chksum += sum(divmod(b, 16))
    chksum = (16 - chksum) % 16
    ret = bytearray(dat)
    ret[6 if addr == 0x394 else 7] |= chksum << (4 if addr == 0x421 else 0)

  return addr, ret, bus


@parameterized_class(LDA_BUTTON)
class TestHyundaiSafety(HyundaiButtonBase, common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest, common.SteerRequestCutSafetyTest):
  TX_MSGS = [[0x340, 0], [0x4F1, 0], [0x485, 0]]
  STANDSTILL_THRESHOLD = 12  # 0.375 kph
  RELAY_MALFUNCTION_ADDRS = {0: (0x340, 0x485)}  # LKAS11
  FWD_BLACKLISTED_ADDRS = {2: [0x340, 0x485]}

  MAX_RATE_UP = 3
  MAX_RATE_DOWN = 7
  MAX_TORQUE_LOOKUP = [0], [384]
  MAX_RT_DELTA = 112
  DRIVER_TORQUE_ALLOWANCE = 50
  DRIVER_TORQUE_FACTOR = 2

  # Safety around steering req bit
  MIN_VALID_STEERING_FRAMES = 89
  MAX_INVALID_STEERING_FRAMES = 2

  cnt_gas = 0
  cnt_speed = 0
  cnt_brake = 0
  cnt_cruise = 0
  cnt_button = 0

  SAFETY_PARAM_SP: int = 0

  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiSafety":
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_can_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(self.SAFETY_PARAM_SP)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, 0)
    self.safety.init_tests()

  def _button_msg(self, buttons, main_button=0, bus=0):
    values = {"CF_Clu_CruiseSwState": buttons, "CF_Clu_CruiseSwMain": main_button, "CF_Clu_AliveCnt1": self.cnt_button}
    self.__class__.cnt_button += 1
    return self.packer.make_can_msg_safety("CLU11", bus, values)

  def test_fc_cancel_never_outside_pedal_mode(self):
    # the timed factory-cruise CANCEL exists only in gas-interceptor mode: every other Hyundai mode (stock, SCC long,
    # camera SCC, legacy, non-SCC without pedal, the FCA11 brake test) never transmits a CLU11 by itself
    # (init_tests() resets current_safety_param_sp, so the mode is identified by the test class)
    if isinstance(self, GasInterceptorSafetyTest):
      raise unittest.SkipTest("pedal mode: covered by the test_fc_cancel_* tests")

    def ems16(main, active):
      values = {"CRUISE_LAMP_M": main, "CRUISE_LAMP_S": active, "AliveCounter": self.cnt_gas % 4}
      self.__class__.cnt_gas += 1
      return self.packer.make_can_msg_safety("EMS16", 0, values, fix_checksum=checksum)

    def clu_raw(counter):
      par = bin(counter).count("1") & 1
      return libsafety_py.make_CANPacket(0x4F1, 0, bytes([par << 5, 0x7A, 0x00, (counter << 4) | 1]))

    self.assertEqual(0, fc_cancel_episode(self.safety, self._rx, ems16, clu_raw, self._user_brake_msg))

  def _user_gas_msg(self, gas):
    values = {"CF_Ems_AclAct": gas, "AliveCounter": self.cnt_gas % 4}
    self.__class__.cnt_gas += 1
    return self.packer.make_can_msg_safety("EMS16", 0, values, fix_checksum=checksum)

  def _user_brake_msg(self, brake):
    values = {"DriverOverride": 2 if brake else random.choice((0, 1, 3)),
              "AliveCounterTCS": self.cnt_brake % 8}
    self.__class__.cnt_brake += 1
    return self.packer.make_can_msg_safety("TCS13", 0, values, fix_checksum=checksum)

  def _speed_msg(self, speed):
    # safety doesn't scale, so undo the scaling
    values = {"WHL_SPD_%s" % s: speed * 0.03125 for s in ["FL", "FR", "RL", "RR"]}
    values["WHL_SPD_AliveCounter_LSB"] = (self.cnt_speed % 16) & 0x3
    values["WHL_SPD_AliveCounter_MSB"] = (self.cnt_speed % 16) >> 2
    self.__class__.cnt_speed += 1
    return self.packer.make_can_msg_safety("WHL_SPD11", 0, values, fix_checksum=checksum)

  def _pcm_status_msg(self, enable):
    values = {"ACCMode": enable, "CR_VSM_Alive": self.cnt_cruise % 16}
    self.__class__.cnt_cruise += 1
    return self.packer.make_can_msg_safety("SCC12", self.SCC_BUS, values, fix_checksum=checksum)

  def _torque_driver_msg(self, torque):
    values = {"CR_Mdps_StrColTq": torque}
    return self.packer.make_can_msg_safety("MDPS12", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"CR_Lkas_StrToqReq": torque, "CF_Lkas_ActToi": steer_req}
    return self.packer.make_can_msg_safety("LKAS11", 0, values)

  def _acc_state_msg(self, enable):
    values = {"MainMode_ACC": enable}
    return self.packer.make_can_msg_safety("SCC11", self.SCC_BUS, values)

  def _lkas_button_msg(self, enabled):
    if self.SAFETY_PARAM_SP & HyundaiSafetyFlagsSP.HAS_LDA_BUTTON:
      values = {"LDA_BTN": enabled}
      return self.packer.make_can_msg_safety("BCM_PO_11", 0, values)
    else:
      raise NotImplementedError

  def _main_cruise_button_msg(self, enabled):
    return self._button_msg(0, enabled)

  def test_pcm_main_cruise_state_availability(self):
    """Test that ACC main state is correctly set when receiving SCC11 (0x420), toggling HYUNDAI_LONG flag.

    Only applicable to SCC-based cars. Non-SCC cars use different messages for ACC state
    and their rx_checks don't include SCC11 after mode reconfiguration.
    """
    if any('NonSCC' in cls.__name__ for cls in type(self).__mro__):
      raise unittest.SkipTest("Non-SCC cars use different ACC state messages, not SCC11")

    prior_safety_mode = self.safety.get_current_safety_mode()
    prior_safety_param = self.safety.get_current_safety_param()
    safety_param_sp = self.SAFETY_PARAM_SP

    for hyundai_longitudinal in (True, False):
      with self.subTest("hyundai_longitudinal", hyundai_longitudinal=hyundai_longitudinal):
        self.safety.set_current_safety_param_sp(self.SAFETY_PARAM_SP)
        self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, 0 if hyundai_longitudinal else HyundaiSafetyFlags.LONG)
        for should_turn_acc_main_on in (True, False):
          with self.subTest("acc_main_on", should_turn_acc_main_on=should_turn_acc_main_on):
            self.safety.set_acc_main_on(False)
            self._rx(self._acc_state_msg(should_turn_acc_main_on))
            expected_acc_main = should_turn_acc_main_on and hyundai_longitudinal
            self.assertEqual(expected_acc_main, self.safety.get_acc_main_on())
    self.safety.set_current_safety_param_sp(safety_param_sp)
    self.safety.set_safety_hooks(prior_safety_mode, prior_safety_param)
    self.safety.init_tests()

  def test_enable_control_allowed_with_mads_button(self):
    """Toggle MADS with MADS button, testing HAS_LDA_BUTTON param gating."""
    default_safety_mode = self.safety.get_current_safety_mode()
    default_safety_param = self.safety.get_current_safety_param()
    default_safety_param_sp = self.SAFETY_PARAM_SP

    try:
      self._lkas_button_msg(False)
    except NotImplementedError as err:
      raise unittest.SkipTest("Skipping test because LDA button is not supported") from err

    # CameraSCC rx_checks always include BCM_PO_11 regardless of HAS_LDA_BUTTON param,
    # so we can only test the has_lda_button=True case for CameraSCC.
    camera_scc = bool(default_safety_param & HyundaiSafetyFlags.CAMERA_SCC)
    lda_button_variants = [True] if camera_scc else [True, False]

    try:
      for enable_mads in (True, False):
        with self.subTest("enable_mads", mads_enabled=enable_mads):
          for has_lda_button_param in lda_button_variants:
            with self.subTest("has_lda_button", has_lda_button_param=has_lda_button_param):
              has_lda_button = HyundaiSafetyFlagsSP.HAS_LDA_BUTTON if has_lda_button_param else 0
              sp = (default_safety_param_sp & ~HyundaiSafetyFlagsSP.HAS_LDA_BUTTON) | has_lda_button
              self.safety.set_current_safety_param_sp(sp)
              self.safety.set_safety_hooks(default_safety_mode, default_safety_param)
              self.safety.init_tests()

              self.safety.set_controls_allowed(False)
              self.safety.set_acc_main_on(False)
              self.safety.set_controls_allowed_lateral(False)
              self.safety.set_mads_params(enable_mads, False, False)
              self.assertEqual(enable_mads, self.safety.get_enable_mads())

              self._rx(self._lkas_button_msg(True))
              self._rx(self._lkas_button_msg(False))
              self.assertEqual(enable_mads and has_lda_button_param, self.safety.get_controls_allowed_lateral())
    finally:
      self.safety.set_current_safety_param_sp(default_safety_param_sp)


@parameterized_class(LDA_BUTTON)
class TestHyundaiSafetyAltLimits(TestHyundaiSafety):
  MAX_RATE_UP = 2
  MAX_RATE_DOWN = 3
  MAX_TORQUE_LOOKUP = [0], [270]

  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiSafetyAltLimits":
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_can_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(self.SAFETY_PARAM_SP)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, HyundaiSafetyFlags.ALT_LIMITS)
    self.safety.init_tests()


@parameterized_class(LDA_BUTTON)
class TestHyundaiSafetyAltLimits2(TestHyundaiSafety):
  MAX_RATE_UP = 2
  MAX_RATE_DOWN = 3
  MAX_TORQUE_LOOKUP = [0], [170]

  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiSafetyAltLimits2":
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_can_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(self.SAFETY_PARAM_SP)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, HyundaiSafetyFlags.ALT_LIMITS_2)
    self.safety.init_tests()


@parameterized_class(LDA_BUTTON)
class TestHyundaiSafetyCameraSCC(TestHyundaiSafety):
  BUTTONS_TX_BUS = 2  # tx on 2, rx on 0
  SCC_BUS = 2  # rx on 2

  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiSafetyCameraSCC":
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_can_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(self.SAFETY_PARAM_SP)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, HyundaiSafetyFlags.CAMERA_SCC)
    self.safety.init_tests()

  def test_pcm_main_cruise_state_availability(self):
    """
    Test that ACC main state is correctly set when receiving 0x420 message.
    For camera SCC, ACC main should always be on when receiving 0x420 message
    """

    for should_turn_acc_main_on in (True, False):
      with self.subTest("acc_main_on", should_turn_acc_main_on=should_turn_acc_main_on):
        self._rx(self._acc_state_msg(should_turn_acc_main_on))
        self.assertEqual(should_turn_acc_main_on, self.safety.get_acc_main_on())


@parameterized_class(LDA_BUTTON)
class TestHyundaiSafetyFCEV(TestHyundaiSafety):
  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiSafetyFCEV":
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_can_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(self.SAFETY_PARAM_SP)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, HyundaiSafetyFlags.FCEV_GAS)
    self.safety.init_tests()

  def _user_gas_msg(self, gas):
    values = {"ACCELERATOR_PEDAL": gas}
    return self.packer.make_can_msg_safety("FCEV_ACCELERATOR", 0, values)


class TestHyundaiLegacySafety(TestHyundaiSafety):
  def setUp(self):
    self.packer = CANPackerSafety("hyundai_can_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiLegacy, 0)
    self.safety.init_tests()


class TestHyundaiLegacySafetyEV(TestHyundaiSafety):
  def setUp(self):
    self.packer = CANPackerSafety("hyundai_can_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiLegacy, HyundaiSafetyFlags.EV_GAS)
    self.safety.init_tests()

  def _user_gas_msg(self, gas):
    values = {"Accel_Pedal_Pos": gas}
    return self.packer.make_can_msg_safety("E_EMS11", 0, values, fix_checksum=checksum)


class TestHyundaiLegacySafetyHEV(TestHyundaiSafety):
  def setUp(self):
    self.packer = CANPackerSafety("hyundai_can_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiLegacy, HyundaiSafetyFlags.HYBRID_GAS)
    self.safety.init_tests()

  def _user_gas_msg(self, gas):
    values = {"CR_Vcu_AccPedDep_Pos": gas}
    return self.packer.make_can_msg_safety("E_EMS11", 0, values, fix_checksum=checksum)


@parameterized_class(LDA_BUTTON)
class TestHyundaiLongitudinalSafety(HyundaiLongitudinalBase, TestHyundaiSafety):
  TX_MSGS = [[0x340, 0], [0x4F1, 0], [0x485, 0], [0x420, 0], [0x421, 0], [0x50A, 0], [0x389, 0], [0x4A2, 0], [0x38D, 0], [0x483, 0], [0x7D0, 0]]

  FWD_BLACKLISTED_ADDRS = {2: [0x340, 0x485, 0x421, 0x420, 0x50A, 0x389]}

  RELAY_MALFUNCTION_ADDRS = {0: (0x340, 0x485, 0x421, 0x420, 0x50A, 0x389)}  # LKAS11, LFAHDA_MFC, SCC12, SCC11, SCC13, SCC14

  DISABLED_ECU_UDS_MSG = (0x7D0, 0)
  DISABLED_ECU_ACTUATION_MSG = (0x421, 0)

  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiLongitudinalSafety":
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_can_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(self.SAFETY_PARAM_SP)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, HyundaiSafetyFlags.LONG)
    self.safety.init_tests()

  def _accel_msg(self, accel, aeb_req=False, aeb_decel=0, aeb_stop_req=False):
    values = {
      "aReqRaw": accel,
      "aReqValue": accel,
      "AEB_CmdAct": int(aeb_req),
      "AEB_StopReq": int(aeb_stop_req),
      "CR_VSM_DecCmd": aeb_decel,
    }
    return self.packer.make_can_msg_safety("SCC12", self.SCC_BUS, values)

  def _fca11_msg(self, idx=0, vsm_aeb_req=False, fca_aeb_req=False, aeb_decel=0):
    values = {
      "CR_FCA_Alive": idx % 0xF,
      "FCA_Status": 2,
      "CR_VSM_DecCmd": aeb_decel,
      "CF_VSM_DecCmdAct": int(vsm_aeb_req),
      "FCA_CmdAct": int(fca_aeb_req),
    }
    return self.packer.make_can_msg_safety("FCA11", 0, values)

  def _tx_acc_state_msg(self, enable):
    values = {"MainMode_ACC": enable}
    return self.packer.make_can_msg_safety("SCC11", 0, values)

  def test_no_aeb_fca11(self):
    self.assertTrue(self._tx(self._fca11_msg()))
    self.assertFalse(self._tx(self._fca11_msg(vsm_aeb_req=True)))
    self.assertFalse(self._tx(self._fca11_msg(fca_aeb_req=True)))
    self.assertFalse(self._tx(self._fca11_msg(aeb_decel=1.0)))

  def test_no_aeb_scc12(self):
    self.assertTrue(self._tx(self._accel_msg(0)))
    self.assertFalse(self._tx(self._accel_msg(0, aeb_req=True)))
    self.assertFalse(self._tx(self._accel_msg(0, aeb_decel=1.0)))
    self.assertFalse(self._tx(self._accel_msg(0, aeb_stop_req=True)))

  def test_fca11_rule_pinned(self):
    # Pins TODAY's FCA11 rule (outside the FCA11 brake test): only CR_VSM_DecCmd / FCA_CmdAct / CF_VSM_DecCmdAct are
    # blocked; prefill, HBA and StopReq alone pass. FCA11_BRAKE_TEST must not change any of it here: it is ignored
    # whenever hyundai_longitudinal is set.
    for extra_sp in (0, HyundaiSafetyFlagsSP.FCA11_BRAKE_TEST, HyundaiSafetyFlagsSP.FCA11_BRAKE_TEST | HyundaiSafetyFlagsSP.NON_SCC):
      with self.subTest(extra_sp=extra_sp):
        self.safety.set_current_safety_param_sp(self.SAFETY_PARAM_SP | extra_sp)
        self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, HyundaiSafetyFlags.LONG)
        for sig in ("CF_VSM_Prefill", "CF_VSM_HBACmd", "FCA_StopReq"):
          self.assertTrue(self._tx(self.packer.make_can_msg_safety("FCA11", 0, {sig: 1})), sig)
        self.assertTrue(self._tx(self._fca11_msg()))
        self.assertFalse(self._tx(self._fca11_msg(aeb_decel=0.01)))
        self.assertFalse(self._tx(self._fca11_msg(vsm_aeb_req=True)))
        self.assertFalse(self._tx(self._fca11_msg(fca_aeb_req=True)))


class TestHyundaiLongitudinalSafetyCameraSCC(HyundaiLongitudinalBase, TestHyundaiSafety):
  TX_MSGS = [[0x340, 0], [0x4F1, 2], [0x485, 0], [0x420, 0], [0x421, 0], [0x50A, 0], [0x389, 0], [0x4A2, 0]]

  FWD_BLACKLISTED_ADDRS = {2: [0x340, 0x485, 0x420, 0x421, 0x50A, 0x389]}
  RELAY_MALFUNCTION_ADDRS = {0: (0x340, 0x485, 0x421, 0x420, 0x50A, 0x389)}  # LKAS11, LFAHDA_MFC, SCC12, SCC11, SCC13, SCC14

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_can_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(HyundaiSafetyFlagsSP.HAS_LDA_BUTTON)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, HyundaiSafetyFlags.LONG | HyundaiSafetyFlags.CAMERA_SCC)
    self.safety.init_tests()

  def _accel_msg(self, accel, aeb_req=False, aeb_decel=0, aeb_stop_req=False):
    values = {
      "aReqRaw": accel,
      "aReqValue": accel,
      "AEB_CmdAct": int(aeb_req),
      "AEB_StopReq": int(aeb_stop_req),
      "CR_VSM_DecCmd": aeb_decel,
    }
    return self.packer.make_can_msg_safety("SCC12", self.SCC_BUS, values)

  def _tx_acc_state_msg(self, enable):
    values = {"MainMode_ACC": enable}
    return self.packer.make_can_msg_safety("SCC11", self.SCC_BUS, values)

  def test_no_aeb_scc12(self):
    self.assertTrue(self._tx(self._accel_msg(0)))
    self.assertFalse(self._tx(self._accel_msg(0, aeb_req=True)))
    self.assertFalse(self._tx(self._accel_msg(0, aeb_decel=1.0)))
    self.assertFalse(self._tx(self._accel_msg(0, aeb_stop_req=True)))

  def test_tester_present_allowed(self):
    pass

  def test_disabled_ecu_alive(self):
    pass


@parameterized_class(LDA_BUTTON)
class TestHyundaiSafetyFCEVLong(TestHyundaiLongitudinalSafety, TestHyundaiSafetyFCEV):
  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiSafetyFCEVLong":
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_can_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(self.SAFETY_PARAM_SP)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, HyundaiSafetyFlags.FCEV_GAS | HyundaiSafetyFlags.LONG)
    self.safety.init_tests()


@parameterized_class(LDA_BUTTON)
class TestHyundaiLongitudinalESCCSafety(HyundaiLongitudinalBase, TestHyundaiSafety):
  TX_MSGS = [[0x340, 0], [0x4F1, 0], [0x485, 0], [0x420, 0], [0x421, 0], [0x50A, 0], [0x389, 0]]

  FWD_BLACKLISTED_ADDRS = {2: [0x340, 0x485, 0x420, 0x421, 0x50A, 0x389]}
  RELAY_MALFUNCTION_ADDRS = {0: (0x340, 0x485, 0x420, 0x421, 0x50A, 0x389)}  # LKAS11, LFAHDA_MFC, SCC12, SCC11, SCC13, SCC14

  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiLongitudinalESCCSafety":
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_can_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(HyundaiSafetyFlagsSP.ESCC | self.SAFETY_PARAM_SP)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, HyundaiSafetyFlags.LONG)
    self.safety.init_tests()

  def _accel_msg(self, accel, aeb_req=False, aeb_decel=0):
    values = {
      "aReqRaw": accel,
      "aReqValue": accel,
    }
    return self.packer.make_can_msg_safety("SCC12", self.SCC_BUS, values)

  def _tx_acc_state_msg(self, enable):
    values = {"MainMode_ACC": enable}
    return self.packer.make_can_msg_safety("SCC11", 0, values)

  def test_tester_present_allowed(self):
    pass

  def test_disabled_ecu_alive(self):
    pass


@parameterized_class(LDA_BUTTON)
class TestHyundaiNonSCCSafety(TestHyundaiSafety):

  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiNonSCCSafety":
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_can_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(HyundaiSafetyFlagsSP.NON_SCC | self.SAFETY_PARAM_SP)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, 0)
    self.safety.init_tests()

  def _pcm_status_msg(self, enable):
    values = {"CRUISE_LAMP_S": enable, "AliveCounter": self.cnt_gas % 4}
    self.__class__.cnt_gas += 1
    return self.packer.make_can_msg_safety("EMS16", 0, values, fix_checksum=checksum)

  def _acc_state_msg(self, enable):
    values = {"CRUISE_LAMP_M": enable, "AliveCounter": self.cnt_gas % 4}
    self.__class__.cnt_gas += 1
    return self.packer.make_can_msg_safety("EMS16", 0, values, fix_checksum=checksum)

  def _user_gas_msg(self, gas: float, controls_allowed: bool = True):
    values = {"CF_Ems_AclAct": gas, "CRUISE_LAMP_M": 1, "CRUISE_LAMP_S": controls_allowed, "AliveCounter": self.cnt_gas % 4}
    self.__class__.cnt_gas += 1
    return self.packer.make_can_msg_safety("EMS16", 0, values, fix_checksum=checksum)

  def test_allow_engage_with_gas_pressed(self):
    self._rx(self._user_gas_msg(1, self.safety.get_controls_allowed()))
    self.safety.set_controls_allowed(True)
    self._rx(self._user_gas_msg(1, self.safety.get_controls_allowed()))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._user_gas_msg(1, self.safety.get_controls_allowed()))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_no_disengage_on_gas(self):
    self._rx(self._user_gas_msg(0, self.safety.get_controls_allowed()))
    self.safety.set_controls_allowed(True)
    self._rx(self._user_gas_msg(self.GAS_PRESSED_THRESHOLD + 1, self.safety.get_controls_allowed()))
    # Test we allow lateral, but not longitudinal
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_longitudinal_allowed())
    # Make sure we can re-gain longitudinal actuation
    self._rx(self._user_gas_msg(0, self.safety.get_controls_allowed()))
    self.assertTrue(self.safety.get_longitudinal_allowed())


# Both pedal CAN ID dialects: standard comma pedal (0x200/0x201) and remapped custom firmware (0x700/0x701).
# OTHER_IDS is the inactive dialect, which must be neither transmittable nor parsed.
PEDAL_DIALECTS = [
  {"PEDAL_IDS": STANDARD_IDS, "OTHER_IDS": REMAPPED_IDS, "DIALECT_SP": 0,
   "TX_MSGS": [[0x340, 0], [0x4F1, 0], [0x485, 0], [STANDARD_IDS.command_addr, 0]]},
  {"PEDAL_IDS": REMAPPED_IDS, "OTHER_IDS": STANDARD_IDS, "DIALECT_SP": HyundaiSafetyFlagsSP.GAS_INTERCEPTOR_REMAPPED,
   "TX_MSGS": [[0x340, 0], [0x4F1, 0], [0x485, 0], [REMAPPED_IDS.command_addr, 0]]},
]


@parameterized_class([{**lda, **d} for d in PEDAL_DIALECTS for lda in LDA_BUTTON])
class TestHyundaiNonSCCGasInterceptorSafety(GasInterceptorSafetyTest, TestHyundaiNonSCCSafety):
  """
    Non-SCC ICE + comma pedal: pause/resume button engagement only (no SCC; up/down never engage), TX = base + the active dialect's GAS_COMMAND only
    (0x200, or 0x700 when remapped), gas from the active dialect's GAS_SENSOR (0x201, or 0x701 when remapped)
  """
  PEDAL_IDS = STANDARD_IDS
  OTHER_IDS = REMAPPED_IDS
  DIALECT_SP = 0
  TX_MSGS = [[0x340, 0], [0x4F1, 0], [0x485, 0], [0x200, 0]]
  # Single source of truth for the panda/openpilot threshold: the boundary test below
  # (test_no_disengage_on_gas_interceptor) fails unless the C constant equals the Python one. Bench-measured value.
  INTERCEPTOR_THRESHOLD = HYUNDAI_GAS_INTERCEPTOR_THRESHOLD

  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiNonSCCGasInterceptorSafety":
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_can_generated")
    self.pedal_packer = CANPackerSafety(GAS_INTERCEPTOR_DBC)
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(self._pedal_sp())
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, 0)
    self.safety.init_tests()

  def _pedal_sp(self) -> int:
    return HyundaiSafetyFlagsSP.NON_SCC | HyundaiSafetyFlagsSP.GAS_INTERCEPTOR | self.DIALECT_SP | self.SAFETY_PARAM_SP

  # *** gas command magnitude: panda enforces openpilot's cap (MAX_INTERCEPTOR_GAS) in raw counts per track ***

  def _interceptor_gas_cmd_raw(self, a: int, b: int, enable: bool = True):
    # raw tracks straight into the frame (the packer scales physical values, which could hide an off-by-one)
    self.__class__.cnt_gas_cmd += 1
    dat = bytes([a >> 8, a & 0xFF, b >> 8, b & 0xFF, (int(enable) << 7) | (self.__class__.cnt_gas_cmd & 0xF), 0])
    return libsafety_py.make_CANPacket(self.PEDAL_IDS.command_addr, 0, dat)

  def test_gas_interceptor_safety_check(self):
    # replaces the generic sweep (which assumed no magnitude limit): within the cap -> needs controls; above -> never
    for gas in np.linspace(0., 1., 41):
      for controls_allowed in (True, False):
        self.safety.set_controls_allowed(controls_allowed)
        within_cap = gas <= MAX_INTERCEPTOR_GAS + 1e-9
        send = (gas == 0.) or (controls_allowed and within_cap)
        self.assertEqual(send, self._tx(self._interceptor_gas_cmd(float(gas))), (gas, controls_allowed))

  def test_gas_ceiling_matches_openpilot_cap(self):
    # the car's own frame at MAX_INTERCEPTOR_GAS is exactly at the ceiling and passes; one count more on either track
    # is blocked. Pins HYUNDAI_GAS_INTERCEPTOR_MAX_GAS_A/_B to the packer output (DBC scaling + cap).
    _, dat, _ = create_gas_interceptor_command(CANPacker(GAS_INTERCEPTOR_DBC), MAX_INTERCEPTOR_GAS, 0,
                                               self.PEDAL_IDS.command_msg)
    a, b = (dat[0] << 8) | dat[1], (dat[2] << 8) | dat[3]
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._interceptor_gas_cmd_raw(a, b)))
    self.assertFalse(self._tx(self._interceptor_gas_cmd_raw(a + 1, b)))
    self.assertFalse(self._tx(self._interceptor_gas_cmd_raw(a, b + 1)))
    self.assertFalse(self._tx(self._interceptor_gas_cmd_raw(0xFFFF, 0xFFFF)))
    # the ceiling applies regardless of ENABLE (a disabled frame carrying a large value is malformed, never needed)
    self.assertFalse(self._tx(self._interceptor_gas_cmd_raw(a + 1, b, enable=False)))
    # the clear / zero frame is always allowed
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)
      self.assertTrue(self._tx(self._interceptor_gas_cmd_raw(0, 0, enable=False)))

  # *** pedal messages (pedal DBC; checksum and counter are enforced by the RX check) ***

  def _interceptor_gas_cmd(self, gas: int, ids=None):
    ids = ids or self.PEDAL_IDS
    values: dict[str, float | int] = {"PEDAL_COUNTER": self.__class__.cnt_gas_cmd & 0xF}
    if gas > 0:
      values["GAS_COMMAND"] = gas * 255.
      values["GAS_COMMAND2"] = gas * 255.
    self.__class__.cnt_gas_cmd += 1
    return self.pedal_packer.make_can_msg_safety(ids.command_msg, 0, values)

  def _interceptor_user_gas(self, gas: int, state: int = 0, counter: int | None = None, bad_checksum: bool = False,
                            ids=None):
    ids = ids or self.PEDAL_IDS
    if counter is None:
      counter = self.__class__.cnt_user_gas & 0xF
      self.__class__.cnt_user_gas += 1
    values = {"INTERCEPTOR_GAS": gas, "INTERCEPTOR_GAS2": gas, "STATE": state, "PEDAL_COUNTER": counter}

    def fix_checksum(msg):
      addr, dat, bus = msg
      dat = bytearray(dat)
      dat[5] = crc8_pedal(dat[:5]) ^ (0xFF if bad_checksum else 0)
      return addr, dat, bus
    return self.pedal_packer.make_can_msg_safety(ids.sensor_msg, 0, values, fix_checksum=fix_checksum)

  # *** EMS16 in pedal mode: factory cruise MAIN (CRUISE_LAMP_M) is a lockout, not openpilot's main switch ***

  def _ems16(self, main=False, active=False, gas=0):
    values = {"CF_Ems_AclAct": gas, "CRUISE_LAMP_M": main, "CRUISE_LAMP_S": active, "AliveCounter": self.cnt_gas % 4}
    self.__class__.cnt_gas += 1
    return self.packer.make_can_msg_safety("EMS16", 0, values, fix_checksum=checksum)

  def _user_gas_msg(self, gas: float, controls_allowed: bool = True):
    # factory cruise MAIN off (the normal pedal-long state); EMS16 gas is ignored in this mode anyway
    return self._ems16(gas=gas)

  def _pcm_status_msg(self, enable):
    # factory cruise active implies its MAIN is armed
    return self._ems16(main=enable, active=enable)

  def _acc_state_msg(self, enable):
    # factory cruise MAIN armed. Pedal-long: acc_main_on (openpilot's cruiseState.available) is always True; MAIN armed
    # is a longitudinal lockout only
    return self._ems16(main=enable)

  def test_enable_control_allowed_with_mads_button_and_disable_with_main_cruise(self):
    # pedal-long: arming / disarming the factory MAIN never touches lateral (MADS), only longitudinal (lockout)
    try:
      self._lkas_button_msg(False)
    except NotImplementedError as err:
      raise unittest.SkipTest("no LDA button on this variant") from err
    for enable_mads in (True, False):
      with self.subTest(enable_mads=enable_mads):
        self.safety.set_mads_params(enable_mads, False, False)
        self._rx(self._lkas_button_msg(True))
        self._rx(self._lkas_button_msg(False))
        self.assertEqual(enable_mads, self.safety.get_controls_allowed_lateral())
        for main in (True, False, True):
          self._rx(self._acc_state_msg(main))
          self.assertTrue(self.safety.get_acc_main_on())
          self.assertEqual(enable_mads, self.safety.get_controls_allowed_lateral())

  # *** CAN ID dialect: exactly one of standard (0x200/0x201) / remapped (0x700/0x701) is active ***

  def test_pedal_messages_use_dialect_addrs(self):
    # guards the helpers themselves: the DBC message names must resolve to the dialect's addresses
    self.assertEqual(self.PEDAL_IDS.command_addr, self._interceptor_gas_cmd(0)[0].addr)
    self.assertEqual(self.PEDAL_IDS.sensor_addr, self._interceptor_user_gas(0)[0].addr)
    self.assertEqual(self.OTHER_IDS.command_addr, self._interceptor_gas_cmd(0, ids=self.OTHER_IDS)[0].addr)

  def test_other_dialect_command_blocked(self):
    self._rx(self._interceptor_user_gas(0))
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)
      self.assertTrue(self._tx(self._interceptor_gas_cmd(0)))
      self.assertEqual(controls_allowed, self._tx(self._interceptor_gas_cmd(GAS_ON)))
      # the inactive dialect's command id is not in the TX allowlist at all, not even a zero command
      self.assertFalse(self._tx(self._interceptor_gas_cmd(0, ids=self.OTHER_IDS)))
      self.assertFalse(self._tx(self._interceptor_gas_cmd(GAS_ON, ids=self.OTHER_IDS)))

  def test_other_dialect_sensor_ignored(self):
    # the inactive dialect's sensor neither satisfies the pedal RX check nor sets gas_pressed
    for _ in range(3):
      self._rx_all_but_pedal()
      self._rx(self._interceptor_user_gas(0x1000, ids=self.OTHER_IDS))
    self.assertFalse(self.safety.safety_config_valid())
    self.assertFalse(self.safety.get_gas_pressed_prev())
    self.assertEqual(0, self.safety.get_gas_interceptor_prev())
    self._rx(self._interceptor_user_gas(0))
    self.assertTrue(self.safety.safety_config_valid())

  def test_gas_override_active_dialect(self):
    self._rx(self._interceptor_user_gas(0))
    self.safety.set_controls_allowed(True)
    self._rx(self._interceptor_user_gas(0x1000))
    self.assertTrue(self.safety.get_gas_pressed_prev())
    self.assertFalse(self.safety.get_longitudinal_allowed())
    self.assertFalse(self._tx(self._interceptor_gas_cmd(GAS_ON)))
    self._rx(self._interceptor_user_gas(0))
    self.assertTrue(self._tx(self._interceptor_gas_cmd(GAS_ON)))

  def test_remapped_bit_alone_is_inert(self):
    # GAS_INTERCEPTOR_REMAPPED without GAS_INTERCEPTOR selects nothing: no pedal command of either dialect
    self.safety.set_current_safety_param_sp(HyundaiSafetyFlagsSP.NON_SCC | HyundaiSafetyFlagsSP.GAS_INTERCEPTOR_REMAPPED |
                                            self.SAFETY_PARAM_SP)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, 0)
    self.safety.set_controls_allowed(True)
    for ids in (STANDARD_IDS, REMAPPED_IDS):
      self.assertFalse(self._tx(self._interceptor_gas_cmd(0, ids=ids)))
      self.assertFalse(self._tx(self._interceptor_gas_cmd(GAS_ON, ids=ids)))

  # *** engagement is button based (like hyundai_longitudinal), not from the factory cruise state ***

  def test_disable_control_allowed_from_cruise(self):
    pass

  def test_enable_control_allowed_from_cruise(self):
    pass

  def test_sampling_cruise_buttons(self):
    pass

  def test_factory_cruise_never_grants_controls(self):
    for _ in range(3):
      self._rx(self._button_msg(Buttons.NONE))
    self._rx(self._button_msg(Buttons.SET))
    self._rx(self._pcm_status_msg(False))
    self._rx(self._pcm_status_msg(True))
    # SET is still held: no falling edge yet, and the factory cruise rising edge must not grant
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._button_msg(Buttons.NONE))
    # SET falling edge while the factory cruise is on (MAIN armed): locked out
    self.assertFalse(self.safety.get_controls_allowed())

  def test_factory_cruise_off_keeps_controls(self):
    self.safety.set_controls_allowed(True)
    self._rx(self._pcm_status_msg(False))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_button_sends(self):
    # root-cause fix for the pedal SCE faults (routes 11c/123): openpilot may NEVER send CLU11 (0x4F1) in pedal mode,
    # in any state: it collides with the cluster's own CLU11 on the same ID
    for controls_allowed in (False, True):
      for cruise_on in (False, True):
        self.safety.set_controls_allowed(controls_allowed)
        self._rx(self._pcm_status_msg(cruise_on))
        self.safety.set_controls_allowed(controls_allowed)
        for btn in range(8):
          self.assertFalse(self._tx(self._button_msg(btn)), f"{btn=} {controls_allowed=} {cruise_on=}")

  def test_factory_main_lockout(self):
    # factory cruise MAIN armed: pedal-long disengages and pause/resume can't engage (up/down never can)
    self.safety.set_controls_allowed(True)
    self._rx(self._acc_state_msg(True))  # MAIN armed
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_acc_main_on())  # lateral main unaffected
    for btn in (Buttons.SET, Buttons.RESUME, Buttons.CANCEL):
      self._press(btn)
      self._release()
      self.assertFalse(self.safety.get_controls_allowed(), btn)
    # MAIN off again: pause/resume works
    self._rx(self._acc_state_msg(False))
    self.assertTrue(self.safety.get_acc_main_on())
    self._press(Buttons.CANCEL)
    self._release(HYUNDAI_PAUSE_RELEASE_SAMPLES)
    self.assertTrue(self.safety.get_controls_allowed())

  def test_set_resume_buttons(self):
    # buttons-v3: replaces HyundaiLongitudinalBase.test_set_resume_buttons. In pedal mode NO single button transition
    # grants controls: the up/down falling edges (RES/ACCEL 1, SET/DECEL 2) never do, and a pause/resume (4) release
    # needs HYUNDAI_PAUSE_RELEASE_SAMPLES released samples (one sample is never enough)
    for btn_prev in range(8):
      for btn_cur in range(8):
        for _ in range(5):
          self._rx(self._button_msg(Buttons.NONE))
        self.safety.set_controls_allowed(0)
        for _ in range(10):
          self._rx(self._button_msg(btn_prev))
          self.assertFalse(self.safety.get_controls_allowed())
        self._rx(self._button_msg(btn_cur))
        self.assertFalse(self.safety.get_controls_allowed(), f"{btn_prev=} {btn_cur=}")

  def test_up_down_never_grant(self):
    # buttons-v3: the up/down arrows only change openpilot's set speed. Whatever the brake / gas / speed / press length /
    # release length, an up or down press never grants controls in pedal mode (only the pause/resume release does)
    for btn in (Buttons.RESUME, Buttons.SET):
      for speed in (0, 100, 500, 1500):
        for gas in (0, 0x1000):
          for held, released in ((1, 1), (5, 1), (5, 20), (40, 40)):
            self.safety.set_controls_allowed(False)
            self._rx(self._user_brake_msg(False))
            self._rx(self._speed_msg(speed))
            self._rx(self._interceptor_user_gas(gas))
            self._release()
            self._press(btn, held)
            self._release(released)
            self.assertFalse(self.safety.get_controls_allowed(), (btn, speed, gas, held, released))
      self._rx(self._interceptor_user_gas(0))

  def test_up_down_never_disengage(self):
    # buttons-v3: and while engaged an up/down press keeps controls (it is a set-speed change, not an on/off)
    for btn in (Buttons.RESUME, Buttons.SET):
      self.safety.set_controls_allowed(True)
      self._release()
      self._press(btn, 40)
      self.assertTrue(self.safety.get_controls_allowed(), btn)
      self._release(20)
      self.assertTrue(self.safety.get_controls_allowed(), btn)

  # *** pause/resume button (CF_Clu_CruiseSwState 4) and NO auto-resume ***

  def _press(self, btn, n=5):
    for _ in range(n):
      self._rx(self._button_msg(btn))

  def _release(self, n=5):
    for _ in range(n):
      self._rx(self._button_msg(Buttons.NONE))

  def _engage_then_brake(self):
    # engage the only way pedal mode allows: a pause/resume press + debounced release, then brake
    self._rx(self._user_brake_msg(False))
    self._rx(self._speed_msg(500))
    self._release()
    self._press(Buttons.CANCEL)
    self._release(HYUNDAI_PAUSE_RELEASE_SAMPLES)
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._user_brake_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_brake_then_no_auto_resume(self):
    # brake disengages, and NOTHING but a driver button press re-engages: not releasing the brake, not time,
    # not speed recovering, not the button samples that keep streaming
    self._engage_then_brake()
    self._rx(self._user_brake_msg(False))
    for i in range(1000):  # 20 s of CLU11 at 50 Hz with brake/speed traffic, no button
      self._rx(self._button_msg(Buttons.NONE))
      if i % 2 == 0:
        self._rx(self._user_brake_msg(False))
        self._rx(self._speed_msg(500 + i))
        self._rx(self._interceptor_user_gas(0))
      self.assertFalse(self.safety.get_controls_allowed(), i)

  def test_pause_button_resumes_after_brake(self):
    self._engage_then_brake()
    self._rx(self._user_brake_msg(False))
    self._release()
    self._press(Buttons.CANCEL)
    self.assertFalse(self.safety.get_controls_allowed())  # nothing while held
    self._release(2)
    self.assertFalse(self.safety.get_controls_allowed())  # debounce: not yet
    self._release(1)
    self.assertTrue(self.safety.get_controls_allowed())  # exactly on the 3rd released sample

  def test_pause_button_disengages_when_engaged(self):
    # held: controls cleared immediately (the disengage). The debounced release re-grants the PERMISSION only: openpilot
    # stays disengaged after a disengage press (pause_resume.py: no resumeCruise when engaged at the press start), sends
    # zero gas, and the heartbeat clears the unused grant. Panda can't tell a stale grant from an engagement, so it may not
    # condition on controls_allowed (route 00000128 @139.2 s divergence).
    self.safety.set_controls_allowed(True)
    self._release()
    self._press(Buttons.CANCEL, 1)
    self.assertFalse(self.safety.get_controls_allowed())
    self._press(Buttons.CANCEL)
    self.assertFalse(self.safety.get_controls_allowed())
    self._release(HYUNDAI_PAUSE_RELEASE_SAMPLES - 1)
    self.assertFalse(self.safety.get_controls_allowed())
    self._release(1)
    self.assertTrue(self.safety.get_controls_allowed())

  def test_pause_button_bounce_grants_once(self):
    # a press with 1-2 sample bounces inside it is ONE press: nothing granted until the final debounced release
    for bounce in (1, 2):
      self.safety.set_controls_allowed(True)
      self._release()
      self._press(Buttons.CANCEL, 3)
      self._release(bounce)
      self.assertFalse(self.safety.get_controls_allowed(), bounce)
      self._press(Buttons.CANCEL, 3)
      self.assertFalse(self.safety.get_controls_allowed(), bounce)
      self._release(HYUNDAI_PAUSE_RELEASE_SAMPLES)
      self.assertTrue(self.safety.get_controls_allowed(), bounce)

  def test_pause_button_after_stale_grant(self):
    # route 00000128 @138.3-139.4 s: a stale panda grant (then: a SET openpilot refused) made the old rule treat the next
    # pause/resume press as a DISENGAGE while openpilot treated it as a resume. Up/down can no longer leave a grant, but
    # the release still grants regardless of controls_allowed (any stale permission, e.g. after a disengage press)
    self._rx(self._user_brake_msg(False))
    self._release()
    self._press(Buttons.SET)
    self._release(1)
    self.assertFalse(self.safety.get_controls_allowed())  # buttons-v3: up/down leave no grant
    self.safety.set_controls_allowed(True)  # stale permission openpilot did not take
    self._release(40)
    self._press(Buttons.CANCEL)
    self.assertFalse(self.safety.get_controls_allowed())
    self._release(HYUNDAI_PAUSE_RELEASE_SAMPLES)
    self.assertTrue(self.safety.get_controls_allowed())

  def test_pause_button_main_armed_during_press(self):
    # MAIN armed mid-press (or still armed at the release): no grant
    self._engage_then_brake()
    self._rx(self._user_brake_msg(False))
    self._release()
    self._press(Buttons.CANCEL)
    self._rx(self._acc_state_msg(True))
    self._release(20)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_pause_button_while_brake_held(self):
    self._engage_then_brake()
    self._release()
    self._press(Buttons.CANCEL)
    self._release()
    self.assertFalse(self.safety.get_controls_allowed())

  def test_pause_button_held_across_brake_release(self):
    self._engage_then_brake()
    self._release()
    self._press(Buttons.CANCEL)       # press starts with the brake down
    self._rx(self._user_brake_msg(False))
    self._press(Buttons.CANCEL)       # still held after the brake release
    self._release(20)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_pause_button_brake_during_press(self):
    self._engage_then_brake()
    self._rx(self._user_brake_msg(False))
    self._release()
    self._press(Buttons.CANCEL)
    self._rx(self._user_brake_msg(True))
    self._rx(self._user_brake_msg(False))
    self._release(20)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_pause_button_gas_pressed(self):
    # gas held does NOT block a resume (route 00000128 @2152.9/2154.2/2155.7: three presses refused with the foot on the
    # gas). It engages into override, exactly like SET with gas; longitudinal_allowed stays False while gas is pressed.
    for phase in ("start", "during", "throughout"):
      self._engage_then_brake()
      self._rx(self._user_brake_msg(False))
      self._rx(self._interceptor_user_gas(0x1000 if phase in ("start", "throughout") else 0))
      self._release()
      self._press(Buttons.CANCEL)
      if phase == "during":
        self._rx(self._interceptor_user_gas(0x1000))
      if phase != "throughout":
        self._rx(self._interceptor_user_gas(0))
      self._release(HYUNDAI_PAUSE_RELEASE_SAMPLES)
      self.assertTrue(self.safety.get_controls_allowed(), phase)
      self.assertEqual(phase != "throughout", self.safety.get_longitudinal_allowed(), phase)
      self._rx(self._interceptor_user_gas(0))

  def test_pause_button_resumes_at_standstill(self):
    # green-light resume: panda has no speed floor for the deliberate pause/resume press (brake released)
    self._rx(self._user_brake_msg(False))
    self._rx(self._speed_msg(0))
    self._release()
    self._press(Buttons.CANCEL)
    self._release(HYUNDAI_PAUSE_RELEASE_SAMPLES)
    self.assertTrue(self.safety.get_controls_allowed())
    # and brake still takes it away instantly, at standstill too (rising edge)
    self._rx(self._user_brake_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_pause_button_engages_at_any_speed(self):
    # buttons-v3: no engage floor at all: 0, ~3, ~15, ~24, ~26 and ~70 mph (WHL_SPD raw 0.03125 km/h)
    for kph in (0., 5., 24., 38., 42., 113.):
      self.safety.set_controls_allowed(False)
      self._rx(self._user_brake_msg(False))
      self._rx(self._speed_msg(kph / 0.03125))
      self._release()
      self._press(Buttons.CANCEL)
      self._release(HYUNDAI_PAUSE_RELEASE_SAMPLES)
      self.assertTrue(self.safety.get_controls_allowed(), kph)

  def test_up_down_after_brake_no_grant(self):
    # buttons-v3: after a brake disengage, up/down presses (held across the brake release or not) never re-grant; only
    # pause/resume does
    for btn in (Buttons.SET, Buttons.RESUME):
      self._engage_then_brake()
      self._release()
      self._press(btn)                  # pressed while braking
      self._rx(self._user_brake_msg(False))
      self._press(btn)
      self._release(20)
      self.assertFalse(self.safety.get_controls_allowed(), btn)
      self._press(btn)                  # a fresh press with the brake released
      self._release(20)
      self.assertFalse(self.safety.get_controls_allowed(), btn)
      self._press(Buttons.CANCEL)
      self._release(HYUNDAI_PAUSE_RELEASE_SAMPLES)
      self.assertTrue(self.safety.get_controls_allowed(), btn)

  def test_cancel_button(self):
    HyundaiLongitudinalBase.test_cancel_button(self)

  # *** gas source ***

  def test_ems16_gas_ignored(self):
    # EMS16 (ECU's view of the pedal) includes openpilot's own command: must not set gas_pressed in pedal mode
    self.safety.set_controls_allowed(True)
    self._rx(self._user_gas_msg(1, True))
    self._rx(self._user_gas_msg(1, True))
    self.assertFalse(self.safety.get_gas_pressed_prev())
    self.assertTrue(self.safety.get_longitudinal_allowed())

  def test_gas_threshold_boundary(self):
    self.safety.set_controls_allowed(True)
    for gas, pressed in ((HYUNDAI_GAS_INTERCEPTOR_THRESHOLD, False), (HYUNDAI_GAS_INTERCEPTOR_THRESHOLD + 1, True)):
      self._rx(self._interceptor_user_gas(gas))
      self.assertEqual(pressed, self.safety.get_gas_pressed_prev())
      self.assertEqual(not pressed, self.safety.get_longitudinal_allowed())
      self.assertEqual(not pressed, self._tx(self._interceptor_gas_cmd(GAS_ON)))
    self._rx(self._interceptor_user_gas(0))

  # *** pedal health: the RX check must invalidate control for a missing / frozen / corrupt pedal ***

  def _rx_all_but_pedal(self):
    self._rx(self._user_gas_msg(0, True))
    self._rx(self._speed_msg(0))
    self._rx(self._user_brake_msg(False))
    self._rx(self._torque_driver_msg(0))
    self._rx(self._button_msg(Buttons.NONE))
    if self.SAFETY_PARAM_SP & HyundaiSafetyFlagsSP.HAS_LDA_BUTTON:
      self._rx(self._lkas_button_msg(False))

  def test_pedal_is_required_rx_check(self):
    for _ in range(3):
      self._rx_all_but_pedal()
    self.assertFalse(self.safety.safety_config_valid())
    self._rx(self._interceptor_user_gas(0))
    self.assertTrue(self.safety.safety_config_valid())

  def test_dead_pedal_invalidates(self):
    for _ in range(3):
      self._rx_all_but_pedal()
      self._rx(self._interceptor_user_gas(0))
    self.assertTrue(self.safety.safety_config_valid())

    # pedal stops transmitting, everything else keeps going
    self.safety.set_controls_allowed(True)
    self.safety.set_timer(int(1.5e6))
    self._rx_all_but_pedal()
    self.safety.safety_tick_current_safety_config()
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.safety_config_valid())

  def test_frozen_pedal_counter(self):
    self._rx(self._interceptor_user_gas(0))
    self.safety.set_controls_allowed(True)
    for _ in range(6):
      self._rx(self._interceptor_user_gas(0, counter=3))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_pedal_bad_checksum(self):
    self._rx(self._interceptor_user_gas(0))
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._rx(self._interceptor_user_gas(0x1000, bad_checksum=True)))
    self.assertFalse(self.safety.get_controls_allowed())
    # a corrupt frame is not parsed
    self.assertEqual(0, self.safety.get_gas_interceptor_prev())

  # *** mode gating ***

  def test_pedal_blocked_without_sp_flag(self):
    for sp in (HyundaiSafetyFlagsSP.NON_SCC | self.DIALECT_SP, HyundaiSafetyFlagsSP.GAS_INTERCEPTOR | self.DIALECT_SP):
      with self.subTest(sp=sp):
        self.safety.set_current_safety_param_sp(sp | self.SAFETY_PARAM_SP)
        self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, 0)
        self.safety.set_controls_allowed(True)
        self.assertFalse(self._tx(self._interceptor_gas_cmd(0)))
        self.assertFalse(self._tx(self._interceptor_gas_cmd(GAS_ON)))

  def test_pedal_blocked_on_unsupported_cars(self):
    sp = self._pedal_sp()
    for param in (HyundaiSafetyFlags.EV_GAS, HyundaiSafetyFlags.HYBRID_GAS, HyundaiSafetyFlags.FCEV_GAS,
                  HyundaiSafetyFlags.CAMERA_SCC):
      with self.subTest(param=param):
        self.safety.set_current_safety_param_sp(sp)
        self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, param)
        self.safety.set_controls_allowed(True)
        self.assertFalse(self._tx(self._interceptor_gas_cmd(GAS_ON)))

    # stock HKG (SCC-replacement) longitudinal and the pedal are mutually exclusive. The LONG param is only honored in
    # ALLOW_DEBUG builds; a release panda ignores it (hyundai_longitudinal=false), in which case pedal-long applies.
    with self.subTest(param=HyundaiSafetyFlags.LONG):
      self.safety.set_current_safety_param_sp(sp)
      self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, HyundaiSafetyFlags.LONG)
      self.safety.set_controls_allowed(True)
      stock_long_active = self._tx(common.make_msg(0, 0x50A, 8))  # SCC13: only in the SCC-replacement TX allowlist
      self.assertEqual(not stock_long_active, self._tx(self._interceptor_gas_cmd(GAS_ON)))
    for mode in (CarParams.SafetyModel.hyundaiLegacy, CarParams.SafetyModel.hyundaiCanfd):
      with self.subTest(mode=mode):
        self.safety.set_current_safety_param_sp(sp)
        self.safety.set_safety_hooks(mode, 0)
        self.safety.set_controls_allowed(True)
        self.assertFalse(self._tx(self._interceptor_gas_cmd(GAS_ON)))

  def test_no_scc_actuation_allowed(self):
    # pedal-long must not unlock the SCC-replacement TX set (SCC11/12/13/14, FRT_RADAR11, FCA11/12, radar UDS)
    self.safety.set_controls_allowed(True)
    for addr in (0x420, 0x421, 0x50A, 0x389, 0x4A2, 0x38D, 0x483, 0x7D0):
      self.assertFalse(self._tx(common.make_msg(0, addr, 8)), f"{addr=:#x}")

  # *** timed factory-cruise CANCEL: panda's own CLU11, only right after a cluster CLU11, only while factory cruise is
  # active under an engaged openpilot longitudinal (hyundai_common.h hyundai_fc_cancel_*) ***

  FC_PERIOD_US = 19800  # cluster CLU11 period on routes 11c/123/127/128

  def _clu_raw(self, button=0, main=0, counter=None, b0_hi=0x00, b1=0x7A, b2=0x00, b3_lo=0x1, good_parity=True):
    """A CLUSTER CLU11 with the real byte layout (route 128: e.g. 00 00 7a 01): CruiseSwState bits 0-2, CruiseSwMain bit 3,
    ParityBit1 bit 5 (parity over bits 0-3 + AliveCnt1), VanzDecimal bits 6-7, Vanz bytes 1-2, AliveCnt1 byte 3 hi."""
    if counter is None:
      counter = self.__class__.cnt_button & 0xF
      self.__class__.cnt_button += 1
    b0 = (b0_hi & 0xC0) | (button & 7) | ((main & 1) << 3)
    par = (bin(b0 & 0xF).count("1") + bin(counter).count("1")) & 1
    b0 |= (par ^ (0 if good_parity else 1)) << 5
    return libsafety_py.make_CANPacket(0x4F1, 0, bytes([b0, b1, b2, (counter << 4) | b3_lo]))

  def _self_tx(self):
    out = []
    for i in range(self.safety.get_self_tx_count()):
      pkt = libsafety_py.ffi.new("CANPacket_t *")
      bus = libsafety_py.ffi.new("int *")
      skip = libsafety_py.ffi.new("bool *")
      ts = self.safety.get_self_tx(i, pkt, bus, skip)
      out.append({"ts": ts, "addr": pkt[0].addr, "bus": bus[0], "skip_hook": skip[0], "len": pkt[0].data_len_code,
                  "dat": bytes(pkt[0].data[0:4]), "pkt_bus": pkt[0].bus, "extended": pkt[0].extended, "fd": pkt[0].fd})
    return out

  def _fc_drive(self, n_frames, t0, active=True, engaged=True, main=None, brake=False, button=0, cl_main=0,
                stop_after=None, every_ems=True):
    """Simulate the bus: cluster CLU11 every 19.8 ms, EMS16 (factory cruise lamp) every 10 ms. Returns the cluster frames
    sent (raw bytes, time). active: bool or callable(t_us) -> bool."""
    sent = []
    main = active if main is None else main
    for k in range(n_frames):
      t = t0 + k * self.FC_PERIOD_US
      is_active = active(t) if callable(active) else active
      if every_ems:
        self.safety.set_timer(t)
        self._rx(self._ems16(main=main or is_active, active=is_active))
      self.safety.set_timer(t + 5000)
      if brake:
        self._rx(self._user_brake_msg(True))
      self.safety.set_timer(t + 9900)
      if every_ems:
        self._rx(self._ems16(main=main or is_active, active=is_active))
      self.safety.set_timer(t + 10000)
      msg = self._clu_raw(button=button, main=cl_main)
      sent.append((t + 10000, bytes(msg[0].data[0:4])))
      self._rx(msg)
      if stop_after is not None and self.safety.get_self_tx_count() >= stop_after:
        break
    return sent

  def _fc_engage(self, t=1_000_000):
    """openpilot longitudinal engaged (panda grant + heartbeat), factory cruise off, brake released"""
    self.safety.set_timer(t)
    self._rx(self._user_brake_msg(False))
    self._rx(self._ems16())
    self.safety.set_controls_allowed(True)
    self.safety.set_heartbeat_engaged(True)
    self.safety.clear_self_tx()

  def test_fc_cancel_parity_matches_real_cluster_frames(self):
    # guards the test helper AND documents the rule: these are real cluster CLU11 payloads from routes 128/123/11c
    for hexs in ("00007a01", "e12e7a01", "22007a01", "24327a01", "c12e7a11", "012f7a21", "c1327a41", "84347a41",
                 "e1327a51", "e4347a51", "22387a61", "24407e61", "682e7a61", "c44c7e71", "882e7a71", "684d7a91",
                 "04377ab1", "c84c7eb1", "e84c7ec1"):
      d = bytes.fromhex(hexs)
      msg = self._clu_raw(button=d[0] & 7, main=(d[0] >> 3) & 1, counter=d[3] >> 4, b0_hi=d[0] & 0xC0, b1=d[1], b2=d[2],
                          b3_lo=d[3] & 0xF)
      self.assertEqual(hexs, bytes(msg[0].data[0:4]).hex())

  def test_fc_cancel_trigger_and_payload(self):
    self._fc_engage()
    sent = self._fc_drive(3, 2_000_000)
    tx = self._self_tx()
    self.assertEqual(3, len(tx))  # one 3-frame attempt
    for f, (t_cl, cl) in zip(tx, sent, strict=True):
      # timing: queued synchronously with the reception of a cluster frame, never at any other time
      self.assertEqual(t_cl, f["ts"])
      self.assertEqual((0x4F1, 0, 0, 0, 0), (f["addr"], f["bus"], f["pkt_bus"], f["extended"], f["fd"]))
      self.assertEqual(4, f["len"])  # DLC 4
      self.assertTrue(f["skip_hook"])
      d = f["dat"]
      self.assertEqual(4, d[0] & 0x7)                       # CF_Clu_CruiseSwState = CANCEL
      self.assertEqual(0, (d[0] >> 3) & 0x3)                # CruiseSwMain, SldMainSW = 0
      self.assertEqual(cl[0] & 0xC0, d[0] & 0xC0)           # VanzDecimal copied
      self.assertEqual(cl[1:3], d[1:3])                     # Vanz / unit / detent / rheostat copied
      self.assertEqual(cl[3] & 0xF, d[3] & 0xF)             # CluInfo / AmpInfo copied
      self.assertEqual(((cl[3] >> 4) + 1) & 0xF, d[3] >> 4)  # AliveCnt1 = cluster + 1
      par = (bin(d[0] & 0xF).count("1") + bin(d[3] >> 4).count("1")) & 1
      self.assertEqual(par, (d[0] >> 5) & 1)                # parity recomputed over the FINAL payload
    # the lockout is unchanged: openpilot longitudinal is dropped on the same EMS16 frame
    self.assertFalse(self.safety.get_controls_allowed())

  def test_fc_cancel_parity_all_counters(self):
    # counter wrap 15 -> 0 and the parity of every counter value, with both VanzDecimal patterns
    for b0_hi in (0x00, 0xC0):
      for c in range(16):
        self.setUp()
        self._fc_engage()
        self._rx(self._ems16(main=True, active=True))
        self.safety.set_timer(1_010_000)
        self._rx(self._clu_raw(counter=c, b0_hi=b0_hi))
        tx = self._self_tx()
        self.assertEqual(1, len(tx), c)
        d = tx[0]["dat"]
        self.assertEqual((c + 1) & 0xF, d[3] >> 4)
        self.assertEqual((bin(d[0] & 0xF).count("1") + bin(d[3] >> 4).count("1")) & 1, (d[0] >> 5) & 1, c)

  def test_fc_cancel_needs_engaged_long(self):
    # never transmits unless openpilot longitudinal is engaged when the factory cruise comes on
    for controls_allowed, heartbeat in ((False, False), (False, True), (True, False)):
      self.setUp()
      self._fc_engage()
      self.safety.set_controls_allowed(controls_allowed)
      self.safety.set_heartbeat_engaged(heartbeat)
      self._fc_drive(150, 2_000_000)
      self.assertEqual(0, self.safety.get_self_tx_count(), (controls_allowed, heartbeat))

  def test_fc_cancel_main_armed_first_never_triggers(self):
    # MAIN armed while engaged -> lockout drops long -> a later factory engagement (driver's own SET) is NOT fought
    self._fc_engage()
    self.safety.set_timer(1_500_000)
    self._rx(self._ems16(main=True))
    self.assertFalse(self.safety.get_controls_allowed())
    self._fc_drive(150, 2_000_000, active=True, main=True)
    self.assertEqual(0, self.safety.get_self_tx_count())

  def test_fc_cancel_no_tx_without_cluster_frame(self):
    # the timing gate: EMS16 / brake / pedal traffic alone never produces a CLU11, however long the factory cruise is on
    self._fc_engage()
    for k in range(150):  # 1.5 s of factory cruise, inside the 2 s window
      self.safety.set_timer(2_000_000 + k * 10_000)
      self._rx(self._ems16(main=True, active=True))
      self._rx(self._interceptor_user_gas(0))
      self._rx(self._user_brake_msg(False))
      self._rx(self._speed_msg(500))
    self.assertEqual(0, self.safety.get_self_tx_count())
    # a cluster frame within the window: exactly one frame, at that frame's time
    self.safety.set_timer(3_505_000)
    self._rx(self._clu_raw())
    self.assertEqual([3_505_000], [f["ts"] for f in self._self_tx()])

  def test_fc_cancel_one_frame_per_cluster_frame_and_rate_cap(self):
    self._fc_engage()
    self._rx(self._ems16(main=True, active=True))
    # a burst of cluster frames 1 ms apart (not a real cluster; e.g. a replayed/duplicated frame): rate cap 15 ms
    for k in range(40):
      self.safety.set_timer(1_100_000 + k * 1000)
      if k % 10 == 0:
        self._rx(self._ems16(main=True, active=True))  # EMS16 every 10 ms: the lamp stays fresh
      self._rx(self._clu_raw())
    ts = [f["ts"] for f in self._self_tx()]
    self.assertGreater(len(ts), 1)
    self.assertTrue(all(b - a >= 15000 for a, b in zip(ts, ts[1:], strict=False)), ts)
    # and never more than one frame per received cluster frame
    self.assertEqual(len(ts), len(set(ts)))

  def test_fc_cancel_schedule_and_give_up(self):
    # factory cruise never goes off: 4 attempts x 3 frames, >= 400 ms apart, nothing after 2 s, no re-trigger
    self._fc_engage()
    self._fc_drive(300, 2_000_000)  # ~6 s
    tx = self._self_tx()
    self.assertEqual(12, len(tx))
    ts = [f["ts"] - tx[0]["ts"] for f in tx]
    self.assertLess(ts[-1], 2_000_000)
    groups = [ts[i:i + 3] for i in range(0, 12, 3)]
    for g in groups:
      self.assertEqual([self.FC_PERIOD_US, self.FC_PERIOD_US], [b - a for a, b in zip(g, g[1:], strict=False)], g)
    for a, b in zip(groups, groups[1:], strict=False):
      self.assertGreaterEqual(b[0] - a[-1], 400_000)
    # re-engaging openpilot while the same factory episode is still on does not restart it (bounded per episode)
    self.safety.set_controls_allowed(True)
    self.safety.set_heartbeat_engaged(True)
    self._fc_drive(100, 9_000_000)
    self.assertEqual(12, self.safety.get_self_tx_count())

  def test_fc_cancel_window_closes_at_2s(self):
    # long gaps between cluster frames (cluster offline): nothing is sent once 2 s have passed since the trigger
    self._fc_engage()
    self.safety.set_timer(2_000_000)
    self._rx(self._ems16(main=True, active=True))
    self.safety.set_timer(2_000_000 + 1_999_990)
    self._rx(self._ems16(main=True, active=True))  # fresh lamp (EMS16 is 100 Hz; only the cluster is offline)
    self.safety.set_timer(2_000_000 + 1_999_999)
    self._rx(self._clu_raw())
    self.assertEqual(1, self.safety.get_self_tx_count())
    for t in (2_000_000, 2_400_000):
      self.safety.set_timer(2_000_000 + t - 5)
      self._rx(self._ems16(main=True, active=True))
      self.safety.set_timer(2_000_000 + t)
      self._rx(self._clu_raw())
    self.assertEqual(1, self.safety.get_self_tx_count())

  def test_fc_cancel_stops_when_factory_cruise_off(self):
    # success: CRUISE_LAMP_S falls after the first frame -> no further frame, ever (4 is a toggle: never send it again)
    self._fc_engage()
    t_off = 2_000_000 + 15_000  # the EMS16 frames before the 2nd cluster frame report CRUISE_LAMP_S = 0
    self._fc_drive(200, 2_000_000, active=lambda t: t < t_off, main=True)
    self.assertEqual(1, self.safety.get_self_tx_count())

  def test_fc_cancel_rearms_for_a_new_episode(self):
    self._fc_engage()
    self._fc_drive(2, 2_000_000, active=True, main=True)
    n1 = self.safety.get_self_tx_count()
    self.assertEqual(2, n1)
    # the factory cruise goes off (episode over), openpilot is engaged again (driver), then the factory cruise comes on
    # again: a new episode with a fresh 3-frame attempt
    self.safety.set_timer(2_100_000)
    self._rx(self._ems16(main=True, active=False))
    self.safety.set_controls_allowed(True)
    self.safety.set_heartbeat_engaged(True)
    self._fc_drive(3, 3_000_000, active=True, main=True)
    self.assertEqual(n1 + 3, self.safety.get_self_tx_count())

  def test_fc_cancel_driver_button_aborts(self):
    # the driver presses SET/RES/pause (CruiseSwState != 0): he is operating the factory cruise himself -> stop for good
    for btn in (Buttons.RESUME, Buttons.SET, Buttons.CANCEL):
      self.setUp()
      self._fc_engage()
      self._rx(self._ems16(main=True, active=True))
      self.safety.set_timer(1_010_000)
      self._rx(self._clu_raw(button=btn))
      self._fc_drive(150, 2_000_000)
      self.assertEqual(0, self.safety.get_self_tx_count(), btn)

  def test_fc_cancel_main_held_frames_skipped(self):
    # cluster frames with MAIN held are never copied (no TX on them); a released frame afterwards is used
    self._fc_engage()
    self._rx(self._ems16(main=True, active=True))
    for k in range(3):
      self.safety.set_timer(1_010_000 + k * self.FC_PERIOD_US)
      self._rx(self._clu_raw(main=1))
    self.assertEqual(0, self.safety.get_self_tx_count())
    self.safety.set_timer(1_195_000)
    self._rx(self._ems16(main=True, active=True))
    self.safety.set_timer(1_200_000)
    self._rx(self._clu_raw())
    tx = self._self_tx()
    self.assertEqual(1, len(tx))
    self.assertEqual(0, (tx[0]["dat"][0] >> 3) & 1)

  def test_fc_cancel_bad_parity_cluster_frame_not_copied(self):
    self._fc_engage()
    self._rx(self._ems16(main=True, active=True))
    self.safety.set_timer(1_010_000)
    self._rx(self._clu_raw(good_parity=False))
    self.assertEqual(0, self.safety.get_self_tx_count())

  def test_fc_cancel_not_while_braking(self):
    self._fc_engage()
    self._rx(self._ems16(main=True, active=True))
    self._fc_drive(20, 2_000_000, brake=True)
    self.assertEqual(0, self.safety.get_self_tx_count())

  def test_fc_cancel_not_on_other_bus(self):
    self._fc_engage()
    self._rx(self._ems16(main=True, active=True))
    for bus in (1, 2):
      self.safety.set_timer(1_010_000 + bus * self.FC_PERIOD_US)
      msg = self._clu_raw()
      msg[0].bus = bus
      self._rx(msg)
    self.assertEqual(0, self.safety.get_self_tx_count())

  def test_fc_cancel_usb_clu11_still_rejected(self):
    # the firmware path does not open CLU11 to openpilot: every USB CLU11 is still blocked, in every state, incl. while
    # a cancel episode is running and with a perfectly formed CANCEL copied from the firmware's own frame
    self._fc_engage()
    self._fc_drive(3, 2_000_000)
    own = self._self_tx()[0]["dat"]
    self.assertFalse(self._tx(libsafety_py.make_CANPacket(0x4F1, 0, own)))
    for btn in range(8):
      for controls_allowed in (False, True):
        self.safety.set_controls_allowed(controls_allowed)
        self.assertFalse(self._tx(self._button_msg(btn)), btn)
        self.assertFalse(self._tx(self._button_msg(btn, bus=2)), btn)

  def test_fc_cancel_inert_without_factory_cruise(self):
    # normal pedal-long driving with buttons, brake, gas: zero CLU11 frames
    self._fc_engage()
    for k in range(500):
      self.safety.set_timer(2_000_000 + k * 10_000)
      self._rx(self._ems16())
      if k % 2 == 0:
        self._rx(self._clu_raw(button=(Buttons.SET if k % 100 < 6 else 0)))
      self._rx(self._interceptor_user_gas(0x1000 if k % 70 < 5 else 0))
    self.assertEqual(0, self.safety.get_self_tx_count())

  def test_fc_cancel_tx_echo_never_triggers(self):
    # review change #1: a CLU11 with returned = 1 (panda's own TX echo) is never treated as a cluster frame, whatever its
    # payload: neither a plain cluster-like frame (would be copied -> TX) nor a CANCEL like the firmware's own (would
    # otherwise abort the episode as a "driver press"). The next real frame (returned = 0) does trigger: positive
    # control, so the zero is the guard, not a dead stimulus. Counters stay consecutive so the RX counter check passes.
    self._fc_engage()
    self._fc_drive(1, 2_000_000)
    self.assertEqual(1, self.safety.get_self_tx_count())
    own = self._self_tx()[0]["dat"]
    c = (own[3] >> 4) & 0xF  # the firmware frame's counter: the cluster's next frame repeats it
    for k in range(30):  # 30 x 39.6 ms ~ 1.2 s: covers the 400 ms attempt gap + lamp confirmation, echoes only
      t = 2_100_000 + k * 2 * self.FC_PERIOD_US
      for j, button in enumerate((0, Buttons.CANCEL)):
        self.safety.set_timer(t + j * self.FC_PERIOD_US)
        self._rx(self._ems16(main=True, active=True))
        echo = self._clu_raw(button=button, counter=c)
        echo[0].returned = 1
        self.assertTrue(self._rx(echo))  # a valid frame for the RX checks: only the guard keeps it out
        c = (c + 1) & 0xF
    self.assertEqual(1, self.safety.get_self_tx_count())
    self.safety.set_timer(2_100_000 + 60 * self.FC_PERIOD_US)
    self._rx(self._ems16(main=True, active=True))
    self._rx(self._clu_raw(counter=c))
    self.assertEqual(2, self.safety.get_self_tx_count())

  def test_fc_cancel_tx_flood_keeps_order_and_rate(self):
    # review change #2 (what libsafety can model): openpilot floods the TX path (pedal command + LKAS11 + CLU11 attempts
    # over USB, 10 per cluster frame) during a whole cancel episode. The firmware's frames are byte-identical, in the
    # same order and at the same times as without the flood, >= 15 ms apart, 4 x 3, and no USB frame is ever recorded
    # as a self-TX (they go through safety_tx_hook, never can_send). The board's TX ring/FIFO is NOT modelled here:
    # see the call-path comment on hyundai_fc_cancel_cluster_frame and the road checklist.
    def episode(flood):
      self.setUp()
      self._fc_engage()
      for k in range(150):
        t = 2_000_000 + k * self.FC_PERIOD_US
        for sub in (0, 9900):
          self.safety.set_timer(t + sub)
          self._rx(self._ems16(main=True, active=True))
          if flood:
            for _ in range(5):
              self._tx(self._interceptor_gas_cmd(0))
              self._tx(self._torque_cmd_msg(0, 0))
              self._tx(self._button_msg(Buttons.CANCEL))
        self.safety.set_timer(t + 10000)
        self._rx(self._clu_raw(counter=k & 0xF))
      return [(f["ts"], f["dat"]) for f in self._self_tx()]

    quiet, flooded = episode(False), episode(True)
    self.assertEqual(12, len(quiet))
    self.assertEqual(quiet, flooded)
    ts = [t for t, _ in flooded]
    self.assertEqual(sorted(ts), ts)
    self.assertTrue(all(b - a >= 15000 for a, b in zip(ts, ts[1:], strict=False)))
    self.assertEqual([((d[3] >> 4) - 1) & 0xF for _, d in flooded], [((t - 2_010_000) // self.FC_PERIOD_US) & 0xF for t in ts])

  def test_fc_cancel_stale_lamp_no_frame(self):
    # every frame needs a fresh CRUISE_LAMP_S = 1: an EMS16 at most 30 ms old (EMS16 is 10 ms; worst log gap 36 ms is
    # rlog USB batching, not bus time). A stale lamp skips the cluster frame; the next fresh one is used.
    for age, sent in ((30_000, 1), (30_001, 0), (100_000, 0)):
      self.setUp()
      self._fc_engage()
      self.safety.set_timer(2_000_000)
      self._rx(self._ems16(main=True, active=True))
      self.safety.set_timer(2_000_000 + age)
      self._rx(self._clu_raw())
      self.assertEqual(sent, self.safety.get_self_tx_count(), age)
    self.safety.set_timer(2_200_000)
    self._rx(self._ems16(main=True, active=True))
    self.safety.set_timer(2_210_000)
    self._rx(self._clu_raw())
    self.assertEqual(1, self.safety.get_self_tx_count())

  def test_fc_cancel_next_attempt_needs_lamp_confirmation(self):
    # toggle-back guard: a further attempt only after >= 2 EMS16 with CRUISE_LAMP_S = 1 received >= 250 ms after the
    # previous attempt's last frame (worst lamp lag on the logs: 150.5 ms). Samples inside the 250 ms do not count, and
    # confirmations from before an attempt never carry over to the next one (checked for attempts 2 and 3).
    self._fc_engage()
    self._fc_drive(3, 2_000_000)
    self.assertEqual(3, self.safety.get_self_tx_count())
    for n in (3, 6):
      lt = self._self_tx()[-1]["ts"]
      for k in range(1, 25):  # lamp still on, every 10 ms, but only up to 240 ms after the attempt
        self.safety.set_timer(lt + k * 10_000)
        self._rx(self._ems16(main=True, active=True))
      for t, ems in ((420_000, True), (425_000, False), (430_000, True), (435_000, False)):
        self.safety.set_timer(lt + t)
        self._rx(self._ems16(main=True, active=True) if ems else self._clu_raw())
        if t == 425_000:
          self.assertEqual(n, self.safety.get_self_tx_count())  # 1 confirmation only
      self.assertEqual(n + 1, self.safety.get_self_tx_count())   # 2 confirmations: the next attempt starts
      self.assertEqual(lt + 435_000, self._self_tx()[-1]["ts"])
      for k in (1, 2):  # finish this attempt (3 frames), lamp fresh
        self.safety.set_timer(lt + 435_000 + k * self.FC_PERIOD_US - 5000)
        self._rx(self._ems16(main=True, active=True))
        self.safety.set_timer(lt + 435_000 + k * self.FC_PERIOD_US)
        self._rx(self._clu_raw())
      self.assertEqual(n + 3, self.safety.get_self_tx_count())

  def test_fc_cancel_late_lamp_fall_never_retried(self):
    # the ECM acted on attempt 1 but the lamp only says so late: up to the worst lag on the logs (150.5 ms) and well
    # beyond, up to just before attempt 2 would go. With the real bus (EMS16 10 ms, cluster 19.8 ms) not one frame
    # follows the lamp fall, and no 2nd attempt is ever started.
    for lag_us in (10_000, 150_500, 260_000, 395_000):
      self.setUp()
      self._fc_engage()
      self._fc_drive(3, 2_000_000)
      lt = self._self_tx()[-1]["ts"]
      self._fc_drive(100, lt + 10_000, active=lambda t, lt=lt, lag_us=lag_us: t < lt + lag_us, main=True)
      self.assertEqual(3, self.safety.get_self_tx_count(), lag_us)

  def test_fc_cancel_timing_constants(self):
    # pinned: 2 s give-up window, 15 ms rate cap (< the 19.8 ms cluster period, so one frame per cluster frame),
    # 400 ms between attempts (> the 352 ms worst ECM reaction seen to a driver pause press), 4 x 3 frames
    hc = open(HYUNDAI_COMMON_H).read()
    for name, val in (("HYUNDAI_FC_CANCEL_WINDOW_US", 2000000), ("HYUNDAI_FC_CANCEL_MIN_GAP_US", 15000),
                      ("HYUNDAI_FC_CANCEL_ATTEMPT_GAP_US", 400000), ("HYUNDAI_FC_CANCEL_ATTEMPTS", 4),
                      ("HYUNDAI_FC_CANCEL_MAX_FRAMES", 12), ("HYUNDAI_FC_CANCEL_BUTTON", 4),
                      ("HYUNDAI_FC_CANCEL_FRAMES_PER_ATTEMPT", 3), ("HYUNDAI_FC_CANCEL_LAMP_LAG_US", 250000),
                      ("HYUNDAI_FC_CANCEL_LAMP_CONFIRM", 2), ("HYUNDAI_FC_CANCEL_LAMP_FRESH_US", 30000)):
      m = re.search(rf"#define\s+{name}\s+(\d+)U", hc)
      self.assertIsNotNone(m, name)
      self.assertEqual(val, int(m.group(1)), name)


@parameterized_class(LDA_BUTTON)
class TestHyundaiNonSCCFCA11BrakeTestSafety(TestHyundaiNonSCCSafety):
  """
    TEST-ONLY FCA11 (0x38D) brake-injection mode on a non-SCC ICE car (HyundaiSafetyFlagsSP.FCA11_BRAKE_TEST).
    TX = CLU11 + FCA11 with check_relay=true: the stock camera FCA11 (bus 2) is no longer forwarded to bus 0, and a
    bus-0 FCA11 would be a relay malfunction (on the target car 0x38D is only ever received on bus 2, per route logs).
    ONLY FCA11 is taken over: LKAS11 (0x340) / LFAHDA_MFC (0x485) are NOT in the TX list, so the camera's own
    lane-keeping messages keep forwarding bus 2 -> 0 and panda can never transmit them (no second source).
  """
  TX_MSGS = [[0x4F1, 0], [0x38D, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x38D,)}  # FCA11 only
  FWD_BLACKLISTED_ADDRS = {2: [0x38D]}
  DEC_CAP = HYUNDAI_FCA11_TEST_MAX_DEC  # parked mode (64 alone): 0.10 g; the rolling subclass overrides it (0013)

  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiNonSCCFCA11BrakeTestSafety":
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_can_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(self._test_sp())
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, 0)
    self.safety.init_tests()

  def _test_sp(self) -> int:
    return HyundaiSafetyFlagsSP.NON_SCC | HyundaiSafetyFlagsSP.FCA11_BRAKE_TEST | self.SAFETY_PARAM_SP

  def _fca11(self, dec=0, prefill=0, hba=0, cmd_act=0, dec_cmd_act=0, stop_req=0, warn=0):
    # dec is RAW CR_VSM_DecCmd (0.01 g/LSB); +0.25 LSB so float scaling can't truncate n*0.01/0.01 down to n-1
    values = {"CR_VSM_DecCmd": (dec + 0.25) * 0.01 if dec else 0, "CF_VSM_Prefill": prefill, "CF_VSM_HBACmd": hba, "FCA_CmdAct": cmd_act,
              "CF_VSM_DecCmdAct": dec_cmd_act, "FCA_StopReq": stop_req, "FCA_Status": 2, "CF_VSM_Warn": warn}
    return self.packer.make_can_msg_safety("FCA11", 0, values)

  def _brake_cmd(self, dec):
    return self._fca11(dec=dec, prefill=1, cmd_act=1, dec_cmd_act=1)

  def test_fca11_helper_raw_decel(self):
    # guards the helper: dec maps to the raw byte the panda reads (data[1])
    for dec in (0, 1, self.DEC_CAP, self.DEC_CAP + 1, 255):
      msg = self._fca11(dec=dec)  # keep the owning cffi pointer alive while reading it
      self.assertEqual(dec, msg[0].data[1])

  def test_c_python_cap_match(self):
    # boundary is the C cap of this mode (HYUNDAI_FCA11_TEST_MAX_DEC parked, HYUNDAI_FCA11_ROLL_MAX_DEC rolling): fails
    # unless it equals the Python mirror
    self.assertTrue(self._tx(self._brake_cmd(self.DEC_CAP)))
    self.assertFalse(self._tx(self._brake_cmd(self.DEC_CAP + 1)))

  def test_decel_zero_allowed(self):
    self.assertTrue(self._tx(self._fca11()))

  def test_decel_within_cap_allowed(self):
    # independent of controls_allowed: openpilot is disengaged during the parked test
    for controls_allowed in (False, True):
      self.safety.set_controls_allowed(controls_allowed)
      for dec in range(self.DEC_CAP + 1):
        self.assertTrue(self._tx(self._fca11(dec=dec)), dec)
        self.assertTrue(self._tx(self._brake_cmd(dec)), dec)
      for sig in ("prefill", "cmd_act", "dec_cmd_act"):
        self.assertTrue(self._tx(self._fca11(**{sig: 1})), sig)

  def test_decel_over_cap_blocked(self):
    for dec in range(self.DEC_CAP + 1, 256):
      self.assertFalse(self._tx(self._fca11(dec=dec)), dec)
      self.assertFalse(self._tx(self._brake_cmd(dec)), dec)

  def test_hba_blocked(self):
    for hba in (1, 2, 3):
      self.assertFalse(self._tx(self._fca11(hba=hba)), hba)
      self.assertFalse(self._tx(self._fca11(dec=1, prefill=1, hba=hba, cmd_act=1, dec_cmd_act=1)), hba)

  def test_stop_req_blocked(self):
    self.assertFalse(self._tx(self._fca11(stop_req=1)))
    self.assertFalse(self._tx(self._fca11(dec=1, prefill=1, cmd_act=1, dec_cmd_act=1, stop_req=1)))

  def test_no_actuation_while_moving(self):
    self._rx(self._speed_msg(self.STANDSTILL_THRESHOLD + 1))
    self.assertTrue(self.safety.get_vehicle_moving())
    self.assertTrue(self._tx(self._fca11()))  # a passive frame is still fine
    for kw in ({"dec": 1}, {"prefill": 1}, {"cmd_act": 1}, {"dec_cmd_act": 1}):
      self.assertFalse(self._tx(self._fca11(**kw)), kw)
    self._rx(self._speed_msg(0))
    self.assertFalse(self.safety.get_vehicle_moving())
    self.assertTrue(self._tx(self._brake_cmd(1)))

  def test_tx_list_structure(self):
    # FCA11 present with check_relay=true: received on bus 0 -> relay malfunction; bus 2 -> not forwarded, no fault
    self._rx(common.make_msg(2, 0x38D, 8))
    self.assertFalse(self.safety.get_relay_malfunction())
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x38D))
    self._rx(common.make_msg(0, 0x38D, 8))
    self.assertTrue(self.safety.get_relay_malfunction())
    # no openpilot longitudinal of any kind: SCC-replacement set, FCA12, radar UDS, both pedal command IDs
    self.safety.set_relay_malfunction(False)
    self.safety.set_controls_allowed(True)
    for addr in (0x420, 0x421, 0x50A, 0x389, 0x4A2, 0x483, 0x7D0, 0x200, 0x700):
      self.assertFalse(self._tx(common.make_msg(0, addr, 8)), f"{addr=:#x}")

  def test_camera_lkas_streams_forwarded(self):
    # the camera's LKAS11 / LFAHDA_MFC keep flowing bus 2 -> 0 while armed (only FCA11 is blocked) ...
    for addr in (0x340, 0x485):
      self._rx(common.make_msg(2, addr, 8))
      self.assertEqual(0, self.safety.safety_fwd_hook(2, addr), f"{addr=:#x}")
      # ... and are not relay-checked: seeing them on bus 0 (e.g. the forwarded stream itself) is no fault
      self._rx(common.make_msg(0, addr, 8))
      self.assertFalse(self.safety.get_relay_malfunction(), f"{addr=:#x}")
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x38D))
    self.assertFalse(self.safety.get_relay_malfunction())

  def test_lkas_tx_rejected(self):
    # panda can't become a second LKAS11/LFAHDA_MFC source in test mode, not even a zero-torque / idle frame,
    # regardless of controls_allowed
    for controls_allowed in (False, True):
      self.safety.set_controls_allowed(controls_allowed)
      self.assertFalse(self._tx(self._torque_cmd_msg(0, steer_req=0)))
      self.assertFalse(self._tx(self._torque_cmd_msg(0, steer_req=1)))
      self.assertFalse(self._tx(common.make_msg(0, 0x340, 8)))
      self.assertFalse(self._tx(common.make_msg(0, 0x485, 4)))
      self.assertFalse(self._tx(self.packer.make_can_msg_safety("LFAHDA_MFC", 0, {})))

  def test_not_armed_keeps_lkas_takeover(self):
    # without the test bit, normal non-SCC openpilot steering is unchanged: LKAS11/LFAHDA_MFC transmittable and the
    # camera's copies blocked (the base HYUNDAI_TX_MSGS rule)
    self.safety.set_current_safety_param_sp(HyundaiSafetyFlagsSP.NON_SCC | self.SAFETY_PARAM_SP)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, 0)
    self.assertTrue(self._tx(self._torque_cmd_msg(0, steer_req=0)))
    self.assertTrue(self._tx(common.make_msg(0, 0x485, 4)))
    for addr in (0x340, 0x485):
      self.assertEqual(-1, self.safety.safety_fwd_hook(2, addr), f"{addr=:#x}")
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x38D))

  # Steering can't be transmitted in this mode (test_lkas_tx_rejected), so the inherited torque-steering suites only
  # fail at the TX allowlist. They are replaced by that one explicit check.
  @unittest.skip("LKAS11 not in the FCA11 test TX list (see test_lkas_tx_rejected)")
  def test_steer_safety_check(self):
    pass

  @unittest.skip("LKAS11 not in the FCA11 test TX list (see test_lkas_tx_rejected)")
  def test_non_realtime_limit_up(self):
    pass

  @unittest.skip("LKAS11 not in the FCA11 test TX list (see test_lkas_tx_rejected)")
  def test_steer_req_bit(self):
    pass

  @unittest.skip("LKAS11 not in the FCA11 test TX list (see test_lkas_tx_rejected)")
  def test_steer_req_bit_frames(self):
    pass

  @unittest.skip("LKAS11 not in the FCA11 test TX list (see test_lkas_tx_rejected)")
  def test_steer_req_bit_multi_invalid(self):
    pass

  @unittest.skip("LKAS11 not in the FCA11 test TX list (see test_lkas_tx_rejected)")
  def test_steer_req_bit_realtime(self):
    pass

  @unittest.skip("LKAS11 not in the FCA11 test TX list (see test_lkas_tx_rejected)")
  def test_against_torque_driver(self):
    pass

  @unittest.skip("LKAS11 not in the FCA11 test TX list (see test_lkas_tx_rejected)")
  def test_realtime_limits(self):
    pass

  def test_test_bit_overrides_gas_interceptor(self):
    for dialect in (0, HyundaiSafetyFlagsSP.GAS_INTERCEPTOR_REMAPPED):
      self.safety.set_current_safety_param_sp(self._test_sp() | HyundaiSafetyFlagsSP.GAS_INTERCEPTOR | dialect)
      self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, 0)
      self.safety.set_controls_allowed(True)
      self.assertTrue(self._tx(self._brake_cmd(1)))
      self.assertFalse(self._tx(common.make_msg(0, 0x200, 6)))
      self.assertFalse(self._tx(common.make_msg(0, 0x700, 6)))
      self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x38D))

  def test_test_bit_overrides_pedal_buttons(self):
    # integration (0002 v2 + 0005): with BOTH bits set, none of the pedal-mode button changes leak into test mode.
    # The factory MAIN lamp is not a lockout here, CLU11 cancel-only stays transmittable (the pedal list has no CLU11),
    # and the pause/resume re-engage state machine is inert.
    for dialect in (0, HyundaiSafetyFlagsSP.GAS_INTERCEPTOR_REMAPPED):
      with self.subTest(dialect=dialect):
        self.safety.set_current_safety_param_sp(self._test_sp() | HyundaiSafetyFlagsSP.GAS_INTERCEPTOR | dialect)
        self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, 0)
        self.safety.init_tests()
        # stock non-SCC MAIN semantics (pedal mode would force acc_main_on = true)
        self._rx(self._acc_state_msg(False))
        self.assertFalse(self.safety.get_acc_main_on())
        self._rx(self._acc_state_msg(True))
        self.assertTrue(self.safety.get_acc_main_on())
        # CLU11: cancel only, while the factory cruise is on
        self._rx(self._pcm_status_msg(True))
        self.safety.set_controls_allowed(False)
        self.assertTrue(self._tx(self._button_msg(Buttons.CANCEL)))
        for btn in (Buttons.RESUME, Buttons.SET):
          self.assertFalse(self._tx(self._button_msg(btn)), btn)
        # pause/resume press + debounced release while disengaged: no grant
        self._rx(self._pcm_status_msg(False))
        self.safety.set_controls_allowed(False)
        for btn in [Buttons.NONE] * 5 + [Buttons.CANCEL] * 5 + [Buttons.NONE] * 5:
          self._rx(self._button_msg(btn))
        self.assertFalse(self.safety.get_controls_allowed())
        # the test brake path itself is unchanged
        self.assertTrue(self._tx(self._brake_cmd(1)))

  def test_test_bit_overrides_timed_factory_cancel(self):
    # integration (timed factory-cruise cancel + FCA11 test modes): with the test bit(s) AND GAS_INTERCEPTOR set (both
    # dialects), panda never transmits a CLU11 by itself. Positive control: the identical episode with the pedal alone
    # (no test bits) does transmit, so the zero is the test bit's gating, not a dead stimulus.
    def ems16(main, active):
      values = {"CRUISE_LAMP_M": main, "CRUISE_LAMP_S": active, "AliveCounter": self.cnt_gas % 4}
      self.__class__.cnt_gas += 1
      return self.packer.make_can_msg_safety("EMS16", 0, values, fix_checksum=checksum)

    def clu_raw(counter):
      par = bin(counter).count("1") & 1
      return libsafety_py.make_CANPacket(0x4F1, 0, bytes([par << 5, 0x7A, 0x00, (counter << 4) | 1]))

    test_bits = self._test_sp() & ~HyundaiSafetyFlagsSP.NON_SCC
    for dialect in (0, HyundaiSafetyFlagsSP.GAS_INTERCEPTOR_REMAPPED):
      for bits in (test_bits, 0):
        with self.subTest(dialect=dialect, test_bits=bits):
          self.safety.set_current_safety_param_sp(self._test_sp() & ~test_bits | bits | HyundaiSafetyFlagsSP.GAS_INTERCEPTOR | dialect)
          self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, 0)
          self.safety.init_tests()
          n = fc_cancel_episode(self.safety, self._rx, ems16, clu_raw, self._user_brake_msg)
          if bits:
            self.assertEqual(0, n)
            self.assertFalse(self._tx(clu_raw(1)))  # USB CLU11 other than cancel-only is still policed
          else:
            self.assertEqual(12, n)  # positive control: pedal mode, 4 attempts x 3 frames

  # *** mode gating: honored only when armed (bit + non-SCC ICE + no hyundai_longitudinal) ***

  def _assert_not_armed(self):
    # normal non-SCC behavior: FCA11 not transmittable at all, stock camera FCA11 forwarded, no relay fault from bus 0
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(self._fca11()))
    self.assertFalse(self._tx(self._brake_cmd(1)))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x38D))
    self._rx(common.make_msg(0, 0x38D, 8))
    self.assertFalse(self.safety.get_relay_malfunction())

  def test_without_test_bit(self):
    self.safety.set_current_safety_param_sp(HyundaiSafetyFlagsSP.NON_SCC | self.SAFETY_PARAM_SP)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, 0)
    self._assert_not_armed()

  def test_ignored_without_non_scc(self):
    self.safety.set_current_safety_param_sp(HyundaiSafetyFlagsSP.FCA11_BRAKE_TEST | self.SAFETY_PARAM_SP)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, 0)
    self._assert_not_armed()

  def test_ignored_on_unsupported_cars(self):
    for param in (HyundaiSafetyFlags.EV_GAS, HyundaiSafetyFlags.HYBRID_GAS, HyundaiSafetyFlags.FCEV_GAS,
                  HyundaiSafetyFlags.CAMERA_SCC):
      with self.subTest(param=param):
        self.safety.set_current_safety_param_sp(self._test_sp())
        self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, param)
        self.safety.set_controls_allowed(True)
        self.assertFalse(self._tx(self._brake_cmd(1)))
    for mode in (CarParams.SafetyModel.hyundaiLegacy, CarParams.SafetyModel.hyundaiCanfd):
      with self.subTest(mode=mode):
        self.safety.set_current_safety_param_sp(self._test_sp())
        self.safety.set_safety_hooks(mode, 0)
        self.safety.set_controls_allowed(True)
        self.assertFalse(self._tx(self._brake_cmd(1)))

  def test_ignored_with_hyundai_longitudinal(self):
    # LONG is honored in this ALLOW_DEBUG test build -> SCC-replacement TX set, FCA11 stays fully blocked for actuation
    self.safety.set_current_safety_param_sp(self._test_sp())
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, HyundaiSafetyFlags.LONG)
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(common.make_msg(0, 0x50A, 8)))  # proves hyundai_longitudinal is active
    self.assertTrue(self._tx(self._fca11()))
    for dec in (1, HYUNDAI_FCA11_TEST_MAX_DEC):
      self.assertFalse(self._tx(self._brake_cmd(dec)))
      self.assertFalse(self._tx(self._fca11(dec=dec)))
    self.assertFalse(self._tx(self._fca11(cmd_act=1)))
    self.assertFalse(self._tx(self._fca11(dec_cmd_act=1)))


@parameterized_class(LDA_BUTTON)
class TestHyundaiNonSCCFCA11RollingTestSafety(TestHyundaiNonSCCFCA11BrakeTestSafety):
  """
    TEST-ONLY low-speed ROLLING extension (FCA11_BRAKE_TEST | FCA11_ROLLING_TEST). Same TX list, cap, HBA/StopReq rules as
    the parked mode; the standstill-only rule is replaced by the rolling window: actuation only with every wheel in
    (MIN, MAX] km/h, gear D (LVR12), driver brake and gas released, every input fresh (<= 100 ms), and never after a
    latched cut (brake / gas / gear != D / over the ceiling / at or below the floor) until the next init.
    setUp puts the car INSIDE the window, so the inherited parked-mode tests run against an open window.
  """
  # camera FCA11 blocking is DYNAMIC in this mode (hyundai_fwd_hook): forwarded until panda transmits, see the tests below
  FWD_BLACKLISTED_ADDRS = {}
  RAW_PER_KPH = 32  # WHL_SPD11 0.03125 km/h/LSB
  IN_WINDOW_RAW = 20 * 32  # 20 km/h
  DEC_CAP = HYUNDAI_FCA11_ROLL_MAX_DEC  # 0013: rolling mode cap 0.30 g (parked stays HYUNDAI_FCA11_TEST_MAX_DEC)
  cnt_lvr = 0

  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiNonSCCFCA11RollingTestSafety":
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    super().setUp()
    self.safety.set_timer(1000000)
    self._open_window()

  def _test_sp(self) -> int:
    return super()._test_sp() | HyundaiSafetyFlagsSP.FCA11_ROLLING_TEST

  def _gear_msg(self, gear):
    return self.packer.make_can_msg_safety("LVR12", 0, {"CF_Lvr_Gear": gear})

  def _wheel_msg(self, fl, fr=None, rl=None, rr=None):
    vals = {"FL": fl, "FR": fl if fr is None else fr, "RL": fl if rl is None else rl, "RR": fl if rr is None else rr}
    values = {"WHL_SPD_%s" % k: v * 0.03125 for k, v in vals.items()}
    values["WHL_SPD_AliveCounter_LSB"] = (self.cnt_speed % 16) & 0x3
    values["WHL_SPD_AliveCounter_MSB"] = (self.cnt_speed % 16) >> 2
    self.__class__.cnt_speed += 1
    return self.packer.make_can_msg_safety("WHL_SPD11", 0, values, fix_checksum=checksum)

  def _open_window(self, speed_raw=None):
    self.assertTrue(self._rx(self._user_brake_msg(False)))
    self.assertTrue(self._rx(self._user_gas_msg(0)))
    self.assertTrue(self._rx(self._gear_msg(5)))
    self.assertTrue(self._rx(self._wheel_msg(self.IN_WINDOW_RAW if speed_raw is None else speed_raw)))

  def _rearm(self, sp=None):
    # a fresh arm = a new 0xdc: set_safety_hooks re-runs hyundai_init (clears the cut and all window evidence)
    self.safety.set_current_safety_param_sp(self._test_sp() if sp is None else sp)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, 0)
    self.safety.set_timer(1000000)

  def test_vehicle_moving(self):
    # the inherited check starts from standstill; setUp put the car inside the rolling window
    self._rearm()
    super().test_vehicle_moving()

  # inherited MADS checks assume a fresh init (no prior brake/gas/speed traffic): run them from a fresh arm
  def test_mads_button_not_engaged_without_press(self):
    self._rearm()
    super().test_mads_button_not_engaged_without_press()

  def test_enable_control_allowed_with_manual_mads_button_state(self):
    self._rearm()
    super().test_enable_control_allowed_with_manual_mads_button_state()

  # *** the window ***

  def test_c_python_speed_window_match(self):
    lo = HYUNDAI_FCA11_ROLL_MIN_SPEED_KPH * self.RAW_PER_KPH
    hi = HYUNDAI_FCA11_ROLL_MAX_SPEED_KPH * self.RAW_PER_KPH
    for raw, ok in ((lo, False), (lo + 1, True), (hi, True), (hi + 1, False)):
      with self.subTest(raw=raw):
        self._rearm()
        self._open_window(raw)
        self.assertEqual(ok, self._tx(self._brake_cmd(self.DEC_CAP)))

  def test_standstill_blocked(self):
    # the rolling mode never actuates at standstill (that is the parked mode's job, and stopping the car is not allowed)
    self._rearm()
    self._open_window(0)
    self.assertTrue(self._tx(self._fca11()))
    for kw in ({"dec": 1}, {"prefill": 1}, {"cmd_act": 1}, {"dec_cmd_act": 1}):
      self.assertFalse(self._tx(self._fca11(**kw)), kw)

  def test_no_actuation_while_moving(self):
    # parked-mode rule replaced: inside the window actuation is allowed while moving
    self.assertTrue(self.safety.get_vehicle_moving())
    self.assertTrue(self._tx(self._brake_cmd(self.DEC_CAP)))

  def test_each_wheel_policed(self):
    lo = HYUNDAI_FCA11_ROLL_MIN_SPEED_KPH * self.RAW_PER_KPH
    hi = HYUNDAI_FCA11_ROLL_MAX_SPEED_KPH * self.RAW_PER_KPH
    for i in range(4):
      for bad in (lo, hi + 1):
        with self.subTest(wheel=i, bad=bad):
          self._rearm()
          self._open_window()
          w = [self.IN_WINDOW_RAW] * 4
          w[i] = bad
          self._rx(self._wheel_msg(*w))
          self.assertFalse(self._tx(self._brake_cmd(1)))

  def test_needs_every_input_seen(self):
    feeds = {"brake": lambda: self._rx(self._user_brake_msg(False)), "gas": lambda: self._rx(self._user_gas_msg(0)),
             "gear": lambda: self._rx(self._gear_msg(5)), "speed": lambda: self._rx(self._wheel_msg(self.IN_WINDOW_RAW))}
    for missing in feeds:
      with self.subTest(missing=missing):
        self._rearm()
        # timer near 0 (just after boot / timer wrap): a never-received input's timestamp (0) looks FRESH here, so only
        # the "every input seen since this arm" rule can block it (brake and gas default to released)
        self.safety.set_timer(50000)
        for name, f in feeds.items():
          if name != missing:
            f()
        self.assertFalse(self._tx(self._brake_cmd(1)))
        self.assertTrue(self._tx(self._fca11()))

  def test_stale_input_blocks(self):
    feeds = {"brake": lambda: self._rx(self._user_brake_msg(False)), "gas": lambda: self._rx(self._user_gas_msg(0)),
             "gear": lambda: self._rx(self._gear_msg(5)), "speed": lambda: self._rx(self._wheel_msg(self.IN_WINDOW_RAW))}
    for stale in feeds:
      with self.subTest(stale=stale):
        self._rearm()
        self._open_window()
        self.assertTrue(self._tx(self._brake_cmd(1)))
        self.safety.set_timer(1000000 + 100001)  # just past 100 ms
        for name, f in feeds.items():
          if name != stale:
            f()
        self.assertFalse(self._tx(self._brake_cmd(1)))
        f = feeds[stale]
        f()
        self.assertTrue(self._tx(self._brake_cmd(1)))
    # exactly 100 ms old is still fresh
    self._rearm()
    self._open_window()
    self.safety.set_timer(1000000 + 100000)
    self.assertTrue(self._tx(self._brake_cmd(1)))

  # *** latching cut ***

  def _assert_cut_latched(self):
    self.assertFalse(self._tx(self._brake_cmd(1)))
    self.assertFalse(self._tx(self._fca11(prefill=1)))
    self.assertTrue(self._tx(self._fca11()))  # the passive carrier stream keeps flowing (no 0x38D gap)
    self._open_window()  # conditions fully restored ...
    self.assertFalse(self._tx(self._brake_cmd(1)))  # ... still cut until re-init
    self._rearm()
    self._open_window()
    self.assertTrue(self._tx(self._brake_cmd(1)))  # a fresh arm (0xdc) starts un-cut

  def test_cut_on_driver_brake(self):
    self.assertTrue(self._tx(self._brake_cmd(5)))
    self._rx(self._user_brake_msg(True))
    self._assert_cut_latched()

  def test_cut_on_driver_gas(self):
    self.assertTrue(self._tx(self._brake_cmd(5)))
    self._rx(self._user_gas_msg(1))
    self._assert_cut_latched()

  def test_cut_on_gear_not_d(self):
    for gear in (0, 4, 6, 7, 8, 12):
      with self.subTest(gear=gear):
        self._rearm()
        self._open_window()
        self.assertTrue(self._tx(self._brake_cmd(1)))
        self._rx(self._gear_msg(gear))
        self._assert_cut_latched()

  def test_cut_on_over_ceiling(self):
    self._rx(self._wheel_msg(HYUNDAI_FCA11_ROLL_MAX_SPEED_KPH * self.RAW_PER_KPH + 1))
    self._assert_cut_latched()

  def test_cut_on_floor(self):
    # coasting down to the floor ends the session: the panda can never brake the car to a stop
    self._rx(self._wheel_msg(HYUNDAI_FCA11_ROLL_MIN_SPEED_KPH * self.RAW_PER_KPH))
    self._assert_cut_latched()

  def test_cut_keeps_cap_hba_stopreq(self):
    # inside the window the cap (0.30 g rolling), HBA and StopReq limits still apply
    self.assertFalse(self._tx(self._brake_cmd(self.DEC_CAP + 1)))
    self.assertFalse(self._tx(self._fca11(dec=1, prefill=1, hba=1, cmd_act=1, dec_cmd_act=1)))
    self.assertFalse(self._tx(self._fca11(dec=1, prefill=1, cmd_act=1, dec_cmd_act=1, stop_req=1)))

  # *** dynamic camera blocking: auto hand-back ***

  def _cam_fca11(self, **kw):
    values = {"FCA_Status": 2}
    values.update(kw)
    return self.packer.make_can_msg_safety("FCA11", 2, values)

  def test_tx_list_structure(self):
    # relay check unchanged (bus-0 FCA11 = relay malfunction); camera blocking is dynamic, see test_camera_handback_*
    # (no _rearm here: set_safety_hooks resets safety_mode_cnt, which mutes the relay check right after init)
    self._rx(common.make_msg(0, 0x38D, 8))
    self.assertTrue(self.safety.get_relay_malfunction())
    self.safety.set_relay_malfunction(False)
    self.safety.set_controls_allowed(True)
    for addr in (0x420, 0x421, 0x50A, 0x389, 0x4A2, 0x483, 0x7D0, 0x200, 0x700):
      self.assertFalse(self._tx(common.make_msg(0, addr, 8)), f"{addr=:#x}")

  def test_camera_lkas_streams_forwarded(self):
    for addr in (0x340, 0x485):
      self._rx(common.make_msg(2, addr, 8))
      self.assertEqual(0, self.safety.safety_fwd_hook(2, addr), f"{addr=:#x}")
      self._rx(common.make_msg(0, addr, 8))
      self.assertFalse(self.safety.get_relay_malfunction(), f"{addr=:#x}")

  def test_camera_handback_on_tx_stale(self):
    self._rearm()
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x38D))  # armed but not yet transmitting: camera still owns it
    self._open_window()
    self.assertTrue(self._tx(self._fca11()))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x38D))  # our stream replaces the camera's
    self.safety.set_timer(1000000 + 100000)
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x38D))  # 100 ms since our last frame: still ours
    self.safety.set_timer(1000000 + 100001)
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x38D))   # host silent > 100 ms: camera forwarded again
    self._open_window()
    self.assertTrue(self._tx(self._fca11()))                      # resuming re-blocks
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x38D))

  def test_rejected_tx_does_not_block_camera(self):
    self._rearm()
    self._open_window()
    self.assertFalse(self._tx(self._brake_cmd(self.DEC_CAP + 1)))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x38D))

  def test_camera_request_hands_back(self):
    # a REAL camera request (any actuation or warning field) hands FCA11 back to the camera for the rest of the arm
    reqs = ({"CR_VSM_DecCmd": 0.5}, {"CF_VSM_Prefill": 1}, {"CF_VSM_HBACmd": 1}, {"CF_VSM_Warn": 1}, {"FCA_CmdAct": 1},
            {"FCA_StopReq": 1}, {"CF_VSM_DecCmdAct": 1})
    for req in reqs:
      with self.subTest(req=req):
        self._rearm()
        self._open_window()
        self.assertTrue(self._rx(self._cam_fca11()))   # a passive camera frame changes nothing
        self.assertTrue(self._tx(self._brake_cmd(5)))
        self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x38D))
        self.assertTrue(self._rx(self._cam_fca11(**req)))
        self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x38D))   # camera forwarded immediately
        self.assertFalse(self._tx(self._brake_cmd(5)))
        self.assertFalse(self._tx(self._fca11()))                    # not even a passive frame: one source only
        self._rx(self._cam_fca11())
        self._open_window()
        self.assertFalse(self._tx(self._fca11()))                    # latched until re-arm
        self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x38D))
        self._rearm()
        self._open_window()
        self.assertTrue(self._tx(self._brake_cmd(5)))

  def test_max_continuous_actuation(self):
    self._rearm()
    self._open_window()
    self.assertTrue(self._tx(self._brake_cmd(5)))
    self.safety.set_timer(1000000 + 1200000)
    self._open_window()
    self.assertTrue(self._tx(self._brake_cmd(5)))   # exactly 1.2 s: still allowed
    self.safety.set_timer(1000000 + 1200001)
    self._open_window()
    self.assertFalse(self._tx(self._brake_cmd(5)))  # longer: rejected
    self.assertTrue(self._tx(self._fca11()))        # a passive frame ends the actuation ...
    self.assertTrue(self._tx(self._brake_cmd(5)))   # ... and a new one may start


  # *** 0013: rolling-mode decel cap 0.30 g (scaling test); parked and normal modes unchanged ***

  def test_rolling_cap_c_python_match(self):
    hm = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "modes", "hyundai.h")).read()
    m = re.search(r"#define\s+HYUNDAI_FCA11_ROLL_MAX_DEC\s+(\d+)", hm)
    self.assertIsNotNone(m)
    self.assertEqual(HYUNDAI_FCA11_ROLL_MAX_DEC, int(m.group(1)))
    self.assertEqual(30, HYUNDAI_FCA11_ROLL_MAX_DEC)   # 0.30 g, the owner-set scaling-test ceiling
    self.assertEqual(10, HYUNDAI_FCA11_TEST_MAX_DEC)   # the parked cap is NOT raised

  def test_rolling_cap_boundary_all_shapes(self):
    # 0.30 accepted, 0.31 rejected, for every frame shape the runner sends (variant A, variant B, full brake_cmd)
    shapes = ({"warn": 2, "dec_cmd_act": 1, "prefill": 1}, {"warn": 3, "cmd_act": 1, "prefill": 1},
              {"prefill": 1, "cmd_act": 1, "dec_cmd_act": 1}, {})
    for kw in shapes:
      for dec, ok in ((HYUNDAI_FCA11_TEST_MAX_DEC + 1, True), (20, True), (30, True), (31, False), (255, False)):
        with self.subTest(kw=kw, dec=dec):
          self._rearm()
          self._open_window()
          self.assertEqual(ok, self._tx(self._fca11(dec=dec, **kw)))
          if not ok:
            self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x38D))  # a rejected frame never blocks the camera

  def test_rolling_cap_keeps_window(self):
    # the raised cap opens NOTHING else: 0.30 g outside the window / at standstill / after a cut is still rejected
    lo = HYUNDAI_FCA11_ROLL_MIN_SPEED_KPH * self.RAW_PER_KPH
    hi = HYUNDAI_FCA11_ROLL_MAX_SPEED_KPH * self.RAW_PER_KPH
    for raw in (0, lo, hi + 1):
      with self.subTest(raw=raw):
        self._rearm()
        self._open_window(raw)
        self.assertFalse(self._tx(self._brake_cmd(30)))
    for c in ("brake", "gas", "gear"):
      with self.subTest(cut=c):
        self._rearm()
        self._open_window()
        self.assertTrue(self._tx(self._brake_cmd(30)))
        {"brake": lambda: self._rx(self._user_brake_msg(True)), "gas": lambda: self._rx(self._user_gas_msg(1)),
         "gear": lambda: self._rx(self._gear_msg(4))}[c]()
        self.assertFalse(self._tx(self._brake_cmd(30)))
        self._open_window()
        self.assertFalse(self._tx(self._brake_cmd(30)))   # latched until re-arm

  def test_rolling_cap_keeps_freshness(self):
    self._rearm()
    self._open_window()
    self.assertTrue(self._tx(self._brake_cmd(30)))
    self.safety.set_timer(1000000 + 100001)
    self.assertFalse(self._tx(self._brake_cmd(30)))

  def test_rolling_cap_keeps_clock(self):
    self._rearm()
    self._open_window()
    self.assertTrue(self._tx(self._fca11(warn=1)))       # lead-in starts the episode
    self.safety.set_timer(1000000 + 1200000)
    self._open_window()
    self.assertTrue(self._tx(self._fca11(dec=30, prefill=1, cmd_act=1, warn=3)))
    self.safety.set_timer(1000000 + 1200001)
    self._open_window()
    self.assertFalse(self._tx(self._fca11(dec=30, prefill=1, cmd_act=1, warn=3)))

  def test_rolling_cap_keeps_camera_handback(self):
    self._rearm()
    self._open_window()
    self.assertTrue(self._tx(self._brake_cmd(30)))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x38D))
    self._rx(self._cam_fca11(CF_VSM_Warn=1))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x38D))
    self.assertFalse(self._tx(self._brake_cmd(30)))
    self.assertFalse(self._tx(self._brake_cmd(1)))

  def test_rolling_cap_camera_handback_each_strength(self):
    # a REAL camera request takes FCA11 back on the next frame at EVERY commanded strength, incl. above the old 0.10 cap
    for dec in (20, 30):
      with self.subTest(dec=dec):
        self._rearm()
        self._open_window()
        self.assertTrue(self._tx(self._fca11(dec=dec, prefill=1, cmd_act=1, warn=3)))
        self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x38D))
        self._rx(self._cam_fca11(CR_VSM_DecCmd=0.5))
        self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x38D))           # camera forwarded at once
        self.assertFalse(self._tx(self._fca11(dec=dec, prefill=1, cmd_act=1, warn=3)))
        self.assertFalse(self._tx(self._fca11()))                           # not even a passive frame
        self._open_window()
        self._rx(self._cam_fca11())
        self.assertFalse(self._tx(self._fca11(dec=dec, prefill=1, cmd_act=1, warn=3)))   # latched until re-arm

  def test_rolling_cap_camera_request_mid_pulse_030(self):
    # 0.30 g pulse running (lead-in + 0.3 s into the actuation), the camera raises its own warning: next frame refused
    self._rearm()
    self._open_window()
    self.assertTrue(self._tx(self._fca11(warn=1)))
    for k in range(1, 9):
      self.safety.set_timer(1000000 + 500000 + k * 40000)
      self._open_window()
      self.assertTrue(self._tx(self._fca11(dec=30, prefill=1, cmd_act=1, warn=3)), k)
    self._rx(self._cam_fca11(CF_VSM_Warn=2))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x38D))
    self.assertFalse(self._tx(self._fca11(dec=30, prefill=1, cmd_act=1, warn=3)))
    self.assertFalse(self._tx(self._fca11(dec=20, prefill=1, cmd_act=1, warn=3)))
    self.assertFalse(self._tx(self._fca11()))

  def test_rolling_cap_not_in_parked_mode(self):
    # 64 alone (parked): the cap stays 0.10 g at standstill
    self._rearm(HyundaiSafetyFlagsSP.NON_SCC | HyundaiSafetyFlagsSP.FCA11_BRAKE_TEST | self.SAFETY_PARAM_SP)
    self._rx(self._wheel_msg(0))
    self._rx(self._speed_msg(0))
    self.assertTrue(self._tx(self._brake_cmd(HYUNDAI_FCA11_TEST_MAX_DEC)))
    for dec in (HYUNDAI_FCA11_TEST_MAX_DEC + 1, 20, 30):
      self.assertFalse(self._tx(self._brake_cmd(dec)), dec)
      self.assertFalse(self._tx(self._fca11(dec=dec)), dec)

  def test_rolling_cap_not_with_bits_unset(self):
    # no test bits (normal driving) and 128 without 64: no decel of any size is ever transmittable
    for sp in (HyundaiSafetyFlagsSP.NON_SCC | self.SAFETY_PARAM_SP,
               HyundaiSafetyFlagsSP.NON_SCC | HyundaiSafetyFlagsSP.FCA11_ROLLING_TEST | self.SAFETY_PARAM_SP):
      for dec in (1, HYUNDAI_FCA11_TEST_MAX_DEC, 30):
        with self.subTest(sp=sp, dec=dec):
          self._rearm(sp)
          self._open_window()
          self.safety.set_controls_allowed(True)
          self.assertFalse(self._tx(self._brake_cmd(dec)))
          self.assertFalse(self._tx(self._fca11(dec=dec)))

  # *** CF_VSM_Warn (dash FCW) is policed like actuation in the rolling mode ***

  def test_warn_helper_bits(self):
    # guards the helper: warn lands in byte0 bits 3-4 (what the panda reads), nothing else set
    for warn in (0, 1, 2, 3):
      msg = self._fca11(warn=warn)
      self.assertEqual(warn << 3, msg[0].data[0])
      self.assertEqual(0, msg[0].data[1])

  def test_warn_only_allowed_in_window(self):
    for warn in (1, 2, 3):
      with self.subTest(warn=warn):
        self._rearm()
        self._open_window()
        self.assertTrue(self._tx(self._fca11(warn=warn)))

  def test_warn_only_blocked_outside_window(self):
    lo = HYUNDAI_FCA11_ROLL_MIN_SPEED_KPH * self.RAW_PER_KPH
    hi = HYUNDAI_FCA11_ROLL_MAX_SPEED_KPH * self.RAW_PER_KPH
    for warn in (1, 2, 3):
      for raw in (0, lo, hi + 1):
        with self.subTest(warn=warn, raw=raw):
          self._rearm()
          self._open_window(raw)
          self.assertFalse(self._tx(self._fca11(warn=warn)))
          self.assertTrue(self._tx(self._fca11()))   # passive carrier still flows
      for c in ("brake", "gas", "gear"):
        with self.subTest(warn=warn, cut=c):
          self._rearm()
          self._open_window()
          self.assertTrue(self._tx(self._fca11(warn=warn)))
          {"brake": lambda: self._rx(self._user_brake_msg(True)), "gas": lambda: self._rx(self._user_gas_msg(1)),
           "gear": lambda: self._rx(self._gear_msg(4))}[c]()
          self.assertFalse(self._tx(self._fca11(warn=warn)))
          self._open_window()   # restored, but the cut is latched until re-arm
          self.assertFalse(self._tx(self._fca11(warn=warn)))

  def test_warn_only_needs_fresh_inputs(self):
    self._rearm()
    self._open_window()
    self.assertTrue(self._tx(self._fca11(warn=1)))
    self.safety.set_timer(1000000 + 100001)
    self.assertFalse(self._tx(self._fca11(warn=1)))
    self._open_window()
    self.assertTrue(self._tx(self._fca11(warn=1)))
    self._rearm()
    self.safety.set_timer(50000)   # nothing seen since this arm: blocked even though timestamps look fresh
    self.assertFalse(self._tx(self._fca11(warn=1)))

  def test_warn_only_blocked_after_camera_request(self):
    self._rearm()
    self._open_window()
    self.assertTrue(self._tx(self._fca11(warn=2)))
    self._rx(self._cam_fca11(CF_VSM_Warn=1))
    self.assertFalse(self._tx(self._fca11(warn=2)))

  def test_warn_only_runs_clock(self):
    # Warn-only alone is capped at 1.2 s
    self._rearm()
    self._open_window()
    self.assertTrue(self._tx(self._fca11(warn=1)))
    self.safety.set_timer(1000000 + 1200000)
    self._open_window()
    self.assertTrue(self._tx(self._fca11(warn=1)))
    self.safety.set_timer(1000000 + 1200001)
    self._open_window()
    self.assertFalse(self._tx(self._fca11(warn=1)))
    self.assertTrue(self._tx(self._fca11()))        # passive ends the episode ...
    self.assertTrue(self._tx(self._fca11(warn=1)))  # ... a new one may start

  def test_warn_leadin_shares_clock_with_actuation(self):
    # a 0.5 s Warn=1 lead-in then Warn=2 + DecCmdAct: ONE episode, 1.2 s total from the first Warn frame
    self._rearm()
    self._open_window()
    self.assertTrue(self._tx(self._fca11(warn=1)))
    self.safety.set_timer(1000000 + 500000)
    self._open_window()
    self.assertTrue(self._tx(self._fca11(dec=10, prefill=1, dec_cmd_act=1, warn=2)))
    self.safety.set_timer(1000000 + 1200000)
    self._open_window()
    self.assertTrue(self._tx(self._fca11(dec=10, prefill=1, dec_cmd_act=1, warn=2)))
    self.safety.set_timer(1000000 + 1200001)
    self._open_window()
    self.assertFalse(self._tx(self._fca11(dec=10, prefill=1, dec_cmd_act=1, warn=2)))
    self.assertFalse(self._tx(self._fca11(warn=1)))   # dropping back to Warn-only does not reset the clock

  def test_variant_frames_allowed_in_window(self):
    # the two next-test variants: A = Warn 2 + DecCmdAct, B = Warn 3 + FCA_CmdAct (DecCmd 10, Prefill 1)
    for kw in ({"warn": 2, "dec_cmd_act": 1}, {"warn": 3, "cmd_act": 1}):
      with self.subTest(kw=kw):
        self._rearm()
        self._open_window()
        self.assertTrue(self._tx(self._fca11(dec=HYUNDAI_FCA11_TEST_MAX_DEC, prefill=1, **kw)))
        self._rearm()
        self._open_window(0)
        self.assertFalse(self._tx(self._fca11(dec=HYUNDAI_FCA11_TEST_MAX_DEC, prefill=1, **kw)))

  # *** gating ***

  def test_rolling_bit_alone_ignored(self):
    # 128 without 64: normal non-SCC, nothing changes
    self.safety.set_current_safety_param_sp(HyundaiSafetyFlagsSP.NON_SCC | HyundaiSafetyFlagsSP.FCA11_ROLLING_TEST | self.SAFETY_PARAM_SP)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, 0)
    self._open_window()
    self._assert_not_armed()

  def test_parked_mode_unchanged_by_rolling_code(self):
    # 64 without 128: the parked mode keeps its standstill-only rule
    self.safety.set_current_safety_param_sp(HyundaiSafetyFlagsSP.NON_SCC | HyundaiSafetyFlagsSP.FCA11_BRAKE_TEST | self.SAFETY_PARAM_SP)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, 0)
    self._open_window()
    self.assertFalse(self._tx(self._brake_cmd(1)))
    # 0012 Warn gating is rolling-only: the parked mode's Warn handling is byte-for-byte what it was (not inspected)
    for warn in (1, 2, 3):
      self.assertTrue(self._tx(self._fca11(warn=warn)))
    self._rx(self._wheel_msg(0))
    self.assertTrue(self._tx(self._brake_cmd(1)))

  def test_gear_rx_check_only_in_rolling(self):
    # LVR12 is an rx check only in the rolling config: a missing gear stream makes the config invalid there
    self._rearm()
    for _ in range(3):
      self._open_window()
    self.safety.set_timer(1000000 + int(2.5e6))
    for _ in range(3):
      self._rx(self._user_brake_msg(False))
      self._rx(self._user_gas_msg(0))
      self._rx(self._wheel_msg(self.IN_WINDOW_RAW))
    self.assertFalse(self._tx(self._brake_cmd(1)))

  def test_test_bit_overrides_gas_interceptor(self):
    for dialect in (0, HyundaiSafetyFlagsSP.GAS_INTERCEPTOR_REMAPPED):
      self._rearm(self._test_sp() | HyundaiSafetyFlagsSP.GAS_INTERCEPTOR | dialect)
      self._open_window()
      self.safety.set_controls_allowed(True)
      self.assertTrue(self._tx(self._brake_cmd(1)))
      self.assertFalse(self._tx(common.make_msg(0, 0x200, 6)))
      self.assertFalse(self._tx(common.make_msg(0, 0x700, 6)))
      self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x38D))

  def test_test_bit_overrides_pedal_buttons(self):
    # identical to the parked class, from a fresh arm; the brake path at the end needs the rolling window open
    for dialect in (0, HyundaiSafetyFlagsSP.GAS_INTERCEPTOR_REMAPPED):
      with self.subTest(dialect=dialect):
        self._rearm(self._test_sp() | HyundaiSafetyFlagsSP.GAS_INTERCEPTOR | dialect)
        self._rx(self._acc_state_msg(False))
        self.assertFalse(self.safety.get_acc_main_on())
        self._rx(self._acc_state_msg(True))
        self.assertTrue(self.safety.get_acc_main_on())
        self._rx(self._pcm_status_msg(True))
        self.safety.set_controls_allowed(False)
        self.assertTrue(self._tx(self._button_msg(Buttons.CANCEL)))
        for btn in (Buttons.RESUME, Buttons.SET):
          self.assertFalse(self._tx(self._button_msg(btn)), btn)
        self._rx(self._pcm_status_msg(False))
        self.safety.set_controls_allowed(False)
        for btn in [Buttons.NONE] * 5 + [Buttons.CANCEL] * 5 + [Buttons.NONE] * 5:
          self._rx(self._button_msg(btn))
        self.assertFalse(self.safety.get_controls_allowed())
        self._open_window()
        self.assertTrue(self._tx(self._brake_cmd(1)))

  def test_rolling_bits_override_pedal_buttons_v2(self):
    # integration (pedal-buttons-v2 + rolling): bits 64+128 with GAS_INTERCEPTOR (both dialects). None of the v2 pedal
    # paths leak into the rolling test: the v2 pause/resume grant (gas held / standstill / stale grant) stays inert, no
    # pedal command of any size is transmittable, gas comes from EMS16 (not the interceptor) and still latches the
    # rolling cut, and the dynamic camera hand-back is intact.
    for dialect in (0, HyundaiSafetyFlagsSP.GAS_INTERCEPTOR_REMAPPED):
      with self.subTest(dialect=dialect):
        sp = self._test_sp() | HyundaiSafetyFlagsSP.GAS_INTERCEPTOR | dialect
        # v2 pause/resume with the gas HELD (v2 allows it in pedal mode): no grant, in the window and at standstill
        for speed_raw in (self.IN_WINDOW_RAW, 0):
          self._rearm(sp)
          self._open_window(speed_raw)
          self.safety.set_controls_allowed(False)
          self._rx(self._user_gas_msg(1))
          for btn in [Buttons.NONE] * 5 + [Buttons.CANCEL] * 5 + [Buttons.NONE] * 5:
            self._rx(self._button_msg(btn))
          self.assertFalse(self.safety.get_controls_allowed(), speed_raw)
          self._rx(self._user_gas_msg(0))
          for btn in [Buttons.NONE] * 5 + [Buttons.CANCEL] * 5 + [Buttons.NONE] * 5:
            self._rx(self._button_msg(btn))
          self.assertFalse(self.safety.get_controls_allowed(), speed_raw)
        # v2 dropped the controls_allowed condition: a stale grant + press must not re-grant either
        self._rearm(sp)
        self._open_window()
        self.safety.set_controls_allowed(True)
        for btn in [Buttons.NONE] * 5 + [Buttons.CANCEL] * 5 + [Buttons.NONE] * 5:
          self._rx(self._button_msg(btn))
        self.safety.set_controls_allowed(False)
        for btn in [Buttons.NONE] * 5 + [Buttons.CANCEL] * 5 + [Buttons.NONE] * 5:
          self._rx(self._button_msg(btn))
        self.assertFalse(self.safety.get_controls_allowed())
        # no pedal command, zero or within the pedal ceiling, of either dialect
        self._rearm(sp)
        self._open_window()
        self.safety.set_controls_allowed(True)
        for addr in (0x200, 0x700):
          self.assertFalse(self._tx(common.make_msg(0, addr, 6)), hex(addr))
          self.assertFalse(self._tx(self._interceptor_gas_cmd_raw(addr, 100)), hex(addr))
        # dynamic camera hand-back still applies: forwarded until we transmit, blocked while we do
        self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x38D))
        self.assertTrue(self._tx(self._brake_cmd(1)))
        self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x38D))
        # EMS16 is the gas source in test mode and still latches the rolling cut
        self._rx(self._user_gas_msg(1))
        self._rx(self._user_gas_msg(0))
        self._open_window()
        self.assertFalse(self._tx(self._brake_cmd(1)))
        self.assertTrue(self._tx(self._fca11()))  # the zero carrier is never cut

  def _interceptor_gas_cmd_raw(self, addr, raw):
    dat = bytearray(6)
    dat[0], dat[1] = (raw >> 8) & 0xFF, raw & 0xFF
    dat[2], dat[3] = (raw >> 9) & 0xFF, (raw >> 1) & 0xFF
    dat[4] = 0x80
    return libsafety_py.make_CANPacket(addr, 0, bytes(dat))

@parameterized_class(LDA_BUTTON)
class TestHyundaiFca11LongSafety(TestHyundaiNonSCCGasInterceptorSafety):
  """
    PRODUCTION FCA11 long braking (bit 256, HyundaiFca11Brake). Rides ON TOP of the comma pedal with openpilot
    ENGAGED (controls_allowed AND heartbeat_engaged): TX = the pedal list + FCA11 (dynamic camera blocking), the
    pedal path stays fully active. Actuating FCA11 is legal only inside the window (gear D, no pedal, every wheel
    above 9 km/h raw 288, fresh inputs, no cut), capped at 0.30 g, rate-limited (+4 LSB per accepted frame, the
    reference restarting from 0 after a passive frame or a stale stream), with NO duration budget and NO cooldown
    (0035), camera request hands back the same frame. A DRIVER-INPUT cut (brake / gas / gear != D) re-arms on the next
    openpilot RESUME edge (controls_allowed rising, mirroring the car layer's blocked_until_resume) but CLEARS only
    on the first CLEAN frame after that grant (G9b): an input merely HELD at the grant (the drive-14 engage-with-foot-
    on-gas) keeps the latch for that frame and stops barring actuation the moment it is actually released. A pedal
    fault and a camera request LATCH until the next init.
  """
  # pedal list + FCA11 (check_relay, dynamic blocking via hyundai_fwd_hook). NO CLU11: the pedal TX list has none.
  TX_MSGS = [[0x340, 0], [0x485, 0], [0x200, 0], [0x38D, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x340, 0x485, 0x38D)}
  # openpilot steers in this production mode (LKAS11/LFAHDA in the TX list), so the camera's copies stay blocked.
  # 0x38D is NOT listed: its blocking is dynamic (hyundai_fwd_hook) and tested explicitly below.
  FWD_BLACKLISTED_ADDRS = {2: [0x340, 0x485]}
  DEC_CAP = HYUNDAI_FCA11_LONG_MAX_DEC
  RAW_PER_KPH = 32
  IN_WINDOW_RAW = 20 * 32  # 20 km/h
  cnt_lvr = 0

  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiFca11LongSafety":
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    super().setUp()

  def _pedal_sp(self) -> int:
    return super()._pedal_sp() | HyundaiSafetyFlagsSP.FCA11_LONG

  def _engage(self, controls=True, heartbeat=True):
    self.safety.set_controls_allowed(controls)
    self.safety.set_heartbeat_engaged(heartbeat)

  def _gear_msg(self, gear):
    return self.packer.make_can_msg_safety("LVR12", 0, {"CF_Lvr_Gear": gear})

  def _wheel_msg(self, fl, fr=None, rl=None, rr=None):
    vals = {"FL": fl, "FR": fl if fr is None else fr, "RL": fl if rl is None else rl, "RR": fl if rr is None else rr}
    values = {"WHL_SPD_%s" % k: v * 0.03125 for k, v in vals.items()}
    values["WHL_SPD_AliveCounter_LSB"] = (self.cnt_speed % 16) & 0x3
    values["WHL_SPD_AliveCounter_MSB"] = (self.cnt_speed % 16) >> 2
    self.__class__.cnt_speed += 1
    return self.packer.make_can_msg_safety("WHL_SPD11", 0, values, fix_checksum=checksum)

  def _open_window(self, speed_raw=None):
    # a complete window implies the production gate: openpilot engaged (controls_allowed + heartbeat)
    self._engage(True, True)
    self.assertTrue(self._rx(self._user_brake_msg(False)))
    # gas freshness in the C window is keyed on EMS16 (0x260); the pedal sensor (0x201) sets gas_pressed, not ts_gas
    self.assertTrue(self._rx(self._user_gas_msg(0)))
    self.assertTrue(self._rx(self._interceptor_user_gas(0)))
    self.assertTrue(self._rx(self._gear_msg(5)))
    self.assertTrue(self._rx(self._wheel_msg(self.IN_WINDOW_RAW if speed_raw is None else speed_raw)))

  def _rearm(self, sp=None):
    self.safety.set_current_safety_param_sp(self._pedal_sp() if sp is None else sp)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, 0)
    self.safety.set_timer(1000000)
    self._engage(True, True)

  def _fca11(self, dec=0, prefill=0, hba=0, cmd_act=0, dec_cmd_act=0, stop_req=0, warn=0, bus=0):
    values = {"CR_VSM_DecCmd": (dec + 0.25) * 0.01 if dec else 0, "CF_VSM_Prefill": prefill, "CF_VSM_HBACmd": hba,
              "FCA_CmdAct": cmd_act, "CF_VSM_DecCmdAct": dec_cmd_act, "FCA_StopReq": stop_req, "FCA_Status": 2,
              "CF_VSM_Warn": warn}
    return self.packer.make_can_msg_safety("FCA11", bus, values)

  def _brake_cmd(self, dec):
    # variant B: Prefill + Warn 3 + CmdAct, DecCmdAct 0
    return self._fca11(dec=dec, prefill=1, warn=3, cmd_act=1, dec_cmd_act=0)

  def _cam_fca11(self, **kw):
    values = {"FCA_Status": 2}
    values.update(kw)
    return self.packer.make_can_msg_safety("FCA11", 2, values)

  def _rx_all_but_pedal(self):
    # FCA11_LONG adds the gear (LVR12) and camera (0x38D bus 2) RX checks; feed them so the pedal stays the only
    # missing required stream the inherited pedal-health tests probe.
    super()._rx_all_but_pedal()
    self._rx(self._gear_msg(5))
    self._rx(self._cam_fca11())

  def _tx_brake(self, target, steps=12):
    """Send a decel ramp accepted by the rate limiter, ending exactly on `target`; returns the last verdict."""
    dec = 0
    last = True
    for _ in range(steps):
      dec = min(target, dec + HYUNDAI_FCA11_LONG_RATE_STEP)
      last = self._tx(self._brake_cmd(dec))
      if dec >= target:
        break
    return last

  # *** window: controls_allowed AND heartbeat_engaged ***

  def test_no_scc_actuation_allowed(self):
    # 0030: FCA11_LONG deliberately allows the 0x38D mirror (that IS the feature), so the inherited pedal-long
    # sweep is overridden to drop 0x38D and keep every OTHER SCC-replacement address blocked.
    self.safety.set_controls_allowed(True)
    for addr in (0x420, 0x421, 0x50A, 0x389, 0x4A2, 0x483, 0x7D0):
      self.assertFalse(self._tx(common.make_msg(0, addr, 8)), f"{addr=:#x}")

  def test_needs_controls_and_heartbeat(self):
    for controls, heartbeat in ((False, False), (True, False), (False, True)):
      with self.subTest(controls=controls, heartbeat=heartbeat):
        self._rearm()
        self._open_window()
        self._engage(controls, heartbeat)
        self.assertFalse(self._tx(self._brake_cmd(1)))
        # 0030 (D2-i): a PASSIVE frame is NOT engagement-gated any more. It is the camera's own idle shape (all
        # brake fields 0, Warn 0) and accepting it can only CLOSE the panda episode, never actuate - so it is
        # accepted even while disengaged / with a stale heartbeat. FALSIFIABLE CONTRAST: on the 29-patch tree this
        # asserted False (the close frame was refused at an engage/disengage edge -> the episode never closed).
        self.assertTrue(self._tx(self._fca11()))
    self._rearm()
    self._open_window()
    self._engage(True, True)
    self.assertTrue(self._tx(self._brake_cmd(1)))
    self.assertTrue(self._tx(self._fca11()))

  def test_passive_close_accepted_while_disengaged_or_heartbeat_stale(self):
    """0030 (D2-i) regression, the 860-frame stale act_active defect (drives 14f): the passive close frame the car
    layer sends on an engage/disengage edge must be ACCEPTED even with !controls_allowed / !heartbeat_engaged, so
    the panda episode CLOSES and the NEXT brake starts fresh (0035: from the onset step, with no cooldown). ACTUATING
    frames stay gated exactly as before. FALSIFIABLE CONTRAST: every assertTrue on a passive frame below fails on
    the deployed 29-patch tree (the passive frame was refused, so the episode never closed)."""
    # (a) passive close at a DISENGAGE edge (!controls_allowed): must be accepted -> episode ends
    self._rearm()
    self._open_window()
    self.assertTrue(self._tx(self._brake_cmd(1)))               # episode open
    self._engage(False, False)                                 # disengaged: !controls_allowed
    self.assertTrue(self._tx(self._fca11()), "passive close must be accepted on a disengage edge")
    # the accepted passive frame ENDED the episode; the next brake is a NEW episode, accepted at once (0035: no cooldown)
    self._open_window()
    self.assertTrue(self._tx(self._brake_cmd(1)))
    # (b) passive close at an ENGAGE edge with the heartbeat still stale (!heartbeat_engaged): accepted
    self._rearm()
    self._open_window()
    self.assertTrue(self._tx(self._brake_cmd(1)))
    self._engage(True, False)                                  # engaged but heartbeat not yet (pandad 10 Hz step)
    self.assertTrue(self._tx(self._fca11()), "passive close must be accepted with a stale heartbeat")
    # an ACTUATING frame is still refused while the heartbeat is stale (actuation gate unchanged)
    self.assertFalse(self._tx(self._brake_cmd(1)))
    self._engage(True, True)
    self._open_window()
    self.assertTrue(self._tx(self._brake_cmd(1)))             # 0035: fresh episode at once, no cooldown

  def test_floor_and_no_ceiling(self):
    # 0040: the speed floor is DELETED. Actuation is ACCEPTED at the old 9 km/h boundary, below it, at a crawl and at
    # a full stop; there is still NO ceiling (200 km/h accepted). FALSIFIABLE CONTRAST: on the 39-patch tree `lo` and
    # `lo - 1` are REFUSED (that was the floor); here everything is accepted.
    lo = 9 * self.RAW_PER_KPH
    for raw, ok in ((lo, True), (lo - 1, True), (1, True), (0, True), (raw_hi := 200 * self.RAW_PER_KPH, True)):
      with self.subTest(raw=raw):
        self._rearm()
        self._open_window(raw)
        self.assertEqual(ok, self._tx(self._brake_cmd(1)))

  def test_each_wheel_policed(self):
    # 0040: there is no per-wheel speed floor any more. Each wheel independently may read ANY speed (including 0) and
    # actuation is still accepted; freshness of the WHL_SPD11 stream is the only speed-related requirement (tested
    # separately by test_stale_input_blocks). FALSIFIABLE CONTRAST: on the 39-patch tree every case below refuses.
    lo = 9 * self.RAW_PER_KPH
    for i in range(4):
      for raw in (0, 1, lo, lo - 1):
        with self.subTest(wheel=i, raw=raw):
          self._rearm()
          self._open_window()
          w = [self.IN_WINDOW_RAW] * 4
          w[i] = raw
          self._rx(self._wheel_msg(*w))
          self.assertTrue(self._tx(self._brake_cmd(1)), f"wheel {i} raw {raw}")

  # *** 0040: the speed FLOOR is removed; a full stop + hold is legal, and the ramp/cap are the only bounds ***

  def test_floor_removed_now_sends_through_zero(self):
    """0040 REQUIREMENT: an actuating frame at v_slowest == 0 is ACCEPTED (it was REFUSED before), and the whole
    sub-9-km/h band is accepted. FALSIFIABLE CONTRAST: on the 39-patch tree every case here is refused (the floor);
    the clean tree + this test file FAILS. The ramp and the 0.30 g cap still bound the frame; the HOST_HB watchdog,
    the camera-owns / pedal-fault / brake / gear latches are all re-checked at v == 0 too."""
    # raw 0 (a full stop) is now accepted, at every level up the ramp
    self._rearm()
    self._open_window(0)
    self.assertTrue(self._tx(self._brake_cmd(1)), "raw 0: first ramp step accepted")
    self.assertTrue(self._tx(self._brake_cmd(1 + HYUNDAI_FCA11_LONG_RATE_STEP)), "raw 0: ramp step accepted")
    # the cap still bounds it even at a standstill: 31 refused, exactly RATE_STEP over the last accepted is fine
    self.assertFalse(self._tx(self._brake_cmd(HYUNDAI_FCA11_LONG_MAX_DEC + 1)), "the 0.30 g cap still bounds at v=0")
    # a fresh arm holding at raw 0, held long: still accepted (no self-imposed stop termination / hold cap)
    self._rearm()
    self._open_window(0)
    t, last, refusals = self._stream(1000000, 5000)
    self.assertEqual((HYUNDAI_FCA11_LONG_MAX_DEC, []), (last, refusals), "a 5 s standstill hold is unimpeded")
    # the whole sub-floor band (slowest wheel 0 .. 9 km/h) is accepted
    for raw in (0, 1, 50, 9 * self.RAW_PER_KPH - 1, 9 * self.RAW_PER_KPH):
      with self.subTest(raw=raw):
        self._rearm()
        self._open_window(raw)
        self.assertTrue(self._tx(self._brake_cmd(1)), f"raw {raw} must be accepted (no floor)")

  def test_floor_removed_still_refuses_on_watchdog_and_latches_at_zero(self):
    """0040: removing the floor must NOT weaken the OTHER gates. At a full stop (raw 0) a stale host is still refused
    by the HOST_HB watchdog, and the camera-owns / pedal-fault / gear != D / driver-brake latches still refuse."""
    # stale host at raw 0: the liveness watchdog still refuses (the floor's replacement bound). Seed the host clock
    # first (a benign host frame) so the watchdog measures a real gap.
    self._rearm()
    self._open_window(0)
    self.assertTrue(self._tx(self._interceptor_gas_cmd(0)), "benign host frame seeds the liveness clock")
    self.safety.set_timer(1000000 + HYUNDAI_FCA11_LONG_HOST_HB_TIMEOUT_US + 100000)
    self._feed_inputs(speed_raw=0)
    self.assertFalse(self._tx(self._brake_cmd(1)), "raw 0 + stale host: still refused")
    # driver brake at raw 0
    self._rearm()
    self._open_window(0)
    self._rx(self._user_brake_msg(True))
    self.assertFalse(self._tx(self._brake_cmd(1)), "raw 0 + driver brake: still refused")
    # gear != D at raw 0
    self._rearm()
    self._open_window(0)
    self._rx(self._gear_msg(4))
    self.assertFalse(self._tx(self._brake_cmd(1)), "raw 0 + gear != D: still refused")
    # pedal fault at raw 0
    self._rearm()
    self._open_window(0)
    self._rx(self._interceptor_user_gas(0, state=1))
    self.assertFalse(self._tx(self._brake_cmd(1)), "raw 0 + pedal fault: still refused")
    # camera owns at raw 0
    self._rearm()
    self._open_window(0)
    self._rx(self._cam_fca11(CF_VSM_Warn=2))
    self.assertFalse(self._tx(self._brake_cmd(1)), "raw 0 + camera request: still refused")

  def test_floor_constant_absent_from_the_header(self):
    """0040: the floor MACRO is DELETED from safety/modes/hyundai.h (not merely unused). FALSIFIABLE CONTRAST: it is
    present on the 39-patch tree, so this test fails there."""
    h = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "modes", "hyundai.h")
    with open(h) as f:
      lines = f.readlines()
    self.assertFalse(any(ln.strip().startswith("#define HYUNDAI_FCA11_LONG_MIN_SPEED") for ln in lines),
                     "the floor macro must be gone from the header")
    # ...and it is used in no CODE line (a comment referencing the deleted macro is fine and deliberate)
    code = [ln.split("//", 1)[0] for ln in lines]
    self.assertFalse(any("HYUNDAI_FCA11_LONG_MIN_SPEED" in ln for ln in code),
                     "no live use of the deleted floor identifier")

  def test_needs_every_input_seen(self):
    feeds = {"brake": lambda: self._rx(self._user_brake_msg(False)), "gas": lambda: self._rx(self._user_gas_msg(0)),
             "gear": lambda: self._rx(self._gear_msg(5)), "speed": lambda: self._rx(self._wheel_msg(self.IN_WINDOW_RAW))}
    for missing in feeds:
      with self.subTest(missing=missing):
        self._rearm()
        self.safety.set_timer(50000)
        for name, f in feeds.items():
          if name != missing:
            f()
        self.assertFalse(self._tx(self._brake_cmd(1)))
        self.assertTrue(self._tx(self._fca11()))

  def test_stale_input_blocks(self):
    feeds = {"brake": lambda: self._rx(self._user_brake_msg(False)), "gas": lambda: self._rx(self._user_gas_msg(0)),
             "gear": lambda: self._rx(self._gear_msg(5)), "speed": lambda: self._rx(self._wheel_msg(self.IN_WINDOW_RAW))}
    for stale in feeds:
      with self.subTest(stale=stale):
        self._rearm()
        self._open_window()
        self.assertTrue(self._tx(self._brake_cmd(1)))
        self.safety.set_timer(1000000 + 100001)  # just past 100 ms
        for name, f in feeds.items():
          if name != stale:
            f()
        self.assertFalse(self._tx(self._brake_cmd(1)))
        feeds[stale]()
        self.assertTrue(self._tx(self._brake_cmd(1)))
    self._rearm()
    self._open_window()
    self.safety.set_timer(1000000 + 100000)  # exactly 100 ms is fresh
    self.assertTrue(self._tx(self._brake_cmd(1)))

  def test_gear_gate(self):
    for gear in (0, 4, 6, 7, 8, 12):
      with self.subTest(gear=gear):
        self._rearm()
        self._open_window()
        self._rx(self._gear_msg(gear))
        self.assertFalse(self._tx(self._brake_cmd(1)))

  # *** cap / shapes ***

  def test_c_python_cap_match(self):
    self._open_window()
    self.assertTrue(self._tx_brake(self.DEC_CAP))
    self.assertFalse(self._tx(self._brake_cmd(self.DEC_CAP + 1)))

  def test_cap_python_mirror(self):
    # the C constant must equal the Python one (values.py is the C mirror's source of truth)
    self.assertEqual(self.DEC_CAP, HYUNDAI_FCA11_LONG_MAX_DEC)

  def test_hba_stopreq_dec_cmd_act_blocked(self):
    self._open_window()
    for hba in (1, 2, 3):
      self.assertFalse(self._tx(self._fca11(dec=1, prefill=1, warn=3, hba=hba, cmd_act=1)))
    self.assertFalse(self._tx(self._fca11(dec=1, prefill=1, warn=3, cmd_act=1, stop_req=1)))
    self.assertFalse(self._tx(self._fca11(dec=1, prefill=1, warn=3, cmd_act=1, dec_cmd_act=1)))  # variant B: DecCmdAct stays 0

  def test_warn_shape_policed(self):
    self._open_window()
    # actuating frames must carry Warn 3; passive frames must carry Warn 0
    self.assertFalse(self._tx(self._fca11(dec=1, prefill=1, warn=0, cmd_act=1)))
    self.assertFalse(self._tx(self._fca11(dec=1, prefill=1, warn=1, cmd_act=1)))
    self.assertTrue(self._tx(self._fca11(prefill=1, warn=3)))          # prefill-only: an actuation bit, Warn 3
    self.assertFalse(self._tx(self._fca11(warn=3)))                    # Warn 3 with no actuation at all: rejected

  # *** rate limit ***

  def test_rate_ramp(self):
    self._open_window()
    step = HYUNDAI_FCA11_LONG_RATE_STEP
    # first frame of an episode is capped at RATE_STEP from zero
    self.assertFalse(self._tx(self._brake_cmd(step + 1)))
    self.assertTrue(self._tx(self._brake_cmd(step)))
    # growth above last accepted + step is rejected, and does not move the reference
    self.assertFalse(self._tx(self._brake_cmd(step * 2 + 1)))
    self.assertTrue(self._tx(self._brake_cmd(step * 2)))
    # release is immediate and resets the reference down
    self.assertTrue(self._tx(self._brake_cmd(1)))
    self.assertTrue(self._tx(self._brake_cmd(1 + step)))
    self.assertFalse(self._tx(self._brake_cmd(1 + step * 2 + 1)))

  # *** 0035: NO duration budget, NO cooldown; the onset rule bounds every (re)open ***
  # All durations below are WALL-CLOCK MILLISECONDS on the panda's microsecond timer, never frame counts: the car
  # layer sends every 20 ms (the camera period) and the panda polices per accepted frame, so a frame-count test with
  # the wrong period assumption could hide a clip. PERIOD_MS is the real host send period.
  PERIOD_MS = 20

  def _feed_inputs(self, speed_raw=None):
    """One fresh sample of every window input (brake/gas/pedal/gear/wheels) WITHOUT touching controls_allowed."""
    self._rx(self._user_brake_msg(False))
    self._rx(self._user_gas_msg(0))
    self._rx(self._interceptor_user_gas(0))
    self._rx(self._gear_msg(5))
    self._rx(self._wheel_msg(self.IN_WINDOW_RAW if speed_raw is None else speed_raw))

  def _stream(self, t0_us, dur_ms, target=HYUNDAI_FCA11_LONG_MAX_DEC, start_dec=0, feed=True, hook=None):
    """A legal-slew actuating stream: every PERIOD_MS the inputs are refreshed and one variant-B frame goes out,
    ramping +RATE_STEP from `start_dec` to `target` and then holding it. Returns (t_end_us, last_dec, refusals) where
    refusals is the list of (t_ms_since_t0, dec) of every refused frame. `hook(t_ms)` runs before each frame (fault
    injection)."""
    dec = start_dec
    refusals = []
    t = t0_us
    for i in range(dur_ms // self.PERIOD_MS + 1):
      t = t0_us + i * self.PERIOD_MS * 1000
      self.safety.set_timer(t)
      if feed:
        self._feed_inputs()
      if hook is not None:
        hook(i * self.PERIOD_MS)
      dec = min(target, dec + HYUNDAI_FCA11_LONG_RATE_STEP)
      if not self._tx(self._brake_cmd(dec)):
        refusals.append((i * self.PERIOD_MS, dec))
    return t, dec, refusals

  def _host_alive(self, t0_us, gap_ms):
    """Advance wall-clock `gap_ms` while the HOST keeps streaming benign zero-throttle frames, exactly as the real
    host does between brake episodes. Returns the new timer value. 0038: without fresh host frames the liveness
    watchdog would (correctly) refuse the next actuating frame, so a gap test that intends to measure the RATE law
    must keep the host alive."""
    t = t0_us
    for i in range(max(1, gap_ms // self.PERIOD_MS)):
      t = t0_us + i * self.PERIOD_MS * 1000
      self.safety.set_timer(t)
      self._tx(self._interceptor_gas_cmd(0))
    return t0_us + gap_ms * 1000

  def test_f1_no_truncation_30s_continuous(self):
    """F1: gates held, a legal-slew actuating stream for 30 000 ms wall clock -> ZERO refusals. RED on the 0034 tree:
    the 2.5 s budget refuses the frame after 2500 ms and the cooldown refuses everything for the next 3000 ms."""
    self._rearm()
    self._open_window()
    _, last, refusals = self._stream(1000000, 30000)
    self.assertEqual(last, HYUNDAI_FCA11_LONG_MAX_DEC)
    self.assertEqual([], refusals, f"{len(refusals)} refusals, first at {refusals[:1]} ms")

  def test_f1_no_truncation_at_every_level(self):
    """F1 (each level): a 10 000 ms hold at a constant mid level is never cut (a cut would show at ~2500 ms)."""
    for level in (4, 12, 20):
      with self.subTest(level=level):
        self._rearm()
        self._open_window()
        _, _, refusals = self._stream(1000000, 10000, target=level)
        self.assertEqual([], refusals)

  def test_f2_reactuation_right_after_close_is_accepted(self):
    """F2: close an episode, then re-actuate on the very next camera period -> accepted (no cooldown). RED on 0034."""
    self._rearm()
    self._open_window()
    t, _, refusals = self._stream(1000000, 1000)
    self.assertEqual([], refusals)
    t += self.PERIOD_MS * 1000
    self.safety.set_timer(t)
    self.assertTrue(self._tx(self._fca11()))                       # passive close
    t += self.PERIOD_MS * 1000
    self.safety.set_timer(t)
    self._feed_inputs()
    self.assertTrue(self._tx(self._brake_cmd(HYUNDAI_FCA11_LONG_RATE_STEP)), "re-actuation 20 ms after close")

  def test_f2_back_to_back_episodes_both_fully_actuate(self):
    """F2: back-to-back episodes separated by gaps well under the old 3 s cooldown, each held LONGER than the old
    2.5 s budget, all fully actuate (zero refusals across the whole profile). RED on 0034. 0038: the gaps are spent
    with the HOST still streaming (the real host does), so the liveness watchdog is not the thing under test here."""
    self._rearm()
    self._open_window()
    t = 1000000
    for gap_ms in (20, 500, 1500, 2980):
      t, _, refusals = self._stream(t, 4000)
      self.assertEqual([], refusals, f"episode before gap {gap_ms} ms refused at {refusals[:1]}")
      t += self.PERIOD_MS * 1000
      self.safety.set_timer(t)
      self.assertTrue(self._tx(self._fca11()))                     # passive close
      t = self._host_alive(t, gap_ms)
    _, _, refusals = self._stream(t, 4000)
    self.assertEqual([], refusals)

  def test_f3_churn_reopen_after_passive_restarts_the_onset(self):
    """F3 (the companion rule): actuate at 30 -> accepted passive -> a reopen requesting 30 is REFUSED, and so is
    anything above RATE_STEP; a reopen at RATE_STEP (4) is accepted, then the legal +4/20 ms slew climbs back to 30.
    The onset cap is RATE_STEP = 4 LSB, i.e. the panda's EXISTING reference reset (an accepted passive frame stores
    dec_last = 0), which is <= the ruling's 8 LSB first-frame cap. RED on 0034: the reopen at 4 is refused by the
    cooldown."""
    self._rearm()
    self._open_window()
    t, last, refusals = self._stream(1000000, 500)
    self.assertEqual((30, []), (last, refusals))
    t += self.PERIOD_MS * 1000
    self.safety.set_timer(t)
    self.assertTrue(self._tx(self._fca11()))                       # accepted passive
    t += self.PERIOD_MS * 1000
    self.safety.set_timer(t)
    self._feed_inputs()
    for dec in (30, 8, HYUNDAI_FCA11_LONG_RATE_STEP + 1):
      self.assertFalse(self._tx(self._brake_cmd(dec)), f"reopen at {dec} after a passive frame must be refused")
    self.assertTrue(self._tx(self._brake_cmd(HYUNDAI_FCA11_LONG_RATE_STEP)))
    _, last, refusals = self._stream(t + self.PERIOD_MS * 1000, 200, start_dec=HYUNDAI_FCA11_LONG_RATE_STEP)
    self.assertEqual((30, []), (last, refusals))

  def test_f3_churn_alternating_passive_never_exceeds_onset(self):
    """F3: alternating passive / actuating frames can never climb: each reopen is capped at RATE_STEP again."""
    self._rearm()
    self._open_window()
    t = 1000000
    for _ in range(20):
      t += self.PERIOD_MS * 1000
      self.safety.set_timer(t)
      self._feed_inputs()
      self.assertFalse(self._tx(self._brake_cmd(HYUNDAI_FCA11_LONG_RATE_STEP + 1)))
      self.assertTrue(self._tx(self._brake_cmd(HYUNDAI_FCA11_LONG_RATE_STEP)))
      self.assertTrue(self._tx(self._fca11()))

  def test_f3_stale_stream_restarts_the_onset(self):
    """F3 (stale variant, 0035): brake at 30, then the host goes SILENT past TX_STALE. hyundai_fwd_hook has handed
    FCA11 back to the camera (the ESC sees the camera's idle frame = 0), so a reopen at 30 would be a 0 -> 30 step at
    the ESC: it is REFUSED and the reopen restarts at RATE_STEP. At exactly TX_STALE the stream is still ours (the
    fwd hook still blocks the camera), so the reference is kept and holding 30 is accepted."""
    for gap_us, held_ok in ((HYUNDAI_FCA11_LONG_TX_STALE_US, True), (HYUNDAI_FCA11_LONG_TX_STALE_US + 1, False),
                            (2000000, False)):
      with self.subTest(gap_us=gap_us):
        self._rearm()
        self._open_window()
        t, last, refusals = self._stream(1000000, 500)
        self.assertEqual((30, []), (last, refusals))
        self.safety.set_timer(t + gap_us)
        self._feed_inputs()
        self.assertEqual(not held_ok, self.safety.safety_fwd_hook(2, 0x38D) == 0)   # camera forwarded iff stale
        self.assertEqual(held_ok, self._tx(self._brake_cmd(30)))
        if not held_ok:
          self.assertFalse(self._tx(self._brake_cmd(HYUNDAI_FCA11_LONG_RATE_STEP + 1)))
          self.assertTrue(self._tx(self._brake_cmd(HYUNDAI_FCA11_LONG_RATE_STEP)))

  def test_f4_overrides_still_fire_10s_into_a_continuous_episode(self):
    """F4: at t = 10 000 ms into a continuous, accepted stream every independent gate still refuses on the SAME
    frame: brake press, cruise-cancel, heartbeat loss, stale inputs, gear != D, gas press. None of them is keyed to
    episode state (the old budget was the only thing that 'did something' at 2.5 s). 0040: the wheel-speed floor is
    GONE, so a low wheel speed is NO LONGER one of these gates (see test_floor_and_no_ceiling)."""
    faults = {
      "brake_press": lambda: self._rx(self._user_brake_msg(True)),
      "cruise_cancel": lambda: self._rx(self._button_msg(Buttons.CANCEL)),
      "heartbeat_loss": lambda: self.safety.set_heartbeat_engaged(False),
      "controls_lost": lambda: self.safety.set_controls_allowed(False),
      "gear_n": lambda: self._rx(self._gear_msg(6)),
      "gear_r": lambda: self._rx(self._gear_msg(7)),
      "gas_press": lambda: self._rx(self._interceptor_user_gas(0x1000)),
    }
    for name, fault in faults.items():
      with self.subTest(fault=name):
        self._rearm()
        self._open_window()
        t, last, refusals = self._stream(1000000, 10000)
        self.assertEqual((30, []), (last, refusals), "the stream must be live and accepted at 10 s")
        t += self.PERIOD_MS * 1000
        self.safety.set_timer(t)
        self._feed_inputs()
        fault()
        self.assertFalse(self._tx(self._brake_cmd(30)), f"{name} must refuse at 10 s")
        self.assertFalse(self._tx(self._brake_cmd(HYUNDAI_FCA11_LONG_RATE_STEP)), f"{name}: any level refused")
    # stale inputs at 10 s: each input individually stops arriving for > 100 ms
    feeds = {"brake": lambda: self._rx(self._user_brake_msg(False)), "gas": lambda: self._rx(self._user_gas_msg(0)),
             "gear": lambda: self._rx(self._gear_msg(5)), "speed": lambda: self._rx(self._wheel_msg(self.IN_WINDOW_RAW))}
    for stale in feeds:
      with self.subTest(stale=stale):
        self._rearm()
        self._open_window()
        t, last, refusals = self._stream(1000000, 10000)
        self.assertEqual((30, []), (last, refusals))
        for k in range(1, 7):                                   # 6 more host frames (120 ms) with `stale` withheld
          self.safety.set_timer(t + k * self.PERIOD_MS * 1000)
          for name, f in feeds.items():
            if name != stale:
              f()
          self._rx(self._interceptor_user_gas(0))
          ok = self._tx(self._brake_cmd(30))
          if k * self.PERIOD_MS > 100:
            self.assertFalse(ok, f"stale {stale} at +{k * self.PERIOD_MS} ms must refuse")
          else:
            self.assertTrue(ok, f"{stale} still fresh at +{k * self.PERIOD_MS} ms")

  def test_f4_stop_termination_at_the_speed_floor(self):
    """0040: THE FLIP. RENAMED from "termination at the speed floor" - a long episode decelerating THROUGH the old
    9 km/h boundary and all the way to a full stop is now ACCEPTED at EVERY frame (raw 289, 288, ... 0), and it stays
    accepted at a standstill. FALSIFIABLE CONTRAST: on the 39-patch tree the first frame whose slowest wheel is <= 288
    is REFUSED, and everything below it too (that was the floor)."""
    self._rearm()
    self._open_window()
    t, _, refusals = self._stream(1000000, 15000)
    self.assertEqual([], refusals)
    speed = 9 * self.RAW_PER_KPH + 10
    n_ok = 0
    while speed >= 0:
      t += self.PERIOD_MS * 1000
      self.safety.set_timer(t)
      self._feed_inputs(speed_raw=speed)
      ok = self._tx(self._brake_cmd(30))
      self.assertTrue(ok, f"speed raw {speed} must be ACCEPTED (no floor)")
      n_ok += 1
      speed -= 1
    self.assertGreater(n_ok, 9 * self.RAW_PER_KPH)   # we did drive the whole range down to zero
    # and at a full stop it is STILL accepted (no self-imposed stop termination), however long we hold
    for _ in range(200):
      t += self.PERIOD_MS * 1000
      self.safety.set_timer(t)
      self._feed_inputs(speed_raw=0)
      self.assertTrue(self._tx(self._brake_cmd(30)), "a standstill hold must stay accepted")

  def test_f5_regression_cap_slew_release_passive(self):
    """F5: 31 refused; +4 per frame enforced (+5 refused); immediate release accepted; passive frames are always
    accepted (mid-episode, after a refusal, while disengaged, at any time)."""
    self._rearm()
    self._open_window()
    t, last, _ = self._stream(1000000, 300)
    self.assertEqual(30, last)
    self.assertFalse(self._tx(self._brake_cmd(31)))
    self.assertTrue(self._tx(self._brake_cmd(2)))                 # immediate release
    self.assertFalse(self._tx(self._brake_cmd(2 + HYUNDAI_FCA11_LONG_RATE_STEP + 1)))
    self.assertTrue(self._tx(self._brake_cmd(2 + HYUNDAI_FCA11_LONG_RATE_STEP)))
    self.assertTrue(self._tx(self._fca11()))                       # passive mid-stream
    self.assertFalse(self._tx(self._brake_cmd(31)))
    self.assertTrue(self._tx(self._fca11()))                       # passive after a refusal
    self._engage(False, False)
    self.assertTrue(self._tx(self._fca11()))                       # passive while disengaged
    self.safety.set_timer(t + 60000000)
    self.assertTrue(self._tx(self._fca11()))                       # passive 60 s later, inputs stale

  def test_f6_camera_owns_mid_episode_with_a_wedged_host(self):
    """F6 (starvation gate): a host that looks WEDGED (keeps streaming 30 LSB every 20 ms, heartbeat fine, controls
    allowed, inputs fresh) is cut the moment the stock camera requests FCA11 mid-episode (10 s in): every host frame
    (actuating AND passive) is refused, and the camera's own 0x38D is forwarded to the ESC from that frame on."""
    for field, val in (("CF_VSM_Warn", 2), ("CR_VSM_DecCmd", 0.10), ("FCA_CmdAct", 1), ("CF_VSM_Prefill", 1)):
      with self.subTest(field=field):
        self._rearm()
        self._open_window()
        self._rx(self._cam_fca11())
        t, last, refusals = self._stream(1000000, 10000)
        self.assertEqual((30, []), (last, refusals))
        self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x38D))   # our stream owns the ESC
        self._rx(self._cam_fca11(**{field: val}))                    # the camera's own request
        self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x38D))
        for k in range(1, 251):                                      # 5 s more of the wedged host
          self.safety.set_timer(t + k * self.PERIOD_MS * 1000)
          self._feed_inputs()
          self.assertFalse(self._tx(self._brake_cmd(30)))
          self.assertFalse(self._tx(self._fca11()))
          self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x38D))

  def test_f0_no_budget_or_cooldown_in_the_header(self):
    """F0: the budget/cooldown mechanism is gone from the safety header, not merely raised."""
    with open(HYUNDAI_H) as f:
      src = f.read()
    for name in ("HYUNDAI_FCA11_LONG_MAX_ACT_US", "HYUNDAI_FCA11_LONG_COOLDOWN_US", "hyundai_fca11_long_act_active",
                 "hyundai_fca11_long_ts_act"):
      self.assertNotIn(name, src)

  # *** latched cut ***

  def _assert_cut_latched(self):
    # after a cut, actuation is refused until re-arm; the passive 0x38D verdict depends on the cut type (a driver
    # brake drops controls_allowed -> everything refused; a pedal fault does not), so only the brake is asserted here.
    self.assertFalse(self._tx(self._brake_cmd(1)))
    self._open_window()
    self.assertFalse(self._tx(self._brake_cmd(1)))
    self._rearm()
    self._open_window()
    self.assertTrue(self._tx(self._brake_cmd(1)))

  def test_cut_on_driver_brake(self):
    self._open_window()
    self.assertTrue(self._tx(self._brake_cmd(1)))
    self._rx(self._user_brake_msg(True))
    self._assert_cut_latched()

  def test_cut_on_driver_gas_does_not_latch(self):
    """0038 (owner requirement): a gas press SUPPRESSES braking per-frame but must NOT latch. Braking is refused
    while the pedal is down, refused for GAS_HOLDOFF_US after it releases (a plain elapsed-time hold-off, not a
    latch), and ACCEPTED again after the hold-off - with NO disengage / re-engage. Contrast the old rule (asserted
    below): today a single gas tap killed braking for the rest of the engagement. FALSIFIABLE: the final
    assertTrue (re-arm without a resume edge) fails on the pre-0038 tree."""
    self._open_window()
    self.assertTrue(self._tx(self._brake_cmd(1)))
    # above HYUNDAI_GAS_INTERCEPTOR_THRESHOLD: the pedal sensor sets gas_pressed -> the live window refuses
    self._rx(self._interceptor_user_gas(0x1000))
    self.assertFalse(self._tx(self._brake_cmd(1)))
    # release the gas: still refused inside the hold-off (brake/gas fight suppression)
    self._rx(self._interceptor_user_gas(0))
    self._feed_inputs()
    self.assertFalse(self._tx(self._brake_cmd(1)))
    # past the hold-off, with NO new resume edge and the same engagement: braking is served again
    self.safety.set_timer(1000000 + HYUNDAI_FCA11_LONG_GAS_HOLDOFF_US)
    self._feed_inputs()
    self.assertTrue(self._tx(self._brake_cmd(1)), "gas must not lock out future braking (no resume edge)")

  def test_gas_press_never_sets_the_driver_latch(self):
    """0038: gas is removed from the driver-input latch entirely. A gas press must not set the latch that survives
    on brake / gear-out, so once the live window is clean (post hold-off) actuation is served with no re-arm."""
    self._open_window()
    self._rx(self._interceptor_user_gas(0x1000))
    self._rx(self._interceptor_user_gas(0))
    self._feed_inputs()
    self.safety.set_timer(1000000 + HYUNDAI_FCA11_LONG_GAS_HOLDOFF_US)
    self._feed_inputs()
    self.assertTrue(self._tx(self._brake_cmd(1)))

  # *** 0038: the owner's exact scenario - gas escape must not lock out the NEXT braking event ***

  def test_owner_scenario_gas_escape_then_a_new_brake_event_is_served(self):
    """ENGAGE -> brake -> driver presses gas (override) -> release -> keep driving -> a NEW brake request seconds
    later, with NO disengage/re-engage, must be APPLIED. This is the exact bug the owner reported (gas to escape an
    over-brake, then the next stop is unbraked). FALSIFIABLE: on the pre-0038 tree the final assertTrue fails (the
    gas latch never clears without a resume edge)."""
    self._rearm()
    self._open_window()
    # (1) an episode is running and accepted
    self.assertTrue(self._tx_brake(self.DEC_CAP))
    # (2) the driver escapes with the gas: braking is refused while the pedal is down
    self._rx(self._interceptor_user_gas(0x1000))
    self.assertFalse(self._tx(self._brake_cmd(1)))
    # (3) the driver lifts off and CRUISES for several seconds (the host keeps streaming); the hold-off expires
    self._rx(self._interceptor_user_gas(0))
    t = 1000000
    for _ in range(50):                       # 1 s of benign host traffic, gas released
      t += self.PERIOD_MS * 1000
      self.safety.set_timer(t)
      self._feed_inputs()
      self._tx(self._interceptor_gas_cmd(0))
    # (4) a NEW, distinct brake request arrives (approaching a slowing car) - NO re-engage happened
    t += self.PERIOD_MS * 1000
    self.safety.set_timer(t)
    self._feed_inputs()
    self.assertTrue(self._tx(self._brake_cmd(HYUNDAI_FCA11_LONG_RATE_STEP)),
                    "a new brake event after a gas escape must be served (no lockout)")

  def test_first_frame_after_the_holdoff_restarts_at_the_onset_floor(self):
    """0038 (criterion c): the first ACCEPTED frame after the hold-off restarts at the onset floor (RATE_STEP from 0,
    the stream having gone stale past TX_STALE during the suppression), so there is no jump back to the pre-gas
    level. A reopen at RATE_STEP+1 is refused; at RATE_STEP it is accepted and then ramps."""
    self._rearm()
    self._open_window()
    self.assertTrue(self._tx_brake(HYUNDAI_FCA11_LONG_MAX_DEC))     # running at the cap
    self._rx(self._interceptor_user_gas(0x1000))                    # gas: episode suppressed
    self.assertFalse(self._tx(self._brake_cmd(HYUNDAI_FCA11_LONG_MAX_DEC)))
    self._rx(self._interceptor_user_gas(0))
    self.safety.set_timer(1000000 + HYUNDAI_FCA11_LONG_GAS_HOLDOFF_US)
    self._feed_inputs()
    # no jump: the pre-gas level is refused, the onset step is accepted
    self.assertFalse(self._tx(self._brake_cmd(HYUNDAI_FCA11_LONG_MAX_DEC)))
    self.assertFalse(self._tx(self._brake_cmd(HYUNDAI_FCA11_LONG_RATE_STEP + 1)))
    self.assertTrue(self._tx(self._brake_cmd(HYUNDAI_FCA11_LONG_RATE_STEP)))

  # *** 0038: host-liveness (heartbeat) watchdog ***

  def test_host_liveness_watchdog_stale_refuses_then_fresh_resumes(self):
    """0038 (criterion e): if the host stops streaming for > HOST_HB_TIMEOUT_US, its next actuating frame is refused
    (the controls loop was gone); the following frame - a few ms later - is fresh again and accepted, so the clause
    cannot wedge. The passive release frame is never gated by the watchdog."""
    self._rearm()
    self._open_window()
    # seed the host clock with a benign frame at t0
    self.assertTrue(self._tx(self._interceptor_gas_cmd(0)))
    # the host went silent for 600 ms, then a fresh window + an actuating frame
    self.safety.set_timer(1000000 + HYUNDAI_FCA11_LONG_HOST_HB_TIMEOUT_US + 100000)
    self._feed_inputs()
    self.assertFalse(self._tx(self._brake_cmd(1)), "stale host: actuation refused")
    # the passive close frame is NOT gated by the watchdog (the release path)
    self.assertTrue(self._tx(self._fca11()))
    # a few ms later the host is fresh again -> accepted
    self.safety.set_timer(1000000 + HYUNDAI_FCA11_LONG_HOST_HB_TIMEOUT_US + 100000 + self.PERIOD_MS * 1000)
    self._feed_inputs()
    self.assertTrue(self._tx(self._brake_cmd(1)), "fresh host: actuation resumes")

  def test_host_liveness_watchdog_never_refuses_the_first_frame_of_an_arm(self):
    """0038: with no host frame yet this arm (host_hb_ts unset) the watchdog reads as fresh, so the very first brake
    of an engagement is never refused by it - however long the arm has been idle."""
    self._rearm()
    self.safety.set_timer(1000000)
    self._open_window()
    self.safety.set_timer(1000000 + 10 * HYUNDAI_FCA11_LONG_HOST_HB_TIMEOUT_US)   # long after the arm
    self._feed_inputs()
    self.assertTrue(self._tx(self._brake_cmd(1)))

  # *** 0038: throttle exclusivity ***

  def test_throttle_stripped_while_the_host_commands_brake(self):
    """0038 (criterion f): while an actuating brake frame is on the wire, a NON-ZERO GAS_COMMAND is refused (a
    wedged host can never brake AND throttle); the ZERO/clear frame is always accepted, and after the brake window
    the throttle passes again. This can only ever remove throttle - braking authority is untouched."""
    self._rearm()
    self._open_window()
    self.assertTrue(self._tx(self._brake_cmd(1)))                     # actuating brake frame accepted
    self.assertFalse(self._tx(self._interceptor_gas_cmd(GAS_ON)), "no throttle alongside a brake command")
    self.assertTrue(self._tx(self._interceptor_gas_cmd(0)))          # the clear frame always passes
    # after the brake window elapses with no brake frame, throttle passes again
    self.safety.set_timer(1000000 + HYUNDAI_FCA11_LONG_BRAKE_THROTTLE_EXCL_US + 1)
    self.assertTrue(self._tx(self._interceptor_gas_cmd(GAS_ON)))

  def test_throttle_not_stripped_by_a_passive_frame(self):
    """0038: only an ACTUATING mirror frame (non-zero brake fields) arms the throttle strip; a passive close frame
    must not, so ordinary throttle passes whenever openpilot is not braking."""
    self._rearm()
    self._open_window()
    self.assertTrue(self._tx(self._fca11()))                         # passive frame
    self.assertTrue(self._tx(self._interceptor_gas_cmd(GAS_ON)), "throttle passes without an actuating brake")

  def test_cut_on_gear_not_d(self):
    self._open_window()
    self.assertTrue(self._tx(self._brake_cmd(1)))
    self._rx(self._gear_msg(4))
    self._assert_cut_latched()

  def test_cut_on_pedal_fault(self):
    # STATE 1 BAD_CHECKSUM / 3 SCE / 6 INVALID latch; 4 STARTUP / 5 TIMEOUT (boot states) do not
    for fault in (1, 3, 6):
      with self.subTest(fault=fault):
        self._rearm()
        self._open_window()
        self.assertTrue(self._tx(self._brake_cmd(1)))
        self._rx(self._interceptor_user_gas(0, state=fault))
        self._assert_cut_latched()
    for boot in (4, 5):
      with self.subTest(boot=boot):
        self._rearm()
        self._open_window()
        self._rx(self._interceptor_user_gas(0, state=boot))
        # boot states do not latch and do not block the window by themselves
        self.assertTrue(self._tx(self._brake_cmd(1)))

  # *** G9 (R7-B): the driver-input cut re-arms on the openpilot RESUME edge ***

  def _disengage_frame(self):
    """Make the ca=false state OBSERVABLE at the frame level: engage=False, then feed one frame. `_engage()`
    writes controls_allowed directly, but hyundai_fca11_long_rx only samples it on an RX frame, so the
    controls_rising tracker needs a frame to have seen ca=false before the next ca=true frame is a rising edge."""
    self._engage(False, False)
    self._rx(self._user_brake_msg(False))

  def _feed_clean_window_inputs(self):
    """Feed the window inputs and freshness WITHOUT touching controls_allowed (no engage); used to set up a
    window while still disengaged so the next `_engage(True, True)` + a single frame is the resume edge."""
    self._rx(self._user_brake_msg(False))
    self._rx(self._user_gas_msg(0))
    self._rx(self._interceptor_user_gas(0))
    self._rx(self._gear_msg(5))
    self._rx(self._wheel_msg(self.IN_WINDOW_RAW))

  def test_driver_cut_before_engage_clears_on_resume(self):
    """T1 (R7-B regression): a driver-input cut latched while openpilot is NOT engaged (gear P / a brake tap
    before the first engage — the route-149 Park->D shuttle) must clear on the next controls_allowed
    rising edge, i.e. the first real engagement / resume. Fails on the shipped rule (the cut never clears until
    hyundai_init)."""
    self._rearm()
    self._disengage_frame()                 # observable ca=false frame
    self._rx(self._gear_msg(0))             # Park while disengaged: driver-input cut latches
    self.assertFalse(self._tx(self._brake_cmd(1)))    # blocked now
    self._feed_clean_window_inputs()        # live inputs clean, but no rising edge yet -> still latched
    self.assertFalse(self._tx(self._brake_cmd(1)))    # still blocked
    self._engage(True, True)                # resume; the NEXT frame carries the rising edge
    self._rx(self._wheel_msg(self.IN_WINDOW_RAW))
    self.assertTrue(self._tx(self._brake_cmd(1)))     # G9: re-armed

  def test_gas_release_within_the_engagement_does_not_lock_out(self):
    """0038: a gas press while ENGAGED no longer latches anything. After the pedal releases and the short hold-off
    elapses (still the same engagement, controls_allowed never dropping), braking is served again - the exact
    scenario the owner described (gas to escape an over-brake, then the NEXT stop must still brake). FALSIFIABLE:
    on the pre-0038 tree the final assertTrue fails (the latch never clears without a resume edge)."""
    self._rearm()
    self._open_window()
    self.assertTrue(self._tx(self._brake_cmd(1)))
    self._rx(self._interceptor_user_gas(0x1000))      # driver gas while ENGAGED (ca stays true)
    self.assertFalse(self._tx(self._brake_cmd(1)))
    self._rx(self._interceptor_user_gas(0))           # release the gas, same engagement, no re-engage
    self._feed_clean_window_inputs()
    self.assertFalse(self._tx(self._brake_cmd(1)))    # still inside the hold-off
    self.safety.set_timer(1000000 + HYUNDAI_FCA11_LONG_GAS_HOLDOFF_US)
    self._feed_clean_window_inputs()
    self.assertTrue(self._tx(self._brake_cmd(1)))     # 0038: served again, no auto-resume needed

  def test_pedal_fault_survives_resume(self):
    """T3: the pedal-FAULT cut is deliberately hard-latched for the whole arm (the pedal sensor is
    untrustworthy under a fault, so a resume must not paper over it). It must survive a resume edge."""
    self._rearm()
    self._disengage_frame()
    self._rx(self._interceptor_user_gas(0, state=3))  # SCE fault -> hyundai_fca11_long_cut (not the driver latch)
    self.assertFalse(self._tx(self._brake_cmd(1)))
    self._feed_clean_window_inputs()
    self._engage(True, True)
    self._rx(self._wheel_msg(self.IN_WINDOW_RAW))
    self.assertFalse(self._tx(self._brake_cmd(1)))    # still latched

  def test_camera_owns_survives_resume(self):
    """T4: a camera FCA11 request keeps the hand-back (and the cut) across a resume edge — the camera's
    authority is not re-armed by an openpilot pause/resume."""
    self._rearm()
    self._disengage_frame()
    self._rx(self._cam_fca11(CF_VSM_Warn=2))          # camera request latches the hand-back
    self.assertFalse(self._tx(self._brake_cmd(1)))
    self._feed_clean_window_inputs()
    self._engage(True, True)
    self._rx(self._wheel_msg(self.IN_WINDOW_RAW))
    self.assertFalse(self._tx(self._brake_cmd(1)))    # still latched

  def test_driver_input_on_the_grant_frame_still_cuts_then_rearms_on_release(self):
    """T5 (G9b, drive-14): the SET runs BEFORE the clear, so a driver input ON the grant frame re-latches (driver
    wins that frame). The frame AFTER the input is released is CLEAN and — because the resume edge already armed
    the re-arm — it is ACCEPTED: an input merely HELD at the grant must not bar FCA11 for the whole engagement
    (the drive-14 bug: 117 refused frames). On the shipped rule AND on plain G9 the frame after the release stays
    refused, so this is the falsifiable contrast for the clear-on-first-clean-frame behaviour.
    """
    self._rearm()
    self._disengage_frame()
    self._rx(self._user_brake_msg(True))               # brake held while disengaged -> driver cut
    self._engage(True, True)                          # grant
    self._rx(self._user_brake_msg(True))              # grant frame CARRYING the brake: driver wins THIS frame
    self.assertFalse(self._tx(self._brake_cmd(1)))    # refused while the brake is actually held
    self._feed_clean_window_inputs()                  # release it, no new edge
    self.assertTrue(self._tx(self._brake_cmd(1)))     # G9b: first CLEAN frame after the grant -> re-armed

  def test_gas_held_at_grant_does_not_lock_out_braking(self):
    """0038: engaging with the foot still on the gas no longer latches anything. The gas press refuses per-frame
    while it is held, and once it releases (plus the short hold-off) braking is served - NOT barred for the whole
    engagement (the drive-14 bug: 117 refused frames under the old gas latch)."""
    self._rearm()
    self._disengage_frame()
    self._rx(self._interceptor_user_gas(0x1000))      # driver gas over threshold while disengaged
    self.assertFalse(self._tx(self._brake_cmd(1)))
    self._engage(True, True)                          # grant with the foot STILL on the gas
    self._rx(self._interceptor_user_gas(0x1000))      # grant frame CARRYING the held gas -> refused this frame
    self.assertFalse(self._tx(self._brake_cmd(1)))     # refused while the gas is actually held
    # the gas falls below threshold (still inside the same engagement, no new rising edge) and the window is clean
    self._feed_clean_window_inputs()
    self.assertFalse(self._tx(self._brake_cmd(1)))     # inside the hold-off
    self.safety.set_timer(1000000 + HYUNDAI_FCA11_LONG_GAS_HOLDOFF_US)
    self._feed_clean_window_inputs()
    self.assertTrue(self._tx(self._brake_cmd(1)))      # 0038: first frame past the hold-off -> accepted

  def test_engage_with_gas_still_held_is_refused(self):
    """0038 required regression: engage with the gas STILL over threshold and KEEP it held -> every actuating frame
    stays refused. The refusal is per-frame on the live pedal, so a pedal that never falls below the threshold never
    lets a brake through (no auto-resume from a re-arm that was never earned)."""
    self._rearm()
    self._disengage_frame()
    self._rx(self._interceptor_user_gas(0x1000))      # gas held while disengaged
    self._engage(True, True)                          # grant with gas held
    self._rx(self._interceptor_user_gas(0x1000))      # grant frame, gas still held
    self.assertFalse(self._tx(self._brake_cmd(0)))
    # window inputs clean EXCEPT the gas (still held): the live !gas_pressed window refuses
    self._rx(self._user_brake_msg(False))
    self._rx(self._user_gas_msg(0))
    self._rx(self._gear_msg(5))
    self._rx(self._wheel_msg(self.IN_WINDOW_RAW))
    self._rx(self._interceptor_user_gas(0x1000))      # still held
    self.assertFalse(self._tx(self._brake_cmd(1)))    # still refused (live gas blocks the window every frame)

  def test_engage_with_brake_held_is_refused(self):
    """G9b required regression: engage with the brake still held -> refused, and refused for as long as it is
    held (the live window also carries !brake_pressed, so this holds with or without the latch)."""
    self._rearm()
    self._disengage_frame()
    self._rx(self._user_brake_msg(True))              # brake held while disengaged
    self._engage(True, True)
    self._rx(self._user_brake_msg(True))              # grant frame, brake still held -> refused
    self.assertFalse(self._tx(self._brake_cmd(1)))
    self._rx(self._user_brake_msg(True))              # keep holding: still refused
    self.assertFalse(self._tx(self._brake_cmd(1)))

  def test_engage_with_gear_not_d_is_refused(self):
    """G9b required regression: engage with the gear out of D -> refused; the gear cut re-arms only once the gear
    is actually in D on a clean frame, and re-latches the moment it leaves D again."""
    self._rearm()
    self._disengage_frame()
    self._rx(self._gear_msg(0))                       # Park while disengaged -> gear cut
    self._engage(True, True)
    self._rx(self._gear_msg(0))                       # grant frame, gear still P
    self._feed_clean_window_inputs()                  # gear now D, everything clean -> first clean frame
    self.assertTrue(self._tx(self._brake_cmd(1)))     # G9b: gear in D on a clean frame -> re-armed
    self._rx(self._gear_msg(0))                       # shift back out of D: cut re-latches immediately
    self.assertFalse(self._tx(self._brake_cmd(1)))

  def test_camera_owns_hard_latched_across_gas_release(self):
    """G9b required: the camera hand-back is HARD-latched (separate state from the driver latch), so a clean frame
    after a camera request must NOT re-arm it (rule 4 of the G9b review)."""
    self._rearm()
    self._disengage_frame()
    self._rx(self._cam_fca11(CF_VSM_Warn=2))          # camera request -> hyundai_fca11_long_cut (hard)
    self._engage(True, True)
    self._feed_clean_window_inputs()                  # a fully clean frame, post-grant
    self.assertFalse(self._tx(self._brake_cmd(1)))    # still refused: camera owns is not the driver latch

  def test_grant_within_one_period_of_reinit_with_brake_held(self):
    """T6 (post-init race, recommended by second-opinion review): a mid-drive re-init while the driver is
    physically holding the brake clears the parsed brake_pressed; the driver-input latch must still start SET
    so that a grant landing before the next brake frame cannot accept an actuating frame. With the latch
    starting false this test accepts (the race the `driver_cut = true` init closes)."""
    self._rearm()
    self._rx(self._user_brake_msg(True))               # driver physically holds the brake
    self._rearm()                                     # mid-drive reinit: brake_pressed/gas_pressed cleared
    self._engage(False, False)
    self._feed_clean_window_inputs()                  # window inputs set while DISENGAGED (no rising edge yet)
    self._engage(True, True)                          # grant, with no further RX frame
    self.assertFalse(self._tx(self._brake_cmd(1)))    # must stay refused (latch starts set)

  # *** item 3: override -> resume -> ACCEPTED, per override source, and the item-2 discriminator ***

  def test_driver_brake_before_engage_clears_on_resume(self):
    """Item-3: the brake override, latched while NOT engaged (the brake-to-leave-Park tap), must re-arm on the
    next controls_allowed rising edge so the first real engagement can actuate. Sibling of T1 (which pins the
    gear override) and of the gas variant below; together they cover brake / gas / gear separately."""
    self._rearm()
    self._disengage_frame()                            # observable ca=false frame
    self._rx(self._user_brake_msg(True))               # brake tap while disengaged -> driver cut latches
    self._rx(self._user_brake_msg(False))              # release: no rising edge -> still latched
    self.assertFalse(self._tx(self._brake_cmd(1)))
    self._feed_clean_window_inputs()                  # clean window inputs, still disengaged -> still latched
    self.assertFalse(self._tx(self._brake_cmd(1)))
    self._engage(True, True)                          # resume; the NEXT frame carries the rising edge
    self._rx(self._wheel_msg(self.IN_WINDOW_RAW))
    self.assertTrue(self._tx(self._brake_cmd(1)))     # G9: re-armed

  def test_driver_gas_before_engage_does_not_latch(self):
    """0038: a gas tap before the first engage no longer latches anything (gas is out of the driver latch), so the
    first engagement actuates freely once the pedal is released and the hold-off elapses - no resume edge needed.
    Sibling of T1 (gear) and of the brake variant above; covers that gas was removed from THIS latch."""
    self._rearm()
    self._disengage_frame()
    self._rx(self._interceptor_user_gas(0x1000))      # driver gas while disengaged (no longer latches)
    self._rx(self._interceptor_user_gas(0))           # release
    self._feed_clean_window_inputs()
    self.assertFalse(self._tx(self._brake_cmd(1)))    # inside the hold-off
    self._engage(True, True)
    self.safety.set_timer(1000000 + HYUNDAI_FCA11_LONG_GAS_HOLDOFF_US)
    self._feed_clean_window_inputs()
    self.assertTrue(self._tx(self._brake_cmd(1)))     # 0038: served with no re-arm

  def test_reinit_with_a_latched_cut_refuses_until_a_genuine_resume_edge(self):
    """Item-2 discriminator (paired with the `controls_prev = controls_allowed` seed). After a mid-drive
    re-init the driver-input latch must NOT clear on its own: actuation stays REFUSED across the clean frames
    that follow, because the post-init state carries no genuine resume edge. Only a real disengage -> re-engage
    cycle (a controls_allowed rising edge a frame actually observes) restores actuation. Fails if the
    `driver_cut = true` seed is dropped: the pre-re-init latch would be lost and the grant below would be
    accepted with no edge. Also pins that the seed is not weakened by a synthetic edge from a stale
    controls_prev (the reviewer's finding): the seed is controls_allowed, and no frame has observed a rise."""
    self._rearm()
    self._open_window()
    self.assertTrue(self._tx(self._brake_cmd(1)))
    self._rx(self._user_brake_msg(True))              # driver cut latched while ENGAGED
    self.assertFalse(self._tx(self._brake_cmd(1)))
    self._rearm()                                    # mid-drive re-init: controls_allowed forced false
    self._engage(False, False)
    # clean frames while DISENGAGED: no rising edge anywhere, latch must survive every one of them
    for _ in range(3):
      self._feed_clean_window_inputs()
      self.assertFalse(self._tx(self._brake_cmd(1)))
    self._engage(True, True)                         # grant ... but no RX frame has observed the edge yet
    self.assertFalse(self._tx(self._brake_cmd(1)))    # still REFUSED: the seed has not seen a genuine edge
    self._rx(self._wheel_msg(self.IN_WINDOW_RAW))    # the first frame observes the grant = the genuine edge
    self.assertTrue(self._tx(self._brake_cmd(1)))     # accepted

  # *** camera hand-back ***

  def test_camera_request_blocks_and_unblocks_fwd(self):
    self._rx(self._cam_fca11())  # camera idle
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x38D))
    self._open_window()
    self.assertTrue(self._tx(self._brake_cmd(1)))
    # panda transmitted -> the camera's copy is blocked while our stream is fresh
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x38D))
    # a real camera request (Warn) hands FCA11 back the same frame, un-blocks the camera, and blocks our TX
    self._rx(self._cam_fca11(CF_VSM_Warn=2))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x38D))
    self._open_window()
    self.assertFalse(self._tx(self._brake_cmd(1)))

  def test_camera_handback_on_stale_tx(self):
    self._open_window()
    self.assertTrue(self._tx(self._brake_cmd(1)))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x38D))
    # panda TX stream goes stale -> the camera forwards again
    self.safety.set_timer(1000000 + HYUNDAI_FCA11_LONG_TX_STALE_US + 1)
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x38D))

  # *** not armed: byte-for-byte pedal behavior ***

  def test_without_bit_is_pedal_only(self):
    self.safety.set_current_safety_param_sp(HyundaiSafetyFlagsSP.NON_SCC | HyundaiSafetyFlagsSP.GAS_INTERCEPTOR | self.SAFETY_PARAM_SP)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, 0)
    self._engage(True, True)
    self._open_window()
    # no FCA11 of any kind, camera forwarded untouched, pedal command still transmittable
    self.assertFalse(self._tx(self._brake_cmd(1)))
    self.assertFalse(self._tx(self._fca11()))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x38D))
    self.assertTrue(self._tx(self._interceptor_gas_cmd(0.1)))
    # G9/T5: with bit 256 unset there is NO FCA11-long latch to set — a gear-P frame while disengaged,
    # then a resume, still transmits no FCA11 of any kind (the whole path is unreachable, pedal unchanged).
    self._engage(False, False)
    self._rx(self._gear_msg(0))
    self._open_window()
    self.assertFalse(self._tx(self._brake_cmd(1)))
    self.assertFalse(self._tx(self._fca11()))

  def test_ignored_on_canfd_and_legacy(self):
    for mode in (CarParams.SafetyModel.hyundaiLegacy, CarParams.SafetyModel.hyundaiCanfd):
      with self.subTest(mode=mode):
        self.safety.set_current_safety_param_sp(self._pedal_sp())
        self.safety.set_safety_hooks(mode, 0)
        self._engage(True, True)
        self.assertFalse(self._tx(self._brake_cmd(1)))


@parameterized_class(LDA_BUTTON)
class TestHyundaiNonSCCCN7SteerRampSafety(TestHyundaiNonSCCSafety):
  """Elantra N (CN7 non-SCC) CN7_STEER_RAMP bit: the full inherited torque/driver/rt/steer-req suite runs with rate-up 4;
  max torque (384), rate-down (7), rt delta (112) and the driver allowance are unchanged."""
  MAX_RATE_UP = 4

  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiNonSCCCN7SteerRampSafety":
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_can_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(HyundaiSafetyFlagsSP.NON_SCC | HyundaiSafetyFlagsSP.CN7_STEER_RAMP |
                                            self.SAFETY_PARAM_SP)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, 0)
    self.safety.init_tests()

  def _hooks(self, sp, param=0):
    self.safety.set_current_safety_param_sp(sp | self.SAFETY_PARAM_SP)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, param)
    self.safety.init_tests()
    self.safety.set_controls_allowed(True)
    self._reset_torque_driver_measurement(0)

  def _step_ok(self, prev, val):
    self._set_prev_torque(prev)
    return self._tx(self._torque_cmd_msg(val))

  def test_cn7_rate_up_boundaries(self):
    for sign in (1, -1):
      for prev in (0, 100, 380):
        self._hooks(HyundaiSafetyFlagsSP.NON_SCC | HyundaiSafetyFlagsSP.CN7_STEER_RAMP)
        self.assertTrue(self._step_ok(sign * prev, sign * (prev + 4)), (sign, prev))
        self.assertFalse(self._step_ok(sign * prev, sign * (prev + 5)), (sign, prev))

  def test_cn7_max_torque_unchanged(self):
    self._hooks(HyundaiSafetyFlagsSP.NON_SCC | HyundaiSafetyFlagsSP.CN7_STEER_RAMP)
    for sign in (1, -1):
      self.assertTrue(self._step_ok(sign * 381, sign * 384))
      self.assertFalse(self._step_ok(sign * 381, sign * 385))
      self.assertFalse(self._step_ok(sign * 384, sign * 385))

  def test_cn7_rate_down_unchanged(self):
    self._hooks(HyundaiSafetyFlagsSP.NON_SCC | HyundaiSafetyFlagsSP.CN7_STEER_RAMP)
    for sign in (1, -1):
      self.assertTrue(self._step_ok(sign * 384, sign * (384 - 7)))
      # dropping faster than 7 is allowed (toward 0); what must hold is that the down-rate did not change for the
      # driver-limit path: with the driver opposing at max, the command must fall by at least 7 per frame
      self._reset_torque_driver_measurement(-sign * 500)
      self._set_prev_torque(sign * 384)
      self.assertFalse(self._tx(self._torque_cmd_msg(sign * (384 - 7 + 1))))
      self._reset_torque_driver_measurement(0)

  def test_cn7_rt_delta_unchanged(self):
    # 4/frame for a whole 250 ms window = 100 < 112, so a continuous max-rate ramp never trips the rt check
    self._hooks(HyundaiSafetyFlagsSP.NON_SCC | HyundaiSafetyFlagsSP.CN7_STEER_RAMP)
    self._set_prev_torque(0)
    for i in range(1, 29):
      self.assertTrue(self._tx(self._torque_cmd_msg(4 * i)), i)
    # but the rt bound itself is still 112: 113 above the rt reference is rejected even if the frame rate is legal
    self.safety.set_rt_torque_last(0)
    self.safety.set_desired_torque_last(110)
    self.assertFalse(self._tx(self._torque_cmd_msg(113)))

  def test_cn7_bit_requires_non_scc(self):
    # bit without NON_SCC: default HKG limits (rate-up 3)
    self._hooks(HyundaiSafetyFlagsSP.CN7_STEER_RAMP)
    self.assertTrue(self._step_ok(0, 3))
    self.assertFalse(self._step_ok(0, 4))

  def test_non_scc_without_bit_unchanged(self):
    self._hooks(HyundaiSafetyFlagsSP.NON_SCC)
    for sign in (1, -1):
      self.assertTrue(self._step_ok(0, sign * 3))
      self.assertFalse(self._step_ok(0, sign * 4))
      self.assertFalse(self._step_ok(sign * 381, sign * 385))

  def test_alt_limits_win_over_cn7_bit(self):
    for param, max_t, up in ((HyundaiSafetyFlags.ALT_LIMITS, 270, 2), (HyundaiSafetyFlags.ALT_LIMITS_2, 170, 2)):
      self._hooks(HyundaiSafetyFlagsSP.NON_SCC | HyundaiSafetyFlagsSP.CN7_STEER_RAMP, param)
      self.assertTrue(self._step_ok(0, up), param)
      self.assertFalse(self._step_ok(0, up + 1), param)
      self.assertTrue(self._step_ok(max_t - 1, max_t), param)
      self.assertFalse(self._step_ok(max_t - 1, max_t + 1), param)

  def test_cn7_bit_reset_on_reinit(self):
    self._hooks(HyundaiSafetyFlagsSP.NON_SCC | HyundaiSafetyFlagsSP.CN7_STEER_RAMP)
    self.assertTrue(self._step_ok(0, 4))
    self._hooks(HyundaiSafetyFlagsSP.NON_SCC)
    self.assertFalse(self._step_ok(0, 4))

  def test_c_and_python_bit_match(self):
    h = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "modes", "hyundai_common.h")
    with open(h) as f:
      m = re.search(r"HYUNDAI_PARAM_SP_CN7_STEER_RAMP\s*=\s*(\d+)", f.read())
    self.assertIsNotNone(m)
    self.assertEqual(int(m.group(1)), HyundaiSafetyFlagsSP.CN7_STEER_RAMP)
    bit = HyundaiSafetyFlagsSP.CN7_STEER_RAMP
    self.assertEqual(bit & (bit - 1), 0)  # single bit
    self.assertEqual(bit, 1024)  # next free SP bit: 256 is FCA11_LONG (patch 0014), 512 is steer-test LKAS_PARK_TEST
    others = [v for k, v in vars(HyundaiSafetyFlagsSP).items() if k.isupper() and k != "CN7_STEER_RAMP"]
    self.assertNotIn(bit, others)
    self.assertLess(bit, 1 << 15)  # Int16 safetyParam


@parameterized_class(ALL_NON_SCC_HEV_EV_COMBOS)
class TestHyundaiNonSCCSafety_HEV_EV(TestHyundaiSafety):

  PCM_STATUS_MSG = ("", "")
  ACC_STATE_MSG = ("", "")
  GAS_MSG = ("", "")
  SAFETY_PARAM = 0

  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiNonSCCSafety_HEV_EV":
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_can_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(HyundaiSafetyFlagsSP.NON_SCC | self.SAFETY_PARAM_SP)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, self.SAFETY_PARAM)
    self.safety.init_tests()

  def _pcm_status_msg(self, enable):
    values = {self.PCM_STATUS_MSG[1]: enable}
    return self.packer.make_can_msg_safety(self.PCM_STATUS_MSG[0], 0, values)

  def _acc_state_msg(self, enable):
    values = {self.ACC_STATE_MSG[1]: enable}
    return self.packer.make_can_msg_safety(self.ACC_STATE_MSG[0], 0, values)

  def _user_gas_msg(self, gas):
    values = {self.GAS_MSG[1]: gas}
    return self.packer.make_can_msg_safety(self.GAS_MSG[0], 0, values, fix_checksum=checksum)


@parameterized_class(LDA_BUTTON)
class TestHyundaiNonSCCLkasParkTestSafety(TestHyundaiNonSCCSafety):
  """
    TEST-ONLY parked LKAS11 steering sweep (HyundaiSafetyFlagsSP.LKAS_PARK_TEST, bit 512) on a non-SCC ICE car.
    TX = LKAS11 ONLY (check_relay, DYNAMIC camera blocking). Passive frames (torque 0, ActToi 0) need only the parked
    state + the 60 s cap; actuating frames need the full window (every input seen + fresh, no latched cut, no gas),
    |torque| <= 384, ActToi with torque, and the +3/-7 rate law + 112/250 ms real-time bound - independent of
    controls_allowed (openpilot is stopped). setUp puts the car PARKED (gear P, wheels 0, all inputs fresh).
  """
  TX_MSGS = [[0x340, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x340,)}
  FWD_BLACKLISTED_ADDRS = {}  # dynamic: see test_camera_lkas_*
  T0 = 1_000_000
  cnt_lvr = 0

  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiNonSCCLkasParkTestSafety":
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_can_generated")
    self.safety = libsafety_py.libsafety
    # arm at T0 WITHOUT a second set_safety_hooks after init_tests (that would reset safety_mode_cnt and mute the relay
    # check, see test_relay_check_kept); init_tests zeroes the timer, so put it back to the arm time
    self.safety.set_timer(self.T0)
    self.safety.set_current_safety_param_sp(self._test_sp())
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, 0)
    self.safety.init_tests()
    self.safety.set_timer(self.T0)
    self._park()

  def _test_sp(self) -> int:
    return HyundaiSafetyFlagsSP.NON_SCC | HyundaiSafetyFlagsSP.LKAS_PARK_TEST | self.SAFETY_PARAM_SP

  # inherited MADS checks assume a fresh init (no prior parked traffic): run them from a fresh arm
  def test_mads_button_not_engaged_without_press(self):
    self._rearm()
    super().test_mads_button_not_engaged_without_press()

  def test_enable_control_allowed_with_manual_mads_button_state(self):
    self._rearm()
    super().test_enable_control_allowed_with_manual_mads_button_state()

  def test_button_sends(self):
    # CLU11 is NOT in this mode's TX list: no button frame of any kind, whatever controls_allowed / cruise state
    for ca in (False, True):
      self.safety.set_controls_allowed(ca)
      for btn in (Buttons.NONE, Buttons.RESUME, Buttons.SET, Buttons.CANCEL):
        self.assertFalse(self._tx(self._button_msg(btn)), (ca, btn))

  def _rearm(self, sp=None, t=None):
    # a fresh arm = a new 0xdc: set_safety_hooks re-runs hyundai_init (clears the cut, the clock and all evidence)
    self.safety.set_timer(self.T0 if t is None else t)
    self.safety.set_current_safety_param_sp(self._test_sp() if sp is None else sp)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, 0)

  # --- messages ---
  def _gear_msg(self, gear):
    return self.packer.make_can_msg_safety("LVR12", 0, {"CF_Lvr_Gear": gear})

  def _wheel_msg(self, fl, fr=None, rl=None, rr=None):
    vals = {"FL": fl, "FR": fl if fr is None else fr, "RL": fl if rl is None else rl, "RR": fl if rr is None else rr}
    values = {"WHL_SPD_%s" % k: v * 0.03125 for k, v in vals.items()}
    values["WHL_SPD_AliveCounter_LSB"] = (self.cnt_speed % 16) & 0x3
    values["WHL_SPD_AliveCounter_MSB"] = (self.cnt_speed % 16) >> 2
    self.__class__.cnt_speed += 1
    return self.packer.make_can_msg_safety("WHL_SPD11", 0, values, fix_checksum=checksum)

  def _sas_msg(self, angle_deg):
    # SAS11 is 6 bytes on the car (the DBC says 5): pad, exactly as the real frame arrives
    dat = bytearray(self.packer.make_can_msg("SAS11", 0, {"SAS_Angle": angle_deg})[1]) + b"\x00"
    return libsafety_py.make_CANPacket(0x2B0, 0, bytes(dat[:6]))

  def _mdps12(self, **kw):
    values = {"CR_Mdps_StrColTq": 0, "CR_Mdps_StrTq": 0.0, "CR_Mdps_OutTq": 0.0}
    values.update(kw)
    return self.packer.make_can_msg_safety("MDPS12", 0, values)

  def _park(self, gear=0, wheel=0, angle=0.0, **mdps):
    self.assertTrue(self._rx(self._user_brake_msg(False)))
    self.assertTrue(self._rx(self._user_gas_msg(0)))
    self.assertTrue(self._rx(self._gear_msg(gear)))
    self.assertTrue(self._rx(self._wheel_msg(wheel)))
    self.assertTrue(self._rx(self._sas_msg(angle)))
    self.assertTrue(self._rx(self._mdps12(**mdps)))

  def _lkas(self, torque, steer_req=None):
    if steer_req is None:
      steer_req = 1 if torque != 0 else 0
    return self._torque_cmd_msg(torque, steer_req=steer_req)

  def _ramp(self, target, step=None):
    # walk the accepted reference to `target` inside the rate law; every frame must pass
    cur = 0
    step = HYUNDAI_LKAS_PARK_RATE_UP if step is None else step
    t = self.safety.get_current_safety_param()  # unused: keep timer untouched (rt window may roll)
    while cur != target:
      cur = cur + max(-step, min(step, target - cur))
      self.assertTrue(self._tx(self._lkas(cur)), cur)
    return cur

  # *** constants: C == Python ***

  def test_c_python_constants_match(self):
    self.assertEqual(HyundaiSafetyFlagsSP.LKAS_PARK_TEST, 512)
    with open(HYUNDAI_COMMON_H) as f:
      self.assertRegex(f.read(), r"HYUNDAI_PARAM_SP_LKAS_PARK_TEST\s*=\s*512\b")
    self.assertEqual(HYUNDAI_LKAS_PARK_MAX_TORQUE, _c_define(HYUNDAI_H, "HYUNDAI_LKAS_PARK_MAX_TORQUE"))
    self.assertEqual(HYUNDAI_LKAS_PARK_RATE_UP, _c_define(HYUNDAI_H, "HYUNDAI_LKAS_PARK_RATE_UP"))
    self.assertEqual(HYUNDAI_LKAS_PARK_RATE_DOWN, _c_define(HYUNDAI_H, "HYUNDAI_LKAS_PARK_RATE_DOWN"))
    self.assertEqual(int(HYUNDAI_LKAS_PARK_MAX_ARM_S * 1e6), _c_define(HYUNDAI_H, "HYUNDAI_LKAS_PARK_MAX_ARM_US"))
    self.assertEqual(HYUNDAI_LKAS_PARK_WHEEL_MAX_RAW, _c_define(HYUNDAI_H, "HYUNDAI_LKAS_PARK_WHEEL_MAX"))
    self.assertEqual(int(HYUNDAI_LKAS_PARK_MAX_ANGLE_DEG * 10), _c_define(HYUNDAI_H, "HYUNDAI_LKAS_PARK_MAX_ANGLE"))
    self.assertEqual(int(HYUNDAI_LKAS_PARK_MAX_DRIVER_TQ_NM * 100), _c_define(HYUNDAI_H, "HYUNDAI_LKAS_PARK_MAX_DRIVER_TQ"))
    # the panda limits equal the car-layer limits the real carcontroller uses for this platform
    from opendbc.car.hyundai.values import CarControllerParams, CAR
    from opendbc.car.structs import CarParams as CP_
    cp = CP_(carFingerprint=CAR.HYUNDAI_ELANTRA_2022_NON_SCC)
    ccp = CarControllerParams(cp)
    # STEER_MAX and the decay rate are the same everywhere (the CN7 ramp only changes rate-UP, and it keeps 384).
    self.assertEqual(ccp.STEER_MAX, HYUNDAI_LKAS_PARK_MAX_TORQUE)
    self.assertEqual(ccp.STEER_DELTA_DOWN, HYUNDAI_LKAS_PARK_RATE_DOWN)
    # The parked sweep has its OWN +3/-7 law (HYUNDAI_LKAS_PARK_RATE_UP, pinned against hyundai.h above). It is NOT the
    # car-layer STEER_DELTA_UP: the normal driving path for this platform carries the CN7_STEER_RAMP, so ccp's rate-up is
    # 4 here. The park mode deliberately keeps +3 (its own contract, reviewed) and does not adopt the ramp.
    self.assertEqual(ccp.STEER_DELTA_UP, 4)  # CN7_STEER_RAMP (patch 0015): normal driving only
    self.assertNotEqual(ccp.STEER_DELTA_UP, HYUNDAI_LKAS_PARK_RATE_UP)

  # *** armed: accepts ***

  def test_armed_accepts_ramp_both_directions(self):
    for sign in (1, -1):
      with self.subTest(sign=sign):
        self._rearm()
        self._park()
        self.assertTrue(self._tx(self._lkas(0)))
        cur = 0
        for _ in range(17):  # 0 -> 51 in 17 frames (inside 112/250 ms)
          cur += 3 * sign
          self.assertTrue(self._tx(self._lkas(cur)), cur)
        self.assertTrue(self._tx(self._lkas(cur - 7 * sign)))  # decay 7
        self.assertTrue(self._tx(self._lkas(0)))               # immediate passive release

  def test_independent_of_controls_allowed(self):
    for ca in (False, True):
      with self.subTest(controls_allowed=ca):
        self._rearm()
        self._park()
        self.safety.set_controls_allowed(ca)
        self.assertTrue(self._tx(self._lkas(3)))

  def test_full_ladder_to_384_accepted(self):
    # the runner's ladder cadence: +3 per 10 ms frame -> 0..384 in 128 frames (1.28 s), each step inside 112/250 ms
    t = self.T0
    cur = 0
    while cur < HYUNDAI_LKAS_PARK_MAX_TORQUE:
      t += 10_000
      self.safety.set_timer(t)
      self._park()
      cur = min(cur + 3, HYUNDAI_LKAS_PARK_MAX_TORQUE)
      self.assertTrue(self._tx(self._lkas(cur)), cur)
    self.assertEqual(cur, 384)
    t += 10_000
    self.safety.set_timer(t)
    self._park()
    self.assertTrue(self._tx(self._lkas(384)))  # hold

  def test_gear_n_accepted(self):
    self._rearm()
    self._park(gear=6)
    self.assertTrue(self._tx(self._lkas(3)))

  # *** unset: rejects exactly like normal non-SCC ***

  def _assert_normal(self):
    # normal behavior: camera LKAS11 statically blocked, openpilot steering requires controls_allowed
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x340))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x485))
    self.safety.set_controls_allowed(False)
    self.assertFalse(self._tx(self._lkas(3)))
    self.assertTrue(self._tx(self._lkas(0)))
    self.assertTrue(self._tx(common.make_msg(0, 0x485, 4)))

  def test_unset_bit_is_normal_mode(self):
    self._rearm(sp=HyundaiSafetyFlagsSP.NON_SCC | self.SAFETY_PARAM_SP)
    self._park()
    self._assert_normal()

  def test_ignored_without_non_scc(self):
    self._rearm(sp=HyundaiSafetyFlagsSP.LKAS_PARK_TEST | self.SAFETY_PARAM_SP)
    self._park()
    self._assert_normal()

  def test_ignored_with_fca11_test_bit(self):
    # one test mode at a time: the FCA11 test bit wins, LKAS11 not transmittable at all
    self._rearm(sp=self._test_sp() | HyundaiSafetyFlagsSP.FCA11_BRAKE_TEST)
    self._park()
    self.assertFalse(self._tx(self._lkas(0)))
    self.assertFalse(self._tx(self._lkas(3)))

  def test_overrides_gas_interceptor(self):
    for dialect in (0, HyundaiSafetyFlagsSP.GAS_INTERCEPTOR_REMAPPED):
      for extra in (0, HyundaiSafetyFlagsSP.FCA11_LONG):
        with self.subTest(dialect=dialect, extra=extra):
          self._rearm(sp=self._test_sp() | HyundaiSafetyFlagsSP.GAS_INTERCEPTOR | dialect | extra)
          self._park()
          self.safety.set_controls_allowed(True)
          self.assertTrue(self._tx(self._lkas(3)))
          for addr in (0x200, 0x700, 0x38D, 0x485, 0x4F1):
            self.assertFalse(self._tx(common.make_msg(0, addr, 8)), f"{addr=:#x}")
          # the driver's gas (EMS16, not the pedal sensor) must still be the gas source and still cut
          self._rx(self._user_gas_msg(1))
          self.assertFalse(self._tx(self._lkas(6)))
          self._rx(self._user_gas_msg(0))
          self.assertFalse(self._tx(self._lkas(3)))

  def test_ignored_on_unsupported_cars(self):
    for param in (HyundaiSafetyFlags.EV_GAS, HyundaiSafetyFlags.HYBRID_GAS, HyundaiSafetyFlags.FCEV_GAS,
                  HyundaiSafetyFlags.CAMERA_SCC, HyundaiSafetyFlags.LONG):
      with self.subTest(param=param):
        self.safety.set_current_safety_param_sp(self._test_sp())
        self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, param)
        self.safety.set_controls_allowed(False)
        self._park()
        self.assertFalse(self._tx(self._lkas(3)))
    for mode in (CarParams.SafetyModel.hyundaiLegacy, CarParams.SafetyModel.hyundaiCanfd):
      with self.subTest(mode=mode):
        self.safety.set_current_safety_param_sp(self._test_sp())
        self.safety.set_safety_hooks(mode, 0)
        self.safety.set_controls_allowed(False)
        self.assertFalse(self._tx(self._lkas(3)))
    # legacy shares hyundai_tx_hook: with the bit set it must still be the NORMAL controls_allowed steering rule
    # (not parked: the park rules would refuse both frames below)
    self.safety.set_current_safety_param_sp(self._test_sp())
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiLegacy, 0)
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._lkas(0)))
    self.assertTrue(self._tx(self._lkas(3)))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x340))   # legacy: camera LKAS11 statically blocked as always

  # *** TX list / forwarding ***

  def test_tx_list_only_lkas11(self):
    self.safety.set_controls_allowed(True)
    for addr in (0x485, 0x4F1, 0x38D, 0x483, 0x420, 0x421, 0x50A, 0x389, 0x4A2, 0x7D0, 0x200, 0x700):
      self.assertFalse(self._tx(common.make_msg(0, addr, 8)), f"{addr=:#x}")

  def test_relay_check_kept(self):
    self._rx(common.make_msg(0, 0x340, 8))
    self.assertTrue(self.safety.get_relay_malfunction())

  def test_camera_lkas_forwarded_until_we_transmit(self):
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x340))     # armed, not transmitting: camera owns LKAS11
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x485))     # LFAHDA always the camera's
    self.assertTrue(self._tx(self._lkas(0)))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x340))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x485))
    self.safety.set_timer(self.T0 + 100_000)
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x340))
    self.safety.set_timer(self.T0 + 100_001)
    self._park()                                                    # inputs fresh: only TX staleness can hand back
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x340))     # runner silent > 100 ms: camera back

  def test_rejected_tx_does_not_block_camera(self):
    self.assertFalse(self._tx(self._lkas(HYUNDAI_LKAS_PARK_MAX_TORQUE + 1)))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x340))

  def test_camera_back_immediately_when_not_parked(self):
    # the moment panda would refuse our next frame (car left park / input stale / cap) the camera's LKAS11 is forwarded
    # on its very next frame - the MDPS is never left without LKAS11 for the 100 ms stale window
    cases = {"wheel": lambda: self._rx(self._wheel_msg(HYUNDAI_LKAS_PARK_WHEEL_MAX_RAW + 1)),
             "gear_d": lambda: self._rx(self._gear_msg(5)),
             "gear_change": lambda: self._rx(self._gear_msg(6)),
             "stale": lambda: self.safety.set_timer(self.T0 + 100_001)}
    for name, f in cases.items():
      with self.subTest(case=name):
        self._rearm()
        self._park()
        self.assertTrue(self._tx(self._lkas(0)))
        self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x340))
        f()
        self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x340))

  def test_camera_back_at_time_cap(self):
    cap_us = int(HYUNDAI_LKAS_PARK_MAX_ARM_S * 1e6)
    self.safety.set_timer(self.T0 + cap_us)
    self._park()
    self.assertTrue(self._tx(self._lkas(0)))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x340))
    self.safety.set_timer(self.T0 + cap_us + 1)
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x340))

  def test_latched_cut_keeps_blocking_while_passive_flows(self):
    # a cut only gates ACTUATION: the passive stream keeps replacing the camera (no source flip-flop at the MDPS)
    self.assertTrue(self._tx(self._lkas(3)))
    self._rx(self._mdps12(CF_Mdps_ToiFlt=1))
    self._park()
    self.assertTrue(self._tx(self._lkas(0)))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x340))

  # *** torque / rate law ***

  def test_torque_clamp(self):
    for torque in (385, 400, 1023, -385, -1024):
      with self.subTest(torque=torque):
        self._rearm()
        self._park()
        self.assertFalse(self._tx(self._lkas(torque)))

  def test_torque_clamp_at_the_limit(self):
    # 384 -> 385 is a legal +1 step: only the clamp can refuse it (both signs)
    for sign in (1, -1):
      with self.subTest(sign=sign):
        t = self.T0
        self._rearm()
        self._park()
        cur = 0
        while abs(cur) < HYUNDAI_LKAS_PARK_MAX_TORQUE:
          t += 10_000
          self.safety.set_timer(t)
          self._park()
          cur = sign * min(abs(cur) + 3, HYUNDAI_LKAS_PARK_MAX_TORQUE)
          self.assertTrue(self._tx(self._lkas(cur)), cur)
        t += 10_000
        self.safety.set_timer(t)
        self._park()
        self.assertFalse(self._tx(self._lkas(sign * (HYUNDAI_LKAS_PARK_MAX_TORQUE + 1))))

  def test_steer_req_required_with_torque(self):
    self.assertFalse(self._tx(self._lkas(3, steer_req=0)))
    self.assertTrue(self._tx(self._lkas(0, steer_req=1)))   # ActToi with zero torque: actuating, in window: ok
    self.assertTrue(self._tx(self._lkas(3, steer_req=1)))

  def test_rate_up_limit(self):
    for sign in (1, -1):
      for step, ok in ((3, True), (4, False), (50, False)):
        with self.subTest(sign=sign, step=step):
          self._rearm()
          self._park()
          self.assertEqual(ok, self._tx(self._lkas(step * sign)))
          if ok:
            self.assertEqual(ok, self._tx(self._lkas(2 * step * sign)))

  def test_rate_up_from_nonzero_reference(self):
    # growth is +3 per frame from ANY reference, not only from zero (both signs)
    for sign in (1, -1):
      for step, ok in ((3, True), (4, False)):
        with self.subTest(sign=sign, step=step):
          self._rearm()
          self._park()
          self._ramp(30 * sign)
          self.assertEqual(ok, self._tx(self._lkas((30 + step) * sign)))

  def test_rate_down_limit(self):
    for sign in (1, -1):
      for drop, ok in ((7, True), (8, False)):
        with self.subTest(sign=sign, drop=drop):
          self._rearm()
          self._park()
          self._ramp(30 * sign)
          self.assertEqual(ok, self._tx(self._lkas((30 - drop) * sign)))

  def test_zero_crossing_limited(self):
    self._ramp(6)
    self.assertFalse(self._tx(self._lkas(-4)))   # 6 -> -4 skips the +/-3 crossing bound
    self._rearm(); self._park()
    self._ramp(3)
    self.assertTrue(self._tx(self._lkas(-3)))    # 3 -> -3: lo = max(3-7, -3) = -3

  def test_realtime_limit(self):
    # 112 per 250 ms: 38 frames of +3 (=114) inside one window is refused at the 38th
    t = self.T0
    for i in range(1, 38):
      t += 1000
      self.safety.set_timer(t)
      self._park()
      self.assertTrue(self._tx(self._lkas(3 * i)), i)
    t += 1000
    self.safety.set_timer(t)
    self._park()
    self.assertFalse(self._tx(self._lkas(3 * 38)))

  def test_rejection_resets_reference(self):
    self._ramp(30)
    self.assertFalse(self._tx(self._lkas(40)))   # rejected
    self.assertFalse(self._tx(self._lkas(30)))   # cannot resume at the old level ...
    self.assertTrue(self._tx(self._lkas(3)))     # ... must ramp from zero again

  def test_passive_release_resets_reference(self):
    self._ramp(30)
    self.assertTrue(self._tx(self._lkas(0)))
    self.assertFalse(self._tx(self._lkas(6)))
    self.assertTrue(self._tx(self._lkas(3)))

  # *** parked state (every frame) ***

  def test_wheel_motion_blocks_every_frame(self):
    for i in range(4):
      for raw, ok in ((HYUNDAI_LKAS_PARK_WHEEL_MAX_RAW, True), (HYUNDAI_LKAS_PARK_WHEEL_MAX_RAW + 1, False)):
        with self.subTest(wheel=i, raw=raw):
          self._rearm()
          self._park()
          w = [0, 0, 0, 0]
          w[i] = raw
          self._rx(self._wheel_msg(*w))
          self.assertEqual(ok, self._tx(self._lkas(3)))
          self.assertEqual(ok, self._tx(self._lkas(0)))   # passive frames obey the same wheel gate

  def test_gear_rules(self):
    for gear in (4, 5, 7, 8, 12):
      with self.subTest(gear=gear):
        self._rearm()
        self._park(gear=gear)
        self.assertFalse(self._tx(self._lkas(0)))
        self.assertFalse(self._tx(self._lkas(3)))

  def test_gear_change_p_to_n_refused(self):
    # the arm's FIRST gear is the reference: P then N inside one arm is a change (re-arm per gear)
    self.assertTrue(self._tx(self._lkas(3)))
    self._rx(self._gear_msg(6))
    self.assertFalse(self._tx(self._lkas(0)))
    self.assertFalse(self._tx(self._lkas(3)))
    self._rx(self._gear_msg(0))
    self.assertTrue(self._tx(self._lkas(0)))     # back in the reference gear: passive ok ...
    self.assertFalse(self._tx(self._lkas(3)))    # ... actuation latched off

  def test_needs_every_input_seen(self):
    feeds = {"gear": lambda: self._rx(self._gear_msg(0)), "speed": lambda: self._rx(self._wheel_msg(0)),
             "angle": lambda: self._rx(self._sas_msg(0)), "mdps": lambda: self._rx(self._mdps12()),
             "gas": lambda: self._rx(self._user_gas_msg(0))}
    for missing in feeds:
      with self.subTest(missing=missing):
        self._rearm(t=50_000)  # near-zero timer: a never-seen input's ts=0 looks FRESH, only the seen-mask blocks it
        for name, f in feeds.items():
          if name != missing:
            f()
        self.assertFalse(self._tx(self._lkas(3)))

  def test_stale_input_blocks(self):
    feeds = {"gear": lambda: self._rx(self._gear_msg(0)), "speed": lambda: self._rx(self._wheel_msg(0)),
             "angle": lambda: self._rx(self._sas_msg(0)), "mdps": lambda: self._rx(self._mdps12()),
             "gas": lambda: self._rx(self._user_gas_msg(0))}
    for stale in feeds:
      with self.subTest(stale=stale):
        self._rearm()
        self._park()
        self.safety.set_timer(self.T0 + 100_000)
        self.assertTrue(self._tx(self._lkas(3)))   # exactly 100 ms: fresh
        self.safety.set_timer(self.T0 + 100_001)
        for name, f in feeds.items():
          if name != stale:
            f()
        self.assertFalse(self._tx(self._lkas(3)))
        feeds[stale]()
        self.assertTrue(self._tx(self._lkas(3)))   # not latched: freshness restored -> ok (from the reset reference)

  # *** latched cut ***

  def _assert_cut_latched(self):
    self.assertFalse(self._tx(self._lkas(3)))
    self._park()                                   # conditions fully restored ...
    self.assertFalse(self._tx(self._lkas(3)))      # ... still cut
    self.assertFalse(self._tx(self._lkas(0, steer_req=1)))
    self.assertTrue(self._tx(self._lkas(0)))       # the passive stream may continue (no LKAS11 gap at the MDPS)
    self._rearm()
    self._park()
    self.assertTrue(self._tx(self._lkas(3)))       # a fresh arm starts un-cut

  def test_cut_on_mdps_fault_bits(self):
    for sig in ("CF_Mdps_Def", "CF_Mdps_ToiUnavail", "CF_Mdps_ToiFlt", "CF_Mdps_FailStat", "CF_Mdps_SErr"):
      with self.subTest(sig=sig):
        self._rearm()
        self._park()
        self.assertTrue(self._tx(self._lkas(3)))
        self._rx(self._mdps12(**{sig: 1}))
        self._assert_cut_latched()

  def test_toi_active_is_not_a_fault(self):
    self._rx(self._mdps12(CF_Mdps_ToiActive=1, CR_Mdps_OutTq=10.0))
    self.assertTrue(self._tx(self._lkas(3)))

  def test_cut_on_driver_torque(self):
    lim = HYUNDAI_LKAS_PARK_MAX_DRIVER_TQ_NM
    for tq, cut in ((lim, False), (-lim, False), (lim + 0.02, True), (-(lim + 0.02), True)):
      with self.subTest(tq=tq):
        self._rearm()
        self._park()
        self._rx(self._mdps12(CR_Mdps_StrTq=tq))
        if cut:
          self._assert_cut_latched()
        else:
          self.assertTrue(self._tx(self._lkas(3)))

  def test_cut_on_angle(self):
    lim = HYUNDAI_LKAS_PARK_MAX_ANGLE_DEG
    for ang, cut in ((lim, False), (-lim, False), (lim + 0.1, True), (-(lim + 0.1), True)):
      with self.subTest(ang=ang):
        self._rearm()
        self._park()
        self._rx(self._sas_msg(ang))
        if cut:
          self._assert_cut_latched()
        else:
          self.assertTrue(self._tx(self._lkas(3)))

  def test_cut_on_gas(self):
    self._rx(self._user_gas_msg(1))
    self.assertFalse(self._tx(self._lkas(3)))
    self._rx(self._user_gas_msg(0))
    self._assert_cut_latched()

  def test_cut_on_wheel_motion_latches(self):
    self._rx(self._wheel_msg(HYUNDAI_LKAS_PARK_WHEEL_MAX_RAW + 1))
    self._rx(self._wheel_msg(0))
    self._assert_cut_latched()

  def test_cut_on_gear_leave(self):
    for gear in (5, 7):
      with self.subTest(gear=gear):
        self._rearm()
        self._park()
        self._rx(self._gear_msg(gear))
        self._rx(self._gear_msg(0))
        self._assert_cut_latched()

  # *** time cap ***

  def test_time_cap(self):
    cap_us = int(HYUNDAI_LKAS_PARK_MAX_ARM_S * 1e6)
    self.safety.set_timer(self.T0 + cap_us)
    self._park()
    self.assertTrue(self._tx(self._lkas(3)))     # exactly 60 s: allowed
    self.safety.set_timer(self.T0 + cap_us + 1)
    self._park()
    self.assertFalse(self._tx(self._lkas(0)))    # past the cap: EVERY frame refused, passive too
    self.assertFalse(self._tx(self._lkas(3)))
    self.safety.set_timer(self.T0 + cap_us + 100_002)
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x340))  # camera owns LKAS11 again
    self._rearm(t=self.T0 + cap_us + 200_000)
    self._park()
    self.assertTrue(self._tx(self._lkas(3)))     # re-arm restarts the clock

  def test_brake_is_not_a_cut(self):
    # parked with the foot on the brake is the normal test posture: never cut on brake
    self._rx(self._user_brake_msg(True))
    self.assertTrue(self._tx(self._lkas(3)))

  # Inherited torque-steering suites assume the NORMAL controls_allowed / driver-limit semantics, which this mode
  # replaces by its own parked rules (covered above: clamp, rate up/down, crossing, real-time, steer_req, latches).
  @unittest.skip("parked LKAS rules replace the controls_allowed steering suite (see test_rate_*/test_torque_clamp)")
  def test_steer_safety_check(self):
    pass

  @unittest.skip("parked LKAS rules replace the controls_allowed steering suite")
  def test_non_realtime_limit_up(self):
    pass

  @unittest.skip("parked LKAS rules replace the controls_allowed steering suite")
  def test_non_realtime_limit_down(self):
    pass

  @unittest.skip("parked LKAS rules replace the controls_allowed steering suite")
  def test_against_torque_driver(self):
    pass

  @unittest.skip("parked LKAS rules replace the controls_allowed steering suite (see test_realtime_limit)")
  def test_realtime_limits(self):
    pass

  @unittest.skip("parked LKAS: torque requires CF_Lkas_ActToi, no steer-req cut tolerance (test_steer_req_required_with_torque)")
  def test_steer_req_bit(self):
    pass

  @unittest.skip("parked LKAS: no steer-req cut tolerance")
  def test_steer_req_bit_frames(self):
    pass

  @unittest.skip("parked LKAS: no steer-req cut tolerance")
  def test_steer_req_bit_multi_invalid(self):
    pass

  @unittest.skip("parked LKAS: no steer-req cut tolerance")
  def test_steer_req_bit_realtime(self):
    pass


if __name__ == "__main__":
  unittest.main()
