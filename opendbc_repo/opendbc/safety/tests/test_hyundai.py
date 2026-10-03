#!/usr/bin/env python3
from opendbc.testing import parameterized_class
import random
import unittest

from opendbc.car.hyundai.values import HyundaiSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.safety.tests.gas_interceptor_common import GasInterceptorSafetyTest
from opendbc.safety.tests.hyundai_common import Buttons, HyundaiButtonBase, HyundaiLongitudinalBase

from opendbc.sunnypilot.car import crc8_pedal
from opendbc.sunnypilot.car.hyundai.gas_interceptor import GAS_INTERCEPTOR_DBC, HYUNDAI_GAS_INTERCEPTOR_THRESHOLD, \
                                                         REMAPPED_IDS, STANDARD_IDS

from opendbc.sunnypilot.car.hyundai.values import HyundaiSafetyFlagsSP, HYUNDAI_FCA11_TEST_MAX_DEC

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

  def _accel_msg(self, accel, aeb_req=False, aeb_decel=0):
    values = {
      "aReqRaw": accel,
      "aReqValue": accel,
      "AEB_CmdAct": int(aeb_req),
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

  def _accel_msg(self, accel, aeb_req=False, aeb_decel=0):
    values = {
      "aReqRaw": accel,
      "aReqValue": accel,
      "AEB_CmdAct": int(aeb_req),
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
    Non-SCC ICE + comma pedal: SET/RES button engagement (no SCC), TX = base + the active dialect's GAS_COMMAND only
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
      self.assertEqual(controls_allowed, self._tx(self._interceptor_gas_cmd(1)))
      # the inactive dialect's command id is not in the TX allowlist at all, not even a zero command
      self.assertFalse(self._tx(self._interceptor_gas_cmd(0, ids=self.OTHER_IDS)))
      self.assertFalse(self._tx(self._interceptor_gas_cmd(1, ids=self.OTHER_IDS)))

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
    self.assertFalse(self._tx(self._interceptor_gas_cmd(1)))
    self._rx(self._interceptor_user_gas(0))
    self.assertTrue(self._tx(self._interceptor_gas_cmd(1)))

  def test_remapped_bit_alone_is_inert(self):
    # GAS_INTERCEPTOR_REMAPPED without GAS_INTERCEPTOR selects nothing: no pedal command of either dialect
    self.safety.set_current_safety_param_sp(HyundaiSafetyFlagsSP.NON_SCC | HyundaiSafetyFlagsSP.GAS_INTERCEPTOR_REMAPPED |
                                            self.SAFETY_PARAM_SP)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, 0)
    self.safety.set_controls_allowed(True)
    for ids in (STANDARD_IDS, REMAPPED_IDS):
      self.assertFalse(self._tx(self._interceptor_gas_cmd(0, ids=ids)))
      self.assertFalse(self._tx(self._interceptor_gas_cmd(1, ids=ids)))

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
    # but it is tracked so openpilot may cancel the factory cruise
    self.assertTrue(self.safety.get_cruise_engaged_prev())

  def test_factory_cruise_off_keeps_controls(self):
    # openpilot cancels the factory cruise on purpose; that must not drop pedal-long
    self.safety.set_controls_allowed(True)
    self._rx(self._pcm_status_msg(False))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_set_resume_buttons(self):
    HyundaiLongitudinalBase.test_set_resume_buttons(self)

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
      self.assertEqual(not pressed, self._tx(self._interceptor_gas_cmd(1)))
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
        self.assertFalse(self._tx(self._interceptor_gas_cmd(1)))

  def test_pedal_blocked_on_unsupported_cars(self):
    sp = self._pedal_sp()
    for param in (HyundaiSafetyFlags.EV_GAS, HyundaiSafetyFlags.HYBRID_GAS, HyundaiSafetyFlags.FCEV_GAS,
                  HyundaiSafetyFlags.CAMERA_SCC):
      with self.subTest(param=param):
        self.safety.set_current_safety_param_sp(sp)
        self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, param)
        self.safety.set_controls_allowed(True)
        self.assertFalse(self._tx(self._interceptor_gas_cmd(1)))

    # stock HKG (SCC-replacement) longitudinal and the pedal are mutually exclusive. The LONG param is only honored in
    # ALLOW_DEBUG builds; a release panda ignores it (hyundai_longitudinal=false), in which case pedal-long applies.
    with self.subTest(param=HyundaiSafetyFlags.LONG):
      self.safety.set_current_safety_param_sp(sp)
      self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, HyundaiSafetyFlags.LONG)
      self.safety.set_controls_allowed(True)
      stock_long_active = self._tx(common.make_msg(0, 0x50A, 8))  # SCC13: only in the SCC-replacement TX allowlist
      self.assertEqual(not stock_long_active, self._tx(self._interceptor_gas_cmd(1)))
    for mode in (CarParams.SafetyModel.hyundaiLegacy, CarParams.SafetyModel.hyundaiCanfd):
      with self.subTest(mode=mode):
        self.safety.set_current_safety_param_sp(sp)
        self.safety.set_safety_hooks(mode, 0)
        self.safety.set_controls_allowed(True)
        self.assertFalse(self._tx(self._interceptor_gas_cmd(1)))

  def test_no_scc_actuation_allowed(self):
    # pedal-long must not unlock the SCC-replacement TX set (SCC11/12/13/14, FRT_RADAR11, FCA11/12, radar UDS)
    self.safety.set_controls_allowed(True)
    for addr in (0x420, 0x421, 0x50A, 0x389, 0x4A2, 0x38D, 0x483, 0x7D0):
      self.assertFalse(self._tx(common.make_msg(0, addr, 8)), f"{addr=:#x}")


@parameterized_class(LDA_BUTTON)
class TestHyundaiNonSCCFCA11BrakeTestSafety(TestHyundaiNonSCCSafety):
  """
    TEST-ONLY FCA11 (0x38D) brake-injection mode on a non-SCC ICE car (HyundaiSafetyFlagsSP.FCA11_BRAKE_TEST).
    TX = base + FCA11 with check_relay=true: the stock camera FCA11 (bus 2) is no longer forwarded to bus 0, and a
    bus-0 FCA11 would be a relay malfunction (on the target car 0x38D is only ever received on bus 2, per route logs).
  """
  TX_MSGS = [[0x340, 0], [0x4F1, 0], [0x485, 0], [0x38D, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x340, 0x485, 0x38D)}  # LKAS11, LFAHDA_MFC, FCA11
  FWD_BLACKLISTED_ADDRS = {2: [0x340, 0x485, 0x38D]}

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

  def _fca11(self, dec=0, prefill=0, hba=0, cmd_act=0, dec_cmd_act=0, stop_req=0):
    # dec is RAW CR_VSM_DecCmd (0.01 g/LSB); +0.25 LSB so float scaling can't truncate n*0.01/0.01 down to n-1
    values = {"CR_VSM_DecCmd": (dec + 0.25) * 0.01 if dec else 0, "CF_VSM_Prefill": prefill, "CF_VSM_HBACmd": hba, "FCA_CmdAct": cmd_act,
              "CF_VSM_DecCmdAct": dec_cmd_act, "FCA_StopReq": stop_req, "FCA_Status": 2}
    return self.packer.make_can_msg_safety("FCA11", 0, values)

  def _brake_cmd(self, dec):
    return self._fca11(dec=dec, prefill=1, cmd_act=1, dec_cmd_act=1)

  def test_fca11_helper_raw_decel(self):
    # guards the helper: dec maps to the raw byte the panda reads (data[1])
    for dec in (0, 1, HYUNDAI_FCA11_TEST_MAX_DEC, HYUNDAI_FCA11_TEST_MAX_DEC + 1, 255):
      msg = self._fca11(dec=dec)  # keep the owning cffi pointer alive while reading it
      self.assertEqual(dec, msg[0].data[1])

  def test_c_python_cap_match(self):
    # boundary is the C HYUNDAI_FCA11_TEST_MAX_DEC: fails unless it equals the Python mirror
    self.assertTrue(self._tx(self._brake_cmd(HYUNDAI_FCA11_TEST_MAX_DEC)))
    self.assertFalse(self._tx(self._brake_cmd(HYUNDAI_FCA11_TEST_MAX_DEC + 1)))

  def test_decel_zero_allowed(self):
    self.assertTrue(self._tx(self._fca11()))

  def test_decel_within_cap_allowed(self):
    # independent of controls_allowed: openpilot is disengaged during the parked test
    for controls_allowed in (False, True):
      self.safety.set_controls_allowed(controls_allowed)
      for dec in range(HYUNDAI_FCA11_TEST_MAX_DEC + 1):
        self.assertTrue(self._tx(self._fca11(dec=dec)), dec)
        self.assertTrue(self._tx(self._brake_cmd(dec)), dec)
      for sig in ("prefill", "cmd_act", "dec_cmd_act"):
        self.assertTrue(self._tx(self._fca11(**{sig: 1})), sig)

  def test_decel_over_cap_blocked(self):
    for dec in range(HYUNDAI_FCA11_TEST_MAX_DEC + 1, 256):
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

  def test_test_bit_overrides_gas_interceptor(self):
    for dialect in (0, HyundaiSafetyFlagsSP.GAS_INTERCEPTOR_REMAPPED):
      self.safety.set_current_safety_param_sp(self._test_sp() | HyundaiSafetyFlagsSP.GAS_INTERCEPTOR | dialect)
      self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, 0)
      self.safety.set_controls_allowed(True)
      self.assertTrue(self._tx(self._brake_cmd(1)))
      self.assertFalse(self._tx(common.make_msg(0, 0x200, 6)))
      self.assertFalse(self._tx(common.make_msg(0, 0x700, 6)))
      self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x38D))

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


if __name__ == "__main__":
  unittest.main()
