"""
fork: FCA11-long param plumbing, END TO END through the REAL path card.py takes.

card.py:113 does `sunnypilot_interfaces.initialize_params(self.params)` and hands the result to
opendbc's `get_car(...)` -> `setup_interfaces(...)`, which is the ONLY place `CP_SP.fca11Brake` and
panda safety bit 256 (`FCA11_LONG`) are set. The pre-existing car-layer tests inject
{'HyundaiFca11Brake': ...} straight into `params_list`, so they never exercise `initialize_params()`
and missed the bug where the key was simply not read there (feature came up OFF on route 149
even though the UI/param said ON).

These tests drive a real `Params` object, the real `initialize_params()`, and the real
`setup_interfaces()`, and assert:
  * the key reaches opendbc exactly when the stored value is 1 (and, defensively, at all);
  * CP_SP.fca11Brake AND the safetyParam FCA11_LONG bit are set for '1', and clear for '0'/absent;
  * a raw string '1' (libparams form) still arms it (the bool/str type tolerance);
  * nothing else about the pedal-only baseline changes when the toggle is off.
"""
import os
import tempfile
import unittest

# Isolated params store. PARAMS_ROOT is honoured by libparams at Params() construction, so this must
# be set before any Params() is built in this process (openpilot's common/params fallback does not).
os.environ["PARAMS_ROOT"] = tempfile.mkdtemp(prefix="fca11-plumbing-test-")

from openpilot.common.params import Params  # noqa: E402
from openpilot.common.test import OpenpilotTestCase  # noqa: E402
from openpilot.sunnypilot.selfdrive.car import interfaces as sp_interfaces  # noqa: E402
from opendbc.car.car_helpers import interfaces  # noqa: E402
from opendbc.car.hyundai.values import CAR  # noqa: E402
from opendbc.sunnypilot.car.hyundai.values import HyundaiSafetyFlagsSP  # noqa: E402
from opendbc.sunnypilot.car.interfaces import setup_interfaces  # noqa: E402

CAR_UNDER_TEST = CAR.HYUNDAI_ELANTRA_2022_NON_SCC
# standard pedal sensor 0x201 present so the gas-interceptor capability flag is set (FCA11 rides on it)
BASE_FINGERPRINT = {0x260: 8, 0x371: 8, 0x386: 8, 0x394: 8, 0x251: 8, 0x4F1: 4, 0x340: 8}
STD = {0x201: 6}
FCA11_BIT = HyundaiSafetyFlagsSP.FCA11_LONG
GAS_BIT = HyundaiSafetyFlagsSP.GAS_INTERCEPTOR


def _run_real_path(fca11):
  """Real Params -> initialize_params() -> setup_interfaces(). fca11 is True / False / None (absent)."""
  params = Params()
  params.remove("HyundaiFca11Brake")
  params.remove("HyundaiGasInterceptor")
  params.put_bool("HyundaiGasInterceptor", True, block=True)
  if fca11 is not None:
    params.put_bool("HyundaiFca11Brake", fca11, block=True)

  init_params_list_sp = sp_interfaces.initialize_params(params)

  fingerprint = {i: {} for i in range(8)}
  fingerprint[0] = {**BASE_FINGERPRINT, **STD}
  CarInterface = interfaces[CAR_UNDER_TEST]
  CP = CarInterface.get_params(CAR_UNDER_TEST, fingerprint, [], alpha_long=False, is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, CAR_UNDER_TEST, fingerprint, [], alpha_long=False, is_release_sp=False, docs=False)
  setup_interfaces(CarInterface, CP, CP_SP, init_params_list_sp)
  return CP, CP_SP, init_params_list_sp


class TestFca11ParamPlumbingE2E(OpenpilotTestCase):

  def test_initialize_params_reads_the_key(self):
    # the plumbing bug: initialize_params() must emit the key at all (it did not before this fix)
    for setting in (True, False, None):
      _, _, plist = _run_real_path(setting)
      self.assertTrue(any("HyundaiFca11Brake" in d for d in plist),
                      f"initialize_params() did not read HyundaiFca11Brake (setting={setting})")

  def test_toggle_on_arms_through_the_real_path(self):
    _, CP_SP, _ = _run_real_path(True)
    self.assertTrue(CP_SP.fca11Brake, "CP_SP.fca11Brake must be True when the stored param is 1")
    self.assertTrue(CP_SP.safetyParam & FCA11_BIT, "safetyParam FCA11_LONG bit (256) must be set")
    # the pedal it rides on is armed too
    self.assertTrue(CP_SP.safetyParam & GAS_BIT)

  def test_toggle_off_is_inert_through_the_real_path(self):
    _, CP_SP, _ = _run_real_path(False)
    self.assertFalse(CP_SP.fca11Brake)
    self.assertFalse(CP_SP.safetyParam & FCA11_BIT)

  def test_key_absent_is_inert_through_the_real_path(self):
    # stale libparams / fresh device: the key is unreadable -> default False -> feature off, no raise
    _, CP_SP, _ = _run_real_path(None)
    self.assertFalse(CP_SP.fca11Brake)
    self.assertFalse(CP_SP.safetyParam & FCA11_BIT)

  def test_on_matches_pedal_only_baseline_plus_bit256(self):
    _, CP_SP_off, _ = _run_real_path(False)
    _, CP_SP_on, _ = _run_real_path(True)
    # turning it on changes exactly fca11Brake + bit 256, nothing else in safetyParam
    self.assertEqual(CP_SP_off.safetyParam | FCA11_BIT, CP_SP_on.safetyParam)
    self.assertEqual(CP_SP_off.safetyParam & FCA11_BIT, 0)

  def test_raw_string_one_still_arms(self):
    # guards the str(...) == "1" clause independently of the bool path (libparams C-string form)
    fingerprint = {i: {} for i in range(8)}
    fingerprint[0] = {**BASE_FINGERPRINT, **STD}
    CarInterface = interfaces[CAR_UNDER_TEST]
    CP = CarInterface.get_params(CAR_UNDER_TEST, fingerprint, [], alpha_long=False, is_release=False, docs=False)
    CP_SP = CarInterface.get_params_sp(CP, CAR_UNDER_TEST, fingerprint, [], alpha_long=False, is_release_sp=False, docs=False)
    setup_interfaces(CarInterface, CP, CP_SP, [{"HyundaiGasInterceptor": True}, {"HyundaiFca11Brake": "1"}])
    self.assertTrue(CP_SP.fca11Brake)
    self.assertTrue(CP_SP.safetyParam & FCA11_BIT)


if __name__ == "__main__":
    unittest.main()
