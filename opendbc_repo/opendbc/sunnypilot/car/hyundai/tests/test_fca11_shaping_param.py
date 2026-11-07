"""
0044: the FCA11 brake-SHAPING param plumbing, opendbc side (HyundaiFca11AffineGain -> CP_SP.fca11AffineGain).

Before 0044 NOTHING wrote CP_SP.fca11AffineGain and the key was not registered in params_keys.h, so the shaping law
could not be turned on on the car; the sim reached it through the FCA11_AFFINE env override, which hid the gap. These
tests pin the opendbc half of the fix (the superproject half is test_fca11_param_plumbing.py, real Params path):
  * setup_interfaces reads the key from the SAME params_dict with the SAME truthy parser as HyundaiFca11Brake;
  * the DEPENDENCY: fca11AffineGain is set ONLY when fca11Brake is set (a persisted ON with braking off is inert);
  * the dataclass field defaults to False and an absent key never raises;
  * the env override is refused at Fca11LongBrake construction.
"""
import os
import unittest

from opendbc.car import structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.hyundai.values import CAR
from opendbc.sunnypilot.car import interfaces as sp_ifaces
from opendbc.sunnypilot.car.hyundai import fca11_long as fl
from opendbc.sunnypilot.car.hyundai.values import HyundaiSafetyFlagsSP
from opendbc.sunnypilot.car.interfaces import setup_interfaces

CAR_UNDER_TEST = CAR.HYUNDAI_ELANTRA_2022_NON_SCC
BASE_FINGERPRINT = {0x260: 8, 0x371: 8, 0x386: 8, 0x394: 8, 0x251: 8, 0x4F1: 4, 0x340: 8}
STD = {0x201: 6}

# (raw value as it can reach params_dict, expected truthiness). True/"1"/b"1" (whitespace-tolerant) ON; all else OFF.
TRUTHY_TABLE = [
  (True, True), ("1", True), (b"1", True), (" 1 ", True), ("1\n", True),
  (False, False), ("0", False), (b"0", False), (None, False), ("", False), (b"", False),
  ("true", False), ("True", False), ("yes", False), ("on", False), ("2", False), ("01", False), ("junk", False),
]


def _cp(params_list):
  fingerprint = {i: {} for i in range(8)}
  fingerprint[0] = {**BASE_FINGERPRINT, **STD}
  CarInterface = interfaces[CAR_UNDER_TEST]
  CP = CarInterface.get_params(CAR_UNDER_TEST, fingerprint, [], alpha_long=False, is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, CAR_UNDER_TEST, fingerprint, [], alpha_long=False, is_release_sp=False, docs=False)
  setup_interfaces(CarInterface, CP, CP_SP, params_list)
  return CP, CP_SP


def _plist(brake, shaping):
  pl = [{"HyundaiGasInterceptor": True}]
  if brake is not None:
    pl.append({"HyundaiFca11Brake": brake})
  if shaping is not None:
    pl.append({"HyundaiFca11AffineGain": shaping})
  return pl


class TestShapingParamPlumbing(unittest.TestCase):

  def setUp(self):
    self._env_prev = os.environ.pop("FCA11_AFFINE", None)

  def tearDown(self):
    os.environ.pop("FCA11_AFFINE", None)
    if self._env_prev is not None:
      os.environ["FCA11_AFFINE"] = self._env_prev

  def test_field_default_false(self):
    self.assertFalse(structs.CarParamsSP().fca11AffineGain)
    _, CP_SP = _cp([])
    self.assertFalse(CP_SP.fca11AffineGain)

  def test_truthy_parser_table_is_shared_by_both_keys(self):
    """The SAME parser for HyundaiFca11Brake and HyundaiFca11AffineGain (the 0018 expression, factored out)."""
    for raw, want in TRUTHY_TABLE:
      self.assertEqual(want, sp_ifaces._param_truthy(raw), repr(raw))
      _, CP_SP = _cp(_plist(raw, raw))
      self.assertEqual(want, CP_SP.fca11Brake, ("brake", raw))
      self.assertEqual(want, CP_SP.fca11AffineGain, ("shaping", raw))

  def test_shaping_parser_table_with_braking_on(self):
    for raw, want in TRUTHY_TABLE:
      _, CP_SP = _cp(_plist(True, raw))
      self.assertTrue(CP_SP.fca11Brake)
      self.assertEqual(want, CP_SP.fca11AffineGain, repr(raw))

  def test_dependency_enforced_in_the_car_layer(self):
    """A persisted shaping ON with FCA11 Braking OFF/absent/junk is INERT; braking ON + shaping ON is the only ON."""
    for brake in (False, "0", None, "junk"):
      for shaping in (True, "1", b"1"):
        _, CP_SP = _cp(_plist(brake, shaping))
        self.assertFalse(CP_SP.fca11Brake, (brake, shaping))
        self.assertFalse(CP_SP.fca11AffineGain, (brake, shaping))
        self.assertFalse(CP_SP.safetyParam & HyundaiSafetyFlagsSP.FCA11_LONG, (brake, shaping))
    _, CP_SP = _cp(_plist(True, True))
    self.assertTrue(CP_SP.fca11Brake)
    self.assertTrue(CP_SP.fca11AffineGain)

  def test_dependency_needs_the_pedal(self):
    """No pedal param -> no FCA11 braking -> no shaping (the whole block is inside the pedal arm)."""
    _, CP_SP = _cp([{"HyundaiFca11Brake": True}, {"HyundaiFca11AffineGain": True}])
    self.assertFalse(CP_SP.fca11Brake)
    self.assertFalse(CP_SP.fca11AffineGain)

  def test_shaping_never_touches_safety_param(self):
    """The shaping flag is a car-layer law choice: it never changes safetyParam (bit 256 rides on fca11Brake only)."""
    _, off = _cp(_plist(True, False))
    _, on = _cp(_plist(True, True))
    self.assertEqual(off.safetyParam, on.safetyParam)
    self.assertTrue(on.safetyParam & HyundaiSafetyFlagsSP.FCA11_LONG)

  def test_absent_key_never_raises_and_is_off(self):
    _, CP_SP = _cp(_plist(True, None))
    self.assertTrue(CP_SP.fca11Brake)
    self.assertFalse(CP_SP.fca11AffineGain)

  def test_env_override_raises(self):
    _, CP_SP = _cp(_plist(True, False))
    for tok in ("1", "0", ""):
      try:
        os.environ["FCA11_AFFINE"] = tok
        with self.assertRaises(RuntimeError):
          fl.Fca11LongBrake(structs.CarParams(), CP_SP)
      finally:
        os.environ.pop("FCA11_AFFINE", None)

  def test_controller_follows_the_written_field(self):
    """setup_interfaces -> CP_SP -> Fca11LongBrake.affine_gain, both directions (no hand-built CP_SP)."""
    for shaping, want in ((True, True), (False, False), (None, False)):
      _, CP_SP = _cp(_plist(True, shaping))
      self.assertEqual(want, fl.Fca11LongBrake(structs.CarParams(), CP_SP).affine_gain, shaping)


if __name__ == "__main__":
  unittest.main()
