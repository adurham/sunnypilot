"""0056: superproject wiring of the FCA11 standstill clamp (HyundaiFca11StandstillClamp, TEST SETTING, default OFF).

The car-layer behaviour is tested in opendbc (patch 0056: tests/test_fca11_standstill_clamp.py). This file pins the
superproject half the car layer cannot see: the key is registered with a BOOL "0" default, initialize_params() feeds it
(and a stale libparams cannot raise), the REAL chain initialize_params -> opendbc setup_interfaces -> CarParamsSP ->
helpers.convert_to_capnp (the exact call card makes at ignition: the 0041 crash class) carries it into the logged
CarParamsSP, and the settings row writes the same key.
"""
import re
import unittest
from pathlib import Path

from openpilot.common.params import UnknownKeyName
from openpilot.selfdrive.car import helpers
from openpilot.sunnypilot.selfdrive.car.interfaces import initialize_params

REPO = Path(__file__).resolve().parents[4]
KEY = "HyundaiFca11StandstillClamp"
KEYS_SRC = REPO / "openpilot" / "common" / "params_keys.h"
PANEL = REPO / "openpilot" / "selfdrive" / "ui" / "sunnypilot" / "layouts" / "settings" / "vehicle" / "brands" / "hyundai.py"


class FakeParams:
  def __init__(self, values=None, unknown=()):
    self.values = dict(values or {})
    self.unknown = set(unknown)

  def get(self, key, block=False, return_default=False):
    if key in self.unknown:
      raise UnknownKeyName(key)
    return self.values.get(key)

  def put(self, key, value, block=False):
    if key in self.unknown:
      raise UnknownKeyName(key)
    self.values[key] = value


def _feed(values, unknown=()):
  out = {}
  for d in initialize_params(FakeParams(values, unknown)):
    out.update(d)
  return out


def _cp_sp(clamp_value):
  """The real chain the car runs at ignition, on the car's own platform, from a params feed."""
  from opendbc.car.car_helpers import interfaces
  from opendbc.car.hyundai.values import CAR
  from opendbc.sunnypilot.car.interfaces import setup_interfaces
  from opendbc.sunnypilot.car.hyundai.tests.test_gas_interceptor import BASE_FINGERPRINT, STD as PEDAL
  car = CAR.HYUNDAI_ELANTRA_2022_NON_SCC
  fp = {i: {} for i in range(8)}
  fp[0] = {**BASE_FINGERPRINT, **PEDAL}
  CarInterface = interfaces[car]
  CP = CarInterface.get_params(car, fp, [], alpha_long=False, is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, car, fp, [], alpha_long=False, is_release_sp=False, docs=False)
  feed = initialize_params(FakeParams({"HyundaiLongitudinalMode": "2", "HyundaiFca11Shaping": "1", "HyundaiLongitudinalTuning": "0",
                                       "HyundaiFca11DynamicBrakes": "1", KEY: clamp_value}))
  setup_interfaces(CarInterface, CP, CP_SP, feed)
  return CP_SP


class TestRegistry(unittest.TestCase):

  def test_registered_bool_default_off(self):
    src = KEYS_SRC.read_text(encoding="utf-8")
    self.assertRegex(src, r'\{"' + KEY + r'", \{PERSISTENT \| BACKUP, BOOL, "0"\}\}')


class TestFeed(unittest.TestCase):

  def test_fed_as_stored_and_absent_is_none(self):
    self.assertIn(KEY, _feed({}))
    self.assertIsNone(_feed({})[KEY])
    self.assertEqual("1", _feed({KEY: "1"})[KEY])

  def test_stale_libparams_does_not_raise(self):
    self.assertNotIn(KEY, _feed({}, unknown=(KEY,)))


class TestRealChainToCapnp(unittest.TestCase):

  def test_on_reaches_the_logged_carparamssp(self):
    cp = _cp_sp("1")
    self.assertTrue(cp.fca11StandstillClamp)
    msg = helpers.convert_to_capnp(cp)          # card.py serializes this at every ignition
    self.assertTrue(msg.fca11StandstillClamp)   # ... and it is the A/B arm marker in every rlog

  def test_default_and_junk_are_off(self):
    for raw in (None, "0", "junk", False):
      with self.subTest(raw=raw):
        cp = _cp_sp(raw)
        self.assertFalse(cp.fca11StandstillClamp)
        self.assertFalse(helpers.convert_to_capnp(cp).fca11StandstillClamp)


class TestPanel(unittest.TestCase):

  def test_row_writes_the_key_and_is_listed(self):
    src = PANEL.read_text(encoding="utf-8")
    self.assertIn(f'param="{KEY}"', src)
    m = re.search(r"self\.items = \[(.*?)\]", src, re.S)
    self.assertIn("self.fca11_standstill_clamp_toggle", m.group(1) if m else "")
    self.assertIn("(self.fca11_standstill_clamp_toggle, self._clamp_desc)", src)   # same enable rule as its siblings


if __name__ == "__main__":
  unittest.main()
