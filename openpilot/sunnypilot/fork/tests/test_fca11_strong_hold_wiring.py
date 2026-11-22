"""0057: superproject wiring of "Stronger Hold At Stops" (HyundaiFca11StrongHold, TEST SETTING, INT, default 0 = OFF).

The car-layer behaviour is tested in opendbc (patch 0057: tests/test_fca11_strong_hold.py). This file pins the
superproject half the car layer cannot see: the key is registered as INT "0", initialize_params() feeds it (and a stale
libparams cannot raise), the REAL chain initialize_params -> opendbc setup_interfaces -> CarParamsSP ->
helpers.convert_to_capnp (the exact call card makes at ignition) carries it into the logged CarParamsSP for 0/1/2 and
OFF for anything else, and the settings row writes the same key with a guarded write and reads it fail-closed.
"""
import re
import unittest
from pathlib import Path

from openpilot.common.params import UnknownKeyName
from openpilot.selfdrive.car import helpers
from openpilot.sunnypilot.selfdrive.car.interfaces import initialize_params

REPO = Path(__file__).resolve().parents[4]
KEY = "HyundaiFca11StrongHold"
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


def _cp_sp(value, extra=None):
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
  vals = {"HyundaiLongitudinalMode": "2", "HyundaiFca11Shaping": "1", "HyundaiLongitudinalTuning": "0",
          "HyundaiFca11DynamicBrakes": "1", KEY: value}
  vals.update(extra or {})
  setup_interfaces(CarInterface, CP, CP_SP, initialize_params(FakeParams(vals)))
  return CarInterface, CP, CP_SP


class TestRegistry(unittest.TestCase):

  def test_registered_int_default_off(self):
    src = KEYS_SRC.read_text(encoding="utf-8")
    self.assertRegex(src, r'\{"' + KEY + r'", \{PERSISTENT \| BACKUP, INT, "0"\}\}')


class TestFeed(unittest.TestCase):

  def test_fed_as_stored_and_absent_is_none(self):
    self.assertIn(KEY, _feed({}))
    self.assertIsNone(_feed({})[KEY])
    self.assertEqual(2, _feed({KEY: 2})[KEY])

  def test_stale_libparams_does_not_raise(self):
    self.assertNotIn(KEY, _feed({}, unknown=(KEY,)))


class TestRealChainToCapnp(unittest.TestCase):

  def test_modes_reach_the_logged_carparamssp_and_the_car_layer(self):
    for raw, want in ((1, 1), ("1", 1), (2, 2), ("2", 2)):
      with self.subTest(raw=raw):
        CarInterface, CP, cp = _cp_sp(raw)
        self.assertEqual(cp.fca11StrongHold, want)
        self.assertEqual(helpers.convert_to_capnp(cp).fca11StrongHold, want)   # the arm marker in every rlog
        self.assertEqual(CarInterface(CP, cp).CC.fca11_brake.sh_mode, want)  # the car layer reads it

  def test_default_and_junk_are_off(self):
    for raw in (None, 0, "0", "junk", 3, "-1", False, True):
      with self.subTest(raw=raw):
        CarInterface, CP, cp = _cp_sp(raw)
        self.assertEqual(cp.fca11StrongHold, 0)
        self.assertEqual(helpers.convert_to_capnp(cp).fca11StrongHold, 0)
        self.assertEqual(CarInterface(CP, cp).CC.fca11_brake.sh_mode, 0)

  def test_inert_without_the_continuous_law_or_in_shadow(self):
    CarInterface, CP, cp = _cp_sp(1, {"HyundaiFca11DynamicBrakes": "0"})
    self.assertEqual(CarInterface(CP, cp).CC.fca11_brake.sh_mode, 0)
    _, _, cp = _cp_sp(1, {"HyundaiFca11Shadow": "1"})
    self.assertEqual(cp.fca11StrongHold, 0)


class TestPanel(unittest.TestCase):

  def test_row_is_listed_guarded_and_fail_closed(self):
    src = PANEL.read_text(encoding="utf-8")
    self.assertIn(f'STRONG_HOLD_KEY = "{KEY}"', src)
    self.assertIn('"Stronger Hold At Stops"', src)
    self.assertIn('STRONG_HOLD_LABELS = ("Off", "On", "Parked test")', src)
    m = re.search(r"self\.items = \[(.*?)\]", src, re.S)
    self.assertIn("self.fca11_strong_hold_item", m.group(1) if m else "")
    # no param= on the row (a stale libparams would raise in the widget's constructor); the write is guarded
    row = src[src.index("self.fca11_strong_hold_item = multiple_button_item_sp("):]
    self.assertNotIn("param=", row[:row.index(")\n")])
    self.assertRegex(src, r"def _on_strong_hold_selected\(index\):\n(.*\n){0,4}\s+try:\n\s+ui_state\.params\.put\(STRONG_HOLD_KEY")

  def test_index_parse(self):
    import importlib.util
    spec = importlib.util.spec_from_file_location("_hy_panel_src", PANEL)
    src = PANEL.read_text(encoding="utf-8")
    fn = src[src.index("def strong_hold_index(raw) -> int:"):src.index("def _read_strong_hold():")]
    ns = {}
    exec(fn, ns)
    del spec
    for raw, want in ((None, 0), (0, 0), (1, 1), (2, 2), (b"2", 2), ("1", 1), (3, 0), ("junk", 0), (True, 0)):
      self.assertEqual(ns["strong_hold_index"](raw), want, raw)


if __name__ == "__main__":
  unittest.main()
