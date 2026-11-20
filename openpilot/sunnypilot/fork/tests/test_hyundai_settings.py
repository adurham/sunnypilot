"""Tests for the 0046 Hyundai settings consolidation: one mode, one shaping key, two one-time shims.

The pure module is headless, so the important parts (migration, precedence, the single car-layer feed)
run here without a UI.  The panel checks are static (AST/source) because importing a raylib page in a
test process that already loaded msgq segfaults.
"""
import unittest
from pathlib import Path

from openpilot.common.params import UnknownKeyName
from openpilot.sunnypilot.fork import hyundai_settings as hs
from openpilot.sunnypilot.selfdrive.car.interfaces import initialize_params

REPO = Path(__file__).resolve().parents[4]
KEYS_SRC = REPO / "openpilot" / "common" / "params_keys.h"


class FakeParams:
  """Minimal Params: dict-backed, same get()/put() contract used by the fork code."""

  def __init__(self, values=None, unknown=()):
    self.values = dict(values or {})
    self.unknown = set(unknown)
    self.writes = []

  def get(self, key, block=False, return_default=False):
    if key in self.unknown:
      raise UnknownKeyName(key)
    if key in self.values:
      return self.values[key]
    return None

  def put(self, key, value, block=False):
    if key in self.unknown:
      raise UnknownKeyName(key)
    self.values[key] = value
    self.writes.append((key, value))


class TestModeMapping(unittest.TestCase):

  def test_legacy_pair_to_mode(self):
    self.assertEqual(hs.MODE_STOCK, hs.mode_from_legacy("0", "0"))
    self.assertEqual(hs.MODE_PEDAL, hs.mode_from_legacy("1", "0"))
    self.assertEqual(hs.MODE_PEDAL_FCA11, hs.mode_from_legacy("1", "1"))
    # (0, 1) is the old inert combination: braking without the pedal never actuated anything.  It
    # maps to Stock, which keeps the car's behaviour identical.
    self.assertEqual(hs.MODE_STOCK, hs.mode_from_legacy("0", "1"))
    self.assertEqual(hs.MODE_STOCK, hs.mode_from_legacy(None, None))

  def test_new_key_wins(self):
    p = FakeParams({hs.MODE_KEY: "1", hs.LEGACY_PEDAL_KEY: "1", hs.LEGACY_BRAKE_KEY: "1"})
    mode, source = hs.resolve_mode(p)
    self.assertEqual((hs.MODE_PEDAL, hs.SOURCE_NEW), (mode, source))

  def test_invalid_new_key_falls_back_to_legacy_not_to_stock(self):
    p = FakeParams({hs.MODE_KEY: "9", hs.LEGACY_PEDAL_KEY: "1", hs.LEGACY_BRAKE_KEY: "1"})
    self.assertEqual(hs.MODE_PEDAL_FCA11, hs.resolve_mode(p)[0])

  def test_stale_libparams_does_not_raise(self):
    p = FakeParams({hs.LEGACY_PEDAL_KEY: "1", hs.LEGACY_BRAKE_KEY: "1"}, unknown={hs.MODE_KEY, hs.SHAPING_KEY})
    self.assertEqual(hs.MODE_PEDAL_FCA11, hs.resolve_mode(p)[0])
    self.assertTrue(hs.resolve_shaping(FakeParams({hs.LEGACY_SHAPING_KEY: "1"}, unknown={hs.SHAPING_KEY})))

  def test_modes_to_car_layer_pair(self):
    want = {
      hs.MODE_STOCK: (0, 0),
      hs.MODE_PEDAL: (1, 0),
      hs.MODE_PEDAL_FCA11: (1, 1),
    }
    for mode, (pedal, brake) in want.items():
      state = hs.HyundaiSettingsState(mode, mode >= hs.MODE_PEDAL, mode == hs.MODE_PEDAL_FCA11, False, hs.SOURCE_NEW)
      over = hs.car_layer_overrides(state)
      self.assertEqual((pedal, brake), (over[hs.LEGACY_PEDAL_KEY], over[hs.LEGACY_BRAKE_KEY]), mode)


class TestMigration(unittest.TestCase):

  def test_mode_migrates_once_from_the_owner_pair(self):
    p = FakeParams({hs.LEGACY_PEDAL_KEY: "1", hs.LEGACY_BRAKE_KEY: "1"})
    self.assertTrue(hs.migrate_mode(p))
    self.assertEqual(2, p.values[hs.MODE_KEY])
    self.assertEqual([(hs.MODE_KEY, 2)], p.writes)
    # Second call is a no-op, even if the legacy pair changes afterwards.
    p.values[hs.LEGACY_BRAKE_KEY] = "0"
    self.assertFalse(hs.migrate_mode(p))
    self.assertEqual(2, p.values[hs.MODE_KEY])
    self.assertEqual(1, len(p.writes))

  def test_mode_migration_never_overwrites_the_new_key(self):
    p = FakeParams({hs.MODE_KEY: "1", hs.LEGACY_PEDAL_KEY: "1", hs.LEGACY_BRAKE_KEY: "1"})
    self.assertFalse(hs.migrate_mode(p))
    self.assertEqual("1", p.values[hs.MODE_KEY])
    self.assertEqual([], p.writes)

  def test_mode_migration_is_silent_on_a_fresh_device(self):
    p = FakeParams()
    self.assertFalse(hs.migrate_mode(p))
    self.assertEqual([], p.writes)
    self.assertEqual(hs.MODE_STOCK, hs.resolve_mode(p)[0])

  def test_inert_legacy_brake_migrates_to_stock(self):
    p = FakeParams({hs.LEGACY_PEDAL_KEY: "0", hs.LEGACY_BRAKE_KEY: "1"})
    self.assertTrue(hs.migrate_mode(p))
    self.assertEqual(hs.MODE_STOCK, p.values[hs.MODE_KEY])
    self.assertEqual("1", p.values[hs.LEGACY_BRAKE_KEY])   # old value stays readable

  def test_shaping_migrates_once_and_old_key_stays(self):
    p = FakeParams({hs.LEGACY_SHAPING_KEY: "1"})
    self.assertTrue(hs.migrate_shaping(p))
    self.assertEqual(True, p.values[hs.SHAPING_KEY])
    self.assertEqual("1", p.values[hs.LEGACY_SHAPING_KEY])
    p.values[hs.LEGACY_SHAPING_KEY] = "0"
    self.assertFalse(hs.migrate_shaping(p))
    self.assertEqual(1, p.values[hs.SHAPING_KEY])

  def test_shaping_default_is_unchanged(self):
    self.assertFalse(hs.resolve_shaping(FakeParams()))
    self.assertFalse(hs.resolve_shaping(FakeParams({hs.LEGACY_SHAPING_KEY: "0"})))
    self.assertTrue(hs.resolve_shaping(FakeParams({hs.SHAPING_KEY: "1"})))


class TestSingleFeed(unittest.TestCase):
  """initialize_params() is the only feed into opendbc setup_interfaces; the resolved mode must hand
  the car layer exactly the pair the legacy keys handed it before 0046."""

  @staticmethod
  def _feed(values):
    pl = initialize_params(FakeParams(values))
    out = {}
    for d in pl:
      for k in (hs.LEGACY_PEDAL_KEY, hs.LEGACY_BRAKE_KEY, hs.LEGACY_SHAPING_KEY):
        if k in d:
          out[k] = d[k]
    return out

  def test_legacy_and_migrated_mode_feeds_match(self):
    for pedal, brake in (("0", "0"), ("1", "0"), ("1", "1")):
      old = self._feed({hs.LEGACY_PEDAL_KEY: pedal, hs.LEGACY_BRAKE_KEY: brake})
      mode = hs.mode_from_legacy(pedal, brake)
      new = self._feed({hs.MODE_KEY: str(mode)})
      self.assertEqual(old, new, (pedal, brake))

  def test_inert_legacy_brake_is_effectively_identical(self):
    old = self._feed({hs.LEGACY_PEDAL_KEY: "0", hs.LEGACY_BRAKE_KEY: "1"})
    new = self._feed({hs.MODE_KEY: "0"})
    # Braking without the pedal is never read by opendbc (_initialize_hyundai_gas_interceptor returns
    # first), so the effective pair is identical even though the raw brake key differs.
    self.assertEqual(0, old[hs.LEGACY_PEDAL_KEY])
    self.assertEqual(old[hs.LEGACY_PEDAL_KEY], new[hs.LEGACY_PEDAL_KEY])
    self.assertEqual(0, new[hs.LEGACY_BRAKE_KEY])

  def test_shaping_feed_matches_old_key(self):
    old = self._feed({hs.LEGACY_SHAPING_KEY: "1"})
    new = self._feed({hs.SHAPING_KEY: "1"})
    self.assertEqual(old[hs.LEGACY_SHAPING_KEY], new[hs.LEGACY_SHAPING_KEY])

  def test_feed_never_turns_a_default_on(self):
    feed = self._feed({})
    self.assertEqual(0, feed[hs.LEGACY_PEDAL_KEY])
    self.assertEqual(0, feed[hs.LEGACY_BRAKE_KEY])
    self.assertEqual(0, feed[hs.LEGACY_SHAPING_KEY])


class TestRegistry(unittest.TestCase):

  def test_new_keys_registered_with_old_defaults(self):
    src = KEYS_SRC.read_text(encoding="utf-8")
    self.assertRegex(src, r'\{"HyundaiLongitudinalMode", \{PERSISTENT \| BACKUP, INT, "0"\}\}')
    self.assertRegex(src, r'\{"HyundaiFca11Shaping", \{PERSISTENT \| BACKUP, BOOL, "0"\}\}')

  def test_legacy_keys_stay_registered_and_readable(self):
    src = KEYS_SRC.read_text(encoding="utf-8")
    for key in (hs.LEGACY_PEDAL_KEY, hs.LEGACY_BRAKE_KEY, hs.LEGACY_SHAPING_KEY):
      self.assertRegex(src, r'\{"' + key + r'", \{PERSISTENT \| BACKUP, (BOOL|STRING), "[^"]+"\}\}')


if __name__ == "__main__":
  unittest.main()
