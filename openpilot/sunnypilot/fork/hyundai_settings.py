"""
Fork (0046): one longitudinal model for the Hyundai settings surface.

The owner's 2026-11-10 feedback: the Hyundai page had accumulated eleven peer switches plus a tuning
selector, several of them invisible, misnamed or inert, so the page no longer said what the car does.
This module is the single source of truth behind that page and behind the one feed into opendbc's
``setup_interfaces`` (``openpilot/sunnypilot/selfdrive/car/interfaces.py::initialize_params``).

The model
---------
``HyundaiLongitudinalMode`` (INT, default 0) owns the two mutually-exclusive master switches:

  0  Stock             no comma pedal, no FCA11 braking  (pedal=0, brake=0)
  1  Comma pedal       comma pedal only                   (pedal=1, brake=0)
  2  Pedal + FCA11     comma pedal + FCA11 braking        (pedal=1, brake=1)

Everything else is a modifier on the path the mode selects (pedal delivery, gas rise lag, brake
shaping, dynamic brakes, drive-mode personality) or an experimental/diagnostic row hidden behind the
page's "Show experimental" reveal (stop-hold clutch, shadow logging).  ``HyundaiGasInterceptorIDSet``
stays a safety selector with no row: it is what refuses to command a pedal the car cannot hear.

Migration (nothing changes a default)
-------------------------------------
``HyundaiLongitudinalMode`` is written ONCE when it is absent and the legacy keys exist, from the
legacy values: ``HyundaiGasInterceptor`` / ``HyundaiFca11Brake``.  The owner's car (pedal=1,
brake=1) migrates to mode 2, so the first ignition after the deploy behaves exactly as before.  A
fresh device has neither legacy key and keeps the default mode 0, which is the old default pair
(0, 0).  The legacy keys stay registered and readable for at least one release (``PERSISTENT |
BACKUP``), but once the new key exists it is authoritative; a backup restore of the old keys can no
longer silently re-enable a path the owner did not select.

``HyundaiFca11Shaping`` (BOOL, default 0) replaces the historical misnomer ``HyundaiFca11AffineGain``
as the user-facing brake-shaping key.  The shim reads the old key when the new one is absent, writes
the new key once from it, and keeps the old key registered/readable.  The car layer still receives
the value under the old key name because its dataclass field (``CarParamsSP.fca11AffineGain``,
capnp ordinal @7) is a wire contract that must not move; only the user-facing key is renamed.

No default is changed by this module: every new key defaults to the behaviour the old key had at its
default, and the migration derives the mode/shaping from the values already stored on the device.
"""
from __future__ import annotations

from typing import NamedTuple

MODE_KEY = "HyundaiLongitudinalMode"
SHAPING_KEY = "HyundaiFca11Shaping"

# Legacy keys: registered, readable, and still the names the opendbc car layer consumes through
# initialize_params().  Never delete them in this release.
LEGACY_PEDAL_KEY = "HyundaiGasInterceptor"
LEGACY_BRAKE_KEY = "HyundaiFca11Brake"
LEGACY_SHAPING_KEY = "HyundaiFca11AffineGain"

MODE_STOCK = 0
MODE_PEDAL = 1
MODE_PEDAL_FCA11 = 2
MODES = (MODE_STOCK, MODE_PEDAL, MODE_PEDAL_FCA11)

MODE_LABELS = ("Stock", "Comma pedal", "Pedal + FCA11")

# Source of the resolved mode, for tests and for the page's own sentence.
SOURCE_NEW = "new"
SOURCE_LEGACY = "legacy"
SOURCE_DEFAULT = "default"


class HyundaiSettingsState(NamedTuple):
  mode: int
  pedal: bool
  brake: bool
  shaping: bool
  mode_source: str


def _truthy(raw) -> bool:
  """The fork's BOOL-param truthiness: True / "1" / b"1" are ON; None/False/"0"/junk are OFF."""
  if isinstance(raw, bytes):
    raw = raw.decode(errors="ignore")
  return bool(raw is True or str(raw if raw is not None else "0").strip() == "1")


def _as_int(raw, default: int = 0) -> int:
  try:
    return int(raw)
  except (TypeError, ValueError):
    return default


def mode_from_legacy(pedal_raw, brake_raw) -> int:
  """The one deterministic mapping from the old pair to the mode.  (0, 1) is Stock: braking without
  the pedal is inert in the car layer, so the behaviour is the old behaviour (nothing actuates)."""
  pedal = _truthy(pedal_raw)
  brake = _truthy(brake_raw)
  if pedal and brake:
    return MODE_PEDAL_FCA11
  if pedal:
    return MODE_PEDAL
  return MODE_STOCK


def resolve_mode(params) -> tuple[int, str]:
  """(mode, source).  The new key wins when it holds a valid value; otherwise derive from the legacy
  pair, which is exactly the pre-0046 behaviour.  An invalid value falls back to the legacy pair
  rather than silently selecting Stock."""
  try:
    raw = params.get(MODE_KEY, return_default=False)
  except Exception:
    raw = None
  if raw is not None:
    mode = _as_int(raw, -1)
    if mode in MODES:
      return mode, SOURCE_NEW
  try:
    pedal = params.get(LEGACY_PEDAL_KEY, return_default=False)
  except Exception:
    pedal = None
  try:
    brake = params.get(LEGACY_BRAKE_KEY, return_default=False)
  except Exception:
    brake = None
  if pedal is None and brake is None:
    return MODE_STOCK, SOURCE_DEFAULT
  return mode_from_legacy(pedal, brake), SOURCE_LEGACY


def resolve_shaping(params) -> bool:
  """The new shaping key wins; the historical key is the fallback until the shim has written the new
  one.  Both default OFF, so a fresh device is unchanged."""
  try:
    raw = params.get(SHAPING_KEY, return_default=False)
  except Exception:
    raw = None
  if raw is not None:
    return _truthy(raw)
  try:
    old = params.get(LEGACY_SHAPING_KEY, return_default=False)
  except Exception:
    old = None
  return _truthy(old)


def resolve_state(params) -> HyundaiSettingsState:
  mode, source = resolve_mode(params)
  return HyundaiSettingsState(mode=mode, pedal=mode >= MODE_PEDAL, brake=mode == MODE_PEDAL_FCA11,
                              shaping=resolve_shaping(params), mode_source=source)


def _put(params, key, value) -> bool:
  try:
    params.put(key, value)
    return True
  except Exception:
    return False


def migrate_mode(params) -> bool:
  """Write ``HyundaiLongitudinalMode`` once, from the legacy pair, when the new key is absent.

  Returns True only when a write happened.  Never overwrites an existing new key and never writes on
  a device where neither legacy key exists (that device is on the old defaults, which equal mode 0)."""
  try:
    if params.get(MODE_KEY, return_default=False) is not None:
      return False
  except Exception:
    return False
  try:
    pedal = params.get(LEGACY_PEDAL_KEY, return_default=False)
  except Exception:
    pedal = None
  try:
    brake = params.get(LEGACY_BRAKE_KEY, return_default=False)
  except Exception:
    brake = None
  if pedal is None and brake is None:
    return False
  return _put(params, MODE_KEY, int(mode_from_legacy(pedal, brake)))


def migrate_shaping(params) -> bool:
  """Write ``HyundaiFca11Shaping`` once, from ``HyundaiFca11AffineGain``, when the new key is absent.

  The old key stays registered and readable for at least one release, so a rollback to a pre-0046
  build still finds the value it wrote."""
  try:
    if params.get(SHAPING_KEY, return_default=False) is not None:
      return False
  except Exception:
    return False
  try:
    old = params.get(LEGACY_SHAPING_KEY, return_default=False)
  except Exception:
    old = None
  if old is None:
    return False
  return _put(params, SHAPING_KEY, _truthy(old))


def migrate_hyundai_settings(params) -> bool:
  """Both one-time migrations, in either order.  True when anything was written."""
  wrote_mode = migrate_mode(params)
  wrote_shaping = migrate_shaping(params)
  return wrote_mode or wrote_shaping


def car_layer_overrides(state: HyundaiSettingsState) -> dict[str, int]:
  """The values the single feed hands to opendbc's ``setup_interfaces``.

  The keys are the LEGACY names the car layer reads; the values are the resolved mode/shaping.  This
  is why the consolidation needs no opendbc or panda change: the model owns the pair, the car layer
  keeps its existing contract, and the wire is byte-identical for the same resolved state."""
  return {
    LEGACY_PEDAL_KEY: 1 if state.pedal else 0,
    LEGACY_BRAKE_KEY: 1 if state.brake else 0,
    LEGACY_SHAPING_KEY: 1 if state.shaping else 0,
  }
