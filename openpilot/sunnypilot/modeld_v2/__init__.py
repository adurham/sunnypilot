"""
OFFLOAD=1 macOS modeld_v2 shim (WS-A).

INTERFACES.md §4 requires the fork run path to tolerate a macOS host where the
native C++ params backend (`openpilot/common/libparams_c.dylib`) is not built and
`Params` storage may be empty.  WS-A may only edit files under
`openpilot/sunnypilot/modeld_v2/`, so the tolerant Params backend is installed
here, at package-import time, by injecting a module into `sys.modules` *before*
any submodule (`modeld.py`, `models.helpers`, `relc`, ...) does
`from openpilot.common.params import Params`.

When `OFFLOAD` is unset this module body is a no-op: `openpilot.common.params`
is imported untouched, so device behavior is byte-identical.

Env vars consumed on the Mac path:
  OFFLOAD=1                  enable this shim
  OFFLOAD_CARPARAMS_PKL      path to a file holding serialized cereal CarParams bytes (modeld.py)
  OFFLOAD_LAT_DELAY          float lat delay override (LagdValueCache substitute)
  OFFLOAD_PLANPLUS_CONTROL   float PlanplusControl substitute
  OFFLOAD_CAMERA_OFFSET      float CameraOffset substitute
  OFFLOAD_VIPC_SERVER        VisionIPC server name (consumed in modeld.py)
"""
import os
import re
import sys
import types
from enum import IntEnum, IntFlag

OFFLOAD = os.environ.get('OFFLOAD') == '1'


def _keys_from_header() -> dict:
  """Parse openpilot/common/params_keys.h for {key: default-string}.

  The shim used to carry a hand-maintained default dict, which silently returned None
  for any key it had not enumerated (first failure: LaneTurnValue -> float(None) during
  DesireHelper() init, killing modeld on the Mac). The header is the single source of
  truth, so read it and stop guessing. Missing/unparsable header -> empty map (falls
  back to the small built-in set below).
  """
  out: dict[str, str] = {}
  try:
    header = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          '..', '..', 'common', 'params_keys.h')
    with open(header) as f:
      for line in f:
        m = re.match(r'\s*\{"([A-Za-z0-9_]+)",\s*\{([^}]*)\}', line)
        if not m:
          continue
        key, body = m.group(1), m.group(2)
        parts = [p.strip() for p in body.split(',')]
        if len(parts) >= 3 and parts[2].startswith('"'):
          out[key] = parts[2].strip('"')
  except OSError:
    pass
  return out


def _install_params_shim() -> None:
  mod = types.ModuleType("openpilot.common.params")
  mod.__doc__ = "OFFLOAD macOS shim: tolerant, storage-optional Params backend."

  class UnknownKeyName(Exception):
    pass

  class ParamKeyFlag(IntFlag):
    PERSISTENT = 0x02
    CLEAR_ON_MANAGER_START = 0x04
    CLEAR_ON_ONROAD_TRANSITION = 0x08
    CLEAR_ON_OFFROAD_TRANSITION = 0x10
    DEVELOPMENT_ONLY = 0x40
    CLEAR_ON_IGNITION_ON = 0x80
    BACKUP = 0x100
    ALL = 0xFFFFFFFF

  class ParamKeyType(IntEnum):
    STRING = 0
    BOOL = 1
    INT = 2
    FLOAT = 3
    TIME = 4
    JSON = 5
    BYTES = 6

  # Real defaults from params_keys.h, plus the float keys modeld_v2 reads that carry
  # no default in the header (the on-device C++ layer supplies these from lagd/params).
  _HEADER_DEFAULTS = _keys_from_header()
  _EXTRA_FLOAT_DEFAULTS = {'LagdValueCache': 0.1}

  class Params:
    def __init__(self, d=""):
      self.d = d

    def __reduce__(self):
      return (type(self), (self.d,))

    def check_key(self, key):
      return key.encode() if isinstance(key, str) else key

    def get_type(self, key):
      return ParamKeyType.STRING

    def _default(self, key):
      """Typed default for `key`, or None when the key genuinely has no default.

      Order: fork-specific float overrides, then the parsed params_keys.h table
      (values are stored as the header's literal strings, coerced by the header's
      declared type where we can tell), then None.
      """
      if key in _EXTRA_FLOAT_DEFAULTS:
        return _EXTRA_FLOAT_DEFAULTS[key]
      raw = _HEADER_DEFAULTS.get(key)
      if raw is None:
        return None
      # The header declares types positionally; coerce the obvious ones so callers
      # doing arithmetic (float(get(...)) is fine either way) or truthiness get sane values.
      if raw == "0":
        return False
      if raw == "1":
        return True
      try:
        return float(raw)
      except ValueError:
        return raw

    def get(self, key, block=False, return_default=False):
      override = os.environ.get(f'OFFLOAD_{key.upper()}')
      if override is not None:
        base = self._default(key)
        return override if isinstance(base, str) else float(override)
      if return_default:
        return self._default(key)
      return None

    def get_bool(self, key, block=False):
      return os.environ.get(f'OFFLOAD_BOOL_{key.upper()}') == '1'

    def put_bool(self, key, val, block=False):
      pass

    def put(self, key, dat, block=False):
      pass

    def remove(self, key):
      pass

    def clear_all(self, tx_flag=ParamKeyFlag.ALL):
      pass

    def get_param_path(self, key=""):
      return ""

    def all_keys(self, flag=ParamKeyFlag.ALL):
      return []

  mod.Params = Params
  mod.UnknownKeyName = UnknownKeyName
  mod.ParamKeyFlag = ParamKeyFlag
  mod.ParamKeyType = ParamKeyType

  # Register under a reserved name and alias the canonical module name BEFORE any
  # submodule import resolves `openpilot.common.params`.
  sys.modules.setdefault('openpilot.sunnypilot.modeld_v2._params_offload_shim', mod)
  if 'openpilot.common.params' not in sys.modules:
    sys.modules['openpilot.common.params'] = mod
    try:
      import openpilot.common as _common
      _common.params = mod
    except Exception:
      pass


if OFFLOAD:
  _install_params_shim()
