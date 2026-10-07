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
import sys
import types
from enum import IntEnum, IntFlag

OFFLOAD = os.environ.get('OFFLOAD') == '1'


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

  _DEFAULTS = {
    'LagdValueCache': 0.1,
    'PlanplusControl': 1.0,
    'CameraOffset': 0.0,
  }

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
      return _DEFAULTS.get(key)

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
