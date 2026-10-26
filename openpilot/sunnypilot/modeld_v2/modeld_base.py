"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import os

from openpilot.common.params import Params


class ModelStateBase:
  def __init__(self):
    # OFFLOAD (macOS): Params storage/backend may be absent or unseeded here, so tolerate a
    # missing LatDelay and allow an explicit env override. Unset => device behavior unchanged.
    if os.environ.get('OFFLOAD') == '1':
      try:
        override = os.environ.get('OFFLOAD_LAT_DELAY')
        self.lat_delay = float(override) if override else Params().get("LagdValueCache", return_default=True)
      except Exception:
        self.lat_delay = 0.1
    else:
      self.lat_delay = Params().get("LagdValueCache", return_default=True)
