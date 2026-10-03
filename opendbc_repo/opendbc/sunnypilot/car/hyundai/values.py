"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import IntFlag


class HyundaiSafetyFlagsSP:
  DEFAULT = 0
  ESCC = 1
  LONG_MAIN_CRUISE_TOGGLEABLE = 2
  HAS_LDA_BUTTON = 4
  NON_SCC = 8
  GAS_INTERCEPTOR = 16  # comma pedal longitudinal; C mirror: HYUNDAI_PARAM_SP_GAS_INTERCEPTOR (safety/modes/hyundai_common.h)
  # pedal uses the REMAPPED IDs (cmd 0x700 / sensor 0x701) instead of 0x200/0x201. Only meaningful with GAS_INTERCEPTOR.
  # C mirror: HYUNDAI_PARAM_SP_GAS_INTERCEPTOR_REMAPPED. Also the single source of truth for the dialect in the car layer.
  GAS_INTERCEPTOR_REMAPPED = 32
  # TEST-ONLY, NOT FOR ROAD USE: parked FCA11 (0x38D) brake-injection research. C mirror: HYUNDAI_PARAM_SP_FCA11_BRAKE_TEST.
  # Panda honors it only on non-SCC ICE without openpilot longitudinal; it then blocks the stock camera FCA11 relay and
  # allows CR_VSM_DecCmd <= HYUNDAI_FCA11_TEST_MAX_DEC at standstill. No car-layer code sets it yet.
  FCA11_BRAKE_TEST = 64


# Raw CR_VSM_DecCmd cap (0.01 g/LSB) for FCA11_BRAKE_TEST; must equal HYUNDAI_FCA11_TEST_MAX_DEC in safety/modes/hyundai.h
HYUNDAI_FCA11_TEST_MAX_DEC = 10


class HyundaiFlagsSP(IntFlag):
  """
    Flags for Hyundai specific quirks within sunnypilot.
  """
  ENHANCED_SCC = 1
  HAS_LFA_BUTTON = 2  # Deprecated in favor of HyundaiFlags.HAS_LDA_BUTTON
  LONGITUDINAL_MAIN_CRUISE_TOGGLEABLE = 2 ** 2
  ENABLE_RADAR_TRACKS_DEPRECATED = 2 ** 3
  LONG_TUNING_DYNAMIC = 2 ** 4
  LONG_TUNING_PREDICTIVE = 2 ** 5
  NON_SCC = 2 ** 6
  NON_SCC_RADAR_FCA = 2 ** 7  # most with FCA come from the camera
  NON_SCC_NO_FCA = 2 ** 8  # not all have FCA
  SPEED_LIMIT_AVAILABLE = 2 ** 9  # platforms with speed limit data available
  HAS_LKAS12 = 2 ** 10
  GAS_INTERCEPTOR_DETECTED = 2 ** 11  # capability only: comma pedal GAS_SENSOR (0x201) seen on bus 0 of a non-SCC ICE car
  # capability only: remapped-ID pedal (custom firmware) GAS_SENSOR_R (0x701) seen on bus 0 of a non-SCC ICE car.
  # Independent of GAS_INTERCEPTOR_DETECTED (which stays 'standard 0x201 seen'), so 'both seen' is representable.
  GAS_INTERCEPTOR_REMAPPED_DETECTED = 2 ** 12
