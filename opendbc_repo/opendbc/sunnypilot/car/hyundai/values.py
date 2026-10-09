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
  # TEST-ONLY, NOT FOR ROAD USE: extends FCA11_BRAKE_TEST (ignored without it) from standstill-only to a low-speed ROLLING
  # window: same decel cap, HBA/StopReq still blocked, actuation only in D with every wheel in (MIN, MAX] km/h, no pedal,
  # fresh inputs, and a latching cut. C mirror: HYUNDAI_PARAM_SP_FCA11_ROLLING_TEST. No car-layer code sets it.
  FCA11_ROLLING_TEST = 128
  # PRODUCTION (toggle-gated, default OFF): FCA11 longitudinal braking on top of the gas interceptor
  # (needs GAS_INTERCEPTOR; armed by _initialize_hyundai_gas_interceptor from the HyundaiFca11Brake param).
  # Panda (bit 256): actuating FCA11 only while controls_allowed AND heartbeat_engaged, gear D, no pedal, fresh
  # inputs, every wheel above 9 km/h (no ceiling), DecCmd <= 30 (0.30 g), rate-limited growth (+0.04 g/camera
  # period, every (re)opened episode restarting at +0.04 g), NO duration budget / cooldown (0035), and an immediate
  # camera hand-back. C mirror:
  # HYUNDAI_PARAM_SP_FCA11_LONG.
  FCA11_LONG = 256
  # TEST-ONLY, NOT FOR ROAD USE: parked LKAS11 (0x340) steering-torque sweep, armed only by the steer-test runner with
  # LKAS_PARK_TEST. Panda honors it only on non-SCC ICE without openpilot longitudinal and never together with the
  # FCA11 test bits; while armed LKAS11 is the ONLY transmittable message, only parked (gear P/N, every wheel
  # <= 5.0 km/h, fresh inputs), |torque| <= HYUNDAI_LKAS_PARK_MAX_TORQUE with the +3/-7 rate law, a 60 s cap per arm and
  # a latched cut on any MDPS12 fault bit / driver torque / gas / motion / gear change / angle. No car-layer code sets
  # it. C mirror: HYUNDAI_PARAM_SP_LKAS_PARK_TEST.
  LKAS_PARK_TEST = 512
  # Elantra N (HYUNDAI_ELANTRA_2022_NON_SCC) only: panda allows LKAS11 torque rate-up 4/frame (default 3); STEER_MAX stays
  # 384. C mirror: HYUNDAI_PARAM_SP_CN7_STEER_RAMP. Must match CarControllerParams.STEER_DELTA_UP for that platform.
  # Both are test/car-layer SP bits on distinct flags: 512 LKAS_PARK_TEST (unshipped, this file) and 1024 CN7_STEER_RAMP.
  CN7_STEER_RAMP = 1024


# Raw CR_VSM_DecCmd cap (0.01 g/LSB) for FCA11_BRAKE_TEST; must equal HYUNDAI_FCA11_TEST_MAX_DEC in safety/modes/hyundai.h
HYUNDAI_FCA11_TEST_MAX_DEC = 10
# Raw CR_VSM_DecCmd cap for the ROLLING mode (FCA11_BRAKE_TEST | FCA11_ROLLING_TEST) only: 0.30 g for the scaling test.
# Must equal HYUNDAI_FCA11_ROLL_MAX_DEC in safety/modes/hyundai.h. The parked mode keeps HYUNDAI_FCA11_TEST_MAX_DEC.
HYUNDAI_FCA11_ROLL_MAX_DEC = 30
# FCA11_ROLLING_TEST speed window (km/h, per wheel); must equal HYUNDAI_FCA11_ROLL_MIN_SPEED / _MAX_SPEED (raw / 32) in
# safety/modes/hyundai.h (unit tests pin both boundaries)
HYUNDAI_FCA11_ROLL_MIN_SPEED_KPH = 5
HYUNDAI_FCA11_ROLL_MAX_SPEED_KPH = 35
# *** PRODUCTION FCA11 long braking (FCA11_LONG) constants; unit tests pin them against safety/modes/hyundai.h ***
# Raw CR_VSM_DecCmd cap: 0.30 g. Measured on the car (roll-20261005T232357Z): 0.30 g -> 2.03 m/s^2 delivered (~69%,
# linear); ~COMFORT_BRAKE. Must equal HYUNDAI_FCA11_LONG_MAX_DEC.
HYUNDAI_FCA11_LONG_MAX_DEC = 30
# 0040: the speed floor constant HYUNDAI_FCA11_LONG_MIN_SPEED_KPH (9 km/h) is DELETED together with the panda's
# HYUNDAI_FCA11_LONG_MIN_SPEED and the planner's FCA11_BRAKE_MIN_KPH. Actuation is legal at any speed, through zero.
# Decel growth rate limit per camera period (20 ms), raw LSB: +0.04 g -> 2.0 g/s. Must equal HYUNDAI_FCA11_LONG_RATE_STEP.
HYUNDAI_FCA11_LONG_RATE_STEP = 4
# 0038: driver-gas hold-off after the pedal RELEASES, microseconds; the panda refuses actuating FCA11 for this long
# (a plain elapsed-time check, NOT a latch and NOT a hold cap). Must equal HYUNDAI_FCA11_LONG_GAS_HOLDOFF_US in
# safety/modes/hyundai.h; the car layer mirrors it as GAS_HOLDOFF_FRAMES (100 Hz controller frames) in fca11_long.py.
HYUNDAI_FCA11_LONG_GAS_HOLDOFF_US = 500000   # 0.5 s
# 0038: panda host-liveness (heartbeat) watchdog: no fresh host TX frame for this long -> refuse actuation. Must equal
# HYUNDAI_FCA11_LONG_HOST_HB_TIMEOUT_US in safety/modes/hyundai.h.
HYUNDAI_FCA11_LONG_HOST_HB_TIMEOUT_US = 500000   # 0.5 s
# *** TEST-ONLY parked LKAS11 sweep (LKAS_PARK_TEST) constants; unit tests pin each against safety/modes/hyundai.h ***
HYUNDAI_LKAS_PARK_MAX_TORQUE = 384      # = CarControllerParams STEER_MAX
HYUNDAI_LKAS_PARK_RATE_UP = 3           # = STEER_DELTA_UP
HYUNDAI_LKAS_PARK_RATE_DOWN = 7         # = STEER_DELTA_DOWN
HYUNDAI_LKAS_PARK_MAX_ARM_S = 60.0      # per-arm hard cap (C: HYUNDAI_LKAS_PARK_MAX_ARM_US)
HYUNDAI_LKAS_PARK_WHEEL_MAX_RAW = 160   # every wheel <= 160 raw (5.0 km/h). Tire scrub at standstill (owner-authorized 2026-10-06)
HYUNDAI_LKAS_PARK_MAX_ANGLE_DEG = 85.0  # |SAS_Angle| above this latches the firmware cut
HYUNDAI_LKAS_PARK_MAX_DRIVER_TQ_NM = 5.0  # |CR_Mdps_StrTq| above this latches the firmware cut


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
