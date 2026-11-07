"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
LIMIT_ADAPT_ACC = -1.  # m/s^2 Ideal acceleration for the adapting (braking) phase when approaching speed limits.
LIMIT_MAX_MAP_DATA_AGE = 10.  # s Maximum time to hold to map data, then consider it invalid inside limits controllers.

# Speed Limit Assist constants
PCM_LONG_REQUIRED_MAX_SET_SPEED = {
  True: (33.3333, 36.1111),  # km/h, (120, 130)
  False: (31.2928, 35.7632),  # mph, (70, 80)
}

CONFIRM_SPEED_THRESHOLD = {
  True: 80,   # km/h
  False: 50,  # mph
}

# Speed Limit Assist (fork): auto-apply a set-speed change without a press at ANY vehicle speed,
# EXCEPT when the change would DECREASE the set speed by more than this cap (then still ask — the
# risky case is a wrong/stale limit at speed forcing an unexpected hard slowdown). Increases always
# auto-apply. Units are the native display unit used by CONFIRM_SPEED_THRESHOLD: km/h : mph.
MAX_AUTO_DECREASE = {
  True: 24.0,   # km/h
  False: 15.0,  # mph
}

# Speed Limit Assist (fork): after a manual set-speed override, keep honouring it for at least this
# many seconds regardless of speed-limit changes, so a fast-changing area (limits that flip every
# block) does not immediately re-engage SLA under the driver's hands. Clearing the override requires
# BOTH a speed-limit change since the override AND this window to have elapsed.
OVERRIDE_MEMORY_S = 600.0  # s
