"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.cereal import log, custom
from opendbc.car import structs

from opendbc.car.chrysler.values import RAM_DT
from opendbc.sunnypilot.car.hyundai.gas_interceptor import GAS_CUT_HYSTERESIS
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName
GearShifter = structs.CarState.GearShifter


class CarSpecificEventsSP:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.low_speed_alert = False

  def update(self, CS: structs.CarState, events: Events, long_active: bool):
    events_sp = EventsSP()

    if self.CP.brand == 'chrysler':
      if self.CP.carFingerprint in RAM_DT:
        # remove belowSteerSpeed event from CarSpecificEvents as RAM_DT uses a different logic
        if events.has(EventName.belowSteerSpeed):
          events.remove(EventName.belowSteerSpeed)

        # TODO-SP: use if/elif to have the gear shifter condition takes precedence over the speed condition
        # TODO-SP: add 1 m/s hysteresis
        if CS.vEgo >= self.CP.minEnableSpeed:
          self.low_speed_alert = False
        if self.CP.minEnableSpeed >= 14.5 and CS.gearShifter != GearShifter.drive:
          self.low_speed_alert = True
      if self.low_speed_alert:
        events.add(EventName.belowSteerSpeed)

    elif self.CP.brand == 'hyundai':
      # fork: comma pedal longitudinal (accelerator only, no brakes). Upstream car_events has no Hyundai minEnableSpeed
      # handling, so add it here: block engagement below minEnableSpeed and, once engaged, warn when the car is slow
      # enough that the pedal command is cut (gas_interceptor.py) -> the driver must take over speed control.
      # resumeRequired never fires for Hyundai (car_events has no Hyundai branch; non-SCC cruiseState.standstill is False).
      if self.CP_SP.enableGasInterceptor:
        # only on a longitudinal engage attempt (SET/RES): an unconditional NO_ENTRY would also block MADS
        # lateral-only engagement (LKAS/main button) at low speed, a regression for this steering-first car
        if CS.vEgo < self.CP.minEnableSpeed and events.has(EventName.buttonEnable):
          events.add(EventName.belowEngageSpeed)
        # takeover warning only while longitudinal is actually engaged (long_active = carControl.longActive, passed by
        # selfdrived): when merely armed and the driver is driving manually, the pedal cut is irrelevant and the
        # 'TAKE CONTROL' alert below ~20 mph would be a false alarm
        if long_active and CS.vEgo < self.CP.minEnableSpeed - GAS_CUT_HYSTERESIS:
          events.add(EventName.manualRestart)

    elif self.CP.brand == 'toyota':
      if self.CP.openpilotLongitudinalControl:
        if CS.cruiseState.standstill and not CS.brakePressed and self.CP_SP.enableGasInterceptor:
          if events.has(EventName.resumeRequired):
            events.remove(EventName.resumeRequired)

    return events_sp
