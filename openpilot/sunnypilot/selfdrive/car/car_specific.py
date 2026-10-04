"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.cereal import log, custom
from opendbc.car import structs

from opendbc.car.chrysler.values import RAM_DT
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
      # fork: comma pedal longitudinal (accelerator only, no brakes), buttons-v3. There is NO engage floor any more
      # (CP.minEnableSpeed = -1): the pause/resume press is the only on/off and engages at any speed incl. a standstill;
      # the up/down arrows only change the set speed. So no belowEngageSpeed, no SET/RES refusal alert and no low-speed
      # takeover warning. The driver remains the brake; any brake press disengages; the launch is throttle-limited
      # (gas_interceptor.py LOW_SPEED_MAX_GAS).
      if self.CP_SP.enableGasInterceptor:
        # pause/resume with no set speed yet this drive engages at the current speed (cruise.py initialize_v_cruise),
        # so selfdrived's 'Press Set to Engage' block (there is no SET-to-engage button in this mode) must not apply
        if events.has(EventName.resumeBlocked):
          events.remove(EventName.resumeBlocked)
        # factory cruise MAIN armed: wrongCruiseMode locks pedal-long out, but MADS strips wrongCruiseMode too (route
        # 00000128 @152.5-187.4 s: 35 s locked out, 13 button presses, no alert at all)
        if CS.cruiseState.nonAdaptive:
          events_sp.add(EventNameSP.pedalFactoryCruiseLockout)

    elif self.CP.brand == 'toyota':
      if self.CP.openpilotLongitudinalControl:
        if CS.cruiseState.standstill and not CS.brakePressed and self.CP_SP.enableGasInterceptor:
          if events.has(EventName.resumeRequired):
            events.remove(EventName.resumeRequired)

    return events_sp
