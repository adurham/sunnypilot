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
ButtonType = structs.CarState.ButtonEvent.Type


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
        # SET/RES below minEnableSpeed: refused (an unconditional NO_ENTRY would also block MADS lateral-only engagement).
        # NOT for the deliberate pause/resume press (resumeCruise, pause_resume.py): that is the green-light resume after
        # a red-light stop and may engage at any speed incl. a standstill (drive-128 report §3). The driver remains the
        # brake (there is no brake actuator); brake press disengages instantly; the throttle is launch-limited in
        # gas_interceptor.py (LOW_SPEED_MAX_GAS). No low-speed pedal cut / 'TAKE CONTROL' any more: it made the resume
        # impossible and the alert would fire for the whole stop-and-go phase.
        pause_resume = any(be.type == ButtonType.resumeCruise for be in CS.buttonEvents)
        if CS.vEgo < self.CP.minEnableSpeed and events.has(EventName.buttonEnable) and not pause_resume:
          events.add(EventName.belowEngageSpeed)
          # MADS removes belowEngageSpeed from the events after the state machine used it (only the alert is lost), so
          # without this the refusal was silent (route 00000128 @128.5-138.5 s, 1685-1689 s: 8 silent refusals)
          events_sp.add(EventNameSP.pedalBelowEngageSpeed)
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
