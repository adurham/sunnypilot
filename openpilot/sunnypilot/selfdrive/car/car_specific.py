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

  def update(self, CS: structs.CarState, CS_SP: structs.CarStateSP, events: Events, long_active: bool):
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

        # fork (0040): the two driver-facing FCA11 braking alerts, raised from CarStateSP (the opendbc half,
        # CarStateExt).
        #   fca11StopComplete  - steady YELLOW at/near a stop while still holding (an explicit hand-off, NOT a cap:
        #                        the command keeps flowing; it tells the driver to put a foot on the brake).
        #   fca11BrakeNow      - persistent RED on ESC non-response or hold-lost; clears only on driver brake.
        # "supervise stop" (steady yellow while braking below 15 km/h) was DELETED on the owner's instruction
        # (drive 16a review, 2026-10-25): it was 12 banners in a 10-minute drive and told him nothing he did not
        # already know. The CarStateSP.superviseStop flag stays in the log for analysis; nothing raises an event.
        # The old hand-over alert ("FCA11: Take Over Below 12 km/h") is DELETED with the floors it warned about.
        if self.CP_SP.fca11Brake:
          # fork (0043): the standstill HOLD COUNTDOWN v2. brakeNow still outranks every nag TIER (a real brake fault
          # outranks a nag), the nag replaces stopComplete while it is up (it says the same thing with the countdown
          # attached), and stopComplete still covers every hold shorter than 60 s - so the screen never shows two
          # overlapping hold banners.
          #
          # nag == 4 IS NOT A TIER: it is the expiry latch, and it is the one state that ACTS. The hold has ended at
          # 2:00, so this frame both tells the driver and takes openpilot's LONGITUDINAL control out: at 2:00
          # longitudinal control disengages and the brakes release; lateral (MADS) stays engaged by design. That is
          # measured, not assumed (Opus review D1): selfdrived calls `mads.update(CS)` one line after
          # `state_machine.update` (selfdrived.py l.660/l.662) and `mads.update_events` REMOVES `pcmDisable`
          # (mads.py l.205) before the MADS state machine looks at anything, so lateral never sees this write.
          #   * `pcmDisable` (ET.USER_DISABLE) on the STANDARD events object this function is handed. selfdrived.py
          #     l.284 passes `self.events` in and l.660 feeds exactly that object to `state_machine.update` in the SAME
          #     update, so the disengage lands on this frame with no selfdrived.py change, no new cereal enum and no
          #     compiled code. The release is then the DEPLOYED driver-disengage release, MEASURED under the car's
          #     own configuration (continuous law ON) through the COMPILED libsafety TX hook
          #     (lane-X3/proof_release_compiled.py): the law puts NOTHING on 0x38D after the disengage (0 frames),
          #     every one of the 6 046 hold frames is ACCEPTED and none refused, and the panda's own fwd rule stops
          #     blocking the camera's 0x38D at +90 ms - inside its 100 ms TX_STALE - so the ESC is back on the
          #     camera's idle level 0. There is NO host ramp on this car: `dynamic_brakes` is False on the shaped
          #     instance (the release ramp is a feel-path feature), so the disengage release is the camera
          #     hand-back. A DRIVER-BRAKE release is different and is also measured: exactly ONE host passive frame
          #     at level 0 (30 -> 0 in one frame, accepted - passive frames carry no rate gate). The law's own
          #     bytes are untouched (P1).
          #   * while the latch is set, `buttonEnable`/`pcmEnable` are removed - the idiom MADS itself uses
          #     (mads.py l.163-164) - so a RES press cannot re-engage until the driver brakes. This is how the main
          #     state machine is blocked WITHOUT inventing a standard ET.NO_ENTRY: every USER_DISABLE+NO_ENTRY standard
          #     event is already bound to a real signal on this car (buttonCancel is genuinely raised by its CANCEL
          #     button, car_events l.162 since pcmCruise is False), and a new standard EventName would be an
          #     upstream-owned cereal change. The SP event below carries NO_ENTRY for MADS (it reads events_sp), which
          #     is what refuses LATERAL RE-ENTRY once the driver has turned lateral off (LKAS) - not the lateral that is
          #     already running: the expiry does not touch it. Cost, stated plainly: `engageable` stays True while latched (it is
          #     `not events.contains(ET.NO_ENTRY)`), so the engage is offered and refused - the red PERMANENT banner is
          #     the feedback. Reported to the owner, not hidden.
          nag = int(getattr(CS_SP, "holdNag", 0))
          if nag == 4:
            events_sp.add(EventNameSP.fca11HoldExpired)
            events.add(EventName.pcmDisable)
            events.remove(EventName.buttonEnable)
            events.remove(EventName.pcmEnable)
          elif CS_SP.brakeNow:
            events_sp.add(EventNameSP.fca11BrakeNow)
          elif nag == 3:
            events_sp.add(EventNameSP.fca11HoldNagFinal)
          elif nag == 2:
            events_sp.add(EventNameSP.fca11HoldNagLoud)
          elif nag == 1:
            events_sp.add(EventNameSP.fca11HoldNagSoft)
          elif CS_SP.stopComplete:
            events_sp.add(EventNameSP.fca11StopComplete)

        # fork (adurham), patch 0030: FCA11-long is unavailable for the rest of this ignition (the panda has
        # handed FCA11 back to the camera / latched a pedal fault). Surfaced from CarStateSP.fca11Unavailable
        # (the opendbc half, CarStateExt): without it, openpilot silently plans stops it cannot execute.
        if self.CP_SP.fca11Brake and CS_SP.fca11Unavailable:
          events_sp.add(EventNameSP.fca11Unavailable)
        # fork (adurham), patch 0053: the camera hand-back is no longer a latch. While the stock camera is BRAKING the
        # driver sees a TRANSIENT notice that clears on its own when openpilot resumes; a Python <-> panda
        # disagreement on that hand-back raises its own alert (the divergence guard, fca11_long.py).
        if self.CP_SP.fca11Brake and CS_SP.fca11Divergence:
          events_sp.add(EventNameSP.fca11Divergence)
        elif self.CP_SP.fca11Brake and CS_SP.fca11CameraBraking:
          events_sp.add(EventNameSP.fca11CameraBraking)

        # fork (17c FEEDBACK): the TRANSIENT driver-mark confirmation ("Marked GOOD" / "Marked BAD", 2 s). The car layer
        # arms CarStateSP.driverMark for exactly MARK_ALERT_FRAMES after a burst of bookmark presses is typed, so this
        # raises nothing on a drive in which the driver never marks anything, and it is a separate ordinal from every
        # FCA11 braking alert above (36/37 vs 30-35) - it can never replace or mask them.
        if getattr(CS_SP, "driverMark", 0) == 2:
          events_sp.add(EventNameSP.driverMarkBad)
        elif getattr(CS_SP, "driverMark", 0) == 1:
          events_sp.add(EventNameSP.driverMarkGood)

    elif self.CP.brand == 'toyota':
      if self.CP.openpilotLongitudinalControl:
        if CS.cruiseState.standstill and not CS.brakePressed and self.CP_SP.enableGasInterceptor:
          if events.has(EventName.resumeRequired):
            events.remove(EventName.resumeRequired)

    return events_sp
