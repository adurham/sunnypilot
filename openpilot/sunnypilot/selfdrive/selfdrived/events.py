"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import openpilot.cereal.messaging as messaging
from openpilot.cereal import log, custom
from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.selfdrived.events_base import EventsBase, Priority, ET, Alert, \
  NoEntryAlert, ImmediateDisableAlert, EngagementAlert, NormalPermanentAlert, AlertCallbackType, wrong_car_mode_alert
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import PCM_LONG_REQUIRED_MAX_SET_SPEED, CONFIRM_SPEED_THRESHOLD
from openpilot.common.hardware import HARDWARE

AlertSize = log.SelfdriveState.AlertSize
AlertStatus = log.SelfdriveState.AlertStatus
VisualAlert = car.CarControl.HUDControl.VisualAlert
AudibleAlert = car.CarControl.HUDControl.AudibleAlert
AudibleAlertSP = custom.SelfdriveStateSP.AudibleAlert
EventNameSP = custom.OnroadEventSP.EventName


# get event name from enum
EVENT_NAME_SP = {v: k for k, v in EventNameSP.schema.enumerants.items()}

IS_MICI = HARDWARE.get_device_type() == 'mici'


def speed_limit_adjust_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  speedLimit = sm['longitudinalPlanSP'].speedLimit.resolver.speedLimit
  speed = round(speedLimit * (CV.MS_TO_KPH if metric else CV.MS_TO_MPH))
  message = f'Adjusting to {speed} {"km/h" if metric else "mph"} speed limit'
  return Alert(
    message,
    "",
    AlertStatus.normal, AlertSize.small,
    Priority.LOW, VisualAlert.none, AudibleAlert.none, 4.)


def speed_limit_pre_active_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  speed_conv = CV.MS_TO_KPH if metric else CV.MS_TO_MPH
  v_cruise_cluster = CS.vCruiseCluster
  set_speed = sm['controlsState'].deprecated.vCruise if v_cruise_cluster == 0.0 else v_cruise_cluster
  set_speed_conv = round(set_speed * speed_conv)

  speed_limit_final_last = sm['longitudinalPlanSP'].speedLimit.resolver.speedLimitFinalLast
  speed_limit_final_last_conv = round(speed_limit_final_last * speed_conv)
  alert_1_str = ""
  alert_size = AlertSize.small

  if CP.openpilotLongitudinalControl and CP.pcmCruise:
    # PCM long
    cst_low, cst_high = PCM_LONG_REQUIRED_MAX_SET_SPEED[metric]
    pcm_long_required_max = cst_low if speed_limit_final_last_conv < CONFIRM_SPEED_THRESHOLD[metric] else cst_high
    pcm_long_required_max_set_speed_conv = round(pcm_long_required_max * speed_conv)
    speed_unit = "km/h" if metric else "mph"

    alert_1_str = f"Speed Limit Assist: set to {pcm_long_required_max_set_speed_conv} {speed_unit} to engage"
  else:
    if IS_MICI:
      if set_speed_conv < speed_limit_final_last_conv:
        alert_1_str = "Press + to confirm speed limit"
      elif set_speed_conv > speed_limit_final_last_conv:
        alert_1_str = "Press - to confirm speed limit"
    else:
      alert_size = AlertSize.none

  return Alert(
    alert_1_str,
    "",
    AlertStatus.normal, alert_size,
    Priority.LOW, VisualAlert.none, AudibleAlertSP.promptSingleLow, .1)


class EventsSP(EventsBase):
  def __init__(self):
    super().__init__()
    self.event_counters = dict.fromkeys(EVENTS_SP.keys(), 0)

  def get_events_mapping(self) -> dict[int, dict[str, Alert | AlertCallbackType]]:
    return EVENTS_SP

  def get_event_name(self, event: int):
    return EVENT_NAME_SP[event]

  def get_event_msg_type(self):
    return custom.OnroadEventSP.Event


EVENTS_SP: dict[int, dict[str, Alert | AlertCallbackType]] = {
  # sunnypilot
  EventNameSP.lkasEnable: {
    ET.ENABLE: EngagementAlert(AudibleAlert.engage),
  },

  EventNameSP.lkasDisable: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
  },

  EventNameSP.manualSteeringRequired: {
    ET.USER_DISABLE: Alert(
      "Automatic Lane Centering is OFF",
      "Manual Steering Required",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.disengage, 1.),
  },

  EventNameSP.manualLongitudinalRequired: {
    ET.WARNING: Alert(
      "Smart/Adaptive Cruise Control: OFF",
      "Manual Speed Control Required",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 1.),
  },

  EventNameSP.silentLkasEnable: {
    ET.ENABLE: EngagementAlert(AudibleAlert.none),
  },

  EventNameSP.silentLkasDisable: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.none),
  },

  EventNameSP.silentBrakeHold: {
    ET.WARNING: EngagementAlert(AudibleAlert.none),
    ET.NO_ENTRY: NoEntryAlert("Brake Hold Active"),
  },

  EventNameSP.silentWrongGear: {
    ET.WARNING: Alert(
      "",
      "",
      AlertStatus.normal, AlertSize.none,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, 0.),
    ET.NO_ENTRY: Alert(
      "Gear not D",
      "openpilot Unavailable",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 0.),
  },

  EventNameSP.silentReverseGear: {
    ET.PERMANENT: Alert(
      "Reverse\nGear",
      "",
      AlertStatus.normal, AlertSize.full,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, .2, creation_delay=0.5),
    ET.NO_ENTRY: NoEntryAlert("Reverse Gear"),
  },

  EventNameSP.silentDoorOpen: {
    ET.WARNING: Alert(
      "",
      "",
      AlertStatus.normal, AlertSize.none,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, 0.),
    ET.NO_ENTRY: NoEntryAlert("Door Open"),
  },

  EventNameSP.silentSeatbeltNotLatched: {
    ET.WARNING: Alert(
      "",
      "",
      AlertStatus.normal, AlertSize.none,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, 0.),
    ET.NO_ENTRY: NoEntryAlert("Seatbelt Unlatched"),
  },

  EventNameSP.silentParkBrake: {
    ET.WARNING: Alert(
      "",
      "",
      AlertStatus.normal, AlertSize.none,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, 0.),
    ET.NO_ENTRY: NoEntryAlert("Parking Brake Engaged"),
  },

  EventNameSP.controlsMismatchLateral: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("Controls Mismatch: Lateral"),
    ET.NO_ENTRY: NoEntryAlert("Controls Mismatch: Lateral"),
  },

  EventNameSP.wrongCarModeAlertOnly: {
    ET.WARNING: wrong_car_mode_alert,
  },

  EventNameSP.pedalPressedAlertOnly: {
    ET.WARNING: NoEntryAlert("Pedal Pressed")
  },

  EventNameSP.laneTurnLeft: {
    ET.WARNING: Alert(
      "Turning Left",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 1.),
  },

  EventNameSP.laneTurnRight: {
    ET.WARNING: Alert(
      "Turning Right",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 1.),
  },

  EventNameSP.speedLimitActive: {
    ET.WARNING: Alert(
      "Auto adjusting to speed limit",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlertSP.promptSingleHigh, 5.),
  },

  EventNameSP.speedLimitChanged: {
    ET.WARNING: Alert(
      "Set speed changed",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlertSP.promptSingleHigh, 5.),
  },

  EventNameSP.speedLimitPreActive: {
    ET.WARNING: speed_limit_pre_active_alert,
  },

  EventNameSP.speedLimitPending: {
    ET.WARNING: Alert(
      "Auto adjusting to last speed limit",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlertSP.promptSingleHigh, 5.),
  },

  EventNameSP.e2eChime: {
    ET.PERMANENT: Alert(
      "",
      "",
      AlertStatus.normal, AlertSize.none,
      Priority.MID, VisualAlert.none, AudibleAlert.prompt, 3.),
  },

  EventNameSP.laneChangeRoadEdge: {
    ET.WARNING: Alert(
      "Lane Change Unavailable: Road Edge",
      "",
      AlertStatus.userPrompt, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.prompt, 0.1),
  },

  # fork: Hyundai comma pedal. The factory cruise MAIN is armed, so pedal-long is locked out (wrongCruiseMode). MADS strips
  # wrongCruiseMode before the alerts are built (so lateral survives), which made this lockout completely silent.
  EventNameSP.pedalFactoryCruiseLockout: {
    ET.PERMANENT: NormalPermanentAlert("Factory Cruise Armed: openpilot Long Off", "Press the CC button to turn it off"),
  },

  # fork (0040): the low-speed hand-over alert "FCA11: Take Over Below 12 km/h" is DELETED - the speed floors it
  # warned about (panda 9 km/h, planner 12 km/h) are gone and FCA11 now commands through zero and holds. It is
  # replaced by the three owner-approved alerts below. EventNameSP.fca11BrakeLowSpeedDEPRECATED @28 holds the retired
  # ordinal so old logs still decode; no alert is defined for it and nothing raises it.
  #
  # steady YELLOW "stop complete - hold brake pedal": FCA11 has brought the car to a stop and is STILL holding. This
  # is an explicit HAND-OFF, not a cap: the brake command keeps flowing. It tells the driver to put a foot on the
  # brake so releasing openpilot/ignition leaves the car held.
  EventNameSP.fca11StopComplete: {
    ET.WARNING: Alert("stop complete - hold brake pedal", "FCA11 is holding the stop; keep a foot ready",
                      AlertStatus.normal, AlertSize.small, Priority.LOW, VisualAlert.none, AudibleAlert.none, 2.),
  },

  # RED persistent "BRAKE NOW": the ESC is not delivering what we asked (non-response) or the hold was lost. Urgent,
  # driver-facing, cleared ONLY by the driver pressing the brake (the opendbc flag clears on that edge). Deliberately
  # NOT a hold-duration alert.
  EventNameSP.fca11BrakeNow: {
    ET.PERMANENT: Alert("BRAKE NOW", "FCA11 braking is not responding - press the brake",
                        AlertStatus.critical, AlertSize.full, Priority.HIGHEST, VisualAlert.none,
                        AudibleAlert.warningImmediate, 3.),
  },

  EventNameSP.bigModelReady: {
    ET.PERMANENT: Alert(
      "Big Model Ready",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.prompt, 2.),
  },

  # fork (adurham): drive mode N / N-Custom with DriveModePersonality ON. On a MADS car the log-level
  # driveModePersonalityBlock is stripped so lateral survives, which would make the lockout silent — this is the
  # driver-facing permanent alert, the same pattern as pedalFactoryCruiseLockout.
  EventNameSP.driveModePersonalityLockout: {
    ET.PERMANENT: NormalPermanentAlert("N Mode: openpilot Long Off", "Drive mode N blocks openpilot longitudinal"),
  },

  # fork (adurham), patches 0030 / 0042 / 0053: FCA11-long braking is UNAVAILABLE for the REST OF THIS IGNITION. Two
  # fail-closed causes converge on this one alert (both published by CarStateExt as CarStateSP.fca11Unavailable).
  # 0053: the camera hand-back is NO LONGER one of them (owner, 2026-10-21: "a camera WARNING must not stop openpilot
  # braking ... do not latch until the next ignition") - it has its own transient alert, fca11CameraBraking, below.
  #   - a PEDAL FAULT latched the panda's cut (0042: mirrored in the car layer as _pedal_cut), or
  #   - the panda is REFUSING a sustained wall of our actuating 0x38D (0042: echo-confirmed, >= 3 consecutive
  #     refusals while braking -> _refused_cut).
  # The main text is deliberately NOT camera-specific because the alert now covers all three (it read
  # "Camera Has Brake" while a pedal fault or an echo-confirmed refusal wall was equally covered). The panda refuses
  # every openpilot 0x38D until the next ignition, so openpilot plans stops it cannot execute; without this the loss
  # is completely silent. Permanent + driver-facing, the same pattern as pedalFactoryCruiseLockout. Raised by
  # car_specific.py from CarStateSP.fca11Unavailable (the opendbc half) only when CP_SP.fca11Brake is on (armed).
  # 0042 v2: the pedal-fault mirror is ALERT-ONLY (it never gates a send) and self-heals on an accepted ACTUATING
  # echo (proof the panda is not latched), so a pre-arm blip shows this alert only until the first echoed actuating
  # frame proves the panda is clear.
  EventNameSP.fca11Unavailable: {
    ET.PERMANENT: NormalPermanentAlert("FCA11 Braking Unavailable", "openpilot braking is off while this shows"),
  },

  # fork (adurham), patch 0053: TRANSIENT - the stock camera is sending actual BRAKE commands (braking-class FCA11), so
  # openpilot hands FCA11 back and stops injecting; it resumes on its own ~1.5 s after the camera's last braking frame.
  # Raised only while CarStateSP.fca11CameraBraking holds; no latch, no restart needed.
  EventNameSP.fca11CameraBraking: {
    # AUDIBLE on purpose (review 2026-10-21): the hand-back pauses openpilot braking while the car may
    # still be cruising toward a lead, so the driver has to hear it, not just catch a small text line.
    # warningSoft (not warningImmediate) keeps it below a take-control alarm and it does not mask one.
    ET.WARNING: Alert("camera braking", "openpilot braking paused",
                      AlertStatus.userPrompt, AlertSize.small, Priority.LOW, VisualAlert.none, AudibleAlert.warningSoft, .2),
  },

  # fork (adurham), patch 0053: the panda refused FCA11 frames that only its camera hand-back can refuse while the car
  # layer believed the camera was NOT braking - the two safety layers disagree. The car layer adopts the panda's view
  # (pauses FCA11 braking); a third disagreement in one ignition withholds FCA11 until restart.
  EventNameSP.fca11Divergence: {
    ET.PERMANENT: NormalPermanentAlert("FCA11 Safety Mismatch", "openpilot braking paused - camera state disagrees"),
  },

  # fork (17c FEEDBACK): the TRANSIENT driver-mark confirmation. The owner marks a felt event with ONE bookmark press
  # (good) or TWO rapid presses (bad); the car layer types the burst, writes the k="m" record and arms
  # CarStateSP.driverMark for 2 s. This alert is the whole point: it proves the mark LANDED. The star-button experiment
  # died because the owner pressed it and nothing happened anywhere. Separate ordinals from every FCA11 braking alert,
  # and it cannot fire on a drive with no mark.
  EventNameSP.driverMarkGood: {
    ET.PERMANENT: NormalPermanentAlert("Marked GOOD", duration=2.0),
  },
  EventNameSP.driverMarkBad: {
    ET.PERMANENT: NormalPermanentAlert("Marked BAD", duration=2.0),
  },
}
