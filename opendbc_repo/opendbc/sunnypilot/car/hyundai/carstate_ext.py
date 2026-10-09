"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import StrEnum
import time

from opendbc.car import Bus, structs
from opendbc.can.parser import CANParser
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.gas_interceptor import GAS_INTERCEPTOR_BUS_KEY, HYUNDAI_GAS_INTERCEPTOR_THRESHOLD, \
                                                         PedalFaultMonitor, get_interceptor_gas, get_interceptor_ids
from opendbc.sunnypilot.car.hyundai.drive_mode import DRIVE_MODE_MSG, DRIVE_MODE_SIGNAL, DriveModeDebouncer
from opendbc.sunnypilot.car.hyundai.fca11_long import FCA11_ADDR
from opendbc.sunnypilot.car.hyundai.pause_resume import PauseResumeTracker
from opendbc.sunnypilot.car.hyundai.wheel_consent import WheelConsentDetector, WheelGesture
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

ButtonType = structs.CarState.ButtonEvent.Type

# panda gives up its timed factory-cruise CANCEL 2 s after the trigger (HYUNDAI_FC_CANCEL_WINDOW_US, hyundai_common.h);
# openpilot alerts 0.5 s later (CarState runs at 100 Hz), so a cancel that lands on the last attempt never alerts
FACTORY_CANCEL_GIVE_UP_FRAMES = 250


class CarStateExt:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.aBasis = 0.0
    # GAS_SENSOR STATE, 0 = NO_FAULT. Read by GasInterceptorCarController.create_gas_command (this CarState instance is
    # the CS it receives) to send the fault-clearing zero frame; escalation comes back through pedal_fault_monitor.
    self.interceptor_state = 0
    self.pedal_fault_monitor = PedalFaultMonitor()
    # fork: FCA11 long braking (HyundaiFca11Brake) inputs the controller reads off CS: the freshest raw bus-2 camera
    # 0x38D frame (t_nanos, dat) for the byte-exact mirror, and the "now" the freshness check compares it against
    # (written by the controller each frame, same pattern as pedal_long_engaged). fca11_lead_closing = a lead is
    # closing (the low-speed hand-over alert gate), also written by the controller from CarControlSP.
    self.fca11_cam_frame: tuple[int, bytes] | None = None
    self.fca11_now_nanos: int = 0
    self.fca11_lead_closing = False
    # 0029 CAL command mode: the lead's distance (the cal sequencer skips a rep when a lead is within
    # plan.conditions.min_lead_m), and the cal-hold flag the controller publishes so the gas
    # neutralization below reads it. Both are inert (False / None) unless cal mode is running.
    self.fca11_lead_drel_m: float | None = None
    self.fca11_cal_active = False
    # 0030 (D1-b): the FCA11-long hand-back flag (camera owns / pedal fault), written by the controller each frame
    # (same pattern as fca11_cal_active) and mirrored onto CarStateSP.fca11Unavailable for the superproject alert.
    self.fca11_unavailable = False
    # 0040: the slowest WHEEL speed (km/h) of the current frame, written by the base CarState (carstate.py) when the
    # feature is armed. The FCA11 episode logger (fca11_log.py) stores it so the post-drive log answers "what did the
    # ESC do below 12 km/h / at standstill". None = not published this frame.
    self.fca11_wheel_min_kph: float | None = None
    # pause/resume button (gas interceptor only). pedal_long_engaged = CarControl.enabled, written by the CarController
    # every frame (this CarState instance is the CS it receives), so it is the engagement state BEFORE a new press.
    self.pause_resume = PauseResumeTracker()
    self.pedal_long_engaged = False
    # factory cruise came on while openpilot long was engaged: panda's timed CANCEL (safety/modes/hyundai_common.h) is
    # trying to turn it off. Still on after FACTORY_CANCEL_GIVE_UP_FRAMES -> accFaulted (audible immediate disable)
    self.factory_cancel_frames = 0          # CarState frames since CRUISE_LAMP_S rose under engaged long; 0 = none
    self.factory_cancel_failed = False
    self.factory_active_prev = False
    # fork (adurham): CLU13 CF_Clu_DriveMode debounce (one frame) + last published raw value. The superproject reads
    # the published raw off CarStateSP.driveMode and maps it to a personality / a longitudinal block.
    self.drive_mode_debouncer = DriveModeDebouncer()
    self.drive_mode = 0
    # 0034: the STEERING-WHEEL consent gesture. The double-press detector lives HERE, in the CarState
    # phase, because that is the ONE place that both sees the raw CLU11 button stream AND can act on a
    # frame BEFORE the CarController consumes CS.buttonEvents -- which is what lets a gesture swallow its
    # own taps so they never reach cruise.py's set-speed path. The detector writes NOTHING; it only
    # produces a "fire"/"dismiss"/None verdict, consumed by the FCA11 controller via the two methods below.
    self.wheel_consent_detector = WheelConsentDetector()
    self._pending_wheel_gesture: str | None = None   # a fresh verdict for the controller to consume this frame
    self._wheel_fca11_brake = None                    # the Fca11LongBrake instance registered by the controller

  def update_drive_mode(self, cp) -> int:
    """Read CLU13 CF_Clu_DriveMode off the C-CAN parser (debounced one source frame). Absent -> unknown (0)."""
    frames = cp.vl_all[DRIVE_MODE_MSG].get(DRIVE_MODE_SIGNAL, []) if DRIVE_MODE_MSG in cp.vl_all else []
    self.drive_mode = self.drive_mode_debouncer.update(frames)
    return self.drive_mode

  def update_speed_limit(self, cp, cp_cam) -> float:
    speed_limit = 0

    if self.CP.flags & HyundaiFlags.CANFD:
      if self.CP_SP.flags & HyundaiFlagsSP.SPEED_LIMIT_AVAILABLE:
        bus = cp if self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG else cp_cam
        speed_limit = bus.vl["FR_CMR_02_100ms"]["ISLW_SpdCluMainDis"]
    else:
      nav, cam = 0, 0
      if self.CP_SP.flags & HyundaiFlagsSP.SPEED_LIMIT_AVAILABLE:
        nav = cp.vl["Navi_HU"]["SpeedLim_Nav_Clu"]
      if self.CP_SP.flags & HyundaiFlagsSP.HAS_LKAS12:
        cam = cp_cam.vl["LKAS12"]["CF_Lkas_TsrSpeed_Display_Clu"]

      speed_limit = cam if cam not in (0, 255) else nav

    if speed_limit in (0, 255):
      speed_limit = 0

    return speed_limit

  def update(self, ret: structs.CarState, ret_sp: structs.CarStateSP, can_parsers: dict[StrEnum, CANParser], speed_conv: float) -> None:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    self.aBasis = cp.vl["TCS13"]["aBasis"]

    if self.CP_SP.flags & HyundaiFlagsSP.NON_SCC:
      cruise_msg = "LABEL11" if self.CP.flags & HyundaiFlags.EV else \
                   "E_CRUISE_CONTROL" if self.CP.flags & HyundaiFlags.HYBRID else \
                   "EMS16"
      cruise_available_sig = "CC_React" if self.CP.flags & HyundaiFlags.EV else "CRUISE_LAMP_M"
      cruise_enabled_sig = "CC_ACT" if self.CP.flags & HyundaiFlags.EV else "CRUISE_LAMP_S"
      cruise_speed_msg = "E_EMS11" if self.CP.flags & HyundaiFlags.EV else \
                         "ELECT_GEAR" if self.CP.flags & HyundaiFlags.HYBRID else \
                         "LVR12"
      cruise_speed_sig = "Cruise_Limit_Target" if self.CP.flags & HyundaiFlags.EV else \
                         "SLC_SET_SPEED" if self.CP.flags & HyundaiFlags.HYBRID else \
                         "CF_Lvr_CruiseSet"
      ret.cruiseState.available = cp.vl[cruise_msg][cruise_available_sig] != 0
      ret.cruiseState.enabled = cp.vl[cruise_msg][cruise_enabled_sig] != 0
      ret.cruiseState.speed = cp.vl[cruise_speed_msg][cruise_speed_sig] * speed_conv
      ret.cruiseState.standstill = False
      ret.cruiseState.nonAdaptive = False

      if not self.CP_SP.flags & HyundaiFlagsSP.NON_SCC_NO_FCA:
        cp_cruise = cp if self.CP_SP.flags & HyundaiFlagsSP.NON_SCC_RADAR_FCA else cp_cam

        aeb_src = "FCA11"
        aeb_warning = cp_cruise.vl[aeb_src]["CF_VSM_Warn"] != 0
        aeb_braking = cp_cruise.vl[aeb_src]["CF_VSM_DecCmdAct"] != 0 or cp_cruise.vl[aeb_src]["FCA_CmdAct"] != 0
        ret.stockFcw = aeb_warning and not aeb_braking
        ret.stockAeb = aeb_warning and aeb_braking

    if self.CP_SP.enableGasInterceptor:
      # The pedal sits between the driver's pedal and the ECU, so EMS16 CF_Ems_AclAct (set in carstate.py) also sees
      # openpilot's own command and would latch gasPressed/override. GAS_SENSOR carries only the driver's input;
      # this assignment runs after carstate.py's and replaces it. Same threshold as panda (hyundai.h).
      cp_gas = can_parsers[GAS_INTERCEPTOR_BUS_KEY]
      sensor_msg = get_interceptor_ids(self.CP_SP).sensor_msg  # GAS_SENSOR (0x201) or GAS_SENSOR_R (0x701)
      ret.gasPressed = get_interceptor_gas(cp_gas, sensor_msg) > HYUNDAI_GAS_INTERCEPTOR_THRESHOLD
      # 0029 CAL: during a scripted CAL hold the commanded pedal is forced to 0, so the bus pedal the
      # car layer parses IS the driver's own input here. gasPressed is therefore left reporting the
      # driver's pedal HONESTLY (not forced False): mid-hold the driver's foot is off (fca11_ok), and a
      # driver intervention must stay visible for the post-drive analysis. The panda independently cuts
      # on its own live gas_pressed, so nothing is hidden from it.
      self.interceptor_state = int(cp_gas.vl[sensor_msg]["STATE"])
      # pedal fault the clear frames could not recover (set by the controller on the previous frame): standard cruise
      # fault -> immediate disengage + alert, so the driver knows openpilot no longer has throttle authority
      if self.pedal_fault_monitor.escalated:
        ret.accFaulted = True

      # fork: FCA11 long braking — keep the freshest raw bus-2 camera 0x38D frame for the byte-exact mirror. The bus-2
      # parser has capture_addrs={0x38D} when the toggle is on (get_can_parsers); empty set otherwise = no hot-loop cost.
      if self.CP_SP.fca11Brake:
        cap = can_parsers[Bus.cam].captured.get(FCA11_ADDR)
        if cap is not None:
          self.fca11_cam_frame = (cap[0], bytes(cap[1]))

      # Factory cruise lockout: openpilot sends no CLU11 in this mode (no CANCEL), so it must never have to fight the
      # factory cruise. The factory cruise can only engage while its MAIN is armed (EMS16 CRUISE_LAMP_M; off at every
      # ignition on routes 112-123), so arming MAIN locks pedal-long out: nonAdaptive -> wrongCruiseMode
      # ("Adaptive Cruise Disabled": disengages longitudinal, blocks engagement). MADS strips wrongCruiseMode from its own
      # state machine, so lateral is unaffected. Pedal-long availability no longer follows the MAIN lamp (always True;
      # panda acc_main_on mirrors this). Panda enforces the lockout independently (hyundai_factory_main_on, hyundai.h).
      factory_main = bool(cp.vl["EMS16"]["CRUISE_LAMP_M"])
      ret.cruiseState.available = True
      ret.cruiseState.nonAdaptive = factory_main or ret.cruiseState.enabled

      # Mirror of panda's timed factory-cruise CANCEL (hyundai_fc_cancel_*): it starts when CRUISE_LAMP_S rises while
      # openpilot long is engaged and gives up after HYUNDAI_FC_CANCEL_WINDOW_US (2 s). If the factory cruise is still
      # on FACTORY_CANCEL_GIVE_UP_FRAMES after the rise, openpilot raises accFaulted: an audible immediate-disable
      # ("Cruise Fault"), held until the factory cruise is off. The lockout above has already dropped longitudinal.
      factory_active = bool(ret.cruiseState.enabled)
      if factory_active and not self.factory_active_prev and self.pedal_long_engaged:
        self.factory_cancel_frames = 1
      elif factory_active and self.factory_cancel_frames > 0:
        self.factory_cancel_frames += 1
      elif not factory_active:
        self.factory_cancel_frames = 0
        self.factory_cancel_failed = False
      if self.factory_cancel_frames > FACTORY_CANCEL_GIVE_UP_FRAMES:
        self.factory_cancel_failed = True
      self.factory_active_prev = factory_active
      if self.factory_cancel_failed:
        ret.accFaulted = True

      # pause/resume (CF_Clu_CruiseSwState 4): replaces the plain cancel mapping. See pause_resume.py.
      pause_events = self.pause_resume.update(cp.vl_all["CLU11"]["CF_Clu_CruiseSwState"], self.pedal_long_engaged,
                                              ret.brakePressed, ret.gasPressed, factory_main)
      # rebuild by value: ret is a capnp builder, re-assigning its own list elements would corrupt them
      kept = [structs.CarState.ButtonEvent(pressed=bool(be.pressed), type=be.type.raw) for be in ret.buttonEvents
              if be.type != ButtonType.cancel]
      # up/down (CF_Clu_CruiseSwState 1 / 2 -> accelCruise / decelCruise) stay as set-speed events only: they never
      # engage in this mode (CarState.update_button_enable), so they need no brake/MAIN bookkeeping here
      ret.buttonEvents = kept + pause_events

      # 0034: the STEERING-WHEEL consent gesture. Runs AFTER ret.buttonEvents is finalised (so it sees the
      # up/down events whether they came from the base create_button_events or any fork override) and
      # BEFORE the CarController consumes them. It is inert unless a rep is actually parked; when it is,
      # it CONSUMES its own two taps (they leave buttonEvents here, exactly like the pause/resume press
      # above) so a wheel consent gesture never also nudges the set speed.
      self.update_wheel_consent(ret)

    ret_sp.speedLimit = self.update_speed_limit(cp, cp_cam) * speed_conv
    ret_sp.driveMode = self.update_drive_mode(cp)
    # 0030 (D1-b): surface the FCA11-long hand-back (set by the controller on the previous frame, same 1-frame pattern
    # as fca11_cal_active). Only when the feature is armed, so the field is exactly "FCA11-long is unavailable now".
    ret_sp.fca11Unavailable = bool(self.CP_SP.fca11Brake and getattr(self, "fca11_unavailable", False))
    # 0040: the three FCA11 braking alerts (see structs.py CarStateSP). Only when the feature is armed, so the fields
    # are exactly "this alert is up right now".
    armed = bool(self.CP_SP.fca11Brake)
    ret_sp.superviseStop = bool(armed and getattr(self, "fca11_supervise_stop", False))
    ret_sp.stopComplete = bool(armed and getattr(self, "fca11_stop_complete", False))
    ret_sp.brakeNow = bool(armed and getattr(self, "fca11_brake_now", False))

  # ------------------------------------------------------------------ 0034 wheel consent gesture
  def register_wheel_brake(self, brake) -> None:
    """0034: the FCA11 controller hands its Fca11LongBrake here (called every controller frame, right
    before it consumes the gesture) so the CarState phase can ask it for the pending rep id. Idempotent."""
    self._wheel_fca11_brake = brake

  def update_wheel_consent(self, ret) -> None:
    """0034: run the double-press detector on this frame's cruise-button events and decide whether the
    taps leave for the set-speed path or are consumed as a wheel consent gesture. Sets
    ``_pending_wheel_gesture`` ("fire" / "dismiss") for the controller to consume this frame.

    INERT unless the FCA11 cal sequencer has a rep parked: with no rep the detector is never called, so
    ``ret.buttonEvents`` is left EXACTLY as built above and set-speed behaves byte-for-byte as today."""
    self._pending_wheel_gesture = None
    brake = self._wheel_fca11_brake
    seq = getattr(brake, "cal_seq", None) if brake is not None else None
    if seq is None:
      return
    pending = seq.pending_rep_id
    if pending is None and not self.wheel_consent_detector.active:
      # no rep parked and no tap window open -> the detector is never stepped, so ordinary set-speed
      # presses (single, double, either arrow) are byte-for-byte today's behaviour.
      return
    gesture, consumed = self.wheel_consent_detector.update(ret.buttonEvents, pending, time.monotonic())
    if gesture != WheelGesture.NONE:
      self._pending_wheel_gesture = "fire" if gesture == WheelGesture.FIRE else "dismiss"
    if consumed:
      # CONSUME the edges the gesture acted on, so its taps do not ALSO nudge the set speed: the gesture
      # and the set-speed action are mutually exclusive BY CONSTRUCTION. Only the triggering tap of a
      # gesture is dropped, and only while a rep is pending -- exactly the frames on which it matters.
      kept = [be for be in ret.buttonEvents
              if (be.type, bool(be.pressed)) not in consumed]
      ret.buttonEvents = kept

  def consume_wheel_consent(self) -> str | None:
    """0034: called by the FCA11 controller each frame AFTER it has stepped the sequencer. Returns the
    gesture to apply ("fire" / "dismiss") or None, consuming it (so a gesture fires at most once)."""
    gesture = self._pending_wheel_gesture
    self._pending_wheel_gesture = None
    return gesture

  def update_canfd_ext(self, ret: structs.CarState, ret_sp: structs.CarStateSP, can_parsers: dict[StrEnum, CANParser],
                       speed_factor: float) -> None:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    self.aBasis = cp.vl["TCS"]["aBasis"]

    ret_sp.speedLimit = self.update_speed_limit(cp, cp_cam) * speed_factor
