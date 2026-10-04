import math
import numpy as np

from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.car.cruise_ext import VCruiseHelperSP


# WARNING: this value was determined based on the model's training distribution,
#          model predictions above this speed can be unpredictable
# V_CRUISE's are in kph
V_CRUISE_MIN = 8
V_CRUISE_MAX = 145
V_CRUISE_UNSET = 255
V_CRUISE_INITIAL = 40
V_CRUISE_INITIAL_EXPERIMENTAL_MODE = 105
IMPERIAL_INCREMENT = round(CV.MPH_TO_KPH, 1)  # round here to avoid rounding errors incrementing set speed

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type
CRUISE_LONG_PRESS = 50
CRUISE_NEAREST_FUNC = {
  ButtonType.accelCruise: math.ceil,
  ButtonType.decelCruise: math.floor,
}
CRUISE_INTERVAL_SIGN = {
  ButtonType.accelCruise: +1,
  ButtonType.decelCruise: -1,
}


class VCruiseHelper(VCruiseHelperSP):
  def __init__(self, CP, CP_SP):
    VCruiseHelperSP.__init__(self, CP, CP_SP)
    self.CP = CP
    self.v_cruise_kph = V_CRUISE_UNSET
    self.v_cruise_cluster_kph = V_CRUISE_UNSET
    self.v_cruise_kph_last = 0
    self.button_timers = {ButtonType.decelCruise: 0, ButtonType.accelCruise: 0}
    self.button_change_states = {btn: {"standstill": False, "enabled": False} for btn in self.button_timers}
    self.is_metric = False  # fork: Hyundai comma pedal, display unit for initialize_v_cruise (set in update_v_cruise)

  @property
  def v_cruise_initialized(self):
    return self.v_cruise_kph != V_CRUISE_UNSET

  def update_v_cruise(self, CS, enabled, is_metric):
    self.v_cruise_kph_last = self.v_cruise_kph
    self.is_metric = is_metric

    self.get_minimum_set_speed(is_metric)

    _enabled = self.update_enabled_state(CS, enabled)

    if CS.cruiseState.available:
      if not self.CP.pcmCruise or (not self.CP_SP.pcmCruiseSpeed and _enabled):
        # if stock cruise is completely disabled, then we can use our own set speed logic
        if self.CP_SP.enableGasInterceptor:
          self._update_v_cruise_pedal(CS, _enabled, is_metric)  # fork: Hyundai comma pedal (buttons-v3)
        else:
          self._update_v_cruise_non_pcm(CS, _enabled, is_metric)
        self.update_speed_limit_assist_v_cruise_non_pcm()
        self.v_cruise_cluster_kph = self.v_cruise_kph
      else:
        self.v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
        self.v_cruise_cluster_kph = CS.cruiseState.speedCluster * CV.MS_TO_KPH
        if CS.cruiseState.speed == 0:
          self.v_cruise_kph = V_CRUISE_UNSET
          self.v_cruise_cluster_kph = V_CRUISE_UNSET
        elif CS.cruiseState.speed == -1:
          self.v_cruise_kph = -1
          self.v_cruise_cluster_kph = -1
    else:
      self.v_cruise_kph = V_CRUISE_UNSET
      self.v_cruise_cluster_kph = V_CRUISE_UNSET

    if not self.CP.pcmCruise or not self.CP_SP.pcmCruiseSpeed:
      self.update_button_timers(CS, enabled)

  def _update_v_cruise_non_pcm(self, CS, enabled, is_metric):
    # handle button presses. TODO: this should be in state_control, but a decelCruise press
    # would have the effect of both enabling and changing speed is checked after the state transition
    if not enabled:
      return

    long_press = False
    button_type = None

    v_cruise_delta = 1. if is_metric else IMPERIAL_INCREMENT

    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers and not b.pressed:
        if self.button_timers[b.type.raw] > CRUISE_LONG_PRESS:
          return  # end long press
        button_type = b.type.raw
        break
    else:
      for k, timer in self.button_timers.items():
        if timer and timer % CRUISE_LONG_PRESS == 0:
          button_type = k
          long_press = True
          break

    if button_type is None:
      return

    # Don't adjust speed when pressing resume to exit standstill
    cruise_standstill = self.button_change_states[button_type]["standstill"] or CS.cruiseState.standstill
    if button_type == ButtonType.accelCruise and cruise_standstill:
      return

    # Don't adjust speed if we've enabled since the button was depressed (some ports enable on rising edge)
    if not self.button_change_states[button_type]["enabled"]:
      return

    # Speed Limit Assist for Non PCM long cars.
    # True: Disallow set speed changes when user confirmed the target set speed during preActive state
    # False: Allow set speed changes as SLA is not requesting user confirmation
    if self.update_speed_limit_assist_pre_active_confirmed(button_type):
      return

    long_press, v_cruise_delta = VCruiseHelperSP.update_v_cruise_delta(self, long_press, v_cruise_delta)
    if long_press and self.v_cruise_kph % v_cruise_delta != 0:  # partial interval
      self.v_cruise_kph = CRUISE_NEAREST_FUNC[button_type](self.v_cruise_kph / v_cruise_delta) * v_cruise_delta
    else:
      self.v_cruise_kph += v_cruise_delta * CRUISE_INTERVAL_SIGN[button_type]

    # If set is pressed while overriding, clip cruise speed to minimum of vEgo
    if CS.gasPressed and button_type in (ButtonType.decelCruise, ButtonType.setCruise):
      self.v_cruise_kph = max(self.v_cruise_kph, CS.vEgo * CV.MS_TO_KPH)

    self.v_cruise_kph = np.clip(round(self.v_cruise_kph, 1), self.v_cruise_min, V_CRUISE_MAX)

  def _update_v_cruise_pedal(self, CS, enabled, is_metric):
    """fork: Hyundai comma pedal (gas interceptor), buttons-v3. The up/down arrows ONLY change the set speed, engaged or
    not (they never engage or disengage: opendbc CarState.update_button_enable + panda hyundai_common.h); pause/resume
    (resumeCruise) is the only on/off. Disengaged changes are kept and shown, and the next pause/resume engages at them.
      * short press (released before CRUISE_LONG_PRESS = 0.5 s): +/- one display unit (1 mph / 1 km/h), on release
      * long press UP: openpilot's normal long-press step every 0.5 s held (5 units, to the next multiple of 5)
      * long press DOWN: set speed = current speed rounded to the display unit, once per press (replaces the -5 step);
        it never engages, and its release applies nothing (no extra -1)
    Without a set speed this drive (V_CRUISE_UNSET) short presses and long UP do nothing; long DOWN creates one."""
    long_press = False
    button_type = None

    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers and not b.pressed:
        if self.button_timers[b.type.raw] > CRUISE_LONG_PRESS:
          return  # end of a long press: its action already happened
        button_type = b.type.raw
        break
    else:
      for k, timer in self.button_timers.items():
        if timer and timer % CRUISE_LONG_PRESS == 0:
          button_type = k
          long_press = True
          break

    if button_type is None:
      return

    if button_type == ButtonType.decelCruise and long_press:
      if self.button_timers[button_type] == CRUISE_LONG_PRESS:  # first long-press tick only: once per press
        self.v_cruise_kph = self.pedal_v_cruise_from_speed(CS.vEgo, is_metric)
      return

    if not self.v_cruise_initialized:
      return

    if self.update_speed_limit_assist_pre_active_confirmed(button_type):
      return

    v_cruise_delta = 1. if is_metric else IMPERIAL_INCREMENT
    long_press, v_cruise_delta = VCruiseHelperSP.update_v_cruise_delta(self, long_press, v_cruise_delta)
    if long_press and self.v_cruise_kph % v_cruise_delta != 0:  # partial interval
      self.v_cruise_kph = CRUISE_NEAREST_FUNC[button_type](self.v_cruise_kph / v_cruise_delta) * v_cruise_delta
    else:
      self.v_cruise_kph += v_cruise_delta * CRUISE_INTERVAL_SIGN[button_type]

    # down pressed while overriding with the gas (engaged only): not below the current speed, as upstream
    if enabled and CS.gasPressed and button_type == ButtonType.decelCruise:
      self.v_cruise_kph = max(self.v_cruise_kph, CS.vEgo * CV.MS_TO_KPH)

    self.v_cruise_kph = np.clip(round(self.v_cruise_kph, 1), self.v_cruise_min, V_CRUISE_MAX)

  @staticmethod
  def pedal_v_cruise_from_speed(v_ego: float, is_metric: bool) -> float:
    """fork: Hyundai comma pedal. Current speed rounded to the display unit, in km/h, never below openpilot's minimum set
    speed V_CRUISE_MIN (8 km/h; in mph rounded up to a whole 5 mph) and never above V_CRUISE_MAX."""
    if is_metric:
      v_cruise, v_min = round(v_ego * CV.MS_TO_KPH), V_CRUISE_MIN
    else:
      v_cruise = round(v_ego * CV.MS_TO_MPH) * CV.MPH_TO_KPH
      v_min = math.ceil(V_CRUISE_MIN * CV.KPH_TO_MPH) * CV.MPH_TO_KPH
    return float(np.clip(round(max(v_cruise, v_min), 1), V_CRUISE_MIN, V_CRUISE_MAX))

  def update_button_timers(self, CS, enabled):
    # increment timer for buttons still pressed
    for k in self.button_timers:
      if self.button_timers[k] > 0:
        self.button_timers[k] += 1

    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers:
        # Start/end timer and store current state on change of button pressed
        self.button_timers[b.type.raw] = 1 if b.pressed else 0
        self.button_change_states[b.type.raw] = {"standstill": CS.cruiseState.standstill, "enabled": enabled}

  def initialize_v_cruise(self, CS, experimental_mode: bool, dynamic_experimental_control: bool) -> None:
    # initializing is handled by the PCM
    if self.CP.pcmCruise:
      return

    # fork: Hyundai comma pedal (buttons-v3). The only engage is the pause/resume press (resumeCruise): it engages at the
    # set speed already stored this drive (kept across disengages and changed by up/down while disengaged), otherwise at
    # the current speed rounded to the display unit. No 40 km/h V_CRUISE_INITIAL floor (the old 25 mph engage floor is
    # gone; engaging at a standstill gives the 5 mph minimum) and no experimental-mode 105 km/h floor (route 00000128:
    # SET at 26 / 30 / 29 mph -> 65 mph on an accelerator-only car).
    if self.CP_SP.enableGasInterceptor:
      if not self.v_cruise_initialized:
        self.v_cruise_kph = self.pedal_v_cruise_from_speed(CS.vEgo, self.is_metric)
      self.v_cruise_cluster_kph = self.v_cruise_kph
      return

    initial_experimental_mode = experimental_mode and not dynamic_experimental_control
    initial = V_CRUISE_INITIAL_EXPERIMENTAL_MODE if initial_experimental_mode else V_CRUISE_INITIAL

    if any(b.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for b in CS.buttonEvents) and self.v_cruise_initialized:
      self.v_cruise_kph = self.v_cruise_kph_last
    else:
      self.v_cruise_kph = int(round(np.clip(CS.vEgo * CV.MS_TO_KPH, initial, V_CRUISE_MAX)))

    self.v_cruise_cluster_kph = self.v_cruise_kph
