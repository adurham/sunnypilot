"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import time
import unittest.mock

from openpilot.common.parameterized import parameterized

from openpilot.cereal import custom
from opendbc.car.car_helpers import interfaces
from opendbc.car.rivian.values import CAR as RIVIAN
from opendbc.car.structs import car
from opendbc.car.tesla.values import CAR as TESLA
from opendbc.car.toyota.values import CAR as TOYOTA
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.car import interfaces as sunnypilot_interfaces
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import PCM_LONG_REQUIRED_MAX_SET_SPEED, OVERRIDE_MEMORY_S
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist import SpeedLimitAssist, \
  PRE_ACTIVE_GUARD_PERIOD, ACTIVE_STATES, CRUISE_BUTTON_CONFIRM_HOLD
from openpilot.sunnypilot.selfdrive.selfdrived.button_state_tracker import ButtonStateTracker
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.common.test import OpenpilotTestCase

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type

SpeedLimitAssistState = custom.LongitudinalPlanSP.SpeedLimit.AssistState

ALL_STATES = tuple(SpeedLimitAssistState.schema.enumerants.values())

SPEED_LIMITS = {
  'residential': 25 * CV.MPH_TO_MS,  # 25 mph
  'city': 35 * CV.MPH_TO_MS,         # 35 mph
  'highway': 65 * CV.MPH_TO_MS,      # 65 mph
  'freeway': 80 * CV.MPH_TO_MS,      # 80 mph
}

DEFAULT_CAR = TOYOTA.TOYOTA_RAV4_TSS2


class TestSpeedLimitAssist(OpenpilotTestCase):
  car_name = DEFAULT_CAR

  def setup_method(self):
    self.params = Params()
    self.reset_custom_params()
    self.events_sp = EventsSP()
    CI = self._setup_platform(self.car_name)
    self.sla = SpeedLimitAssist(CI.CP, CI.CP_SP)
    self.sla.pre_active_timer = int(PRE_ACTIVE_GUARD_PERIOD[self.sla.pcm_op_long] / DT_MDL)
    self.pcm_long_max_set_speed = PCM_LONG_REQUIRED_MAX_SET_SPEED[self.sla.is_metric][1]  # use 80 MPH for now
    self.speed_conv = CV.MS_TO_KPH if self.sla.is_metric else CV.MS_TO_MPH

  def teardown_method(self):
    self.reset_state()

  def _setup_platform(self, car_name):
    CarInterface = interfaces[car_name]
    CP = CarInterface.get_non_essential_params(car_name)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)
    CI = CarInterface(CP, CP_SP)
    CI.CP.openpilotLongitudinalControl = True  # always assume it's openpilot longitudinal
    sunnypilot_interfaces.setup_interfaces(CI, self.params)
    return CI

  def reset_custom_params(self):
    self.params.put("IsReleaseSpBranch", True, block=True)
    self.params.put("SpeedLimitMode", int(Mode.assist), block=True)
    self.params.put_bool("IsMetric", False, block=True)
    self.params.put("SpeedLimitOffsetType", 0, block=True)
    self.params.put("SpeedLimitValueOffset", 0, block=True)

  def reset_state(self):
    self.sla.state = SpeedLimitAssistState.disabled
    self.sla.frame = -1
    self.sla.last_op_engaged_frame = 0
    self.sla.op_engaged = False
    self.sla.op_engaged_prev = False
    self.sla._speed_limit = 0.
    self.sla.speed_limit_prev = 0.
    self.sla.last_valid_speed_limit_offsetted = 0.
    self.sla._distance = 0.
    self.events_sp.clear()

  def initialize_active_state(self, initialize_v_cruise):
    self.sla.state = SpeedLimitAssistState.active
    self.sla.v_cruise_cluster = initialize_v_cruise
    self.sla.v_cruise_cluster_prev = initialize_v_cruise
    self.sla.prev_v_cruise_cluster_conv = round(initialize_v_cruise * self.speed_conv)

  def test_initial_state(self):
    assert self.sla.state == SpeedLimitAssistState.disabled
    assert not self.sla.is_enabled
    assert not self.sla.is_active
    assert V_CRUISE_UNSET == self.sla.get_v_target_from_control()

  @parameterized.expand([RIVIAN.RIVIAN_R1, TESLA.TESLA_MODEL_Y], names=["car_name"])
  def test_disallowed_brands(self, car_name):
    """
      Speed Limit Assist is disabled for the following brands and conditions:
      - All Tesla and is a release branch;
      - All Rivian
    """
    self.car_name = car_name
    self.openpilot_setup_method()  # rebuild the platform for this brand

    assert not self.sla.enabled

    # stay disallowed even when the param may have changed from somewhere else
    self.params.put("SpeedLimitMode", int(Mode.assist), block=True)
    for _ in range(int(PARAMS_UPDATE_PERIOD / DT_MDL)):
      self.sla.update(True, False, SPEED_LIMITS['city'], 0, SPEED_LIMITS['highway'], SPEED_LIMITS['city'],
                      SPEED_LIMITS['city'], True, 0, self.events_sp)
    assert not self.sla.enabled

  def test_disabled(self):
    self.params.put("SpeedLimitMode", int(Mode.off), block=True)
    for _ in range(int(10. / DT_MDL)):
      self.sla.update(True, False, SPEED_LIMITS['city'], 0, SPEED_LIMITS['highway'], SPEED_LIMITS['city'], SPEED_LIMITS['city'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.disabled

  def test_transition_disabled_to_preactive(self):
    for _ in range(int(3. / DT_MDL)):
      self.sla.update(True, False, SPEED_LIMITS['city'], 0, SPEED_LIMITS['highway'], SPEED_LIMITS['city'], SPEED_LIMITS['city'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.preActive
    assert self.sla.is_enabled and not self.sla.is_active

  def test_transition_disabled_to_pending_no_speed_limit_not_max_initial_set_speed(self):
    for _ in range(int(3. / DT_MDL)):
      self.sla.update(True, False, SPEED_LIMITS['highway'], 0, SPEED_LIMITS['city'], 0, 0, False, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.pending
    assert self.sla.is_enabled and not self.sla.is_active

  def test_preactive_to_active_with_max_speed_confirmation(self):
    self.sla.state = SpeedLimitAssistState.preActive
    self.sla.update(True, False, SPEED_LIMITS['city'], 0, self.pcm_long_max_set_speed, SPEED_LIMITS['highway'],
                    SPEED_LIMITS['highway'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.active
    assert self.sla.is_enabled and self.sla.is_active
    assert self.sla.output_v_target == SPEED_LIMITS['highway']

  def test_preactive_timeout_to_inactive(self):
    self.sla.state = SpeedLimitAssistState.preActive
    self.sla.update(True, False, SPEED_LIMITS['city'], 0, SPEED_LIMITS['highway'], SPEED_LIMITS['city'], SPEED_LIMITS['city'], True, 0, self.events_sp)

    for _ in range(int(PRE_ACTIVE_GUARD_PERIOD[self.sla.pcm_op_long] / DT_MDL)):
      self.sla.update(True, False, SPEED_LIMITS['city'], 0, SPEED_LIMITS['highway'], SPEED_LIMITS['city'], SPEED_LIMITS['city'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.inactive

  def test_preactive_to_pending_no_speed_limit(self):
    self.sla.state = SpeedLimitAssistState.preActive
    self.sla.update(True, False, SPEED_LIMITS['highway'], 0, self.pcm_long_max_set_speed, 0, 0, False, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.pending
    assert self.sla.is_enabled and not self.sla.is_active

  def test_pending_to_active_when_speed_limit_available(self):
    self.sla.state = SpeedLimitAssistState.pending
    self.sla.v_cruise_cluster_prev = self.pcm_long_max_set_speed
    self.sla.prev_v_cruise_cluster_conv = round(self.pcm_long_max_set_speed * self.speed_conv)

    self.sla.update(True, False, SPEED_LIMITS['highway'], 0, self.pcm_long_max_set_speed,
                    SPEED_LIMITS['highway'], SPEED_LIMITS['highway'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.active

  def test_pending_to_adapting_when_below_speed_limit(self):
    self.sla.state = SpeedLimitAssistState.pending
    self.sla.v_cruise_cluster_prev = self.pcm_long_max_set_speed
    self.sla.prev_v_cruise_cluster_conv = round(self.pcm_long_max_set_speed * self.speed_conv)

    self.sla.update(True, False, SPEED_LIMITS['highway'] + 5, 0, self.pcm_long_max_set_speed,
                    SPEED_LIMITS['highway'], SPEED_LIMITS['highway'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.adapting
    assert self.sla.is_enabled and self.sla.is_active

  def test_active_to_adapting_transition(self):
    self.initialize_active_state(self.pcm_long_max_set_speed)

    self.sla.update(True, False, SPEED_LIMITS['highway'] + 2, 0, self.pcm_long_max_set_speed, SPEED_LIMITS['highway'],
                    SPEED_LIMITS['highway'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.adapting

  def test_adapting_to_active_transition(self):
    self.sla.state = SpeedLimitAssistState.adapting
    self.sla.v_cruise_cluster_prev = self.pcm_long_max_set_speed
    self.sla.prev_v_cruise_cluster_conv = round(self.pcm_long_max_set_speed * self.speed_conv)

    self.sla.update(True, False, SPEED_LIMITS['city'], 0, self.pcm_long_max_set_speed, SPEED_LIMITS['highway'],
                    SPEED_LIMITS['highway'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.active

  def test_manual_cruise_change_detection(self):
    self.sla.state = SpeedLimitAssistState.active
    expected_cruise = SPEED_LIMITS['highway']
    self.sla.v_cruise_cluster_prev = expected_cruise

    different_cruise = SPEED_LIMITS['highway'] + 5
    self.sla.update(True, False, SPEED_LIMITS['city'], 0, different_cruise, SPEED_LIMITS['city'], SPEED_LIMITS['city'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.inactive

  # TODO-SP: test lower CST cases
  def test_rapid_speed_limit_changes(self):
    self.initialize_active_state(self.pcm_long_max_set_speed)
    speed_limits = [SPEED_LIMITS['highway'], SPEED_LIMITS['freeway']]

    for _, speed_limit in enumerate(speed_limits):
      self.sla.update(True, False, speed_limit, 0, self.pcm_long_max_set_speed, speed_limit, speed_limit, True, 0, self.events_sp)
    assert self.sla.state in ACTIVE_STATES

  def test_invalid_speed_limits_handling(self):
    self.initialize_active_state(self.pcm_long_max_set_speed)

    invalid_limits = [-10, 0, 200 * CV.MPH_TO_MS]

    for invalid_limit in invalid_limits:
      self.sla.update(True, False, SPEED_LIMITS['city'], 0, self.pcm_long_max_set_speed, invalid_limit, SPEED_LIMITS['city'], True, 0, self.events_sp)
      assert isinstance(self.sla.output_v_target, (int, float))
      assert self.sla.output_v_target == V_CRUISE_UNSET or self.sla.output_v_target > 0

  def test_stale_data_handling(self):
    self.initialize_active_state(self.pcm_long_max_set_speed)
    old_speed_limit = SPEED_LIMITS['city']

    self.sla.update(True, False, SPEED_LIMITS['city'], 0, self.pcm_long_max_set_speed, 0, old_speed_limit, True, 0, self.events_sp)
    assert self.sla.state in ACTIVE_STATES
    assert self.sla.output_v_target == old_speed_limit

  def test_distance_based_adapting(self):
    self.sla.state = SpeedLimitAssistState.adapting
    self.sla.v_cruise_cluster_prev = self.pcm_long_max_set_speed
    self.sla.prev_v_cruise_cluster_conv = round(self.pcm_long_max_set_speed * self.speed_conv)

    distance = 100.0
    current_speed = SPEED_LIMITS['freeway']
    target_speed = SPEED_LIMITS['highway']

    self.sla.update(True, False, current_speed, 0, self.pcm_long_max_set_speed, target_speed, target_speed, True, distance, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.adapting
    assert self.sla.output_v_target == target_speed  # TODO-SP: assert expected accel, need to enable self.acceleration_solutions

  def test_long_disengaged_to_disabled(self):
    self.initialize_active_state(self.pcm_long_max_set_speed)

    self.sla.update(False, False, SPEED_LIMITS['city'], 0, self.pcm_long_max_set_speed, SPEED_LIMITS['city'],
                    SPEED_LIMITS['city'], True, 0, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.disabled
    assert self.sla.output_v_target == V_CRUISE_UNSET

  def test_maintain_states_with_no_changes(self):
    """Test that states are maintained when no significant changes occur"""
    test_states = [
      SpeedLimitAssistState.preActive,
      SpeedLimitAssistState.pending,
      SpeedLimitAssistState.active,
      SpeedLimitAssistState.adapting
    ]

    for state in test_states:
      self.sla.state = state
      self.sla.op_engaged = True

      initial_state = state

      self.sla.update(True, False, SPEED_LIMITS['city'], 0, self.pcm_long_max_set_speed, SPEED_LIMITS['city'], SPEED_LIMITS['city'], True, 0, self.events_sp)

      assert self.sla.state in ALL_STATES  # Sanity check

      if initial_state == SpeedLimitAssistState.preActive:
        assert self.sla.state in [SpeedLimitAssistState.preActive, SpeedLimitAssistState.active]
      elif initial_state in ACTIVE_STATES:
        assert self.sla.state in ACTIVE_STATES


class TestButtonStateTrackerSLAIntegration(OpenpilotTestCase):

  def setup_method(self):

    self.tracker = ButtonStateTracker()
    self.params = Params()
    self.params.put("IsReleaseSpBranch", True, block=True)
    self.params.put("SpeedLimitMode", int(Mode.assist), block=True)
    self.params.put_bool("IsMetric", False, block=True)
    self.params.put("SpeedLimitOffsetType", 0, block=True)
    self.params.put("SpeedLimitValueOffset", 0, block=True)

    CarInterface = interfaces[DEFAULT_CAR]
    CP = CarInterface.get_non_essential_params(DEFAULT_CAR)
    CP.openpilotLongitudinalControl = True
    CP_SP = CarInterface.get_non_essential_params_sp(CP, DEFAULT_CAR)
    self.sla = SpeedLimitAssist(CP, CP_SP)

  def _make_cs(self, events=None) -> car.CarState:
    CS = car.CarState()
    CS.buttonEvents = events or []
    return CS

  def _run_ctrl_frames(self, frames: list[car.CarState]) -> None:
    for cs in frames:
      self.tracker.update(cs)

  def test_button_confirm_via_tracker(self) -> None:
    self._run_ctrl_frames([
      self._make_cs([ButtonEvent(type=ButtonType.accelCruise, pressed=True)]),
      self._make_cs(),
      self._make_cs([ButtonEvent(type=ButtonType.accelCruise, pressed=False)]),
      self._make_cs(),
      self._make_cs(),
    ])
    self.sla.update_buttons(self.tracker.release_toggle)
    assert self.sla._get_button_release(req_plus=True, req_minus=False)

  def test_rapid_press_release_between_polls(self) -> None:
    self.sla.update_buttons(self.tracker.release_toggle)

    self._run_ctrl_frames([
      self._make_cs([ButtonEvent(type=ButtonType.decelCruise, pressed=True)]),
      self._make_cs([ButtonEvent(type=ButtonType.decelCruise, pressed=False)]),
      self._make_cs(),
      self._make_cs(),
      self._make_cs(),
    ])
    self.sla.update_buttons(self.tracker.release_toggle)
    assert self.sla._get_button_release(req_plus=False, req_minus=True)

  def test_multiple_releases_between_polls(self) -> None:
    self.sla.update_buttons(self.tracker.release_toggle)

    self._run_ctrl_frames([
      self._make_cs([
        ButtonEvent(type=ButtonType.accelCruise, pressed=True),
        ButtonEvent(type=ButtonType.decelCruise, pressed=True),
      ]),
      self._make_cs([
        ButtonEvent(type=ButtonType.accelCruise, pressed=False),
        ButtonEvent(type=ButtonType.decelCruise, pressed=False),
      ]),
    ])
    self.sla.update_buttons(self.tracker.release_toggle)
    assert self.sla._get_button_release(req_plus=True, req_minus=False)
    assert self.sla._get_button_release(req_plus=False, req_minus=True)

  def test_no_false_positive_same_toggle(self) -> None:
    self.sla.update_buttons(self.tracker.release_toggle)
    self.sla.update_buttons(self.tracker.release_toggle)
    assert not self.sla._get_button_release(req_plus=True, req_minus=False)
    assert not self.sla._get_button_release(req_plus=False, req_minus=True)

  def test_button_confirm_expires(self) -> None:
    self._run_ctrl_frames([
      self._make_cs([ButtonEvent(type=ButtonType.accelCruise, pressed=True)]),
      self._make_cs([ButtonEvent(type=ButtonType.accelCruise, pressed=False)]),
    ])
    self.sla.update_buttons(self.tracker.release_toggle)
    time.sleep(CRUISE_BUTTON_CONFIRM_HOLD + 0.1)
    assert not self.sla._get_button_release(req_plus=True, req_minus=False)


# --- fork: SLA auto-apply + override memory ------------------------------------------------------

MAX_AUTO_DECREASE_MPH = 15.0


class _SLAForkTestBase(OpenpilotTestCase):
  """fork: exercises the owner's NON-pcm (pcmCruise=False) path, which is what the Elantra N runs."""
  car_name = DEFAULT_CAR

  def setup_method(self):
    self.params = Params()
    self.reset_custom_params()
    self.events_sp = EventsSP()
    CI = self._setup_platform(self.car_name)
    self.sla = SpeedLimitAssist(CI.CP, CI.CP_SP)
    self.sla.pcm_op_long = False  # owner's car: openpilot long on a pcmCruise=False Hyundai
    self.speed_conv = CV.MS_TO_MPH

  def teardown_method(self):
    pass

  def _setup_platform(self, car_name):
    CarInterface = interfaces[car_name]
    CP = CarInterface.get_non_essential_params(car_name)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)
    CI = CarInterface(CP, CP_SP)
    CI.CP.openpilotLongitudinalControl = True
    sunnypilot_interfaces.setup_interfaces(CI, self.params)
    return CI

  def reset_custom_params(self):
    self.params.put("IsReleaseSpBranch", True, block=True)
    self.params.put("SpeedLimitMode", int(Mode.assist), block=True)
    self.params.put_bool("IsMetric", False, block=True)
    self.params.put("SpeedLimitOffsetType", 0, block=True)
    self.params.put("SpeedLimitValueOffset", 0, block=True)

  def _drive(self, vc_mph, limit_mph, sll_mph=None, state=None, v_ego_mph=10.0):
    """One non-pcm frame. vc_mph is the set speed the driver/system shows; sll_mph the (offset)
    target SLA drives to; limit_mph the raw limit (for speed_limit_changed)."""
    if state is not None:
      self.sla.state = state
    self.sla.update(True, False,
                    v_ego_mph * CV.MPH_TO_MS, 0.,
                    vc_mph * CV.MPH_TO_MS,
                    limit_mph * CV.MPH_TO_MS,
                    (sll_mph if sll_mph is not None else limit_mph) * CV.MPH_TO_MS,
                    True, 0., self.events_sp)

  def _init_active(self, vc_mph, limit_mph, sll_mph=None):
    self._drive(vc_mph, limit_mph, sll_mph=sll_mph, state=SpeedLimitAssistState.active)
    # second frame settles the change tracking so prev == current (no spurious edge)
    self._drive(vc_mph, limit_mph, sll_mph=sll_mph)

  def _prime_override(self, vc_mph=35.0, limit_mph=35.0):
    """Active at the limit, then the driver bumps +5: genuine manual edge -> arms override memory."""
    self._init_active(vc_mph, limit_mph)
    self._drive(vc_mph + 5.0, limit_mph)  # driver edit; limit unchanged
    return self.sla


class TestSpeedLimitAssistConfirmPureFunction(_SLAForkTestBase):
  """A: the confirm decision is a pure function of (current set speed, target set speed) and does not
  depend on the vehicle speed at all. Falsifiable contrast: on the pre-change code there is no
  `target_set_speed_change_requires_confirm` at all (AttributeError), and `apply_confirm_speed_threshold`
  returned True whenever v_cruise_cluster < 50 mph."""

  def _confirm(self, vc_mph, tgt_mph, is_metric=False):
    self.sla.is_metric = is_metric
    self.sla.v_cruise_cluster_conv = round(vc_mph)
    self.sla.target_set_speed_conv = round(tgt_mph)
    return self.sla.target_set_speed_change_requires_confirm

  @parameterized.expand([
    # (name, v_cruise, target, expect_press)   -- imperial (mph)
    ("increase_below_50", 35, 50, False),       # old: PRESS (vc<50) -> new: auto
    ("increase_below_50_small", 30, 40, False),  # old: PRESS -> new: auto
    ("increase_above_50", 65, 80, False),        # both auto
    ("decrease_in_cap_below_50", 35, 25, False),  # -10 within cap, old: PRESS -> new: auto
    ("decrease_at_cap", 35, 20, False),          # exactly -15: still auto (cap is strict >)
    ("decrease_beyond_cap_below_50", 35, 15, True),   # -20: still asks
    ("decrease_in_cap_above_50", 60, 55, False),  # both auto
    ("decrease_above_50_old_autoed", 70, 50, True),   # old: auto, new: ASKS (the new friction)
    ("decrease_beyond_cap_above_50", 60, 40, True),   # both ask
    ("no_change_increase_zero", 40, 40, False),
  ])
  def test_confirm_is_pure_capped_decrease(self, name, vc, tgt, expect_press):
    assert self._confirm(vc, tgt) is expect_press

  @parameterized.expand([
    ("metric_increase", 60, 100, False),     # +40 km/h
    ("metric_decrease_in_cap", 100, 80, False),  # -20 <= 24
    ("metric_decrease_beyond_cap", 110, 60, True),  # -50 > 24
  ])
  def test_confirm_metric_cap_is_24_kph(self, name, vc, tgt, expect_press):
    assert self._confirm(vc, tgt, is_metric=True) is expect_press

  def test_gate_does_not_depend_on_vehicle_speed(self):
    """The same (vc, target) must decide identically regardless of the raw cluster/vehicle speed -
    the whole point of A. v_cruise_cluster_conv is the set speed; assert the old helper would have
    flipped on it while the new one does not."""
    self.sla.target_set_speed_conv = 50
    self.sla.v_cruise_cluster_conv = 40
    assert not self.sla.target_set_speed_change_requires_confirm  # 50 > 40-15 -> auto
    # old rule would have required a press purely because 40 < 50 mph
    assert self.sla.v_cruise_cluster_below_confirm_speed_threshold


class TestSpeedLimitAssistAutoApplyStateMachine(_SLAForkTestBase):
  """A: end-to-end through the non-pcm state machine (no press anywhere)."""

  def test_increase_below_50_auto_applies_no_press(self):
    self._init_active(35.0, 35.0)          # active at 35 mph
    # limit 35 -> 50, car still showing a set speed of 35 (below the old 50 mph confirm floor)
    self._drive(35.0, 50.0, sll_mph=50.0)
    assert self.sla.state in ACTIVE_STATES, self.sla.state        # old code: preActive (a press)
    assert abs(self.sla.output_v_target - 50.0 * CV.MPH_TO_MS) < 1e-3

  def test_small_decrease_below_50_auto_applies_no_press(self):
    self._init_active(40.0, 40.0)
    self._drive(40.0, 30.0, sll_mph=30.0)  # -10, within the 15 cap
    assert self.sla.state in ACTIVE_STATES, self.sla.state        # old code: preActive (a press)

  def test_decrease_beyond_cap_still_asks(self):
    self._init_active(40.0, 40.0)
    self._drive(40.0, 20.0, sll_mph=20.0)  # -20, beyond the cap
    assert self.sla.state == SpeedLimitAssistState.preActive  # the request-for-press state

  def test_increase_above_50_auto_applies(self):
    self._init_active(55.0, 55.0)
    self._drive(55.0, 65.0, sll_mph=65.0)
    assert self.sla.state in ACTIVE_STATES, self.sla.state


class TestSpeedLimitAssistOffsetApplies(_SLAForkTestBase):
  """A: the configured offset flows into the target set speed as before (limit + offset)."""

  def test_offset_is_included_in_target_and_gate(self):
    self.sla.is_metric = False
    # active with the offset target: raw limit 35 + 5 offset -> set speed 40, SLA active at 40
    self._init_active(40.0, 35.0, sll_mph=40.0)
    assert abs(self.sla.output_v_target - 40.0 * CV.MPH_TO_MS) < 1e-3
    # limit drops 35 -> 30 (offset target 40 -> 35): a 5 mph decrease, within the cap -> auto-apply
    self._drive(40.0, 30.0, sll_mph=35.0)
    assert self.sla.state in ACTIVE_STATES
    assert abs(self.sla.output_v_target - 35.0 * CV.MPH_TO_MS) < 1e-3
    # the gate uses the offset target (35), not the raw limit (30): 35 > 40 - 15 -> auto
    self.sla._speed_limit_final_last = 35.0 * CV.MPH_TO_MS
    self.sla.update_calculations(40.0 * CV.MPH_TO_MS)
    assert self.sla.target_set_speed_conv == 35
    assert self.sla.v_cruise_cluster_conv == 40
    assert not self.sla.target_set_speed_change_requires_confirm


class TestSpeedLimitAssistOverrideMemory(_SLAForkTestBase):
  """B: a manual set-speed override survives a limit change until OVERRIDE_MEMORY_S has elapsed, and
  SLA's own writes never arm it. Falsifiable contrast: pre-change, one limit change cleared it."""

  def test_driver_change_arms_override(self):
    self._prime_override(35.0, 35.0)
    assert self.sla._override_tripped
    assert self.sla.override_memory_active

  def test_override_holds_through_limit_change_within_window(self):
    self._prime_override(35.0, 35.0)
    assert self.sla.state == SpeedLimitAssistState.inactive
    # a limit change a few seconds later (fast-changing area) must NOT re-engage SLA
    self._drive(40.0, 45.0, sll_mph=45.0)
    assert self.sla.state == SpeedLimitAssistState.inactive
    assert self.sla.override_memory_active

  def test_override_releases_after_window(self):
    self._prime_override(35.0, 35.0)
    self._drive(40.0, 45.0, sll_mph=45.0)          # within window -> held
    assert self.sla.state == SpeedLimitAssistState.inactive
    # simulate OVERRIDE_MEMORY_S elapsing, then another limit change -> releases
    base = self.sla._override_ts
    with unittest.mock.patch("openpilot.sunnypilot.selfdrive.controls.lib.speed_limit."
                             "speed_limit_assist.time.monotonic", return_value=base + OVERRIDE_MEMORY_S + 1.0):
      self._drive(40.0, 55.0, sll_mph=55.0)
    assert not self.sla.override_memory_active
    assert self.sla.state == SpeedLimitAssistState.preActive

  def test_sla_own_write_does_not_arm_override(self):
    """SLA auto-applies 35 -> 45 and the cluster then shows 45 == target. That self-write must not be
    mistaken for a driver edit."""
    self._init_active(35.0, 35.0)
    self._drive(45.0, 45.0, sll_mph=45.0)  # cluster jumped to the target (SLA's own write)
    assert not self.sla._override_tripped
    assert not self.sla.override_memory_active

  def test_driver_change_away_from_target_arms(self):
    """Same jump in set speed, but to a value != target -> a genuine driver edit -> arms."""
    self._init_active(35.0, 35.0)
    self._drive(50.0, 35.0, sll_mph=35.0)  # driver +15, target stays 35
    assert self.sla._override_tripped
    assert self.sla.override_memory_active

  def test_manual_change_while_disabled_arms(self):
    self.sla.state = SpeedLimitAssistState.disabled
    self._drive(80.0, 35.0, sll_mph=35.0)   # first frame: prev cluster 0 -> not an edge
    self._drive(45.0, 35.0, sll_mph=35.0)   # driver edit while disabled -> edge
    assert self.sla._override_tripped

  def test_confirm_press_clears_override(self):
    self._prime_override(35.0, 35.0)
    assert self.sla.override_memory_active
    # driver presses to confirm the new limit in preActive -> SLA owns the set speed
    self.sla.state = SpeedLimitAssistState.preActive
    self._drive(45.0, 45.0, sll_mph=45.0)
    assert not self.sla._override_tripped
