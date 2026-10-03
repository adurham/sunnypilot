"""
fork: Hyundai comma-pedal (gas interceptor) low-speed events in CarSpecificEventsSP.

manualRestart (pedal-cut takeover warning) must only fire while longitudinal is actually engaged
(carControl.longActive); belowEngageSpeed only on a SET/RES engage attempt below minEnableSpeed.
"""
from types import SimpleNamespace

import pytest

from opendbc.sunnypilot.car.hyundai.gas_interceptor import GAS_CUT_HYSTERESIS
from openpilot.cereal import log
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.selfdrive.car.car_specific import CarSpecificEventsSP

EventName = log.OnroadEvent.EventName

MIN_ENABLE_SPEED = 11.2  # m/s (~25 mph)
BELOW_CUT = MIN_ENABLE_SPEED - GAS_CUT_HYSTERESIS - 1.0
BETWEEN = MIN_ENABLE_SPEED - GAS_CUT_HYSTERESIS / 2  # below minEnableSpeed, above the cut
ABOVE = MIN_ENABLE_SPEED + 1.0


def _run(v_ego: float, long_active: bool, interceptor: bool = True, button_enable: bool = False) -> Events:
  CP = SimpleNamespace(brand='hyundai', carFingerprint='HYUNDAI_ELANTRA_2021', minEnableSpeed=MIN_ENABLE_SPEED,
                       openpilotLongitudinalControl=True)
  CP_SP = SimpleNamespace(enableGasInterceptor=interceptor)
  CS = SimpleNamespace(vEgo=v_ego)
  events = Events()
  if button_enable:
    events.add(EventName.buttonEnable)
  CarSpecificEventsSP(CP, CP_SP).update(CS, events, long_active)
  return events


class TestPedalLowSpeedEvents:
  def test_hysteresis_positive(self):
    assert GAS_CUT_HYSTERESIS > 0

  @pytest.mark.parametrize("v_ego", [0.0, BELOW_CUT])
  def test_armed_not_engaged_below_cut_no_takeover(self, v_ego):
    events = _run(v_ego, long_active=False)
    assert not events.has(EventName.manualRestart)
    assert not events.has(EventName.belowEngageSpeed)

  @pytest.mark.parametrize("v_ego", [0.0, BELOW_CUT])
  def test_armed_engaged_below_cut_takeover(self, v_ego):
    assert _run(v_ego, long_active=True).has(EventName.manualRestart)

  @pytest.mark.parametrize("v_ego", [BETWEEN, ABOVE])
  def test_armed_engaged_above_cut_no_takeover(self, v_ego):
    events = _run(v_ego, long_active=True)
    assert not events.has(EventName.manualRestart)
    assert not events.has(EventName.belowEngageSpeed)

  @pytest.mark.parametrize("long_active", [False, True])
  def test_below_engage_speed_only_on_button_enable(self, long_active):
    for v in (BELOW_CUT, BETWEEN):
      assert _run(v, long_active, button_enable=True).has(EventName.belowEngageSpeed)
      assert not _run(v, long_active, button_enable=False).has(EventName.belowEngageSpeed)
    assert not _run(ABOVE, long_active, button_enable=True).has(EventName.belowEngageSpeed)

  @pytest.mark.parametrize("long_active", [False, True])
  @pytest.mark.parametrize("button_enable", [False, True])
  def test_feature_off_no_events(self, long_active, button_enable):
    events = _run(BELOW_CUT, long_active, interceptor=False, button_enable=button_enable)
    assert not events.has(EventName.manualRestart)
    assert not events.has(EventName.belowEngageSpeed)
