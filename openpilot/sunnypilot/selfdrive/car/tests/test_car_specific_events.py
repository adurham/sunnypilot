"""
fork: Hyundai comma-pedal (gas interceptor) low-speed / lockout events in CarSpecificEventsSP.

belowEngageSpeed (+ the audible pedalBelowEngageSpeed alert, since MADS strips belowEngageSpeed) only on a SET/RES engage
attempt below minEnableSpeed; never for the pause/resume press (green-light resume). pedalFactoryCruiseLockout whenever
the factory MAIN is armed (nonAdaptive). The low-speed pedal cut and its manualRestart warning are gone.
"""
from types import SimpleNamespace

import pytest

from opendbc.car import structs
from openpilot.cereal import log, custom
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.selfdrive.car.car_specific import CarSpecificEventsSP

EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName
ButtonType = structs.CarState.ButtonEvent.Type

MIN_ENABLE_SPEED = 11.2  # m/s (~25 mph)


def _run(v_ego: float, long_active: bool = False, interceptor: bool = True, button_enable: bool = False,
         resume: bool = False, non_adaptive: bool = False):
  CP = SimpleNamespace(brand='hyundai', carFingerprint='HYUNDAI_ELANTRA_2021', minEnableSpeed=MIN_ENABLE_SPEED,
                       openpilotLongitudinalControl=True)
  CP_SP = SimpleNamespace(enableGasInterceptor=interceptor)
  be = [SimpleNamespace(type=ButtonType.resumeCruise, pressed=p) for p in (True, False)] if resume else \
       [SimpleNamespace(type=ButtonType.decelCruise, pressed=False)]
  CS = SimpleNamespace(vEgo=v_ego, buttonEvents=be, cruiseState=SimpleNamespace(nonAdaptive=non_adaptive))
  events = Events()
  if button_enable:
    events.add(EventName.buttonEnable)
  events_sp = CarSpecificEventsSP(CP, CP_SP).update(CS, events, long_active)
  return events, events_sp


class TestPedalLowSpeedEvents:
  @pytest.mark.parametrize("v_ego", [0.0, 5.0, MIN_ENABLE_SPEED - 0.5])
  @pytest.mark.parametrize("long_active", [False, True])
  def test_no_takeover_warning_any_more(self, v_ego, long_active):
    events, _ = _run(v_ego, long_active)
    assert not events.has(EventName.manualRestart)

  @pytest.mark.parametrize("v_ego", [0.0, 5.0, MIN_ENABLE_SPEED - 0.5])
  def test_set_below_engage_speed_refused_with_alert(self, v_ego):
    events, events_sp = _run(v_ego, button_enable=True)
    assert events.has(EventName.belowEngageSpeed)
    assert events_sp.has(EventNameSP.pedalBelowEngageSpeed)

  def test_set_above_engage_speed(self):
    events, events_sp = _run(MIN_ENABLE_SPEED + 1., button_enable=True)
    assert not events.has(EventName.belowEngageSpeed) and not events_sp.has(EventNameSP.pedalBelowEngageSpeed)

  @pytest.mark.parametrize("v_ego", [0.0, 3.0, MIN_ENABLE_SPEED - 0.5])
  def test_pause_resume_exempt_from_floor(self, v_ego):
    events, events_sp = _run(v_ego, button_enable=True, resume=True)
    assert not events.has(EventName.belowEngageSpeed) and not events_sp.has(EventNameSP.pedalBelowEngageSpeed)

  def test_no_button_no_events(self):
    events, events_sp = _run(0., button_enable=False)
    assert not events.has(EventName.belowEngageSpeed) and len(events_sp) == 0

  def test_factory_main_lockout_alert(self):
    _, events_sp = _run(25., non_adaptive=True)
    assert events_sp.has(EventNameSP.pedalFactoryCruiseLockout)
    _, events_sp = _run(25., non_adaptive=False)
    assert not events_sp.has(EventNameSP.pedalFactoryCruiseLockout)

  @pytest.mark.parametrize("button_enable", [False, True])
  def test_feature_off_no_events(self, button_enable):
    events, events_sp = _run(0., interceptor=False, button_enable=button_enable, non_adaptive=True)
    assert not events.has(EventName.manualRestart) and not events.has(EventName.belowEngageSpeed)
    assert len(events_sp) == 0
