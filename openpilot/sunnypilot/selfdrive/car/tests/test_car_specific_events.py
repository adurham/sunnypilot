"""
fork: Hyundai comma-pedal (gas interceptor) events in CarSpecificEventsSP (buttons-v3).

There is no engage floor any more: no belowEngageSpeed and no pedalBelowEngageSpeed refusal beep at any speed, for any
button; no low-speed takeover warning (manualRestart). 'Press Set to Engage' (resumeBlocked) is removed in pedal mode,
because pause/resume is the only on switch and engages at the current speed when no set speed exists yet.
pedalFactoryCruiseLockout whenever the factory MAIN is armed (nonAdaptive).
"""
from types import SimpleNamespace

import pytest

from opendbc.car import structs
from openpilot.cereal import log, custom
from openpilot.selfdrive.selfdrived.events import ET, Events
from openpilot.sunnypilot.selfdrive.selfdrived.events_base import Priority
from openpilot.sunnypilot.selfdrive.car.car_specific import CarSpecificEventsSP

EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName
ButtonType = structs.CarState.ButtonEvent.Type

OLD_FLOOR = 11.2  # m/s (~25 mph): the pre-buttons-v3 SET/RES floor, kept here only to probe around it


def _run(v_ego: float, long_active: bool = False, interceptor: bool = True, button_enable: bool = False,
         button=ButtonType.decelCruise, non_adaptive: bool = False, resume_blocked: bool = False, min_enable=-1.):
  CP = SimpleNamespace(brand='hyundai', carFingerprint='HYUNDAI_ELANTRA_2021', minEnableSpeed=min_enable,
                       openpilotLongitudinalControl=True)
  CP_SP = SimpleNamespace(enableGasInterceptor=interceptor, fca11Brake=False)
  CS_SP = SimpleNamespace()   # CarSpecificEventsSP reads only a few SP fields; fca11Brake is False above
  be = [SimpleNamespace(type=button, pressed=p) for p in (True, False)]
  CS = SimpleNamespace(vEgo=v_ego, buttonEvents=be, cruiseState=SimpleNamespace(nonAdaptive=non_adaptive))
  events = Events()
  if button_enable:
    events.add(EventName.buttonEnable)
  if resume_blocked:
    events.add(EventName.resumeBlocked)
  events_sp = CarSpecificEventsSP(CP, CP_SP).update(CS, CS_SP, events, long_active)
  return events, events_sp


class TestPedalEvents:
  @pytest.mark.parametrize("v_ego", [0.0, 5.0, OLD_FLOOR - 0.5])
  @pytest.mark.parametrize("long_active", [False, True])
  def test_no_takeover_warning(self, v_ego, long_active):
    events, _ = _run(v_ego, long_active)
    assert not events.has(EventName.manualRestart)

  @pytest.mark.parametrize("v_ego", [0.0, 0.5, 5.0, OLD_FLOOR - 0.5, OLD_FLOOR + 1., 30.])
  @pytest.mark.parametrize("button", [ButtonType.resumeCruise, ButtonType.accelCruise, ButtonType.decelCruise])
  @pytest.mark.parametrize("min_enable", [-1., OLD_FLOOR])
  def test_no_engage_floor_no_refusal_beep(self, v_ego, button, min_enable):
    # buttons-v3: no speed floor and no SET/RES refusal alert, whatever the button and speed. Also with a stale
    # CarParams that still carries the old 25 mph minEnableSpeed: the floor code itself is gone, not just the constant.
    events, events_sp = _run(v_ego, button_enable=True, button=button, min_enable=min_enable)
    assert not events.has(EventName.belowEngageSpeed)
    assert not events_sp.has(EventNameSP.pedalBelowEngageSpeed)
    assert events.has(EventName.buttonEnable)

  @pytest.mark.parametrize("v_ego", [0.0, 5.0, 25.])
  def test_resume_blocked_removed(self, v_ego):
    # pause/resume with no set speed yet engages at the current speed (cruise.py): 'Press Set to Engage' can't apply
    events, _ = _run(v_ego, button_enable=True, button=ButtonType.resumeCruise, resume_blocked=True)
    assert not events.has(EventName.resumeBlocked)

  def test_resume_blocked_kept_without_interceptor(self):
    events, _ = _run(0., interceptor=False, button=ButtonType.accelCruise, resume_blocked=True)
    assert events.has(EventName.resumeBlocked)

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


class TestFca11CameraHandbackEvents:
  """fork (0053): the camera hand-back is TRANSIENT (fca11CameraBraking) and no longer the permanent fca11Unavailable;
  a Python <-> panda disagreement raises fca11Divergence, which takes precedence over the transient notice."""

  def _run(self, unavailable=False, camera=False, divergence=False, armed=True):
    CP = SimpleNamespace(brand='hyundai', carFingerprint='HYUNDAI_ELANTRA_2021', minEnableSpeed=-1.,
                         openpilotLongitudinalControl=True)
    CP_SP = SimpleNamespace(enableGasInterceptor=True, fca11Brake=armed)
    CS_SP = SimpleNamespace(brakeNow=False, stopComplete=False, fca11Unavailable=unavailable,
                            fca11CameraBraking=camera, fca11Divergence=divergence)
    CS = SimpleNamespace(vEgo=20., buttonEvents=[], cruiseState=SimpleNamespace(nonAdaptive=False))
    return CarSpecificEventsSP(CP, CP_SP).update(CS, CS_SP, Events(), True)

  def test_camera_braking_is_transient_not_unavailable(self):
    ev = self._run(camera=True)
    assert ev.has(EventNameSP.fca11CameraBraking)
    assert not ev.has(EventNameSP.fca11Unavailable)

  def test_divergence_wins_over_camera_braking(self):
    ev = self._run(camera=True, divergence=True)
    assert ev.has(EventNameSP.fca11Divergence)
    assert not ev.has(EventNameSP.fca11CameraBraking)

  def test_unavailable_still_raised_for_its_own_causes(self):
    ev = self._run(unavailable=True)
    assert ev.has(EventNameSP.fca11Unavailable)
    assert not ev.has(EventNameSP.fca11CameraBraking)

  def test_nothing_when_not_armed(self):
    ev = self._run(camera=True, divergence=True, unavailable=True, armed=False)
    for e in (EventNameSP.fca11CameraBraking, EventNameSP.fca11Divergence, EventNameSP.fca11Unavailable):
      assert not ev.has(e)

  def test_alert_definitions(self):
    from openpilot.selfdrive.selfdrived.events import ET, Alert
    from openpilot.sunnypilot.selfdrive.selfdrived.events import EVENTS_SP
    assert ET.WARNING in EVENTS_SP[EventNameSP.fca11CameraBraking]
    assert ET.PERMANENT in EVENTS_SP[EventNameSP.fca11Divergence]
    # drive 16a (2026-10-25): the condition cleared itself 170.5 s later on the SAME ignition, so the old second line
    # ("off until restart") over-claimed. The text must now scope the loss to the time the banner is up.
    unavailable = EVENTS_SP[EventNameSP.fca11Unavailable][ET.PERMANENT]
    assert isinstance(unavailable, Alert)  # a plain alert, not a callback: the text is fixed at import
    assert unavailable.alert_text_2 == "openpilot braking is off while this shows"
    assert "until restart" not in unavailable.alert_text_2


class TestFca11SuperviseStopRemoved:
  """fork (drive 0000016a, 2026-10-25): "supervise stop" fired 12 times in a 10-minute drive and told the owner nothing
  he did not already know, so the alert is DELETED. CarStateSP.superviseStop is still published (the opendbc half keeps
  it in the log for analysis); the car layer must no longer turn it into any event. brakeNow / stopComplete remain."""

  FCA11_EVENTS = (EventNameSP.fca11BrakeNow, EventNameSP.fca11StopComplete, EventNameSP.fca11Unavailable,
                  EventNameSP.fca11CameraBraking, EventNameSP.fca11Divergence)

  def _run(self, stop_complete=False):
    # vEgo 5 m/s (18 km/h) is inside the old "braking below 15 km/h" window, so this call would have raised the banner.
    # superviseStop=True is set on every call: the flag on its own must raise nothing.
    CP = SimpleNamespace(brand='hyundai', carFingerprint='HYUNDAI_ELANTRA_2021', minEnableSpeed=-1.,
                         openpilotLongitudinalControl=True)
    CP_SP = SimpleNamespace(enableGasInterceptor=True, fca11Brake=True)
    CS_SP = SimpleNamespace(brakeNow=False, stopComplete=stop_complete, superviseStop=True,
                            fca11Unavailable=False, fca11CameraBraking=False, fca11Divergence=False)
    CS = SimpleNamespace(vEgo=5., buttonEvents=[], cruiseState=SimpleNamespace(nonAdaptive=False))
    return CarSpecificEventsSP(CP, CP_SP).update(CS, CS_SP, Events(), True)

  def test_supervise_stop_raises_no_fca11_event(self):
    ev = self._run()
    assert not any(ev.has(e) for e in self.FCA11_EVENTS)
    assert len(ev) == 0  # nothing at all in this state: no buttons, nonAdaptive False

  def test_stop_complete_still_raised_with_supervise_stop_set(self):
    ev = self._run(stop_complete=True)
    assert ev.has(EventNameSP.fca11StopComplete)
    assert not ev.has(EventNameSP.fca11BrakeNow)

  def test_enum_still_in_cereal_but_has_no_alert_definition(self):
    # the enum ordinal stays reserved in custom.capnp (the log keeps the flag); only the alert dict entry is gone
    from openpilot.sunnypilot.selfdrive.selfdrived.events import EVENTS_SP
    assert EventNameSP.fca11SuperviseStop not in EVENTS_SP


class TestFca11HoldNagEvents:
  """fork (0043): the standstill HOLD COUNTDOWN v2. CarStateSP.holdNag (0 none / 1 soft 60 s / 2 loud 90 s / 3 final
  110 s / 4 EXPIRED 120 s) sits BETWEEN the existing alerts: brakeNow outranks every TIER, the nag replaces
  stopComplete while it is up, and stopComplete still covers every hold shorter than 60 s - so the screen never shows
  two hold banners at once. 4 is not a tier: it is the latch, and it is the only state that acts - at 2:00 longitudinal
  control disengages and the brakes release; lateral (MADS) stays engaged by design - and it refuses a longitudinal
  re-engage until the driver brakes. The state-machine proof is in tests/test_fca11_hold_countdown.py."""

  def _run(self, hold_nag=0, brake_now=False, stop_complete=False, armed=True, button_enable=False):
    CP = SimpleNamespace(brand='hyundai', carFingerprint='HYUNDAI_ELANTRA_2021', minEnableSpeed=-1.,
                         openpilotLongitudinalControl=True)
    CP_SP = SimpleNamespace(enableGasInterceptor=True, fca11Brake=armed)
    CS_SP = SimpleNamespace(brakeNow=brake_now, stopComplete=stop_complete, superviseStop=False, holdNag=hold_nag,
                            fca11Unavailable=False, fca11CameraBraking=False, fca11Divergence=False)
    CS = SimpleNamespace(vEgo=0., buttonEvents=[], cruiseState=SimpleNamespace(nonAdaptive=False))
    events = Events()
    if button_enable:
      events.add(EventName.buttonEnable)
    return events, CarSpecificEventsSP(CP, CP_SP).update(CS, CS_SP, events, False)

  @pytest.mark.parametrize("nag,event", [(1, EventNameSP.fca11HoldNagSoft), (2, EventNameSP.fca11HoldNagLoud),
                                           (3, EventNameSP.fca11HoldNagFinal)])
  def test_each_tier_raises_exactly_its_own_alert_and_never_disengages(self, nag, event):
    events, ev_sp = self._run(hold_nag=nag, stop_complete=True)
    assert ev_sp.has(event)
    others = {EventNameSP.fca11HoldNagSoft, EventNameSP.fca11HoldNagLoud, EventNameSP.fca11HoldNagFinal,
              EventNameSP.fca11HoldExpired} - {event}
    assert not any(ev_sp.has(o) for o in others), "one tier at a time"
    assert not ev_sp.has(EventNameSP.fca11StopComplete), "the nag replaces the plain hand-off banner"
    # a TIER never disengages: no standard event is written and nothing is stripped
    assert events.names == [], "a nag tier must not touch the standard events"
    assert not ev_sp.has(EventNameSP.fca11HoldExpired)

  def test_brake_now_outranks_every_tier(self):
    for nag in (1, 2, 3):
      _events, ev_sp = self._run(hold_nag=nag, stop_complete=True, brake_now=True)
      assert ev_sp.has(EventNameSP.fca11BrakeNow)
      assert not any(ev_sp.has(e) for e in (EventNameSP.fca11HoldNagSoft, EventNameSP.fca11HoldNagLoud,
                                            EventNameSP.fca11HoldNagFinal, EventNameSP.fca11HoldExpired))

  def test_expiry_acts_disengage_and_refuse_a_new_engage(self):
    events, ev_sp = self._run(hold_nag=4, stop_complete=True, button_enable=True)
    assert ev_sp.has(EventNameSP.fca11HoldExpired)
    assert not ev_sp.has(EventNameSP.fca11StopComplete)
    # the disengage: USER_DISABLE on the STANDARD object the state machine reads
    assert events.has(EventName.pcmDisable)
    assert events.contains(ET.USER_DISABLE)
    # and no ENABLE survives the latched frame, so the disabled branch cannot re-engage on the same frame
    assert not events.has(EventName.buttonEnable)
    assert not events.has(EventName.pcmEnable)

  def test_expiry_is_not_suppressed_by_brake_now(self):
    # the brakes are coming off whatever else is true - brakeNow must not be able to hide the release
    events, ev_sp = self._run(hold_nag=4, brake_now=True, button_enable=True)
    assert ev_sp.has(EventNameSP.fca11HoldExpired)
    assert events.has(EventName.pcmDisable)

  def test_the_brake_press_clears_everything(self):
    events, ev_sp = self._run(hold_nag=0, button_enable=True)
    assert not ev_sp.has(EventNameSP.fca11HoldExpired)
    assert not events.has(EventName.pcmDisable)
    assert events.has(EventName.buttonEnable), "with the latch clear the engage is allowed again"
    # below the soft threshold nothing changes: the existing hand-off banner is still the one that shows
    assert self._run(hold_nag=0, stop_complete=True)[1].has(EventNameSP.fca11StopComplete)

  def test_nothing_when_the_feature_is_off(self):
    for nag in (0, 1, 2, 3, 4):
      events, ev_sp = self._run(hold_nag=nag, armed=False, button_enable=True)
      assert ev_sp.names == [], "OFF must be inert (the deployed path)"
      assert events.names == [EventName.buttonEnable], "OFF must not add or strip anything"

  def test_alert_definitions(self):
    """The four alerts, by VALUE: who is loud, who outranks whom, and what the words must say now that the hold ends."""
    from openpilot.sunnypilot.selfdrive.selfdrived.events import EVENTS_SP
    soft = EVENTS_SP[EventNameSP.fca11HoldNagSoft][ET.WARNING]
    loud = EVENTS_SP[EventNameSP.fca11HoldNagLoud][ET.WARNING]
    final = EVENTS_SP[EventNameSP.fca11HoldNagFinal][ET.WARNING]
    expired_perm = EVENTS_SP[EventNameSP.fca11HoldExpired][ET.PERMANENT]
    expired_ne = EVENTS_SP[EventNameSP.fca11HoldExpired][ET.NO_ENTRY]
    brake_now = EVENTS_SP[EventNameSP.fca11BrakeNow][ET.PERMANENT]

    # escalation, by value
    assert soft.priority == Priority.LOW
    assert loud.priority == Priority.HIGH
    assert final.priority == Priority.HIGHEST
    assert expired_perm.priority == Priority.HIGHEST
    assert soft.audible_alert == log.SelfdriveState.AudibleAlert.none, "60 s is the SILENT banner"
    assert loud.audible_alert == log.SelfdriveState.AudibleAlert.warningSoft
    assert final.audible_alert == log.SelfdriveState.AudibleAlert.warningImmediate
    assert expired_perm.audible_alert == log.SelfdriveState.AudibleAlert.warningImmediate
    # a real brake fault still outranks the loudest TIER (expiry is a different thing - it is not a nag)
    assert loud.priority < brake_now.priority and final.priority <= brake_now.priority

    # the words: PRESS the brake, and WHEN the brakes go. The old 0041 claims are now lies and must be gone.
    for a in (soft, loud, final, expired_perm):
      text = (a.alert_text_1 + " " + a.alert_text_2).lower()
      assert "press" in text and "brake" in text, text
      for dead in ("2 minutes", "over 2 min", "so long", "keep holding"):
        assert dead not in text, f"{dead!r} is the OLD (never-release) wording: {text}"
    assert "2:00" in (soft.alert_text_2 + loud.alert_text_2).lower(), "the release time is stated where the driver reads it"
    # CD3 (owner): the 2:00 is measured from the DEAD STOP, and the driver is told so
    for a in (soft, loud):
      assert "stop" in a.alert_text_2.lower(), \
             f"the countdown starts at a dead stop - the alert must say so: {a.alert_text_2!r}"
    assert "released" in expired_perm.alert_text_1.lower(), "at 2:00 the banner must say the brakes ARE released"
    # NoEntryAlert puts the reason in the second line and "openpilot Unavailable" first (swapped on mici)
    assert "press the brake pedal first" in (expired_ne.alert_text_1 + " " + expired_ne.alert_text_2).lower(), \
           "and what to do about it"
    # D4: a MID alert's second line is drawn (upstream rule: small => text_2 empty, mid => non-empty)
    assert soft.alert_size == log.SelfdriveState.AlertSize.mid and soft.alert_text_2
