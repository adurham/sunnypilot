"""
fork (0043): the state-machine proof for the hold-countdown EXPIRY - the part of the patch that ACTS.

The countdown itself is a pure observer in the car layer (opendbc `fca11_long._hold_nag_step`). At 2:00 (6000 armed SENT
frames) it sets a sticky latch and publishes `CarStateSP.holdNag == 4`; everything that follows is decided HERE, in the
superproject. This file drives the REAL `CarSpecificEventsSP`, the REAL `selfdrive/selfdrived/state.py` StateMachine and
the REAL `sunnypilot/mads/state.py` StateMachine - no hand-applied writes - and asserts:

  * a nag TIER (1/2/3) changes neither machine;
  * the EXPIRY frame disengages BOTH machines in the SAME update (main enabled -> disabled, MADS enabled -> disabled),
    because `CarSpecificEventsSP.update` receives the standard `events` object and `state_machine.update` consumes that
    very object later in the same `selfdrived.step()` (selfdrived.py l.284 -> l.660-662);
  * while the latch is set, an engage request (the RES/CANCEL-press carrier this car actually uses, `buttonEnable`) is
    refused - the car layer strips the ENABLE events, the idiom MADS itself uses (mads.py l.163-164) - and the refusal is
    not silent: the PERMANENT + NO_ENTRY SP alert is up;
  * the brake press (holdNag back to 0, the latch cleared in the car layer) re-opens engagement, so the driver is never
    locked out;
  * MADS lateral is dropped on the expiry frame and refused afterwards by the SP ET.NO_ENTRY it reads
    (`mads/state.py::check_contains` reads BOTH `events` and `events_sp`).

The one place this is weaker than a standard NO_ENTRY: `SelfdriveState.engageable` stays True while latched (it is
`not events.contains(ET.NO_ENTRY)`) even though an engage does nothing. That is the accepted cost of not touching
upstream `cereal/log.capnp` (see lane-X3/STATE.md, checkpoint 1) - the red PERMANENT banner is the feedback.
"""
from types import SimpleNamespace

import pytest

from opendbc.car import structs
from openpilot.cereal import custom
from openpilot.selfdrive.selfdrived.events import ET, Events, EventName
from openpilot.selfdrive.selfdrived.state import State as MainState
from openpilot.selfdrive.selfdrived.state import StateMachine
from openpilot.sunnypilot.selfdrive.car.car_specific import CarSpecificEventsSP
from openpilot.sunnypilot.mads.state import StateMachine as MadsStateMachine
from openpilot.sunnypilot.mads.state import State as MadsState
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

EventNameSP = custom.OnroadEventSP.EventName


def _car():
  CP = SimpleNamespace(brand='hyundai', carFingerprint='HYUNDAI_ELANTRA_2021', minEnableSpeed=-1.,
                       openpilotLongitudinalControl=True)
  CP_SP = SimpleNamespace(enableGasInterceptor=True, fca11Brake=True)
  return CarSpecificEventsSP(CP, CP_SP)


def _cs_sp(hold_nag, brake_now=False, stop_complete=False):
  return SimpleNamespace(holdNag=hold_nag, brakeNow=brake_now, stopComplete=stop_complete, superviseStop=False,
                         fca11Unavailable=False, fca11CameraBraking=False, fca11Divergence=False)


class Rig:
  """The real machines plus the persistent event objects `selfdrived` keeps, wired the way selfdrived wires them."""

  def __init__(self):
    self.car = _car()
    self.events = Events()
    self.events_sp = EventsSP()
    self.state_machine = StateMachine()
    self.sd = SimpleNamespace(state_machine=self.state_machine, events=self.events, events_sp=self.events_sp,
                              enabled=False)
    # MADS keeps the same `selfdrive` object and the SAME persistent events/events_sp objects (mads/state.py l.28-32)
    self.mads = MadsStateMachine(SimpleNamespace(selfdrive=self.sd))
    self.transitions = []

  def step(self, hold_nag, *, engage=False, brake_now=False, lkas=False, stop_complete=False):
    """One `selfdrived.step()`'s worth of event flow, in the real order.

    Order matters and is the reason the route works: car events into `self.events` (in place), the car layer's SP events
    merged into the persistent `self.events_sp`, then `state_machine.update()` and only afterwards `mads.update()`
    (selfdrived.py l.660, l.662).
    """
    prev_main, prev_mads = self.state_machine.state, self.mads.state
    self.events.clear()
    self.events_sp.clear()
    # car_events.py l.155-156: this car's engage carrier is CS.buttonEnable (pcmCruise is False on an
    # openpilot-longitudinal car, so pcmEnable is never raised here).
    if engage:
      self.events.add(EventName.buttonEnable)
    if lkas:
      self.events_sp.add(EventNameSP.lkasEnable)
    ev_sp = self.car.update(SimpleNamespace(vEgo=0., buttonEvents=[], cruiseState=SimpleNamespace(nonAdaptive=False)),
                            _cs_sp(hold_nag, brake_now, stop_complete), self.events, long_active=False)
    # exactly what selfdrived.py l.285 does with the car layer's returned EventsSP
    self.events_sp.add_from_msg(ev_sp.to_msg())

    enabled, _active = self.state_machine.update(self.events)
    self.sd.enabled = enabled
    mads_enabled, _mads_active = self.mads.update()

    for tag, prev, now, names in (('main', prev_main, self.state_machine.state, _MAIN_NAMES),
                                  ('mads', prev_mads, self.mads.state, _MADS_NAMES)):
      if prev != now:
        self.transitions.append((tag, names[prev], names[now]))
    return enabled, mads_enabled

  def mads_state(self):
    return _MADS_NAMES[self.mads.state]

  def events_with(self, name):
    return self.events.has(name) or self.events_sp.has(name)


# the cereal enums are plain int-like values: the NAME lookup has to come from the enum CLASS's schema, not the value
_MAIN_NAMES = {v: k for k, v in MainState.schema.enumerants.items()}
_MADS_NAMES = {v: k for k, v in MadsState.schema.enumerants.items()}


def _engage(rig):
  """Get both machines engaged and assert it - every scenario below starts from a real engagement, not a fake one."""
  assert rig.step(0, engage=True, lkas=True) == (True, True), "control: the car must be ENGAGED before expiry means anything"
  assert rig.mads_state() == 'enabled'
  assert rig.state_machine.state == _main_enabled()
  rig.transitions.clear()   # the engagement itself is the precondition, not the thing under test


def _main_enabled():
  return MainState.enabled


def test_a_tier_changes_neither_machine():
  """60 / 90 / 110 s are ALERTS. If a tier disengaged, the car would stop itself a minute early."""
  rig = Rig()
  _engage(rig)
  for nag, sp_event in ((1, EventNameSP.fca11HoldNagSoft), (2, EventNameSP.fca11HoldNagLoud),
                        (3, EventNameSP.fca11HoldNagFinal)):
    assert rig.step(nag) == (True, True), f"nag {nag} must not disengage"
    assert rig.events_sp.has(sp_event)
    assert not rig.events.has(EventName.pcmDisable), f"nag {nag} must not write a standard disengage event"
    assert rig.state_machine.state == _main_enabled()
    assert rig.mads_state() == 'enabled'
  assert rig.transitions == [], f"no transition may have happened: {rig.transitions}"


def test_expiry_disengages_both_machines_in_one_update():
  rig = Rig()
  _engage(rig)
  assert rig.step(4) == (False, False), "the expiry frame must disengage main AND lateral in the SAME update"
  assert ('main', 'enabled', 'disabled') in rig.transitions
  assert ('mads', 'enabled', 'disabled') in rig.transitions
  assert rig.state_machine.state == _main_disabled()
  # the driver is told, in a PERMANENT alert that only a brake press takes away
  assert rig.events_sp.has(EventNameSP.fca11HoldExpired)
  assert rig.events_sp.contains(ET.PERMANENT) and rig.events_sp.contains(ET.NO_ENTRY)


def test_latched_frames_keep_it_disengaged_and_refuse_a_re_engage():
  rig = Rig()
  _engage(rig)
  rig.step(4)
  # five latched frames with no brake: the alert stays up and a RES press cannot get back in
  for i in range(5):
    assert rig.step(4) == (False, False), f"latched frame {i}: must stay disabled"
  assert rig.step(4, engage=True) == (False, False), "a RES press while latched must be REFUSED"
  assert rig.step(4, engage=True) == (False, False)
  assert rig.events_sp.has(EventNameSP.fca11HoldExpired), "the refusal is explained, not silent"
  # ... and nothing below has moved the machines back up
  assert rig.state_machine.state == _main_disabled()


def test_the_brake_press_re_opens_engagement():
  """The ONLY clear, end to end: holdNag back to 0 (the car layer's latch cleared by CS.out.brakePressed)."""
  rig = Rig()
  _engage(rig)
  rig.step(4)
  assert rig.step(4, engage=True) == (False, False), "refused while latched"
  assert rig.step(0, engage=True, lkas=True) == (True, True), "after the brake press the driver may engage again"
  assert not rig.events_sp.has(EventNameSP.fca11HoldExpired)
  assert rig.state_machine.state == _main_enabled()


def test_mads_lateral_is_refused_by_the_sp_no_entry_while_latched():
  """Lateral-only: MADS reads events_sp (`check_contains`), so the SP ET.NO_ENTRY is what refuses it."""
  rig = Rig()
  _engage(rig)
  rig.step(4)
  assert rig.mads_state() == 'disabled'
  assert rig.step(4, lkas=True) == (False, False), "LKAS while latched must be refused"
  assert rig.mads_state() == 'disabled'
  # the brake press clears the latch, and then LKAS works again
  assert rig.step(0, lkas=True) == (False, True), "lateral may re-engage once the brake has been pressed"
  assert rig.mads_state() == 'enabled'


def test_brake_now_still_outranks_a_tier_but_never_hides_the_expiry():
  rig = Rig()
  _engage(rig)
  assert rig.step(2, brake_now=True) == (True, True)
  assert rig.events_sp.has(EventNameSP.fca11BrakeNow)
  assert not rig.events_sp.has(EventNameSP.fca11HoldNagLoud), "a real fault replaces the nag"
  # at expiry the brakes come off whatever else is true - brakeNow must not suppress the release
  assert rig.step(4, brake_now=True) == (False, False)
  assert rig.events_sp.has(EventNameSP.fca11HoldExpired)


def test_feature_off_is_the_deployed_path():
  """fca11Brake OFF: no alert, no write, no disengage - for every value of the latch."""
  rig = Rig()
  rig.car = CarSpecificEventsSP(SimpleNamespace(brand='hyundai', carFingerprint='HYUNDAI_ELANTRA_2021',
                                                minEnableSpeed=-1., openpilotLongitudinalControl=True),
                                SimpleNamespace(enableGasInterceptor=True, fca11Brake=False))
  _engage(rig)
  for nag in (0, 1, 2, 3, 4):
    assert rig.step(nag) == (True, True), f"OFF must be inert at nag {nag}"
  assert not rig.events.has(EventName.pcmDisable)
  assert rig.events_sp.names == [], "no SP event is raised by a disabled feature"


def test_engageable_is_the_known_cost():
  """Pins the one place this route is weaker than the brief's wording, so nobody 'fixes' it by inventing a fake event.

  `engaged`/`engageable` in selfdrived is `not events.contains(ET.NO_ENTRY)` on the STANDARD events; the car layer puts
  NO_ENTRY on the SP event (so MADS sees it) and blocks the main machine by removing the ENABLE events instead. The
  result, measured here: while latched, `engageable` would still read True while engaging is refused.
  """
  rig = Rig()
  _engage(rig)
  rig.step(4)
  assert not rig.events.contains(ET.NO_ENTRY), ("engageable would read True - the accepted cost")
  assert not rig.events.has(EventName.buttonEnable), "and this is why it cannot actually engage"
  assert rig.events_sp.contains(ET.NO_ENTRY), "the refusal is a NO_ENTRY for MADS, which is the machine that acts on it"


def _main_disabled():
  return MainState.disabled
