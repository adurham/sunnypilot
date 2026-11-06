"""
fork (0043): the state-machine proof for the hold-countdown EXPIRY - the part of the patch that ACTS.

The countdown itself is a pure observer in the car layer (opendbc `fca11_long._hold_nag_step`). At 2:00 (6000 armed SENT
frames) it sets a sticky latch and publishes `CarStateSP.holdNag == 4`; everything that follows is decided in the
superproject. This file drives the REAL `CarSpecificEventsSP`, the REAL `selfdrive/selfdrived/state.py` StateMachine and
the REAL `sunnypilot/mads/mads.py::ModularAssistiveDrivingSystem` through `mads.update(CS)` - the very call selfdrived
makes at l.662 - with ONLY `Params` doubled (the four MADS toggles). `mads/state.py` is exercised through that call, not
called directly: calling the MADS state machine on its own skips `mads.update_events`, which is where mads.py l.205
removes `EventName.pcmDisable` before the machine looks at anything.

The contract asserted here is the one the car actually implements (owner decision, Opus review D1):

  * a nag TIER (1/2/3) changes neither machine;
  * **at 2:00 longitudinal control disengages and the brakes release; lateral (MADS) stays engaged by design.**
    Concretely: `CarSpecificEventsSP` writes `EventName.pcmDisable` (USER_DISABLE) on the standard `events` object it is
    handed, and `state_machine.update` (selfdrived.py l.660) consumes it in the same `step()`, so main goes
    enabled -> disabled and controlsd's `longActive` (controlsd.py l.120: needs `CC.enabled`) drops to False. One line
    LATER selfdrived calls `self.mads.update(CS)` (l.662), whose `update_events` REMOVES `pcmDisable` (mads.py l.205)
    before its state machine runs, so lateral never sees the disengage and stays enabled and active. Measured for the
    default settings (Mads=1, MadsMainCruiseAllowed=1, UnifiedEngagement=1, SteeringMode=0) and for every mode Opus
    probed (UEM off; steering-mode-on-brake REMAIN/PAUSE/DISENGAGE; Mads off);
  * while the latch is set, an engage request is refused - the RES press on this car is carried as `buttonEnable`
    (car_events.py l.155-156, because `pcmCruise` is False; `pcmEnable` is the pcm-cruise carrier and is stripped by the
    same two lines) - and the refusal is not silent: the PERMANENT + NO_ENTRY SP alert is up;
  * lateral, if the driver turns it off with the LKAS button while latched (mads.py l.174-181), cannot be turned back on
    until the brake press clears the latch: `mads/state.py::check_contains` reads BOTH `events` and `events_sp`, so the
    SP `ET.NO_ENTRY` is what refuses it;
  * the FIRST brake press clears the latch (holdNag back to 0 - the car layer's `CS.out.brakePressed`) and engagement is
    allowed again: the driver is never locked out;
  * `pcmDisable` is re-written on EVERY latched frame, including a frame on which carStateSP did NOT update (SubMaster
    keeps only the latest message): that repetition is what keeps the latch effective when the CS_SP edge is skipped.

The one place this is weaker than a standard NO_ENTRY: `SelfdriveState.engageable` stays True while latched (it is
`not events.contains(ET.NO_ENTRY)` on the STANDARD events) even though an engage does nothing. That is the accepted cost
of not touching upstream `cereal/log.capnp` (see lane-X3/STATE.md, checkpoint 1) - the red PERMANENT banner is the
feedback.
"""
from types import SimpleNamespace

import pytest

from opendbc.car import structs
from openpilot.cereal import custom
from openpilot.selfdrive.selfdrived.events import ET, Events, EventName
from openpilot.selfdrive.selfdrived.state import State as MainState
from openpilot.selfdrive.selfdrived.state import StateMachine
from openpilot.sunnypilot.selfdrive.car.car_specific import CarSpecificEventsSP
from openpilot.sunnypilot.selfdrive.selfdrived.events import EVENTS_SP, EventsSP
from openpilot.sunnypilot.mads import mads as mads_module
from openpilot.sunnypilot.mads.mads import ModularAssistiveDrivingSystem

EventNameSP = custom.OnroadEventSP.EventName
MadsState = custom.ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState


# params_keys.h l.206-209: these are the DEFAULTS the car ships with, so the first configuration below is the deployed one
DEFAULTS = {"Mads": True, "MadsMainCruiseAllowed": True, "MadsUnifiedEngagementMode": True, "MadsSteeringMode": 0,
            "DisengageOnAccelerator": False}


class FakeParams:
  """Only the toggles: the four MADS keys plus DisengageOnAccelerator. Defaults are params_keys.h's."""

  def __init__(self, overrides=None):
    self.d = dict(DEFAULTS, **(overrides or {}))

  def get_bool(self, key, *a, **kw):
    return bool(self.d.get(key, False))

  def get(self, key, block=False, return_default=False, *a, **kw):
    return self.d.get(key, 0)


class Rig:
  """The real superproject objects, wired the way `selfdrived.step()` wires them.

  Order (selfdrived.py l.281-305 -> l.660 -> l.662): car-layer standard events -> CarSpecificEventsSP (which is handed
  `self.events` itself) -> pedalPressed on a brake rise -> `state_machine.update(self.events)` -> `self.mads.update(CS)`.
  """

  def __init__(self, overrides=None):
    self.params = FakeParams(overrides)
    CP = structs.CarParams()
    CP.brand = 'hyundai'
    CP.carFingerprint = 'HYUNDAI_ELANTRA_2021'
    CP.minEnableSpeed = -1.0
    CP.openpilotLongitudinalControl = True
    CP.passive = False
    CP_SP = structs.CarParamsSP()
    CP_SP.fca11Brake = True
    CP_SP.enableGasInterceptor = True
    self.car = CarSpecificEventsSP(CP, CP_SP)

    self.events = Events()
    self.events_sp = EventsSP()
    self.state_machine = StateMachine()
    CS_prev = self._cs()
    self.sd = SimpleNamespace(CP=CP, CP_SP=CP_SP, params=self.params, events=self.events, events_sp=self.events_sp,
                              state_machine=self.state_machine, enabled=False, initialized=True,
                              sm={'pandaStates': []}, CS_prev=CS_prev)
    self.mads = ModularAssistiveDrivingSystem(self.sd)
    self.transitions = []
    self._last_cs_sp = None
    self._cs_prev = CS_prev

  # -- inputs -----------------------------------------------------------------------------------------------------
  @staticmethod
  def _cs(*, engage=False, brake=False, lkas=False, gas=False, standstill=True):
    c = structs.CarState()
    c.vEgo = 0.0
    c.standstill = standstill
    c.brakePressed = brake
    c.gasPressed = gas
    c.gearShifter = structs.CarState.GearShifter.drive
    c.cruiseState.available = True
    if lkas:
      # the real carrier: mads.update_events reads CS.buttonEvents itself (mads.py l.170-181)
      c.buttonEvents = [structs.CarState.ButtonEvent(type=structs.CarState.ButtonEvent.Type.lkas, pressed=True)]
    return c

  @staticmethod
  def _cs_sp(hold_nag, brake_now=False, stop_complete=False):
    m = structs.CarStateSP()
    m.holdNag = hold_nag
    m.brakeNow = brake_now
    m.stopComplete = stop_complete
    return m

  # -- one selfdrived.step() ---------------------------------------------------------------------------------------
  def step(self, hold_nag, *, engage=False, brake_now=False, lkas=False, stop_complete=False, skip_cs_sp=False):
    prev_main, prev_mads = self.state_machine.state, self.mads.state_machine.state
    C = self._cs(engage=engage, brake=self._brake, lkas=lkas)

    self.events.clear()
    self.events_sp.clear()
    # car_events.py l.281: a RES press on this car is raised here as EventName.buttonEnable (l.155-156; pcmCruise is
    # False on an openpilot-longitudinal car, so EventName.pcmEnable is never the carrier).
    if engage:
      self.events.add(EventName.buttonEnable)

    if skip_cs_sp:
      # SubMaster keeps only the LATEST message: a frame on which carStateSP did not update still hands the car layer
      # the previous (latched) sample.
      assert self._last_cs_sp is not None
      cs_sp = self._last_cs_sp
    else:
      cs_sp = self._last_cs_sp = self._cs_sp(hold_nag, brake_now, stop_complete)

    ev_sp = self.car.update(C, cs_sp, self.events, long_active=self.sd.enabled)

    # exactly what selfdrived.py l.285 does with the car layer's returned EventsSP
    self.events_sp.add_from_msg(ev_sp.to_msg())

    # the car layer's write, before anything consumes it: this is what `state_machine.update` below sees
    pcm_disable = self.events.has(EventName.pcmDisable)

    # selfdrived.py l.302-305: a brake RISE raises pedalPressed on the standard events (MADS strips it again at l.207
    # of mads.py, so it is not what disengages lateral in REMAIN mode - the steering-mode-on-brake block is).
    if C.brakePressed and not self._cs_prev.brakePressed:
      self.events.add(EventName.pedalPressed)

    enabled, _active = self.state_machine.update(self.events)
    self.sd.enabled = enabled

    # the REAL mads.update(CS): data_sample -> update_events (strips pcmDisable at mads.py l.205) -> state machine.
    # It returns None; the result is on the instance (`self.enabled`/`self.active`), exactly as selfdrived reads it.
    self.mads.update(C)
    mads_active = self.mads.active

    self._cs_prev = C
    for tag, prev, now, names in (('main', prev_main, self.state_machine.state, _MAIN_NAMES),
                                  ('mads', prev_mads, self.mads.state_machine.state, _MADS_NAMES)):
      if prev != now:
        self.transitions.append((tag, names[prev], names[now]))
    return dict(main=self.main_state(), mads=self.mads_state(), mads_active=mads_active,
                enabled=enabled, pcm_disable=pcm_disable,
                # `mads.update_events` removes pcmDisable (mads.py l.205) before the MADS state machine runs: that is
                # exactly why lateral survives the expiry, so it is recorded rather than asserted away
                pcm_disable_stripped=not self.events.has(EventName.pcmDisable),
                expired=self.events_sp.has(EventNameSP.fca11HoldExpired))

  # -- state accessors ----------------------------------------------------------------------------------------------
  _brake = False

  def brake(self, on=True):
    self._brake = on

  def main_state(self):
    return _MAIN_NAMES[self.state_machine.state]

  def mads_state(self):
    return _MADS_NAMES[self.mads.state_machine.state]

  @property
  def enabled(self):
    return self.sd.enabled


_MAIN_NAMES = {v: k for k, v in MainState.schema.enumerants.items()}
_MADS_NAMES = {v: k for k, v in MadsState.schema.enumerants.items()}


@pytest.fixture
def rig(monkeypatch):
  """Patch ONLY `mads.py`'s module-level `Params` (used once, for DisengageOnAccelerator), then hand out a Rig."""
  def make(overrides=None):
    p = FakeParams(overrides)
    monkeypatch.setattr(mads_module, "Params", lambda: p)
    return Rig(overrides)
  return make


def _engage_both(rig):
  """A real engagement of both machines, through the real event flow; asserts it happened."""
  rig.step(0, engage=True)
  if rig.mads_state() != 'enabled' and rig.params.get_bool("Mads"):
    rig.step(0, lkas=True)   # UEM off needs its own lateral press
  assert rig.main_state() == 'enabled', "control: main must be ENGAGED before expiry means anything"
  if rig.params.get_bool("Mads"):
    assert rig.mads_state() == 'enabled', "control: lateral must be ENGAGED before expiry means anything"
  rig.transitions.clear()


CONFIGS = [
  # name, overrides, is UEM on (lateral engages with RES)
  ("defaults_mads1_uem1_mode0", {}, True),
  ("uem_off", {"MadsUnifiedEngagementMode": False}, False),
  ("mode_remain", {"MadsSteeringMode": 0}, True),
  ("mode_pause", {"MadsSteeringMode": 1}, True),
  ("mode_disengage", {"MadsSteeringMode": 2}, True),
]


# ---------------------------------------------------------------------------------------------------------------------
# D1 - the contract, on the real mads.update(CS) path, for every configuration Opus probed
# ---------------------------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("name,overrides,uem", CONFIGS, ids=[c[0] for c in CONFIGS])
def test_tiers_change_neither_machine(rig, name, overrides, uem):
  """60 / 90 / 110 s are ALERTS. If a tier disengaged, the car would stop itself a minute early."""
  r = rig(overrides)
  _engage_both(r)
  for nag, sp_event in ((1, EventNameSP.fca11HoldNagSoft), (2, EventNameSP.fca11HoldNagLoud),
                        (3, EventNameSP.fca11HoldNagFinal)):
    out = r.step(nag)
    assert (out["main"], out["mads"]) == ('enabled', 'enabled'), f"[{name}] nag {nag} must not disengage: {out}"
    assert r.events_sp.has(sp_event)
    assert not out["pcm_disable"], f"[{name}] nag {nag} must not write a standard disengage event"
  assert r.transitions == [], f"[{name}] no transition may have happened: {r.transitions}"


@pytest.mark.parametrize("name,overrides,uem", CONFIGS, ids=[c[0] for c in CONFIGS])
def test_expiry_drops_longitudinal_and_keeps_lateral(rig, name, overrides, uem):
  """THE contract: at 2:00 main goes enabled -> disabled (longitudinal drops, brakes release) and MADS lateral STAYS
  enabled and active. `pcmDisable` is written on the frame but never reaches MADS: mads.py l.205 removes it inside
  mads.update_events, before the MADS state machine runs."""
  r = rig(overrides)
  _engage_both(r)
  out = r.step(4)
  assert out["main"] == 'disabled', f"[{name}] longitudinal must drop at 2:00: {out}"
  assert ('main', 'enabled', 'disabled') in r.transitions, r.transitions
  # this is what controlsd reads (controlsd.py l.120: longActive needs CC.enabled) -> the law stops sending -> release
  assert out["enabled"] is False
  assert out["pcm_disable"] is True, "the car layer must write the standard disengage on the expiry frame"
  if r.params.get_bool("Mads"):
    assert out["mads"] == 'enabled', f"[{name}] lateral must STAY engaged by design: {out}"
    assert out["mads_active"] is True, f"[{name}] lateral must still be ACTIVE: {out}"
    assert ('mads', 'enabled', 'disabled') not in r.transitions, \
      f"[{name}] lateral must not be dropped by the expiry: {r.transitions}"
    # the mechanism: mads.update_events removes pcmDisable (mads.py l.205) before the MADS state machine looks at
    # anything. Pinned so the reason lateral survives cannot be lost with the behaviour.
    assert out["pcm_disable_stripped"] is True, "mads.update_events must strip pcmDisable before its state machine"
  else:
    assert out["mads"] == 'disabled' and out["mads_active"] is False, f"[{name}] no MADS, no lateral: {out}"
  # the driver is told, in a PERMANENT alert that only a brake press takes away
  assert out["expired"]
  assert r.events_sp.contains(ET.PERMANENT) and r.events_sp.contains(ET.NO_ENTRY)


@pytest.mark.parametrize("carrier", [EventName.buttonEnable, EventName.pcmEnable])
def test_latched_frames_refuse_engage_requests(rig, carrier):
  """RES on this car is `buttonEnable` (car_events l.155-156); `pcmEnable` is the pcm-cruise carrier. The car layer
  removes BOTH on every latched frame, so the disabled branch of the main machine never sees ET.ENABLE."""
  r = rig()
  _engage_both(r)
  r.step(4)
  for i in range(5):
    out = r.step(4)
    assert out["main"] == 'disabled', f"latched frame {i}: must stay disabled: {out}"
    assert out["expired"], f"latched frame {i}: the alert must stay up"
    assert not r.events.has(carrier), f"latched frame {i}: {carrier} must be refused"
  # and explicitly: the request IS made and IS refused on the same frame
  r.events.add(carrier)
  out = r.step(4)
  assert out["main"] == 'disabled', f"a {carrier} press while latched must be REFUSED: {out}"
  assert not r.events.has(carrier), "the car layer must strip the ENABLE event"


def test_brake_press_clears_the_latch_and_reengage_is_allowed(rig):
  """The ONLY clear, end to end: holdNag back to 0 (the car layer's latch cleared by CS.out.brakePressed)."""
  r = rig()
  _engage_both(r)
  r.step(4)
  assert r.step(4, engage=True)["main"] == 'disabled', "refused while latched"
  r.brake(True)                       # the press: the car layer clears the latch as holdNag drops
  out = r.step(0)
  assert out["main"] == 'disabled'
  assert not out["expired"], "the brake press must clear the PERMANENT alert"
  r.brake(False)
  out = r.step(0, engage=True)
  assert out["main"] == 'enabled', f"after the brake press the driver may engage again: {out}"
  assert r.main_state() == 'enabled'


@pytest.mark.parametrize("overrides,expected_after_brake", [
  ({}, 'enabled'),                                  # REMAIN: lateral is not touched by a brake press (pedalPressed
                                                    # is stripped at mads.py l.207 before the machine sees it)
  ({"MadsSteeringMode": 1}, 'paused'),              # PAUSE: silentLkasDisable -> paused
  ({"MadsSteeringMode": 2}, 'disabled'),            # DISENGAGE: lkasDisable -> disabled
], ids=["remain", "pause", "disengage"])
def test_steering_mode_on_brake_decides_lateral_after_the_press(rig, overrides, expected_after_brake):
  """The latch clears for LONGITUDINAL in every mode; what lateral does on the brake press is the driver's setting."""
  r = rig(overrides)
  _engage_both(r)
  r.step(4)
  assert r.mads_state() == 'enabled', "still latched: lateral is untouched"
  r.brake(True)
  r.step(0)
  assert r.mads_state() == expected_after_brake, f"mode {overrides}: {r.mads_state()}"
  # re-engaging LONGITUDINAL is allowed in every mode
  r.brake(False)
  assert r.step(0, engage=True)["main"] == 'enabled'


def test_lateral_reengage_while_latched_is_refused_by_the_sp_no_entry(rig):
  """Lateral-only: the driver turns lateral off with LKAS (mads.py l.174-181), then cannot turn it back on while the
  latch is set - `mads/state.py::check_contains` reads events_sp, and fca11HoldExpired carries ET.NO_ENTRY there."""
  r = rig()
  _engage_both(r)
  r.step(4)
  assert r.mads_state() == 'enabled'
  out = r.step(4, lkas=True)          # LKAS while MADS is engaged -> lkasDisable (a normal user disengage)
  assert out["mads"] == 'disabled', out
  for i in range(2):
    out = r.step(4, lkas=True)        # LKAS again -> lkasEnable, refused by the SP NO_ENTRY
    assert out["mads"] == 'disabled', f"lateral re-entry must be refused while latched (press {i}): {out}"
    assert r.events_sp.contains(ET.NO_ENTRY)
  r.brake(True)                       # the brake clears the latch, and then LKAS works again
  r.step(0)
  r.brake(False)
  out = r.step(0, lkas=True)
  assert out["mads"] == 'enabled', f"lateral may re-engage once the brake has been pressed: {out}"


def test_brake_now_still_outranks_a_tier_but_never_hides_the_expiry(rig):
  r = rig()
  _engage_both(r)
  out = r.step(2, brake_now=True)
  assert (out["main"], out["mads"]) == ('enabled', 'enabled')
  assert r.events_sp.has(EventNameSP.fca11BrakeNow)
  assert not r.events_sp.has(EventNameSP.fca11HoldNagLoud), "a real fault replaces the nag"
  # at expiry the brakes come off whatever else is true - brakeNow must not suppress the release
  out = r.step(4, brake_now=True)
  assert out["main"] == 'disabled' and out["mads"] == 'enabled'
  assert r.events_sp.has(EventNameSP.fca11HoldExpired)


def test_feature_off_is_the_deployed_path(rig):
  """fca11Brake OFF: no alert, no write, no disengage - for every value of the latch."""
  r = rig()
  r.car = CarSpecificEventsSP(r.sd.CP, SimpleNamespace(fca11Brake=False, enableGasInterceptor=True))
  _engage_both(r)
  for nag in (0, 1, 2, 3, 4):
    out = r.step(nag)
    assert (out["main"], out["mads"]) == ('enabled', 'enabled'), f"OFF must be inert at nag {nag}: {out}"
    assert not out["pcm_disable"]
  assert r.events_sp.names == [], "no SP event is raised by a disabled feature"


# ---------------------------------------------------------------------------------------------------------------------
# D2 - the write is REPEATED on every latched frame, not just on the edge
# ---------------------------------------------------------------------------------------------------------------------
def test_pcm_disable_is_written_on_every_latched_frame(rig):
  """`pcmDisable` is re-added on EVERY latched frame (car_specific.py l.101-105). That repetition is what keeps the
  latch effective when the carStateSP edge is SKIPPED - SubMaster keeps only the latest message, so selfdrived can hand
  the car layer the same `holdNag == 4` sample twice (or more) while the main machine is already disabled."""
  r = rig()
  _engage_both(r)
  out = r.step(4)
  assert out["pcm_disable"] is True, "the expiry frame must write it"
  # five more latched frames, THREE of them with NO carStateSP update at all (a stale, repeated message)
  for i, skip in enumerate((False, True, True, False, True)):
    out = r.step(4, skip_cs_sp=skip)
    assert out["pcm_disable"] is True, \
      f"latched frame {i} (skipped carStateSP={skip}) MUST still write pcmDisable: {out}"
    assert out["main"] == 'disabled'
  # and the frame that carries a fresh ENABLE request while the CS_SP is skipped is still refused
  out = r.step(4, skip_cs_sp=True, engage=True)
  assert out["main"] == 'disabled' and out["pcm_disable"] is True, out


# ---------------------------------------------------------------------------------------------------------------------
# Guards around the wording and the accepted cost
# ---------------------------------------------------------------------------------------------------------------------
def test_expired_alert_text_does_not_imply_steering_is_off():
  """The banner says the BRAKES are released. It must not claim, or hint, that steering/lateral is off: under MADS it
  is not (this file's contract test above). Checked on the shipped strings, not on a claim in a comment."""
  entry = EVENTS_SP[EventNameSP.fca11HoldExpired]
  texts = []
  for alert in entry.values():
    for attr in ("alert_text_1", "alert_text_2"):
      t = getattr(alert, attr, "")
      if t:
        texts.append(t.lower())
  assert texts, "the expired event must have alert text"
  assert any("brake" in t for t in texts), texts
  for t in texts:
    assert "steer" not in t and "lateral" not in t and "mads" not in t, f"alert text implies steering is off: {t!r}"


def test_engageable_is_the_known_cost(rig):
  """Pins the one place this route is weaker than the brief's wording, so nobody 'fixes' it by inventing a fake event.

  `engaged`/`engageable` in selfdrived is `not events.contains(ET.NO_ENTRY)` on the STANDARD events; the car layer puts
  NO_ENTRY on the SP event (so MADS sees it) and blocks the main machine by removing the ENABLE events instead. The
  result, measured here: while latched, `engageable` would still read True while engaging is refused.
  """
  r = rig()
  _engage_both(r)
  r.step(4)
  assert not r.events.contains(ET.NO_ENTRY), "engageable would read True - the accepted cost"
  assert not r.events.has(EventName.buttonEnable), "and this is why it cannot actually engage"
  assert r.events_sp.contains(ET.NO_ENTRY), "the refusal is a NO_ENTRY for MADS, which is the machine that acts on it"
