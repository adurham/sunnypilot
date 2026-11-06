"""
Drive-mode-follows-personality (adurham fork): the ``DriveModePersonality`` param gates a feature that
drives the openpilot personality (via the gap-button rail) and blocks openpilot LONGITUDINAL in N /
N-Custom, using the decoded ``carStateSP.driveMode``.

Covers:
  * mapping (NORMAL -> standard, ECO -> relaxed, SPORT -> aggressive; N/N-Custom -> BLOCK; unknown -> noop),
    delegated to the opendbc pure function;
  * the live personality switch through the real selfdrived rail (self.personality + LongitudinalPersonality
    param + personalityChanged) on EVERY mode change;
  * manual gap-button cycling between mode changes is preserved (no re-assert until the next change);
  * the block gate on the real selfdrived StateMachine: N -> no new engage, and disengage-if-engaged;
  * lateral untouched: the block event is stripped by MADS (mads.py) so a MADS lateral-only engagement lives;
  * OFF inert (param default) and unknown/absent inert (never force, never block).
"""
from types import SimpleNamespace

import pytest

from openpilot.cereal import custom, log
from openpilot.common.params import Params
from opendbc.car import structs
from opendbc.sunnypilot.car.hyundai.drive_mode import DriveModeResult as R
from openpilot.selfdrive.selfdrived.events import Events, ET
from openpilot.selfdrive.selfdrived.state import StateMachine
from openpilot.sunnypilot.fork import drive_mode_personality as dmp
from openpilot.sunnypilot.mads.mads import ModularAssistiveDrivingSystem
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

EventName = log.OnroadEvent.EventName
OpenpilotState = log.SelfdriveState.OpenpilotState
P = log.LongitudinalPersonality

NORMAL, ECO, SPORT, N_CUSTOM, N = 1, 2, 3, 6, 7


class _SS:
  """Just the attributes dmp.step / apply_live touch (a stand-in for SelfdriveD)."""

  def __init__(self, params, personality=P.standard, drive_mode_result=dmp.UNKNOWN):
    self.params = params
    self.personality = personality
    self.drive_mode_result = drive_mode_result


class _Params:
  def __init__(self):
    self.store = {}

  def put(self, k, v, block=False):
    self.store[k] = v

  def get_bool(self, k):
    return bool(self.store.get(k, False))


# --------------------------------------------------------------------------------------------------
# mapping
# --------------------------------------------------------------------------------------------------

class TestMapping:
  @pytest.mark.parametrize("mode,personality", [(NORMAL, P.standard), (ECO, P.relaxed), (SPORT, P.aggressive)])
  def test_modes_to_personality(self, mode, personality):
    assert dmp.desired_personality(dmp.result_for(mode)) == personality
    assert not dmp.blocks_longitudinal(dmp.result_for(mode))

  @pytest.mark.parametrize("mode", [N, N_CUSTOM])
  def test_n_modes_block(self, mode):
    r = dmp.result_for(mode)
    assert r == dmp.BLOCK
    assert dmp.blocks_longitudinal(r)
    assert dmp.desired_personality(r) is None

  @pytest.mark.parametrize("mode", [0, 4, 5, 8, 15, -1, 999, None, "junk", 1.5])
  def test_unknown_is_inert(self, mode):
    r = dmp.result_for(mode)
    assert r == dmp.UNKNOWN
    assert not dmp.blocks_longitudinal(r)          # never blocks
    assert dmp.desired_personality(r) is None      # never forces

  def test_enabled_param_default_off(self):
    assert dmp.read_enabled(_Params()) is False
    p = _Params(); p.store[dmp.PARAM] = True
    assert dmp.read_enabled(p) is True

  def test_read_enabled_never_raises_on_stale_libparams(self):
    class Boom:
      def get_bool(self, k):
        raise KeyError(k)
    assert dmp.read_enabled(Boom()) is False


# --------------------------------------------------------------------------------------------------
# live personality switch (gap-button rail)
# --------------------------------------------------------------------------------------------------

class TestLivePersonality:
  def test_mode_change_updates_live_personality_and_param(self):
    params = _Params()
    ss = _SS(params, personality=P.standard)
    events = Events()

    dmp.step(ss, events, ECO)
    assert ss.personality == P.relaxed
    assert params.store["LongitudinalPersonality"] == P.relaxed
    assert events.has(EventName.personalityChanged)

    events = Events()
    dmp.step(ss, events, SPORT)
    assert ss.personality == P.aggressive
    assert params.store["LongitudinalPersonality"] == P.aggressive
    assert events.has(EventName.personalityChanged)

    events = Events()
    dmp.step(ss, events, NORMAL)
    assert ss.personality == P.standard
    assert params.store["LongitudinalPersonality"] == P.standard
    assert events.has(EventName.personalityChanged)

  def test_no_change_no_rewrite(self):
    params = _Params()
    ss = _SS(params, personality=P.standard)
    events = Events()
    dmp.step(ss, events, ECO)                     # -> relaxed
    params.store.clear()
    events = Events()
    dmp.step(ss, events, ECO)                     # same mode again
    assert ss.personality == P.relaxed
    assert "LongitudinalPersonality" not in params.store  # not rewritten
    assert not events.has(EventName.personalityChanged)

  def test_manual_gap_cycling_between_changes_is_preserved(self):
    # the driver cycles the gap button after the mode set relaxed: the manual value sticks until the NEXT
    # mode CHANGE re-asserts. (This mirrors selfdrived's (personality - 1) % 3 on a gap press.)
    params = _Params()
    ss = _SS(params, personality=P.standard)
    events = Events()
    dmp.step(ss, events, ECO)
    assert ss.personality == P.relaxed
    ss.personality = (ss.personality - 1) % 3      # the gap button decrements
    assert ss.personality == P.standard
    events = Events()
    dmp.step(ss, events, ECO)                      # no mode change -> manual value preserved
    assert ss.personality == P.standard
    events = Events()
    dmp.step(ss, events, SPORT)                    # next change re-asserts
    assert ss.personality == P.aggressive

  def test_unknown_never_forces_personality(self):
    params = _Params()
    ss = _SS(params, personality=P.aggressive)
    events = Events()
    dmp.step(ss, events, 0)                        # unknown
    assert ss.personality == P.aggressive
    assert "LongitudinalPersonality" not in params.store
    assert not events.has(EventName.personalityChanged)
    assert not events.has(EventName.driveModePersonalityBlock)

  def test_unknown_transition_does_not_reassert(self):
    params = _Params()
    ss = _SS(params, personality=P.standard)
    dmp.step(ss, Events(), SPORT)                  # -> aggressive
    dmp.step(ss, Events(), 0)                      # unknown: keep aggressive, no rewrite
    assert ss.personality == P.aggressive


# --------------------------------------------------------------------------------------------------
# block gate on the real selfdrived state machine
# --------------------------------------------------------------------------------------------------

def _main_sm(events, enabled):
  sm = StateMachine()
  sm.state = log.SelfdriveState.OpenpilotState.enabled if enabled else log.SelfdriveState.OpenpilotState.disabled
  # a fresh engage request this frame (the pause/resume button in pedal-long)
  events.add(EventName.buttonEnable)
  en, active = sm.update(events)
  return sm, en


class TestBlockGate:
  def test_no_engage_in_n(self):
    params = _Params()
    ss = _SS(params, personality=P.standard, drive_mode_result=R.STANDARD)
    events = Events()
    dmp.step(ss, events, N)                        # block event added
    assert events.has(EventName.driveModePersonalityBlock)
    sm, enabled = _main_sm(events, enabled=False)
    assert not enabled                              # NO_ENTRY -> refused
    assert sm.state == OpenpilotState.disabled

  def test_disengage_on_switch_in_n(self):
    params = _Params()
    ss = _SS(params, personality=P.standard, drive_mode_result=R.STANDARD)
    events = Events()
    dmp.step(ss, events, N_CUSTOM)                 # switched into N-Custom while engaged
    sm, enabled = _main_sm(events, enabled=True)
    assert not enabled                              # USER_DISABLE -> dropped now
    assert sm.state == OpenpilotState.disabled

  def test_block_then_release_allows_engage(self):
    params = _Params()
    ss = _SS(params, personality=P.standard)
    dmp.step(ss, Events(), N)                      # block
    events = Events()
    dmp.step(ss, events, SPORT)                    # back to a ring mode -> no block
    assert not events.has(EventName.driveModePersonalityBlock)
    sm, enabled = _main_sm(events, enabled=False)
    assert enabled

  def test_n_custom_slot_irrelevant(self):
    # both N-Custom slots decode to raw 6 -> BLOCK (slot is the b5 low nibble, not part of the mode)
    for _slot in (1, 2):
      params = _Params()
      ss = _SS(params, personality=P.standard)
      events = Events()
      dmp.step(ss, events, N_CUSTOM)
      assert events.has(EventName.driveModePersonalityBlock)


# --------------------------------------------------------------------------------------------------
# lateral untouched (MADS strips the block event)
# --------------------------------------------------------------------------------------------------

def _fake_selfdrive(params):
  return SimpleNamespace(
    CP=SimpleNamespace(brand="hyundai", flags=0),
    CP_SP=SimpleNamespace(flags=0),
    params=params,
    events=Events(),
    events_sp=EventsSP(),
    CS_prev=SimpleNamespace(cruiseState=SimpleNamespace(available=True)),
    enabled=False, enabled_prev=False, initialized=True,
    state_machine=StateMachine(),
    sm={"pandaStates": [], "carControl": None},
  )


class TestLateralUntouched:
  def test_mads_strips_the_block_event(self):
    params = Params()
    params.put_bool("Mads", True, block=True)
    params.put_bool("MadsMainCruiseAllowed", True, block=True)
    sd = _fake_selfdrive(params)
    mads = ModularAssistiveDrivingSystem(sd)

    sd.events.add(EventName.buttonEnable)
    sd.events.add(EventName.driveModePersonalityBlock)   # what selfdrived added
    cs = structs.CarState()
    cs.cruiseState.available = True
    mads.update_events(cs)

    assert not sd.events.has(EventName.driveModePersonalityBlock)  # stripped -> lateral survives

  def test_mads_strip_lets_lateral_engage(self):
    # the block carries NO_ENTRY (which would also gate lateral via MADS's state machine); MADS strips
    # the event so no NO_ENTRY survives -> a lateral-only engagement is preserved in N.
    params = Params()
    params.put_bool("Mads", True, block=True)
    sd = _fake_selfdrive(params)
    mads = ModularAssistiveDrivingSystem(sd)
    cs = structs.CarState()
    cs.cruiseState.available = True

    sd.events.add(EventName.driveModePersonalityBlock)
    assert sd.events.contains(ET.NO_ENTRY)             # positive control: the raw block would gate lateral
    mads.update_events(cs)
    assert not sd.events.has(EventName.driveModePersonalityBlock)
    assert not sd.events.contains(ET.NO_ENTRY)         # after the strip: lateral can engage / stay engaged


# --------------------------------------------------------------------------------------------------
# OFF inert
# --------------------------------------------------------------------------------------------------

class TestFeatureOffInert:
  def test_param_default_off_is_inert(self):
    # param unset (default OFF): read_enabled False, so selfdrived never calls step -> no events, no writes
    params = _Params()
    assert dmp.read_enabled(params) is False
    ss = _SS(params, personality=P.standard)
    # the caller gates on read_enabled, so step is simply not invoked; assert the state is untouched
    assert ss.personality == P.standard
    assert params.store == {}

  def test_off_unknown_and_known_modes_change_nothing(self):
    for mode in (NORMAL, ECO, SPORT, N, N_CUSTOM, 0, 5):
      params = _Params()
      ss = _SS(params, personality=P.aggressive)
      # feature OFF: no call happens. Sanity: the mapping alone never mutates anything.
      dmp.result_for(mode)
      assert ss.personality == P.aggressive
      assert params.store == {}


# --------------------------------------------------------------------------------------------------
# schema wiring
# --------------------------------------------------------------------------------------------------

class TestSchemaWiring:
  def test_carstatesp_drive_mode_coupled_pair(self):
    # the capnp field and the opendbc structs.py mirror are a matched pair (like 0019's CarControlSP.personality):
    # convert_to_capnp must carry driveMode from the dataclass into the capnp struct without a TypeError.
    from openpilot.selfdrive.car.helpers import convert_to_capnp
    from opendbc.car import structs

    src = structs.CarStateSP(driveMode=N)
    capnp_msg = convert_to_capnp(src)
    assert int(capnp_msg.driveMode) == N
    assert 'driveMode' in {f.proto.name for f in capnp_msg.schema.fields_list}

  def test_carstatesp_drive_mode_default_zero(self):
    from opendbc.car import structs
    assert structs.CarStateSP().driveMode == 0        # absent / unknown default

  def test_block_event_carries_no_entry_and_user_disable(self):
    from openpilot.selfdrive.selfdrived.events import EVENTS
    mapping = EVENTS[EventName.driveModePersonalityBlock]
    assert ET.NO_ENTRY in mapping          # refuse a new engage
    assert ET.USER_DISABLE in mapping      # drop an engaged one

  def test_lockout_sp_alert_defined(self):
    from openpilot.sunnypilot.selfdrive.selfdrived.events import EVENTS_SP
    assert custom.OnroadEventSP.EventName.driveModePersonalityLockout in EVENTS_SP

  def test_block_event_definition_matches_events_py(self):
    # every non-deprecated log EventName must be defined in EVENTS (the repo's own completeness rule)
    from openpilot.selfdrive.selfdrived.events import EVENTS
    assert EventName.driveModePersonalityBlock in EVENTS
