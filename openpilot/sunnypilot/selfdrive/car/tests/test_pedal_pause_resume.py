"""
fork: Hyundai comma-pedal (gas interceptor) steering-wheel buttons, end to end through openpilot's own engagement path
(buttons-v3).

Real opendbc CarInterface (CAN -> CarState incl. buttonEvents/buttonEnable) -> real CarSpecificEvents (buttonEnable,
buttonCancel, wrongCarMode) + selfdrived's pedalPressed / resumeBlocked rules + CarSpecificEventsSP -> real selfdrived
StateMachine -> real VCruiseHelper (card.py call order).

The model under test:
  * pause/resume (CF_Clu_CruiseSwState 4) is the ONLY on/off for openpilot long. On: at the set speed stored this drive,
    else at the current speed rounded to the display unit (>= 5 mph). Any speed incl. standstill. Never by itself.
  * up / down (1 / 2) ONLY change the set speed (engaged or not): short +/-1 mph, long up = openpilot's long step (+5),
    long down = set speed := current speed (once, no extra -1 on release). They never engage or disengage.
"""
from opendbc.can import CANPacker
from opendbc.car import DT_CTRL, structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.hyundai.values import CAR
from opendbc.sunnypilot.car import crc8_pedal
from opendbc.sunnypilot.car.hyundai import gas_interceptor as gi
from opendbc.sunnypilot.car.interfaces import setup_interfaces
from openpilot.cereal import log
from openpilot.common.constants import CV
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.car.car_events import CarEvents
from openpilot.selfdrive.car.cruise import VCruiseHelper, V_CRUISE_UNSET
from openpilot.selfdrive.car.helpers import convert_to_capnp
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.selfdrive.selfdrived.state import StateMachine
from openpilot.sunnypilot.selfdrive.car.car_specific import CarSpecificEventsSP

EventName = log.OnroadEvent.EventName
CAR_UNDER_TEST = CAR.HYUNDAI_ELANTRA_2022_NON_SCC
FINGERPRINT = {0x260: 8, 0x371: 8, 0x386: 8, 0x394: 8, 0x251: 8, 0x4F1: 4, 0x340: 8, 0x701: 6}
# only the engagement-relevant events; the rest (doors, gear, ...) need a full car simulation and don't matter here
RELEVANT = {EventName.buttonEnable, EventName.buttonCancel, EventName.wrongCarMode, EventName.wrongCruiseMode,
            EventName.pedalPressed, EventName.belowEngageSpeed, EventName.resumeBlocked}
PK = CANPacker("hyundai_can_generated")
PPK = CANPacker(gi.GAS_INTERCEPTOR_DBC)
UP, DOWN, PAUSE = 1, 2, 4  # CF_Clu_CruiseSwState (route 00000128: RES/ACCEL 1 x23, SET/DECEL 2 x39, pause/resume 4 x5)
SHORT = 5                  # CLU11 samples (0.1 s): a short press
LONG = 40                  # CLU11 samples (0.8 s): past CRUISE_LONG_PRESS (50 frames = 0.5 s), before the 2nd tick


def _sensor(gas: int, counter: int):
  dat = bytearray(PPK.make_can_msg(gi.REMAPPED_IDS.sensor_msg, 0, {"INTERCEPTOR_GAS": gas, "INTERCEPTOR_GAS2": gas,
                                                                     "PEDAL_COUNTER": counter & 0xF})[1])
  dat[5] = crc8_pedal(dat[:5])
  return gi.REMAPPED_IDS.sensor_addr, bytes(dat), 0


def mph(v_kph: float) -> int:
  """what the HUD shows (selfdrive/ui/onroad/hud_renderer.py: round(vCruiseCluster * KM_TO_MILE))"""
  return round(v_kph * CV.KPH_TO_MPH)


class _Openpilot:
  def __init__(self, is_metric=False):
    fingerprint = {i: {} for i in range(8)}
    fingerprint[0] = dict(FINGERPRINT)
    CarInterface = interfaces[CAR_UNDER_TEST]
    CP = CarInterface.get_params(CAR_UNDER_TEST, fingerprint, [], alpha_long=False, is_release=False, docs=False)
    CP_SP = CarInterface.get_params_sp(CP, CAR_UNDER_TEST, fingerprint, [], alpha_long=False, is_release_sp=False, docs=False)
    setup_interfaces(CarInterface, CP, CP_SP, [{"HyundaiGasInterceptor": True}])
    assert CP_SP.enableGasInterceptor and not CP.pcmCruise
    self.CP, self.CP_SP = CP, CP_SP
    self.CI = CarInterface(CP, CP_SP)
    self.car_events = CarEvents(CP)
    self.car_events_sp = CarSpecificEventsSP(CP, CP_SP)
    self.sm = StateMachine()
    self.vch = VCruiseHelper(CP, convert_to_capnp(CP_SP))
    self.is_metric = is_metric
    self.enabled = False
    self.enabled_prev = False
    self.CS_prev = None
    self.CS_prev_card = None
    self.t = 0
    self.k = 0
    self.enable_frames = []
    self.alerts = set()

  def step(self, btn=0, brake=False, gas=False, main=False, v=20., experimental=False):
    """one 50 Hz CLU11 sample = two 100 Hz openpilot frames"""
    for sub in range(2):
      self.t += int(DT_CTRL * 1e9)
      spd = v * 3.6
      frames = [
        PK.make_can_msg("EMS16", 0, {"CRUISE_LAMP_M": main, "AliveCounter": self.k % 4}),
        PK.make_can_msg("TCS13", 0, {"DriverOverride": 2 if brake else 0}),
        PK.make_can_msg("WHL_SPD11", 0, {f"WHL_SPD_{w}": spd for w in ("FL", "FR", "RL", "RR")}),
        _sensor(2000 if gas else 0, self.k),
      ]
      if sub == 0:
        frames.append(PK.make_can_msg("CLU11", 0, {"CF_Clu_CruiseSwState": btn}))
      self.k += 1
      # carcontroller stored CarControl.enabled on the previous frame (CI.apply -> CS.pedal_long_engaged)
      self.CI.CS.pedal_long_engaged = self.enabled
      CS, _ = self.CI.update([(self.t, frames)])
      CS = CS.as_reader()

      # card.py order: update_v_cruise, then initialize on the enable edge using the previous CarState
      self.vch.update_v_cruise(CS, self.enabled, is_metric=self.is_metric)
      if self.enabled and not self.enabled_prev:
        self.vch.initialize_v_cruise(self.CS_prev_card, experimental, False)
      self.enabled_prev = self.enabled
      self.CS_prev_card = CS

      # selfdrived: car events + pedalPressed rule + resumeBlocked, then CarSpecificEventsSP (selfdrived order)
      events = Events()
      CC = structs.CarControl(enabled=self.enabled, longActive=self.enabled).as_reader()
      ev = self.car_events.update(CS, self.CS_prev or CS, CC)
      for name in ev.names:
        if name in RELEVANT:
          events.add(name)
      if CS.brakePressed and (self.CS_prev is None or not self.CS_prev.brakePressed or not CS.standstill):
        events.add(EventName.pedalPressed)
      resume_pressed = any(be.type in (structs.CarState.ButtonEvent.Type.accelCruise,
                                       structs.CarState.ButtonEvent.Type.resumeCruise) for be in CS.buttonEvents)
      if self.vch.v_cruise_kph > 250 and resume_pressed:
        events.add(EventName.resumeBlocked)
      events_sp = self.car_events_sp.update(CS, events, self.enabled)
      self.alerts |= set(events.names) | {f"sp:{n}" for n in events_sp.names}

      self.enabled, _ = self.sm.update(events)
      if self.enabled and not self.enabled_prev:
        self.enable_frames.append(self.k)
      self.CS_prev = CS
    return self.enabled

  def run(self, n, **kw):
    for _ in range(n):
      self.step(**kw)
    return self.enabled

  def press(self, btn, n=SHORT, **kw):
    """press for n samples, then release for 10"""
    self.run(n, btn=btn, **kw)
    return self.run(10, **kw)

  @property
  def v_set_mph(self) -> int:
    assert self.vch.v_cruise_cluster_kph == self.vch.v_cruise_kph  # what the HUD shows is the stored set speed
    return mph(self.vch.v_cruise_kph)


class TestPedalButtons(OpenpilotTestCase):
  def _engaged(self, op: _Openpilot, v=20.) -> float:
    """engage with pause/resume (the only way) at v, then raise the set speed with up x2"""
    op.run(10, v=v)
    op.press(PAUSE, v=v)
    assert op.enabled
    op.press(UP, v=v)
    op.press(UP, v=v)
    v_set = op.vch.v_cruise_kph
    assert v_set not in (V_CRUISE_UNSET, 0) and mph(v_set) == round(v * CV.MS_TO_MPH) + 2
    return v_set

  # --- pause/resume: the only on/off ---

  def test_pause_resume_first_engage_at_current_speed(self):
    # no set speed yet this drive: engages at the current speed rounded to mph ('Press Set to Engage' no longer applies)
    for v, want in ((12., 27), (20., 45), (31.3, 70)):
      op = _Openpilot()
      op.run(10, v=v)
      assert op.vch.v_cruise_kph == V_CRUISE_UNSET
      op.press(PAUSE, v=v)
      assert op.enabled, v
      assert op.v_set_mph == want, v
      # stored exactly on a whole mph (so later +/-1 mph steps stay on whole mph), not a rounded km/h value
      assert op.vch.v_cruise_kph == round(want * CV.MPH_TO_KPH, 1), v

  def test_pause_resume_engages_at_any_speed(self):
    # no floor: standstill, crawl, and just below / above the old 25 mph SET/RES floor
    for v, want in ((0., 5), (0.4, 5), (3., 7), (9., 20), (11., 25), (11.7, 26)):
      op = _Openpilot()
      op.run(10, v=v)
      op.press(PAUSE, v=v)
      assert op.enabled, v
      assert op.v_set_mph == want, v
      assert "belowEngageSpeed" not in op.alerts and "sp:pedalBelowEngageSpeed" not in op.alerts

  def test_brake_then_pause_resume_restores_set_speed(self):
    op = _Openpilot()
    v_set = self._engaged(op)
    op.run(5, brake=True, v=15.)
    assert not op.enabled
    assert op.vch.v_cruise_kph == v_set  # retained across the brake disengage (and shown)
    op.run(10, v=15.)
    op.run(5, btn=PAUSE, v=15.)
    assert not op.enabled  # nothing while held
    op.run(5, v=15.)
    assert op.enabled
    assert op.vch.v_cruise_kph == v_set  # previous set speed, not vEgo

  def test_pause_resume_from_standstill_restores_set_speed(self):
    op = _Openpilot()
    v_set = self._engaged(op)
    op.run(5, brake=True, v=0.)
    op.run(10, v=0.)
    assert not op.enabled
    op.press(PAUSE, v=0.)
    assert op.enabled and op.vch.v_cruise_kph == v_set

  def test_pause_while_engaged_disengages(self):
    op = _Openpilot()
    v_set = self._engaged(op)
    op.press(PAUSE)
    op.run(50)
    assert not op.enabled
    assert op.vch.v_cruise_kph == v_set
    op.press(PAUSE)  # and on again at the same set speed
    assert op.enabled and op.vch.v_cruise_kph == v_set

  def test_no_auto_resume_after_brake(self):
    op = _Openpilot()
    self._engaged(op)
    op.run(5, brake=True)
    n = len(op.enable_frames)
    # brake released, speed recovering, 30 s, no button
    for i in range(1500):
      op.step(v=15. + i * 0.01)
      assert not op.enabled
    assert len(op.enable_frames) == n

  def test_pause_resume_held_across_brake_release(self):
    op = _Openpilot()
    self._engaged(op)
    op.run(5, brake=True)
    op.run(5, btn=PAUSE, brake=True)
    op.run(5, btn=PAUSE)
    op.run(50)
    assert not op.enabled

  def test_pause_resume_with_brake_pressed(self):
    op = _Openpilot()
    self._engaged(op)
    op.run(5, brake=True)
    op.run(5)
    op.run(5, btn=PAUSE, brake=True)
    op.run(5, brake=True)
    op.run(50)
    assert not op.enabled

  def test_pause_resume_with_gas_pressed_engages(self):
    # route 00000128 @2152.9/2154.2/2155.7: three presses with the foot on the gas, all refused. Engages (override)
    op = _Openpilot()
    v_set = self._engaged(op)
    op.run(5, brake=True)
    op.run(5)
    op.run(5, btn=PAUSE, gas=True)
    op.run(5, gas=True)
    assert op.enabled and op.vch.v_cruise_kph == v_set

  def test_pause_in_experimental_mode_uses_current_speed(self):
    # route 00000128 (experimental mode ON): SET at 26-30 mph gave a 105 km/h (65 mph) set speed. Must be ~vEgo now.
    op = _Openpilot()
    op.run(10, v=12., experimental=True)
    op.press(PAUSE, v=12., experimental=True)
    assert op.enabled and op.v_set_mph == 27

  def test_metric_first_engage_rounds_to_kph(self):
    op = _Openpilot(is_metric=True)
    op.run(10, v=12.)
    op.press(PAUSE, v=12.)
    assert op.enabled and op.vch.v_cruise_kph == round(12. * 3.6)
    op = _Openpilot(is_metric=True)
    op.run(10, v=0.)
    op.press(PAUSE, v=0.)
    assert op.enabled and op.vch.v_cruise_kph == 8  # V_CRUISE_MIN

  # --- up / down: set speed only, never on/off ---

  def test_up_down_never_engage(self):
    for v in (0., 5., 11., 20., 30.):
      for btn in (UP, DOWN):
        for n in (1, SHORT, LONG, 120):
          op = _Openpilot()
          op.run(10, v=v)
          op.press(btn, n, v=v)
          op.run(40, v=v)
          assert not op.enabled and not op.enable_frames, (v, btn, n)
          assert "belowEngageSpeed" not in op.alerts and "sp:pedalBelowEngageSpeed" not in op.alerts, (v, btn, n)
          assert "buttonEnable" not in op.alerts, (v, btn, n)

  def test_up_down_never_engage_after_set_speed_or_brake(self):
    op = _Openpilot()
    self._engaged(op)
    op.run(5, brake=True)
    op.run(10)
    for btn in (UP, DOWN):
      for n in (SHORT, LONG):
        op.press(btn, n)
        op.press(btn, n, gas=True)
        assert not op.enabled, (btn, n)
    # up/down held across the brake release: still nothing
    op.run(5, btn=UP, brake=True)
    op.run(5, btn=UP)
    op.run(30)
    assert not op.enabled

  def test_up_down_never_disengage(self):
    op = _Openpilot()
    self._engaged(op)
    for btn in (UP, DOWN):
      for n in (SHORT, LONG, 120):
        op.press(btn, n)
        assert op.enabled, (btn, n)

  def test_short_press_one_mph(self):
    op = _Openpilot()
    self._engaged(op)  # 47 mph
    op.press(DOWN)
    assert op.v_set_mph == 46
    op.press(DOWN)
    assert op.v_set_mph == 45
    op.press(UP)
    assert op.v_set_mph == 46

  def test_long_up_step(self):
    # openpilot's normal long press: +5 per 0.5 s held, to the next multiple of 5 (47 -> 50, held longer -> 55)
    op = _Openpilot()
    self._engaged(op)  # 47 mph
    op.press(UP, LONG)
    assert op.v_set_mph == 50  # one tick, no +1 on release
    op.press(UP, 60)  # 1.2 s held (120 frames): ticks at 50 and 100 frames -> two steps
    assert op.v_set_mph == 60

  def test_long_down_sets_current_speed_engaged(self):
    op = _Openpilot()
    self._engaged(op, v=20.)  # 47 mph set, driving 44.7 mph
    op.press(DOWN, LONG, v=26.)  # now at 58.2 mph
    assert op.enabled
    assert op.v_set_mph == 58  # current speed, no extra -1 on release
    assert op.vch.v_cruise_kph == round(58 * CV.MPH_TO_KPH, 1)  # exactly 58 mph (display unit), not rounded km/h
    op.press(DOWN, 150, v=17.)  # held 3 s: only once per press (no repeated action)
    assert op.v_set_mph == round(17. * CV.MS_TO_MPH)

  def test_long_down_acts_once_per_press(self):
    # the speed is captured when the long press is recognised (0.5 s), once: holding on while the speed changes (and
    # past the next 0.5 s ticks) does not re-capture it, and the release applies nothing
    op = _Openpilot()
    self._engaged(op, v=20.)
    op.run(LONG, btn=DOWN, v=26.)    # tick at 0.5 s: 58 mph
    op.run(60, btn=DOWN, v=17.)      # still held 1.2 s more at 38 mph (ticks at 1.0 / 1.5 s)
    op.run(10, v=17.)
    assert op.enabled and op.v_set_mph == 58

  def test_long_down_sets_current_speed_disengaged_no_engage(self):
    op = _Openpilot()
    op.run(10, v=13.)
    op.press(DOWN, LONG, v=13.)  # no set speed yet: creates one = current speed (29 mph), shown
    assert not op.enabled and op.v_set_mph == 29
    op.run(10, v=25.)
    op.press(DOWN, LONG, v=25.)
    assert not op.enabled and op.v_set_mph == 56
    op.press(PAUSE, v=25.)  # pause/resume then uses it
    assert op.enabled and op.v_set_mph == 56

  def test_long_down_at_standstill_min_set_speed(self):
    op = _Openpilot()
    self._engaged(op)
    op.run(5, brake=True, v=0.)
    op.press(DOWN, LONG, v=0.)
    assert not op.enabled and op.v_set_mph == 5  # openpilot's minimum set speed (V_CRUISE_MIN 8 km/h -> 5 mph)

  def test_set_speed_adjusted_while_disengaged_is_used(self):
    # stored set speed retained across a brake disengage, changed by up/down while disengaged (shown), reused by pause
    op = _Openpilot()
    self._engaged(op)  # 47
    op.run(5, brake=True)
    op.run(10)
    op.press(DOWN)
    op.press(DOWN)
    op.press(DOWN)
    assert not op.enabled and op.v_set_mph == 44
    op.press(UP, LONG)
    assert not op.enabled and op.v_set_mph == 45
    op.press(PAUSE)
    assert op.enabled and op.v_set_mph == 45

  def test_short_up_down_without_set_speed_do_nothing(self):
    op = _Openpilot()
    op.run(10)
    for btn in (UP, DOWN):
      op.press(btn)
      op.press(btn, LONG) if btn == UP else None
      assert op.vch.v_cruise_kph == V_CRUISE_UNSET and not op.enabled

  def test_min_set_speed_floor(self):
    op = _Openpilot()
    op.run(10, v=2.)
    op.press(PAUSE, v=2.)
    assert op.v_set_mph == 5
    op.press(DOWN, v=2.)
    assert op.v_set_mph == 5 and op.enabled

  # --- factory MAIN lockout unchanged ---

  def test_arming_factory_main_disengages(self):
    op = _Openpilot()
    self._engaged(op)
    op.run(5, main=True)
    assert not op.enabled
    op.run(5, btn=PAUSE, main=True)
    op.run(20, main=True)
    assert not op.enabled

  def test_factory_main_armed_blocks(self):
    for btn in (UP, DOWN, PAUSE):
      op = _Openpilot()
      op.run(10, main=True)
      op.run(5, btn=btn, main=True)
      op.run(20, main=True)
      assert not op.enabled, btn
