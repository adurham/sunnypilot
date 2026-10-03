"""
fork: Hyundai comma-pedal (gas interceptor) pause/resume button, end to end through openpilot's own engagement path.

Real opendbc CarInterface (CAN -> CarState incl. buttonEvents/buttonEnable) -> real CarSpecificEvents (buttonEnable,
buttonCancel, wrongCarMode) + CarSpecificEventsSP (belowEngageSpeed) + selfdrived's pedalPressed rule -> real selfdrived
StateMachine -> real VCruiseHelper (card.py call order). Proves: brake disengages and NOTHING but a deliberate driver
button press re-engages; the pause/resume press restores the previous set speed.
"""
from opendbc.can import CANPacker
from opendbc.car import DT_CTRL, structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.hyundai.values import CAR
from opendbc.sunnypilot.car import crc8_pedal
from opendbc.sunnypilot.car.hyundai import gas_interceptor as gi
from opendbc.sunnypilot.car.interfaces import setup_interfaces
from openpilot.cereal import log
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
            EventName.pedalPressed,
            EventName.belowEngageSpeed, EventName.resumeBlocked}
PK = CANPacker("hyundai_can_generated")
PPK = CANPacker(gi.GAS_INTERCEPTOR_DBC)
SPEED_RAW_PER_MS = 3.6 / 0.03125  # WHL_SPD11 raw units per m/s


def _sensor(gas: int, counter: int):
  dat = bytearray(PPK.make_can_msg(gi.REMAPPED_IDS.sensor_msg, 0, {"INTERCEPTOR_GAS": gas, "INTERCEPTOR_GAS2": gas,
                                                                     "PEDAL_COUNTER": counter & 0xF})[1])
  dat[5] = crc8_pedal(dat[:5])
  return gi.REMAPPED_IDS.sensor_addr, bytes(dat), 0


class _Openpilot:
  def __init__(self):
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
    self.enabled = False
    self.enabled_prev = False
    self.CS_prev = None
    self.CS_prev_card = None
    self.t = 0
    self.k = 0
    self.enable_frames = []

  def step(self, btn=0, brake=False, gas=False, main=False, v=20.):
    """one 50 Hz CLU11 sample = two 100 Hz openpilot frames"""
    for sub in range(2):
      self.t += int(DT_CTRL * 1e9)
      spd = v * SPEED_RAW_PER_MS * 0.03125
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
      self.vch.update_v_cruise(CS, self.enabled, is_metric=True)
      if self.enabled and not self.enabled_prev:
        self.vch.initialize_v_cruise(self.CS_prev_card, False, False)
      self.enabled_prev = self.enabled
      self.CS_prev_card = CS

      # selfdrived: car events + pedalPressed rule + resumeBlocked
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
      self.car_events_sp.update(CS, events, self.enabled)

      self.enabled, _ = self.sm.update(events)
      if self.enabled and not self.enabled_prev:
        self.enable_frames.append(self.k)
      self.CS_prev = CS
    return self.enabled

  def run(self, n, **kw):
    for _ in range(n):
      self.step(**kw)
    return self.enabled


class TestPedalPauseResume(OpenpilotTestCase):
  def _engaged_at_set_speed(self, op: _Openpilot) -> float:
    op.run(10)
    op.run(5, btn=2)            # SET at 20 m/s
    op.run(5)
    assert op.enabled
    op.run(5, btn=1)            # RES/+ a few times -> set speed above vEgo
    op.run(5)
    op.run(5, btn=1)
    op.run(5)
    v = op.vch.v_cruise_kph
    assert v not in (V_CRUISE_UNSET, 0) and v > 20 * 3.6
    return v

  def test_brake_then_pause_resume_restores_set_speed(self):
    op = _Openpilot()
    v_set = self._engaged_at_set_speed(op)
    op.run(5, brake=True)
    assert not op.enabled
    op.run(10)
    assert not op.enabled
    op.run(5, btn=4)            # pause/resume
    assert not op.enabled       # nothing while held
    op.run(5)
    assert op.enabled
    assert op.vch.v_cruise_kph == v_set  # previous set speed, not vEgo

  def test_no_auto_resume_after_brake(self):
    op = _Openpilot()
    self._engaged_at_set_speed(op)
    op.run(5, brake=True)
    n = len(op.enable_frames)
    # brake released, speed recovering, 30 s, no button
    for i in range(1500):
      op.step(v=15. + i * 0.01)
      assert not op.enabled
    assert len(op.enable_frames) == n

  def test_pause_resume_held_across_brake_release(self):
    op = _Openpilot()
    self._engaged_at_set_speed(op)
    op.run(5, brake=True)
    op.run(5, btn=4, brake=True)
    op.run(5, btn=4)
    op.run(50)
    assert not op.enabled

  def test_pause_resume_with_brake_or_gas_pressed(self):
    for kw in ({"brake": True}, {"gas": True}):
      op = _Openpilot()
      self._engaged_at_set_speed(op)
      op.run(5, brake=True)
      op.run(5)
      op.run(5, btn=4, **kw)
      op.run(5, **kw)
      op.run(50)
      assert not op.enabled, kw

  def test_pause_resume_below_min_enable_speed(self):
    op = _Openpilot()
    self._engaged_at_set_speed(op)
    op.run(5, brake=True)
    v_low = op.CP.minEnableSpeed - 1.
    op.run(10, v=v_low)
    op.run(5, btn=4, v=v_low)
    op.run(50, v=v_low)
    assert not op.enabled

  def test_pause_while_engaged_disengages(self):
    op = _Openpilot()
    self._engaged_at_set_speed(op)
    op.run(5, btn=4)
    op.run(50)
    assert not op.enabled

  def test_resume_before_any_set_is_blocked(self):
    op = _Openpilot()
    op.run(10)
    op.run(5, btn=4)
    op.run(20)
    assert not op.enabled  # resumeBlocked: no previous set speed

  def test_arming_factory_main_disengages(self):
    op = _Openpilot()
    self._engaged_at_set_speed(op)
    op.run(5, main=True)
    assert not op.enabled
    op.run(5, btn=4, main=True)
    op.run(20, main=True)
    assert not op.enabled

  def test_factory_main_armed_blocks(self):
    for btn in (1, 2, 4):
      op = _Openpilot()
      op.run(10, main=True)
      op.run(5, btn=btn, main=True)
      op.run(20, main=True)
      assert not op.enabled, btn
