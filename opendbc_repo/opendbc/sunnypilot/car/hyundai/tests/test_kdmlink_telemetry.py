"""
KDMLink-derived telemetry (adurham fork, patch 0048): intake temp / accels / battery / N-mode block.

Covers:
  * the DBC layout of the four messages folded in by patch 0046 (EMS_453, ESP_49D, BATT_568, N_MODE_4A0);
  * the CarStateExt wiring: packed frames -> real CANParser -> the CarStateSP fields, including the
    real SPORT-mode N-mode pattern (0x55 / 0x50) and the all-zero NORMAL pattern;
  * the parser subscription list (get_can_parsers) carrying the four messages (regression like 0028).
"""
import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.can.dbc import DBC
from opendbc.car import Bus, structs
from opendbc.sunnypilot.car.hyundai import carstate_ext

PACKER = CANPacker("hyundai_can_generated")
BUS = 0
DBC_NAME = "hyundai_can_generated"
SUBS = [("EMS_453", 100), ("ESP_49D", 10), ("BATT_568", 20), ("N_MODE_4A0", 20)]


def _parser():
  return CANParser(DBC_NAME, SUBS, BUS)


def _generated_dbc_path():
  from opendbc import get_generated_dbcs
  import tempfile, os
  p = os.path.join(tempfile.mkdtemp(), "hyundai_can_generated.dbc")
  with open(p, "w") as f:
    f.write(get_generated_dbcs()["hyundai_can_generated"])
  return DBC(p)


def _run(addr, values):
  cp = _parser()
  ext = carstate_ext.CarStateExt(None, None)
  ret_sp = structs.CarStateSP()
  dat = PACKER.pack(addr, values)
  cp.update([(1_000_000, [(addr, bytes(dat), BUS)])])
  ext.update_telemetry(ret_sp, cp)
  return ret_sp


class TestDbcLayout:
  def test_messages_present(self):
    d = _generated_dbc_path()
    for name, addr in (("EMS_453", 0x453), ("ESP_49D", 0x49D), ("BATT_568", 0x568), ("N_MODE_4A0", 0x4A0)):
      assert d.name_to_msg[name].address == addr

  def test_signal_layouts(self):
    d = _generated_dbc_path()
    nm = d.name_to_msg["N_MODE_4A0"]
    assert (nm.sigs["NMode_Engine"].lsb, nm.sigs["NMode_Engine"].size) == (0, 2)
    assert (nm.sigs["NMode_Steering"].lsb, nm.sigs["NMode_Steering"].size) == (12, 2)
    assert nm.sigs["NMode_RevMatch"].size == 1
    e = d.name_to_msg["ESP_49D"]
    assert (e.sigs["LateralAccel"].lsb, e.sigs["LateralAccel"].size) == (0, 16)
    assert (e.sigs["LongAccel"].lsb, e.sigs["LongAccel"].size) == (24, 12)


class TestWiring:
  def test_intake_temp(self):
    ret = _run(0x453, {"IntakeTemp": 52.5})
    assert ret.intakeTemp == pytest.approx(52.5, abs=0.4)

  def test_battery_voltage(self):
    ret = _run(0x568, {"BatteryVoltage": 13.4})
    assert ret.batteryVoltage == pytest.approx(13.4, abs=0.11)

  def test_accels(self):
    ret = _run(0x49D, {"LateralAccel": 0.21, "LongAccel": -0.35})
    assert ret.lateralAccel == pytest.approx(0.21, abs=0.0002)
    assert ret.longAccel == pytest.approx(-0.35, abs=0.002)

  def test_nmode_sport_pattern(self):
    """Real SPORT-mode frames on this car read bytes 0x55 / 0x50 -> engine/susp/esc/exh = 1,
    revmatch/eLSD = 0, steering = 1 (the 0x55 nibble pattern + steering bit 12)."""
    ret = _run(0x4A0, {"NMode_Engine": 1, "NMode_Suspension": 1, "NMode_ESC": 1, "NMode_Exhaust": 1,
                       "NMode_RevMatch": 0, "NMode_eLSD": 0, "NMode_Steering": 1})
    assert (ret.nModeEngine, ret.nModeSuspension, ret.nModeEsc, ret.nModeExhaust) == (1, 1, 1, 1)
    assert (ret.nModeRevMatch, ret.nModeELSD, ret.nModeSteering) == (0, 0, 1)

  def test_nmode_normal_pattern_is_zero(self):
    """NORMAL mode: the whole block reads zero on the bus."""
    ret = _run(0x4A0, {})
    assert (ret.nModeEngine, ret.nModeSuspension, ret.nModeEsc, ret.nModeExhaust,
            ret.nModeRevMatch, ret.nModeELSD, ret.nModeSteering) == (0,) * 7

  def test_fields_are_int_for_uint8(self):
    ret = _run(0x4A0, {"NMode_Engine": 2})
    assert isinstance(ret.nModeEngine, int) and isinstance(ret.nModeRevMatch, int)


class TestSubscriptions:
  def test_get_can_parsers_subscribes_telemetry_messages(self):
    """0048 regression: the four telemetry messages must be in the powertrain parser's subscription
    list (validity hygiene; vl_all-style reads would otherwise never decode them)."""
    from opendbc.car.hyundai.carstate import CarState
    from opendbc.car.hyundai.values import CAR

    CP = structs.CarParams()
    CP.carFingerprint = CAR.HYUNDAI_ELANTRA_2022_NON_SCC
    CP_SP = structs.CarParamsSP()
    CP_SP.enableGasInterceptor = False

    parsers = CarState.get_can_parsers(object.__new__(CarState), CP, CP_SP)
    pt = parsers[Bus.pt]
    for addr in (0x50C, 0x453, 0x49D, 0x568, 0x4A0):
      assert addr in pt.addresses, f"{hex(addr)} must be subscribed on the powertrain parser"
    assert "IntakeTemp" in pt.vl["EMS_453"]
    assert "NMode_Engine" in pt.vl["N_MODE_4A0"]
