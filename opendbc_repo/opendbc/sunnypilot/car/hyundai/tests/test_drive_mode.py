"""
CN7 Elantra N drive-mode decode + mapping (adurham fork).

Covers:
  * the DBC signal ``CF_Clu_DriveMode`` (CLU13 0x50C, 44|4) parses out of a real CANParser and
    decodes each mode value, and the DBC VAL_/comment additions are present (no collision with the
    existing CLU13 signals);
  * ``map_drive_mode`` (mode -> personality | BLOCK | UNKNOWN), including unknown/absent values;
  * ``DriveModeDebouncer`` (one-frame debounce, transient rejection, sustained change, unknown hold);
  * the CarStateExt wiring (vl_all -> debounced CarStateSP.driveMode), end to end through a CANParser.
"""
import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.can.dbc import DBC
from opendbc.sunnypilot.car.hyundai import drive_mode as dm
from opendbc.sunnypilot.car.hyundai import carstate_ext

PACKER = CANPacker("hyundai_can_generated")
BUS = 0          # CLU13 is on C-CAN (src 0)
CLU13_DBC = "hyundai_can_generated"


def _parser():
  return CANParser(CLU13_DBC, [("CLU13", 10)], BUS)


def _generated_dbc_path():
  from opendbc import get_generated_dbcs
  import tempfile, os
  p = os.path.join(tempfile.mkdtemp(), "hyundai_can_generated.dbc")
  with open(p, "w") as f:
    f.write(get_generated_dbcs()["hyundai_can_generated"])
  return DBC(p)


class TestDbcSignal:
  def test_signal_present_and_layout(self):
    d = _generated_dbc_path()
    msg = d.name_to_msg["CLU13"]
    sig = next(s for s in msg.sigs.values() if s.name == "CF_Clu_DriveMode")
    assert (sig.msb, sig.lsb, sig.size) == (47, 44, 4)

  def test_no_collision_with_existing_clu13_signals(self):
    d = _generated_dbc_path()
    msg = d.name_to_msg["CLU13"]
    # EcoDriveInf (the N-custom slot on CN7) is untouched at 40|3
    eco = next(s for s in msg.sigs.values() if s.name == "CF_Clu_EcoDriveInf")
    assert (eco.msb, eco.lsb, eco.size) == (42, 40, 3)
    # IsaMainSW at 43|1 (bit 43) is still intact and does not overlap the new nibble (44-47)
    isa = next(s for s in msg.sigs.values() if s.name == "CF_Clu_IsaMainSW")
    assert (isa.msb, isa.lsb, isa.size) == (43, 43, 1)

  def test_val_table_labels(self):
    from opendbc import get_generated_dbcs
    import tempfile, os
    p = os.path.join(tempfile.mkdtemp(), "hyundai_can_generated.dbc")
    with open(p, "w") as f:
      f.write(get_generated_dbcs()["hyundai_can_generated"])
    c = open(p).read()
    assert 'VAL_ 897 CF_Mdps_CurrMode 0 "Normal" 1 "Sport" 3 "N";' in c
    assert "CF_Clu_DriveMode" in c

  @pytest.mark.parametrize("mode,raw", [(1, 0x10), (2, 0x20), (3, 0x30), (6, 0x60), (7, 0x70),
                                        (6, 0x61), (6, 0x62)])
  def test_parser_decodes_each_mode(self, mode, raw):
    pk = _parser()
    dat = bytearray(8)
    dat[5] = raw
    pk.update([(0, [(0x50C, bytes(dat), BUS)])])
    assert pk.vl["CLU13"]["CF_Clu_DriveMode"] == mode


class TestMapDriveMode:
  @pytest.mark.parametrize("mode,expected", [
    (1, dm.DriveModeResult.STANDARD),
    (2, dm.DriveModeResult.RELAXED),
    (3, dm.DriveModeResult.AGGRESSIVE),
    (6, dm.DriveModeResult.BLOCK),
    (7, dm.DriveModeResult.BLOCK),
  ])
  def test_known_modes(self, mode, expected):
    assert dm.map_drive_mode(mode) == expected

  def test_nonnegative_results_are_personality_ints(self):
    # non-negative results must be exactly log.LongitudinalPersonality (aggressive 0 / standard 1 / relaxed 2)
    assert int(dm.DriveModeResult.AGGRESSIVE) == 0
    assert int(dm.DriveModeResult.STANDARD) == 1
    assert int(dm.DriveModeResult.RELAXED) == 2

  @pytest.mark.parametrize("mode", [0, 4, 5, 8, 9, 10, 11, 12, 13, 14, 15, -1, 999, None, "junk", 1.5])
  def test_unknown_modes_never_map(self, mode):
    assert dm.map_drive_mode(mode) == dm.DriveModeResult.UNKNOWN

  def test_block_and_unknown_are_negative(self):
    assert dm.DriveModeResult.BLOCK < 0 and dm.DriveModeResult.UNKNOWN < 0

  def test_mapping_table_is_closed(self):
    # every value the table claims maps to a known result; nothing silently becomes a personality
    for k, v in dm.DRIVE_MODE_TO_RESULT.items():
      assert dm.map_drive_mode(k) == v


class TestDebouncer:
  def test_starts_unknown(self):
    d = dm.DriveModeDebouncer()
    assert d.update([]) == 0

  def test_single_frame_transient_dropped(self):
    d = dm.DriveModeDebouncer()
    d.update([1, 1])           # NORMAL published
    assert d.update([3]) == 1  # one SPORT frame -> still NORMAL
    assert d.update([1]) == 1  # back to NORMAL

  def test_requires_two_consecutive_source_frames(self):
    d = dm.DriveModeDebouncer()
    d.update([1, 1])
    assert d.update([3]) == 1        # first SPORT frame -> not yet
    assert d.update([3]) == 3        # second consecutive SPORT frame -> published
    assert d.update([1]) == 3        # first NORMAL frame -> still SPORT
    assert d.update([1]) == 1        # second NORMAL frame -> published

  def test_change_within_a_single_tick(self):
    # two SPORT frames parsed in ONE update (a 100 Hz tick spanning a 10 Hz frame) publishes SPORT
    d = dm.DriveModeDebouncer()
    d.update([1, 1])
    assert d.update([3, 3]) == 3

  def test_no_frames_keeps_last(self):
    d = dm.DriveModeDebouncer()
    d.update([2, 2])
    assert d.update([]) == 2
    assert d.update([]) == 2

  def test_unknown_hold_from_start(self):
    d = dm.DriveModeDebouncer()
    assert d.update([0, 0]) == 0     # absent -> unknown, and it is held, not a mode

  def test_block_mode_publishes(self):
    d = dm.DriveModeDebouncer()
    d.update([7, 7])
    assert d.update([]) == 7
    assert dm.map_drive_mode(d.update([7])) == dm.DriveModeResult.BLOCK

  def test_publishes_after_n_frames_generic(self):
    d = dm.DriveModeDebouncer(hold_frames=3)
    assert d.update([2]) == 0
    assert d.update([2]) == 0
    assert d.update([2]) == 2


class _ParserShim:
  """Just enough to drive CarStateExt.update_drive_mode (vl_all like a CANParser)."""

  def __init__(self, parser):
    self.vl = parser.vl
    self.vl_all = parser.vl_all


class TestCarStateWiring:
  def test_get_can_parsers_subscribes_clu13(self):
    """Regression for the drive-mode blackout: CarState.get_can_parsers must SUBSCRIBE CLU13 (0x50C) on the
    powertrain parser. CANParser builds message_states ONLY from its `messages` list, so without this the
    parser never decoded CLU13 and CarStateSP.driveMode stayed 0 for every drive. The existing tests below
    subscribe CLU13 by hand, which is exactly why they did not catch the missing subscription.
    """
    from opendbc.car.hyundai.carstate import CarState
    from opendbc.car.hyundai.interface import CarInterface
    from opendbc.car.hyundai.values import CAR
    from opendbc.car import Bus, structs

    CP = structs.CarParams()
    CP.carFingerprint = CAR.HYUNDAI_ELANTRA_2022_NON_SCC
    CP_SP = structs.CarParamsSP()
    CP_SP.enableGasInterceptor = False

    parsers = CarState.get_can_parsers(object.__new__(CarState), CP, CP_SP)
    pt = parsers[Bus.pt]
    assert 0x50C in pt.addresses, "CLU13 (0x50C) must be subscribed on the powertrain parser"
    assert pt.message_states[0x50C].name == "CLU13"
    assert "CF_Clu_DriveMode" in pt.vl["CLU13"], "the drive-mode signal must be decodable on the subscribed msg"
    # the interface builds its parsers through this exact method (interfaces.CarInterfaceBase.__init__:114)
    assert CarInterface.CarState.get_can_parsers is CarState.get_can_parsers

  def test_real_frames_reach_drive_mode_and_block(self):
    """End-to-end on the REAL parser: real CLU13 N-mode frames (0x50C, bus 0) through get_can_parsers must reach
    CarStateSP.driveMode == 7 and map to BLOCK — the exact predicate the superproject's driveModePersonalityBlock
    event keys on (openpilot/sunnypilot/fork/drive_mode_personality.py). Before the subscription this path was dead:
    driveMode stayed 0 on every drive while the bus said the mode.
    """
    from opendbc.car.hyundai.carstate import CarState
    from opendbc.car.hyundai.values import CAR
    from opendbc.car import Bus, structs

    CP = structs.CarParams()
    CP.carFingerprint = CAR.HYUNDAI_ELANTRA_2022_NON_SCC
    CP_SP = structs.CarParamsSP()
    CP_SP.enableGasInterceptor = False
    parsers = CarState.get_can_parsers(object.__new__(CarState), CP, CP_SP)
    pt = parsers[Bus.pt]

    ext = object.__new__(carstate_ext.CarStateExt)
    ext.drive_mode = 0
    ext.drive_mode_debouncer = dm.DriveModeDebouncer()

    def tick(mode, slot, t):
      dat = bytearray(8)
      dat[5] = ((mode & 0xF) << 4) | (slot & 0xF)
      pt.update([(t, [(0x50C, bytes(dat), BUS)])])
      return ext.update_drive_mode(pt)

    assert tick(1, 0, 0) == 0            # NORMAL, not yet published
    assert tick(1, 0, 100_000_000) == 1  # NORMAL published
    assert tick(7, 1, 200_000_000) == 1  # N (slot 1), not yet
    assert tick(7, 1, 300_000_000) == 7  # N published -> the superproject now raises driveModePersonalityBlock
    assert dm.map_drive_mode(7) == dm.DriveModeResult.BLOCK

  def test_update_drive_mode_end_to_end(self):
    pk = _parser()
    shim = _ParserShim(pk)
    ext = object.__new__(carstate_ext.CarStateExt)          # skip __init__ (needs CP/CP_SP)
    ext.drive_mode = 0
    ext.drive_mode_debouncer = dm.DriveModeDebouncer()

    def tick(mode, slot=0, t=0):
      dat = bytearray(8)
      dat[5] = ((mode & 0xF) << 4) | (slot & 0xF)
      pk.update([(t, [(0x50C, bytes(dat), BUS)])])
      return ext.update_drive_mode(shim)

    assert tick(1) == 0            # first NORMAL frame: not yet published
    assert tick(1, t=100_000_000) == 1
    assert tick(2, t=200_000_000) == 1   # ECO not yet published
    assert tick(2, t=300_000_000) == 2
    assert tick(7, t=400_000_000) == 2   # N not yet published
    assert tick(7, t=500_000_000) == 7

  def test_update_drive_mode_tolerates_absent_message(self):
    # a parser that never saw CLU13 -> unknown, no exception
    pk = _parser()
    shim = _ParserShim(pk)
    ext = object.__new__(carstate_ext.CarStateExt)
    ext.drive_mode = 0
    ext.drive_mode_debouncer = dm.DriveModeDebouncer()
    assert ext.update_drive_mode(shim) == 0
