"""
CN7 Elantra N fuel level from CLU13 (adurham fork; KDMLink decode).

Covers:
  * the DBC signal ``CF_Clu_FuelLevel`` (CLU13 0x50C, 24|16 - added by opendbc patch 0046) parses out
    of a real CANParser and the counts -> percent conversion matches the KDMLink calibration
    (raw * 0.30201 - 612.17);
  * the validity band: out-of-band counts/frames read "no data", never a false empty;
  * ``latest_fuel_pct`` last-valid selection over a vl_all window;
  * ``FuelLevelTracker``: step acceptance, gaps hold, and rejection of the two real bus transients
    (start burst -> out of band; one-frame end-of-drive zero payload, raw 2048 -> +6.3 %, in band);
  * the CarStateExt wiring (vl_all -> published fuel level), end to end through a CANParser.
"""
import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.can.dbc import DBC
from opendbc.sunnypilot.car.hyundai import carstate_ext
from opendbc.sunnypilot.car.hyundai import fuel_level as fl

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


class _Shim:
  def __init__(self, vals):
    self.vl_all = {"CLU13": {"CF_Clu_FuelLevel": vals}}


# real values seen on the owner's routes (2026-10-13 sweep)
JUNK_END = 2048 * 0.30201 - 612.17        # +6.35 %: the end-of-drive zero-payload frame
BURST_START = 6361 * 0.30201 - 612.17     # +1309 %: the ~6 s start burst on some ignitions


class TestDbcSignal:
  def test_signal_present_and_layout(self):
    d = _generated_dbc_path()
    msg = d.name_to_msg["CLU13"]
    sig = next(s for s in msg.sigs.values() if s.name == "CF_Clu_FuelLevel")
    assert (sig.msb, sig.lsb, sig.size) == (39, 24, 16)

  def test_conversion_matches_kdmlink_calibration(self):
    # real 2022 EN DCT readings from the owner's routes: raw 2332-2333 -> 92.117-92.419 %
    assert fl.fuel_pct_from_counts(2332) == pytest.approx(92.117, abs=0.01)
    assert fl.fuel_pct_from_counts(2333) == pytest.approx(92.419, abs=0.01)
    # the sender's full-tank rail is ~2358 counts = ~100 %
    assert fl.fuel_pct_from_counts(2358) == pytest.approx(100.0, abs=0.1)


class TestValidityBand:
  def test_out_of_band_counts_are_no_data(self):
    assert fl.fuel_pct_from_counts(0) == 0.0          # raw 0 -> -612 %
    assert fl.fuel_pct_from_counts(1287) == 0.0       # 2024+ facelift band -> must read no data
    assert fl.fuel_pct_from_counts(3000) == 0.0       # junk above the cap
    assert fl.fuel_pct_from_counts(65535) == 0.0

  def test_start_burst_is_out_of_band(self):
    assert fl.fuel_pct_from_counts(6361) == 0.0       # +1309 % start burst -> dropped
    assert fl.fuel_pct_from_counts(6445) == 0.0       # +1334 % variant


class TestLatestFuelPct:
  def test_last_valid_frame_wins(self):
    assert fl.latest_fuel_pct([92.4, 92.1]) == pytest.approx(92.1)

  def test_bad_frames_are_skipped(self):
    assert fl.latest_fuel_pct([92.1, BURST_START]) == pytest.approx(92.1)

  def test_no_valid_frame(self):
    assert fl.latest_fuel_pct([]) is None
    assert fl.latest_fuel_pct([BURST_START]) is None

  def test_valid_band_near_empty_clamps_to_zero_not_none(self):
    assert fl.latest_fuel_pct([-3.6]) == 0.0


class TestTracker:
  def test_fresh_ignition_accepts_first_valid_value(self):
    tr = fl.FuelLevelTracker()
    assert tr.update([]) == 0.0                       # nothing yet
    assert tr.update([BURST_START]) == 0.0            # burst ignored
    assert tr.update([95.14]) == pytest.approx(95.14)

  def test_normal_sender_steps_are_accepted(self):
    tr = fl.FuelLevelTracker()
    tr.update([95.14])
    for v in (94.83, 94.53, 94.23, 93.93):
      assert tr.update([v]) == pytest.approx(v)

  def test_gaps_hold_the_last_value(self):
    tr = fl.FuelLevelTracker()
    tr.update([64.63])
    assert tr.update([]) == pytest.approx(64.63)
    assert tr.update([]) == pytest.approx(64.63)

  def test_end_of_drive_junk_frame_is_rejected(self):
    """The real shutdown frame (raw 2048, +6.35 %) is IN band; observed runs are <= 3 frames."""
    tr = fl.FuelLevelTracker()
    tr.update([64.33])
    for _ in range(3):                                # a 3-frame run, worst observed
      assert tr.update([JUNK_END]) == pytest.approx(64.33)
    # ... and the level re-tracks normally afterwards
    assert tr.update([64.33]) == pytest.approx(64.33)
    assert tr.update([64.03]) == pytest.approx(64.03)

  def test_sustained_jump_is_accepted_after_confirmation(self):
    """A real step change while powered (refuel) confirms after FUEL_CONFIRM_WINDOWS sightings."""
    tr = fl.FuelLevelTracker()
    tr.update([21.44])
    for i in range(fl.FUEL_CONFIRM_WINDOWS - 1):
      assert tr.update([97.27]) == pytest.approx(21.44)
    assert tr.update([97.27]) == pytest.approx(97.27)


class TestCarStateWiring:
  def test_update_reads_and_holds_last_good(self):
    ext = carstate_ext.CarStateExt(None, None)
    assert ext.update_fuel_level(_Shim([92.4])) == pytest.approx(92.4)
    # a bad window must HOLD the last good value, not flicker to no-data
    assert ext.update_fuel_level(_Shim([])) == pytest.approx(92.4)
    assert ext.update_fuel_level(_Shim([BURST_START])) == pytest.approx(92.4)
    assert ext.update_fuel_level(_Shim([JUNK_END])) == pytest.approx(92.4)
    assert ext.update_fuel_level(_Shim([91.9])) == pytest.approx(91.9)

  def test_no_valid_frame_ever_reads_zero(self):
    ext = carstate_ext.CarStateExt(None, None)
    assert ext.update_fuel_level(_Shim([])) == 0.0
    assert ext.update_fuel_level(_Shim([BURST_START])) == 0.0
    assert ext.update_fuel_level(_Shim([1287 * 0.30201 - 612.17])) == 0.0

  def test_real_frames_reach_fuel_level(self):
    """End-to-end on the REAL parser: CLU13 frames packed with a real EN reading must reach the
    published fuel level through the vl_all read path (signal added by opendbc patch 0046)."""
    cp = _parser()
    ext = carstate_ext.CarStateExt(None, None)
    dat = PACKER.pack(0x50C, {"CF_Clu_FuelLevel": 92.12})
    cp.update([(1_000_000, [(0x50C, bytes(dat), BUS)])])
    assert ext.update_fuel_level(cp) == pytest.approx(92.117, abs=0.05)
    # a following shutdown-style frame (raw 2048) on the bus must not pull the level down
    dat2 = PACKER.pack(0x50C, {"CF_Clu_FuelLevel": 6.35})
    cp.update([(2_000_000, [(0x50C, bytes(dat2), BUS)])])
    assert ext.update_fuel_level(cp) == pytest.approx(92.117, abs=0.05)
