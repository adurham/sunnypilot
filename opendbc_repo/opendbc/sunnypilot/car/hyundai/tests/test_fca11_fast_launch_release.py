"""FAST LAUNCH RELEASE (fca11_long.FCA11_FAST_LAUNCH_RELEASE_ENABLED, code switch, ENABLED in this shipped patch).

On a positive plan ask from a stop, the FCA11 hold releases at the panda's own per-frame rate
(HYUNDAI_FCA11_LONG_RATE_STEP = 4 LSB per sent frame) down to <= 3 LSB, instead of walking the
~0.5 s comfort ramp. OFF is byte-identical to the deployed path. The switch is read as a module
constant; this build ships with it True.

Tests use the car's own configuration (Elantra 2022 non-SCC, pedal + FCA11, AffineGain + DynamicBrakes +
StandstillClamp) and feed frames through GasInterceptorCarController.create_gas_command, exactly as
the launch-overlap tests do.
"""
import hashlib
import math
import unittest
from unittest import mock

from opendbc.car import structs
import opendbc.sunnypilot.car.hyundai.fca11_long as fl
import opendbc.sunnypilot.car.hyundai.gas_interceptor as gi
from opendbc.sunnypilot.car.hyundai.tests.test_gas_interceptor import _Fca11CS, _fca11_cam_frame, BASE_FINGERPRINT
from opendbc.sunnypilot.car.hyundai.tests.test_gas_interceptor import STD as PEDAL_STD
from opendbc.sunnypilot.car.hyundai.tests.test_gas_interceptor_launch_overlap import _ci, _cc, _frame, _seg, _approach, _hold

D = structs.CarState.GearShifter.drive
PEDAL_ADDRS = (0x200, 0x700)


def _run(seq, fast_release=False, ci=None):
  """Feed seq through create_gas_command; return per-frame records."""
  CI = ci or _ci()
  rec = []
  with mock.patch.object(fl, "FCA11_FAST_LAUNCH_RELEASE_ENABLED", True) if fast_release else _nullctx():
    for f, s in enumerate(seq):
      cam = (0, _fca11_cam_frame(alive=f % 16)) if s.get("cam", "ok") == "ok" else None
      cs = _Fca11CS(v_ego=s["v"], cam=cam, gear=s["gear"], brake=s["brake"], gas=s["gas"])
      cs.out.standstill = s["ss"]
      cs.out.aEgo = 0.0
      sends = CI.CC.create_gas_command(_cc(s["accel"], s["la"]), cs, f, now_nanos=0, personality=fl.PERSONALITY_STANDARD)
      ped = None
      b38 = None
      gas = 0.0
      for a, d, bus in sends:
        if a in PEDAL_ADDRS:
          ped = bytes(d)
        elif a == 0x38D:
          b38 = bytes(d)
      # published gas command (what the pedal WOULD be, before panda stripping)
      gas = float(CI.CC.gas) if hasattr(CI.CC, "gas") else 0.0
      rec.append(dict(f=f, s=s, ped=ped, b38=b38, gas=gas,
                      actuating=b38 is not None and (b38[1] != 0 or (b38[0] & 1) or ((b38[2] >> 4) & 1) or ((b38[3] >> 7) & 1)),
                      level=int(b38[1]) if b38 is not None else 0))
  return rec


class _nullctx:
  def __enter__(self):
    return self
  def __exit__(self, *a):
    return False


def _word(d):
  return (d[0] << 8) | d[1] if d is not None else 0


def _find_release(rec, start):
  """Find the first frame >= start where the host 0x38D stops being actuating (level 0 / passive)."""
  for r in rec[start:]:
    if r["b38"] is not None and not r["actuating"]:
      return r["f"]
  return None


def _sent_levels(rec, start, end):
  """List of (frame, level) for sent host 0x38D frames in [start, end)."""
  out = []
  for r in rec[start:end]:
    if r["b38"] is not None:
      out.append((r["f"], r["level"]))
  return out


class TestSwitch(unittest.TestCase):
  def test_default_on_pinned(self):
    self.assertIs(fl.FCA11_FAST_LAUNCH_RELEASE_ENABLED, True,
                  "FCA11_FAST_LAUNCH_RELEASE_ENABLED is shipped ON in patch 0058: replay proof (0.32 s earlier pedal, "
                  "0 refusals, OFF byte-identical) is recorded in F-LAUNCH-RELEASE-2026-11-25.md")


class TestOnLaunch(unittest.TestCase):
  """ON: a launch from the standstill hold reaches <= 3 LSB fast, with no throttle while level > 0."""

  def _launch_seq(self, hold_frames=300, launch_accel=0.5, launch_frames=200):
    return _approach() + _hold(hold_frames) + _seg(launch_frames, launch_accel, 0.0, ss=True) + _seg(100, launch_accel, 2.0)

  def test_reaches_le3_within_panda_frames(self):
    seq = self._launch_seq()
    off = _run(seq, fast_release=False)
    on = _run(seq, fast_release=True)
    ask_frame = len(_approach()) + len(_hold(300))  # first launch-ask frame
    # find first frame where level <= 3 after the ask
    off_le3 = next((r["f"] for r in off[ask_frame:] if r["level"] > 0 and r["level"] <= 3), None)
    on_le3 = next((r["f"] for r in on[ask_frame:] if r["level"] > 0 and r["level"] <= 3), None)
    self.assertIsNotNone(on_le3)
    # find the level at the ask frame (should be the standstill hold level)
    start_lvl = max(r["level"] for r in on[:ask_frame] if r["actuating"])
    self.assertGreaterEqual(start_lvl, 8)
    # minimum sent frames the panda permits: drop by HYUNDAI_FCA11_LONG_RATE_STEP per sent frame
    min_sent = int(math.ceil((start_lvl - 3) / float(fl.HYUNDAI_FCA11_LONG_RATE_STEP)))
    # convert to controller frames (sent every FCA11_SEND_EVERY)
    min_ctrl = min_sent * fl.FCA11_SEND_EVERY
    self.assertLessEqual(on_le3 - ask_frame, min_ctrl + fl.FCA11_SEND_EVERY,  # one frame slack for the close
                         f"start={start_lvl} min_sent={min_sent} on_le3={on_le3} ask={ask_frame}")
    # OFF takes much longer (comfort ramp ~0.5 s)
    self.assertIsNotNone(off_le3)
    self.assertGreater(off_le3 - ask_frame, on_le3 - ask_frame + 20)

  def test_release_is_faster_than_off(self):
    seq = self._launch_seq()
    off = _run(seq, fast_release=False)
    on = _run(seq, fast_release=True)
    ask_frame = len(_approach()) + len(_hold(300))
    # frames to first passive close after the ask
    off_close = _find_release(off, ask_frame)
    on_close = _find_release(on, ask_frame)
    self.assertIsNotNone(on_close)
    self.assertGreater(off_close - ask_frame, on_close - ask_frame)

  def test_no_throttle_while_brake_level_positive(self):
    seq = self._launch_seq()
    on = _run(seq, fast_release=True)
    ask_frame = len(_approach()) + len(_hold(300))
    close_frame = _find_release(on, ask_frame)
    for r in on[ask_frame:close_frame]:
      # gas_frac > 0 means a non-zero throttle command; it must not appear while an actuating 0x38D is on the wire
      if r["level"] > 0:
        self.assertEqual(r["gas"], 0.0, f"throttle at frame {r['f']} while brake level={r['level']}")

  def test_first_acceptable_pedal_earlier_than_off(self):
    """The panda will accept a pedal frame once the wire-level window (11 controller frames) expires after the last
    actuating 0x38D. The fast release closes the episode sooner, so the first acceptable pedal slot is earlier."""
    seq = self._launch_seq()
    off = _run(seq, fast_release=False)
    on = _run(seq, fast_release=True)
    ask_frame = len(_approach()) + len(_hold(300))
    # first pedal frame with gas > 0 after the ask
    off_ped = next((r["f"] for r in off[ask_frame:] if r["ped"] is not None and _word(r["ped"]) > 0), None)
    on_ped = next((r["f"] for r in on[ask_frame:] if r["ped"] is not None and _word(r["ped"]) > 0), None)
    self.assertIsNotNone(on_ped)
    self.assertIsNotNone(off_ped)
    self.assertLessEqual(on_ped - ask_frame, off_ped - ask_frame)


class TestUnchangedPaths(unittest.TestCase):
  """ON must not change stop/hold or releases on negative / coast asks."""

  def test_hold_and_negative_ask_unchanged(self):
    """A hold followed by dithering negative asks (no positive launch) uses the comfort ramp in both arms."""
    seq = _approach() + _hold(200) + [_frame(accel=-0.1 - 0.05 * (i % 3), v=0.0, ss=True) for i in range(200)]
    off = _run(seq, fast_release=False)
    on = _run(seq, fast_release=True)
    # byte-for-byte identical 0x38D stream
    off_b38 = [r["b38"] for r in off]
    on_b38 = [r["b38"] for r in on]
    self.assertEqual(off_b38, on_b38)

  def test_stop_hold_unchanged(self):
    """A plain stop with no release: the hold levels are identical."""
    seq = _approach() + _hold(100)
    off = _run(seq, fast_release=False)
    on = _run(seq, fast_release=True)
    self.assertEqual([r["b38"] for r in off], [r["b38"] for r in on])

  def test_coast_release_unchanged(self):
    """A release onto a coast ask (accel just above the release threshold, but not positive) uses comfort ramp."""
    seq = _approach() + _hold(100) + _seg(100, -0.1, 0.0, ss=True) + _seg(100, -0.1, 1.0)
    off = _run(seq, fast_release=False)
    on = _run(seq, fast_release=True)
    self.assertEqual([r["b38"] for r in off], [r["b38"] for r in on])

  def test_driver_tap_ends_ramp_immediately(self):
    """A driver brake tap during the launch is a veto: the episode closes on that frame in both arms."""
    base = _approach() + _hold(100) + _seg(30, 0.5, 0.0, ss=True)
    tap_frame = len(base)
    seq = base + [_frame(accel=0.5, v=0.0, ss=True, brake=True)] + _seg(60, 0.5, 0.0, ss=True)
    off = _run(seq, fast_release=False)
    on = _run(seq, fast_release=True)
    # the tap frame and every frame after it must carry no actuating host 0x38D in either arm
    for r in off[tap_frame:]:
      self.assertFalse(r["actuating"], f"OFF actuating after driver tap at frame {r['f']}")
    for r in on[tap_frame:]:
      self.assertFalse(r["actuating"], f"ON actuating after driver tap at frame {r['f']}")
    # no throttle while any level was still on the wire after the tap
    for r in on[tap_frame:tap_frame + 30]:
      if r["level"] > 0:
        self.assertEqual(r["gas"], 0.0, f"throttle after tap at frame {r['f']}")


if __name__ == "__main__":
  unittest.main()
