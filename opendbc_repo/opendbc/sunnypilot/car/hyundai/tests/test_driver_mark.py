"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

17c FEEDBACK marker (k="m") unit tests - car layer.

Covers, per BRIEF-FEEDBACK-marker.md:
  * the pure typing state machine (1 press -> good; 2 presses <= 2.0 s -> BOTH bad; 2 presses 5 s apart -> two goods;
    the UI's per-press PAIR of messages counts ONCE; a burst of 3 is all bad);
  * the 5 s level window (null, never 0, when nothing was sent);
  * the record: EVERY field of a k="m" line asserted from a synthetic CS, plus lead/clutch pass-through;
  * logging disabled -> no record, no file, no exception (and the confirmation still arms);
  * the 2 s confirmation code window;
  * NO mark -> no k="m" at all (negative control) and no file created (idle = no I/O);
  * marks fired mid-drive do not change a single CanData byte of the law's output.
"""
import json
import os
import shutil
import tempfile
import unittest

from opendbc.car import structs
from opendbc.sunnypilot.car.hyundai import driver_mark as dm
from opendbc.sunnypilot.car.hyundai import fca11_long as fl

S = 1_000_000_000

# ---- the numbers MEASURED on the car (route 181: 7 known presses, read from BOTH the rlog and the qlog) -------------
# One press publishes FOUR messages - userBookmark x2 + bookmarkButton x2, the duplication being rlog+qlog - and the
# largest gap WITHIN one press was 0.106 ms. The smallest gap between two DISTINCT presses on that drive was 49.79 s.
# The owner's intended "bad" gesture is two presses about 1.0 s apart. These are observations, not invented bounds:
# they are what makes the 0.25 s pair guard safe (0.25 s / 0.106 ms ~= 2360x margin) and they are pinned below so that
# a later tweak to the guard cannot silently swallow the gesture it exists to keep.
REAL_INTRA_PRESS_GAP_S = 0.000106
REAL_MIN_INTER_PRESS_GAP_S = 49.79
REAL_BAD_GESTURE_S = 1.0


def _real_camera_frame(dec=0, alive=5):
  b = bytearray(8)

  def setb(start, length, val):
    for i in range(length):
      pos = start + i
      b[pos // 8] |= ((int(val) >> i) & 1) << (pos % 8)
  setb(0, 1, 1)          # prefill
  setb(3, 2, 3)          # warn = 3 (no warning)
  setb(8, 8, dec)        # CR_VSM_DecCmd
  setb(18, 2, 2)         # FCA_Status
  setb(23, 4, alive)     # CR_FCA_Alive
  return bytes(b)


class _CS:
  """Minimal CarState double: the attributes the law reads, plus the ones a mark snapshots."""

  def __init__(self, v_ego=20.0, a_ego=0.0, brake=False, gas=False, standstill=False, cam=None,
               lead_m=None, lead_vrel=None, clutch_tq=None, lead_one=None):
    self.out = structs.CarState()
    self.out.vEgo = v_ego
    self.out.aEgo = a_ego
    self.out.gearShifter = structs.CarState.GearShifter.drive
    self.out.brakePressed = brake
    self.out.gasPressed = gas
    self.out.standstill = standstill
    self.interceptor_state = 0
    self.interceptor_fault_seen = False
    self.fca11_cam_frame = cam
    self.fca11_now_nanos = 0
    self.fca11_lead_closing = False
    self.fca11_wheel_min_kph = None
    self.fca11_echoes = []
    self.fca11_clutch_tq_nm = clutch_tq
    if lead_m is not None or lead_one is not None:
      self.fca11_lead_drel_m = lead_m
      self.fca11_lead_vrel_mps = lead_vrel
      self.fca11_lead_one = lead_one if lead_one is not None else object()


def _cc(accel=0.0, enabled=True, long_active=True):
  cc = structs.CarControl()
  cc.enabled = enabled
  cc.longActive = long_active
  cc.actuators.accel = accel
  return cc


def _cp_sp(fca11=True, pedal=True, dyn=False):
  cp_sp = structs.CarParamsSP()
  cp_sp.fca11Brake = fca11
  cp_sp.enableGasInterceptor = pedal
  cp_sp.fca11DynamicRelease = dyn
  return cp_sp


class TestMarkTyping(unittest.TestCase):
  """The gesture, on a fake clock. Pure - no car, no log."""

  def _burst(self, times_s):
    t = dm.MarkTyping()
    seqs = [t.press(int(x * S)) for x in times_s]
    return t, seqs

  def test_single_press_is_good_only_after_the_gap(self):
    t, seqs = self._burst([10.0])
    self.assertEqual(1, seqs[0] is not None and t.open_presses)
    self.assertEqual([], t.poll(int(11.9 * S)), "a press is never typed before the window closes")
    self.assertEqual([(0, "good", 1, "bookmark_single")], t.poll(int(12.05 * S)))

  def test_double_press_is_bad_including_the_first(self):
    t, _ = self._burst([10.0, 11.0])
    self.assertEqual([], t.poll(int(11.5 * S)))
    self.assertEqual([(0, "bad", 2, "bookmark_double"), (1, "bad", 2, "bookmark_double")], t.poll(int(13.1 * S)))

  def test_two_presses_five_seconds_apart_are_two_goods(self):
    t, _ = self._burst([10.0, 15.0])
    out = []
    out += t.poll(int(12.1 * S))
    out += t.poll(int(17.1 * S))
    self.assertEqual([(0, "good", 1, "bookmark_single"), (1, "good", 1, "bookmark_single")], out)

  def test_the_ui_pair_counts_once(self):
    # the UI publishes userBookmark AND bookmarkButton per press, back to back
    t, seqs = self._burst([10.0, 10.002])
    self.assertIsNotNone(seqs[0])
    self.assertIsNone(seqs[1], "the other half of the same press must not become a second press")
    self.assertEqual([(0, "good", 1, "bookmark_single")], t.poll(int(12.1 * S)))

  def test_burst_of_three_is_all_bad(self):
    t, _ = self._burst([10.0, 11.0, 12.0])
    self.assertEqual([(0, "bad", 3, "bookmark_double"), (1, "bad", 3, "bookmark_double"), (2, "bad", 3, "bookmark_double")], t.poll(int(14.1 * S)))

  def test_a_later_press_closes_the_previous_burst_immediately(self):
    t, _ = self._burst([10.0, 20.0])   # press 2 arrives while burst 1 is still open (poll was never called)
    out = t.poll(int(22.1 * S))
    self.assertEqual([(0, "good", 1, "bookmark_single"), (1, "good", 1, "bookmark_single")], out)

  def test_boundary_just_inside_the_gap_is_a_burst(self):
    t, _ = self._burst([10.0, 11.99])
    self.assertEqual("bad", t.poll(int(14.1 * S))[0][1])

  def test_boundary_just_outside_the_gap_is_two_marks(self):
    t, _ = self._burst([10.0, 12.01])
    self.assertEqual([(0, "good", 1, "bookmark_single"), (1, "good", 1, "bookmark_single")], t.poll(int(14.1 * S)))

  def test_source_names(self):
    self.assertEqual("bookmark_single", dm.mark_source(1))
    self.assertEqual("bookmark_double", dm.mark_source(2))
    self.assertEqual("bookmark_double", dm.mark_source(3))


class TestRealPressSpacing(unittest.TestCase):
  """The route-181 measurements, typed through the pure state machine.

  The service NAME never reaches ``MarkTyping`` (both services land in the same ``press()`` call), so the four messages
  of one press are pinned here as a TIME pattern; the rlog/qlog message order is pinned in the glue file, where the
  two services are actually distinguished.
  """

  def _msgs(self, t0_s, offsets_s):
    """Feed one message per offset (in nanoseconds, exactly as ``int(x * 1e9)`` would truncate on the car)."""
    t = dm.MarkTyping()
    return t, [t.press(int(round((t0_s + o) * S))) for o in offsets_s]

  def test_four_messages_of_one_press_collapse_to_one_press(self):
    # route 181: one press = 4 messages, the largest measured gap inside it 0.106 ms
    g = REAL_INTRA_PRESS_GAP_S
    t, seqs = self._msgs(10.0, [0.0, 0.00005, 0.00005 + g, 0.00005 + 2 * g])
    self.assertEqual(1, sum(s is not None for s in seqs), "the four messages are ONE press, not four")
    self.assertEqual(1, t.open_presses)
    self.assertEqual([], t.poll(int(12.0 * S)), "still inside the burst window: nothing typed yet")
    self.assertEqual([(0, "good", 1, "bookmark_single")], t.poll(int(12.1 * S)))

  def test_pair_guard_collapses_one_millisecond(self):
    t, seqs = self._msgs(10.0, [0.0, 0.001])
    self.assertIsNotNone(seqs[0])
    self.assertIsNone(seqs[1], "1 ms is the same press")
    self.assertEqual([(0, "good", 1, "bookmark_single")], t.poll(int(12.1 * S)))

  def test_pair_guard_does_not_collapse_a_quarter_second(self):
    # at exactly the guard value the comparison is strict (<), so the message IS a new press - the boundary is
    # asserted rather than implied: a future widening of MARK_PAIR_DEDUP_S has to argue with this test.
    t, seqs = self._msgs(10.0, [0.0, dm.MARK_PAIR_DEDUP_S])
    self.assertIsNotNone(seqs[1], "0.25 s apart is a NEW press, not the other half of a pair")
    self.assertEqual([(0, "bad", 2, "bookmark_double"), (1, "bad", 2, "bookmark_double")], t.poll(int(12.5 * S)),
                     "and a real second press 0.25 s later is a burst, not two marks")

  def test_the_guard_is_far_below_the_slowest_message_and_the_fastest_real_press(self):
    self.assertGreater(dm.MARK_PAIR_DEDUP_S / REAL_INTRA_PRESS_GAP_S, 2000,
                       "0.25 s vs the measured 0.106 ms intra-press gap: ~2360x, the measured margin")
    self.assertGreaterEqual(REAL_BAD_GESTURE_S / dm.MARK_PAIR_DEDUP_S, 4.0,
                            "the owner's 1.0 s bad gesture must stay >=4x clear of the guard")
    self.assertLess(dm.MARK_PAIR_DEDUP_S, REAL_MIN_INTER_PRESS_GAP_S,
                    "and so must the closest two distinct presses ever measured")

  def test_the_typing_constants_are_pinned(self):
    self.assertEqual(2.0, dm.MARK_GAP_S, "the burst window is 2.0 s; changing it is a gesture change")
    self.assertEqual(0.25, dm.MARK_PAIR_DEDUP_S, "the pair guard is 0.25 s; changing it is a gesture change")
    self.assertLess(dm.MARK_PAIR_DEDUP_S, dm.MARK_GAP_S)

  def test_the_real_bad_gesture_is_two_bad_presses(self):
    # the owner's intended "negative": two presses ~1.0 s apart
    t, seqs = self._msgs(10.0, [0.0, REAL_BAD_GESTURE_S])
    self.assertEqual([0, 1], [s for s in seqs if s is not None])
    self.assertEqual([(0, "bad", 2, "bookmark_double"), (1, "bad", 2, "bookmark_double")], t.poll(int(13.1 * S)),
                     "every press of the burst is typed bad, the first included")

  def test_presses_forty_nine_seconds_apart_are_two_goods(self):
    # the closest two distinct presses on route 181 - far outside MARK_GAP_S, so two separate verdicts
    t, _ = self._msgs(10.0, [0.0, REAL_MIN_INTER_PRESS_GAP_S])
    out = t.poll(int(12.1 * S))
    out += t.poll(int((10.0 + REAL_MIN_INTER_PRESS_GAP_S + 2.1) * S))
    self.assertEqual([(0, "good", 1, "bookmark_single"), (1, "good", 1, "bookmark_single")], out)

  def test_burst_of_three_one_second_apart_is_all_bad(self):
    # the fastest honest rule for 3+ presses: the burst IS the message, so every press of it is typed bad.
    t, _ = self._msgs(10.0, [0.0, REAL_BAD_GESTURE_S, 2 * REAL_BAD_GESTURE_S])
    self.assertEqual([], t.poll(int(13.9 * S)))
    self.assertEqual([(0, "bad", 3, "bookmark_double"), (1, "bad", 3, "bookmark_double"), (2, "bad", 3, "bookmark_double")], t.poll(int(14.1 * S)))


class TestLevelWindow(unittest.TestCase):
  def test_window_stats(self):
    samples = [(100, 4), (200, 8), (400, 2), (900, 30)]
    st = dm.level_window_stats(samples, 600)     # window = frames 100..600 (frame 900 is NEWER than frame_now)
    self.assertEqual(dict(lsb_min=2, lsb_max=8, lsb_mean=round(14 / 3, 2), lsb_frames=3), st)
    st = dm.level_window_stats([], 600)
    self.assertEqual(dict(lsb_min=None, lsb_max=None, lsb_mean=None, lsb_frames=0), st,
                     "no sent frame in the window -> null, never 0")


class TestMarkRecord(unittest.TestCase):
  def setUp(self):
    self.dir = tempfile.mkdtemp(prefix="fca11mark-")
    self._env = os.environ.get("FCA11_LOG_DIR")
    os.environ["FCA11_LOG_DIR"] = self.dir
    self.brake = fl.Fca11LongBrake(structs.CarParams(), _cp_sp())

  def tearDown(self):
    if self._env is None:
      os.environ.pop("FCA11_LOG_DIR", None)
    else:
      os.environ["FCA11_LOG_DIR"] = self._env
    shutil.rmtree(self.dir, ignore_errors=True)

  def _records(self):
    p = os.path.join(self.dir, "fca11-shadow.jsonl")
    if not os.path.exists(p):
      return []
    with open(p) as f:
      return [json.loads(line) for line in f if line.strip()]

  def test_every_snapshot_field_is_populated(self):
    cs = _CS(v_ego=12.5, a_ego=-0.42, gas=False, brake=False, clutch_tq=37.5,
             lead_m=24.0, lead_vrel=-1.75)
    snap = self.brake.mark_snapshot(cs, now_nanos=123_456_789, frame=4242, engaged=True, long_active=True)
    self.assertEqual(set(dm.MARK_SNAPSHOT_FIELDS), set(snap))
    self.assertEqual(4242, snap["frame"])
    self.assertEqual(123456789, snap["t_mono_ns"])
    self.assertAlmostEqual(12.5, snap["v_ego_mps"])
    self.assertAlmostEqual(-0.42, snap["a_ego_mps2"])
    self.assertFalse(snap["gas_pressed"])
    self.assertFalse(snap["brake_pressed"])
    self.assertFalse(snap["standstill"])
    self.assertAlmostEqual(37.5, snap["clutch_tq_nm"])
    self.assertTrue(snap["lead_present"])
    self.assertAlmostEqual(24.0, snap["lead_drel_m"])
    self.assertAlmostEqual(-1.75, snap["lead_vrel_mps"])
    self.assertTrue(snap["engaged"])
    self.assertTrue(snap["long_active"])
    self.assertTrue(snap["armed"])
    self.assertEqual(0, snap["lsb_cmd"])
    self.assertFalse(snap["braking"])
    self.assertEqual(0, snap["lsb_frames"])
    self.assertIsNone(snap["lsb_min"])
    self.assertIsNone(snap["lsb_max"])
    self.assertIsNone(snap["lsb_mean"])
    self.assertIsInstance(snap["t_wall"], float)

  def test_no_lead_means_null_never_zero(self):
    snap = self.brake.mark_snapshot(_CS(), now_nanos=1, frame=1)
    self.assertFalse(snap["lead_present"])
    self.assertIsNone(snap["lead_drel_m"])
    self.assertIsNone(snap["lead_vrel_mps"])

  def test_asked_and_personality_come_from_the_frame_the_law_saw(self):
    self.brake.update(_cc(-1.5), _CS(cam=(0, _real_camera_frame())), 7, personality=2)   # 2 = relaxed (raw)
    snap = self.brake.mark_snapshot(_CS(), now_nanos=1)
    self.assertEqual(7, snap["frame"], "frame defaults to the last controller frame")
    self.assertAlmostEqual(-1.5, snap["ask_mps2"])
    self.assertEqual(2, snap["personality"])

  def test_record_lands_in_the_shared_stream_with_every_field(self):
    snap = self.brake.mark_snapshot(_CS(), now_nanos=5, frame=9, engaged=True, long_active=False)
    rec = self.brake.on_driver_mark("good", "bookmark_single", snap)
    on_disk = [r for r in self._records() if r["k"] == "m"]
    self.assertEqual(1, len(on_disk))
    self.assertEqual(on_disk[0], json.loads(json.dumps(rec)))
    self.assertEqual({"k", "verdict", "source", "schema"} | set(dm.MARK_SNAPSHOT_FIELDS), set(on_disk[0]))
    self.assertEqual("good", on_disk[0]["verdict"])
    self.assertEqual("bookmark_single", on_disk[0]["source"])
    self.assertEqual(2, on_disk[0]["schema"])

  def test_level_window_is_wired_to_the_send_path(self):
    # drive real actuating frames: the level the law puts on the wire must reach the mark's 5 s window
    cs = _CS(cam=(0, _real_camera_frame()))
    for f in range(0, 60, 2):
      self.brake.update(_cc(-2.0), cs, f)
    self.assertTrue(self.brake._lsb_hist, "a sent frame must be remembered")
    snap = self.brake.mark_snapshot(cs, now_nanos=1, frame=58)
    self.assertGreater(snap["lsb_frames"], 0)
    self.assertGreater(snap["lsb_max"], 0)
    self.assertEqual(int(self.brake._dec_last_sent), snap["lsb_cmd"])
    self.assertTrue(snap["braking"])

  def test_share_the_writer_never_two(self):
    # the dynamic-release channel opens the stream first; the mark must ride the SAME writer (one header, one file)
    brake = fl.Fca11LongBrake(structs.CarParams(), _cp_sp(dyn=True))
    self.assertIsNotNone(brake._dyn_log)
    brake.update(_cc(-2.0), _CS(cam=(0, _real_camera_frame())), 0)
    brake.on_driver_mark("bad", "bookmark_double", brake.mark_snapshot(_CS(), now_nanos=1, frame=2))
    recs = self._records()
    self.assertEqual(1, len([r for r in recs if r["k"] == "hdr"]), "one writer per path: never a second header")
    self.assertEqual(1, len([r for r in recs if r["k"] == "m"]))


class TestInert(unittest.TestCase):
  def setUp(self):
    self.dir = tempfile.mkdtemp(prefix="fca11mark-off-")
    self._env = os.environ.get("FCA11_LOG_DIR")

  def tearDown(self):
    if self._env is None:
      os.environ.pop("FCA11_LOG_DIR", None)
    else:
      os.environ["FCA11_LOG_DIR"] = self._env
    shutil.rmtree(self.dir, ignore_errors=True)

  def test_logging_disabled_no_record_no_exception_but_the_confirmation_still_arms(self):
    os.environ["FCA11_LOG_DIR"] = "0"
    brake = fl.Fca11LongBrake(structs.CarParams(), _cp_sp())
    rec = brake.on_driver_mark("bad", "bookmark_double", brake.mark_snapshot(_CS(), now_nanos=1, frame=3))
    self.assertEqual("bad", rec["verdict"], "the record is still built (the caller can log it), it just is not written")
    self.assertEqual(dm.MARK_CODE_BAD, brake.mark_alert_code(3), "the owner must still SEE that the mark landed")

  def test_negative_control_no_mark_no_record_and_no_file(self):
    os.environ["FCA11_LOG_DIR"] = self.dir
    brake = fl.Fca11LongBrake(structs.CarParams(), _cp_sp())
    cs = _CS(cam=(0, _real_camera_frame()))
    for f in range(300):
      brake.update(_cc(-2.0 if f < 150 else 0.0), cs, f)
    self.assertFalse(os.path.exists(os.path.join(self.dir, "fca11-shadow.jsonl")),
                     "no shadow channel, no dynamic channel and no mark -> card must not open the mark stream at all")
    # the pre-existing episode logger (fca11_log, not this lane) may write its own file; nothing may carry a mark
    for name in os.listdir(self.dir):
      with open(os.path.join(self.dir, name)) as fh:
        self.assertNotIn('"k":"m"', fh.read(), f"no k=m record may exist in {name}")
    self.assertEqual(dm.MARK_CODE_NONE, brake.mark_alert_code(299), "an idle drive raises no confirmation")

  def test_marks_never_change_a_byte_of_the_law_output(self):
    os.environ["FCA11_LOG_DIR"] = "0"   # keep the log out of the comparison
    asks = [-2.0, -1.8, -1.2, -0.9, -0.6, -0.4, -0.2, 0.0, 0.5, -3.0] * 12
    plain = fl.Fca11LongBrake(structs.CarParams(), _cp_sp())
    marked = fl.Fca11LongBrake(structs.CarParams(), _cp_sp())
    a, b = [], []
    for f, ask in enumerate(asks):
      cs = _CS(v_ego=18.0 - 0.02 * f, cam=(0, _real_camera_frame(dec=0)))
      a.append([(int(s.address), bytes(s.dat)) for s in plain.update(_cc(ask), cs, f)])
      if f % 10 == 0:   # mark in the middle of the drive, as the driver would
        marked.on_driver_mark("bad", "bookmark_double", marked.mark_snapshot(cs, now_nanos=f, frame=f, engaged=True))
      b.append([(int(s.address), bytes(s.dat)) for s in marked.update(_cc(ask), cs, f)])
    self.assertEqual(a, b, "a mark must not touch a single CanData byte of the law's output")


class TestWriterUnavailable(unittest.TestCase):
  """A mark whose writer cannot exist must still be inert: no record, no exception, confirmation unaffected."""

  def setUp(self):
    self._env = os.environ.get("FCA11_LOG_DIR")
    self.tmp = tempfile.mkdtemp(prefix="fca11mark-nowrite-")

  def tearDown(self):
    if self._env is None:
      os.environ.pop("FCA11_LOG_DIR", None)
    else:
      os.environ["FCA11_LOG_DIR"] = self._env
    shutil.rmtree(self.tmp, ignore_errors=True)

  def test_uncreatable_log_dir_raises_nothing_and_writes_nothing(self):
    # "no log dir": the parent of the configured dir is a FILE, so ShadowLogger's makedirs fails with ENOTDIR and the
    # writer stays disabled - the same inertness as FCA11_LOG_DIR="0", reached the way a bad path reaches it.
    blocker = os.path.join(self.tmp, "not-a-dir")
    with open(blocker, "w") as f:
      f.write("")
    unwritable = os.path.join(blocker, "fca11-log")
    os.environ["FCA11_LOG_DIR"] = unwritable
    brake = fl.Fca11LongBrake(structs.CarParams(), _cp_sp())
    snap = brake.mark_snapshot(_CS(), now_nanos=1, frame=7)
    rec = brake.on_driver_mark("good", "bookmark_single", snap)
    self.assertEqual("good", rec["verdict"], "the record is still built for the caller")
    self.assertEqual(dm.MARK_CODE_GOOD, brake.mark_alert_code(7), "the confirmation never depends on the writer")
    self.assertFalse(os.path.exists(unwritable), "nothing may be created")
    self.assertFalse(os.path.exists(os.path.join(unwritable, "fca11-shadow.jsonl")))
    # and a second mark on the dead writer is still inert
    brake.on_driver_mark("bad", "bookmark_double", brake.mark_snapshot(_CS(), now_nanos=2, frame=9))
    self.assertEqual(dm.MARK_CODE_BAD, brake.mark_alert_code(9))


class TestAlertWindow(unittest.TestCase):
  def test_confirmation_is_2s_then_silent(self):
    os.environ["FCA11_LOG_DIR"] = "0"
    try:
      brake = fl.Fca11LongBrake(structs.CarParams(), _cp_sp())
      brake.update(_cc(), _CS(), 1000)
      brake.on_driver_mark("good", "bookmark_single", brake.mark_snapshot(_CS(), now_nanos=1))
      self.assertEqual(dm.MARK_CODE_GOOD, brake.mark_alert_code(1000))
      self.assertEqual(dm.MARK_CODE_GOOD, brake.mark_alert_code(1000 + dm.MARK_ALERT_FRAMES - 1))
      self.assertEqual(dm.MARK_CODE_NONE, brake.mark_alert_code(1000 + dm.MARK_ALERT_FRAMES))
      self.assertEqual(200, dm.MARK_ALERT_FRAMES, "2 s at the 100 Hz controller rate")
    finally:
      os.environ.pop("FCA11_LOG_DIR", None)


if __name__ == "__main__":
  unittest.main()


class TestTwoFingerTap(unittest.TestCase):
  """The BAD gesture the owner can actually make while driving: a two-finger tap = `bookmarkButton` ALONE.

  The bookmark swipe publishes the pair, the tap publishes only the button half (the UI's `_on_bad_mark`). Everything
  below is about that one difference, and about the tap being ONE action: no counting, no second press, no waiting.
  """

  def test_a_tap_is_bad_immediately_and_says_so(self):
    t = dm.MarkTyping()
    self.assertEqual(0, t.press(int(10.0 * S), "bookmarkButton"))
    self.assertEqual([], t.poll(int(10.2 * S)), "still inside the press window")
    self.assertEqual([(0, "bad", 1, dm.MARK_SOURCE_TOUCH2)], t.poll(int(10.3 * S)),
                     "typed as soon as its own press window closes - not after MARK_GAP_S")

  def test_a_tap_needs_no_second_action(self):
    t = dm.MarkTyping()
    t.press(int(10.0 * S), "bookmarkButton")
    self.assertEqual([(0, "bad", 1, dm.MARK_SOURCE_TOUCH2)], t.poll(int(10.3 * S)))
    self.assertEqual([], t.poll(int(12.5 * S)), "nothing else is typed afterwards")

  def test_a_tap_and_a_swipe_do_not_contaminate_each_other(self):
    t = dm.MarkTyping()
    t.press(int(10.0 * S), "bookmarkButton")                 # tap
    self.assertIsNotNone(t.press(int(11.0 * S), "userBookmark"))   # swipe, its partner 0.1 ms later
    t.press(int(11.0 * S + 100), "bookmarkButton")
    self.assertEqual([(0, "bad", 1, dm.MARK_SOURCE_TOUCH2)], t.poll(int(11.3 * S)))
    self.assertEqual([(1, "good", 1, "bookmark_single")], t.poll(int(13.2 * S)))

  def test_a_swipe_and_a_tap_do_not_contaminate_each_other(self):
    t = dm.MarkTyping()
    t.press(int(10.0 * S), "bookmarkButton")
    t.press(int(10.0 * S + 100), "userBookmark")
    t.press(int(11.0 * S), "bookmarkButton")                 # the tap, 1 s later
    self.assertEqual([(1, "bad", 1, dm.MARK_SOURCE_TOUCH2)], t.poll(int(11.3 * S)))
    self.assertEqual([(0, "good", 1, "bookmark_single")], t.poll(int(13.2 * S)))

  def test_a_tap_never_joins_an_open_burst(self):
    t = dm.MarkTyping()
    t.press(int(10.0 * S), "userBookmark")
    t.press(int(10.0 * S + 100), "bookmarkButton")
    t.press(int(10.5 * S), "bookmarkButton")                 # the tap, INSIDE MARK_GAP_S of the swipe
    self.assertEqual([(1, "bad", 1, dm.MARK_SOURCE_TOUCH2), (0, "good", 1, "bookmark_single")],
                     t.poll(int(13.0 * S)),
                     "the tap is its own record; it neither joins the burst nor makes the swipe bad")

  def test_the_pair_is_never_a_tap_whatever_the_order(self):
    for order in (("bookmarkButton", "userBookmark"), ("userBookmark", "bookmarkButton")):
      with self.subTest(order=order):
        t = dm.MarkTyping()
        t.press(int(10.0 * S), order[0])
        t.press(int(10.0 * S + 100), order[1])
        self.assertEqual([(0, "good", 1, "bookmark_single")], t.poll(int(13.0 * S)))

  def test_the_button_alone_inside_the_window_is_not_a_tap_yet(self):
    # the same first message, but the partner arrives 0.1 s later (inside the 0.25 s window): a bookmark press
    t = dm.MarkTyping()
    t.press(int(10.0 * S), "bookmarkButton")
    t.press(int(10.1 * S), "userBookmark")
    self.assertEqual([(0, "good", 1, "bookmark_single")], t.poll(int(13.0 * S)))

  def test_the_tap_source_is_its_own_name(self):
    self.assertEqual("touch2", dm.MARK_SOURCE_TOUCH2)
    self.assertNotIn(dm.MARK_SOURCE_TOUCH2, (dm.mark_source(1), dm.mark_source(2)),
                     "the record must never confuse the two-finger tap with a bookmark press")

  def test_the_services_are_the_ui_pair(self):
    self.assertEqual(("userBookmark", "bookmarkButton"), dm.MARK_SERVICES)

  def test_an_unknown_service_is_refused(self):
    with self.assertRaises(ValueError):
      dm.MarkTyping().press(int(10.0 * S), "bookmark")
