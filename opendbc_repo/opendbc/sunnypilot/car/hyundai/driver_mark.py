"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

17c FEEDBACK MARKER: the driver's verdict on a felt event, typed by BOOKMARK PRESS COUNT.

WHY
The owner marks felt events with the on-screen bookmark button; what lands in the log is a bare
``userBookmark``/``bookmarkButton`` pair with no verdict and no state, so a review has to guess what each press meant
(route 17c: 15 presses, one of them ``19:13:21 bookmark x2`` that the review read as ONE event). The originally
planned trigger - the steering-wheel star button - was proven to reach none of the logged CAN buses. The bookmark
pair is the mark mechanism that does land (recorded in rlog AND qlog).

THE GESTURE (PLAN-NEXT "Feedback"): 1 press = GOOD, 2 rapid presses = BAD. Every press of a burst is typed with the
burst's verdict, the first included - the burst IS the message.

TYPING IS DEFERRED, NEVER CORRECTED. An isolated press is only KNOWN isolated MARK_GAP_S later, so the record is
written at burst close while the state snapshot is frozen AT THE PRESS (the caller buffers ``mark_snapshot()`` per
press). Cost: a "good" record and its on-screen confirmation appear ~2.0 s after the press; a "bad" burst appears at
the second press. Nothing is ever rewritten, so a record stream read in time order can never show a verdict flip.

WHAT LIVES HERE: the pure typing state machine + the level-window statistics. No I/O, no messaging, no car state - so
both are testable against a fake clock. The write path is one function, ``Fca11LongBrake.on_driver_mark(verdict,
source, snapshot)``, so a future wheel-button trigger is a caller change and not a law change.
"""

# A second press within this many seconds of the previous one makes a BURST (verdict "bad").
MARK_GAP_S = 2.0
# The UI publishes userBookmark AND bookmarkButton per press, back to back, so one press is a PAIR of messages. A
# message arriving this soon after the previous COUNTED press is the other half of that pair and is not a new press.
# (The shortest meaningful second press in this lane is 1.0 s, so a 0.25 s guard cannot eat a real verdict.)
MARK_PAIR_DEDUP_S = 0.25
# "our own level over the last 5 s": the controller runs at 100 Hz, so 5 s is 500 frames.
MARK_LEVEL_WINDOW_FRAMES = 500
# the on-screen confirmation is transient: 2 s at 100 Hz.
MARK_ALERT_FRAMES = 200
# CarStateSP.driverMark codes
MARK_CODE_NONE = 0
MARK_CODE_GOOD = 1
MARK_CODE_BAD = 2
MARK_VERDICT_CODE = {"good": MARK_CODE_GOOD, "bad": MARK_CODE_BAD}

# Every k="m" record carries exactly these keys (plus "k"/"verdict"/"source"); the tests assert the whole set.
MARK_SNAPSHOT_FIELDS = (
  "t_wall", "t_mono_ns", "frame", "ep",
  "lsb_cmd", "braking", "ask_mps2", "a_ego_mps2", "v_ego_mps",
  "standstill", "gas_pressed", "brake_pressed", "personality",
  "clutch_tq_nm", "lead_present", "lead_drel_m", "lead_vrel_mps",
  "engaged", "long_active", "armed",
  "lsb_min", "lsb_max", "lsb_mean", "lsb_frames",
)


def level_window_stats(samples, frame_now: int, window_frames: int = MARK_LEVEL_WINDOW_FRAMES) -> dict:
  """min/max/mean/frames of OUR OWN commanded level over the last ``window_frames`` controller frames.

  ``samples`` is an iterable of ``(frame, lsb)`` for frames that actually SENT an actuating 0x38D (the same population
  the k="b" trace records), oldest first. No samples in the window -> ``min/max/mean`` are null and ``frames`` is 0:
  null, never 0, so "we sent nothing" can never be read as "we commanded zero".
  """
  lo = frame_now - window_frames
  lv = [int(l) for f, l in samples if lo <= f <= frame_now]
  if not lv:
    return dict(lsb_min=None, lsb_max=None, lsb_mean=None, lsb_frames=0)
  return dict(lsb_min=min(lv), lsb_max=max(lv), lsb_mean=round(sum(lv) / len(lv), 2), lsb_frames=len(lv))


class MarkTyping:
  """Press-count typing. Pure: it only ever sees a monotonic clock in ns.

  ``press(t_ns)`` -> the press's sequence id, or None when the message is the other half of a PAIR (the UI publishes
  two services per press). ``poll(t_ns)`` -> the bursts that CLOSED at this clock, as ``(seq, verdict, n_presses)``,
  oldest first; every press of a closed burst is reported exactly once, in press order.
  """

  def __init__(self, gap_s: float = MARK_GAP_S, dedup_s: float = MARK_PAIR_DEDUP_S):
    self._gap_ns = int(gap_s * 1e9)
    self._dedup_ns = int(dedup_s * 1e9)
    self._open: list[int] = []        # sequence ids of the presses in the OPEN burst
    self._last_press_ns: int | None = None
    self._last_counted_ns: int | None = None
    self._seq = 0
    self._done: list[tuple[int, str, int]] = []

  @property
  def open_presses(self) -> int:
    return len(self._open)

  def press(self, t_ns: int) -> int | None:
    """Register one press MESSAGE. Returns the press seq id, or None if it is a duplicate half of the same press."""
    t_ns = int(t_ns)
    if self._last_counted_ns is not None and (t_ns - self._last_counted_ns) < self._dedup_ns:
      return None
    # a press further away than the gap closes the open burst first (it cannot retro-join it)
    if self._open and self._last_press_ns is not None and (t_ns - self._last_press_ns) > self._gap_ns:
      self._finalize()
    seq = self._seq
    self._seq += 1
    self._open.append(seq)
    self._last_press_ns = t_ns
    self._last_counted_ns = t_ns
    return seq

  def poll(self, t_ns: int) -> list[tuple[int, str, int]]:
    """Close the open burst if MARK_GAP_S has elapsed since its last press; return (and clear) the closed bursts."""
    if self._open and self._last_press_ns is not None and (int(t_ns) - self._last_press_ns) > self._gap_ns:
      self._finalize()
    out, self._done = self._done, []
    return out

  def _finalize(self) -> None:
    n = len(self._open)
    if not n:
      return
    verdict = "bad" if n >= 2 else "good"
    self._done.extend((seq, verdict, n) for seq in self._open)
    self._open = []
    self._last_press_ns = None


def mark_source(n_presses: int) -> str:
  """The record's ``source``: the mechanism + the burst shape the press count produced."""
  return "bookmark_single" if n_presses <= 1 else "bookmark_double"
