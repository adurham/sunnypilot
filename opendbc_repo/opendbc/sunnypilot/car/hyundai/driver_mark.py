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

THE GESTURES. The owner's constraint, stated after driving build 3: "I can't take my eyes off the road for 2 seconds to
tap 1 second apart" - a verdict must be ONE action, with no counting and no timing by the driver.
- GOOD: the bookmark SWIPE (drag the bookmark in from the right edge) - unchanged muscle memory. The pair of messages is
  one bookmark press; the original convention is kept on top of it: 1 press = good, a 2nd within MARK_GAP_S = bad.
- BAD: a TWO-FINGER TAP anywhere on the driving screen (openpilot/selfdrive/ui/sunnypilot/mici/onroad/bookmark_icon.py,
  decision in openpilot/sunnypilot/fork/mark_gesture.py). The tap publishes the SAME pair in the REVERSED order -
  ``userBookmark`` first, where the swipe sends ``bookmarkButton`` first - and that order IS the verdict, typed
  immediately. Order rather than a new service or a message count: no new process/param, every upstream consumer still
  sees a normal bookmark pair, and card's SubMaster keeps each message's own logMonoTime (it coalesces same-service
  messages inside a cycle, so a count would not survive the trip). A dropped half stays a bookmark press, so a lost
  message costs a mark instead of inventing a BAD verdict the driver never made.

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
# The UI publishes userBookmark AND bookmarkButton back to back for a BOOKMARK press (measured: <= 0.106 ms apart on
# route 181), so two messages landing inside this window are ONE press. The SAME window decides the two-finger tap: it
# publishes bookmarkButton ALONE, and "alone" is only known this far after its first message.
MARK_PAIR_DEDUP_S = 0.25
# the two-finger tap's source string in the k="m" record (the bookmark swipe uses mark_source()'s strings)
MARK_SOURCE_TOUCH2 = "touch2"
# what the UI can publish per press. Either name alone is a press; together they are ONE bookmark press.
MARK_SERVICES = ("userBookmark", "bookmarkButton")
# The tap is signalled by the ORDER of the pair: the layout publishes ``userBookmark`` FIRST and ``bookmarkButton``
# second, the reverse of the bookmark swipe (upstream's `_on_bookmark_clicked` always sends the button half first).
# ORDER, NOT COUNT: card's SubMaster coalesces several messages of one service inside a cycle (it keeps the last one and
# sets `updated`), so a count is not observable there - but every message's own `logMonoTime` is, which is why the hook
# feeds THIS value in as the press time rather than the frame clock. A lone half (a dropped message) stays a bookmark
# press, so a lost message can only ever cost a mark and never invent a BAD verdict the driver did not make.
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
  """Gesture typing. Pure: it only ever sees a monotonic clock in ns and the service name of each message.

  ``press(t_ns, service)`` -> the press's sequence id, or None when the message is the second half of the press already
  open (the UI publishes two services per bookmark press). ``poll(t_ns)`` -> the verdicts that became KNOWN at this
  clock, as ``(seq, verdict, n_presses, source)``, in the order they became known; every press is reported once.

  TWO GESTURES, one press each, and the DRIVER never counts or times anything:
  - the bookmark swipe -> the pair -> a bookmark press: ONE press = good, a second within MARK_GAP_S = bad (the original
    convention, kept), typed when the burst closes;
  - the two-finger tap -> ``bookmarkButton`` ALONE -> an immediate bad with source MARK_SOURCE_TOUCH2. It is typed the
    moment its press window closes (~0.25 s) and never joins a burst, so a tap can neither be swallowed by, nor
    retro-type, a nearby swipe.
  """

  def __init__(self, gap_s: float = MARK_GAP_S, dedup_s: float = MARK_PAIR_DEDUP_S):
    self._gap_ns = int(gap_s * 1e9)
    self._dedup_ns = int(dedup_s * 1e9)
    self._open: list[int] = []               # sequence ids of the BOOKMARK presses in the OPEN burst
    self._last_burst_ns: int | None = None   # first-message time of the last BOOKMARK press in the burst
    self._pending: dict | None = None        # the press whose window has not closed yet
    self._seq = 0
    self._done: list[tuple[int, str, int, str]] = []

  @property
  def open_presses(self) -> int:
    return len(self._open) + (1 if self._pending is not None else 0)

  def press(self, t_ns: int, service: str = MARK_SERVICES[0]) -> int | None:
    """Register one press MESSAGE of ``service`` (default: the bookmark half). None = the second half of the open press."""
    t_ns = int(t_ns)
    if service not in MARK_SERVICES:
      raise ValueError(f"unknown mark service {service!r}")
    if self._pending is not None and abs(t_ns - self._pending["t_first"]) < self._dedup_ns:
      p = self._pending
      p["services"].add(service)
      if t_ns < p["t_first"]:            # the messages can be offered in any call order, so the EARLIEST one wins
        p["t_first"] = t_ns
        p["first_service"] = service
      return None
    self._classify(t_ns)
    seq = self._seq
    self._seq += 1
    self._pending = {"seq": seq, "services": {service}, "t_first": t_ns, "first_service": service}
    return seq

  def poll(self, t_ns: int) -> list[tuple[int, str, int, str]]:
    """Close what is decidable at this clock: an expired press window, then an expired burst."""
    t_ns = int(t_ns)
    self._classify(t_ns)
    if self._open and self._last_burst_ns is not None and (t_ns - self._last_burst_ns) > self._gap_ns:
      self._finalize()
    out, self._done = self._done, []
    return out

  def _classify(self, t_ns: int) -> None:
    """Decide the pending press once its window has closed.

    The complete pair with ``userBookmark`` FIRST is the two-finger tap (bad, now, source MARK_SOURCE_TOUCH2); anything
    else - the bookmark swipe's pair (button half first), a lone half, a single service - is a bookmark press and joins
    the burst accounting, so a dropped message can only cost a mark and never invent a BAD verdict."""
    p = self._pending
    if p is None or (t_ns - p["t_first"]) < self._dedup_ns:
      return
    self._pending = None
    if len(p["services"]) == len(MARK_SERVICES) and p["first_service"] == MARK_SERVICES[0]:
      self._done.append((p["seq"], "bad", 1, MARK_SOURCE_TOUCH2))
      return
    if self._open and self._last_burst_ns is not None and (p["t_first"] - self._last_burst_ns) > self._gap_ns:
      self._finalize()
    self._open.append(p["seq"])
    self._last_burst_ns = p["t_first"]

  def _finalize(self) -> None:
    n = len(self._open)
    if not n:
      return
    verdict = "bad" if n >= 2 else "good"
    source = mark_source(n)
    self._done.extend((seq, verdict, n, source) for seq in self._open)
    self._open = []
    self._last_burst_ns = None


def mark_source(n_presses: int) -> str:
  """The record's ``source``: the mechanism + the burst shape the press count produced."""
  return "bookmark_single" if n_presses <= 1 else "bookmark_double"
