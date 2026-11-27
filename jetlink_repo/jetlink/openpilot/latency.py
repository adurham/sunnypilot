"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.

Where a frame's time goes, joined across the link by frame id.

The comma has only ever timed a frame from its own side: the warp, the send,
and then one number for everything the far end did, which is the phone's own
queue and run, the wire both ways, and the wait loop together. And a frame past
HOLD_FRAME publishes the one before it and its reply is read later as a stale
answer, so the frames that miss the deadline were exactly the ones missing from
the statistics: the latency picture was censored at 46 ms.

This joins the two ends by the model frame id. The comma records its stages as
it runs the frame (warp, send, the wait, the publish); the server records its
own and sends them as protocol FRAME_STAMPS, keyed by the same id. The phone's
clock is not the comma's, and no sync between them is worth trusting, so what
crosses is the three spans the phone measured, not its timestamps: the phone's
residence (receive -> send) is the only part of a frame the comma cannot see
for itself, and everything else is subtraction.

A server that sends no stamps (an older phone, or one that ignored the request
flag) costs the joined stages only: the round trip is still reported, which is
the phone's residence and the wire together, and the report says so
(`lat_stamps`). See model_state.log_telemetry.

Reported at the telemetry period, not per frame: at 20 Hz a line of stages
would be most of an rlog to answer a question one line a second does.
"""
from __future__ import annotations

import time
from dataclasses import dataclass

# The stages, in the order a frame passes them. Each is reported as
# <stage>_p50_ms, <stage>_p95_ms and <stage>_max_ms over the window.
# infer and the phone's send are the ingredients of phone_residence, not stages
# of their own: the phone logs its inference time itself (jetlinkTelemetry's
# total_*), and a frame's residency is what the comma cannot see.
STAGES = ('warp', 'send', 'queue', 'phone_residence', 'wire', 'rtt', 'wait', 'publish')
# A frame with no reply by now is not coming: the client's own deadline
# (link.INFERENCE_TIMEOUT, 0.2 s) has failed the link long before, and a held
# frame's late reply lands within a frame or two. Anything older is a frame
# whose stamps will never be joined to anything.
PENDING_S = 5.0
# Frames kept at once: a second of frames at 20 Hz, several times over. A link
# that dies with a burst of frames out must not grow this without bound.
PENDING_KEPT = 256


def _ms(seconds: float) -> float:
  return seconds * 1e3


def percentile(values: list[float], fraction: float) -> float:
  """Nearest rank of the sorted values, indexed round(fraction * (n - 1)),
  the way the server's FrameStats takes its p95 and Trips its p99: a hundred
  frames have a p95 that is one of them, not an interpolation."""
  ordered = sorted(values)
  return ordered[min(len(ordered) - 1, int(round(fraction * (len(ordered) - 1))))]


@dataclass
class Trace:
  """One frame's timings, in milliseconds, as each end measured them. None
  where that end has not said yet, or cannot (an older server sends no
  stamps).

  The comma's are perf_counter seconds from its own frame loop; the phone's
  are spans it measured on its own clock, so they join without a shared
  clock."""
  frame_id: int
  at: float                             # perf_counter of the warp's start
  warp: float | None = None             # warp start -> warped frame ready
  send: float | None = None             # warped frame ready -> request written
  sent_at: float | None = None          # perf_counter the request was written
  reply_at: float | None = None         # perf_counter the reply was read
  waited_from: float | None = None      # perf_counter infer_end began waiting
  published_at: float | None = None     # perf_counter the output was published
  queue: float | None = None            # phone: request read -> inference start
  infer: float | None = None            # phone: inference start -> end
  phone_send: float | None = None       # phone: inference end -> reply written
  hold_deadline: float | None = None    # perf_counter the hold expired
  held: bool = False
  not_sent: bool = False


class Latency:
  """The frames of the last window, joined by frame id and summarised at the
  telemetry period.

  Fed by the frame loop as it runs a frame (sent/not_sent, replied, held,
  published) and by the client as the far end's stamps come off the link
  (stamps). Every call is idempotent for its stage: a reply the client's own
  callback recorded and end() timed again keeps the first, which is the read
  itself rather than the return from it."""

  def __init__(self, now=time.perf_counter):
    self.now = now
    self._open: dict[int, Trace] = {}
    # frames answered since the last report, sampled when it runs so that a
    # held frame's reply, or the phone's stamps, can still arrive first
    self._done: list[Trace] = []
    # the newest frame id reported: a stamp for one of those is a stamp that
    # lost its race with the report, not a frame to open again
    self._reported = 0
    self._window = now()
    self._held = 0        # frames the comma held (sent and not: Trace.not_sent)
    self._held_late = 0   # held frames whose late reply was measured
    self._not_sent = 0    # frames the link could not take
    self._dropped = 0     # frames that expired unanswered

  # -- what the frame loop says ------------------------------------------------

  def sent(self, frame_id: int, t0: float, t1: float, t2: float, sent: float | None) -> None:
    """A frame taken up: its warp began at `t0`, was launched at `t1` and had
    finished at `t2`; `sent` is when its request was written, or None for a
    frame the link could not take."""
    trace = self._trace(frame_id, t0)
    trace.warp = _ms(t2 - t0)
    if sent is not None:
      trace.send = _ms(sent - t2)
      trace.sent_at = sent

  def not_sent(self, frame_id: int) -> None:
    """A frame the link could not take, held without ever going out: it has no
    reply to wait for."""
    self._trace(frame_id, self.now()).not_sent = True
    self._not_sent += 1

  def replied(self, frame_id: int, at: float, waited_from: float | None = None) -> None:
    """The frame's reply was read in userspace at `at`; `waited_from` is when
    infer_end began blocking for it, when this frame loop is the one waiting."""
    trace = self._trace(frame_id, at)
    if trace.reply_at is None:
      trace.reply_at = at
    if waited_from is not None and trace.waited_from is None:
      trace.waited_from = waited_from

  def held(self, frame_id: int, deadline: float) -> None:
    """The frame was published from the one before it, its own reply still
    out. `deadline` is when the hold expired: the reply's lateness is measured
    from it, whenever it lands."""
    trace = self._trace(frame_id, self.now())
    trace.held = True
    if trace.hold_deadline is None:
      trace.hold_deadline = deadline
    self._held += 1

  def published(self, frame_id: int, at: float) -> None:
    """modeld's outputs were handed back at `at`: the parse and the dicts, after
    the reply was read."""
    self._trace(frame_id, at).published_at = at

  # -- what the far end says ---------------------------------------------------

  def stamps(self, frame_id: int, queue_us: int, infer_us: int, send_us: int) -> None:
    """The server's own spans for the frame, in microseconds (protocol
    FRAME_STAMPS). Its residence is their sum. A frame already reported and
    gone is a stamp that lost its race with the report: dropped, not opened
    again. A held frame is still open when its stamps arrive — its reply landed
    a frame or two late — and its wire is exactly what is wanted."""
    if frame_id not in self._open and frame_id <= self._reported:
      return
    trace = self._trace(frame_id, self.now())
    trace.queue = queue_us / 1e3
    trace.infer = infer_us / 1e3
    trace.phone_send = send_us / 1e3

  # -- the window --------------------------------------------------------------

  def report(self) -> dict:
    """The window since the last report as the fields of one log line, or {}
    when no frame of it finished and nothing was held. Clears the window."""
    now = self.now()
    for frame_id, trace in list(self._open.items()):
      if trace.reply_at is not None:
        self._done.append(trace)
        self._reported = max(self._reported, frame_id)
        del self._open[frame_id]
      elif trace.not_sent:
        # never went out, so no reply is coming: counted in lat_not_sent, not
        # dropped
        del self._open[frame_id]
      elif now - trace.at > PENDING_S:
        del self._open[frame_id]
        self._dropped += 1

    rows = [self._sample(trace) for trace in self._done]
    self._done.clear()
    window, self._window = now - self._window, now
    out = {}
    if rows or self._held or self._not_sent or self._dropped:
      stamped = sum(1 for row in rows if 'wire' in row)
      out.update(lat_window_s=round(window, 2), lat_frames=len(rows), lat_stamped=stamped,
                 # the one thing a reader must know before reading wire_ms as
                 # the wire: 'none' is a phone that sent no stamps, and the
                 # round trip it leaves holds the phone's own time too
                 lat_stamps='none' if not stamped else ('all' if stamped == len(rows) else 'some'),
                 lat_held=self._held, lat_held_late=self._held_late,
                 lat_not_sent=self._not_sent, lat_dropped=self._dropped)
      for stage in STAGES:
        values = [row[stage] for row in rows if stage in row]
        if values:
          out[f'{stage}_p50_ms'] = round(percentile(values, 0.50), 1)
          out[f'{stage}_p95_ms'] = round(percentile(values, 0.95), 1)
          out[f'{stage}_max_ms'] = round(max(values), 1)
      late = [row['late'] for row in rows if 'late' in row]
      if late:
        out['late_reply_p50_ms'] = round(percentile(late, 0.50), 1)
        out['late_reply_p95_ms'] = round(percentile(late, 0.95), 1)
        out['late_reply_max_ms'] = round(max(late), 1)
    self._held = self._not_sent = self._dropped = self._held_late = 0
    return out

  # -- internals ---------------------------------------------------------------

  def _trace(self, frame_id: int, at: float) -> Trace:
    trace = self._open.get(frame_id)
    if trace is None:
      if len(self._open) >= PENDING_KEPT:
        self._open.pop(min(self._open), None)
      trace = self._open[int(frame_id)] = Trace(int(frame_id), at)
    return trace

  def _sample(self, trace: Trace) -> dict:
    """The stage values this frame has, in ms. Every one is what an end
    measured; wire is the only subtraction, the round trip less the phone's own
    residence."""
    row = {}
    for stage in ('warp', 'send', 'queue'):
      value = getattr(trace, stage)
      if value is not None:
        row[stage] = value
    queue, infer, phone_send = trace.queue, trace.infer, trace.phone_send
    if queue is not None and infer is not None and phone_send is not None:
      row['phone_residence'] = queue + infer + phone_send
    reply_at = trace.reply_at
    if reply_at is None:
      return row
    if trace.sent_at is not None:
      row['rtt'] = max(0.0, _ms(reply_at - trace.sent_at))
      if 'phone_residence' in row:
        # what is left of the round trip once the phone's own time is out of
        # it: both legs of the wire, however the cable split them
        row['wire'] = max(0.0, row['rtt'] - row['phone_residence'])
    if trace.waited_from is not None:
      row['wait'] = max(0.0, _ms(reply_at - trace.waited_from))
    if trace.published_at is not None:
      row['publish'] = max(0.0, _ms(trace.published_at - reply_at))
    if trace.hold_deadline is not None:
      row['late'] = max(0.0, _ms(reply_at - trace.hold_deadline))
      self._held_late += 1
    return row
