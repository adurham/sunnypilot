#!/usr/bin/env python3
"""FrameTable — thread-safe FrameEvent store for the WS-B Mac frame path.

Contract (INTERFACES.md §3): the FrameTable is the single source of truth for
FrameEvent rows; exposes append(evt), frame(frame_id), join_window(frame_id,
timeout_ms); thread-safe; retention >= 600 frames.

Rows are the frozen contract dataclass `openpilot.offload.contract.FrameEvent`.
Keyed by frame_id; encode_id is expected 1:1 monotone with frame_id on the wire
(verified on route 00000149: encodeId 43200..44399, frameId 43206..44405).

join_window(frame_id, timeout_ms) blocks until a row for `frame_id` exists (it
may already exist), then returns it together with the contiguous decoded window
ending at that frame, so a consumer can compute per-frame latency for a joined
window. Returns None on timeout. It never invents rows, so frame continuity
(no synthetic/dup frame_ids — G1) is preserved.

Retention: keeps the last `retention` appended rows (>= 600 enforced).
"""
from __future__ import annotations

import threading
import time
from collections import deque

from openpilot.offload.contract import FrameEvent

MIN_RETENTION = 600


class FrameTable:
  """Thread-safe FrameEvent store keyed by frame_id with a bounded ring."""

  def __init__(self, retention: int = 1200):
    if retention < MIN_RETENTION:
      raise ValueError(f"retention must be >= {MIN_RETENTION}, got {retention}")
    self._retention = retention
    self._lock = threading.Lock()
    self._by_id: dict[int, FrameEvent] = {}          # frame_id -> evt (live window)
    self._order: deque[int] = deque()                # insertion order of retained ids
    self._cond = threading.Condition(self._lock)
    self._count = 0                                  # total appended (monotone)

  # --- writers ---------------------------------------------------------------
  def append(self, evt: FrameEvent) -> None:
    """Insert/overwrite the row for evt.frame_id and notify waiters."""
    with self._cond:
      fid = evt.frame_id
      if fid in self._by_id:
        # Overwrite in place (e.g. a decode_ms backfill); keep queue position.
        self._by_id[fid] = evt
      else:
        self._by_id[fid] = evt
        self._order.append(fid)
        self._count += 1
      # Evict oldest beyond the retention window.
      while len(self._order) > self._retention:
        old = self._order.popleft()
        self._by_id.pop(old, None)
      self._cond.notify_all()

  def update(self, frame_id: int, **fields) -> bool:
    """Patch fields on an existing row (e.g. decode_ms, publish_mac_ns).

    Returns True if the row existed. Uses the contract dataclass slots.
    """
    with self._lock:
      evt = self._by_id.get(frame_id)
      if evt is None:
        return False
      for k, v in fields.items():
        setattr(evt, k, v)
      return True

  # --- readers ---------------------------------------------------------------
  def frame(self, frame_id: int) -> FrameEvent | None:
    """Return the row for frame_id, or None if not retained."""
    with self._lock:
      return self._by_id.get(frame_id)

  def join_window(self, frame_id: int, timeout_ms: int = 2000):
    """Wait for `frame_id`, then return (evt, window) where window is the list of
    contiguous FrameEvent rows ending at frame_id (ascending frame_id), scanned
    back over consecutive ids only. Returns None on timeout.

    `window` contains at most the retained rows and stops at the first gap, so it
    never reports a synthetic/duplicate frame.
    """
    deadline = time.monotonic() + max(0, timeout_ms) / 1000.0
    with self._cond:
      while frame_id not in self._by_id:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
          return None
        self._cond.wait(remaining)
      evt = self._by_id[frame_id]
      window = []
      fid = frame_id
      while fid in self._by_id:
        window.append(self._by_id[fid])
        fid -= 1
      window.reverse()
      return evt, window

  # --- introspection ---------------------------------------------------------
  @property
  def count(self) -> int:
    """Total rows appended over the table's lifetime (monotone)."""
    with self._lock:
      return self._count

  @property
  def size(self) -> int:
    """Number of currently retained rows."""
    with self._lock:
      return len(self._by_id)

  @property
  def retention(self) -> int:
    return self._retention

  def frame_ids(self) -> list[int]:
    with self._lock:
      return list(self._order)

  def __contains__(self, frame_id: int) -> bool:
    with self._lock:
      return frame_id in self._by_id
