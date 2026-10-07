"""Unit tests for FrameTable (WS-B)."""
import threading
import time

import pytest

from openpilot.offload.contract import FrameEvent
from openpilot.offload.mac.frametable import FrameTable, MIN_RETENTION


def _evt(fid, eid=None, sof=None, eof=None, key=False):
  return FrameEvent(frame_id=fid, encode_id=eid if eid is not None else fid,
                    timestamp_sof=sof if sof is not None else fid * 1000,
                    timestamp_eof=eof if eof is not None else fid * 1000 + 500,
                    is_keyframe=key, payload_bytes=10, recv_mac_ns=time.monotonic_ns())


def test_retention_floor():
  with pytest.raises(ValueError):
    FrameTable(retention=10)
  FrameTable(retention=MIN_RETENTION)  # ok at the floor


def test_append_frame_roundtrip():
  t = FrameTable()
  t.append(_evt(5, eid=7))
  got = t.frame(5)
  assert got is not None and got.encode_id == 7
  assert t.frame(999) is None
  assert 5 in t
  assert t.count == 1 and t.size == 1


def test_overwrite_backfill():
  t = FrameTable()
  t.append(_evt(1))
  t.append(_evt(1))            # same frame_id -> overwrite, not duplicate
  assert t.count == 1 and t.size == 1
  assert t.update(1, decode_ms=2.5)
  assert t.frame(1).decode_ms == 2.5
  assert not t.update(42, decode_ms=1.0)


def test_retention_eviction():
  t = FrameTable(retention=600)
  for i in range(1000):
    t.append(_evt(i))
  assert t.size == 600
  assert t.frame(0) is None
  assert t.frame(999) is not None
  assert t.count == 1000


def test_frame_ids_order():
  t = FrameTable()
  for i in (3, 1, 2):
    t.append(_evt(i))
  assert t.frame_ids() == [3, 1, 2]


def test_join_window_immediate():
  t = FrameTable()
  for i in range(1, 6):
    t.append(_evt(i))
  evt, window = t.join_window(5, timeout_ms=10)
  assert evt.frame_id == 5
  assert [e.frame_id for e in window] == [1, 2, 3, 4, 5]


def test_join_window_stops_at_gap():
  t = FrameTable()
  for i in (1, 2, 4, 5):     # 3 is missing
    t.append(_evt(i))
  evt, window = t.join_window(5, timeout_ms=10)
  assert [e.frame_id for e in window] == [4, 5]


def test_join_window_times_out():
  t = FrameTable()
  t.append(_evt(1))
  t0 = time.monotonic()
  assert t.join_window(2, timeout_ms=100) is None
  assert (time.monotonic() - t0) >= 0.05


def test_join_window_waits_for_append():
  t = FrameTable()
  result = {}

  def waiter():
    result["r"] = t.join_window(7, timeout_ms=2000)

  th = threading.Thread(target=waiter)
  th.start()
  time.sleep(0.05)
  t.append(_evt(7))
  th.join(2)
  assert result["r"] is not None
  assert result["r"][0].frame_id == 7


def test_thread_safe_concurrent_appends():
  t = FrameTable(retention=2000)
  def worker(base):
    for i in range(500):
      t.append(_evt(base + i))
  ths = [threading.Thread(target=worker, args=(b,)) for b in (0, 500, 1000, 1500)]
  for th in ths:
    th.start()
  for th in ths:
    th.join()
  assert t.count == 2000
  assert t.size == 2000
