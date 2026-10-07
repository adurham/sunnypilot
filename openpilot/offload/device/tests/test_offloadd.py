"""Offline unit tests for openpilot.offload.device.offloadd.

Runnable on the Mac (no device): uses a real localhost ZMQ pair plus the in-process
cereal msgq bindings (openpilot.cereal.messaging, backed by the editable `msgq` build),
exactly like other openpilot in-process tooling. If the native cereal/msgq bindings are
not importable on the host, the msgq-integration tests are skipped (the pure-function and
byte-level tests still run).

Run:
  PYTHONPATH=$PWD:$PWD/opendbc_repo:$PWD/msgq_repo:$PWD/tinygrad_repo \
    .venv/bin/python -m pytest openpilot/offload/device/tests/test_offloadd.py -v

Determinism note: ZMQ PUB/SUB and msgq PUB/SUB connect asynchronously, so the fixture
waits for the ZMQ subscription to be ACCEPTED (socket monitor) and drives the msgq camera
link until the daemon has seen it. The daemon's *clock* is injected via ``step(now_ns=...)``,
so freshness/stale/gap semantics do not depend on wall-clock timing.
"""
from __future__ import annotations

import os
import socket
import struct
import time

import pytest
import zmq

from openpilot.offload.contract import RETURN_SERVICES
from openpilot.offload.device import offloadd as od

try:
  import msgq  # noqa: F401
  from openpilot.cereal import log as cereal_log
  from openpilot.cereal.messaging import PubMaster, SubMaster, new_message
  _HAVE_CEREAL = True
except Exception:  # pragma: no cover - host without native bindings
  _HAVE_CEREAL = False

needs_cereal = pytest.mark.skipif(not _HAVE_CEREAL, reason="cereal/msgq native bindings not importable")
_TRAV = 2 ** 64 - 1


def _wait_updated(sub, service: str, timeout: float = 2.0) -> bool:
  """Spin ``sub.update`` until ``service`` is seen, or timeout. Returns whether it arrived."""
  end = time.monotonic() + timeout
  while time.monotonic() < end:
    sub.update(200)
    if sub.seen.get(service):
      return True
    time.sleep(0.05)
  return False


def _free_port() -> int:
  s = socket.socket()
  s.bind(("127.0.0.1", 0))
  p = s.getsockname()[1]
  s.close()
  return p


def _load(data: bytes):
  return cereal_log.Event.from_bytes(data, traversal_limit_in_words=_TRAV)


def _camera_state(fid: int, sof_ns: int) -> bytes:
  m = new_message("narrowRoadCameraState")
  m.valid = True
  m.narrowRoadCameraState.frameId = int(fid)
  m.narrowRoadCameraState.timestampSof = int(sof_ns)
  m.narrowRoadCameraState.timestampEof = int(sof_ns) + 5_000_000
  return m.to_bytes()


def _modelv2(fid: int, eof_ns: int, frame_age: int = 3) -> bytes:
  m = new_message("modelV2")
  m.valid = True
  m.modelV2.frameId = int(fid)
  m.modelV2.frameIdExtra = int(fid) + 1000
  m.modelV2.timestampEof = int(eof_ns)
  m.modelV2.frameAge = int(frame_age)
  m.modelV2.modelExecutionTime = 12.5
  return m.to_bytes()


# ---------------------------------------------------------------------------
# pure functions (no msgq / no sockets)
# ---------------------------------------------------------------------------

def test_enabled_mode():
  assert od.enabled_mode("shadow")
  assert od.enabled_mode(b"drive")
  assert od.enabled_mode(" Drive ")
  assert not od.enabled_mode(None)
  assert not od.enabled_mode("")
  assert not od.enabled_mode("off")
  assert not od.enabled_mode(b"\xff\xfe")


def test_read_offload_mode_tolerates_missing_key():
  # absent key -> disabled (Params raises UnknownKeyName; any backend error is tolerated)
  class P2:
    def get(self, _k):
      raise RuntimeError("UnknownKeyName")

  assert od.read_offload_mode(None) is None
  assert od.read_offload_mode(P2()) is None

  class P3:
    def get(self, _k):
      return b"shadow"

  assert od.read_offload_mode(P3()) == "shadow"


def test_sof_for_service_prefers_frameid_map():
  class Msg:
    frameId = 77
    timestampSof = 999

  assert od.sof_for_service(Msg(), {77: 111}) == 111        # frameId map wins
  assert od.sof_for_service(Msg(), {}) == 999               # inline fallback
  assert od.sof_for_service(Msg(), {78: 111}) == 999        # no match -> inline


def test_sof_for_service_none_when_unageable():
  class NoSof:
    frameId = 5

  assert od.sof_for_service(NoSof(), {}) is None


# ---------------------------------------------------------------------------
# byte-level re-stamp semantics (the core contract of §1.3)
# ---------------------------------------------------------------------------

@needs_cereal
def test_restamp_only_changes_header():
  raw = _modelv2(42, 999, frame_age=7)
  out = od._restamp(raw, 123456789)

  with _load(raw) as before, _load(out) as after:
    assert before.which() == after.which() == "modelV2"
    assert after.logMonoTime == 123456789
    # payload untouched, field by field
    assert after.modelV2.frameId == before.modelV2.frameId == 42
    assert after.modelV2.frameIdExtra == before.modelV2.frameIdExtra
    assert after.modelV2.timestampEof == before.modelV2.timestampEof == 999
    assert after.modelV2.frameAge == before.modelV2.frameAge == 7
    assert after.modelV2.modelExecutionTime == before.modelV2.modelExecutionTime


@needs_cereal
def test_restamp_payload_byte_identical():
  """Normalizing the header back must reproduce the original bytes exactly -> payload untouched."""
  raw = _modelv2(42, 999, frame_age=7)

  def normalize(data: bytes, hdr: int) -> bytes:
    with _load(data) as r:
      b = r.as_builder()
      b.logMonoTime = hdr
      return b.to_bytes()

  out = od._restamp(raw, 555)
  assert out != raw                                   # header did change
  assert normalize(out, 111) == normalize(raw, 111)   # payload byte-identical


@needs_cereal
def test_restamp_all_return_services_roundtrip():
  for svc in RETURN_SERVICES:
    e = cereal_log.Event.new_message()
    e.logMonoTime = 1
    e.init(svc)
    out = od._restamp(e.to_bytes(), 7)
    with _load(out) as r:
      assert r.which() == svc
      assert r.logMonoTime == 7


# ---------------------------------------------------------------------------
# integration: localhost ZMQ + in-process msgq
# ---------------------------------------------------------------------------

class Fixture:
  def __init__(self, gap_ms=400, stale_ms=300):
    self.ports_map = {svc: _free_port() for svc in RETURN_SERVICES}
    self.cfg = od.Config(
      stale_ms=stale_ms, gap_ms=gap_ms, recv_timeout_ms=20,
      endpoint_for=lambda svc: f"tcp://127.0.0.1:{self.ports_map[svc]}",
      services=tuple(RETURN_SERVICES),
    )
    self.ctx = zmq.Context()
    self.pub = self.ctx.socket(zmq.PUB)
    self.pub.setsockopt(zmq.LINGER, 0)
    self.pub.bind(self.cfg.endpoint_for("modelV2"))
    self.pub.monitor(f"inproc://offloadd-test-mon-{os.getpid()}", zmq.EVENT_ACCEPTED)
    self.mon = self.pub.get_monitor_socket()

    self.daemon = od.Offloadd(self.cfg)
    self.daemon.open()

    self.cam_pub = PubMaster(["narrowRoadCameraState"]) if _HAVE_CEREAL else None
    self.recv_sub = SubMaster(["modelV2"]) if _HAVE_CEREAL else None

    self._wait_zmq_accepted()
    # SUBSCRIBE propagates just after ACCEPTED; give the link a moment so the first
    # modelV2 send is not lost to the PUB/SUB envelope race.
    time.sleep(0.25)

  # -- ZMQ connection barrier -------------------------------------------------
  def _wait_zmq_accepted(self, timeout=5.0) -> None:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
      if self.mon.poll(100):
        try:
          frames = self.mon.recv_multipart()
        except zmq.ZMQError:
          continue
        evt = struct.unpack("<H", frames[0][:2])[0]
        if evt & zmq.EVENT_ACCEPTED:
          return
    raise AssertionError("ZMQ subscription never accepted (slow-joiner barrier timed out)")

  # -- helpers ----------------------------------------------------------------
  def learn_camera(self, fid: int, sof_fn=None, timeout=5.0) -> tuple[int, int]:
    """Publish a camera state until the daemon has ingested it; return (device-clock now, sof).

    ``sof_fn(now_ns)`` maps the current device clock to the camera SOF to advertise, so tests
    can create deliberately-aged frames without tripping the SOF TTL eviction (< 1 s).
    """
    sof_fn = sof_fn or (lambda n: n)
    end = time.monotonic() + timeout
    while time.monotonic() < end:
      n = time.monotonic_ns()
      sof = sof_fn(n)
      self.cam_pub.send("narrowRoadCameraState", _camera_state(fid, sof))
      self.daemon.step(now_ns=n)
      if self.daemon.sof_by_frame.get(fid) == sof:
        return n, sof
      time.sleep(0.05)
    raise AssertionError("daemon never ingested the camera state")

  def step_until(self, pred, now_ns: int, timeout=5.0) -> bool:
    """Drive the daemon at a fixed device clock until ``pred()`` holds (or timeout)."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
      self.daemon.step(now_ns=now_ns)
      if pred():
        return True
      time.sleep(0.05)
    return False

  def send_modelv2(self, fid: int, eof_ns: int, frame_age: int = 3) -> None:
    self.pub.send(_modelv2(fid, eof_ns, frame_age))

  def close(self):
    self.daemon.close()
    try:
      self.mon.close(0)
    except Exception:
      pass
    self.pub.close(0)
    self.ctx.term()


@needs_cereal
def test_fresh_message_is_restamped_and_published():
  fx = Fixture()
  try:
    n, _sof = fx.learn_camera(1)
    fx.send_modelv2(1, n + 10_000_000)
    publish_now = n + 20_000_000          # 20 ms later -> well within 300 ms
    assert fx.step_until(lambda: fx.daemon.published == 1, publish_now)
    assert fx.daemon.published == 1
    assert fx.daemon.dropped_stale == 0
    assert _wait_updated(fx.recv_sub, "modelV2", 2.0)
    assert fx.recv_sub.logMonoTime["modelV2"] == publish_now   # header = device now
    assert fx.recv_sub["modelV2"].frameId == 1                  # payload intact
    assert fx.recv_sub["modelV2"].frameAge == 3
  finally:
    fx.close()


@needs_cereal
def test_stale_message_is_dropped_not_republished():
  fx = Fixture(stale_ms=300)
  try:
    # camera frame advertised as 900 ms old: > stale (300) but < SOF TTL (1000) so it survives
    n, _sof = fx.learn_camera(1, sof_fn=lambda now: now - 900_000_000)
    fx.send_modelv2(1, n)
    assert fx.step_until(lambda: fx.daemon.dropped_stale == 1, n)
    assert fx.daemon.published == 0
    fx.recv_sub.update(200)
    assert not fx.recv_sub.seen["modelV2"]
  finally:
    fx.close()


@needs_cereal
def test_no_sof_is_dropped():
  fx = Fixture()
  try:
    n = time.monotonic_ns()
    fx.send_modelv2(999, n)                   # frameId 999 was never announced
    assert fx.step_until(lambda: fx.daemon.dropped_no_sof == 1, n)
    assert fx.daemon.published == 0
  finally:
    fx.close()


@needs_cereal
def test_gap_watchdog_fires_then_clears():
  fx = Fixture(gap_ms=100, stale_ms=300)
  try:
    n, _sof = fx.learn_camera(1)
    # establish a valid republish first (this is the gap baseline)
    fx.send_modelv2(1, n + 1_000_000)
    assert fx.step_until(lambda: fx.daemon.published == 1, n + 20_000_000)
    assert not fx.daemon.gap_open

    # nothing valid for > gap_ms (100 ms), age still < stale (300 ms) -> gap opens, no publish
    fx.daemon.step(now_ns=n + 200_000_000)
    assert fx.daemon.gap_open

    # a fresh frame for the same camera frame resumes republish and clears the gap
    fx.send_modelv2(1, n + 200_000_001)
    assert fx.step_until(lambda: fx.daemon.published == 2, n + 200_000_000)
    assert not fx.daemon.gap_open
  finally:
    fx.close()


@needs_cereal
def test_wrong_service_on_port_is_not_republished():
  fx = Fixture()
  try:
    n, _sof = fx.learn_camera(1)
    # send a cameraOdometry event on the modelV2 port: must never be republished
    e = cereal_log.Event.new_message()
    e.logMonoTime = n
    e.init("cameraOdometry")
    e.cameraOdometry.frameId = 1
    fx.pub.send(e.to_bytes())
    assert fx.step_until(lambda: fx.daemon.dropped_error >= 1, n)
    assert fx.daemon.published == 0
  finally:
    fx.close()


def test_stats_json_shape():
  cfg = od.Config(gap_ms=1000)
  d = od.Offloadd(cfg, cereal=od._CerealIO(cfg))
  st = d.stats(now_mono=10.0)
  assert {"published", "dropped_stale", "dropped_no_sof", "dropped_error",
              "zmq_msgs", "gap_open", "uptime_s"}.issubset(st.keys())
  assert st["published"] == 0


if __name__ == "__main__":
  raise SystemExit(pytest.main([os.path.abspath(__file__), "-v"]))
