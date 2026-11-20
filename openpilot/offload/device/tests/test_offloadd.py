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
  import msgq
  from openpilot.cereal import log as cereal_log
  from openpilot.cereal.messaging import PubMaster, new_message
  from openpilot.cereal.services import SERVICE_LIST
  # namespace this process's msgq topics at import, before any socket/context exists, so other
  # sessions on the shared Mac cannot feed our camera subscription a stray frameId (RESULTS.md).
  msgq.set_fake_prefix(f"offloadtest-{os.getpid()}")
  _HAVE_CEREAL = True
except Exception:  # pragma: no cover - host without native bindings
  _HAVE_CEREAL = False
  SERVICE_LIST = {}

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


def _modeldatav2sp(turn: int = 0, left: bool = False, right: bool = False) -> bytes:
  """A modelDataV2SP return message (custom.ModelDataV2SP: no frameId, no timestamps)."""
  m = new_message("modelDataV2SP")
  m.valid = True
  m.modelDataV2SP.laneTurnDirection = int(turn)
  m.modelDataV2SP.leftLaneChangeEdgeBlock = bool(left)
  m.modelDataV2SP.rightLaneChangeEdgeBlock = bool(right)
  return m.to_bytes()


def _fields_without_header(raw: bytes) -> dict:
  """Decode a raw Event and return its payload dict minus the header (for byte-identity checks)."""
  with _load(raw) as r:
    d = r.to_dict()
  d.pop("logMonoTime", None)
  return d


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
  def __init__(self, gap_ms=400, stale_ms=300, cfg_extra=None):
    self.ports_map = {svc: _free_port() for svc in RETURN_SERVICES}
    extra = dict(cfg_extra or {})
    extra.setdefault("hold_policy", False)   # default-off unless a test opts in
    self.cfg = od.Config(
      stale_ms=stale_ms, gap_ms=gap_ms, recv_timeout_ms=20,
      endpoint_for=lambda svc: f"tcp://127.0.0.1:{self.ports_map[svc]}",
      services=tuple(RETURN_SERVICES),
      **extra,
    )
    self.ctx = zmq.Context()
    self.pub = self.ctx.socket(zmq.PUB)
    self.pub.setsockopt(zmq.LINGER, 0)
    self.pub.bind(self.cfg.endpoint_for("modelV2"))
    self.pub.monitor(f"inproc://offloadd-test-mon-{os.getpid()}", zmq.EVENT_ACCEPTED)
    self.mon = self.pub.get_monitor_socket()

    # a second PUB for the SP return service (own port) — the §7 pairing tests need it.
    self.sp_pub = self.ctx.socket(zmq.PUB)
    self.sp_pub.setsockopt(zmq.LINGER, 0)
    self.sp_pub.bind(self.cfg.endpoint_for("modelDataV2SP"))
    self.sp_pub.monitor(f"inproc://offloadd-test-sp-mon-{os.getpid()}", zmq.EVENT_ACCEPTED)
    self.sp_mon = self.sp_pub.get_monitor_socket()

    self.daemon = od.Offloadd(self.cfg)
    self.daemon.open()

    self.cam_pub = PubMaster(["narrowRoadCameraState"]) if _HAVE_CEREAL else None
    # Forward verification uses the RAW msgq transport, because a cereal SubMaster cannot yet
    # construct on the shadow name: offloadModelV2 is registered in services.py but has no member
    # in log.capnp's Event.union, so `new_message("offloadModelV2")` raises KjException (RESULTS.md
    # "Jetlink borrows"). msgq itself carries the topic fine with a matching segment size.
    self.shadow_name = self.cfg.shadow_map["modelV2"]
    self.shadow_sub = (msgq.sub_sock(self.shadow_name, addr="127.0.0.1", timeout=200,
                                    segment_size=SERVICE_LIST[self.shadow_name].queue_size)
                       if _HAVE_CEREAL else None)
    self.sp_shadow_name = self.cfg.shadow_map["modelDataV2SP"]
    self.sp_shadow_sub = (msgq.sub_sock(self.sp_shadow_name, addr="127.0.0.1", timeout=200,
                                       segment_size=SERVICE_LIST[self.sp_shadow_name].queue_size)
                          if _HAVE_CEREAL else None)

    self._wait_zmq_accepted()
    self._wait_accepted(self.sp_mon, "inproc://offloadd-test-sp-accepted")
    # SUBSCRIBE propagates just after ACCEPTED; give the link a moment so the first
    # modelV2 send is not lost to the PUB/SUB envelope race.
    time.sleep(0.25)

  # -- ZMQ connection barrier -------------------------------------------------
  def _wait_zmq_accepted(self, timeout=5.0) -> None:
    self._wait_accepted(self.mon, "offloadd-test-mon", timeout)

  def _wait_accepted(self, mon, tag: str, timeout=5.0) -> None:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
      if mon.poll(100):
        try:
          frames = mon.recv_multipart()
        except zmq.ZMQError:
          continue
        evt = struct.unpack("<H", frames[0][:2])[0]
        if evt & zmq.EVENT_ACCEPTED:
          return
    raise AssertionError(f"ZMQ subscription never accepted ({tag}; slow-joiner barrier timed out)")

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

  def send_sp(self, turn: int = 1, left: bool = False, right: bool = False) -> bytes:
    """Send one modelDataV2SP on its return port; return the wire bytes it carried."""
    raw = _modeldatav2sp(turn, left, right)
    self.sp_pub.send(raw)
    return raw

  def recv_sp_shadow(self, timeout_s: float = 2.0):
    """Raw msgq receive on the SP shadow topic: (logMonoTime, laneTurnDirection), or None."""
    d = self.recv_sp_raw(timeout_s)
    if d is None:
      return None
    with _load(d) as r:
      assert r.which() == "modelDataV2SP"   # payload unchanged: still a modelDataV2SP
      return r.logMonoTime, int(r.modelDataV2SP.laneTurnDirection.raw)

  def recv_sp_raw(self, timeout_s: float = 2.0):
    """Raw msgq bytes from the SP shadow topic (or None) — for byte-identity checks."""
    if self.sp_shadow_sub is None:
      return None
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
      d = self.sp_shadow_sub.receive()
      if d:
        return bytes(d)
    return None

  def recv_shadow(self, timeout_s: float = 3.0):
    """Raw msgq receive on the shadow topic: (logMonoTime, frameId, frameAge), or None."""
    if self.shadow_sub is None:
      return None
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
      d = self.shadow_sub.receive()
      if d:
        with _load(bytes(d)) as r:
          assert r.which() == "modelV2"          # payload unchanged: it is still a modelV2
          return r.logMonoTime, r.modelV2.frameId, r.modelV2.frameAge
    return None

  def close(self):
    self.daemon.close()
    try:
      self.mon.close(0)
    except Exception:
      pass
    try:
      self.sp_mon.close(0)
    except Exception:
      pass
    self.pub.close(0)
    self.sp_pub.close(0)
    self.ctx.term()


@needs_cereal
def test_fresh_message_is_restamped_and_forwarded():
  fx = Fixture()
  try:
    n, _sof = fx.learn_camera(1)
    fx.send_modelv2(1, n + 10_000_000)
    publish_now = n + 20_000_000          # 20 ms later -> well within 300 ms
    assert fx.step_until(lambda: fx.daemon.forwarded == 1, publish_now)
    assert fx.daemon.forwarded == 1
    assert fx.daemon.dropped_stale == 0
    # forwarded under the SHADOW name, not the real modelV2
    got = fx.recv_shadow()
    assert got is not None
    mt, fid, age = got
    assert mt == publish_now                # header re-stamped to device now
    assert fid == 1 and age == 3            # payload intact
  finally:
    fx.close()


@needs_cereal
def test_stale_message_is_dropped_not_forwarded():
  fx = Fixture(stale_ms=300)
  try:
    # camera frame advertised as 900 ms old: > stale (300) but < SOF TTL (1000) so it survives
    n, _sof = fx.learn_camera(1, sof_fn=lambda now: now - 900_000_000)
    fx.send_modelv2(1, n)
    assert fx.step_until(lambda: fx.daemon.dropped_stale == 1, n)
    assert fx.daemon.forwarded == 0
    assert fx.recv_shadow(timeout_s=0.5) is None     # nothing reached the shadow topic
  finally:
    fx.close()


@needs_cereal
def test_nonfinite_message_is_dropped_not_forwarded():
  """A remote message carrying a NaN/Inf is never forwarded (Jetlink's all-finite rule)."""
  fx = Fixture()
  try:
    n, _sof = fx.learn_camera(1)
    raw = _modelv2(1, n + 2_000_000)
    with _load(raw) as r:
      b = r.as_builder()
      b.modelV2.modelExecutionTime = float("nan")   # a non-finite float in the payload
      bad = b.to_bytes()
    fx.pub.send(bad)
    assert fx.step_until(lambda: fx.daemon.dropped_nonfinite == 1, n)
    assert fx.daemon.forwarded == 0
  finally:
    fx.close()


@needs_cereal
def test_no_sof_is_dropped():
  fx = Fixture()
  try:
    n = time.monotonic_ns()
    fx.send_modelv2(999, n)                   # frameId 999 was never announced
    assert fx.step_until(lambda: fx.daemon.dropped_no_sof == 1, n)
    assert fx.daemon.forwarded == 0
  finally:
    fx.close()


# ---------------------------------------------------------------------------
# INTERFACES §7 'SP pairing rule': modelDataV2SP is forwarded only when paired
# within OFFLOAD_SP_PAIR_MS of an aged-able message that passed the freshness gate
# ---------------------------------------------------------------------------

@needs_cereal
def test_sp_alone_without_partner_is_dropped_unpaired():
  """SP with no aged-able msg within the window: dropped_unpaired, and its shadow topic stays silent."""
  fx = Fixture()
  try:
    n, _sof = fx.learn_camera(1)
    fx.send_sp(turn=2)
    assert fx.step_until(lambda: fx.daemon.pending_sp is not None, n)      # SP arrives at n
    expire_now = n + 30_000_000                        # past the 25 ms pairing window
    assert fx.step_until(lambda: fx.daemon.dropped_unpaired >= 1, expire_now)
    assert fx.daemon.forwarded == 0
    assert fx.daemon.pending_sp is None
    # never forwarded blind: the SP shadow topic must be silent
    assert fx.recv_sp_raw(timeout_s=0.5) is None
  finally:
    fx.close()


@needs_cereal
def test_sp_before_modelv2_within_window_pairs_both():
  """SP arrives just before a fresh modelV2 (within the window): both are forwarded."""
  fx = Fixture()
  try:
    n, _sof = fx.learn_camera(1)
    publish_now = n + 20_000_000
    sp_wire = fx.send_sp(turn=2)
    assert fx.step_until(lambda: fx.daemon.pending_sp is not None, n)
    fx.send_modelv2(1, n + 10_000_000)
    assert fx.step_until(lambda: fx.daemon.forwarded == 2, publish_now)
    assert fx.daemon.dropped_unpaired == 0
    got = fx.recv_sp_raw()
    assert got is not None
    with _load(got) as r:
      assert r.which() == "modelDataV2SP"
      assert int(r.modelDataV2SP.laneTurnDirection.raw) == 2
      assert r.logMonoTime == publish_now            # header re-stamped
    assert _fields_without_header(got) == _fields_without_header(sp_wire)   # payload byte-identical
  finally:
    fx.close()


@pytest.mark.parametrize("sp_first", [True, False])
@needs_cereal
def test_sp_and_modelv2_same_drain_either_send_order_pairs(sp_first):
  """(a) Order is irrelevant: SP + fresh modelV2 on the wire before ONE step pair regardless of send order.

  This is the production shape (the Mac ships the four outputs of a frame back-to-back and offloadd
  drains them in one pass), so both messages share the step's receipt stamp.
  """
  fx = Fixture()
  try:
    n, _sof = fx.learn_camera(1)
    now = n + 15_000_000
    if sp_first:
      sp_wire = fx.send_sp(turn=2)
      fx.send_modelv2(1, n + 5_000_000)
    else:
      fx.send_modelv2(1, n + 5_000_000)
      sp_wire = fx.send_sp(turn=2)
    assert fx.step_until(lambda: fx.daemon.forwarded == 2, now)
    assert fx.daemon.dropped_unpaired == 0 and fx.daemon.pending_sp is None
    assert fx.recv_shadow() is not None
    got = fx.recv_sp_raw()
    assert got is not None
    assert _normalize_header(got, 1) == _normalize_header(sp_wire, 1)
  finally:
    fx.close()


@needs_cereal
def test_sp_after_modelv2_within_window_pairs_both():
  """SP arrives just after a fresh modelV2 (within the window): forwarded immediately on arrival."""
  fx = Fixture()
  try:
    n, _sof = fx.learn_camera(1)
    fx.send_modelv2(1, n + 10_000_000)
    assert fx.step_until(lambda: fx.daemon.forwarded == 1, n + 20_000_000)
    assert fx.daemon.last_forwarded_msg_ns == n + 20_000_000   # the aged anchor
    sp_wire = fx.send_sp(turn=1)
    after = n + 30_000_000                             # 10 ms after the aged msg: within 25 ms
    assert fx.step_until(lambda: fx.daemon.forwarded == 2, after)
    assert fx.daemon.pending_sp is None
    got = fx.recv_sp_raw()
    assert got is not None
    assert _fields_without_header(got) == _fields_without_header(sp_wire)
  finally:
    fx.close()


@needs_cereal
def test_stale_modelv2_with_nearby_sp_forwards_neither():
  """A stale aged message is dropped, so a nearby SP gets no partner -> neither is forwarded."""
  fx = Fixture(stale_ms=300)
  try:
    n, _sof = fx.learn_camera(1)
    stale_now = n + 400_000_000                        # SOF is > 300 ms old -> modelV2 is stale
    fx.send_sp(turn=1)
    assert fx.step_until(lambda: fx.daemon.pending_sp is not None, stale_now)   # SP buffered at stale_now
    fx.send_modelv2(1, n)                              # the (stale) partner arrives in the same instant
    assert fx.step_until(lambda: fx.daemon.dropped_stale >= 1, stale_now)
    assert fx.daemon.forwarded == 0                    # the stale modelV2 did NOT become an anchor
    assert fx.daemon.pending_sp is not None            # so the SP is still waiting for a fresh partner ...
    # ... and once the window elapses with none, it is dropped as unpaired
    expire_now = stale_now + 30_000_000
    assert fx.step_until(lambda: fx.daemon.dropped_unpaired >= 1, expire_now)
    assert fx.daemon.forwarded == 0
    assert fx.recv_sp_raw(timeout_s=0.5) is None       # SP shadow silent: no fresh partner
    assert fx.recv_shadow(timeout_s=0.5) is None       # modelV2 shadow silent: it was stale
  finally:
    fx.close()


@needs_cereal
def test_nonfinite_modelv2_with_nearby_sp_forwards_neither():
  """A non-finite aged message is dropped, so it cannot anchor a pair either: its nearby SP is never forwarded."""
  fx = Fixture()
  try:
    n, _sof = fx.learn_camera(1)
    with _load(_modelv2(1, n + 2_000_000)) as r:
      b = r.as_builder()
      b.modelV2.modelExecutionTime = float("nan")
      bad = b.to_bytes()
    fx.send_sp(turn=1)
    assert fx.step_until(lambda: fx.daemon.pending_sp is not None, n)   # SP buffered at n
    fx.pub.send(bad)
    assert fx.step_until(lambda: fx.daemon.dropped_nonfinite == 1, n)
    assert fx.daemon.pending_sp is not None            # the SP waits for a partner that never comes ...
    assert fx.step_until(lambda: fx.daemon.dropped_unpaired == 1, n + 30_000_000)   # ... then expires
    assert fx.daemon.forwarded == 0
    assert fx.recv_sp_raw(timeout_s=0.5) is None
    assert fx.recv_shadow(timeout_s=0.5) is None
  finally:
    fx.close()


@needs_cereal
def test_no_fresh_partner_never_forwards_sp():
  """The safety property: an SP that never sees a fresh partner is never forwarded, only counted."""
  fx = Fixture()
  try:
    n, _sof = fx.learn_camera(1)
    for p in range(3):                                 # repeated lone SPs, spaced past the window
      fx.send_sp(turn=1)
      t = n + p * 60_000_000
      fx.step_until(lambda: True, t)
    assert fx.daemon.forwarded == 0
    assert fx.daemon.dropped_unpaired >= 1
    assert fx.daemon.dropped_no_sof == 0               # SP no longer rides the no-age-source counter
    assert fx.recv_sp_raw(timeout_s=0.5) is None
  finally:
    fx.close()


def _normalize_header(data: bytes, hdr: int) -> bytes:
  """Re-serialize ``data`` with the Event header ``logMonoTime`` forced to ``hdr`` (payload untouched)."""
  with _load(data) as r:
    b = r.as_builder()
    b.logMonoTime = hdr
    return b.to_bytes()


@needs_cereal
def test_full_pair_frameid_preserved_and_sp_payload_byte_identical():
  """(e) The paired set arrives intact: aged msg keeps its frameId; SP is byte-identical but for the header."""
  fx = Fixture()
  try:
    n, _sof = fx.learn_camera(7)
    sp_wire = fx.send_sp(turn=2, left=True, right=False)
    assert fx.step_until(lambda: fx.daemon.pending_sp is not None, n)
    fx.send_modelv2(7, n + 5_000_000)
    now = n + 15_000_000
    assert fx.step_until(lambda: fx.daemon.forwarded == 2, now)
    assert fx.daemon.dropped_unpaired == 0 and fx.daemon.dropped_no_sof == 0

    mv2 = fx.recv_shadow()
    assert mv2 is not None
    mt, fid, age = mv2
    assert fid == 7 and age == 3                      # frameId + payload preserved on the aged msg
    assert mt == now                                  # header = device now

    sp_got = fx.recv_sp_raw()
    assert sp_got is not None
    assert sp_got != sp_wire                          # the header did change...
    assert _normalize_header(sp_got, 111) == _normalize_header(sp_wire, 111)   # ...and ONLY the header
    with _load(sp_got) as r:
      assert r.which() == "modelDataV2SP" and r.logMonoTime == now
  finally:
    fx.close()


@pytest.mark.parametrize("gap_ms,paired", [(25, True), (26, False)])
@needs_cereal
def test_sp_after_partner_window_boundary(gap_ms, paired):
  """The pairing window is inclusive at OFFLOAD_SP_PAIR_MS (25 ms): +25 pairs, +26 does not."""
  fx = Fixture()
  try:
    n, _sof = fx.learn_camera(1)
    fx.send_modelv2(1, n + 5_000_000)
    anchor = n + 10_000_000
    assert fx.step_until(lambda: fx.daemon.forwarded == 1, anchor)
    fx.send_sp(turn=1)
    sp_t = anchor + gap_ms * 1_000_000
    if paired:
      assert fx.step_until(lambda: fx.daemon.forwarded == 2, sp_t)
      assert fx.recv_sp_raw() is not None
    else:
      assert fx.step_until(lambda: fx.daemon.pending_sp is not None, sp_t)
      assert fx.step_until(lambda: fx.daemon.dropped_unpaired == 1, sp_t + 30_000_000)
      assert fx.daemon.forwarded == 1
      assert fx.recv_sp_raw(timeout_s=0.5) is None
  finally:
    fx.close()


@needs_cereal
def test_sp_burst_single_slot_only_newest_pairs():
  """Two SPs before one fresh partner: the older is superseded (counted unpaired); only the newest is forwarded."""
  fx = Fixture()
  try:
    n, _sof = fx.learn_camera(1)
    fx.send_sp(turn=1)
    fx.send_sp(turn=2)
    assert fx.step_until(lambda: fx.daemon.dropped_unpaired == 1, n)   # SP#1 superseded by SP#2
    assert fx.daemon.pending_sp is not None
    fx.send_modelv2(1, n + 5_000_000)
    assert fx.step_until(lambda: fx.daemon.forwarded == 2, n + 10_000_000)
    got = fx.recv_sp_shadow()
    assert got is not None and got[1] == 2             # the NEWEST SP (turnRight) was the one forwarded
    assert fx.recv_sp_raw(timeout_s=0.3) is None       # and nothing else
    assert fx.daemon.dropped_unpaired == 1
  finally:
    fx.close()


@needs_cereal
def test_one_anchor_pairs_at_most_one_sp():
  """Pairing is one-to-one: a second SP inside the window of an already-used anchor is NOT forwarded."""
  fx = Fixture()
  try:
    n, _sof = fx.learn_camera(1)
    fx.send_modelv2(1, n + 5_000_000)
    anchor = n + 10_000_000
    assert fx.step_until(lambda: fx.daemon.forwarded == 1, anchor)
    fx.send_sp(turn=1)
    assert fx.step_until(lambda: fx.daemon.forwarded == 2, anchor + 5_000_000)   # SP#1 uses the anchor
    fx.send_sp(turn=2)                                                           # 10 ms after it: in-window
    assert fx.step_until(lambda: fx.daemon.pending_sp is not None, anchor + 10_000_000)
    assert fx.daemon.forwarded == 2                                              # ...but the anchor is spent
    assert fx.step_until(lambda: fx.daemon.dropped_unpaired == 1, anchor + 40_000_000)
    assert fx.daemon.forwarded == 2
    got = fx.recv_sp_shadow()
    assert got is not None and got[1] == 1                                       # only SP#1 reached the topic
    assert fx.recv_sp_raw(timeout_s=0.3) is None
  finally:
    fx.close()


def _step_real(fx, pred, timeout=5.0) -> bool:
  """Drive the daemon with NO injected clock (the production path) until ``pred()`` holds."""
  end = time.monotonic() + timeout
  while time.monotonic() < end:
    fx.daemon.step()
    if pred():
      return True
    time.sleep(0.02)
  return False


@needs_cereal
def test_long_camera_block_in_later_step_does_not_overpair_sp():
  """Receipt stamps are TRUE drain times, not the step-top clock (production path, fake monotonic source).

  The step-top stamp predates the blocking camera ``update``. If it were used, an SP drained in a step whose
  camera poll blocked 60 ms would still look simultaneous with a partner forwarded in the previous step
  (both stamped at their step tops) and be forwarded although the two are really 60 ms apart — the unsafe
  direction, invisible downstream. With true stamps it must be refused and counted.
  """
  fx = Fixture()
  try:
    n, _sof = fx.learn_camera(1)
    clk = {"t": n}
    block_ns = {"v": 0}
    fx.daemon._clock = lambda: clk["t"]
    real_update = fx.daemon.cereal.update

    def update(timeout_ms):                # the camera poll "blocks": the clock moves, then the poll is non-blocking
      clk["t"] += block_ns["v"]
      real_update(0)

    fx.daemon.cereal.update = update

    fx.send_modelv2(1, n)
    assert _step_real(fx, lambda: fx.daemon.forwarded == 1)          # partner forwarded, anchor == n
    assert fx.daemon.last_forwarded_msg_ns == n
    fx.send_sp(turn=1)
    block_ns["v"] = 60_000_000                                       # the next steps block 60 ms each
    assert _step_real(fx, lambda: fx.daemon.dropped_unpaired == 1)   # SP drained 60 ms after the partner: refused
    assert fx.daemon.forwarded == 1 and fx.daemon.sp_paired == 0
    assert fx.recv_sp_raw(timeout_s=0.5) is None
  finally:
    fx.close()


@needs_cereal
def test_sp_straddling_two_steps_still_pairs_on_true_stamps():
  """The partner is forwarded in one step and the SP lands in the next, 3 ms later in real time: it pairs."""
  fx = Fixture()
  try:
    n, _sof = fx.learn_camera(1)
    clk = {"t": n}
    fx.daemon._clock = lambda: clk["t"]
    real_update = fx.daemon.cereal.update
    block_ns = {"v": 0}

    def update(timeout_ms):
      clk["t"] += block_ns["v"]
      real_update(0)

    fx.daemon.cereal.update = update
    fx.send_modelv2(1, n)
    assert _step_real(fx, lambda: fx.daemon.forwarded == 1)
    fx.send_sp(turn=2)
    block_ns["v"] = 3_000_000                                        # next step "blocks" only 3 ms
    assert _step_real(fx, lambda: fx.daemon.forwarded == 2)
    assert fx.daemon.sp_paired == 1 and fx.daemon.dropped_unpaired == 0
    assert fx.recv_sp_raw() is not None
  finally:
    fx.close()


@needs_cereal
def test_camera_poll_shortens_only_while_sp_pairing_in_flight():
  """The camera poll that paces the loop is shortened ONLY inside an SP pairing (pending SP, or a fresh unclaimed anchor)."""
  fx = Fixture()                                    # recv_timeout_ms == 20
  try:
    n, _sof = fx.learn_camera(1)
    fx.daemon._injected_ns = n
    assert fx.daemon._camera_poll_ms() == 20                          # idle: normal pacing
    fx.daemon.pending_sp = (b"x", n)
    assert fx.daemon._camera_poll_ms() == od.SP_PAIR_POLL_MS          # SP waiting for its partner
    fx.daemon.pending_sp = None
    fx.daemon.last_forwarded_msg_ns = n
    assert fx.daemon._camera_poll_ms() == od.SP_PAIR_POLL_MS          # partner just forwarded, SP not seen yet
    fx.daemon._injected_ns = n + 30_000_000
    assert fx.daemon._camera_poll_ms() == 20                          # window elapsed: back to normal pacing
  finally:
    fx.close()


@needs_cereal
def test_wrong_service_on_sp_port_is_not_buffered():
  """A non-SP payload on the SP port is an error: never buffered, never forwarded."""
  fx = Fixture()
  try:
    n, _sof = fx.learn_camera(1)
    fx.sp_pub.send(_modelv2(1, n))                     # a modelV2 on the modelDataV2SP port
    assert fx.step_until(lambda: fx.daemon.dropped_error >= 1, n)
    assert fx.daemon.pending_sp is None
    assert fx.daemon.forwarded == 0
  finally:
    fx.close()


def test_sp_pair_ms_default_and_env(monkeypatch):
  monkeypatch.delenv("OFFLOAD_SP_PAIR_MS", raising=False)
  assert od.Config().sp_pair_ms == 25
  monkeypatch.setenv("OFFLOAD_SP_PAIR_MS", "40")
  assert od.Config().sp_pair_ms == 40
  monkeypatch.setenv("OFFLOAD_SP_PAIR_MS", "junk")
  assert od.Config().sp_pair_ms == 25


@needs_cereal
def test_gap_watchdog_fires_then_clears():
  fx = Fixture(gap_ms=100, stale_ms=300)
  try:
    n, _sof = fx.learn_camera(1)
    # establish a valid forward first (this is the gap baseline)
    fx.send_modelv2(1, n + 1_000_000)
    assert fx.step_until(lambda: fx.daemon.forwarded == 1, n + 20_000_000)
    assert not fx.daemon.gap_open

    # nothing valid for > gap_ms (100 ms), age still < stale (300 ms) -> gap opens, no forward
    fx.daemon.step(now_ns=n + 200_000_000)
    assert fx.daemon.gap_open

    # a fresh frame for the same camera frame resumes forward and clears the gap
    fx.send_modelv2(1, n + 200_000_001)
    assert fx.step_until(lambda: fx.daemon.forwarded == 2, n + 200_000_000)
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
  assert {"published", "dropped_stale", "dropped_no_sof", "dropped_unpaired", "dropped_error",
              "zmq_msgs", "gap_open", "uptime_s", "sp_paired"}.issubset(st.keys())
  assert st["published"] == 0
  assert st["dropped_unpaired"] == 0 and st["sp_paired"] == 0


if __name__ == "__main__":
  raise SystemExit(pytest.main([os.path.abspath(__file__), "-v"]))
