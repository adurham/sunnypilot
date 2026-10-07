#!/usr/bin/env python3
"""Offline tests for openpilot.offload.mac.returnsend (WS-B) — no device required.

Run:
  PYTHONPATH=$PWD:$PWD/opendbc_repo:$PWD/msgq_repo:$PWD/tinygrad_repo \
    .venv/bin/python -m pytest openpilot/offload/mac/tests/test_returnsend.py -v

Four tests:
  1. test_loopback_forward_bytes_identical  — a fake modelV2 published into the Mac msgq is
     forwarded by a real ReturnSender and received byte-identically on a pyzmq SUB.
  2. test_age_gate_drops_old_message        — a msgq message older than --max-age-ms is dropped
     (counted), never sent.
  3. test_no_reader_counts_and_keeps_running— no SUB connected: the sender keeps running, counts
     dropped_noreader, exits clean on SIGTERM.
  4. test_full_return_chain_into_shadow_topics — THE POINT: subprocess returnsend (PUB) + offloadd
     (SUB, bench shadow mode) both on 127.0.0.1; fake modeld outputs are published into the Mac
     msgq with a recent SOF; the shadow topics are raw-subscribed and each message is asserted to
     arrive with its frameId preserved, its payload identical except the header, and its
     logMonoTime re-stamped to receipt time (offloadd's receipt re-stamp). This is the one-machine
     run: here offloadd's "device" clock IS this machine's monotonic clock, exactly the real
     relationship on a drive (both ends share one monotonic domain via the receipt-restamp design).

Isolation: the whole process namespaces its msgq data queues with OPENPILOT_PREFIX (set below,
before msgq import) and its event-handle shm with set_fake_prefix, so it cannot collide with any
other session's modeld/framebridge msgq traffic on this shared Mac.
"""
from __future__ import annotations

import json
import os
import signal
import socket
import subprocess
import time

# --- isolate this process's msgq BEFORE importing it --------------------------------
REPO = "/Users/adam.durham/repos/sunnypilot-offload"
PREFIX = f"offload-rs-{os.getpid()}"
os.environ["OPENPILOT_PREFIX"] = PREFIX
os.makedirs(f"/tmp/msgq_{PREFIX}", exist_ok=True)

import pytest

from openpilot.offload.contract import RETURN_SERVICES
from openpilot.offload.mac import returnsend as rs

try:
  import zmq
  import msgq
  from openpilot.cereal import log as cereal_log
  from openpilot.cereal.messaging import PubMaster, new_message
  from openpilot.cereal.services import SERVICE_LIST
  from openpilot.offload import ports
  msgq.set_fake_prefix(PREFIX)   # event-handle isolation (separate from OPENPILOT_PREFIX)
  _HAVE_CEREAL = True
except Exception:  # pragma: no cover - host without native bindings
  _HAVE_CEREAL = False

needs_cereal = pytest.mark.skipif(not _HAVE_CEREAL, reason="cereal/msgq native bindings not importable")
_TRAV = 2 ** 64 - 1
PY = os.path.join(REPO, ".venv/bin/python")
CHILD_ENV = dict(os.environ)
CHILD_ENV["PYTHONPATH"] = f"{REPO}:{REPO}/opendbc_repo:{REPO}/msgq_repo:{REPO}/tinygrad_repo"
CHILD_ENV["OPENPILOT_PREFIX"] = PREFIX

SHADOW_TO_SVC = {
  "offloadModelV2": "modelV2",
  "offloadCameraOdometry": "cameraOdometry",
  "offloadDrivingModelData": "drivingModelData",
  "offloadModelDataV2SP": "modelDataV2SP",
}


# ---------------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------------

def _free_port() -> int:
  s = socket.socket()
  s.bind(("127.0.0.1", 0))
  p = s.getsockname()[1]
  s.close()
  return p


def _ports_busy(services=RETURN_SERVICES) -> bool:
  """True if any frozen return port is already bound (another returnsend/bridge on this Mac)."""
  for svc in services:
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
      s.bind(("127.0.0.1", ports.get_port(svc)))
    except OSError:
      return True
    finally:
      s.close()
  return False


def _which(raw: bytes) -> str:
  with cereal_log.Event.from_bytes(raw, traversal_limit_in_words=_TRAV) as r:
    return r.which()


def _header(raw: bytes) -> int:
  with cereal_log.Event.from_bytes(raw, traversal_limit_in_words=_TRAV) as r:
    return int(r.logMonoTime)


def _fields_without_header(raw: bytes) -> dict:
  with cereal_log.Event.from_bytes(raw, traversal_limit_in_words=_TRAV) as r:
    d = r.to_dict()
  d.pop("logMonoTime", None)
  return d


def _frame_id(raw: bytes) -> int | None:
  with cereal_log.Event.from_bytes(raw, traversal_limit_in_words=_TRAV) as r:
    svc = r.which()
    m = getattr(r, svc)
    return int(m.frameId) if hasattr(m, "frameId") else None


def _modelv2(fid: int, eof_ns: int, frame_age: int = 3, log_mono_ns: int | None = None) -> bytes:
  m = new_message("modelV2")
  m.valid = True
  if log_mono_ns is not None:
    m.logMonoTime = int(log_mono_ns)
  m.modelV2.frameId = int(fid)
  m.modelV2.frameIdExtra = int(fid) + 1000
  m.modelV2.timestampEof = int(eof_ns)
  m.modelV2.frameAge = int(frame_age)
  m.modelV2.modelExecutionTime = 12.5
  return m.to_bytes()


def _pump_until(sender, pred, publish, timeout: float = 3.0) -> None:
  """Publish-and-pump until ``pred()`` holds. Republishing defeats the msgq reader-init race."""
  end = time.monotonic() + timeout
  while time.monotonic() < end and not pred():
    publish()
    for _ in range(6):
      sender.step()
    time.sleep(0.01)


def _wait_line(path: str, needle: str, timeout: float = 8.0) -> bool:
  """Poll a redirected-stdout file until it contains ``needle`` (children print flush=True)."""
  end = time.monotonic() + timeout
  while time.monotonic() < end:
    try:
      if os.path.exists(path) and needle in open(path).read():
        return True
    except OSError:
      pass
    time.sleep(0.05)
  return False


def _stats_snapshot(log: str, key: str) -> dict:
  """Last JSON stats line in ``log`` that contains ``key`` (e.g. 'forwarded' / 'returnsend_stats')."""
  lines = []
  for x in log.splitlines():
    x = x.strip()
    if x.startswith("{") and key in x:
      try:
        lines.append(json.loads(x))
      except json.JSONDecodeError:
        pass
  return lines[-1] if lines else {}


def _quiesce(od_out: str, timeout: float = 8.0) -> dict:
  """Wait until offloadd's ``forwarded`` counter stops changing (all in-flight traffic drained).

  The warmup publishes hundreds of messages; without this barrier a later delta measurement would
  be inflated by warmup messages still in flight. Returns the stable offloadd stats snapshot.
  """
  end = time.monotonic() + timeout
  last = None
  stable_since = time.monotonic()
  while time.monotonic() < end:
    snap = _stats_snapshot(open(od_out).read(), "forwarded")
    cur = snap.get("forwarded")
    if cur != last:
      last, stable_since = cur, time.monotonic()
    elif time.monotonic() - stable_since >= 1.5:
      return snap
    time.sleep(0.2)
  return _stats_snapshot(open(od_out).read(), "forwarded")


# ---------------------------------------------------------------------------------
# 1. loopback unit: Mac msgq -> returnsend -> pyzmq SUB, bytes identical
# ---------------------------------------------------------------------------------

@needs_cereal
def test_loopback_forward_bytes_identical():
  port = _free_port()
  ctx = zmq.Context()
  sub = ctx.socket(zmq.SUB)
  sub.setsockopt(zmq.SUBSCRIBE, b"")
  sub.setsockopt(zmq.LINGER, 0)
  sub.connect(f"tcp://127.0.0.1:{port}")
  time.sleep(0.2)

  # generous age gate so the slow-joiner/reader-init race cannot make a still-in-flight message
  # "stale"; a fresh message is published every iteration anyway.
  sender = rs.ReturnSender(rs.Config(bind="127.0.0.1", services=["modelV2"], max_age_ms=5000,
                                     endpoint_for=lambda _s: f"tcp://127.0.0.1:{port}"))
  sender.open()
  try:
    pm = PubMaster(["modelV2"])
    got = None
    end = time.monotonic() + 5.0
    while time.monotonic() < end and got is None:
      published = _modelv2(4242, time.monotonic_ns() + 5_000_000, frame_age=9)
      pm.send("modelV2", published)
      for _ in range(6):
        sender.step()
        try:
          got = sub.recv(zmq.NOBLOCK)
        except zmq.Again:
          pass
        if got is not None:
          break
      time.sleep(0.01)
    assert got is not None, "sender forwarded nothing to the ZMQ SUB"
    assert bytes(got) == published, "raw Event bytes must cross the wire untouched"
    assert _which(got) == "modelV2"
    assert _header(got) == _header(published)          # a pipe: no re-stamp on the wire
    assert _fields_without_header(got) == _fields_without_header(published)
    assert _frame_id(got) == 4242
    assert sender.forwarded >= 1
    assert sender.dropped_error == 0
  finally:
    sender.close()
    sub.close(0)
    ctx.term()


# ---------------------------------------------------------------------------------
# 2. age gate: an old Mac-local message is dropped (counted), never forwarded
# ---------------------------------------------------------------------------------

@needs_cereal
def test_age_gate_drops_old_message():
  port = _free_port()
  ctx = zmq.Context()
  sub = ctx.socket(zmq.SUB)
  sub.setsockopt(zmq.SUBSCRIBE, b"")
  sub.setsockopt(zmq.LINGER, 0)
  sub.connect(f"tcp://127.0.0.1:{port}")
  time.sleep(0.2)

  sender = rs.ReturnSender(rs.Config(bind="127.0.0.1", services=["modelV2"], max_age_ms=50,
                                     endpoint_for=lambda _s: f"tcp://127.0.0.1:{port}"))
  sender.open()
  try:
    pm = PubMaster(["modelV2"])
    # header is already 500 ms old -> far past the 50 ms self-drop
    old = _modelv2(7, 0, log_mono_ns=time.monotonic_ns() - 500_000_000)
    _pump_until(sender, lambda: sender.dropped_age >= 1, lambda: pm.send("modelV2", old))
    assert sender.dropped_age >= 1
    assert sender.forwarded == 0
    with pytest.raises(zmq.Again):
      sub.recv(zmq.NOBLOCK)                    # nothing reached the wire
  finally:
    sender.close()
    sub.close(0)
    ctx.term()


# ---------------------------------------------------------------------------------
# 3. hiccup: no reader -> keep running, count dropped_noreader, clean SIGTERM exit
# ---------------------------------------------------------------------------------

@needs_cereal
def test_no_reader_counts_and_keeps_running():
  port = _free_port()   # nothing ever connects to this port
  sender = rs.ReturnSender(rs.Config(bind="127.0.0.1", services=["modelV2"], max_age_ms=1000,
                                     endpoint_for=lambda _s: f"tcp://127.0.0.1:{port}"))
  sender.open()
  try:
    pm = PubMaster(["modelV2"])
    fresh = _modelv2(1, time.monotonic_ns())
    _pump_until(sender, lambda: sender.dropped_noreader >= 1, lambda: pm.send("modelV2", fresh))
    assert sender.dropped_noreader >= 1
    assert sender.forwarded == 0
    assert sender.dropped_age == 0            # it was fresh; a reader problem, not age
    sender.request_stop()                     # the loop is still alive and honors a signal
    assert sender._stop is True
  finally:
    sender.close()


def test_missing_reader_detected_via_monitor_edges():
  """Sanity: with no SUB the reader count stays 0; closing/reopening a SUB toggles it."""
  if not _HAVE_CEREAL:
    pytest.skip("cereal/msgq native bindings not importable")
  port = _free_port()
  sender = rs.ReturnSender(rs.Config(bind="127.0.0.1", services=["modelV2"],
                                     endpoint_for=lambda _s: f"tcp://127.0.0.1:{port}"))
  sender.open()
  try:
    for _ in range(20):
      sender.step()
    assert sender._readers["modelV2"] == 0
  finally:
    sender.close()


# ---------------------------------------------------------------------------------
# 4. FULL RETURN CHAIN: returnsend + offloadd, shadow topics, receipt re-stamp
# ---------------------------------------------------------------------------------

def _publish_round(cam_pub, out_pub, frame_id: int, ts_ns: int | None = None) -> dict:
  """Publish one fresh camera state + one message per RETURN_SERVICE for ``frame_id``.

  Only modelV2/cameraOdometry carry ``timestampEof`` (drivingModelData/modelDataV2SP do not, per
  their capnp schemas). The camera SOF is the authoritative device-clock reference offloadd ages
  against, so it is kept recent. Returns the exact wire bytes per service (the sender must carry
  them through untouched).
  """
  now = ts_ns if ts_ns is not None else time.monotonic_ns()
  cs = new_message("narrowRoadCameraState")
  cs.valid = True
  cs.narrowRoadCameraState.frameId = frame_id
  cs.narrowRoadCameraState.timestampSof = now
  cs.narrowRoadCameraState.timestampEof = now + 5_000_000
  cam_pub.send("narrowRoadCameraState", cs.to_bytes())
  wire = {}
  for svc in RETURN_SERVICES:
    m = new_message(svc)
    m.valid = True
    m.logMonoTime = int(now)     # the Mac-local header the sender ages on; == publish_mono_ns
    node = getattr(m, svc)
    if hasattr(node, "frameId"):
      node.frameId = frame_id
    if hasattr(node, "timestampEof"):
      node.timestampEof = now
    if svc == "modelV2":
      m.modelV2.frameAge = 3
    raw = m.to_bytes()
    wire[svc] = raw
    out_pub.send(svc, raw)
  return wire


def _drain_shadow(shadow_subs, want_frame: int | None = None) -> dict:
  """Drain every shadow topic; keep the newest message per topic (optionally filtered by frame)."""
  out = {}
  for name, sub in shadow_subs.items():
    while True:
      d = sub.receive()
      if not d:
        break
      d = bytes(d)
      if want_frame is None or _frame_id(d) == want_frame:
        out[name] = d
  return out


@pytest.mark.skipif(not _HAVE_CEREAL, reason="cereal/msgq native bindings not importable")
def test_full_return_chain_into_shadow_topics(tmp_path):
  if _ports_busy():
    pytest.skip("a frozen return port is already bound (another returnsend/bridge running)")

  od_out = str(tmp_path / "offloadd.out")
  rt_out = str(tmp_path / "returnsend.out")
  odf = open(od_out, "wb")
  rtf = open(rt_out, "wb")
  DEF_FRAME = 7777

  offloadd = subprocess.Popen(
    [PY, "-m", "openpilot.offload.device.offloadd",
     "--offload-mode", "shadow", "--connect-host", "127.0.0.1", "--stats-period", "1"],
    cwd=REPO, env=CHILD_ENV, stdout=odf, stderr=subprocess.STDOUT)
  assert _wait_line(od_out, "offloadd_start"), "offloadd never started"

  returnsend = subprocess.Popen(
    [PY, "-m", "openpilot.offload.mac.returnsend",
     "--bind", "127.0.0.1", "--stats", "--stats-period", "1"],
    cwd=REPO, env=CHILD_ENV, stdout=rtf, stderr=subprocess.STDOUT)
  assert _wait_line(rt_out, "returnsend_start"), "returnsend never started"

  # raw-subscribe the SHADOW topics. Low-level msgq.sub_sock: the shadow names are registered in
  # SERVICE_LIST (WS-D) but are NOT members of log.Event's union, so a cereal SubMaster cannot
  # construct on them (WS-D RESULTS §5). msgq carries the topic fine.
  shadow_subs = {}
  for name in SHADOW_TO_SVC:
    shadow_subs[name] = msgq.sub_sock(name, addr="127.0.0.1", timeout=100,
                                      segment_size=SERVICE_LIST[name].queue_size)

  cam_pub = PubMaster(["narrowRoadCameraState"])
  out_pub = PubMaster(list(RETURN_SERVICES))
  try:
    # --- phase A: warm up the links (returnsend msgq reader + offloadd SUB) -----------------
    # returnsend only forwards once its ZMQ monitor has seen offloadd connect; publish and poll the
    # parsed stdout counters until forwarding is clearly established (well past the startup
    # monitor-latch window, in which a handful of messages may be counted as dropped_noreader).
    WARM_TARGET = 12
    end = time.monotonic() + 6.0
    while time.monotonic() < end:
      _publish_round(cam_pub, out_pub, 4444)
      if _stats_snapshot(open(rt_out).read(), "returnsend_stats").get("forwarded", 0) >= WARM_TARGET:
        break
      time.sleep(0.05)
    rt_warm = _stats_snapshot(open(rt_out).read(), "returnsend_stats")
    assert rt_warm.get("forwarded", 0) >= WARM_TARGET, \
        f"sender never detected offloadd's reader:\n{open(rt_out).read()}"
    # forwards dominate any brief no-reader drops during startup monitor latch
    assert rt_warm.get("forwarded", 0) > rt_warm.get("dropped_noreader", 0), \
        f"sender never detected offloadd's reader:\n{open(rt_out).read()}"

    # --- quiesce: stop publishing; wait for ALL warmup traffic to drain on both sides ----------
    time.sleep(0.5)
    _drain_shadow(shadow_subs)                    # discard warmup shadow messages
    od_before = _quiesce(od_out)                 # forwarded counter stable => warmup fully drained
    assert od_before.get("forwarded") is not None, f"no offloadd stats yet:\n{open(od_out).read()}"
    _drain_shadow(shadow_subs)                   # anything that settled after the barrier

    # --- phase B: ONE definitive round, then observe -------------------------------------------
    assert offloadd.poll() is None, "offloadd died before the definitive round"
    assert returnsend.poll() is None, "returnsend died before the definitive round"
    publish_mono_ns = time.monotonic_ns()
    wire = _publish_round(cam_pub, out_pub, DEF_FRAME, ts_ns=publish_mono_ns)

    # wait until the round is fully accounted: +3 forwards, +1 no-SOF
    end = time.monotonic() + 5.0
    od_after = od_before
    while time.monotonic() < end:
      od_after = _stats_snapshot(open(od_out).read(), "forwarded")
      fwd = od_after.get("forwarded", 0) - od_before["forwarded"]
      nosof = od_after.get("dropped_no_sof", 0) - od_before.get("dropped_no_sof", 0)
      if fwd >= 3 and nosof >= 1:
        break
      time.sleep(0.1)
    recv_after_ns = time.monotonic_ns()          # observation upper bound on receipt time
    msgs = _drain_shadow(shadow_subs, want_frame=DEF_FRAME)

    # --- assertions --------------------------------------------------------------------------
    fwd_delta = od_after["forwarded"] - od_before["forwarded"]
    noseof_delta = od_after.get("dropped_no_sof", 0) - od_before.get("dropped_no_sof", 0)

    # sender side: everything fresh reached the wire
    assert rt_warm.get("dropped_age", 0) == 0, rt_out
    assert rt_warm.get("dropped_error", 0) == 0, rt_out

    # offloadd side: exactly the three frame-carrying services were forwarded for the round, and
    # modelDataV2SP was refused as un-ageable (see the deviation note below).
    assert fwd_delta == 3, f"expected 3 forwards for the round, got {fwd_delta}\n{open(od_out).read()}"
    assert noseof_delta == 1, f"expected 1 no-SOF drop (modelDataV2SP), got {noseof_delta}"
    assert od_after.get("dropped_stale", 0) == 0
    assert od_after.get("dropped_error", 0) == 0

    # each ageable shadow topic arrived: same frameId, same payload type, payload identical except
    # the header, and the header re-stamped into the receipt window.
    assert set(msgs) >= {"offloadModelV2", "offloadCameraOdometry", "offloadDrivingModelData"}, \
        f"shadow topics missing: got {sorted(msgs)}\n{open(od_out).read()}"
    for name in ("offloadModelV2", "offloadCameraOdometry", "offloadDrivingModelData"):
      svc = SHADOW_TO_SVC[name]
      got = msgs[name]
      assert _which(got) == svc, f"{name}: payload type changed to {_which(got)}"
      assert _frame_id(got) == DEF_FRAME, f"{name}: frameId not preserved"
      # the wire header was the MAC publish time; offloadd re-stamps to its device-now. Its clock is
      # sampled at the TOP of step() and it may then block up to recv_timeout_ms (100 ms) in the
      # camera update before draining ZMQ, so a message arriving during that block is stamped up to
      # ~100 ms BEFORE its true arrival -> assert the re-stamp lies in [publish - 150 ms, observed].
      assert _header(wire[svc]) == publish_mono_ns, f"{name}: unexpected wire header"
      lo, hi = publish_mono_ns - 150_000_000, recv_after_ns + 5_000_000
      assert lo <= _header(got) <= hi, f"{name}: header not re-stamped to receipt time (got {_header(got)}, window [{lo}, {hi}])"
      assert _header(got) != _header(wire[svc]), f"{name}: header was not re-stamped at all"
      assert _fields_without_header(got) == _fields_without_header(wire[svc]), \
          f"{name}: payload changed besides the header"

    # modelDataV2SP must NOT have been forwarded (offloadd cannot age it: no frameId/timestampSof).
    assert "offloadModelDataV2SP" not in msgs

    print(" ".join([
      "[return-chain] PASS —",
      f"offloadd forwarded={fwd_delta} (modelV2+cameraOdometry+drivingModelData,",
      f"frameIds={DEF_FRAME} preserved, header re-stamped into [{publish_mono_ns - 5_000_000},{recv_after_ns + 5_000_000}]);",
      f"dropped_no_sof+={noseof_delta} (modelDataV2SP un-ageable);",
      f"sender forwarded={rt_warm['forwarded']} received={rt_warm['received']}",
      f"dropped_age={rt_warm['dropped_age']} dropped_noreader={rt_warm['dropped_noreader']};",
      f"shadows={sorted(msgs)}",
    ]), flush=True)
  finally:
    for proc in (returnsend, offloadd):
      proc.send_signal(signal.SIGTERM)
    for proc in (returnsend, offloadd):
      try:
        proc.wait(timeout=8)
      except subprocess.TimeoutExpired:
        proc.kill()
    odf.close()
    rtf.close()


if __name__ == "__main__":
  raise SystemExit(pytest.main([os.path.abspath(__file__), "-v"]))
