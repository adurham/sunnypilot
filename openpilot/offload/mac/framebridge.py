#!/usr/bin/env python3
"""framebridge.py — WS-B Mac-side frame path hub.

Responsibilities (INTERFACES.md §1-3):

  (a) ZMQ SUB (pyzmq, connect) to the device bridge for narrowRoadEncodeData,
      wideRoadEncodeData and the small services in contract.MAC_LOCAL_SERVICES,
      ports from openpilot.offload.ports, host from --host.
      NOTE: this repo's msgq build has NO ZMQ transport (MSGQSubSocket::connect
      asserts addr == "127.0.0.1"), so the device link uses pyzmq directly with
      the same framing the C++ bridge_zmq uses: single-part raw cereal Event
      bytes, subscribe "", tcp://<host>:<port>.
  (b) Feed each camera's EncodeData to its vtdec subprocess (one per camera).
  (c) Receive decoded frames; publish into a LOCAL VisionIpcServer named
      OFFLOAD_VIPC_SERVER (default 'camerad') with the DEVICE frame_id / sof / eof.
      Buffers use DEVICE NV12 geometry via create_buffers_with_sizes — stride/
      y_height/uv_height/size from nv12_info (VENUS), NOT the tight create_buffers
      w*h*3/2 (OPEN-C1).
  (d) Synthesize + publish Mac-local narrowRoadCameraState / wideRoadCameraState
      from EncodeData idx, header logMonoTime re-stamped to Mac time.monotonic_ns()
      per §1.2 (payload timestamps stay device values). Also re-publish all small
      services received from the device, header-restamped only, into local msgq —
      that is what makes modeld_v2's SubMaster work.
  (e) Single ZMQ context; per-service pyzmq SUB sockets; PubMaster for local pubs.
  (f) --replay: read a directory of pre-extracted fixtures (see mac/fixtures.py)
      instead of ZMQ.
  (g) Latency logging: append contract.FrameEvent / LatencyRecord rows to jsonl
      (--out).
  Clean shutdown on SIGINT/SIGTERM (terminate vtdec children).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import signal
import struct
import subprocess
import sys
import threading
import time

# --- ZMQ semantics must be decided before importing messaging ----------------
# We deliberately do NOT set os.environ["ZMQ"]="1": this msgq build has no ZMQ
# transport and would abort. Local pubs/subs run over the msgq shared-memory path.
import openpilot.cereal.messaging as messaging
from openpilot.cereal import log as capnp_log
from openpilot.cereal.visionipc import VisionStreamType
from openpilot.offload import contract, ports
from openpilot.offload.mac.frametable import FrameTable
from openpilot.offload.mac import fixtures as fix
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info

try:
  from msgq.visionipc import VisionIpcServer
except Exception as e:  # pragma: no cover
  raise SystemExit(f"msgq.visionipc unavailable: {e}") from e

CAMERAS = {
  "narrow": {
    "encode": "narrowRoadEncodeData",
    "cam_state": "narrowRoadCameraState",
    "stream": VisionStreamType.VISION_STREAM_NARROW_ROAD,
  },
  "wide": {
    "encode": "wideRoadEncodeData",
    "cam_state": "wideRoadCameraState",
    "stream": VisionStreamType.VISION_STREAM_WIDE_ROAD,
  },
}
V4L2_BUF_FLAG_KEYFRAME = 8

# Record framing sent to / received from vtdec (all little-endian):
IN_HDR = struct.Struct("<I")          # [u32 nbytes]
IN_META = struct.Struct("<QI")        # [u64 pts][u32 flags]
OUT_HDR = struct.Struct("<I")         # [u32 nbytes]
OUT_META = struct.Struct("<QQ")       # [u64 pts][u64 decode_ns]


def log(msg: str) -> None:
  print(f"[framebridge] {msg}", flush=True)


def elog(msg: str) -> None:
  print(f"[framebridge] {msg}", file=sys.stderr, flush=True)


# --------------------------------------------------------------------------- #
#  Device link: raw cereal over pyzmq on the frozen port scheme.
# --------------------------------------------------------------------------- #
class ZmqSubscriber(threading.Thread):
  """pyzmq SUB for one service; hands raw cereal bytes to a callback."""

  def __init__(self, ctx, service: str, host: str, on_bytes):
    super().__init__(daemon=True, name=f"zmq-{service}")
    import zmq
    self._zmq = zmq
    self.service = service
    self.endpoint = ports.endpoint(service, host)
    self.sock = ctx.socket(zmq.SUB)
    self.sock.setsockopt(zmq.SUBSCRIBE, b"")
    self.sock.setsockopt(zmq.RCVHWM, 100)
    self.sock.connect(self.endpoint)
    self._on_bytes = on_bytes
    self._stop = threading.Event()

  def run(self):
    poller = self._zmq.Poller()
    poller.register(self.sock, self._zmq.POLLIN)
    while not self._stop.is_set():
      try:
        events = dict(poller.poll(100))
      except Exception:
        break
      if events.get(self.sock) == self._zmq.POLLIN:
        try:
          raw = self.sock.recv()
        except Exception:
          continue
        self._on_bytes(self.service, raw)

  def stop(self):
    self._stop.set()
    try:
      self.sock.close(linger=0)
    except Exception:
      pass


# --------------------------------------------------------------------------- #
#  vtdec subprocess wrapper
# --------------------------------------------------------------------------- #
class Vtdec:
  """One vtdec child; stdin framed AU in, stdout framed decoded NV12 out."""

  def __init__(self, binary: str, name: str, params: bytes | None, on_frame):
    self.name = name
    self._on_frame = on_frame
    cmd = [binary]
    self._params_path = None
    if params:
      self._params_path = os.path.join(_scratch(), f"vtdec_params_{name}_{os.getpid()}.hevc")
      with open(self._params_path, "wb") as f:
        f.write(params)
      cmd += ["--params", self._params_path]
    self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                 stderr=None, bufsize=0)
    self._wlock = threading.Lock()
    self._rthread = threading.Thread(target=self._read_loop, daemon=True, name=f"vtdec-r-{name}")
    self._rthread.start()
    self._closed = False

  def feed(self, data: bytes, pts: int, flags: int) -> bool:
    if self._closed or self.proc.poll() is not None:
      return False
    try:
      with self._wlock:
        self.proc.stdin.write(IN_HDR.pack(len(data)))
        self.proc.stdin.write(data)
        self.proc.stdin.write(IN_META.pack(pts, flags))
      return True
    except (BrokenPipeError, OSError):
      self._closed = True
      return False

  def _read_exact(self, n: int) -> bytes | None:
    buf = bytearray()
    f = self.proc.stdout
    while len(buf) < n:
      chunk = f.read(n - len(buf))
      if not chunk:
        return None
      buf += chunk
    return bytes(buf)

  def _read_loop(self):
    while True:
      hdr = self._read_exact(OUT_HDR.size)
      if hdr is None:
        return
      (nbytes,) = OUT_HDR.unpack(hdr)
      payload = self._read_exact(nbytes) if nbytes else b""
      meta = self._read_exact(OUT_META.size)
      if payload is None or meta is None:
        return
      pts, dec_ns = OUT_META.unpack(meta)
      self._on_frame(self.name, pts, payload, dec_ns)

  def close(self):
    self._closed = True
    try:
      self.proc.stdin.close()
    except Exception:
      pass
    try:
      self.proc.terminate()
      self.proc.wait(timeout=2)
    except Exception:
      try:
        self.proc.kill()
      except Exception:
        pass


def _scratch() -> str:
  d = os.environ.get("TMPDIR", "/tmp")
  os.makedirs(d, exist_ok=True)
  return d


# --------------------------------------------------------------------------- #
#  Bridge
# --------------------------------------------------------------------------- #
class FrameBridge:
  def __init__(self, args):
    self.args = args
    self.vipc_name = os.environ.get("OFFLOAD_VIPC_SERVER", contract.DEFAULT_VIPC_SERVER)
    self.table = FrameTable(retention=max(600, args.retention))
    self.stop = threading.Event()
    self.table_lock = threading.Lock()

    # pending EncodeData per camera, keyed by pts we hand to vtdec (monotone)
    self._pending = {c: {} for c in CAMERAS}          # cam -> pts -> dict
    self._next_pts = dict.fromkeys(CAMERAS, 1)
    self._pts_lock = threading.Lock()

    # per-frame latency rows (contract.LatencyRecord), keyed (cam, frame_id)
    self._lat = {}
    self._lat_lock = threading.Lock()

    self._vipc = None
    self._vipc_streams = {}          # cam -> stream
    self._vipc_ready = threading.Event()
    self._dims = {}                  # cam -> (w, h)
    self._dims_lock = threading.Lock()

    self._decoders = {}
    self._out_fp = None
    self._out_lock = threading.Lock()
    self._digest_fp = None                  # opt-in --digest-out (WS-C integrity harness)
    self._digest_lock = threading.Lock()
    self._published = dict.fromkeys(CAMERAS, 0)
    self._received = dict.fromkeys(CAMERAS, 0)
    self._decoded = dict.fromkeys(CAMERAS, 0)

    self.pm = None
    self._pm_services = None

  # --- local publish setup (must be built after we know which services flow) --
  def _ensure_publishers(self, services: list[str]):
    want = sorted(set(services))
    if self._pm_services == want:
      return
    # PubMaster cannot be rebuilt cheaply; build the union once.
    union = sorted(set(want) | set(self._pm_services or []))
    self.pm = messaging.PubMaster(union)
    self._pm_services = union

  # --- VisionIPC ----------------------------------------------------------
  def _ensure_vipc(self, cam: str, w: int, h: int):
    """Lazily create the VisionIpcServer once dimensions for any camera known.

    VisionIpcServer.create_buffers must be called for every stream we intend to
    publish before start_listener(); we create buffers for both cameras using the
    first EncodeData dimensions (narrow and wide share dimensions on this route).
    """
    with self._dims_lock:
      self._dims[cam] = (w, h)
      if self._vipc is not None:
        return
      if len(self._dims) < 1:
        return
      dims = next(iter(self._dims.values()))
      srv = VisionIpcServer(self.vipc_name)
      geo = {}
      for c, meta in CAMERAS.items():
        cw, ch = self._dims.get(c, dims)
        # OPEN-C1: allocate buffers with DEVICE NV12 geometry (stride/y_height/uv_height/
        # size from nv12_info), matching camerad's create_buffers_with_sizes on-device —
        # NOT msgq's tight create_buffers (w*h*3/2, stride==width).
        stride, y_height, uv_height, size = get_nv12_info(cw, ch)
        uv_offset = stride * y_height
        srv.create_buffers_with_sizes(meta["stream"], self.args.num_buffers, cw, ch, size, stride, uv_offset)
        self._vipc_streams[c] = meta["stream"]
        geo[c] = (stride, uv_offset, size)
      srv.start_listener()
      self._vipc = srv
      s0, u0, z0 = geo[next(iter(geo))]
      log(f"VisionIPC server '{self.vipc_name}' listening: {dims[0]}x{dims[1]}, {self.args.num_buffers} buffers, stride={s0} uv_offset={u0} size={z0}")
      self._vipc_ready.set()

  # --- latency jsonl ------------------------------------------------------
  def _open_out(self):
    if self.args.out:
      self._out_fp = open(self.args.out, "a", buffering=1)
    if getattr(self.args, "digest_out", None):
      self._digest_fp = open(self.args.digest_out, "a", buffering=1)

  def _emit_jsonl(self, obj: dict):
    if self._out_fp is None:
      return
    with self._out_lock:
      self._out_fp.write(json.dumps(obj, separators=(",", ":")) + "\n")

  def _emit_digest(self, obj: dict):
    """Opt-in (--digest-out): one jsonl row per received EncodeData, carrying the
    sha256 of the EXACT received payload so WS-C's integrity checker can compare it
    against the fixture digest for that (frame_id, encode_id). Default path is
    unchanged when the flag is absent (self._digest_fp stays None)."""
    if self._digest_fp is None:
      return
    with self._digest_lock:
      self._digest_fp.write(json.dumps(obj, separators=(",", ":")) + "\n")

  # --- EncodeData handling ------------------------------------------------
  def _on_encode(self, cam: str, raw: bytes):
    evt = messaging.log_from_bytes(raw)
    which = evt.which()
    if which != CAMERAS[cam]["encode"]:
      return
    ed = getattr(evt, which)
    idx = ed.idx
    frame_id = int(idx.frameId)
    encode_id = int(idx.encodeId)
    sof = int(idx.timestampSof)
    eof = int(idx.timestampEof)
    flags = int(idx.flags)
    data = ed.data
    width = int(ed.width)
    height = int(ed.height)
    recv_mac = time.monotonic_ns()

    self._received[cam] += 1

    # (WS-C integrity, opt-in via --digest-out): digest the EXACT received payload so
    # the checker can prove bytes on the wire == fixture bytes for this (frame_id,
    # encode_id). Emitted on receipt, before any decode/publish, so it cannot be
    # affected by downstream drops. Guarded so the flag-absent path computes nothing.
    if self._digest_fp is not None:
      self._emit_digest({
        "kind": "encode", "cam": cam, "frame_id": frame_id, "encode_id": encode_id,
        "sha256": hashlib.sha256(bytes(data)).hexdigest(), "len": len(data),
        "recv_mac_ns": recv_mac,
      })

    # (c) init VisionIPC on first EncodeData dimensions.
    if cam not in self._dims and width > 0 and height > 0:
      self._ensure_vipc(cam, width, height)

    # (d) synthesize + publish Mac-local cameraState (header re-stamped).
    if self.pm is not None:
      cs = messaging.new_message(CAMERAS[cam]["cam_state"])
      fs = getattr(cs, CAMERAS[cam]["cam_state"])
      fs.frameId = frame_id
      fs.frameIdSensor = frame_id
      fs.requestId = 0
      fs.encodeId = encode_id
      fs.timestampSof = sof
      fs.timestampEof = eof
      fs.processingTime = (eof - sof) / 1e6 if eof > sof else 0.0
      # header logMonoTime is set by new_message to time.monotonic()*1e9 (Mac clock)
      cs.logMonoTime = recv_mac
      self.pm.send(CAMERAS[cam]["cam_state"], cs)

    # FrameTable row (single source of truth).
    evt_row = contract.FrameEvent(
      frame_id=frame_id, encode_id=encode_id, timestamp_sof=sof, timestamp_eof=eof,
      is_keyframe=bool(flags & V4L2_BUF_FLAG_KEYFRAME), payload_bytes=len(data),
      recv_mac_ns=recv_mac)
    self.table.append(evt_row)
    with self._lat_lock:
      self._lat[(cam, frame_id)] = contract.LatencyRecord(
        frame_id=frame_id, cam=cam, sof_dev_ns=sof, eof_dev_ns=eof, recv_mac_ns=recv_mac)

    # (b) feed vtdec.
    dec = self._decoders.get(cam)
    if dec is None:
      return
    # The device puts VPS/SPS/PPS in EncodeData.header on keyframes (encoder.cc
    # `if (flags & V4L2_BUF_FLAG_KEYFRAME) edat.setHeader(header)`); .data is VCL
    # only. Forward the header as a codec-config block so vtdec can (re)build its
    # format description in-band. Harmless to repeat.
    header = getattr(ed, "header", b"") or b""
    if len(header) > 0:
      dec.feed(header, 0, 2)   # flags bit1 = codec-config block
    with self._pts_lock:
      pts = self._next_pts[cam]
      self._next_pts[cam] += 1
    self._pending[cam][pts] = {"frame_id": frame_id, "encode_id": encode_id,
                               "sof": sof, "eof": eof, "flags": flags,
                               "recv_mac": recv_mac, "bytes": len(data)}
    ok = dec.feed(data, pts, flags)
    if not ok:
      self._pending[cam].pop(pts, None)

  # --- decoded frame handling ---------------------------------------------
  def _on_decoded(self, cam: str, pts: int, payload: bytes, dec_ns: int):
    meta = self._pending[cam].pop(pts, None)
    if meta is None:
      return
    self._decoded[cam] += 1
    now_mac = time.monotonic_ns()
    frame_id = meta["frame_id"]
    sof, eof = meta["sof"], meta["eof"]

    # (c) publish into local VisionIPC with DEVICE frame_id / sof / eof.
    self._vipc_ready.wait(5.0)
    if self._vipc is not None:
      vst = self._vipc_streams.get(cam)
      if vst is not None:
        try:
          self._vipc.send(vst, payload, frame_id, sof, eof)
          self._published[cam] += 1
        except Exception as e:
          elog(f"VisionIPC send failed ({cam} {frame_id}): {e}")

    # update FrameTable + latency rows
    self.table.update(frame_id, decode_ms=dec_ns / 1e6, publish_mac_ns=now_mac)
    with self._lat_lock:
      lr = self._lat.get((cam, frame_id))
      if lr is not None:
        lr.decode_done_mac_ns = now_mac
        lr.vipc_publish_mac_ns = now_mac

    self._emit_jsonl({
      "kind": "frame", "cam": cam, "frame_id": frame_id, "encode_id": meta["encode_id"],
      "sof_dev_ns": sof, "eof_dev_ns": eof, "recv_mac_ns": meta["recv_mac"],
      "decode_done_mac_ns": now_mac, "vipc_publish_mac_ns": now_mac,
      "decode_ms": dec_ns / 1e6, "payload_bytes": meta["bytes"],
    })

  # --- small-service republish --------------------------------------------
  def _on_small(self, service: str, raw: bytes):
    # (d) re-stamp header logMonoTime to Mac clock, payload untouched (§1.2).
    try:
      evt = messaging.log_from_bytes(raw)
      which = evt.which()
      b = evt.as_builder()
      b.logMonoTime = time.monotonic_ns()
      out = b.to_bytes()
    except Exception as e:
      elog(f"small-service restamp failed ({service}): {e}")
      return
    try:
      self.pm.send(which, out)
    except Exception as e:
      elog(f"small-service republish failed ({service}): {e}")

  # --- run ----------------------------------------------------------------
  def run_zmq(self):
    import zmq
    ctx = zmq.Context.instance()
    encode_services = [m["encode"] for m in CAMERAS.values()]
    small_services = list(contract.MAC_LOCAL_SERVICES)

    self._ensure_publishers(small_services + [m["cam_state"] for m in CAMERAS.values()])
    self._open_out()

    for cam, _meta in CAMERAS.items():
      params = None
      if self.args.params_dir:
        p = os.path.join(self.args.params_dir, f"{cam}.params")
        if os.path.exists(p):
          params = open(p, "rb").read()
      self._decoders[cam] = Vtdec(self.args.vtdec, cam, params, self._on_decoded)

    subs = []
    for svc in encode_services:
      cam = next(c for c, m in CAMERAS.items() if m["encode"] == svc)
      subs.append(ZmqSubscriber(ctx, svc, self.args.host,
                                lambda s, r, _cam=cam: self._on_encode(_cam, r)))
    for svc in small_services:
      if svc in ("narrowRoadCameraState", "wideRoadCameraState"):
        continue  # synthesized locally, not consumed from device
      subs.append(ZmqSubscriber(ctx, svc, self.args.host,
                                lambda s, r, _svc=svc: self._on_small(_svc, r)))
    for s in subs:
      s.start()
    log(f"ZMQ SUB to {self.args.host}: {len(subs)} services ({', '.join(encode_services)} + small)")
    self._loop()
    for s in subs:
      s.stop()

  def run_replay(self):
    cam = self.args.replay_cam
    meta = CAMERAS[cam]
    self._ensure_publishers([meta["cam_state"]])
    self._open_out()

    params = None
    if self.args.hevc_params:
      params = fix.load_params_from_fixture_source(self.args.hevc_params)
    self._decoders[cam] = Vtdec(self.args.vtdec, cam, params, self._on_decoded)

    path = self.args.replay_path
    if os.path.isdir(path):
      path = fix.fixture_path_for(path, cam)
    n = 0
    t0 = time.monotonic()
    for rec in fix.iter_fixture(path, max_frames=self.args.replay_frames):
      if self.stop.is_set():
        break
      # emulate a device EncodeData message locally, then reuse the same path
      raw = _encode_data_bytes(meta["encode"], rec)
      self._on_encode(cam, raw)
      n += 1
      if self.args.replay_pace > 0:
        time.sleep(self.args.replay_pace)
    dt = time.monotonic() - t0
    log(f"replay: fed {n} records from {path} in {dt:.2f}s")
    # drain
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
      with self._pts_lock:
        inflight = sum(len(self._pending[c]) for c in CAMERAS)
      if inflight == 0:
        break
      time.sleep(0.05)

  def _loop(self):
    while not self.stop.is_set():
      time.sleep(0.2)

  # --- reporting ----------------------------------------------------------
  def summary(self) -> dict:
    return {
      "received": dict(self._received),
      "decoded": dict(self._decoded),
      "published": dict(self._published),
      "frametable_rows": self.table.count,
    }

  def close(self):
    self.stop.set()
    for d in self._decoders.values():
      d.close()
    if self._out_fp:
      try:
        self._out_fp.close()
      except Exception:
        pass
    if self._digest_fp:
      try:
        self._digest_fp.close()
      except Exception:
        pass


def _encode_data_bytes(service: str, rec: fix.FixtureRecord) -> bytes:
  """Build raw cereal Event bytes for a camera EncodeData from a fixture record.

  Mirrors VideoEncoder::publisher_publish (encoder.cc): idx.frameId/encodeId/
  timestampSof/timestampEof/type/segmentNum/segmentId/flags/len, data, width/height.
  """
  evt = capnp_log.Event.new_message()
  evt.logMonoTime = rec.timestamp_sof  # device clock on the wire (§1.1)
  evt.init(service)
  ed = getattr(evt, service)
  idx = ed.idx
  idx.frameId = rec.frame_id
  idx.encodeId = rec.encode_id
  idx.timestampSof = rec.timestamp_sof
  idx.timestampEof = rec.timestamp_eof
  idx.flags = rec.flags
  idx.len = len(rec.data)
  idx.type = "fullHEVC"
  ed.data = rec.data
  ed.width = 1928
  ed.height = 1208
  return evt.to_bytes()


# --------------------------------------------------------------------------- #
def build_argparser():
  p = argparse.ArgumentParser(description="WS-B Mac frame path hub")
  p.add_argument("--host", default="127.0.0.1", help="device bridge host (ZMQ SUB connect)")
  p.add_argument("--vtdec", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "vtdec"))
  p.add_argument("--out", default=None, help="jsonl latency log path")
  p.add_argument("--digest-out", default=None,
                 help="opt-in per-recv-EncodeData digest jsonl {frame_id,encode_id,sha256(data),len,recv_mac_ns} for WS-C integrity")
  p.add_argument("--retention", type=int, default=1200, help="FrameTable retention (>=600)")
  p.add_argument("--num-buffers", type=int, default=4, help="VisionIPC buffers")
  p.add_argument("--params-dir", default=None, help="dir with <cam>.params Annex-B files (ZMQ mode)")
  p.add_argument("--replay", action="store_true", help="replay mode (read fixtures, no ZMQ)")
  p.add_argument("--replay-path", default=None, help="fixtures dir or .enc file (replay mode)")
  p.add_argument("--replay-cam", default="narrow", choices=list(CAMERAS))
  p.add_argument("--replay-frames", type=int, default=None, help="max records (replay)")
  p.add_argument("--replay-pace", type=float, default=0.0, help="seconds to sleep per record (replay)")
  p.add_argument("--hevc-params", default=None, help="raw .hevc to source VPS/SPS/PPS (replay mode)")
  return p


def main(argv=None):
  args = build_argparser().parse_args(argv)
  br = FrameBridge(args)

  def _sig(signum, frame):
    log(f"signal {signum} — shutting down")
    br.stop.set()

  signal.signal(signal.SIGINT, _sig)
  signal.signal(signal.SIGTERM, _sig)

  try:
    if args.replay:
      if not args.replay_path:
        raise SystemExit("--replay requires --replay-path")
      br.run_replay()
      time.sleep(1.0)  # let vtdec drain
    else:
      br.run_zmq()
  except KeyboardInterrupt:
    pass
  finally:
    s = br.summary()
    log(f"summary: {json.dumps(s)}")
    br.close()
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
