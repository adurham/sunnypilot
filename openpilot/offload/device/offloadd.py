#!/usr/bin/env python3
"""openpilot.offload.device.offloadd — DEVICE-side return-path republisher.

Status: P2 code-complete, P4-prep. **NOT activated in P1-P4 without an explicit user gate.**
It is inert unless the device param ``OffloadMode`` is ``shadow``/``drive`` (or the daemon is
run with ``--offload-mode shadow|drive`` for bench use).

What it does (normative spec = ../INTERFACES.md §1, ../contract.py):

  drive/shadow mode: the Mac computes the driving model and publishes its outputs
  (``RETURN_SERVICES``) back over ZMQ. This daemon subscribes to those, freshness-checks them
  against the DEVICE monotonic clock, re-stamps the cereal ``Event`` header ``logMonoTime`` to
  device now, and republishes into the **local** msgq so that the unchanged ``controlsd``/
  ``plannerd`` see a normal stream. No clock sync is ever performed anywhere.

Steps:
  (a) ZMQ SUB (connect) to the Mac for every RETURN_SERVICES port (Mac side binds PUB,
      device connects; ports from ``openpilot.offload.ports``).
  (b) Freshness gate per §1.3 using the device clock: track ``timestampSof`` per camera from the
      last local ``narrowRoadCameraState``. For each incoming return msg, ``sof_of_its_frame``
      is the device-clock SOF of the camera frame the model output refers to. If
      ``now_device_ns - sof > OFFLOAD_STALE_MS`` (default 300 ms) -> DROP + count.
      Else -> keep the payload byte-identical, set header ``logMonoTime = time.monotonic_ns()``,
      publish into local msgq via ``PubMaster``.
  (c) Gap watchdog: if there is no *valid republish* for ``OFFLOAD_GAP_MS`` (default 400 ms) while
      engaged, publish nothing. The resulting absence of ``modelV2``/``cameraOdometry`` messages is
      itself the staleness signal -> ``controlsd``'s existing soft-disable path (``SubMaster``
      alive/valid checks) fires. One event line is logged when a gap opens and when it closes.
  (d) Stats: one JSON object per line to **stdout only**. Bench mode (``--bench``) may also append
      a local JSONL for convenience. Never writes to /data.
  (e) Mode gate: only runs when ``params.get("OffloadMode")`` in {"shadow", "drive"}; tolerates
      ``UnknownKeyName`` (key absent) by staying disabled.
  (f) SIGTERM/SIGINT -> publish nothing further, close sockets, exit 0.

Failure-mode table (design reference / operational rules)
---------------------------------------------------------
  | Mode                        | Detected by                          | Daemon action                    | System effect                                  |
  |-----------------------------|--------------------------------------|----------------------------------|------------------------------------------------|
  | link death (tether drops)   | ZMQ recv timeout budget exhausted    | stop republishing; open gap      | modelV2 staleness -> soft-disable              |
  | bridge death (device)       | local narrowRoadCameraState stalls   | SOF map ages out -> all drops    | gap -> soft-disable                            |
  | Mac pipeline stall/lag      | age of return frame SOF > stale_ms   | drop + count stale               | controlsd sees no fresh modelV2 -> soft-disable|
  | segment boundary / route end| cameraOdometry frameId resets/gap    | drop unmatched; keep republishing| brief stale window; loggerd unaffected        |
  | encoder GOP gap            | no return msg for a frame id         | nothing to publish for that frame| handled upstream (WS-B); not this daemon's job |
  | ZMQ HWM drops              | SUB side: messages silently dropped  | (see note) count via recv gaps   | gap watchdog covers the symptom                |
  | echo loop (Mac sub'd to dev)| Mac SubMaster on modelV2 sees its own| MUST NOT HAPPEN                  | OPERATIONAL RULE: the Mac runner must NOT      |
  |                             | republished output                    |                                  | subscribe to modelV2 on the device ZMQ while it |
  |                             |                                       |                                  | is republishing; keep the return PUB separate   |

Notes:
  * SOF (not EOF) is used for the freshness age, matching §1.4 ("now_device - timestamp_sof").
  * ``drivingModelData`` has no ``timestampSof`` field; its SOF is taken from the same-frame
    camera state (by ``frameId``), exactly like the others. If no camera SOF is known for a
    frame, the message is dropped as un-ageable (never republished blind).
  * ZMQ SUB socket ``RCVTIMEO`` is set (default 100 ms) so the gap watchdog and SIGTERM handling
    are promptly serviced even when the Mac goes quiet. ``CONFLATE`` is deliberately NOT set:
    every frame must be seen so the stale accounting is exact.
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import time
from collections import deque
from dataclasses import dataclass, field

import zmq

from openpilot.offload import ports
from openpilot.offload.contract import RETURN_SERVICES

# --- tunables (env-overridable; see INTERFACES/README) -----------------------------
DEFAULT_STALE_MS = 300          # OFFLOAD_STALE_MS — max device-clock age of a return frame
DEFAULT_GAP_MS = 400            # OFFLOAD_GAP_MS   — no-valid-republish watchdog
DEFAULT_SOF_TTL_MS = 1000       # drop camera SOF entries older than this
DEFAULT_RECV_TIMEOUT_MS = 100   # ZMQ SUB RCVTIMEO (prompt watchdog + SIGTERM)
RING_SIZE = 4096                # recent republish timestamps for the watchdog
CAMERA_SERVICES = ("narrowRoadCameraState", "wideRoadCameraState")
ENABLED_MODES = ("shadow", "drive")


def _env_ms(name: str, default: int) -> int:
  """Read an integer-ms env var, tolerating garbage."""
  raw = os.getenv(name)
  if raw is None or raw == "":
    return default
  try:
    return int(float(raw))
  except ValueError:
    return default


def enabled_mode(param_value) -> bool:
  """True when ``param_value`` (bytes | str | None) names an enabling OffloadMode."""
  if param_value is None:
    return False
  if isinstance(param_value, bytes):
    try:
      param_value = param_value.decode()
    except UnicodeDecodeError:
      return False
  return str(param_value).strip().lower() in ENABLED_MODES


def read_offload_mode(params) -> str | None:
  """Best-effort read of the OffloadMode param; tolerant of an absent key and of bad values."""
  if params is None:
    return None
  try:
    v = params.get("OffloadMode")
  except Exception:
    # covers UnknownKeyName and any Params backend hiccup: stay disabled
    return None
  if v is None:
    return None
  if isinstance(v, bytes):
    try:
      v = v.decode()
    except UnicodeDecodeError:
      return None
  return str(v).strip() or None


def sof_for_service(msg, camera_sof: dict) -> int | None:
  """Device-clock SOF (ns) of the camera frame a return message refers to, or None if unknown.

  Prefers an inline ``timestampSof`` when the message carries one (cameraOdometry does not —
  it has only ``timestampEof``/``frameId``; modelV2/drivingModelData/modelDataV2SP carry
  ``timestampEof`` and ``frameId``). Falls back to the ``frameId`` -> SOF map fed by the local
  camera states; that map is the authoritative device-clock reference.
  """
  fid = None
  if hasattr(msg, "frameId"):
    try:
      fid = int(msg.frameId)
    except Exception:
      fid = None
  if fid is not None:
    sof = camera_sof.get(fid)
    if sof is not None:
      return sof
  # inline SOF (present on some services, e.g. FrameData family); last resort only
  if hasattr(msg, "timestampSof"):
    try:
      return int(msg.timestampSof)
    except Exception:
      return None
  return None


@dataclass
class Config:
  connect_host: str = "127.0.0.1"          # where the Mac PUBs the RETURN services
  stale_ms: int = field(default_factory=lambda: _env_ms("OFFLOAD_STALE_MS", DEFAULT_STALE_MS))
  gap_ms: int = field(default_factory=lambda: _env_ms("OFFLOAD_GAP_MS", DEFAULT_GAP_MS))
  sof_ttl_ms: int = field(default_factory=lambda: _env_ms("OFFLOAD_SOF_TTL_MS", DEFAULT_SOF_TTL_MS))
  recv_timeout_ms: int = DEFAULT_RECV_TIMEOUT_MS
  bench: bool = False
  bench_path: str = "/tmp/offloadd_bench.jsonl"
  offload_mode: str = "auto"               # auto | shadow | drive  (explicit value = bench bypass)
  camera_addr: str = "127.0.0.1"           # local msgq address for camera states
  services: tuple = field(default_factory=lambda: tuple(RETURN_SERVICES))
  endpoint_for: object = None             # callable(service)->endpoint; tests inject a free port


class _CerealIO:
  """Thin wrapper so tests can drive the daemon without a real device msgq.

  Lazily imports the cereal bindings on first use. Any import failure is surfaced by
  ``ready()`` so the daemon can stay inert (and the manager must not start it) on a host
  that lacks the built native extension.
  """

  def __init__(self, cfg: Config):
    self.cfg = cfg
    self._SubMaster = None
    self._PubMaster = None
    self._log = None
    self.sub = None
    self.pub = None

  def ready(self) -> bool:
    if self._SubMaster is not None:
      return True
    try:
      from openpilot.cereal import log
      from openpilot.cereal.messaging import PubMaster, SubMaster
    except Exception:
      return False
    self._log = log
    self._SubMaster = SubMaster
    self._PubMaster = PubMaster
    return True

  def open(self) -> None:
    self.sub = self._SubMaster(list(CAMERA_SERVICES), addr=self.cfg.camera_addr)
    self.pub = self._PubMaster(list(self.cfg.services))

  def update(self, timeout_ms: int) -> None:
    self.sub.update(timeout_ms)

  def camera_sof(self, service: str) -> tuple[int, int] | None:
    """(frameId, timestampSof) of the latest camera state, or None if never received."""
    if not self.sub.seen.get(service):
      return None
    fr = self.sub[service]
    try:
      return int(fr.frameId), int(fr.timestampSof)
    except Exception:
      return None

  def publish(self, service: str, payload: bytes) -> None:
    self.pub.send(service, payload)


def _restamp(raw: bytes, now_ns: int) -> bytes:
  """Return ``raw`` with the Event header ``logMonoTime`` set to ``now_ns`` and payload untouched.

  Uses pycapnp's zero-copy ``as_builder()`` on the reader; only the root header scalar is
  written, so every payload field survives byte-identically (proven by
  tests/test_offloadd.py::test_restamp_payload_byte_identical).
  """
  from openpilot.cereal import log
  limit = 2 ** 64 - 1
  with log.Event.from_bytes(raw, traversal_limit_in_words=limit) as reader:
    b = reader.as_builder()
    b.logMonoTime = int(now_ns)
    return b.to_bytes()


class Offloadd:
  """Return-path republisher. Construct, then ``run()`` (blocking) — or drive with ``step()`` in tests."""

  def __init__(self, cfg: Config | None = None, cereal: _CerealIO | None = None):
    self.cfg = cfg or Config()
    self.cereal = cereal or _CerealIO(self.cfg)
    self.sof_by_frame: dict[int, int] = {}
    self.recent_republish: deque[float] = deque(maxlen=RING_SIZE)
    self.zmq_msgs = 0
    self.published = 0
    self.dropped_stale = 0
    self.dropped_no_sof = 0
    self.dropped_error = 0
    self.gap_open = False
    self.last_republish_mono: float | None = None
    self._stop = False
    self._sub_socks: dict[str, zmq.Socket] = {}
    self._zmq_ctx = None
    self._start_mono = time.monotonic()

  # -- lifecycle ---------------------------------------------------------------
  def request_stop(self, *_: object) -> None:
    self._stop = True

  def install_signal_handlers(self) -> None:
    signal.signal(signal.SIGTERM, self.request_stop)
    signal.signal(signal.SIGINT, self.request_stop)

  def open(self) -> None:
    """Open ZMQ SUB sockets (connect to the Mac) and the local msgq endpoints."""
    if not self.cereal.ready():
      raise RuntimeError("cereal bindings unavailable: openpilot.cereal.messaging / log not importable; manager must not start offloadd on this host")
    self._zmq_ctx = zmq.Context()
    for svc in self.cfg.services:
      s = self._zmq_ctx.socket(zmq.SUB)
      s.setsockopt(zmq.SUBSCRIBE, b"")
      s.setsockopt(zmq.RCVTIMEO, self.cfg.recv_timeout_ms)
      s.setsockopt(zmq.LINGER, 0)
      endpoint = (self.cfg.endpoint_for(svc) if callable(self.cfg.endpoint_for)
                  else ports.endpoint(svc, self.cfg.connect_host))
      s.connect(endpoint)
      self._sub_socks[svc] = s
    self.cereal.open()

  def close(self) -> None:
    for s in self._sub_socks.values():
      try:
        s.close(0)
      except Exception:
        pass
    self._sub_socks.clear()
    if self._zmq_ctx is not None:
      try:
        self._zmq_ctx.term()
      except Exception:
        pass
      self._zmq_ctx = None

  # -- core step ---------------------------------------------------------------
  def step(self, now_ns: int | None = None) -> None:
    """One iteration: refresh camera SOF, drain ZMQ, gate+republish, run gap watchdog.

    ``now_ns`` injects the device clock for deterministic tests; production callers omit it.
    """
    if now_ns is None:
      now_ns = time.monotonic_ns()
    now_mono = now_ns / 1e9
    self._refresh_camera_sof(now_mono)
    self._drain_zmq(now_ns, now_mono)
    self._watchdog(now_mono)

  def _refresh_camera_sof(self, now_mono: float) -> None:
    self.cereal.update(self.cfg.recv_timeout_ms)
    for svc in CAMERA_SERVICES:
      got = self.cereal.camera_sof(svc)
      if got is not None:
        fid, sof = got
        if sof > 0:
          self.sof_by_frame[fid] = sof
    # evict stale entries (route/segment rollover safety)
    if self.sof_by_frame:
      cutoff = (now_mono - self.cfg.sof_ttl_ms / 1000.0) * 1e9
      stale_ids = [fid for fid, sof in self.sof_by_frame.items() if sof < cutoff]
      for fid in stale_ids:
        del self.sof_by_frame[fid]

  def _drain_zmq(self, now_ns: int, now_mono: float) -> None:
    for svc, sock in self._sub_socks.items():
      # drain everything currently queued for this service
      while True:
        try:
          raw = sock.recv(zmq.NOBLOCK)
        except zmq.Again:
          break
        except Exception:
          break
        self.zmq_msgs += 1
        self._handle_return(svc, raw, now_ns, now_mono)

  def _handle_return(self, svc: str, raw: bytes, now_ns: int, now_mono: float) -> None:
    try:
      from openpilot.cereal import log
      limit = 2 ** 64 - 1
      with log.Event.from_bytes(raw, traversal_limit_in_words=limit) as reader:
        if reader.which() != svc:
          # wrong service on this port: never republish
          self.dropped_error += 1
          return
        msg = getattr(reader, svc)
        sof = sof_for_service(msg, self.sof_by_frame)
        if sof is None:
          self.dropped_no_sof += 1
          return
        age_ns = now_ns - sof
        if age_ns > self.cfg.stale_ms * 1_000_000:
          self.dropped_stale += 1
          return
        # fresh: single-pass re-stamp; only the root header scalar is written, payload untouched
        b = reader.as_builder()
        b.logMonoTime = int(now_ns)
        out = b.to_bytes()
    except Exception:
      self.dropped_error += 1
      return

    try:
      self.cereal.publish(svc, out)
    except Exception:
      self.dropped_error += 1
      return

    self.published += 1
    self.recent_republish.append(now_mono)
    self.last_republish_mono = now_mono

  def _watchdog(self, now_mono: float) -> None:
    """Publish nothing if no valid republish happened within gap_ms; log the gap edges."""
    if self.last_republish_mono is None:
      if not self.gap_open and (now_mono - self._start_mono) * 1000.0 >= self.cfg.gap_ms:
        self._open_gap(now_mono)
      return
    if (now_mono - self.last_republish_mono) * 1000.0 >= self.cfg.gap_ms:
      self._open_gap(now_mono)
    elif self.gap_open:
      self.gap_open = False
      self._event("gap_closed", now_mono)

  def _open_gap(self, now_mono: float) -> None:
    if not self.gap_open:
      self.gap_open = True
      self._event("gap_open", now_mono)

  # -- output ------------------------------------------------------------------
  def _event(self, kind: str, now_mono: float) -> None:
    print(json.dumps({
      "ts_mono": round(now_mono, 6),
      "event": kind,
      "gap_ms": self.cfg.gap_ms,
    }), flush=True)

  def stats(self, now_mono: float | None = None) -> dict:
    now_mono = time.monotonic() if now_mono is None else now_mono
    return {
      "ts_mono": round(now_mono, 6),
      "uptime_s": round(now_mono - self._start_mono, 3),
      "zmq_msgs": self.zmq_msgs,
      "published": self.published,
      "dropped_stale": self.dropped_stale,
      "dropped_no_sof": self.dropped_no_sof,
      "dropped_error": self.dropped_error,
      "sof_frames_tracked": len(self.sof_by_frame),
      "gap_open": self.gap_open,
    }

  def _emit_stats(self, now_mono: float) -> dict:
    st = self.stats(now_mono)
    line = json.dumps(st, separators=(",", ":"))
    print(line, flush=True)
    if self.cfg.bench:
      try:
        with open(self.cfg.bench_path, "a") as f:
          f.write(line + "\n")
      except OSError:
        pass
    return st

  # -- run loop ----------------------------------------------------------------
  def run(self, stats_period_s: float = 5.0) -> int:
    self.install_signal_handlers()
    self.open()
    self._event("offloadd_start", time.monotonic())
    last_stats = time.monotonic()
    try:
      while not self._stop:
        self.step()
        now_mono = time.monotonic()
        if now_mono - last_stats >= stats_period_s:
          self._emit_stats(now_mono)
          last_stats = now_mono
    finally:
      self._emit_stats(time.monotonic())
      self.close()
      self._event("offloadd_stop", time.monotonic())
    return 0


def main() -> int:
  ap = argparse.ArgumentParser(description="device-side offload return-path republisher")
  ap.add_argument("--offload-mode", default=os.getenv("OFFLOAD_MODE", "auto"),
                  choices=("auto", "shadow", "drive"),
                  help="auto = read OffloadMode param (production); shadow/drive = explicit enable (bench)")
  ap.add_argument("--connect-host", default=os.getenv("OFFLOAD_RETURN_HOST", "127.0.0.1"),
                  help="host where the Mac publishes RETURN_SERVICES")
  ap.add_argument("--bench", action="store_true", help="also append stats to a local /tmp jsonl")
  ap.add_argument("--bench-path", default="/tmp/offloadd_bench.jsonl")
  ap.add_argument("--stats-period", type=float, default=5.0)
  args = ap.parse_args()

  cfg = Config(bench=args.bench, bench_path=args.bench_path,
               connect_host=args.connect_host, offload_mode=args.offload_mode)

  # mode gate (step e)
  if args.offload_mode == "auto":
    try:
      from openpilot.common.params import Params
      params = Params()
    except Exception:
      params = None
    mode = read_offload_mode(params)
    if not enabled_mode(mode):
      print(json.dumps({"event": "disabled", "reason": "OffloadMode not in {shadow,drive}",
                        "offload_mode": mode}), flush=True)
      return 0
    cfg.offload_mode = mode or args.offload_mode
  else:
    cfg.offload_mode = args.offload_mode

  if not _CerealIO(cfg).ready():
    print(json.dumps({"event": "disabled", "reason": "cereal bindings unavailable",
                      "offload_mode": cfg.offload_mode}), flush=True)
    return 0

  return Offloadd(cfg).run(stats_period_s=args.stats_period)


if __name__ == "__main__":
  sys.exit(main())
