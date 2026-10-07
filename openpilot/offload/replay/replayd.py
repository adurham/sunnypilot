#!/usr/bin/env python3
"""WS-C replayd — publish a recorded route over LOCAL ZMQ as the device bridge would.

Emulates the WIRE (INTERFACES.md §1): each service is published as raw cereal
Event bytes on tcp://<host>:<port>, port = openpilot.offload.ports.get_port(name).
Header `logMonoTime` is the DEVICE clock — nothing is re-stamped here (the
Mac-local re-stamp happens later, in WS-B's framebridge, per §1.2).

Published services:

  narrowRoadEncodeData / wideRoadEncodeData   EncodeData built from the fixture:
        idx.frameId/encodeId/timestampSof/timestampEof/flags from the .enc record,
        data = AU bytes, header = parameter sets (first record only), 1928x1208.
  narrowRoadCameraState / wideRoadCameraState  verbatim from the rlog, at 20 Hz.
  carState, deviceState, carControl, extrinsicsCalibration,
  driverMonitoringState, lateralDelay         verbatim from the rlog, native rates.

Pacing is clock-independent (§1.4): wall target = base + (dev_ns - dev0)/1e9/speed,
using payload SOF for EncodeData and the recorded logMonoTime for the rest.

Usage (from the worktree root):
  PYTHONPATH=$PWD:$PWD/opendbc_repo:$PWD/msgq_repo:$PWD/tinygrad_repo \\
    .venv/bin/python -m openpilot.offload.replay.replayd <route> \\
      [--fixture-dir DIR] [--host 127.0.0.1] [--speed 1.0|max] [--jitter-ms X] \\
      [--loop] [--start S --duration D] [--pub-log out.jsonl] [--stats]

Exit: 0 clean, 2 usage / fixture / IO error. Ctrl-C => 0.
"""
from __future__ import annotations

import argparse
import json
import os
import random
import signal
import sys
import time

import zmq

from openpilot.cereal import log as capnp_log
from openpilot.offload.ports import get_port
from openpilot.offload.replay import fixture as fx
from openpilot.tools.lib.logreader import LogReader

DEFAULT_FIXTURE_DIR = os.path.expanduser("~/.hermes/cache/scratch/car-features/offload/replay-fixtures")

# Small services replayed verbatim from the rlog (contract FORWARD_SMALL_SERVICES
# minus the two cameraState services, which are also replayed verbatim).
SMALL_SERVICES = ["narrowRoadCameraState", "wideRoadCameraState", "carState", "deviceState",
                  "carControl", "extrinsicsCalibration", "driverMonitoringState", "lateralDelay"]

_STOP = False

# The device sets EncodeData.header (VPS/SPS/PPS) on keyframes. Republishing it
# is wire-faithful, but WS-B's vtdec rejects the in-band header block on this
# build (error -17694), dropping every frame; the consumer already owns the
# parameter sets (--params-dir / .header). Default OFF — set
# OFFLOAD_REPLAY_SEND_HEADER=1 to send it as the device would.
_SEND_HEADER = os.environ.get("OFFLOAD_REPLAY_SEND_HEADER") == "1"


def _on_sigint(_sig, _frm):
  global _STOP
  _STOP = True


class ZmqWire:
  """One ZMQ PUB socket per service, bound on the frozen port scheme."""

  def __init__(self, host: str, services: list[str], sndhwm: int = 1000):
    self.ctx = zmq.Context.instance()
    self.pub: dict[str, zmq.Socket] = {}
    self.host = host
    for svc in services:
      s = self.ctx.socket(zmq.PUB)
      s.setsockopt(zmq.SNDHWM, sndhwm)
      s.setsockopt(zmq.LINGER, 0)
      s.bind(f"tcp://{host}:{get_port(svc)}")
      self.pub[svc] = s

  def send(self, service: str, payload: bytes) -> None:
    self.pub[service].send(payload)

  def close(self) -> None:
    for s in self.pub.values():
      s.close(0)


def _load_small(route_dirs: list[str]):
  """Read every small service + cameraState verbatim from the rlog(s).

  Returns a list of (dev_ns, service, raw_bytes). Multiple route dirs (a
  concatenated fixture) are read in order.
  """
  out = []
  for route_dir in route_dirs:
    rlog = os.path.join(route_dir, "rlog.zst")
    if not os.path.exists(rlog):
      raise FileNotFoundError(f"{route_dir}: missing rlog.zst")
    for msg in LogReader(rlog):
      w = msg.which()
      if w not in SMALL_SERVICES:
        continue
      out.append((int(msg.logMonoTime), w, msg.as_builder().to_bytes()))
  return out


def _load_fixture_cam(fixture_dir: str, cam: str):
  enc = os.path.join(fixture_dir, fx.ENC_NAME[cam])
  if not os.path.exists(enc):
    raise FileNotFoundError(f"{fixture_dir}: missing {fx.ENC_NAME[cam]}")
  header = fx.read_header(os.path.join(fixture_dir, fx.HDR_NAME[cam]))
  recs = list(fx.read_records(enc))
  return header, recs


def build_timeline(route_dirs: list[str], fixture_dir: str):
  """Return a time-sorted list of wire events.

  Each event: dict(dev_ns, service, kind, raw, frame_id, encode_id, nbytes).
  Video events are built eagerly (raw bytes cached) — ~2.4 k events, fine.
  """
  events = []

  for cam in fx.CAMERAS:
    header, recs = _load_fixture_cam(fixture_dir, cam)
    svc = fx.ENCODE_DATA_SVC[cam]
    for i, rec in enumerate(recs):
      raw = _encode_data_msg_cam(rec, svc, header, include_header=(i == 0))
      events.append({"dev_ns": rec.sof, "service": svc, "kind": "video",
                     "raw": raw, "frame_id": rec.frame_id, "encode_id": rec.encode_id,
                     "nbytes": len(rec.au)})

  for dev_ns, svc, raw in _load_small(route_dirs):
    events.append({"dev_ns": dev_ns, "service": svc, "kind": "small",
                   "raw": raw, "frame_id": None, "encode_id": None, "nbytes": len(raw)})

  events.sort(key=lambda e: e["dev_ns"])
  return events


def _encode_data_msg_cam(rec: fx.FixtureRecord, svc: str, header: bytes, include_header: bool):
  evt = capnp_log.Event.new_message(valid=True, logMonoTime=int(rec.eof))
  ed = evt.init(svc)
  idx = ed.idx
  idx.frameId = rec.frame_id
  idx.encodeId = rec.encode_id
  idx.type = "fullHEVC"
  idx.timestampSof = rec.sof
  idx.timestampEof = rec.eof
  idx.flags = rec.flags
  idx.len = len(rec.au)
  ed.data = rec.au
  if include_header and header and _SEND_HEADER:
    ed.header = header
  ed.unixTimestampNanos = 0
  ed.width = fx.FRAME_W
  ed.height = fx.FRAME_H
  return evt.to_bytes()


def run(args) -> int:
  global _STOP
  route_arg = os.path.abspath(os.path.expanduser(args.route))
  route = os.path.basename(os.path.normpath(route_arg))
  fixture_dir = args.fixture_dir or os.path.join(DEFAULT_FIXTURE_DIR, route)
  if args.fixture_dir and not os.path.exists(os.path.join(args.fixture_dir, "meta.json")) \
      and os.path.exists(os.path.join(args.fixture_dir, route, "meta.json")):
    fixture_dir = os.path.join(args.fixture_dir, route)

  meta_path = os.path.join(fixture_dir, "meta.json")
  if not os.path.exists(meta_path):
    print(f"[replayd] ERROR: no fixture at {fixture_dir} "
          f"(run extractor first, or pass --fixture-dir)", file=sys.stderr)
    return 2

  try:
    meta = fx.read_meta(fixture_dir)
    route_dirs = meta.get("route_dirs")
    if not route_dirs:                       # single-dir fixture: prefer the CLI route if valid
      route_dirs = [route_arg if os.path.isdir(route_arg) else os.path.expanduser(
        f"~/comma-routes/{meta['route']}")]
    events = build_timeline(route_dirs, fixture_dir)
  except Exception as e:  # noqa: BLE001
    print(f"[replayd] ERROR building timeline: {e!r}", file=sys.stderr)
    return 2
  if not events:
    print("[replayd] ERROR: empty timeline", file=sys.stderr)
    return 2

  services = sorted({e["service"] for e in events})
  wire = ZmqWire(args.host, services)

  # --start / --duration crop against device time relative to the first event.
  dev0_full = events[0]["dev_ns"]
  if args.start or args.duration:
    s_ns = int(args.start * 1e9) if args.start else 0
    e_ns = int((args.start + args.duration) * 1e9) if args.duration else None
    lo = dev0_full + s_ns
    hi = (dev0_full + e_ns) if e_ns is not None else None
    events = [e for e in events if e["dev_ns"] >= lo and (hi is None or e["dev_ns"] < hi)]
  if not events:
    print("[replayd] ERROR: no events in selected window", file=sys.stderr)
    wire.close()
    return 2

  speed = args.speed
  jitter = max(0.0, args.jitter_ms) / 1000.0
  rng = random.Random(args.seed)

  def pub_log(f, e, mac_ns):
    if f is None:
      return
    f.write(json.dumps({"mac_ns": mac_ns, "dev_ns": e["dev_ns"], "service": e["service"],
                        "kind": e["kind"], "frame_id": e["frame_id"],
                        "encode_id": e["encode_id"], "nbytes": e["nbytes"]},
                       separators=(",", ":")) + "\n")

  plog = open(args.pub_log, "w") if args.pub_log else None
  sent = 0
  loops = 0
  t_start = time.monotonic()
  try:
    while not _STOP:
      dev0 = events[0]["dev_ns"]
      base = time.monotonic()
      for e in events:
        if _STOP:
          break
        if speed != "max":
          target = base + (e["dev_ns"] - dev0) / 1e9 / speed
          if jitter:
            target += rng.uniform(0.0, jitter)
          now = time.monotonic()
          if target > now:
            time.sleep(target - now)
        wire.send(e["service"], e["raw"])
        pub_log(plog, e, time.monotonic_ns())
        sent += 1
      loops += 1
      if plog:
        plog.flush()
      if not args.loop:
        break
    wall = time.monotonic() - t_start
  except KeyboardInterrupt:
    wall = time.monotonic() - t_start
  finally:
    if plog:
      plog.close()
    wire.close()

  dev_span = (events[-1]["dev_ns"] - events[0]["dev_ns"]) / 1e9
  if args.stats:
    n_vid = sum(1 for e in events if e["kind"] == "video")
    print(f"[replayd] route={route} events={len(events)} (video={n_vid} small={len(events)-n_vid}) "
          f"loops={loops} sent={sent} speed={speed} jitter_ms={args.jitter_ms}")
    print(f"[replayd] device span {dev_span:.2f}s  wall {wall:.2f}s  "
          f"ports {{{', '.join(f'{s}:{get_port(s)}' for s in services)}}}")
  return 0


def main(argv=None) -> int:
  ap = argparse.ArgumentParser(description="Replay a recorded route over local ZMQ (device-bridge wire emulation)")
  ap.add_argument("route", help="route directory (needs rlog.zst) or its basename under the fixture dir")
  ap.add_argument("--fixture-dir", default=None)
  ap.add_argument("--host", default="127.0.0.1")
  ap.add_argument("--speed", default="1.0", help="'1.0' real time, 'max' blast, or a float multiplier")
  ap.add_argument("--jitter-ms", type=float, default=0.0, help="add uniform [0,X] ms jitter per event")
  ap.add_argument("--loop", action="store_true")
  ap.add_argument("--start", type=float, default=0.0, help="skip the first S device seconds")
  ap.add_argument("--duration", type=float, default=None, help="play only D device seconds")
  ap.add_argument("--pub-log", default=None, help="write one jsonl row per publish")
  ap.add_argument("--seed", type=int, default=0)
  ap.add_argument("--stats", action="store_true")
  args = ap.parse_args(argv)
  if args.speed != "max":
    try:
      args.speed = float(args.speed)
      assert args.speed > 0
    except (ValueError, AssertionError):
      print("[replayd] ERROR: --speed must be 'max' or a positive float", file=sys.stderr)
      return 2
  signal.signal(signal.SIGINT, _on_sigint)
  signal.signal(signal.SIGTERM, _on_sigint)
  return run(args)


if __name__ == "__main__":
  sys.exit(main())
