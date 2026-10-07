#!/usr/bin/env python3
"""offload bench: a tiny TCP/ZMQ echo server AND a matching latency client.

Runs on the comma device (system python 3.12, stdlib + pyzmq) and on the Mac (repo venv), so the
same file measures a round trip in either direction. 06_latency_roundtrip.sh scp's this to the
device's /tmp and drives it; it is also useful by hand.

Server modes (blocking; SIGTERM/SIGINT exit 0):
  echo_server.py --tcp 5599            # TCP echo: read, send the same bytes straight back
  echo_server.py --zmq 5599            # ZMQ REP echo: recv one frame, send it back

Client mode (drives N round trips per size, prints p50/p95/p99/p99.9/max in ms):
  echo_server.py --client --proto tcp --host 169.254.x.y --port 5599 \
                 --sizes 256,8192,8324 --n 2000 --warmup 50

No writes except the jsonl the client prints to stdout (bench tool; not part of offloadd).
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import sys
import time


def _stop(*_: object) -> None:
  raise SystemExit(0)


# --- servers -----------------------------------------------------------------

def serve_tcp(port: int) -> int:
  """Streaming TCP echo: whatever arrives goes straight back. One connection at a time is
  enough for a latency bench; each client is handled to completion."""
  srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
  srv.bind(("0.0.0.0", port))
  srv.listen(1)
  print(json.dumps({"event": "listening", "proto": "tcp", "port": port}), flush=True)
  while True:
    conn, addr = srv.accept()
    try:
      conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
      while True:
        data = conn.recv(65536)
        if not data:
          break
        conn.sendall(data)
    except OSError:
      pass
    finally:
      conn.close()


def serve_zmq(port: int) -> int:
  import zmq
  ctx = zmq.Context()
  sock = ctx.socket(zmq.REP)
  sock.bind(f"tcp://*:{port}")
  print(json.dumps({"event": "listening", "proto": "zmq", "port": port}), flush=True)
  while True:
    frame = sock.recv()
    sock.send(frame)


# --- client ------------------------------------------------------------------

def _round_trips(host: str, port: int, payload: int, n: int, warmup: int) -> list[float]:
  """One fresh TCP connection; N request/response round trips; returns per-trip ms.

  Request: send exactly ``payload`` bytes. Response: read exactly ``payload`` bytes back.
  A single byte of framing is not needed: the count is known on both sides."""
  body = os.urandom(payload)
  out: list[float] = []
  s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
  s.connect((host, port))
  try:
    for i in range(warmup + n):
      t0 = time.perf_counter()
      s.sendall(body)
      got = 0
      while got < payload:
        chunk = s.recv(min(65536, payload - got))
        if not chunk:
          raise ConnectionError("echo server closed early")
        got += len(chunk)
      dt = (time.perf_counter() - t0) * 1e3
      if i >= warmup:
        out.append(dt)
  finally:
    s.close()
  return out


def _pct(vals: list[float], p: float) -> float:
  if not vals:
    return float("nan")
  s = sorted(vals)
  return s[min(len(s) - 1, int(round((p / 100.0) * (len(s) - 1))))]


def client(args) -> int:
  sizes = [int(x) for x in args.sizes.split(",") if x.strip()]
  for size in sizes:
    trips = _round_trips(args.host, args.port, size, args.n, args.warmup)
    print(json.dumps({
      "proto": args.proto, "host": args.host, "port": args.port, "bytes": size,
      "n": len(trips),
      "p50_ms": round(_pct(trips, 50), 3), "p95_ms": round(_pct(trips, 95), 3),
      "p99_ms": round(_pct(trips, 99), 3), "p99.9_ms": round(_pct(trips, 99.9), 3),
      "max_ms": round(max(trips), 3), "mean_ms": round(sum(trips) / len(trips), 3),
    }), flush=True)
  return 0


def main() -> int:
  ap = argparse.ArgumentParser(description="offload bench echo server / latency client")
  ap.add_argument("--tcp", type=int, help="serve a TCP echo on this port")
  ap.add_argument("--zmq", type=int, help="serve a ZMQ REP echo on this port")
  ap.add_argument("--client", action="store_true", help="drive round trips and print percentiles")
  ap.add_argument("--proto", choices=("tcp",), default="tcp")
  ap.add_argument("--host", default="127.0.0.1")
  ap.add_argument("--port", type=int, default=5599)
  ap.add_argument("--sizes", default="256,8192,8324", help="comma-separated payload sizes (bytes)")
  ap.add_argument("--n", type=int, default=2000, help="round trips per size")
  ap.add_argument("--warmup", type=int, default=50)
  args = ap.parse_args()

  signal.signal(signal.SIGTERM, _stop)
  signal.signal(signal.SIGINT, _stop)
  if args.tcp:
    return serve_tcp(args.tcp)
  if args.zmq:
    return serve_zmq(args.zmq)
  if args.client:
    return client(args)
  ap.error("choose --tcp PORT, --zmq PORT, or --client")


if __name__ == "__main__":
  sys.exit(main())
