#!/usr/bin/env python3
"""openpilot.offload.mac.returnsend — MAC-side RETURN-path sender (WS-B).

The missing half of the return loop (INTERFACES.md §1, §7). The Mac-local modeld_v2 publishes
its outputs (``contract.RETURN_SERVICES``) into the **Mac-local** msgq under the REAL service
names. This sender subscribes there with a RAW msgq socket, and republishes the exact same bytes
over ZMQ to the device, where ``openpilot.offload.device.offloadd`` receives them, freshness-gates
them against the DEVICE clock, re-stamps the cereal header ``logMonoTime``, and forwards them
under the SHADOW service names (``contract.SHADOW_OUTPUT_SERVICES``) for the modeld_v2
arbitration.

Wire semantics (normative; see contract.py docstring and INTERFACES.md):
  1. ZMQ carries raw cereal Event bytes exactly as the Mac-local msgq emitted them. The payload
     is never touched and the header ``logMonoTime`` is NOT re-stamped on the wire — offloadd does
     the receipt re-stamp device-side (device monotonic now). This sender is a pipe.
  2. Ports come from ``openpilot.offload.ports`` (frozen scheme). The Mac BINDS one ZMQ PUB per
     service on ``ports.get_port(name)``; offloadd CONNECTs a SUB to the same port. One PUB per
     service mirrors the device bridge exactly.
  3. There is no clock sync anywhere. offloadd computes age device-side as
     ``now_device - timestamp_sof``. This sender's only clock duty is a self-imposed freshness
     gate on the MAC-local ``logMonoTime``: a message already older than ``OFFLOAD_SEND_MAX_AGE_MS``
     (default 150 ms) when this process reads it is dropped here so a stalled consumer can never
     replay a backlog into offloadd later.

Failure semantics (never block the loop):
  * No SUB connected yet, or the PUB high-water mark hit: the message is counted
    (``dropped_noreader``) and skipped — never blocked on. Reader presence is tracked with a ZMQ
    socket monitor (ACCEPTED/DISCONNECTED); a HWM back-pressure (``zmq.Again`` on a DONTWAIT send)
    is folded into the same counter, because in both cases the message did not reach the device.
  * Un-parseable bytes (no readable ``logMonoTime``) are dropped and counted as ``dropped_age``
    (fail safe: an un-ageable message is never forwarded blind).

Output: JSON-per-line to stdout only (``--stats`` for periodic counters; a final line on exit).
No ``/data`` writes. SIGINT/SIGTERM -> stop, close sockets, exit 0.

Typical use (bench, one machine):

  PYTHONPATH=$PWD:$PWD/opendbc_repo:$PWD/msgq_repo:$PWD/tinygrad_repo \\
    .venv/bin/python -m openpilot.offload.mac.returnsend --bind 127.0.0.1

See ``mac/RUNBOOK.md`` "Return path (Mac -> device)" and ``mac/return_path_test.sh``.
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import struct
import sys
import time

import zmq

from openpilot.offload import ports
from openpilot.offload.contract import RETURN_SERVICES

# --- tunables (env-overridable; CLI wins) -------------------------------------------
DEFAULT_MAX_AGE_MS = 150        # OFFLOAD_SEND_MAX_AGE_MS — Mac-local logMonoTime age self-drop
DEFAULT_POLL_TIMEOUT_MS = 20    # msgq poller timeout, so SIGTERM is serviced promptly
DEFAULT_SNDHWM = 200            # PUB high-water mark (frames); a backlog beyond this is dropped


def _env_ms(name: str, default: int) -> int:
  """Read an integer-ms env var, tolerating garbage."""
  raw = os.getenv(name)
  if raw is None or raw == "":
    return default
  try:
    return int(float(raw))
  except ValueError:
    return default


class Config:
  """Sender configuration. ``max_age_ms=None`` -> take the env default."""

  def __init__(self, bind: str = "0.0.0.0", services=None, max_age_ms: int | None = None,
               sub_addr: str = "127.0.0.1", poll_timeout_ms: int = DEFAULT_POLL_TIMEOUT_MS,
               stats: bool = False, stats_period: float = 5.0, quiet: bool = False,
               endpoint_for=None):
    self.bind = bind
    self.services = list(services) if services else list(RETURN_SERVICES)
    self.max_age_ms = _env_ms("OFFLOAD_SEND_MAX_AGE_MS", DEFAULT_MAX_AGE_MS) if max_age_ms is None else int(max_age_ms)
    self.sub_addr = sub_addr
    self.poll_timeout_ms = poll_timeout_ms
    self.stats = stats
    self.stats_period = stats_period
    self.quiet = quiet
    # callable(service)->endpoint overriding the frozen ports scheme (tests inject free ports);
    # production leaves it None so every PUB lands on ports.get_port(name), exactly as offloadd
    # expects. Mirrors offloadd.Config.endpoint_for.
    self.endpoint_for = endpoint_for


class ReturnSender:
  """Subscribe the Mac-local msgq for RETURN_SERVICES and republish raw bytes over ZMQ.

  Construct, then ``open()`` + ``step()`` (tests) or ``run()`` (blocking, signal-handled).
  """

  def __init__(self, cfg: Config | None = None):
    self.cfg = cfg or Config()
    # counters (the four the mission names, plus a misroute/error bucket)
    self.received = 0
    self.forwarded = 0
    self.dropped_age = 0
    self.dropped_noreader = 0
    self.dropped_error = 0
    self._stop = False
    self._poller = None
    self._subs: dict[str, object] = {}
    self._ctx = None
    self._pub: dict[str, zmq.Socket] = {}
    self._mon: dict[str, zmq.Socket] = {}
    self._readers: dict[str, int] = dict.fromkeys(self.cfg.services, 0)
    self._log_from_bytes = None
    self._start_mono = time.monotonic()

  # -- lifecycle ---------------------------------------------------------------
  def request_stop(self, *_: object) -> None:
    self._stop = True

  def install_signal_handlers(self) -> None:
    signal.signal(signal.SIGTERM, self.request_stop)
    signal.signal(signal.SIGINT, self.request_stop)

  def open(self) -> None:
    """Open the Mac-local msgq SUBs (raw) and bind one ZMQ PUB per service."""
    import msgq
    from openpilot.cereal import messaging

    if not self.cfg.services:
      raise RuntimeError("no services to send")
    self._log_from_bytes = messaging.log_from_bytes
    self._poller = msgq.Poller()
    for svc in self.cfg.services:
      # raw msgq subscribe on the Mac-local topic; receive() returns the raw Event bytes.
      self._subs[svc] = messaging.sub_sock(svc, poller=self._poller, addr=self.cfg.sub_addr,
                                           timeout=self.cfg.poll_timeout_ms)
    self._ctx = zmq.Context()
    for svc in self.cfg.services:
      s = self._ctx.socket(zmq.PUB)
      s.setsockopt(zmq.LINGER, 0)
      s.setsockopt(zmq.SNDHWM, DEFAULT_SNDHWM)
      endpoint = (self.cfg.endpoint_for(svc) if callable(self.cfg.endpoint_for)
                  else f"tcp://{self.cfg.bind}:{ports.get_port(svc)}")
      s.bind(endpoint)
      # track whether any SUB (offloadd) is connected, so a no-reader send is counted, not
      # silently swallowed by PUB/SUB.
      s.monitor(f"inproc://returnsend-mon-{svc}-{os.getpid()}",
                zmq.EVENT_ACCEPTED | zmq.EVENT_DISCONNECTED)
      self._pub[svc] = s
      self._mon[svc] = s.get_monitor_socket()

  def close(self) -> None:
    for mon in self._mon.values():
      try:
        mon.close(0)
      except Exception:
        pass
    self._mon.clear()
    for s in self._pub.values():
      try:
        s.close(0)
      except Exception:
        pass
    self._pub.clear()
    if self._ctx is not None:
      try:
        self._ctx.term()
      except Exception:
        pass
      self._ctx = None

  # -- core step ---------------------------------------------------------------
  def _pump_monitors(self) -> None:
    """Update per-service connected-reader counts from the ZMQ monitors (non-blocking)."""
    for svc, mon in self._mon.items():
      while True:
        try:
          if not mon.poll(0):
            break
          frames = mon.recv_multipart()
        except zmq.ZMQError:
          break
        if not frames:
          continue
        try:
          evt = struct.unpack("<H", frames[0][:2])[0]
        except struct.error:
          continue
        if evt & zmq.EVENT_ACCEPTED:
          self._readers[svc] = self._readers.get(svc, 0) + 1
        elif evt & zmq.EVENT_DISCONNECTED:
          self._readers[svc] = max(0, self._readers.get(svc, 0) - 1)

  def step(self) -> None:
    """One iteration: pump reader monitors, drain the msgq, age-gate + forward each message."""
    self._pump_monitors()
    for sock in self._poller.poll(self.cfg.poll_timeout_ms):
      while True:
        raw = sock.receive(non_blocking=True)
        if raw is None:
          break
        self._handle(bytes(raw))

  def _handle(self, raw: bytes) -> None:
    """Age-gate and forward one raw Event; count and drop instead of ever blocking."""
    self.received += 1
    try:
      reader = self._log_from_bytes(raw)
      which = reader.which()
      log_mono_ns = int(reader.logMonoTime)
    except Exception:
      # un-ageable bytes: never forward blind (fail safe)
      self.dropped_age += 1
      return
    if which not in self._pub:
      # a topic we did not subscribe/port for: never forward
      self.dropped_error += 1
      return
    # freshness self-drop on the MAC-local header (offloadd re-stamps at receipt)
    age_ns = time.monotonic_ns() - log_mono_ns
    if age_ns > self.cfg.max_age_ms * 1_000_000:
      self.dropped_age += 1
      return
    if self._readers.get(which, 0) <= 0:
      # no offloadd SUB connected yet: count, keep the loop free
      self.dropped_noreader += 1
      return
    try:
      self._pub[which].send(raw, zmq.DONTWAIT)   # raw bytes as-is; never blocks
    except zmq.Again:
      # HWM back-pressure: the message did not reach the device
      self.dropped_noreader += 1
      return
    except Exception:
      self.dropped_error += 1
      return
    self.forwarded += 1

  # -- output ------------------------------------------------------------------
  def stats(self) -> dict:
    return {
      "event": "returnsend_stats",
      "ts_mono": round(time.monotonic(), 6),
      "uptime_s": round(time.monotonic() - self._start_mono, 3),
      "bind": self.cfg.bind,
      "max_age_ms": self.cfg.max_age_ms,
      "services": list(self.cfg.services),
      "received": self.received,
      "forwarded": self.forwarded,
      "dropped_age": self.dropped_age,
      "dropped_noreader": self.dropped_noreader,
      "dropped_error": self.dropped_error,
      "readers": dict(self._readers),
      "ports": {svc: ports.get_port(svc) for svc in self.cfg.services},
    }

  def _emit(self, line: dict) -> None:
    if not self.cfg.quiet:
      print(json.dumps(line), flush=True)

  # -- run loop ----------------------------------------------------------------
  def run(self) -> int:
    self.install_signal_handlers()
    self.open()
    self._emit({"event": "returnsend_start", "bind": self.cfg.bind,
                "services": list(self.cfg.services), "max_age_ms": self.cfg.max_age_ms,
                "ports": {svc: ports.get_port(svc) for svc in self.cfg.services}})
    last_stats = time.monotonic()
    try:
      while not self._stop:
        self.step()
        now = time.monotonic()
        if self.cfg.stats and now - last_stats >= self.cfg.stats_period:
          self._emit(self.stats())
          last_stats = now
    finally:
      self._emit(self.stats())
      self.close()
      self._emit({"event": "returnsend_stop", "forwarded": self.forwarded})
    return 0


def _parse_services(raw: str | None) -> list[str]:
  if not raw:
    return list(RETURN_SERVICES)
  out = [s.strip() for s in raw.split(",") if s.strip()]
  unknown = [s for s in out if s not in RETURN_SERVICES]
  if unknown:
    raise SystemExit(f"unknown service(s) {unknown}; known: {RETURN_SERVICES}")
  return out


def main() -> int:
  ap = argparse.ArgumentParser(description="Mac-side RETURN-path sender (msgq -> ZMQ for offloadd)")
  ap.add_argument("--bind", default=os.getenv("OFFLOAD_RETURN_BIND", "0.0.0.0"),
                  help="interface to bind the ZMQ PUBs on (0.0.0.0 = all, tether-reachable; 127.0.0.1 = same-machine test)")
  ap.add_argument("--services", default=None,
                  help=f"comma-separated subset of {RETURN_SERVICES} (default: all)")
  ap.add_argument("--max-age-ms", type=int, default=None,
                  help=f"drop Mac-local msgs older than this (default OFFLOAD_SEND_MAX_AGE_MS={DEFAULT_MAX_AGE_MS})")
  ap.add_argument("--sub-addr", default="127.0.0.1",
                  help="Mac-local msgq address to subscribe (msgq transport only supports 127.0.0.1)")
  ap.add_argument("--poll-timeout-ms", type=int, default=DEFAULT_POLL_TIMEOUT_MS)
  ap.add_argument("--stats", action="store_true", help="emit periodic JSON counter lines to stdout")
  ap.add_argument("--stats-period", type=float, default=5.0)
  ap.add_argument("--quiet", action="store_true", help="suppress all stdout")
  args = ap.parse_args()

  cfg = Config(bind=args.bind, services=_parse_services(args.services), max_age_ms=args.max_age_ms,
               sub_addr=args.sub_addr, poll_timeout_ms=args.poll_timeout_ms,
               stats=args.stats, stats_period=args.stats_period, quiet=args.quiet)
  return ReturnSender(cfg).run()


if __name__ == "__main__":
  sys.exit(main())
