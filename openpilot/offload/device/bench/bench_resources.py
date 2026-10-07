#!/usr/bin/env python3
"""offload bench: sample bench processes and VM state without work on the model frame thread.

Ported from jetlink/scripts/comma/bench_resources.py (the PSS/smaps sampler). Runs ON the comma
device (user-gated; AGNOS Ubuntu 24.04 / kernel 4.9.103), stdlib only.

Two facts from the reference that drive the code and must not be "cleaned up":
  * AGNOS 4.9 has NO /proc/<pid>/smaps_rollup (that is 4.14+), so PSS is summed by parsing every
    line of smaps that starts with 'Pss:' — see task(). smaps is large, so it is read on only
    every 10th tick, not every second.
  * /proc/<pid>/stat is split after the FINAL ')' because comm can contain spaces and ')'. A
    naive split() on the whole line breaks for a process named e.g. 'foo) bar'.
It also records, as the first line, what the tuned sysctls are right now, so a run proves whether
05_device_tuning.sh was applied (Jetlink did the same, reading the list off its root script).

Writes ONLY to stdout by default, or to --output (path it is given). Writes nothing to /data.

Usage (on the device):
  /usr/local/venv/bin/python3 bench_resources.py <pid> [<pid> ...]                 # jsonl to stdout
  /usr/local/venv/bin/python3 bench_resources.py --output /tmp/res.jsonl <pid> ...
  ssh comma ... 'python3 bench_resources.py $(pgrep -f modeld)' > res.jsonl        # from the Mac

Fields per sample line: monotonic, meminfo_kb (whole table), vmstat (allocstall*, compact_*,
pgscan_*, pgsteal_*, nr_dirty, nr_writeback), buddyinfo, processes{pid: {...}}, sampler_ms.
Every 10th sample also carries processes[pid].pss_kb.
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import time
from pathlib import Path

# The device-side knobs our 05_device_tuning.sh tunes (jetlink-root.sh:93, :127-128); their current
# values are recorded as the run's first line so a bench can prove whether tuning was applied.
FFS_LOG_OFF = Path("/sys/kernel/debug/ipc_logging/f_fs/log_disable")
SYSCTL_KEYS = (
  "vm.extra_free_kbytes",
  "vm.dirty_bytes",
  "vm.dirty_background_bytes",
  "vm.dirty_ratio",
  "vm.dirty_background_ratio",
  "net.core.wmem_max",
  "net.core.rmem_max",
)
# The record 05_device_tuning.sh writes the pre-apply originals to (shown if present).
SYSCTL_PREV = Path("/dev/shm/offload-sysctl-prev")


def read(path: Path) -> str:
  try:
    return path.read_text()
  except (OSError, UnicodeError):
    return ""


def counters(text: str) -> dict:
  """'/proc' key-number table -> {key: int}, e.g. meminfo or vmstat."""
  result = {}
  for line in text.splitlines():
    fields = line.replace(":", "").split()
    if len(fields) >= 2 and fields[1].isdigit():
      result[fields[0]] = int(fields[1])
  return result


def task(path: Path) -> dict | None:
  """Per-process sample. stat is split after the final ')' (comm-safe), so fields[0] is 'S'."""
  fields = read(path / "stat").rpartition(")")[2].split()
  if len(fields) < 22:
    return None
  status = counters(read(path / "status"))
  return {
    "user_ticks": int(fields[11]),
    "system_ticks": int(fields[12]),
    "minor_faults": int(fields[7]),
    "major_faults": int(fields[9]),
    "start_ticks": int(fields[19]),
    "rss_pages": int(fields[21]),
    "voluntary_switches": status.get("voluntary_ctxt_switches", 0),
    "involuntary_switches": status.get("nonvoluntary_ctxt_switches", 0),
    "schedstat": read(path / "schedstat").strip(),
  }


def pss_kb(path: Path) -> int | None:
  """PSS summed from smaps (no smaps_rollup on AGNOS 4.9). None if smaps is unreadable."""
  maps = read(path / "smaps")
  if not maps:
    return None
  return sum(int(line.split()[1]) for line in maps.splitlines() if line.startswith("Pss:"))


def sample(pids: list[int], proc: Path = Path("/proc"), pss: bool = False) -> dict:
  vm = counters(read(proc / "vmstat"))
  result = {
    "monotonic": time.monotonic(),
    "meminfo_kb": counters(read(proc / "meminfo")),
    "vmstat": {k: v for k, v in vm.items() if k.startswith(
      ("allocstall", "compact_", "pgscan_", "pgsteal_", "nr_dirty", "nr_writeback"))},
    "buddyinfo": read(proc / "buddyinfo"),
    "processes": {},
  }
  for pid in pids:
    path = proc / str(pid)
    stats = task(path)
    if stats is None:
      continue
    stats["threads"] = {p.name: task(p) for p in (path / "task").glob("[0-9]*")}
    if pss:
      stats["pss_kb"] = pss_kb(path)
    result["processes"][str(pid)] = stats
  return result


def sysctl_snapshot(proc_sys: Path = Path("/proc/sys")) -> dict:
  out = {k: read(proc_sys / k.replace(".", "/")).strip() for k in SYSCTL_KEYS}
  out["f_fs_log_disable"] = read(FFS_LOG_OFF).strip() or "<absent>"
  out["prev_record_present"] = SYSCTL_PREV.exists()
  return out


def metadata(pids: list[int], proc: Path = Path("/proc")) -> dict:
  return {
    "clock_ticks": os.sysconf("SC_CLK_TCK"),
    "page_bytes": os.sysconf("SC_PAGE_SIZE"),
    "commands": {str(pid): read(proc / str(pid) / "cmdline").replace("\0", " ").strip() for pid in pids},
    "sysctls": sysctl_snapshot(),
  }


def main() -> int:
  ap = argparse.ArgumentParser(description="offload device resource/PSS sampler")
  ap.add_argument("--output", type=Path, default=None, help="jsonl path; default stdout")
  ap.add_argument("--pss-every", type=int, default=10, help="read smaps every Nth tick (default 10)")
  ap.add_argument("--period", type=float, default=1.0, help="seconds between samples (default 1)")
  ap.add_argument("pids", type=int, nargs="+")
  args = ap.parse_args()

  stop = False

  def finish(*_: object) -> None:
    nonlocal stop
    stop = True

  signal.signal(signal.SIGINT, finish)
  signal.signal(signal.SIGTERM, finish)

  out = args.output.open("w") if args.output else sys.stdout
  try:
    out.write(json.dumps(metadata(args.pids)) + "\n")
    out.flush()
    tick = 0
    while not stop:
      started = time.monotonic()
      data = sample(args.pids, pss=(tick % max(1, args.pss_every) == 0))
      data["sampler_ms"] = (time.monotonic() - started) * 1000
      out.write(json.dumps(data) + "\n")
      out.flush()
      tick += 1
      time.sleep(max(0, args.period - (time.monotonic() - started)))
  finally:
    if args.output:
      out.close()
  return 0


if __name__ == "__main__":
  sys.exit(main())
