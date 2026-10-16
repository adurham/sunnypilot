#!/usr/bin/env python3
"""bench_resources_mac.py — macOS soak evidence sampler (WS-B Mac counterpart to the
device-side PSS sampler).

Why this exists: the soak / "flat memory" claim on device is backed by a per-PID PSS
sampler reading /proc. macOS has no /proc, so this samples the Mac-side equivalents
every N seconds and produces the *evidence* for a soak:

  * per-PID RSS + %CPU for a set of PIDs or process-name patterns (framebridge,
    vtdec, modeld_runner, replayd, python, ...), by default
  * system memory pressure via `memory_pressure` (pages free / system free %),
    with a `vm_stat` fallback when that command is unavailable
  * swap usage via `sysctl vm.swapusage`, plus vm_stat swap-in/out deltas
  * compressor occupancy (the macOS analogue of anonymous-page growth)
  * thermal pressure one-shot via `pmset -g therm` (non-sudo; best effort)

Output: one JSON object per sample (jsonl) + a start-vs-end summary with a
least-squares MB/hour trend per series and leak flags. stdlib + subprocess only.
Designed to bracket a run of `run_framebridge.sh` for soak/bench.

Usage:
  .venv/bin/python openpilot/offload/mac/bench_resources_mac.py \
      --interval 10 --duration 1800 --out openpilot/offload/mac/logs/soak.jsonl
  # then, while it samples, run the bridge under the caffeinate wrapper in another shell.

The summary flags a leak when a tracked RSS series rises faster than
--leak-mb-per-hour (default 5) with least-squares R^2 >= 0.5 AND a window of at
least --min-window-s (default 300); a short window cannot flag (it would only be
extrapolating normal churn). Exit code 0 always (the report is the artifact); 2 on
bad args.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import time

PAGE_SIZE_DEFAULT = 16384
# Default process-name patterns: the Mac frame path + its children + bench helpers.
DEFAULT_NAMES = ["framebridge", "vtdec", "modeld_runner", "replayd", "python"]
# Leak threshold: a tracked series rising faster than this (least-squares slope)
# with a leak-consistent shape is flagged. Conservative — steady-state soaks far below.
DEFAULT_LEAK_MB_PER_HOUR = 5.0
# Noise tolerance (MB) for "monotonic": consecutive changes smaller than this are
# treated as flat so page-cache flutter / a freed buffer do not defeat the flag.
MONOTONIC_EPS_MB = 0.5
# A growth rate measured over a window shorter than this is not trustworthy (a few
# MB of normal churn over seconds extrapolates to hundreds of MB/h). Below it we
# still report the rate but suppress the leak flag. Set with --min-window-s.
DEFAULT_MIN_WINDOW_S = 300.0
# R^2 of the least-squares fit above which a rising series counts as leak-consistent
# even if it is not strictly monotonic (slow leak + sampling noise).
TREND_R2 = 0.5


def _run(cmd: list[str], timeout: float = 5.0) -> str:
  """Run a command, return stdout ('' on any failure). Never raises."""
  try:
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    return r.stdout
  except Exception:
    return ""


def _parse_mb(ps_column: str) -> float:
  """Parse a `ps` byte-like column (rss in KB, or a 'M'/'G' suffix) to MB."""
  s = ps_column.strip().lower()
  m = re.match(r"^([0-9.]+)\s*([kmg]?)", s)
  if not m:
    return 0.0
  v = float(m.group(1))
  unit = m.group(2)
  if unit == "k":
    return v / 1024.0
  if unit == "m":
    return v
  if unit == "g":
    return v * 1024.0
  return v / 1024.0  # bare number => assume KB (ps rss)


# --------------------------------------------------------------------------- #
#  Process sampling (ps)
# --------------------------------------------------------------------------- #
def sample_processes(pids: list[int], name_pats: list[str]) -> dict:
  """One `ps` call: match by explicit PID set, else by regex on the command."""
  out = _run(["ps", "-axo", "pid=,ppid=,rss=,%cpu=,etime=,command="], timeout=4.0)
  pats = [p for p in name_pats if p]
  rx = re.compile("|".join(re.escape(p) for p in pats)) if pats else None
  want_pids = {int(p) for p in pids}
  procs = []
  for line in out.splitlines():
    parts = line.strip().split(None, 5)
    if len(parts) < 6:
      continue
    try:
      pid = int(parts[0])
      ppid = int(parts[1])
      cpu = float(parts[3])
    except ValueError:
      continue
    rss_mb = _parse_mb(parts[2])
    command = parts[5]
    matched = pid in want_pids or (rx is not None and rx.search(command) is not None)
    if not matched:
      continue
    procs.append({"pid": pid, "ppid": ppid, "rss_mb": round(rss_mb, 3), "cpu_pct": cpu,
                  "etime": parts[4], "command": command[:160]})
  total_rss = sum(p["rss_mb"] for p in procs)
  total_cpu = sum(p["cpu_pct"] for p in procs)
  return {"count": len(procs), "total_rss_mb": round(total_rss, 3),
          "total_cpu_pct": round(total_cpu, 2), "procs": procs}


# --------------------------------------------------------------------------- #
#  vm_stat (always present) + memory_pressure (preferred when present)
# --------------------------------------------------------------------------- #
def sample_vm_stat() -> dict:
  out = _run(["vm_stat"], timeout=3.0)
  page_size = PAGE_SIZE_DEFAULT
  m = re.search(r"page size of (\d+) bytes", out)
  if m:
    page_size = int(m.group(1))
  pages: dict[str, float] = {}
  for line in out.splitlines():
    mm = re.match(r"^([A-Za-z][^:]*):\s*([0-9.]+)\.?\s*$", line.strip())
    if mm:
      pages[mm.group(1).strip().lower().replace(" ", "_")] = float(mm.group(2))
  k = page_size / (1024.0 * 1024.0)  # pages -> MB

  def mb(key: str) -> float:
    return round(pages.get(key, 0.0) * k, 3)

  return {
    "page_size": page_size,
    "pages_free": int(pages.get("pages_free", 0)),
    "pages_active": int(pages.get("pages_active", 0)),
    "pages_inactive": int(pages.get("pages_inactive", 0)),
    "pages_wired": int(pages.get("pages_wired_down", 0)),
    "pages_compressor": int(pages.get("pages_occupied_by_compressor", 0)),
    "free_mb": mb("pages_free"),
    "active_mb": mb("pages_active"),
    "wired_mb": mb("pages_wired_down"),
    "compressor_mb": mb("pages_occupied_by_compressor"),
    "pageins": int(pages.get("pageins", 0)),
    "pageouts": int(pages.get("pageouts", 0)),
    "swapins": int(pages.get("swapins", 0)),
    "swapouts": int(pages.get("swapouts", 0)),
  }


def sample_memory_pressure() -> dict | None:
  """`memory_pressure` is non-sudo and present by default; None if unavailable."""
  if shutil.which("memory_pressure") is None:
    return None
  out = _run(["memory_pressure"], timeout=6.0)
  if not out:
    return None
  res: dict = {}
  m = re.search(r"System-wide memory free percentage:\s*(\d+)%", out)
  if m:
    res["system_free_pct"] = int(m.group(1))
  for pat, key in ((r"Pages free:\s*(\d+)", "pages_free"),
                   (r"Pages purgeable:\s*(\d+)", "pages_purgeable"),
                   (r"Pages wired down:\s*(\d+)", "pages_wired"),
                   (r"Pages used by compressor:\s*(\d+)", "pages_compressor"),
                   (r"Swapins:\s*(\d+)", "swapins"),
                   (r"Swapouts:\s*(\d+)", "swapouts"),
                   (r"Pageins:\s*(\d+)", "pageins"),
                   (r"Pageouts:\s*(\d+)", "pageouts")):
    mm = re.search(pat, out)
    if mm:
      res[key] = int(mm.group(1))
  return res or None


def sample_swapusage() -> dict:
  out = _run(["sysctl", "vm.swapusage"], timeout=3.0)
  res: dict = {}
  for key, pat in (("total_mb", r"total = ([0-9.]+)M"),
                   ("used_mb", r"used = ([0-9.]+)M"),
                   ("free_mb", r"free = ([0-9.]+)M")):
    m = re.search(pat, out)
    if m:
      res[key] = float(m.group(1))
  res["encrypted"] = "encrypted" in out
  return res


# --------------------------------------------------------------------------- #
#  Thermal (one-shot, non-sudo, best effort)
# --------------------------------------------------------------------------- #
def sample_thermal() -> dict:
  if shutil.which("pmset") is None:
    return {"available": False}
  out = _run(["pmset", "-g", "therm"], timeout=3.0)
  if not out:
    return {"available": False}
  res = {"available": True, "warning": None, "performance": None, "cpu_power": None}
  m = re.search(r"thermal warning level has been recorded:\s*(\S+)", out)
  if m:
    res["warning"] = m.group(1)
  m = re.search(r"performance warning level has been recorded:\s*(\S+)", out)
  if m:
    res["performance"] = m.group(1)
  m = re.search(r"CPU power status has been recorded:\s*(.+)", out)
  if m:
    res["cpu_power"] = m.group(1).strip()
  return res


# --------------------------------------------------------------------------- #
#  One sample
# --------------------------------------------------------------------------- #
def take_sample(pids: list[int], name_pats: list[str]) -> dict:
  t0 = time.perf_counter()
  mp = sample_memory_pressure()
  vm = sample_vm_stat()
  sample = {
    "mono": time.monotonic(),
    "wall": time.strftime("%Y-%m-%dT%H:%M:%S"),
    "memory_pressure_available": mp is not None,
    "processes": sample_processes(pids, name_pats),
    "vm_stat": vm,
    "memory_pressure": mp,
    "swapusage": sample_swapusage(),
    "thermal": sample_thermal(),
  }
  sample["sampler_ms"] = round((time.perf_counter() - t0) * 1000.0, 2)
  return sample


# --------------------------------------------------------------------------- #
#  Series / trend analysis
# --------------------------------------------------------------------------- #
def _proc_label(p: dict) -> str:
  """Stable, readable label for a process row."""
  cmd = p.get("command", "")
  toks = cmd.split()
  base = os.path.basename(toks[0]) if toks else "?"
  tail = toks[-1] if toks else base
  if base.startswith("python"):
    base = os.path.basename(tail) or "python"
  return f'pid{p["pid"]}:{os.path.basename(base)[:32]}'


def _rss_series(samples: list[dict]) -> dict[str, list[float]]:
  """(a) total RSS and (b) per-PID RSS over time, keyed by a stable label."""
  series: dict[str, list[float]] = {"__total__": []}
  for s in samples:
    procs = s["processes"]
    series["__total__"].append(procs["total_rss_mb"])
    for p in procs["procs"]:
      series.setdefault(_proc_label(p), []).append(p["rss_mb"])
  # a PID that appeared late: pad the front so trends stay aligned by index
  n = len(samples)
  for key, vals in series.items():
    if key == "__total__" or len(vals) == n:
      continue
    series[key] = [vals[0]] * (n - len(vals)) + vals
  return series


def analyse_series(name: str, vals: list[float], times_s: list[float],
                   leak_mb_per_hour: float, min_window_s: float) -> dict:
  """Deltas + least-squares MB/hour slope + leak flag for one series."""
  n = len(vals)
  if n < 2:
    return {"name": name, "n": n, "start_mb": vals[0] if vals else 0.0,
            "end_mb": vals[-1] if vals else 0.0, "delta_mb": 0.0,
            "slope_mb_per_hour": 0.0, "r2": 0.0, "monotonic": False,
            "window_s": 0.0, "leak_flagged": False}
  start, end = vals[0], vals[-1]
  delta = end - start
  window_s = times_s[-1] - times_s[0]
  # least-squares slope in MB/s -> MB/h (robust to sampling noise and to a
  # short-window extrapolation blowing the end-start delta out of proportion)
  tmean = sum(times_s) / n
  vmean = sum(vals) / n
  sxx = sum((t - tmean) ** 2 for t in times_s)
  sxy = sum((times_s[i] - tmean) * (vals[i] - vmean) for i in range(n))
  slope_per_s = (sxy / sxx) if sxx > 0 else 0.0
  slope_per_hour = slope_per_s * 3600.0
  # R^2 of the linear fit (how leak-consistent the shape is)
  sst = sum((v - vmean) ** 2 for v in vals)
  ssr = sum((vals[i] - (vmean + slope_per_s * (times_s[i] - tmean))) ** 2 for i in range(n))
  r2 = (1.0 - ssr / sst) if sst > 0 else 0.0
  # monotonic-with-flat-band, for the strict shape claim
  mono = True
  prev = vals[0]
  for v in vals[1:]:
    if v < prev - MONOTONIC_EPS_MB:
      mono = False
      break
    prev = max(prev, v) if v < prev else v
  rising = slope_per_hour > leak_mb_per_hour and r2 >= TREND_R2
  long_enough = window_s >= min_window_s
  return {
    "name": name, "n": n, "start_mb": round(start, 3), "end_mb": round(end, 3),
    "delta_mb": round(delta, 3), "slope_mb_per_hour": round(slope_per_hour, 3),
    "r2": round(max(0.0, min(1.0, r2)), 3), "window_s": round(window_s, 1),
    "monotonic": bool(mono and delta > MONOTONIC_EPS_MB),
    "leak_flagged": bool(rising and long_enough),
  }


def build_summary(samples: list[dict], leak_mb_per_hour: float,
                  min_window_s: float = DEFAULT_MIN_WINDOW_S) -> dict:
  if not samples:
    return {"n_samples": 0, "error": "no samples"}
  first, last = samples[0], samples[-1]
  hours = (last["mono"] - first["mono"]) / 3600.0
  times = [s["mono"] for s in samples]
  rss = _rss_series(samples)

  total_trend = analyse_series("__total__", rss["__total__"], times, leak_mb_per_hour, min_window_s)
  rss_trends = [analyse_series(k, v, times, leak_mb_per_hour, min_window_s) for k, v in rss.items()]
  rss_trends.sort(key=lambda d: (not d["leak_flagged"], -d["slope_mb_per_hour"]))

  def swap_used(s: dict) -> float:
    return s.get("swapusage", {}).get("used_mb", 0.0)

  def compressor(s: dict) -> float:
    return s.get("vm_stat", {}).get("compressor_mb", 0.0)

  def free_pct(s: dict) -> float:
    return float((s.get("memory_pressure") or {}).get("system_free_pct", 0.0))

  swap_series = [swap_used(s) for s in samples]
  comp_series = [compressor(s) for s in samples]
  free_series = [free_pct(s) for s in samples]

  swap_trend = analyse_series("swap_used_mb", swap_series, times, leak_mb_per_hour, min_window_s)
  comp_trend = analyse_series("compressor_mb", comp_series, times, leak_mb_per_hour, min_window_s)

  def delta(first_s: dict, last_s: dict, section: str, key: str) -> int:
    a = first_s.get(section, {}).get(key, 0) or 0
    b = last_s.get(section, {}).get(key, 0) or 0
    return int(b - a)

  swap_io = {
    "swapins_delta": delta(first, last, "memory_pressure", "swapins") or delta(first, last, "vm_stat", "swapins"),
    "swapouts_delta": delta(first, last, "memory_pressure", "swapouts") or delta(first, last, "vm_stat", "swapouts"),
    "pageins_delta": delta(first, last, "vm_stat", "pageins"),
    "pageouts_delta": delta(first, last, "vm_stat", "pageouts"),
  }

  flagged = [t["name"] for t in rss_trends if t["leak_flagged"]]
  if swap_trend["leak_flagged"]:
    flagged.append("swap_used_mb")
  if comp_trend["leak_flagged"]:
    flagged.append("compressor_mb")

  return {
    "n_samples": len(samples),
    "window_s": round(last["mono"] - first["mono"], 3),
    "window_hours": round(hours, 5),
    "leak_mb_per_hour_threshold": leak_mb_per_hour,
    "min_window_s": min_window_s,
    "trend_r2_threshold": TREND_R2,
    "thermal": last.get("thermal", {}),
    "memory_pressure_available": last.get("memory_pressure_available", False),
    "system_free_pct": {"start": free_series[0], "end": free_series[-1]},
    "rss_total": {
      "start_mb": first["processes"]["total_rss_mb"],
      "end_mb": last["processes"]["total_rss_mb"],
      "delta_mb": round(last["processes"]["total_rss_mb"] - first["processes"]["total_rss_mb"], 3),
      "slope_mb_per_hour": total_trend["slope_mb_per_hour"],
      "max_proc_count": max((s["processes"]["count"] for s in samples), default=0),
      "min_proc_count": min((s["processes"]["count"] for s in samples), default=0),
    },
    "swap_used_mb": {"start_mb": swap_series[0], "end_mb": swap_series[-1],
                     "delta_mb": round(swap_series[-1] - swap_series[0], 3),
                     "slope_mb_per_hour": swap_trend["slope_mb_per_hour"]},
    "compressor_mb": {"start_mb": comp_series[0], "end_mb": comp_series[-1],
                      "delta_mb": round(comp_series[-1] - comp_series[0], 3),
                      "slope_mb_per_hour": comp_trend["slope_mb_per_hour"]},
    "swap_io": swap_io,
    "process_trends": rss_trends,
    "leak_flagged": bool(flagged),
    "flagged_series": flagged,
    "sampler_ms": {"min": min((s["sampler_ms"] for s in samples), default=0.0),
                   "max": max((s["sampler_ms"] for s in samples), default=0.0)},
  }


def print_summary(summary: dict) -> None:
  print("\n=== bench_resources_mac summary ===")
  mp = "available" if summary["memory_pressure_available"] else "MISSING (vm_stat fallback)"
  print(f"samples={summary['n_samples']} window={summary['window_s']:.0f}s ({summary['window_hours'] * 60:.2f} min) memory_pressure={mp}")
  rt = summary["rss_total"]
  print(f"RSS total: {rt['start_mb']:.1f} -> {rt['end_mb']:.1f} MB (delta {rt['delta_mb']:+.1f} MB, {rt['slope_mb_per_hour']:+.2f} MB/h)")
  print(f"proc count {rt['min_proc_count']}..{rt['max_proc_count']}")
  sw = summary["swap_used_mb"]
  print(f"swap used: {sw['start_mb']:.1f} -> {sw['end_mb']:.1f} MB ({sw['delta_mb']:+.1f} MB, {sw['slope_mb_per_hour']:+.2f} MB/h)")
  cm = summary["compressor_mb"]
  print(f"compressor: {cm['start_mb']:.1f} -> {cm['end_mb']:.1f} MB ({cm['delta_mb']:+.1f} MB)")
  sio = summary["swap_io"]
  print(f"swap I/O deltas: swapins={sio['swapins_delta']} swapouts={sio['swapouts_delta']} pageins={sio['pageins_delta']} pageouts={sio['pageouts_delta']}")
  th = summary["thermal"]
  print(f"thermal: warning={th.get('warning')} performance={th.get('performance')}")
  print(f"\n{'series':<34} {'start':>9} {'end':>9} {'delta':>9} {'MB/h':>9} {'r2':>6}  flags")
  for t in summary["process_trends"]:
    flag = "LEAK" if t["leak_flagged"] else ("mono" if t["monotonic"] else "")
    print(f"{t['name'][:34]:<34} {t['start_mb']:>9.1f} {t['end_mb']:>9.1f} {t['delta_mb']:>+9.1f} {t['slope_mb_per_hour']:>+9.2f} {t['r2']:>6.3f}  {flag}")
  if summary["leak_flagged"]:
    print(f"\n** LEAK FLAGGED: {', '.join(summary['flagged_series'])} (slope>{summary['leak_mb_per_hour_threshold']} MB/h, r2>={TREND_R2}) **")
  else:
    print(f"\nno leak flagged (threshold {summary['leak_mb_per_hour_threshold']} MB/h) — flat.")


# --------------------------------------------------------------------------- #
def build_argparser() -> argparse.ArgumentParser:
  p = argparse.ArgumentParser(description="macOS soak/resource sampler (WS-B)")
  p.add_argument("--interval", type=float, default=10.0, help="seconds between samples (default 10)")
  p.add_argument("--duration", type=float, default=0.0, help="seconds to sample; 0 = until Ctrl-C")
  p.add_argument("--pids", default="", help="comma-separated explicit PIDs to always track")
  p.add_argument("--names", default=",".join(DEFAULT_NAMES), help="comma-separated process-name substrings to track")
  p.add_argument("--out", default=None, help="jsonl path for raw samples")
  p.add_argument("--summary-out", default=None, help="json path for the summary (default <out>.summary.json)")
  p.add_argument("--leak-mb-per-hour", type=float, default=DEFAULT_LEAK_MB_PER_HOUR, help=f"leak threshold MB/h (default {DEFAULT_LEAK_MB_PER_HOUR})")
  p.add_argument("--min-window-s", type=float, default=DEFAULT_MIN_WINDOW_S, help=f"minimum window to flag (default {DEFAULT_MIN_WINDOW_S:.0f}s)")
  p.add_argument("--label", default=None, help="free-form tag stored on every sample")
  return p


def main(argv=None) -> int:
  args = build_argparser().parse_args(argv)
  if args.interval <= 0:
    print("--interval must be > 0", file=sys.stderr)
    return 2
  pids = [int(x) for x in args.pids.split(",") if x.strip().isdigit()] if args.pids else []
  names = [n.strip() for n in args.names.split(",") if n.strip()]

  out_fp = None
  if args.out:
    os.makedirs(os.path.dirname(os.path.abspath(args.out)) or ".", exist_ok=True)
    out_fp = open(args.out, "w", buffering=1)

  print(f"[bench_resources_mac] interval={args.interval}s duration={args.duration or 'inf'}s names={names} pids={pids} out={args.out or '(stdout only)'}")
  samples: list[dict] = []
  t_start = time.monotonic()
  try:
    while True:
      s = take_sample(pids, names)
      if args.label:
        s["label"] = args.label
      samples.append(s)
      if out_fp is not None:
        out_fp.write(json.dumps(s, separators=(",", ":")) + "\n")
      p = s["processes"]
      stamp = time.strftime("%H:%M:%S")
      sw_mb = s["swapusage"].get("used_mb", 0.0)
      print(f"[{stamp}] procs={p['count']} rss={p['total_rss_mb']:.1f}MB cpu={p['total_cpu_pct']:.1f}% swap={sw_mb:.1f}MB", flush=True)
      if args.duration > 0 and (time.monotonic() - t_start) >= args.duration:
        break
      elapsed = time.monotonic() - s["mono"]
      time.sleep(max(0.0, args.interval - elapsed))
  except KeyboardInterrupt:
    print("\n[bench_resources_mac] interrupted — summarising what we have")

  summary = build_summary(samples, args.leak_mb_per_hour, args.min_window_s)
  print_summary(summary)
  if args.summary_out or args.out:
    path = args.summary_out or (args.out + ".summary.json")
    try:
      with open(path, "w") as f:
        json.dump(summary, f, indent=2)
      print(f"\nsummary: {path}")
    except Exception as e:
      print(f"could not write summary: {e}", file=sys.stderr)
  if out_fp is not None:
    out_fp.close()
  return 0


if __name__ == "__main__":
  sys.exit(main())
