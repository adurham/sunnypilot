#!/usr/bin/env python3
"""WS-A: steady-state latency bench for the FORK-flavor Metal supercombo pkl.

Method (INTERFACES.md §4 / task item 2):
  - load compiled pkl via load_oob(open_file_chunked(...))
  - rebuild queues via the fork run path's own helper (make_input_queues)
  - warm up >=15 runs, then measure >=100 runs
  - per run: t=perf_counter -> run_model(**queues) -> Device.default.synchronize() -> perf_counter
    records both enqueue (pre-sync) and wall (post-sync).
  - loadavg sampled by a background thread across the whole run.

Usage: PYTHONPATH=... python bench_fork.py <pkl_path> <out_dir> [n_warmup] [n_runs]
"""

import csv
import json
import os
import statistics
import sys
import threading
import time


from openpilot.common.file_chunker import open_file_chunked
from openpilot.sunnypilot.modeld_v2.helpers import load_oob
from openpilot.sunnypilot.modeld_v2.compile_modeld import nv12_copy_size, derive_frame_skip
from openpilot.selfdrive.modeld.compile_modeld import make_input_queues, MODELD_INPUTS
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
from tinygrad import Device


def pct(sorted_vals, q):
  if not sorted_vals:
    return float('nan')
  k = (len(sorted_vals) - 1) * q
  lo = int(k)
  hi = min(lo + 1, len(sorted_vals) - 1)
  return sorted_vals[lo] + (sorted_vals[hi] - sorted_vals[lo]) * (k - lo)


def main():
  pkl = sys.argv[1]
  outdir = sys.argv[2]
  n_warm = int(sys.argv[3]) if len(sys.argv) > 3 else 15
  n_runs = int(sys.argv[4]) if len(sys.argv) > 4 else 200

  samples = []
  stop = threading.Event()

  def sampler():
    while not stop.is_set():
      samples.append(os.getloadavg())
      time.sleep(2.0)

  t = threading.Thread(target=sampler, daemon=True)
  t.start()

  print(f"Device.DEFAULT={Device.DEFAULT}")
  print(f"loadavg before load: {os.getloadavg()}")
  st_load = time.perf_counter()
  jits = load_oob(open_file_chunked(pkl))
  load_s = time.perf_counter() - st_load

  md = jits['metadata']['model']
  input_shapes = md['input_shapes']
  frame_skip = derive_frame_skip({}, input_shapes)
  cam_w, cam_h = 1928, 1208
  nv12_info = get_nv12_info(cam_w, cam_h)
  frame_copy_size = nv12_copy_size(*nv12_info[:3])
  run_model = jits['run_model'][(cam_w, cam_h)]

  queues, npy, frame_views = make_input_queues(input_shapes, frame_skip, device=Device.DEFAULT, frame_copy_size=frame_copy_size)
  print(f"loaded in {load_s * 1e3:.1f} ms; frame_skip={frame_skip} frame_copy_size={frame_copy_size}")

  def do_run():
    st = time.perf_counter()
    outs = run_model(**{k: queues[k] for k in MODELD_INPUTS})
    mt = time.perf_counter()
    Device.default.synchronize()
    et = time.perf_counter()
    return outs, (mt - st) * 1e3, (et - st) * 1e3

  # warmup
  for _ in range(n_warm):
    do_run()
  Device.default.synchronize()
  print(f"warmup done ({n_warm} runs)")

  rows = []
  out_checksum = None
  for i in range(n_runs):
    outs, enq_ms, wall_ms = do_run()
    arr = outs[0].numpy().flatten() if isinstance(outs, tuple) else outs.numpy().flatten()
    if out_checksum is None:
      out_checksum = float(arr.sum())
    rows.append((i, enq_ms, wall_ms))

  stop.set()
  t.join(timeout=1)

  enq = sorted(r[1] for r in rows)
  wall = sorted(r[2] for r in rows)
  summary = {
    'pkl': pkl,
    'device': Device.DEFAULT,
    'n_warmup': n_warm,
    'n_runs': n_runs,
    'load_s': load_s,
    'output_sum': out_checksum,
    'enqueue_ms': {
      k: pct(enq, q) for k, q in [('p50', 0.5), ('p95', 0.95), ('p99', 0.99), ('p999', 0.999), ('max', 1.0), ('min', 0.0), ('mean', None)] if q is not None
    },
    'wall_ms': {
      k: pct(wall, q) for k, q in [('p50', 0.5), ('p95', 0.95), ('p99', 0.99), ('p999', 0.999), ('max', 1.0), ('min', 0.0), ('mean', None)] if q is not None
    },
    'loadavg_samples': samples,
    'loadavg_before': samples[0] if samples else None,
    'loadavg_after': samples[-1] if samples else None,
  }
  summary['enqueue_ms']['mean'] = statistics.fmean(enq)
  summary['wall_ms']['mean'] = statistics.fmean(wall)

  with open(os.path.join(outdir, 'latency_fork_raw.csv'), 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['run', 'enqueue_ms', 'wall_ms'])
    for r in rows:
      w.writerow([r[0], f"{r[1]:.6f}", f"{r[2]:.6f}"])

  with open(os.path.join(outdir, 'latency_fork_results.json'), 'w') as f:
    json.dump(summary, f, indent=2)

  print(json.dumps({k: {kk: round(vv, 4) for kk, vv in v.items()} for k, v in summary.items() if k in ('enqueue_ms', 'wall_ms')}, indent=2))
  print(f"loadavg first/last: {summary['loadavg_before']} / {summary['loadavg_after']}")


if __name__ == '__main__':
  main()
