#!/usr/bin/env python3
"""WS-A: prove the OFFLOAD=1-patched modeld_v2 run path works OFFLINE.

Exercises the patched fork run path end to end without VisionIPC / msgq:
  * imports `openpilot.sunnypilot.modeld_v2.modeld` under OFFLOAD=1 on macOS
    (tolerant Params backend, no chestnut/AMD import, METAL device override)
  * loads the fork-flavor Metal pkl via load_oob(open_file_chunked(...))
  * builds queues with the run path's own helper (make_stock_input_queues)
  * drives ModelState.run() for N>=20 iterations from a minimal local stub frame
    source (WS-C `openpilot/offload/replay/` fixtures are not present yet)
  * checks outputs finite/nonzero and deterministic across two fresh runs (rel <= 1e-6)

Usage: OFFLOAD=1 OFFLOAD_VIPC_SERVER=offload_smoke python smoke_offline.py <pkl> <out_dir> [n_iters]
"""
import json
import os
import sys

import numpy as np

# OFFLOAD must be set before importing the modeld_v2 package.
if os.environ.get('OFFLOAD') != '1':
  raise SystemExit("run with OFFLOAD=1 (this script proves the OFFLOAD-patched path)")

from openpilot.sunnypilot.modeld_v2 import modeld as M
from openpilot.sunnypilot.modeld_v2.constants import ModelConstants

CAM_W, CAM_H = 1928, 1208


def build_inputs(model, rng):
  """Minimal deterministic stub for the inputs the run loop synthesizes each frame."""
  vec_desire = np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32)
  vec_desire[rng.integers(0, ModelConstants.DESIRE_LEN)] = 1
  inputs = {
    model.desire_key: vec_desire,
    'traffic_convention': np.array([0.0, 1.0], dtype=np.float32),
  }
  if 'action_t' in model.numpy_inputs:
    inputs['action_t'] = np.array([0.15, 0.25], dtype=np.float32)
  return inputs


def run_pass(pkl, n_iters, seed):
  os.environ['COMBINED_MODEL_PKL'] = pkl
  model = M.ModelState(cam_w=CAM_W, cam_h=CAM_H, chestnut=False)
  model.warmup()

  frame = np.arange(model.frame_copy_size, dtype=np.uint8) % 251
  bufs = {name: frame.tobytes() for name in model.vision_input_names}
  transforms = {name: np.eye(3, dtype=np.float32) for name in model.vision_input_names}

  rng = np.random.default_rng(seed)
  outputs = []
  for _ in range(n_iters):
    out = model.run(bufs, transforms, build_inputs(model, rng))
    key = 'plan' if 'plan' in out else sorted(out)[0]
    outputs.append(np.asarray(out[key], dtype=np.float64).ravel())
  return model, outputs


def main():
  pkl, outdir = sys.argv[1], sys.argv[2]
  n_iters = int(sys.argv[3]) if len(sys.argv) > 3 else 25

  devs = {}
  m0, outs0 = run_pass(pkl, n_iters, seed=7)
  devs['run1'] = {'DEV': m0.DEV, 'WARP_DEV': m0.WARP_DEV, 'QUEUE_DEV': m0.QUEUE_DEV,
                  'lat_delay': m0.lat_delay, 'combined_model_type': m0._combined_model_type,
                  'frame_skip': m0.frame_skip, 'n_vision_input_names': len(m0.vision_input_names)}
  m1, outs1 = run_pass(pkl, n_iters, seed=7)
  devs['run2'] = {'DEV': m1.DEV}

  all_finite = all(np.all(np.isfinite(o)) for o in outs0)
  frac_nonzero = float(np.mean([np.count_nonzero(o) / o.size for o in outs0]))

  rel = []
  for a, b in zip(outs0, outs1, strict=False):
    denom = max(float(np.max(np.abs(a))), 1e-12)
    rel.append(float(np.max(np.abs(a - b)) / denom))
  max_rel = max(rel)

  result = {
    'pkl': pkl,
    'n_iters': n_iters,
    'devices': devs,
    'all_finite': all_finite,
    'mean_nonzero_fraction': frac_nonzero,
    'plan_shape': list(outs0[0].shape),
    'plan_absmax_run1': float(np.max(np.abs(outs0[0]))),
    'max_rel_diff_two_runs': max_rel,
    'deterministic_rel_le_1e-6': max_rel <= 1e-6,
  }
  with open(os.path.join(outdir, 'smoke_offline_results.json'), 'w') as f:
    json.dump(result, f, indent=2)
  print(json.dumps(result, indent=2))
  ok = all_finite and frac_nonzero > 0.5 and max_rel <= 1e-6
  print("SMOKE:", "PASS" if ok else "FAIL")
  return 0 if ok else 1


if __name__ == '__main__':
  sys.exit(main())
