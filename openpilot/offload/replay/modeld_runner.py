#!/usr/bin/env python3
"""WS-C modeld_runner — drive the WS-A OFFLOAD modeld_v2 run path over VisionIPC.

Consumes the local VisionIPC server that WS-B's `framebridge` publishes decoded
NV12 frames into (INTERFACES.md §3), runs the fork-flavor Metal supercombo
`run_model` JIT, and — each frame — publishes a `modelV2` message into the LOCAL
msgq (so G2/G6 can join modelV2 outputs to frames) plus one jsonl row with the
Mac-clock modelV2 publish time.

This is a *runtime binding*, not a rewrite of modeld: it imports the OFFLOAD=1
patched `openpilot.sunnypilot.modeld_v2.modeld`, uses `ModelState.warmup/run`
exactly as `main()` does, and only supplies the frame source from VisionIPC.

Env (set by p1_gate if absent):
  OFFLOAD=1, OFFLOAD_VIPC_SERVER, COMBINED_MODEL_PKL, OFFLOAD_CARPARAMS_PKL,
  OFFLOAD_DEV/OFFLOAD_WARP_DEV (default METAL).

Usage (from the worktree root):
  OFFLOAD=1 OFFLOAD_VIPC_SERVER=camerad COMBINED_MODEL_PKL=<pkl> \\
  PYTHONPATH=$PWD:$PWD/opendbc_repo:$PWD/msgq_repo:$PWD/tinygrad_repo \\
    .venv/bin/python -m openpilot.offload.replay.modeld_runner \\
      --out <jsonl> [--frames N] [--timeout 30] [--cam narrow|wide] [--publish]

Exit: 0 ran >=1 frame, 2 setup failure (no vipc/model), 3 no frames within timeout.
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import time

import numpy as np

if os.environ.get("OFFLOAD") != "1":
  print("[modeld_runner] must run with OFFLOAD=1", file=sys.stderr)
  raise SystemExit(2)

from msgq.visionipc import VisionIpcClient  # noqa: E402
from openpilot.cereal.visionipc import VisionStreamType  # noqa: E402
import openpilot.cereal.messaging as messaging  # noqa: E402
from openpilot.sunnypilot.modeld_v2.constants import ModelConstants  # noqa: E402

_STOP = False


def _sig(_s, _f):
  global _STOP
  _STOP = True


def _inputs(model, v_ego: float, lat_delay: float):
  vec_desire = np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32)
  vec_desire[0] = 1.0
  inp = {model.desire_key: vec_desire,
         "traffic_convention": np.array([0.0, 1.0], dtype=np.float32)}
  if "lateral_control_params" in model.numpy_inputs:
    inp["lateral_control_params"] = np.array([v_ego, lat_delay], dtype=np.float32)
  if "action_t" in model.numpy_inputs:
    inp["action_t"] = np.array([lat_delay + 0.05, lat_delay + 0.05], dtype=np.float32)
  return inp


def main(argv=None) -> int:
  ap = argparse.ArgumentParser(description="Drive OFFLOAD modeld_v2 over VisionIPC")
  ap.add_argument("--out", default=None, help="jsonl for modelV2 publish rows")
  ap.add_argument("--server", default=os.environ.get("OFFLOAD_VIPC_SERVER", "camerad"))
  ap.add_argument("--frames", type=int, default=0, help="stop after N frames (0 = until timeout/SIGINT)")
  ap.add_argument("--timeout", type=float, default=30.0, help="max wall seconds")
  ap.add_argument("--cam", default="narrow", choices=["narrow", "wide"], help="main stream")
  ap.add_argument("--publish", action="store_true", help="publish modelV2 into local msgq each frame")
  ap.add_argument("--v-ego", type=float, default=0.0)
  ap.add_argument("--lat-delay", type=float, default=0.1)
  args = ap.parse_args(argv)
  signal.signal(signal.SIGINT, _sig)
  signal.signal(signal.SIGTERM, _sig)

  # --- import the patched model stack (needs OFFLOAD=1 already set) ----------
  try:
    from openpilot.sunnypilot.modeld_v2 import modeld as M
  except Exception as e:  # noqa: BLE001
    print(f"[modeld_runner] import modeld failed: {e!r}", file=sys.stderr)
    return 2

  main_stream = (VisionStreamType.VISION_STREAM_NARROW_ROAD if args.cam == "narrow"
                 else VisionStreamType.VISION_STREAM_WIDE_ROAD)
  extra_stream = VisionStreamType.VISION_STREAM_WIDE_ROAD

  # wait for the VisionIPC server / streams to appear
  t0 = time.monotonic()
  while time.monotonic() - t0 < args.timeout:
    if _STOP:
      return 3
    streams = VisionIpcClient.available_streams(args.server, block=False)
    if streams:
      break
    time.sleep(0.1)
  else:
    print(f"[modeld_runner] no VisionIPC server '{args.server}' within {args.timeout}s", file=sys.stderr)
    return 2

  cli = VisionIpcClient(args.server, main_stream, True)
  cli_extra = VisionIpcClient(args.server, extra_stream, False)
  t0 = time.monotonic()
  while not cli.connect(False):
    if time.monotonic() - t0 > args.timeout or _STOP:
      print("[modeld_runner] VisionIPC main connect failed", file=sys.stderr)
      return 2
    time.sleep(0.1)
  have_extra = False
  t0 = time.monotonic()
  while not cli_extra.connect(False):
    if time.monotonic() - t0 > 2.0:
      break
    time.sleep(0.1)
  else:
    have_extra = True

  W, H = int(cli.width), int(cli.height)
  print(f"[modeld_runner] connected {args.server}/{args.cam} {W}x{H} extra={have_extra}", flush=True)

  try:
    model = M.ModelState(cam_w=W, cam_h=H, chestnut=False)
    model.warmup()
  except Exception as e:  # noqa: BLE001
    print(f"[modeld_runner] model load failed: {e!r}", file=sys.stderr)
    return 2

  pm = messaging.PubMaster(["modelV2"]) if args.publish else None
  out_fp = open(args.out, "w", buffering=1) if args.out else None
  transforms = {name: np.eye(3, dtype=np.float32) for name in model.vision_input_names}
  lat_delay = float(getattr(model, "lat_delay", args.lat_delay))

  n = 0
  t_start = time.monotonic()
  last_extra_sof = -1
  last_extra_buf = None
  try:
    while not _STOP and (args.frames == 0 or n < args.frames) and time.monotonic() - t_start < args.timeout:
      buf = cli.recv()
      if buf is None:
        time.sleep(0.002)
        continue
      fid = int(cli.frame_id)
      sof = int(cli.timestamp_sof)
      eof = int(cli.timestamp_eof)
      # modeld frame sync: keep the extra (wide) stream at/behind the main SOF
      if have_extra:
        while last_extra_sof < sof - 25_000_000:
          eb = cli_extra.recv()
          if eb is None:
            break
          last_extra_buf = eb
          last_extra_sof = int(cli_extra.timestamp_sof)
      bufs = {name: (last_extra_buf if "big" in name else buf) for name in model.vision_input_names}
      if last_extra_buf is None:
        bufs = {name: buf for name in model.vision_input_names}
      # WS-B's VisionIPC buffers are tight (1928*1208*3/2) but modeld copies
      # frame_copy_size (stride-aligned, larger). Pad to frame_copy_size so the
      # run path's np.frombuffer(count=...) succeeds. NOTE: the NV12 stride/layout
      # still differs (see OPEN-C1 in RESULTS.md) — structural gates only.
      fcs = int(getattr(model, "frame_copy_size", 0))
      if fcs:
        padded = {}
        for name, b in bufs.items():
          if b is None:
            continue
          data = bytes(b.data) if hasattr(b, "data") else bytes(b)
          if len(data) < fcs:
            data = data + b"\x00" * (fcs - len(data))
          padded[name] = data
        if padded:
          bufs = padded
      inp = _inputs(model, args.v_ego, lat_delay)
      t_in = time.perf_counter()
      try:
        model.run(bufs, transforms, inp)
      except Exception as e:  # noqa: BLE001
        print(f"[modeld_runner] run failed at frame {fid}: {e!r}", file=sys.stderr)
        return 2
      exec_ms = (time.perf_counter() - t_in) * 1e3
      pub_ns = time.monotonic_ns()
      if pm is not None:
        m = messaging.new_message("modelV2")
        m.modelV2.frameId = fid
        # frameAge is UInt32 in the schema and is a DEVICE-side quantity
        # (now_device - sof); this offline runner has no device clock, so leave 0.
        m.modelV2.frameAge = 0
        m.logMonoTime = pub_ns
        pm.send("modelV2", m)
      if out_fp:
        out_fp.write(json.dumps({"kind": "modelv2", "cam": args.cam, "frame_id": fid,
                                 "sof_dev_ns": sof, "eof_dev_ns": eof,
                                 "modelv2_mac_ns": pub_ns, "model_exec_ms": exec_ms},
                                separators=(",", ":")) + "\n")
      n += 1
  finally:
    if out_fp:
      out_fp.close()

  print(f"[modeld_runner] ran {n} frames in {time.monotonic() - t_start:.2f}s", flush=True)
  return 0 if n > 0 else 3


if __name__ == "__main__":
  sys.exit(main())
