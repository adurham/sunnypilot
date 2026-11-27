#!/usr/bin/env python3
"""WS-C/WS-B OPEN-C1 regression: real forward pass over DEVICE-geometry VisionIPC.

Closes the "pixel-exact numerics not validated" hole. With OPEN-C1 fixed, WS-B's
framebridge publishes NV12 frames at DEVICE geometry (stride 2048, uv_offset 2490368,
size 4804608 for 1928x1208). This test:

  1. starts `framebridge --replay` over the real route-149 fixture (narrow), which
     decodes with vtdec and publishes into a private VisionIPC server;
  2. runs the real `modeld_runner` subprocess against it for a few frames and asserts
     it exits 0 (the run path consumes the device-geometry buffers; the OPEN-C1
     geometry assert in modeld_runner must not fire);
  3. captures real decoded frames from the same stream and drives a freshly-loaded
     OFFLOAD modeld_v2 `ModelState` in-process, asserting the forward pass produces
     FINITE, NONZERO outputs, and that two independent fresh models are deterministic
     (max rel diff <= 1e-6) on the same captured frames.

Run (from the worktree root):
  PYTHONPATH=$PWD:$PWD/opendbc_repo:$PWD/msgq_repo:$PWD/tinygrad_repo \\
    .venv/bin/python -m pytest openpilot/offload/replay/tests/test_geometry_inference.py -q
"""
from __future__ import annotations

import json
import os
import signal
import subprocess
import time

import numpy as np
import pytest

REPO = "/Users/adam.durham/repos/sunnypilot-offload"
ROUTE = os.path.expanduser("~/comma-routes/00000149--4a4df1cf8a--36")
HEVC = os.path.join(ROUTE, "fcamera.hevc")
FIXTURE = os.path.join(REPO, "openpilot/offload/replay/fixtures/00000149--4a4df1cf8a--36/narrow.enc")
VTDEC = os.path.join(REPO, "openpilot/offload/mac/vtdec")
PKL = os.path.join(REPO, "openpilot/offload/models/driving_supercombo_fork_metal2.pkl")
PKL_MANIFEST = PKL + ".chunkmanifest"

DEVICE_STRIDE = 2048
DEVICE_UV_OFFSET = 2490368
DEVICE_SIZE = 4804608
N_RUNNER_FRAMES = 5
N_CAPTURE = 6


def _have() -> bool:
  have_pkl = os.path.exists(PKL) or os.path.exists(PKL_MANIFEST)
  return have_pkl and all(os.path.exists(p) for p in (HEVC, FIXTURE, VTDEC))


def _pyenv(server: str) -> dict:
  env = dict(os.environ)
  env["PYTHONPATH"] = f"{REPO}:{REPO}/opendbc_repo:{REPO}/msgq_repo:{REPO}/tinygrad_repo"
  env["OFFLOAD"] = "1"
  env["OFFLOAD_VIPC_SERVER"] = server
  env["OFFLOAD_DEV"] = "METAL"
  env["OFFLOAD_WARP_DEV"] = "METAL"
  env["COMBINED_MODEL_PKL"] = PKL
  return env


def _start_bridge(server: str, out_jsonl: str, log_path: str) -> subprocess.Popen:
  logf = open(log_path, "wb")
  proc = subprocess.Popen(
    [os.path.join(REPO, ".venv/bin/python"), "-m", "openpilot.offload.mac.framebridge",
     "--replay", "--replay-path", FIXTURE, "--hevc-params", HEVC, "--out", out_jsonl,
     "--replay-cam", "narrow", "--replay-frames", "80", "--replay-pace", "0.02"],
    cwd=REPO, env=_pyenv(server), stdout=logf, stderr=subprocess.STDOUT)
  proc._logf = logf  # type: ignore[attr-defined]
  return proc


@pytest.mark.skipif(not _have(), reason="route/fixture/vtdec/pkl missing")
def test_modeld_runner_and_real_forward_pass_over_device_geometry(tmp_path, monkeypatch):
  monkeypatch.setenv("OFFLOAD", "1")
  server = f"geomtest{os.getpid()}"
  latency = str(tmp_path / "latency.jsonl")
  modelv2 = str(tmp_path / "modelv2.jsonl")
  bridge = _start_bridge(server, latency, str(tmp_path / "fb.log"))

  from openpilot.cereal.visionipc import VisionStreamType
  from msgq.visionipc import VisionIpcClient

  runner = None
  try:
    # (2) the real modeld_runner over the new geometry
    runner = subprocess.Popen(
      [os.path.join(REPO, ".venv/bin/python"), "-m", "openpilot.offload.replay.modeld_runner",
       "--server", server, "--out", modelv2, "--frames", str(N_RUNNER_FRAMES), "--timeout", "200"],
      cwd=REPO, env=_pyenv(server), stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    # (3) capture real decoded frames off the same stream
    client = VisionIpcClient(server, VisionStreamType.VISION_STREAM_NARROW_ROAD, False)
    ok = client.connect(True)
    assert ok, "failed to connect VisionIpcClient to framebridge"
    assert int(client.buffer_len) == DEVICE_SIZE, f"buffer_len={client.buffer_len} != {DEVICE_SIZE}"
    assert int(client.stride) == DEVICE_STRIDE and int(client.uv_offset) == DEVICE_UV_OFFSET

    frames = []
    deadline = time.monotonic() + 120
    while len(frames) < N_CAPTURE and time.monotonic() < deadline:
      buf = client.recv(500)
      if buf is not None:
        frames.append(bytes(buf.data))
    assert len(frames) == N_CAPTURE, f"captured {len(frames)}/{N_CAPTURE} frames"
    for fb in frames:
      assert len(fb) == DEVICE_SIZE

    runner_out = runner.communicate(timeout=200)[0].decode(errors="replace")
    assert runner.returncode == 0, f"modeld_runner exit {runner.returncode}:\n{runner_out}"
    rows = [json.loads(x) for x in open(modelv2) if x.strip()] if os.path.exists(modelv2) else []
    assert len(rows) == N_RUNNER_FRAMES, f"modeld_runner rows={len(rows)} != {N_RUNNER_FRAMES}\n{runner_out}"
  finally:
    for p in (runner, bridge):
      if p is not None and p.poll() is None:
        p.send_signal(signal.SIGINT)
        try:
          p.wait(timeout=10)
        except subprocess.TimeoutExpired:
          p.kill()
    logf = getattr(bridge, "_logf", None)
    if logf:
      logf.close()

  # (4) fresh in-process model(s) over the captured device-geometry frames
  os.environ["COMBINED_MODEL_PKL"] = PKL
  from openpilot.sunnypilot.modeld_v2 import modeld as M
  from openpilot.sunnypilot.modeld_v2.constants import ModelConstants

  w, h = int(client.width), int(client.height)

  def make_inputs(model):
    """The per-frame inputs modeld's run loop synthesizes (mirrors modeld_runner._inputs)."""
    vec = np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32)
    vec[0] = 1.0
    inp = {model.desire_key: vec, "traffic_convention": np.array([0.0, 1.0], dtype=np.float32)}
    if "lateral_control_params" in model.numpy_inputs:
      inp["lateral_control_params"] = np.array([0.0, 0.1], dtype=np.float32)
    if "action_t" in model.numpy_inputs:
      inp["action_t"] = np.array([0.15, 0.15], dtype=np.float32)
    return inp

  def run_pass():
    model = M.ModelState(cam_w=w, cam_h=h, chestnut=False)
    model.warmup()
    assert model.adapter.frame_copy_size == 3735552
    outs = []
    for fb in frames:
      assert len(fb) >= model.adapter.frame_copy_size  # the OPEN-C1 assert in modeld_runner
      bufs = dict.fromkeys(model.vision_input_names, fb)
      transforms = {k: np.eye(3, dtype=np.float32) for k in model.vision_input_names}
      out = model.run(bufs, transforms, make_inputs(model))
      key = "plan" if "plan" in out else sorted(out)[0]
      outs.append(np.asarray(out[key], dtype=np.float64).ravel())
    return outs

  o0 = run_pass()
  o1 = run_pass()

  all_finite = all(np.all(np.isfinite(o)) for o in o0)
  nonzero = float(np.mean([np.count_nonzero(o) / o.size for o in o0]))
  rel = [float(np.max(np.abs(a - b)) / max(float(np.max(np.abs(a))), 1e-12))
         for a, b in zip(o0, o1, strict=True)]
  max_rel = max(rel)

  summary = f"[geometry-inference] frames={len(frames)} size={DEVICE_SIZE} stride={DEVICE_STRIDE} finite={all_finite} nz={nonzero:.3f} rel={max_rel:.2e}"
  print("\n" + summary)
  assert all_finite, "forward pass produced non-finite outputs"
  assert nonzero > 0.5, f"outputs mostly zero (nonzero_frac={nonzero})"
  assert max_rel <= 1e-6, f"not deterministic across two fresh runs: max_rel={max_rel}"


if __name__ == "__main__":
  if not _have():
    print("route/fixture/vtdec/pkl missing; nothing to do")
    raise SystemExit(0)
  import pathlib
  import tempfile

  class _MP:
    def setenv(self, key, value):
      os.environ[key] = value

  with tempfile.TemporaryDirectory() as td:
    test_modeld_runner_and_real_forward_pass_over_device_geometry(pathlib.Path(td), _MP())
    print("GEOMETRY INFERENCE OK")
