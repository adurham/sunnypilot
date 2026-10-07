#!/usr/bin/env python3
"""Self-contained integration test for the WS-B Mac frame path.

Builds a SYNTHETIC replay fixture from a REAL recorded camera segment
(~/comma-routes/00000149--4a4df1cf8a--36), runs `framebridge.py --replay`, and
asserts a VisionIpcClient('camerad', VISION_STREAM_NARROW_ROAD) receives frames
with the EXACT device frame_id / timestampSof from the fixture.

eof is synthesized as `sof + 50 ms` per the WS-B task brief (the real qlog also
carries a ~14.697 ms eof; the synthesized marker is used here deliberately and
documented in the module and in the run report). frameId / encodeId / sof come
from the route's qlog EncodeIndex.

Run directly:
  PYTHONPATH=$PWD:$PWD/opendbc_repo:$PWD/msgq_repo:$PWD/tinygrad_repo \\
    .venv/bin/python openpilot/offload/mac/tests/test_integration_replay.py
or via pytest (the test is skipped if the route or the built vtdec binary is
absent).
"""
from __future__ import annotations

import json
import os
import signal
import struct
import subprocess
import sys
import time

import pytest

REPO = "/Users/adam.durham/repos/sunnypilot-offload"
ROUTE = os.path.expanduser("~/comma-routes/00000149--4a4df1cf8a--36")
HEVC = os.path.join(ROUTE, "fcamera.hevc")
QLOG = os.path.join(ROUTE, "qlog.zst")
VTDEC = os.path.join(REPO, "openpilot/offload/mac/vtdec")
EOF_SYNTH_NS = 50_000_000     # 50 ms marker (synthesized, see module docstring)
HDR = struct.Struct("<IIIQQI")
N_FRAMES = 60


def _have_route() -> bool:
  return os.path.exists(HEVC) and os.path.exists(QLOG) and os.path.exists(VTDEC)


def build_fixture(out_path: str, n_frames: int = N_FRAMES):
  """Write a synthetic .enc fixture; return list of (frame_id, encode_id, sof, eof)."""
  sys.path.insert(0, REPO)
  from openpilot.offload.mac import make_fixture as mk

  with open(HEVC, "rb") as f:
    buf = f.read()
  nals = mk.parse_nals(buf)
  aus = mk.group_access_units(nals)
  idx = mk.read_qlog_idx(QLOG)["narrow"]

  n = min(n_frames, len(aus), len(idx))
  expect = []
  os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
  with open(out_path, "wb") as f:
    for i in range(n):
      au = buf[aus[i].span_pos:aus[i].span_end]
      e = idx[i]
      eof = e.timestamp_sof + EOF_SYNTH_NS     # synthesized marker
      f.write(HDR.pack(len(au), e.frame_id, e.encode_id, e.timestamp_sof, eof, e.flags))
      f.write(au)
      expect.append((e.frame_id, e.encode_id, e.timestamp_sof, eof))
  return expect


def extract_params(out_dir: str) -> str:
  sys.path.insert(0, REPO)
  from openpilot.offload.mac import fixtures as fix
  p = os.path.join(out_dir, "_fcam.params")
  with open(p, "wb") as f:
    f.write(fix.load_params_from_fixture_source(HEVC))
  return p


def run_integration(tmpdir: str, n_frames: int = N_FRAMES, pace: float = 0.01):
  """Run the whole flow; return (received, decode_ms_list, summary)."""
  sys.path.insert(0, REPO)
  from openpilot.cereal.visionipc import VisionStreamType
  from msgq.visionipc import VisionIpcClient

  fix_path = os.path.join(tmpdir, "narrow.enc")
  expect = build_fixture(fix_path, n_frames)
  params = extract_params(tmpdir)
  out_jsonl = os.path.join(tmpdir, "latency.jsonl")
  if os.path.exists(out_jsonl):
    os.remove(out_jsonl)

  env = dict(os.environ)
  env["PYTHONPATH"] = f"{REPO}:{REPO}/opendbc_repo:{REPO}/msgq_repo:{REPO}/tinygrad_repo"
  cmd = [os.path.join(REPO, ".venv/bin/python"), "-m", "openpilot.offload.mac.framebridge",
         "--replay", "--replay-path", fix_path, "--hevc-params", params,
         "--out", out_jsonl, "--replay-frames", str(n_frames), "--replay-pace", str(pace)]
  proc = subprocess.Popen(cmd, cwd=REPO, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

  received = []
  try:
    client = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_NARROW_ROAD, False)
    # connect(True) blocks until the server is up
    ok = client.connect(True)
    assert ok, "VisionIpcClient failed to connect to camerad"
    deadline = time.monotonic() + max(15.0, n_frames * pace * 3 + 5)
    while time.monotonic() < deadline:
      buf = client.recv(500)
      if buf is not None:
        received.append((int(client.frame_id), int(client.timestamp_sof), int(client.timestamp_eof)))
        if len(received) >= n_frames:
          break
  finally:
    time.sleep(0.2)
    proc.send_signal(signal.SIGINT)
    try:
      out = proc.communicate(timeout=10)[0].decode(errors="replace")
    except subprocess.TimeoutExpired:
      proc.kill()
      out = ""

  dec = []
  if os.path.exists(out_jsonl):
    for line in open(out_jsonl):
      line = line.strip()
      if not line:
        continue
      try:
        r = json.loads(line)
      except json.JSONDecodeError:
        continue
      if r.get("kind") == "frame" and isinstance(r.get("decode_ms"), (int, float)):
        dec.append(r["decode_ms"])
  return received, dec, out, expect


# --------------------------- pytest entry points --------------------------- #

@pytest.mark.skipif(not _have_route(), reason="real route or built vtdec missing")
def test_replay_exact_frame_ids_and_sof(tmp_path):
  received, dec, out, expect = run_integration(str(tmp_path))
  assert received, f"no frames received via VisionIPC; framebridge output:\n{out}"
  # Every received frame must carry the EXACT device frame_id / sof / eof.
  by_id = {fid: (sof, eof) for fid, sof, eof in [(e[0], e[2], e[3]) for e in expect]}
  for fid, sof, eof in received:
    assert fid in by_id, f"unexpected frame_id {fid}"
    assert (sof, eof) == by_id[fid], f"frame {fid}: got sof/eof {(sof, eof)} want {by_id[fid]}"
  # decode_ms must be logged for the frames we published.
  assert dec, f"no decode_ms rows logged; framebridge output:\n{out}"
  p50, p99 = _pctl(dec, 0.50), _pctl(dec, 0.99)
  print(f"\n[integration] received={len(received)}/{len(expect)} "
        f"decode_ms p50={p50:.3f} p99={p99:.3f} (n={len(dec)})")


def _pctl(xs, p):
  s = sorted(xs)
  if not s:
    return float("nan")
  return s[min(len(s) - 1, int(len(s) * p))]


if __name__ == "__main__":
  if not _have_route():
    print("route or vtdec missing; nothing to do")
    raise SystemExit(0)
  import tempfile
  with tempfile.TemporaryDirectory() as td:
    received, dec, out, expect = run_integration(td, n_frames=N_FRAMES)
    print("framebridge output:\n" + out)
    print(f"received {len(received)} frames, {len(dec)} decode_ms rows")
    if dec:
      print(f"decode_ms p50={_pctl(dec,0.5):.3f} p99={_pctl(dec,0.99):.3f} max={max(dec):.3f}")
    by_id = {e[0]: (e[2], e[3]) for e in expect}
    bad = [(fid, sof, eof) for fid, sof, eof in received if by_id.get(fid) != (sof, eof)]
    print("exact-match violations:", bad)
    print("first 3 received:", received[:3])
    print("expected first 3:", expect[:3][:1] and [(e[0], e[2], e[3]) for e in expect[:3]])
    assert received, "no frames received"
    assert not bad, f"exact-match violations: {bad}"
    print("INTEGRATION OK")
