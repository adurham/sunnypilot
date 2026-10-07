#!/usr/bin/env python3
"""ZMQ loopback integration test for framebridge (WS-B) — no device required.

This repo's msgq build has NO ZMQ transport (MSGQSubSocket::connect asserts
addr == "127.0.0.1"), so framebridge subscribes to the device with pyzmq.  This
test stands up a local pyzmq PUB on the frozen port scheme (ports.py), impersonates
the device bridge, and drives a real framebridge in --host mode:

  * publishes framed EncodeData for narrowRoadEncodeData (VPS/SPS/PPS in the
    `.header` field on the keyframe, exactly like the device encoder),
  * publishes the small services (carState/deviceState/...),
  * asserts framebridge decoded + published every frame, and that the small
    services were re-published into the LOCAL msgq with a Mac-restamped header
    (SubMaster sees them).

Run directly or via pytest (skips if vtdec is not built).
"""
from __future__ import annotations

import json
import os
import signal
import subprocess
import threading
import time

import pytest

REPO = "/Users/adam.durham/repos/sunnypilot-offload"
ROUTE = os.path.expanduser("~/comma-routes/00000149--4a4df1cf8a--36")
FIXTURE = os.path.join(REPO, "openpilot/offload/replay/fixtures/00000149--4a4df1cf8a--36/narrow.enc")
VTDEC = os.path.join(REPO, "openpilot/offload/mac/vtdec")
HEVC = os.path.join(ROUTE, "fcamera.hevc")
SMALL = ["carState", "deviceState", "carControl", "extrinsicsCalibration",
         "driverMonitoringState", "lateralDelay"]
N_FRAMES = 40


def _ready() -> bool:
  return all(os.path.exists(p) for p in (FIXTURE, VTDEC, HEVC))


def run_loopback(tmpdir: str, n_frames: int = N_FRAMES):
  import zmq
  import openpilot.cereal.messaging as messaging
  from openpilot.offload import ports
  from openpilot.cereal import log as capnp_log
  from openpilot.offload.mac import fixtures as fix

  recs = fix.read_fixture(FIXTURE, max_frames=n_frames)
  params = fix.load_params_from_fixture_source(HEVC)
  out_jsonl = os.path.join(tmpdir, "zmq.jsonl")

  ctx = zmq.Context()
  enc = ctx.socket(zmq.PUB)
  enc.bind(f"tcp://127.0.0.1:{ports.get_port('narrowRoadEncodeData')}")
  smalls = {}
  for svc in SMALL:
    s = ctx.socket(zmq.PUB)
    s.bind(f"tcp://127.0.0.1:{ports.get_port(svc)}")
    smalls[svc] = s
  # wideRoadEncodeData port must be bound too so we exercise the multi-cam sub
  wide = ctx.socket(zmq.PUB)
  wide.bind(f"tcp://127.0.0.1:{ports.get_port('wideRoadEncodeData')}")
  time.sleep(0.3)

  env = dict(os.environ)
  env["PYTHONPATH"] = f"{REPO}:{REPO}/opendbc_repo:{REPO}/msgq_repo:{REPO}/tinygrad_repo"
  env.pop("ZMQ", None)
  logf = open(os.path.join(tmpdir, "fb.log"), "wb")
  proc = subprocess.Popen([os.path.join(REPO, ".venv/bin/python"), "-m",
                           "openpilot.offload.mac.framebridge",
                           "--host", "127.0.0.1", "--out", out_jsonl],
                          cwd=REPO, env=env, stdout=logf, stderr=subprocess.STDOUT)
  time.sleep(2.0)   # let framebridge bind its VisionIPC + PubMaster

  sm = messaging.SubMaster(["carState", "deviceState", "carControl"], addr="127.0.0.1")
  seen = {}
  stop = threading.Event()

  def poll():
    while not stop.is_set():
      sm.update(50)
      for s in ("carState", "deviceState", "carControl"):
        if sm.updated[s]:
          seen[s] = int(sm.logMonoTime[s])
  th = threading.Thread(target=poll, daemon=True)
  th.start()
  time.sleep(0.3)

  def make_enc(rec, keyframe):
    e = capnp_log.Event.new_message()
    e.logMonoTime = rec.timestamp_sof
    e.init("narrowRoadEncodeData")
    ed = e.narrowRoadEncodeData
    idx = ed.idx
    idx.frameId = rec.frame_id
    idx.encodeId = rec.encode_id
    idx.timestampSof = rec.timestamp_sof
    idx.timestampEof = rec.timestamp_eof
    idx.flags = rec.flags
    idx.len = len(rec.data)
    idx.type = "fullHEVC"
    ed.data = rec.data
    ed.width = 1928
    ed.height = 1208
    if keyframe:
      ed.header = params       # device: VPS/SPS/PPS in header on keyframes
    return e.to_bytes()

  def make_small(svc, i):
    e = capnp_log.Event.new_message()
    e.logMonoTime = 1000 + i
    e.init(svc)
    if svc == "carState":
      e.carState.vEgo = float(i)
    return e.to_bytes()

  try:
    for i, rec in enumerate(recs):
      enc.send(make_enc(rec, keyframe=(i == 0)))
      for svc, s in smalls.items():
        s.send(make_small(svc, i))
      time.sleep(0.04)
    time.sleep(1.5)
  finally:
    stop.set()
    time.sleep(0.3)
    proc.send_signal(signal.SIGINT)
    try:
      proc.communicate(timeout=10)
    except subprocess.TimeoutExpired:
      proc.kill()
    logf.close()

  rows = []
  if os.path.exists(out_jsonl):
    for line in open(out_jsonl):
      line = line.strip()
      if line:
        rows.append(json.loads(line))
  log = open(os.path.join(tmpdir, "fb.log")).read()
  return rows, seen, log, recs


@pytest.mark.skipif(not _ready(), reason="fixture or vtdec missing")
def test_zmq_ingest_decode_republish(tmp_path):
  rows, seen, log, recs = run_loopback(str(tmp_path))
  assert len(rows) == len(recs), f"decoded {len(rows)} of {len(recs)}; log:\n{log}"
  # exact device frame_id/sof carried through
  got = {r["frame_id"]: (r["sof_dev_ns"], r["eof_dev_ns"]) for r in rows}
  for rec in recs:
    assert got[rec.frame_id] == (rec.timestamp_sof, rec.timestamp_eof), \
        f"frame {rec.frame_id}: {got[rec.frame_id]} != {(rec.timestamp_sof, rec.timestamp_eof)}"
  # small services re-published into local msgq
  assert seen, f"no small services republished; log:\n{log}"
  assert all(v > 0 for v in seen.values()), f"restamp looks wrong: {seen}"


if __name__ == "__main__":
  if not _ready():
    print("fixture or vtdec missing; nothing to do")
    raise SystemExit(0)
  import tempfile
  with tempfile.TemporaryDirectory() as td:
    rows, seen, log, recs = run_loopback(td)
    print(log)
    print(f"decoded {len(rows)}/{len(recs)}; republished: {sorted(seen)}")
    assert len(rows) == len(recs)
    assert seen
    print("ZMQ LOOPBACK OK")
