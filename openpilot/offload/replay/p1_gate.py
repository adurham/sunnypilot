#!/usr/bin/env python3
"""WS-C p1_gate — THE single offline evidence collector.

Runs the full offline pipeline end to end and prints one PASS/FAIL line per
implemented gate, then writes report.md + summary.json.

Pipeline per route (INTERFACES.md §1/§3/§5):
  (a) extractor            route -> fixture (.enc + .header + meta.json) under
                           ~/.hermes/cache/scratch/car-features/offload/replay-fixtures/
  (b) replayd              publishes the wire over local ZMQ (--jitter-ms 10 by default,
                           real-time pacing) exactly like the device bridge
  (c) framebridge (WS-B)   ZMQ SUB -> vtdec -> local VisionIPC + Mac-local cameraState,
                           writes the FrameEvent/LatencyRecord jsonl (--out)
  (d) modeld_runner        WS-A OFFLOAD modeld_v2 run path over VisionIPC, publishes
                           modelV2 into local msgq, writes modelV2 publish rows
  (e) metrics              G1 timeline, G2 counts, G6 SOF->modelV2, G7 decode + rlog
                           baselines -> PASS/FAIL per gate
  exit 0 if and only if every implemented gate PASSes.

Graceful degradation (do NOT stub a missing workstream):
  * WS-B absent (openpilot/offload/mac/framebridge.py or vtdec missing)
        -> print "[p1_gate] WS-B missing" and exit 2.
  * WS-A absent (OFFLOAD patch not in modeld_v2 + no compiled pkl in offload/models/)
        -> print "[p1_gate] WS-A missing" and exit 2.
  Both are still *reported* (with the exact missing path) in the report.

Usage (from the worktree root):
  PYTHONPATH=$PWD:$PWD/opendbc_repo:$PWD/msgq_repo:$PWD/tinygrad_repo \\
    .venv/bin/python -m openpilot.offload.replay.p1_gate \\
      [--route NAME=DIR[,DIR...]] ...  [--jitter-ms 10] [--duration 60] \\
      [--reports-dir openpilot/offload/replay/reports] [--speed 1.0]

Exit codes: 0 all gates PASS, 1 a gate FAILED, 2 dependency missing / setup error.
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import shutil
import signal
import subprocess
import sys
import time

# --- paths -------------------------------------------------------------------
REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))  # .../sunnypilot-offload
REPLAY_DIR = os.path.dirname(os.path.abspath(__file__))
FIXTURE_DIR = os.path.expanduser("~/.hermes/cache/scratch/car-features/offload/replay-fixtures")
SCRATCH = os.path.expanduser("~/.hermes/cache/scratch/car-features/offload/p1-gate")
ROUTES_ROOT = os.path.expanduser("~/comma-routes")
MODELS_DIR = os.path.join(REPO, "openpilot", "offload", "models")
MAC_DIR = os.path.join(REPO, "openpilot", "offload", "mac")
MODELD_PY = os.path.join(REPO, "openpilot", "sunnypilot", "modeld_v2", "modeld.py")

DEFAULT_ROUTES = [
  ("149-36", [os.path.join(ROUTES_ROOT, "00000149--4a4df1cf8a--36")]),
  ("113-0-11", [os.path.join(ROUTES_ROOT, f"00000113--8cc572ac6e--{i}") for i in range(12)]),
]


def pyenv() -> dict:
  env = dict(os.environ)
  env["PYTHONPATH"] = os.pathsep.join([REPO, os.path.join(REPO, "opendbc_repo"),
                                       os.path.join(REPO, "msgq_repo"),
                                       os.path.join(REPO, "tinygrad_repo")])
  return env


def log(msg: str) -> None:
  print(f"[p1_gate] {msg}", flush=True)


# --- dependency checks -------------------------------------------------------

def find_fork_pkl() -> str | None:
  for p in sorted(glob.glob(os.path.join(MODELS_DIR, "*.pkl"))):
    if os.path.exists(p):
      return p
  cands = sorted(glob.glob(os.path.join(MODELS_DIR, "*.pkl.chunkmanifest")))
  if cands:
    return cands[0][: -len(".chunkmanifest")]
  return None


def check_ws_a() -> tuple[bool, str, str | None]:
  if not os.path.exists(MODELD_PY):
    return False, f"modeld.py not found at {MODELD_PY}", None
  try:
    src = open(MODELD_PY).read()
  except OSError as e:
    return False, f"cannot read modeld.py ({e})", None
  if "OFFLOAD" not in src:
    return False, "OFFLOAD patches not present in modeld_v2/modeld.py", None
  pkl = find_fork_pkl()
  if pkl is None:
    return False, f"no compiled pkl under {MODELS_DIR}", None
  return True, f"pkl={os.path.relpath(pkl, REPO)}", pkl


def check_ws_b() -> tuple[bool, str]:
  fb = os.path.join(MAC_DIR, "framebridge.py")
  vt = os.path.join(MAC_DIR, "vtdec")
  if not os.path.exists(fb):
    return False, f"missing {os.path.relpath(fb, REPO)}"
  if not os.path.exists(vt):
    return False, f"missing {os.path.relpath(vt, REPO)} (binary not built)"
  return True, "framebridge.py + vtdec present"


# --- fixture helpers ---------------------------------------------------------

def ensure_fixture(name: str, route_dirs: list[str]) -> str:
  dest = os.path.join(FIXTURE_DIR, name)
  if os.path.exists(os.path.join(dest, "meta.json")):
    return dest
  log(f"extractor: building fixture {name} from {len(route_dirs)} segment(s)")
  cmd = [os.path.join(REPO, ".venv", "bin", "python"), "-m", "openpilot.offload.replay.extractor",
         *route_dirs, "--name", name]
  r = subprocess.run(cmd, env=pyenv(), cwd=REPO, capture_output=True, text=True)
  sys.stdout.write(r.stdout)
  sys.stderr.write(r.stderr)
  if r.returncode != 0:
    raise SystemExit(f"extractor failed for {name} (exit {r.returncode})")
  return dest


def write_params_files(fixture_dir: str, out_dir: str) -> str:
  """Write narrow.params / wide.params (Annex-B VPS/SPS/PPS) for framebridge."""
  os.makedirs(out_dir, exist_ok=True)
  for cam, src in (("narrow", "narrow.header"), ("wide", "wide.header")):
    with open(os.path.join(fixture_dir, src), "rb") as f:
      data = f.read()
    with open(os.path.join(out_dir, f"{cam}.params"), "wb") as f:
      f.write(data)
  return out_dir


def build_carparams_pkl(route_dirs: list[str]) -> str | None:
  """Extract CarParams bytes from the first rlog that has them."""
  from openpilot.tools.lib.logreader import LogReader
  out = os.path.join(SCRATCH, "carparams.bin")
  os.makedirs(SCRATCH, exist_ok=True)
  for rd in route_dirs:
    rlog = os.path.join(rd, "rlog.zst")
    if not os.path.exists(rlog):
      continue
    for msg in LogReader(rlog):
      if msg.which() == "carParams":
        with open(out, "wb") as f:
          f.write(msg.as_builder().to_bytes())
        return out
  return None


# --- process management ------------------------------------------------------

class Proc:
  def __init__(self, name: str, cmd: list[str], env: dict, log_path: str, cwd: str = REPO):
    self.name = name
    self.log_path = log_path
    self.fh = open(log_path, "w")
    self.proc = subprocess.Popen(cmd, env=env, cwd=cwd, stdout=self.fh,
                                 stderr=subprocess.STDOUT, text=True)

  def alive(self) -> bool:
    return self.proc.poll() is None

  def stop(self, grace: float = 5.0) -> int:
    if self.proc.poll() is None:
      self.proc.send_signal(signal.SIGTERM)
      try:
        self.proc.wait(timeout=grace)
      except subprocess.TimeoutExpired:
        self.proc.kill()
        self.proc.wait(timeout=5)
    try:
      self.fh.close()
    except Exception:
      pass
    return self.proc.returncode

  def tail(self, n: int = 12) -> str:
    try:
      with open(self.log_path) as f:
        return "".join(f.readlines()[-n:])
    except OSError:
      return ""


# --- one end-to-end route run ------------------------------------------------

def _parse_fb_summary(log_path: str) -> dict:
  """Parse framebridge's final 'summary: {...}' line -> {received,decoded,published}."""
  empty = {"received": {}, "decoded": {}, "published": {}}
  try:
    with open(log_path) as f:
      lines = f.readlines()
  except OSError:
    return empty
  for line in reversed(lines):
    i = line.find("framebridge] summary: ")
    if i >= 0:
      try:
        return json.loads(line[i + len("framebridge] summary: "):])
      except json.JSONDecodeError:
        return empty
  return empty


def run_wire_full(fixture_dir: str, route_dir: str, out_dir: str, host: str) -> dict:
  """Blast the whole fixture through replayd at --speed max with no subscribers and
  validate the emitted encodeId/frameId sequence against the full fixture. This
  proves the frame timeline (G1) over the entire fixture span (>=10 min for the
  multi-segment route) without waiting in real time."""
  log_path = os.path.join(out_dir, "wire_full.jsonl")
  proc = Proc("replayd_full",
              [os.path.join(REPO, ".venv", "bin", "python"), "-m", "openpilot.offload.replay.replayd",
               route_dir, "--fixture-dir", fixture_dir, "--host", host, "--speed", "max",
               "--pub-log", log_path],
              pyenv(), os.path.join(out_dir, "replayd_full.log"))
  proc.proc.wait(timeout=120)
  proc.stop()
  return wire_g1(log_path, fixture_dir)


def run_route(name: str, route_dirs: list[str], fixture_dir: str, args, out_dir: str) -> dict:
  """Launch replayd + framebridge + modeld_runner; return paths + logs."""
  os.makedirs(out_dir, exist_ok=True)
  env = pyenv()
  latency_jsonl = os.path.join(out_dir, "latency.jsonl")
  modelv2_jsonl = os.path.join(out_dir, "modelv2.jsonl")
  replay_log = os.path.join(out_dir, "replayd_pub.jsonl")
  params_dir = write_params_files(fixture_dir, os.path.join(out_dir, "params"))

  carparams = build_carparams_pkl(route_dirs)

  procs: list[Proc] = []
  first_route = route_dirs[0]

  # (c) framebridge (WS-B) FIRST so its msgq pubs + VisionIPC server exist before any
  # wire traffic; it feeds vtdec and writes the FrameEvent/LatencyRecord jsonl (--out).
  fb_cmd = [os.path.join(REPO, ".venv", "bin", "python"), os.path.join(MAC_DIR, "framebridge.py"),
            "--host", args.host, "--vtdec", os.path.join(MAC_DIR, "vtdec"),
            "--out", latency_jsonl, "--params-dir", params_dir]
  fb = Proc("framebridge", fb_cmd, env, os.path.join(out_dir, "framebridge.log"))
  procs.append(fb)
  time.sleep(1.5)

  # (b) replayd: publish the wire
  replayd_cmd = [os.path.join(REPO, ".venv", "bin", "python"), "-m", "openpilot.offload.replay.replayd",
                 first_route, "--fixture-dir", fixture_dir, "--host", args.host,
                 "--speed", str(args.speed), "--jitter-ms", str(args.jitter_ms),
                 "--pub-log", replay_log, "--stats"]
  if args.duration:
    replayd_cmd += ["--start", str(args.start), "--duration", str(args.duration)]
  replayd = Proc("replayd", replayd_cmd, env, os.path.join(out_dir, "replayd.log"))
  procs.append(replayd)
  time.sleep(1.5)

  # (d) modeld_runner (WS-A OFFLOAD path) over VisionIPC
  if not args.no_model:
    menv = dict(env)
    menv.update({"OFFLOAD": "1", "OFFLOAD_VIPC_SERVER": "camerad",
                 "OFFLOAD_DEV": "METAL", "OFFLOAD_WARP_DEV": "METAL"})
    if args.pkl:
      menv["COMBINED_MODEL_PKL"] = args.pkl
    if carparams:
      menv["OFFLOAD_CARPARAMS_PKL"] = carparams
    mr_cmd = [os.path.join(REPO, ".venv", "bin", "python"), "-m", "openpilot.offload.replay.modeld_runner",
              "--out", modelv2_jsonl, "--server", "camerad", "--publish",
              "--timeout", str(max(30.0, (args.duration or 60.0) + 15.0))]
    procs.append(Proc("modeld_runner", mr_cmd, menv, os.path.join(out_dir, "modeld_runner.log")))

  # run for the window (real-time): until replayd exits (+drain) or framebridge dies
  wall = (args.duration or 60.0) / max(float(args.speed), 1e-6) + 12.0
  t0 = time.monotonic()
  while time.monotonic() - t0 < wall:
    time.sleep(0.5)
    if replayd.proc.poll() is not None or not fb.alive():
      break

  logs = {}
  for p in reversed(procs):
    rc = p.stop()
    logs[p.name] = {"exit": rc, "tail": p.tail(10)}

  # framebridge's own summary carries what it received/decoded/published; trust it
  # over our line count for G1/G2 denominators.
  fb_summary = _parse_fb_summary(os.path.join(out_dir, "framebridge.log"))

  # collision detector: another session's framebridge stole our msgq endpoints
  collided = False
  try:
    with open(os.path.join(out_dir, "framebridge.log")) as f:
      fx = f.read()
    collided = "Killing old publisher" in fx or "Address already in use" in fx
  except OSError:
    pass

  return {"name": name, "fixture_dir": fixture_dir, "out_dir": out_dir,
          "latency_jsonl": latency_jsonl, "modelv2_jsonl": modelv2_jsonl,
          "replay_log": replay_log, "logs": logs, "collided": collided,
          "fb_summary": fb_summary,
          "counts": {"latency_rows": _count_lines(latency_jsonl),
                     "modelv2_rows": _count_lines(modelv2_jsonl),
                     "replay_pub_rows": _count_lines(replay_log)}}


def _count_lines(path: str) -> int:
  try:
    with open(path) as f:
      return sum(1 for _ in f)
  except OSError:
    return 0


def _port_free(host: str, port: int) -> bool:
  import socket
  s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  try:
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((host, port))
    return True
  except OSError:
    return False
  finally:
    s.close()


# Services replayd binds (video + small) — used for the preflight port check.
from openpilot.offload.ports import get_port
BOUND_SERVICES = ["narrowRoadEncodeData", "wideRoadEncodeData", "narrowRoadCameraState",
                  "wideRoadCameraState", "carState", "deviceState", "carControl",
                  "extrinsicsCalibration", "driverMonitoringState", "lateralDelay"]


def wait_for_clear(host: str, timeout: float = 180.0) -> tuple[bool, str]:
  """Block until no competitor replayd/framebridge is running and our tcp ports
  are free. Other sessions share this machine; their framebridge grabs the same
  msgq endpoints and their replayd the same tcp ports — both break a clean run."""
  t0 = time.monotonic()
  procs, locked = [], []
  while time.monotonic() - t0 < timeout:
    busy = subprocess.run(["pgrep", "-fl", "offload.replay.replayd|offload.mac.framebridge"],
                          capture_output=True, text=True)
    procs = [ln for ln in busy.stdout.splitlines() if "pgrep" not in ln]
    locked = [s for s in BOUND_SERVICES if not _port_free(host, get_port(s))]
    if not procs and not locked:
      return True, "clear"
    time.sleep(3.0)
  return False, f"still busy after {timeout}s (procs={procs[:3]} ports={locked[:3]})"


def combine_and_analyze(run: dict, route_dirs: list[str], args) -> dict:
  from openpilot.offload.replay import metrics as M
  combined = os.path.join(run["out_dir"], "combined.jsonl")
  with open(combined, "w") as out:
    for p in (run["latency_jsonl"], run["modelv2_jsonl"]):
      if os.path.exists(p):
        with open(p) as f:
          shutil.copyfileobj(f, out)
  rep = M.analyze(combined, run["fixture_dir"], window=None, replay_log=run["replay_log"],
                  received=run.get("fb_summary", {}).get("received"))
  rep["route"] = run["name"]
  rep["fixture_dir"] = run["fixture_dir"]
  rep["out_dir"] = run["out_dir"]
  rep["counts"] = run["counts"]
  rep["fb_summary"] = run.get("fb_summary", {})
  rep["collided"] = run.get("collided", False)
  rep["component_logs"] = run["logs"]
  rep["wire_check"] = wire_g1(run["replay_log"], run["fixture_dir"])
  rep["wire_full"] = run_wire_full(run["fixture_dir"], route_dirs[0], run["out_dir"], args.host)
  return rep


def wire_g1(replay_log: str, fixture_dir: str) -> dict:
  """G1 on the wire: every fixture frame emitted exactly once, in order, no dup,
  encodeId/frameId 1:1 monotone. With --duration the emitted records are a prefix
  of the fixture, so the check is 'emitted == first n emitted fixture records'."""
  from openpilot.offload.replay import fixture as fx
  try:
    with open(replay_log) as f:
      rows = [json.loads(x) for x in f if x.strip()]
  except OSError:
    return {"status": "SKIP", "reason": "no replayd pub-log"}
  vids = [r for r in rows if r.get("kind") == "video" and r["service"] == "narrowRoadEncodeData"]
  recs = list(fx.read_records(os.path.join(fixture_dir, fx.ENC_NAME["narrow"])))
  exp_eids = [r.encode_id for r in recs]
  exp_fids = [r.frame_id for r in recs]
  n = len(vids)
  eids = [r["encode_id"] for r in vids]
  dup = len(eids) != len(set(eids))
  matches = (eids == exp_eids[:n]) and ([r["frame_id"] for r in vids] == exp_fids[:n])
  mono = all(eids[i + 1] == eids[i] + 1 for i in range(len(eids) - 1)) if len(eids) > 1 else True
  ok = (n > 0) and (n <= len(recs)) and mono and (not dup) and matches
  return {"status": "PASS" if ok else "FAIL", "n_emitted": n, "n_fixture": len(recs),
          "encode_id_monotone": mono, "dup": dup, "emitted_matches_fixture_prefix": matches}


# --- reporting ---------------------------------------------------------------

def fmt_gate(g: dict) -> str:
  d = g.get("detail", {})
  if g["gate"] == "G6" and "p50_ms" in d:
    head = f"{g['gate']} {g['status']:4s}  SOF->modelV2  n={d['n']}  p50={d['p50_ms']}  "
    return head + f"p99={d['p99_ms']}  p99.9={d['p99.9_ms']} ms   (<= {d['limits'][0]}/{d['limits'][1]}/{d['limits'][2]})"
  if g["gate"] == "G7" and "p50_ms" in d:
    return f"{g['gate']} {g['status']:4s}  decode  n={d['n']}  p50={d['p50_ms']}  p99.9={d['p99.9_ms']} ms   (<= {d['limits'][0]}/{d['limits'][1]})"
  if g["gate"] == "G1":
    return f"{g['gate']} {g['status']:4s}  " + " | ".join(
      "".join([
        f"{c}: frames={v['frames']}/{v['source_frames']} drops={v['drops_in_span']} ",
        f"not_replayed={v['not_replayed']} dup={v['dup_frame_ids']} ",
        f"subseq={v['out_is_subsequence_of_source']} eid_mono={v['encode_id_monotone_strict']} ",
        f"contig={v['encode_id_contiguous']} 1to1={v['encode_id_frame_id_1to1']}",
      ]) for c, v in d.items())
  if g["gate"] == "G2":
    return f"{g['gate']} {g['status']:4s}  " + " | ".join(
      "".join([
        f"{c}: dec={v['decoded']} src_win={v['decoded_window'].get('source_in_window','-')} ",
        f"(no_drops={v['decoded_eq_encoded_in_window']}) mv2={v['joined_window'].get('modelv2',0)}",
        f"/{v['joined_window'].get('decoded_in_window','-')}",
      ]) for c, v in d.items())
  return f"{g['gate']} {g['status']}"


def write_report(reports_dir: str, ts: str, reports: list[dict], dep: dict, args) -> str:
  rd = os.path.join(reports_dir, ts)
  os.makedirs(rd, exist_ok=True)
  summary = {"timestamp": ts, "deps": dep, "jitter_ms": args.jitter_ms, "speed": args.speed,
             "duration": args.duration, "reports": reports}
  all_status = []
  for rep in reports:
    all_status += [g["status"] for g in rep["gates"]]
    all_status += [rep["wire_check"]["status"], rep.get("wire_full", {}).get("status", "PASS")]
  gate_status = "PASS" if all(s == "PASS" for s in all_status) else (
    "FAIL" if any(s == "FAIL" for s in all_status) else "SKIP")
  summary["gate_status"] = gate_status
  with open(os.path.join(rd, "summary.json"), "w") as f:
    json.dump(summary, f, indent=2, sort_keys=True)

  lines = [f"# offload p1_gate report — {ts}", "",
           f"- jitter injected: **{args.jitter_ms} ms**  speed: {args.speed}  duration: {args.duration}s",
           f"- WS-A: {dep['ws_a']['msg']}",
           f"- WS-B: {dep['ws_b']['msg']}",
           f"- overall: **{gate_status}**", ""]
  for rep in reports:
    lines += [f"## route `{rep['route']}`", "",
              f"fixture: `{rep['fixture_dir']}`  rows: {rep['counts']}", "",
              "```"]
    for g in rep["gates"]:
      lines.append(fmt_gate(g))
    parts = [
      f"G1w  {rep['wire_check']['status']:4s}  wire: emitted={rep['wire_check'].get('n_emitted')} ",
      f"fixture={rep['wire_check'].get('n_fixture')} dup={rep['wire_check'].get('dup')} ",
      f"prefix_match={rep['wire_check'].get('emitted_matches_fixture_prefix')}",
    ]
    lines.append("".join(parts))
    wf = rep.get("wire_full", {})
    if wf:
      parts = [
        f"G1wF {wf.get('status','SKIP'):4s}  full wire: emitted={wf.get('n_emitted')} ",
        f"fixture={wf.get('n_fixture')} dup={wf.get('dup')} ",
        f"prefix_match={wf.get('emitted_matches_fixture_prefix')}",
      ]
      lines.append("".join(parts))
    lines += ["```", "", "### rlog baselines (report only)", "", "```"]
    b = rep.get("baselines", {})
    for k, v in b.items():
      lines.append(f"{k}: {v}")
    lines += ["```", "", "### component logs", "```"]
    for cname, ci in rep.get("component_logs", {}).items():
      lines.append(f"--- {cname} (exit {ci['exit']}) ---")
      lines += ci["tail"].rstrip().splitlines()
    lines += ["```", ""]
  with open(os.path.join(rd, "report.md"), "w") as f:
    f.write("\n".join(lines) + "\n")
  return rd


# --- main --------------------------------------------------------------------

def main(argv=None) -> int:
  ap = argparse.ArgumentParser(description="WS-C offline pipeline gate collector")
  ap.add_argument("--route", action="append", default=None,
                  help="NAME=DIR[,DIR...] (repeatable). Default: 149-36 and 113-0-11")
  ap.add_argument("--jitter-ms", type=float, default=10.0)
  ap.add_argument("--speed", default="1.0")
  ap.add_argument("--duration", type=float, default=60.0, help="device seconds to replay per route")
  ap.add_argument("--start", type=float, default=0.0)
  ap.add_argument("--host", default="127.0.0.1")
  ap.add_argument("--reports-dir", default=os.path.join(REPLAY_DIR, "reports"))
  ap.add_argument("--runs-dir", default=SCRATCH)
  ap.add_argument("--no-model", action="store_true", help="skip modeld (still runs gates on frames)")
  ap.add_argument("--preflight-timeout", type=float, default=180.0,
                  help="seconds to wait for shared msgq/ports to clear before a run")
  args = ap.parse_args(argv)

  routes = []
  if args.route:
    for spec in args.route:
      name, _, dirs = spec.partition("=")
      routes.append((name, dirs.split(",")))
  else:
    routes = DEFAULT_ROUTES

  ts = time.strftime("%Y%m%d-%H%M%S", time.localtime())

  # dependency checks --------------------------------------------------------
  ws_b_ok, ws_b_msg = check_ws_b()
  ws_a_ok, ws_a_msg, pkl = (check_ws_a() if not args.no_model else (True, "skipped (--no-model)", None))
  args.pkl = pkl
  dep = {"ws_a": {"ok": ws_a_ok, "msg": ws_a_msg}, "ws_b": {"ok": ws_b_ok, "msg": ws_b_msg}}

  if not ws_b_ok:
    log(f"WS-B missing: {ws_b_msg}")
    write_report(args.reports_dir, ts, [], dep, args)
    return 2
  if not ws_a_ok:
    log(f"WS-A missing: {ws_a_msg}")
    write_report(args.reports_dir, ts, [], dep, args)
    return 2

  # (a) extractor on every route --------------------------------------------
  fixtures = {}
  for name, dirs in routes:
    try:
      fixtures[name] = ensure_fixture(name, dirs)
    except SystemExit as e:
      log(str(e))
      write_report(args.reports_dir, ts, [], dep, args)
      return 2

  # (b-e) pipeline per route -------------------------------------------------
  reports = []
  for name, dirs in routes:
    out_dir = os.path.join(args.runs_dir, ts, name)
    log(f"=== route {name}: {len(dirs)} segment(s), {args.duration}s @ speed {args.speed} ===")
    for attempt in (1, 2, 3):
      ok, why = wait_for_clear(args.host, timeout=args.preflight_timeout)
      if not ok:
        log(f"WARNING: ports/processes busy ({why}); attempting anyway")
      run = run_route(name, dirs, fixtures[name], args, out_dir)
      rep = combine_and_analyze(run, dirs, args)
      if not rep["collided"] and rep["counts"]["latency_rows"] > 0:
        break
      log(f"attempt {attempt}: collided={rep['collided']} rows={rep['counts']['latency_rows']} — retrying after a wait")
      time.sleep(5.0)
    reports.append(rep)
    for g in rep["gates"]:
      log(fmt_gate(g))
    log(f"G1w  {rep['wire_check']['status']:4s}  wire emitted={rep['wire_check'].get('n_emitted')}")
    log(f"G1wF {rep.get('wire_full',{}).get('status','SKIP'):4s}  full wire emitted={rep.get('wire_full',{}).get('n_emitted')}")

  rd = write_report(args.reports_dir, ts, reports, dep, args)
  statuses = [g["status"] for r in reports for g in r["gates"]] + \
             [r["wire_check"]["status"] for r in reports] + \
             [r.get("wire_full", {}).get("status", "PASS") for r in reports]
  overall = "PASS" if statuses and all(s == "PASS" for s in statuses) else (
    "FAIL" if any(s == "FAIL" for s in statuses) else "SKIP")
  log(f"overall {overall}  report: {os.path.relpath(rd, REPO)}/report.md")
  if all(s == "PASS" for s in statuses):
    return 0
  return 1


if __name__ == "__main__":
  sys.exit(main())
