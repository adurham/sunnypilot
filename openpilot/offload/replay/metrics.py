#!/usr/bin/env python3
"""WS-C metrics — per-gate metrics from a latency jsonl + the recorded route.

Consumes a FrameEvent/LatencyRecord jsonl written by WS-B's framebridge --out
(one JSON object per line) and the recorded rlog/fixture, and computes:

  G1  frame timeline continuity: no dup frame_ids; encodeId<->frameId 1:1 and
      monotone; frame_id gaps mirror the source stream.
  G2  counts: decoded == encoded frames; modelV2 outputs == frames processed.
  G6  e2e replay SOF->modelV2-published (Mac clock, jitter injected): p50/p99/p99.9.
  G7  decode latency (recv->decode done): p50/p99.9.
  Baselines (report-only, recomputed from the rlog):
      stock glass(SOF)->modelV2, glass->carControl, glass->sendcan.

Row schema accepted (superset of contract.LatencyRecord / FrameEvent):
  frame_id, cam, sof_dev_ns, eof_dev_ns, recv_mac_ns,
  decode_done_mac_ns, vipc_publish_mac_ns, modelv2_mac_ns,
  optional: encode_id, decode_ms, publish_mac_ns, timestamp_sof, timestamp_eof.
  Timestamps named `*_mac_ns` are Mac monotonic ns; `*_dev_ns` are device ns.

G6 is computed in the Mac clock domain (INTERFACES.md §1.2/§1.4: never subtract
across clocks): sof_mac ~= recv_mac_ns (the Mac's receipt of that frame's
EncodeData, ~0 transport on loopback), e2e = modelv2_mac_ns - recv_mac_ns.

Usage (from the worktree root):
  PYTHONPATH=$PWD:$PWD/opendbc_repo:$PWD/msgq_repo:$PWD/tinygrad_repo \\
    .venv/bin/python -m openpilot.offload.replay.metrics \\
      --latency <latency.jsonl> --route <route_dir_or_fixture_dir> \\
      [--replay-log <replayd_pub.jsonl>] [--window <frameId_lo:frameId_hi>] \\
      [--json <out.json>]
Exit: 0 all implemented gates PASS, 1 any implemented gate FAIL, 2 usage/IO error.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from bisect import bisect_right

import numpy as np

from openpilot.offload.replay import fixture as fx
from openpilot.tools.lib.logreader import LogReader

# --- gate thresholds (INTERFACES.md §5) --------------------------------------
G6_P50, G6_P99, G6_P999 = 75.0, 150.0, 250.0     # ms
G7_P50, G7_P999 = 3.0, 10.0                      # ms

CAM_SVC = {"narrow": "narrowRoadCameraState", "wide": "wideRoadCameraState"}


def pct(a, p):
  return float(np.percentile(np.asarray(a, dtype=float), p)) if len(a) else float("nan")


# --- row parsing -------------------------------------------------------------

def _g(d, *names):
  for n in names:
    if n in d and d[n] is not None:
      return d[n]
  return None


def load_latency(path: str) -> list[dict]:
  """Load a latency jsonl. Accepts framebridge frame rows and modeld_runner
  modelV2 rows (kind='modelv2'); both are returned in one list, tagged by kind."""
  rows = []
  with open(path) as f:
    for ln, line in enumerate(f, 1):
      line = line.strip()
      if not line:
        continue
      try:
        d = json.loads(line)
      except json.JSONDecodeError as e:
        raise ValueError(f"{path}:{ln}: bad json ({e})") from e
      kind = d.get("kind", "frame")
      row = {
        "kind": kind,
        "frame_id": _g(d, "frame_id", "frameId"),
        "cam": _g(d, "cam") or "narrow",
        "encode_id": _g(d, "encode_id", "encodeId"),
        "sof_dev_ns": _g(d, "sof_dev_ns", "timestamp_sof"),
        "eof_dev_ns": _g(d, "eof_dev_ns", "timestamp_eof"),
        "recv_mac_ns": _g(d, "recv_mac_ns"),
        "decode_done_mac_ns": _g(d, "decode_done_mac_ns"),
        "vipc_publish_mac_ns": _g(d, "vipc_publish_mac_ns", "publish_mac_ns"),
        "modelv2_mac_ns": _g(d, "modelv2_mac_ns"),
        "decode_ms": _g(d, "decode_ms"),
      }
      if row["frame_id"] is None:
        raise ValueError(f"{path}:{ln}: row has no frame_id/frameId")
      rows.append(row)
  return rows


def join_modelv2(rows: list[dict]) -> list[dict]:
  """Join modeld_runner modelV2 rows into the framebridge frame rows by
  (cam, frame_id); returns only the frame rows, with modelv2_mac_ns filled."""
  mv: dict[tuple, int] = {}
  for r in rows:
    if r["kind"] == "modelv2" and r["modelv2_mac_ns"] is not None:
      mv[(r["cam"], r["frame_id"])] = r["modelv2_mac_ns"]
      mv[("*", r["frame_id"])] = r["modelv2_mac_ns"]   # cam-agnostic fallback
  frames = []
  for r in rows:
    if r["kind"] == "modelv2":
      continue
    if r["modelv2_mac_ns"] is None:
      r["modelv2_mac_ns"] = mv.get((r["cam"], r["frame_id"])) or mv.get(("*", r["frame_id"]))
    frames.append(r)
  return frames


# --- source metadata ---------------------------------------------------------

def load_source_frame_ids(route_or_fixture: str) -> dict[str, list[int]]:
  """frameId list per camera from a fixture .enc (preferred) or an rlog."""
  def _read(cam, path):
    if os.path.exists(path):
      return [r.frame_id for r in fx.read_records(path)]
    return None
  out = {}
  for cam in fx.CAMERAS:
    ids = _read(cam, os.path.join(route_or_fixture, fx.ENC_NAME[cam]))
    if ids is None:
      ids = []
      rlog = os.path.join(route_or_fixture, "rlog.zst")
      if os.path.exists(rlog):
        svc = CAM_SVC[cam]
        for msg in LogReader(rlog):
          if msg.which() == svc:
            ids.append(int(getattr(msg, svc).frameId))
    out[cam] = ids
  return out


def load_source_encode_ids(route_or_fixture: str, cam: str) -> list[int]:
  enc = os.path.join(route_or_fixture, fx.ENC_NAME[cam])
  if os.path.exists(enc):
    return [r.encode_id for r in fx.read_records(enc)]
  return []


# --- G1 ----------------------------------------------------------------------

def _is_subsequence(sub, seq) -> bool:
  """True if `sub` is an order-preserving subsequence of `seq` (both ascending)."""
  it = iter(seq)
  return all(any(x == y for y in it) for x in sub)


def gate_g1(rows: list[dict], src_ids: dict[str, list[int]], src_eids: dict[str, list[int]],
            received: dict | None = None) -> dict:
  """Timeline integrity: no dup/synthetic frame_ids; encodeId monotone 1:1; the Mac
  frame stream is an order-preserving subsequence of the source (gaps explained by
  counted transport drops, never invented frames)."""
  by_cam: dict[str, list[dict]] = {}
  for r in rows:
    by_cam.setdefault(r["cam"], []).append(r)

  detail = {}
  ok = True
  for cam, rs in by_cam.items():
    rs = sorted(rs, key=lambda r: r["frame_id"])
    fids = [r["frame_id"] for r in rs]
    dup = len(fids) - len(set(fids))
    src = list(src_ids.get(cam, []))
    src_set = set(src)
    out_gaps = sorted({fids[i + 1] - fids[i] for i in range(len(fids) - 1) if fids[i + 1] - fids[i] > 1})
    src_gaps = sorted({src[i + 1] - src[i] for i in range(len(src) - 1) if src[i + 1] - src[i] > 1})
    eids = [r["encode_id"] for r in rs if r["encode_id"] is not None]
    eid_mono = all(eids[i + 1] > eids[i] for i in range(len(eids) - 1)) if len(eids) > 1 else True
    eid_contig = all(eids[i + 1] == eids[i] + 1 for i in range(len(eids) - 1)) if len(eids) > 1 else True
    pe = [r for r in rs if r["encode_id"] is not None]
    pair_ok = all(pe[i + 1]["encode_id"] > pe[i]["encode_id"] and
                  pe[i + 1]["frame_id"] > pe[i]["frame_id"] for i in range(len(pe) - 1))
    subseq = _is_subsequence(fids, src) if src else True
    all_in_src = all(f in src_set for f in fids) if src else True
    # drops within the emitted span (gaps in the timeline), and separately the
    # source frames outside the span (not replayed, not drops)
    if src and fids:
      span = (fids[0], fids[-1])
      src_in_span = sum(1 for f in src if span[0] <= f <= span[1])
      drops = src_in_span - len(fids)
      not_replayed = len(src) - src_in_span
    else:
      drops, not_replayed = 0, 0
    cam_ok = (dup == 0) and eid_mono and pair_ok and subseq and all_in_src
    ok = ok and cam_ok
    detail[cam] = {
      "frames": len(fids), "dup_frame_ids": dup, "source_frames": len(src),
      "drops_in_span": drops, "not_replayed": not_replayed,
      "frame_id_gaps_out": out_gaps, "frame_id_gaps_src": src_gaps,
      "out_is_subsequence_of_source": subseq, "all_frame_ids_in_source": all_in_src,
      "encode_id_monotone_strict": eid_mono, "encode_id_contiguous": eid_contig,
      "encode_id_frame_id_1to1": pair_ok,
    }
  return {"gate": "G1", "status": "PASS" if ok and by_cam else ("FAIL" if by_cam else "SKIP"),
          "detail": detail}


# --- G2 ----------------------------------------------------------------------

def gate_g2(rows: list[dict], src_ids: dict[str, list[int]], window: tuple[int, int] | None) -> dict:
  """Counts: no decode drops in the decoded window; modelV2 outputs == frames processed.

  The decoded window is measured from the first to the last decoded frame (the
  recorded route starts mid-GOP, so vtdec must wait for the next IDR — ~1.4 s of
  warmup at the head). Inside that window decoded count must equal the source
  frame count, i.e. zero decode drops.
  The modelV2 check uses the joined window (span with both a decoded frame and a
  modelV2 output); modeld connects after model load, so earlier frames are
  legitimately unprocessed.
  """
  def in_win(fid, w):
    return w is None or (w[0] <= fid <= w[1])

  detail = {}
  ok = True
  for cam in sorted({r["cam"] for r in rows}):
    rs = [r for r in rows if r["cam"] == cam and in_win(r["frame_id"], window)]
    decoded = [r for r in rs if r["vipc_publish_mac_ns"] is not None]
    src = [f for f in src_ids.get(cam, []) if in_win(f, window)]
    mv = [r for r in rs if r["modelv2_mac_ns"] is not None]
    # The recorded route starts mid-GOP; vtdec needs the next IDR, so the first
    # decode lands ~1.4 s in. Measure "no decode drops" over the decoded window:
    # decoded count must equal the number of source frames inside it.
    if decoded:
      dw = (min(r["frame_id"] for r in decoded), max(r["frame_id"] for r in decoded))
      expected_in_window = sum(1 for f in src if dw[0] <= f <= dw[1])
      count_ok = len(decoded) == expected_in_window
      dec_win = {"lo": dw[0], "hi": dw[1], "source_in_window": expected_in_window,
                 "decoded": len(decoded)}
    else:
      count_ok = False
      dec_win = {"decoded": 0}
    if mv:
      jw = (min(r["frame_id"] for r in mv), max(r["frame_id"] for r in mv))
      decoded_j = [r for r in decoded if jw[0] <= r["frame_id"] <= jw[1]]
      mv2_ok = len(mv) == len(decoded_j)
      joined = {"lo": jw[0], "hi": jw[1], "decoded_in_window": len(decoded_j), "modelv2": len(mv)}
    else:
      mv2_ok = False
      joined = {"modelv2": 0}
    cam_ok = count_ok and mv2_ok
    ok = ok and cam_ok
    detail[cam] = {"source_frames": len(src), "decoded": len(decoded),
                   "decoded_window": dec_win, "decoded_eq_encoded_in_window": count_ok,
                   "joined_window": joined, "modelv2_eq_decoded_in_window": mv2_ok, "ok": cam_ok}
  return {"gate": "G2", "status": "PASS" if ok and detail else ("FAIL" if detail else "SKIP"),
          "detail": detail}


# --- G6 / G7 -----------------------------------------------------------------

def gate_g6(rows: list[dict]) -> dict:
  """SOF->modelV2-published in the Mac clock domain (recv_mac_ns ~= SOF on Mac)."""
  ages, misses = [], 0
  for r in rows:
    if r["modelv2_mac_ns"] is not None and r["recv_mac_ns"] is not None:
      ages.append((r["modelv2_mac_ns"] - r["recv_mac_ns"]) / 1e6)
    elif r["modelv2_mac_ns"] is None:
      misses += 1
  if not ages:
    return {"gate": "G6", "status": "SKIP", "detail": {"n": 0, "unjoined": misses}}
  p50, p99, p999 = pct(ages, 50), pct(ages, 99), pct(ages, 99.9)
  ok = p50 <= G6_P50 and p99 <= G6_P99 and p999 <= G6_P999
  return {"gate": "G6", "status": "PASS" if ok else "FAIL",
          "detail": {"n": len(ages), "unjoined": misses, "p50_ms": round(p50, 2),
                     "p99_ms": round(p99, 2), "p99.9_ms": round(p999, 2),
                     "max_ms": round(max(ages), 2),
                     "limits": [G6_P50, G6_P99, G6_P999]}}


def gate_g7(rows: list[dict]) -> dict:
  """Decode latency. Prefer the decoder's OWN timing (decode_ms = vtdec decode_ns);
  fall back to recv->decode_done only when decode_ms is absent, so the gate measures
  decode compute, not the framebridge stdin/stdout pipe + Python thread hop."""
  dec_vt, dec_e2e = [], []
  for r in rows:
    dms = r.get("decode_ms")
    if dms is not None and dms >= 0:
      dec_vt.append(float(dms))
    if r.get("decode_done_mac_ns") is not None and r.get("recv_mac_ns") is not None:
      dec_e2e.append((r["decode_done_mac_ns"] - r["recv_mac_ns"]) / 1e6)
  dec = dec_vt if dec_vt else dec_e2e
  if not dec:
    return {"gate": "G7", "status": "SKIP", "detail": {"n": 0}}
  p50, p999 = pct(dec, 50), pct(dec, 99.9)
  ok = p50 <= G7_P50 and p999 <= G7_P999
  d = {"n": len(dec), "source": "vtdec_decode_ns" if dec_vt else "recv_to_decode_done",
       "p50_ms": round(p50, 3), "p99.9_ms": round(p999, 3), "max_ms": round(max(dec), 3),
       "limits": [G7_P50, G7_P999]}
  if dec_e2e:
    d["recv_to_decode_done_p50_ms"] = round(pct(dec_e2e, 50), 3)
    d["recv_to_decode_done_p99.9_ms"] = round(pct(dec_e2e, 99.9), 3)
  return {"gate": "G7", "status": "PASS" if ok else "FAIL", "detail": d}


# --- baselines (report only) -------------------------------------------------

def baselines_from_rlog(route_dir: str) -> dict:
  """Stock glass->modelV2 / glass->carControl / glass->sendcan, from the rlog."""
  rlog = os.path.join(route_dir, "rlog.zst")
  if not os.path.exists(rlog):
    return {}
  sof = {}
  mvt, mvf, cct, sct = [], [], [], []
  for msg in LogReader(rlog):
    w = msg.which()
    if w == "narrowRoadCameraState":
      d = msg.narrowRoadCameraState
      sof[int(d.frameId)] = int(d.timestampSof)
    elif w == "modelV2":
      mvt.append(int(msg.logMonoTime)); mvf.append(int(msg.modelV2.frameId))
    elif w == "carControl":
      cct.append(int(msg.logMonoTime))
    elif w == "sendcan":
      sct.append(int(msg.logMonoTime))

  def rep(a):
    a = np.asarray(a, dtype=float)
    if not len(a):
      return {"n": 0}
    return {"n": len(a), "p50_ms": round(pct(a, 50), 2), "p99_ms": round(pct(a, 99), 2),
            "p99.9_ms": round(pct(a, 99.9), 2)}

  g2mv = [(t - sof[mvf[i]]) / 1e6 for i, t in enumerate(mvt) if mvf[i] in sof]
  glass_cc, hop, glass_sc, age_model = [], [], [], []
  for t in cct:
    i = bisect_right(mvt, t) - 1
    if i < 0:
      continue
    age_model.append((t - mvt[i]) / 1e6)
    s = sof.get(mvf[i])
    if s is not None:
      glass_cc.append((t - s) / 1e6)
  for t in sct:
    j = bisect_right(cct, t) - 1
    if j < 0:
      continue
    hop.append((t - cct[j]) / 1e6)
    i = bisect_right(mvt, cct[j]) - 1
    if i < 0:
      continue
    s = sof.get(mvf[i])
    if s is not None:
      glass_sc.append((t - s) / 1e6)
  return {
    "counts": {"modelV2": len(mvt), "carControl": len(cct), "sendcan": len(sct), "camFrames": len(sof)},
    "glass_sof_to_modelV2": rep(g2mv),
    "modelV2_age_at_carControl": rep(age_model),
    "glass_sof_to_carControl": rep(glass_cc),
    "carControl_to_sendcan": rep(hop),
    "glass_sof_to_sendcan": rep(glass_sc),
  }


# --- driver ------------------------------------------------------------------

def analyze(latency_jsonl: str, route_or_fixture: str, window=None, replay_log=None,
            received: dict | None = None) -> dict:
  rows = join_modelv2(load_latency(latency_jsonl))
  src_ids = load_source_frame_ids(route_or_fixture)
  src_eids = {cam: load_source_encode_ids(route_or_fixture, cam) for cam in fx.CAMERAS}
  gates = [gate_g1(rows, src_ids, src_eids, received),
           gate_g2(rows, src_ids, window),
           gate_g6(rows),
           gate_g7(rows)]
  # route dir for baselines: fixture meta records route_dirs
  base_dir = route_or_fixture
  meta_path = os.path.join(route_or_fixture, "meta.json")
  if os.path.exists(meta_path):
    m = fx.read_meta(route_or_fixture)
    rds = m.get("route_dirs") or []
    if rds:
      base_dir = rds[0]
  report = {
    "latency_jsonl": os.path.abspath(latency_jsonl),
    "source": os.path.abspath(route_or_fixture),
    "n_rows": len(rows),
    "window": list(window) if window else None,
    "gates": gates,
    "baselines": baselines_from_rlog(base_dir),
  }
  if replay_log:
    report["replay_log"] = {"path": os.path.abspath(replay_log)}
  return report


def _fmt(g):
  d = g["detail"]
  if g["gate"] == "G6" and "p50_ms" in d:
    return (f"{g['gate']} {g['status']:4s} n={d['n']} SOF->modelV2 p50={d['p50_ms']} "
            f"p99={d['p99_ms']} p99.9={d['p99.9_ms']} ms (<= {G6_P50}/{G6_P99}/{G6_P999})")
  if g["gate"] == "G7" and "p50_ms" in d:
    return (f"{g['gate']} {g['status']:4s} n={d['n']} decode p50={d['p50_ms']} "
            f"p99.9={d['p99.9_ms']} ms (<= {G7_P50}/{G7_P999})")
  if g["gate"] == "G1":
    parts = [f"{c}:frames={v['frames']} dup={v['dup_frame_ids']} "
             f"gaps_mirror={v['gaps_mirror_source']} eid_mono={v['encode_id_monotone_by_1']} "
             f"1to1={v['encode_id_frame_id_1to1']}" for c, v in d.items()]
    return f"{g['gate']} {g['status']:4s} " + " | ".join(parts)
  if g["gate"] == "G2":
    parts = [f"{c}:enc={v['encoded']} dec={v['decoded']} mv2={v['modelv2_published']}"
             for c, v in d.items()]
    return f"{g['gate']} {g['status']:4s} " + " | ".join(parts)
  return f"{g['gate']} {g['status']}"


def main(argv=None) -> int:
  ap = argparse.ArgumentParser(description="Offline replay gate metrics")
  ap.add_argument("--latency", required=True, help="FrameEvent/LatencyRecord jsonl")
  ap.add_argument("--route", required=True, help="route dir or fixture dir (source of truth)")
  ap.add_argument("--replay-log", default=None, help="replayd --pub-log jsonl")
  ap.add_argument("--window", default=None, help="frameId_lo:frameId_hi")
  ap.add_argument("--json", default=None, help="write the full report json here")
  args = ap.parse_args(argv)

  window = None
  if args.window:
    lo, hi = args.window.split(":")
    window = (int(lo), int(hi))

  if not os.path.exists(args.latency):
    print(f"[metrics] ERROR: no such latency log: {args.latency}", file=sys.stderr)
    return 2
  try:
    rep = analyze(args.latency, args.route, window, args.replay_log)
  except Exception as e:  # noqa: BLE001
    print(f"[metrics] ERROR: {e!r}", file=sys.stderr)
    return 2

  for g in rep["gates"]:
    print("[metrics] " + _fmt(g))
  if args.json:
    with open(args.json, "w") as f:
      json.dump(rep, f, indent=2, sort_keys=True)
      f.write("\n")

  statuses = [g["status"] for g in rep["gates"]]
  if any(s == "FAIL" for s in statuses):
    return 1
  if all(s == "SKIP" for s in statuses):
    return 2
  return 0


if __name__ == "__main__":
  sys.exit(main())
