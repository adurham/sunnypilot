#!/usr/bin/env python3
"""WS-C uplink byte-integrity harness (see replay/INTEGRITY.md, COMPARISON-COMMS R11).

This closes the gap that we had ZERO byte-integrity evidence for the ZMQ/NCM
uplink we actually ship on. The idea is borrowed from jetlink's
`scripts/comma/jetlink_usb_integrity.py` (digest + frame/offset stamp + forced
stress + zero-replays assertion) but reimplemented against OUR path: fixtures ->
replayd (ZMQ PUB) -> framebridge (ZMQ SUB, --digest-out) -> this checker. The
USB-vendor specifics are deliberately NOT ported.

Modes (all offline):

  expected <fixture_dir> --out <expected.jsonl>
      Walk a fixture with fixture.py and emit one jsonl row per record per camera:
      {"cam","frame_id","encode_id","sha256","len"}. Ground truth = the digest of
      the EXACT AU bytes the device puts in EncodeData.data.

  check <expected.jsonl> <fb.jsonl>
      Consume framebridge's --digest-out jsonl (one row per received EncodeData) and
      verify, per camera and across the WHOLE run (including across consumer
      reconnects):
        * for every received frame the sha256 of the received payload equals the
          fixture' digest for that (frame_id, encode_id);
        * frame_id and encode_id are STRICTLY increasing with zero duplicates and
          zero regressions anywhere in the run;
        * count received vs expected and classify missing (never received) vs
          mismatched vs duplicated vs replayed.
      Exit non-zero on ANY mismatch / duplicate / replay / regression. Missing
      frames are counted, not failed by default (a reconnecting consumer drops the
      boundary), unless --strict-missing.

  report <expected.jsonl> <fb.jsonl>
      check --summary alias; prints one machine-readable summary line (used by
      integrity_run.sh) and still exits the same way.

Exit codes: 0 clean, 1 any mismatch/dup/replay/regression (or missing with
--strict-missing), 2 usage / IO / format error.

Usage (from the worktree root):
  PYTHONPATH=$PWD:$PWD/opendbc_repo:$PWD/msgq_repo:$PWD/tinygrad_repo \\
    .venv/bin/python -m openpilot.offload.replay.integrity expected <fixture_dir> --out expected.jsonl
  ... check <expected.jsonl> <fb_digest.jsonl> [--json summary.json] [--summary]
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
from collections import defaultdict


def log(msg: str) -> None:
  print(f"[integrity] {msg}", flush=True)


# --- mode: expected ----------------------------------------------------------

def build_expected(fixture_dir: str) -> list[dict]:
  """One row per fixture record per camera, in fixture file order (cameras are
  grouped, not interleaved; the checker groups by cam regardless)."""
  from openpilot.offload.replay import fixture as fx
  rows: list[dict] = []
  for cam in fx.CAMERAS:
    enc = os.path.join(fixture_dir, fx.ENC_NAME[cam])
    if not os.path.exists(enc):
      raise FileNotFoundError(f"{fixture_dir}: missing {fx.ENC_NAME[cam]}")
    for rec in fx.read_records(enc):
      rows.append({
        "cam": cam,
        "frame_id": int(rec.frame_id),
        "encode_id": int(rec.encode_id),
        "sha256": hashlib.sha256(rec.au).hexdigest(),
        "len": len(rec.au),
      })
  return rows


def cmd_expected(args) -> int:
  fixture_dir = os.path.abspath(os.path.expanduser(args.fixture_dir))
  if not os.path.isdir(fixture_dir):
    log(f"ERROR: not a directory: {fixture_dir}")
    return 2
  try:
    rows = build_expected(fixture_dir)
  except (OSError, ValueError) as e:
    log(f"ERROR: {e!r}")
    return 2
  out = open(args.out, "w") if args.out else sys.stdout
  try:
    for r in rows:
      out.write(json.dumps(r, separators=(",", ":")) + "\n")
  finally:
    if args.out:
      out.close()
  if args.out:
    log(f"expected: {len(rows)} rows -> {args.out}")
  return 0


# --- mode: check -------------------------------------------------------------

def _load_jsonl(path: str, allow_trailing_partial: bool = False) -> list[dict]:
  rows = []
  with open(path) as f:
    lines = f.readlines()
  for ln, line in enumerate(lines, 1):
    line = line.strip()
    if not line:
      continue
    try:
      rows.append(json.loads(line))
    except json.JSONDecodeError as e:
      # A stress kill can leave the final line half-written; tolerate ONLY that one,
      # loudly, so it is visible (a truncated digest row would otherwise hide bytes).
      if allow_trailing_partial and ln == len(lines):
        log(f"WARNING: {path}:{ln}: trailing partial line ignored ({e})")
        continue
      raise ValueError(f"{path}:{ln}: bad json ({e})") from e
  return rows


def _normalize_fb(rows: list[dict]) -> list[dict]:
  """Accept framebridge --digest-out rows; tolerate frameId/encodeId aliases."""
  out = []
  for r in rows:
    cam = r.get("cam")
    fid = r.get("frame_id", r.get("frameId"))
    eid = r.get("encode_id", r.get("encodeId"))
    dig = r.get("sha256", r.get("digest"))
    n = r.get("len", r.get("payload_bytes"))
    if cam is None or fid is None or eid is None or dig is None:
      raise ValueError(f"received row missing fields: {r!r}")
    out.append({
      "cam": cam, "frame_id": int(fid), "encode_id": int(eid),
      "sha256": str(dig), "len": int(n) if n is not None else -1,
      "recv_mac_ns": r.get("recv_mac_ns"),
    })
  return out


def check(expected_rows: list[dict], fb_rows: list[dict], strict_missing: bool = False) -> dict:
  exp_by_cam: dict[str, dict[tuple[int, int], dict]] = defaultdict(dict)
  for r in expected_rows:
    exp_by_cam[r["cam"]][(int(r["frame_id"]), int(r["encode_id"]))] = r

  fb_by_cam: dict[str, list[dict]] = defaultdict(list)
  for r in fb_rows:
    fb_by_cam[r["cam"]].append(r)

  detail: dict[str, dict] = {}
  total: dict[str, float] = {"expected": 0, "received": 0, "missing": 0, "mismatch": 0,
                             "duplicate": 0, "replay": 0, "regression": 0, "digest_ok": 0}
  bad = False

  for cam in sorted(set(exp_by_cam) | set(fb_by_cam)):
    exp = exp_by_cam.get(cam, {})
    rows = fb_by_cam.get(cam, [])
    seen: set[tuple[int, int]] = set()
    dup_keys: list[tuple[int, int]] = []
    mismatch_rows: list[dict] = []
    replay_rows: list[dict] = []
    regression_rows: list[dict] = []
    digest_ok = 0

    last_fid: int | None = None
    last_eid: int | None = None
    for r in rows:
      fid, eid = r["frame_id"], r["encode_id"]
      key = (fid, eid)
      e = exp.get(key)

      if key in seen:
        dup_keys.append(key)
      seen.add(key)

      # Strictly increasing frame_id and encode_id anywhere in the run. A repeat of
      # an already-seen pair is a duplicate; a lower id is a regression/rewind.
      if last_fid is not None and fid <= last_fid:
        regression_rows.append({"frame_id": fid, "prev_frame_id": last_fid, "encode_id": eid})
      if last_eid is not None and eid <= last_eid:
        regression_rows.append({"frame_id": fid, "encode_id": eid, "prev_encode_id": last_eid})
      last_fid, last_eid = fid, eid

      if e is None:
        # A (frame_id, encode_id) the fixture never emitted: a replay of a sequence
        # (ids from a rewind) or an invented frame.
        replay_rows.append({"frame_id": fid, "encode_id": eid,
                            "sha256": r["sha256"], "len": r["len"]})
        continue
      if r["sha256"] == e["sha256"]:
        digest_ok += 1
      else:
        mismatch_rows.append({"frame_id": fid, "encode_id": eid,
                              "got": r["sha256"], "want": e["sha256"],
                              "got_len": r["len"], "want_len": e["len"]})

    missing = [k for k in exp if k not in seen]
    n_exp, n_recv = len(exp), len(rows)
    n_missing, n_mismatch = len(missing), len(mismatch_rows)
    n_dup, n_replay, n_reg = len(dup_keys), len(replay_rows), len(regression_rows)
    cam_bad = n_mismatch > 0 or n_replay > 0 or n_reg > 0 or (strict_missing and n_missing > 0)
    bad = bad or cam_bad
    detail[cam] = {
      "expected": n_exp, "received": n_recv, "missing": n_missing,
      "mismatch": n_mismatch, "duplicate": n_dup, "replay": n_replay,
      "regression": n_reg, "digest_ok": digest_ok,
      "digest_match_rate": round(digest_ok / n_recv, 6) if n_recv else 1.0,
      "missing_frame_ids": [k[0] for k in missing[:50]],
      "mismatch_sample": mismatch_rows[:5],
      "replay_sample": replay_rows[:5],
      "duplicate_sample": [list(k) for k in dup_keys[:5]],
      "regression_sample": regression_rows[:5],
      "strict_missing": strict_missing,
      "ok": not cam_bad,
    }
    for k in ("expected", "received", "missing", "mismatch", "duplicate", "replay", "regression", "digest_ok"):
      total[k] += detail[cam][k]

  total["digest_match_rate"] = round(total["digest_ok"] / total["received"], 6) if total["received"] else 1.0
  return {"status": "FAIL" if bad else "PASS", "cams": detail, "totals": total}


def cmd_check(args) -> int:
  try:
    expected_rows = _load_jsonl(args.expected)
    fb_rows = _normalize_fb(_load_jsonl(args.fb, allow_trailing_partial=True))
  except (OSError, ValueError) as e:
    log(f"ERROR: {e!r}")
    return 2
  if not expected_rows:
    log("ERROR: expected.jsonl is empty")
    return 2
  res = check(expected_rows, fb_rows, strict_missing=args.strict_missing)
  res["expected_jsonl"] = os.path.abspath(args.expected)
  res["fb_jsonl"] = os.path.abspath(args.fb)
  t = res["totals"]
  log(f"{res['status']}  frames: expected={t['expected']} received={t['received']} "
      + f"missing={t['missing']} mismatch={t['mismatch']} dup={t['duplicate']} "
      + f"replay={t['replay']} regression={t['regression']} "
      + f"digest_match_rate={t['digest_match_rate']}")
  for cam, d in res["cams"].items():
    log(f"  {cam}: exp={d['expected']} recv={d['received']} miss={d['missing']} "
        + f"mismatch={d['mismatch']} dup={d['duplicate']} replay={d['replay']} "
        + f"rate={d['digest_match_rate']}")
  if args.json:
    with open(args.json, "w") as f:
      json.dump(res, f, indent=2, sort_keys=True)
      f.write("\n")
  if args.summary:
    print(json.dumps({"status": res["status"], **t}, separators=(",", ":")))
  return 0 if res["status"] == "PASS" else 1


# --- main --------------------------------------------------------------------

def build_argparser() -> argparse.ArgumentParser:
  ap = argparse.ArgumentParser(description="uplink byte-integrity harness")
  sub = ap.add_subparsers(dest="mode", required=True)

  pe = sub.add_parser("expected", help="fixture -> expected per-record digests")
  pe.add_argument("fixture_dir")
  pe.add_argument("--out", default=None, help="output jsonl (default stdout)")
  pe.set_defaults(func=cmd_expected)

  pc = sub.add_parser("check", help="verify framebridge digest jsonl against expected")
  pc.add_argument("expected")
  pc.add_argument("fb")
  pc.add_argument("--json", default=None, help="write the full result json here")
  pc.add_argument("--summary", action="store_true", help="print one machine-readable summary line")
  pc.add_argument("--strict-missing", action="store_true",
                  help="fail on missing frames too (default: missing is counted, not failed)")
  pc.set_defaults(func=cmd_check)

  pr = sub.add_parser("report", help="check + always print the summary line")
  pr.add_argument("expected")
  pr.add_argument("fb")
  pr.add_argument("--json", default=None)
  pr.add_argument("--strict-missing", action="store_true")
  pr.set_defaults(func=cmd_check, summary=True)
  return ap


def main(argv=None) -> int:
  args = build_argparser().parse_args(argv)
  return args.func(args)


if __name__ == "__main__":
  sys.exit(main())
