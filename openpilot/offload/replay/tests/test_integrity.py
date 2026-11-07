"""WS-C unit tests for the uplink byte-integrity harness (replay/integrity.py).

Covers the four required behaviors, all with small SYNTHETIC fixtures (the real
run in INTEGRITY.md covers scale):
  * fixture -> expected digest round-trip;
  * the checker catches a hand-injected corrupted byte;
  * the checker catches a duplicated row;
  * the checker catches a replayed seq (a rewind / replayed (frame_id, encode_id));
  * the checker accepts a clean run.

Run (from the worktree root):
  PYTHONPATH=$PWD:$PWD/opendbc_repo:$PWD/msgq_repo:$PWD/tinygrad_repo \\
    .venv/bin/python -m pytest openpilot/offload/replay/tests/test_integrity.py -q
"""
from __future__ import annotations

import hashlib
import json
import os

import pytest

from openpilot.offload.replay import fixture as fx
from openpilot.offload.replay import integrity as integ


# --- synthetic fixtures ------------------------------------------------------

def _make_fixture(dir_: str) -> dict:
  """Write narrow.enc + wide.enc with a handful of records; return the records."""
  recs = {"narrow": [], "wide": []}
  specs = {
    "narrow": [(10, 0, 8), (11, 1, 8), (12, 2, 0), (13, 3, 8)],
    "wide": [(10, 0, 8), (11, 1, 0), (12, 2, 8)],
  }
  for cam, rows in specs.items():
    path = os.path.join(dir_, fx.ENC_NAME[cam])
    with open(path, "wb") as f:
      for i, (fid, eid, flags) in enumerate(rows):
        au = b"\x00\x00\x00\x01\x26" + bytes([cam == "wide", i, fid & 0xFF, eid]) * 8
        sof = 1_000_000_000 + i * 50_000_000
        fx.write_record(f, fid, eid, sof, sof + 14_000_000, flags, au)
        recs[cam].append({"cam": cam, "frame_id": fid, "encode_id": eid,
                          "sha256": hashlib.sha256(au).hexdigest(), "len": len(au)})
  return recs


def _fb_rows(recs: dict, drop=(), dup=(), corrupt=(), rewind=False) -> list[dict]:
  """Build framebridge-style digest rows from fixture records (one row per frame),
  optionally dropping indices, duplicating, corrupting a digest, or replaying."""
  rows = []
  for cam in fx.CAMERAS:
    for i, r in enumerate(recs[cam]):
      if (cam, i) in drop:
        continue
      row = {"cam": cam, "frame_id": r["frame_id"], "encode_id": r["encode_id"],
             "sha256": r["sha256"], "len": r["len"], "recv_mac_ns": 1}
      if (cam, i) in corrupt:
        row = dict(row, sha256="0" * 64)
      rows.append(row)
      if (cam, i) in dup:
        rows.append(dict(row))
  if rewind:
    # a replayed sequence: ids from earlier appear again, out of order
    rows.append({"cam": "narrow", "frame_id": 10, "encode_id": 0,
                 "sha256": recs["narrow"][0]["sha256"], "len": recs["narrow"][0]["len"],
                 "recv_mac_ns": 2})
  return rows


def _write_jsonl(path, rows):
  with open(path, "w") as f:
    for r in rows:
      f.write(json.dumps(r) + "\n")


# --- expected round-trip -----------------------------------------------------

def test_expected_roundtrip(tmp_path):
  recs = _make_fixture(str(tmp_path))
  rows = integ.build_expected(str(tmp_path))
  assert len(rows) == len(recs["narrow"]) + len(recs["wide"])
  # every emitted digest must equal the sha256 of the fixture's AU bytes
  by_key = {(r["cam"], r["frame_id"], r["encode_id"]): r for r in rows}
  for cam in fx.CAMERAS:
    for r in recs[cam]:
      assert by_key[(cam, r["frame_id"], r["encode_id"])]["sha256"] == r["sha256"]
      assert by_key[(cam, r["frame_id"], r["encode_id"])]["len"] == r["len"]
  # a clean fb stream built from the same records must be a clean PASS
  res = integ.check(rows, _fb_rows(recs))
  assert res["status"] == "PASS"
  assert res["totals"] == {"expected": 7, "received": 7, "missing": 0, "mismatch": 0,
                           "duplicate": 0, "replay": 0, "regression": 0,
                           "digest_ok": 7, "digest_match_rate": 1.0}


# --- corrupted byte ----------------------------------------------------------

def test_check_catches_corrupted_byte(tmp_path):
  recs = _make_fixture(str(tmp_path))
  exp = integ.build_expected(str(tmp_path))
  fb = _fb_rows(recs, corrupt=[("narrow", 1)])
  res = integ.check(exp, fb)
  assert res["status"] == "FAIL"
  assert res["totals"]["mismatch"] == 1
  assert res["totals"]["digest_match_rate"] < 1.0
  assert res["cams"]["narrow"]["mismatch_sample"][0]["frame_id"] == 11


# --- duplicated row ----------------------------------------------------------

def test_check_catches_duplicate_row(tmp_path):
  recs = _make_fixture(str(tmp_path))
  exp = integ.build_expected(str(tmp_path))
  fb = _fb_rows(recs, dup=[("narrow", 2)])
  res = integ.check(exp, fb)
  assert res["status"] == "FAIL"
  assert res["totals"]["duplicate"] >= 1
  # a duplicate also repeats an id => a regression (non-increasing)
  assert res["totals"]["regression"] >= 1
  assert res["cams"]["narrow"]["duplicate_sample"][0] == [12, 2]


# --- replayed seq ------------------------------------------------------------

def test_check_catches_replayed_seq(tmp_path):
  recs = _make_fixture(str(tmp_path))
  exp = integ.build_expected(str(tmp_path))
  fb = _fb_rows(recs, rewind=True)
  res = integ.check(exp, fb)
  assert res["status"] == "FAIL"
  # the rewound (10,0) is a fixture frame that reappears => duplicate of an id pair
  # already seen, and non-increasing => regression.
  assert res["totals"]["duplicate"] >= 1
  assert res["totals"]["regression"] >= 1
  assert res["cams"]["narrow"]["regression_sample"][0]["frame_id"] == 10


def test_check_catches_invented_frame_as_replay(tmp_path):
  recs = _make_fixture(str(tmp_path))
  exp = integ.build_expected(str(tmp_path))
  fb = _fb_rows(recs)
  # a (frame_id, encode_id) the fixture NEVER emitted, monotonically after the rest
  fb.append({"cam": "wide", "frame_id": 999, "encode_id": 999,
             "sha256": "a" * 64, "len": 123, "recv_mac_ns": 3})
  res = integ.check(exp, fb)
  assert res["status"] == "FAIL"
  assert res["totals"]["replay"] == 1
  assert res["cams"]["wide"]["replay_sample"][0]["frame_id"] == 999


# --- missing: counted, not failed (unless strict) ----------------------------

def test_missing_counted_not_failed(tmp_path):
  recs = _make_fixture(str(tmp_path))
  exp = integ.build_expected(str(tmp_path))
  fb = _fb_rows(recs, drop=[("narrow", 0), ("narrow", 3)])
  res = integ.check(exp, fb)
  assert res["status"] == "PASS"          # a reconnect boundary drop is acceptable
  assert res["totals"]["missing"] == 2
  res_strict = integ.check(exp, fb, strict_missing=True)
  assert res_strict["status"] == "FAIL"


# --- CLI exit codes ----------------------------------------------------------

def test_cli_exit_codes(tmp_path):
  recs = _make_fixture(str(tmp_path))
  exp_path = str(tmp_path / "exp.jsonl")
  rc = integ.main(["expected", str(tmp_path), "--out", exp_path])
  assert rc == 0 and os.path.exists(exp_path)

  clean = str(tmp_path / "clean.jsonl")
  _write_jsonl(clean, _fb_rows(recs))
  assert integ.main(["check", exp_path, clean]) == 0

  bad = str(tmp_path / "bad.jsonl")
  _write_jsonl(bad, _fb_rows(recs, corrupt=[("wide", 1)]))
  assert integ.main(["check", exp_path, bad]) == 1

  # report mode always prints the summary line and returns the same code
  assert integ.main(["report", exp_path, bad]) == 1
  assert integ.main(["report", exp_path, clean]) == 0


def test_cli_bad_paths_exit2(tmp_path):
  assert integ.main(["expected", str(tmp_path / "nope")]) == 2
  exp = str(tmp_path / "e.jsonl")
  _write_jsonl(exp, [])
  assert integ.main(["check", exp, str(tmp_path / "missing.jsonl")]) == 2


# --- framebridge --digest-out flag (default path unchanged) ------------------

def test_framebridge_digest_flag_default_none():
  pytest.importorskip("msgq.visionipc")
  from openpilot.offload.mac import framebridge as fb
  ap = fb.build_argparser()
  a = ap.parse_args(["--host", "127.0.0.1"])
  assert a.digest_out is None                       # absent => today's behavior
  b = ap.parse_args(["--digest-out", "/tmp/x.jsonl"])
  assert b.digest_out == "/tmp/x.jsonl"
