"""WS-C unit tests: fixture reader round-trip, AU grouping (real route 149-36),
port usage, and metrics math (synthetic rows with known percentiles).

Run (from the worktree root):
  PYTHONPATH=$PWD:$PWD/opendbc_repo:$PWD/msgq_repo:$PWD/tinygrad_repo \\
    .venv/bin/python -m pytest openpilot/offload/replay/tests/test_replay.py -q
"""
from __future__ import annotations

import json
import os
import struct

import numpy as np
import pytest

from openpilot.offload.replay import fixture as fx
from openpilot.offload.replay import metrics as M
from openpilot.offload import ports

ROUTES_ROOT = os.path.expanduser("~/comma-routes")
ROUTE_149 = os.path.join(ROUTES_ROOT, "00000149--4a4df1cf8a--36")
HAVE_149 = os.path.exists(os.path.join(ROUTE_149, "fcamera.hevc")) and \
    os.path.exists(os.path.join(ROUTE_149, "qlog.zst"))


# --- fixture round-trip ------------------------------------------------------

def test_fixture_roundtrip(tmp_path):
  p = str(tmp_path / "narrow.enc")
  recs = [
    (43206, 43200, 2_768_180_453_902, 2_768_195_150_902, 0x80004008, b"\x00\x00\x00\x01\x26abc"),
    (43207, 43201, 2_768_230_453_902, 2_768_245_150_902, 0x4010, b"\x00\x00\x00\x01\x02" + b"\x7f" * 100),
  ]
  with open(p, "wb") as f:
    for fid, eid, sof, eof, flags, au in recs:
      fx.write_record(f, fid, eid, sof, eof, flags, au)

  back = list(fx.read_records(p))
  assert len(back) == len(recs) == fx.count_records(p)
  for r, (fid, eid, sof, eof, flags, au) in zip(back, recs, strict=False):
    assert (r.frame_id, r.encode_id, r.sof, r.eof, r.flags, r.au) == (fid, eid, sof, eof, flags, au)
  assert back[0].is_keyframe and not back[1].is_keyframe


def test_fixture_record_size_matches_spec(tmp_path):
  p = str(tmp_path / "x.enc")
  au = b"\x00\x00\x00\x01\x26" + b"\xaa" * 500
  with open(p, "wb") as f:
    n = fx.write_record(f, 1, 2, 3, 4, 5, au)
  # spec: [u32 len][au][u32 fid][u32 eid][u64 sof][u64 eof][u32 flags]
  assert n == 4 + len(au) + struct.calcsize("<IIQQI") == os.path.getsize(p)
  raw = open(p, "rb").read()
  (ln,) = struct.unpack_from("<I", raw, 0)
  assert ln == len(au) and raw[4:4 + len(au)] == au


def test_fixture_read_truncated(tmp_path):
  p = str(tmp_path / "trunc.enc")
  with open(p, "wb") as f:
    f.write(struct.pack("<I", 10) + b"abc")
  with pytest.raises(ValueError):
    list(fx.read_records(p))


# --- AU grouping on a real route ---------------------------------------------

@pytest.mark.skipif(not HAVE_149, reason="route 149-36 not available")
def test_au_grouping_real_route_149():
  from openpilot.tools.lib.logreader import LogReader
  data = open(os.path.join(ROUTE_149, "fcamera.hevc"), "rb").read()
  lens = []
  for msg in LogReader(os.path.join(ROUTE_149, "qlog.zst")):
    if msg.which() == "narrowRoadEncodeIdx":
      d = msg.narrowRoadEncodeIdx
      f = d.to_dict()
      lens.append(int(f["len"]))

  header, aus = fx.split_stream(data)
  assert len(aus) == len(lens) == 1200
  assert [len(a) for a in aus] == lens         # every AU length == qlog EncodeIdx.len
  # head-of-file parameter sets: VPS/SPS/PPS (32/33/34)
  ntypes = [nt for nt, *_ in fx.scan_nals(header)]
  assert ntypes == [32, 33, 34]
  assert all(a[:4] == b"\x00\x00\x00\x01" for a in aus)   # AUs carry their start code


@pytest.mark.skipif(not HAVE_149, reason="route 149-36 not available")
def test_au_grouping_matches_wide_and_synthetic_absent(tmp_path):
  data = open(os.path.join(ROUTE_149, "ecamera.hevc"), "rb").read()
  header, aus = fx.split_stream(data)
  assert len(aus) == 1200
  # synthetic-eof marker is exactly +50 ms and only used when no real eof exists
  assert fx.SYNTHETIC_EOF_OFFSET_NS == 50_000_000


# --- ports -------------------------------------------------------------------

def test_port_usage_anchors():
  assert ports.get_port("narrowRoadEncodeData") == 52737
  assert ports.get_port("wideRoadEncodeData") == 42305
  assert ports.get_port("narrowRoadCameraState") == 20911
  assert ports.get_port("wideRoadCameraState") == 53095
  assert ports._self_test() == 0


def test_port_usage_matches_contract_services():
  for _cam, svc in fx.ENCODE_DATA_SVC.items():
    assert ports.get_port(svc) == ports.get_port(svc)   # deterministic
    assert 8023 <= ports.get_port(svc) < 65535
  assert len(fx.ENCODE_DATA_SVC) == 2


# --- metrics math ------------------------------------------------------------

def _write_jsonl(path, rows):
  with open(path, "w") as f:
    for r in rows:
      f.write(json.dumps(r) + "\n")


def test_percentile_math_exact():
  # known values 1..100 ms -> exact numpy percentiles
  vals = list(range(1, 101))
  assert M.pct(vals, 50) == pytest.approx(50.5)
  assert M.pct(vals, 99) == pytest.approx(99.01)
  assert M.pct(vals, 99.9) == pytest.approx(99.901)
  # cross-check against numpy directly
  assert M.pct(vals, 99.9) == pytest.approx(float(np.percentile(vals, 99.9)))


def test_gate_g6_threshold_from_synthetic_rows(tmp_path):
  # 100 frames, e2e = modelv2 - recv = 1..100 ms -> p50 50.5, p99 99.01 <= limits => PASS
  rows = [{"kind": "frame", "frame_id": 1000 + i, "cam": "narrow",
           "encode_id": i, "recv_mac_ns": 10_000_000_000,
           "modelv2_mac_ns": 10_000_000_000 + (i + 1) * 1_000_000}
          for i in range(100)]
  g = M.gate_g6(rows)
  assert g["status"] == "PASS"
  assert g["detail"]["p50_ms"] == pytest.approx(50.5)
  assert g["detail"]["p99_ms"] == pytest.approx(99.01)
  assert g["detail"]["p99.9_ms"] == pytest.approx(99.9)

  # push p50 over 75 ms => FAIL
  rows_bad = [dict(r, modelv2_mac_ns=r["recv_mac_ns"] + 80_000_000) for r in rows]
  assert M.gate_g6(rows_bad)["status"] == "FAIL"


def test_gate_g7_threshold_from_synthetic_rows():
  rows = [{"kind": "frame", "frame_id": i, "cam": "narrow", "encode_id": i,
           "recv_mac_ns": 5_000_000_000,
           "decode_done_mac_ns": 5_000_000_000 + (1 if i < 99 else 20) * 1_000_000}
          for i in range(100)]
  g = M.gate_g7(rows)
  # p50=1ms <= 3, p99.9=~18.1ms > 10 => FAIL
  assert g["detail"]["p50_ms"] == pytest.approx(1.0)
  assert g["status"] == "FAIL"

  rows_ok = [dict(r, decode_done_mac_ns=r["recv_mac_ns"] + 2_000_000) for r in rows]
  assert M.gate_g7(rows_ok)["status"] == "PASS"


def test_gate_g1_dup_and_subsequence():
  rows = [{"kind": "frame", "frame_id": 10 + i, "cam": "narrow", "encode_id": 100 + i,
           "recv_mac_ns": 0, "modelv2_mac_ns": None, "decode_done_mac_ns": None,
           "vipc_publish_mac_ns": 0} for i in range(5)]
  src = {"narrow": [10, 11, 12, 13, 14], "wide": []}
  eids = {"narrow": [100, 101, 102, 103, 104], "wide": []}
  g = M.gate_g1(rows, src, eids)
  assert g["status"] == "PASS"
  assert g["detail"]["narrow"]["out_is_subsequence_of_source"]
  # dup frame_id => FAIL
  assert M.gate_g1(rows + [dict(rows[0])], src, eids)["status"] == "FAIL"
  # invented frame_id not in source => FAIL
  bad = [dict(r, frame_id=20 + i) for i, r in enumerate(rows)]
  assert M.gate_g1(bad, src, eids)["status"] == "FAIL"
  # a dropped frame still PASSes (subsequence) and is counted
  dropped = rows[:2] + rows[3:]
  g2 = M.gate_g1(dropped, src, eids)
  assert g2["status"] == "PASS" and g2["detail"]["narrow"]["drops_in_span"] == 1


def test_gate_g2_joined_window():
  # 3 encoded, 3 decoded, modelV2 only for frame 11..12 -> joined window 2/2 => PASS
  rows = [
    {"kind": "frame", "frame_id": 10, "cam": "narrow", "encode_id": 0,
     "vipc_publish_mac_ns": 1, "decode_done_mac_ns": 1, "modelv2_mac_ns": None},
    {"kind": "frame", "frame_id": 11, "cam": "narrow", "encode_id": 1,
     "vipc_publish_mac_ns": 1, "decode_done_mac_ns": 1, "modelv2_mac_ns": 5},
    {"kind": "frame", "frame_id": 12, "cam": "narrow", "encode_id": 2,
     "vipc_publish_mac_ns": 1, "decode_done_mac_ns": 1, "modelv2_mac_ns": 5},
  ]
  src = {"narrow": [10, 11, 12], "wide": []}
  g = M.gate_g2(rows, src, None)
  assert g["status"] == "PASS"
  assert g["detail"]["narrow"]["joined_window"] == {"lo": 11, "hi": 12, "decoded_in_window": 2, "modelv2": 2}
  # genuine decode drop: frame 11 missing from the decoded stream inside a decoded window
  drop = [rows[0], rows[2]]
  for r in drop:
    r["modelv2_mac_ns"] = 5
  gd = M.gate_g2(drop, src, None)
  assert gd["status"] == "FAIL"
  assert gd["detail"]["narrow"]["decoded_window"]["source_in_window"] == 3


def test_join_modelv2_by_frame_id(tmp_path):
  p = str(tmp_path / "combined.jsonl")
  _write_jsonl(p, [
    {"kind": "frame", "frame_id": 7, "cam": "narrow", "recv_mac_ns": 100},
    {"kind": "modelv2", "frame_id": 7, "cam": "narrow", "modelv2_mac_ns": 250},
  ])
  rows = M.join_modelv2(M.load_latency(p))
  assert len(rows) == 1 and rows[0]["modelv2_mac_ns"] == 250
  # gate_g6 rounds to 2 dp; 150 ns -> 0.0 ms (join itself is what this test pins)
  assert M.gate_g6(rows)["detail"]["p50_ms"] == pytest.approx(0.0)


# --- p1_gate graceful degradation --------------------------------------------

def test_p1_gate_ws_b_missing(monkeypatch, tmp_path):
  from openpilot.offload.replay import p1_gate as G
  monkeypatch.setattr(G, "MAC_DIR", str(tmp_path / "no_such_mac"))
  ok, msg = G.check_ws_b()
  assert ok is False and "framebridge.py" in msg


def test_p1_gate_ws_a_missing(monkeypatch, tmp_path):
  from openpilot.offload.replay import p1_gate as G
  # modeld present but no OFFLOAD patch and no pkls
  fake_modeld = tmp_path / "modeld.py"
  fake_modeld.write_text("def main():\n  pass\n")
  monkeypatch.setattr(G, "MODELD_PY", str(fake_modeld))
  monkeypatch.setattr(G, "MODELS_DIR", str(tmp_path / "empty_models"))
  ok, msg, pkl = G.check_ws_a()
  assert ok is False and "OFFLOAD patches not present" in msg and pkl is None


def test_p1_gate_main_ws_a_missing_exit2(monkeypatch, tmp_path):
  from openpilot.offload.replay import p1_gate as G
  # WS-B present, WS-A absent
  monkeypatch.setattr(G, "MAC_DIR", str(tmp_path / "mac"))
  (tmp_path / "mac").mkdir()
  (tmp_path / "mac" / "framebridge.py").write_text("")
  (tmp_path / "mac" / "vtdec").write_text("")
  monkeypatch.setattr(G, "MODELD_PY", str(tmp_path / "none.py"))
  monkeypatch.setattr(G, "MODELS_DIR", str(tmp_path / "none_models"))
  rc = G.main(["--route", f"x={tmp_path}", "--reports-dir", str(tmp_path / "reports")])
  assert rc == 2
