"""Unit tests for the fixture reader (WS-B).

The reader's spec is frozen by openpilot/offload/mac/fixtures_README.md:
  per record, little-endian [u32 nbytes][u32 frameId][u32 encodeId][u64 sof]
  [u64 eof][u32 flags] then `nbytes` of Annex-B AU bytes.
A self-generated 10-record sample is written here so these tests do not depend
on the (large) real fixture being present.
"""
import os
import struct

import pytest

from openpilot.offload.mac import fixtures as fix

HDR = struct.Struct("<IIIQQI")


def _write_sample(path, n=10, payload_len=64):
  recs = []
  with open(path, "wb") as f:
    for i in range(n):
      au = bytes([j & 0xFF for j in range(payload_len)])  # arbitrary AU bytes
      frame_id = 43206 + i
      encode_id = 43200 + i
      sof = 2_768_180_453_902 + i * 50_000_000
      eof = sof + 14_697_000
      flags = 0x80004008 if i % 30 == 0 else 0x4010
      f.write(HDR.pack(len(au), frame_id, encode_id, sof, eof, flags))
      f.write(au)
      recs.append((au, frame_id, encode_id, sof, eof, flags))
  return recs


def test_roundtrip(tmp_path):
  p = str(tmp_path / "narrow.enc")
  recs = _write_sample(p, n=10, payload_len=64)
  got = fix.read_fixture(p)
  assert len(got) == 10
  for (au, fid, eid, sof, eof, flags), r in zip(recs, got):
    assert r.data == au
    assert (r.frame_id, r.encode_id, r.timestamp_sof, r.timestamp_eof, r.flags) == (fid, eid, sof, eof, flags)
    assert r.nbytes == len(au)
  assert got[0].is_keyframe is True
  assert got[1].is_keyframe is False


def test_iter_matches_read(tmp_path):
  p = str(tmp_path / "wide.enc")
  _write_sample(p, n=7)
  assert [r.frame_id for r in fix.iter_fixture(p)] == [43206 + i for i in range(7)]


def test_max_frames(tmp_path):
  p = str(tmp_path / "narrow.enc")
  _write_sample(p, n=10)
  assert len(fix.read_fixture(p, max_frames=3)) == 3
  # max_frames full read validation is skipped, so a truncated file still works
  assert [r.frame_id for r in fix.iter_fixture(p, max_frames=2)] == [43206, 43207]


def test_truncated_header(tmp_path):
  p = str(tmp_path / "bad.enc")
  with open(p, "wb") as f:
    f.write(b"\x00\x01\x02")  # 3 bytes, short of a 32-byte header
  with pytest.raises(fix.FixtureFormatError):
    fix.read_fixture(p)


def test_truncated_payload(tmp_path):
  p = str(tmp_path / "bad2.enc")
  with open(p, "wb") as f:
    f.write(HDR.pack(1000, 1, 1, 1, 2, 8))  # claims 1000 AU bytes, writes none
  with pytest.raises(fix.FixtureFormatError):
    fix.read_fixture(p)


def test_trailing_garbage(tmp_path):
  p = str(tmp_path / "bad3.enc")
  _write_sample(p, n=2)
  with open(p, "ab") as f:
    f.write(b"\x00\x00\x00")  # 3 stray bytes: full read must reject
  with pytest.raises(fix.FixtureFormatError):
    fix.read_fixture(p)


def test_monotone_ids_real_sample(tmp_path):
  p = str(tmp_path / "narrow.enc")
  _write_sample(p, n=50)
  recs = fix.read_fixture(p)
  assert all(recs[i + 1].frame_id - recs[i].frame_id == 1 for i in range(len(recs) - 1))
  assert all(recs[i + 1].encode_id - recs[i].encode_id == 1 for i in range(len(recs) - 1))
