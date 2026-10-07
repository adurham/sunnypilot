"""WS-C — frozen on-disk fixture format for offline replay.

FROZEN SPEC (see openpilot/offload/INTERFACES.md §1/§3; do not change without a
CHANGES REQUESTED note to PM):

  fixtures/<route>/narrow.enc   road  camera, concatenated records
  fixtures/<route>/wide.enc     wide  camera, concatenated records
  fixtures/<route>/narrow.header  raw H.265 parameter-set preamble (VPS/SPS/PPS)
  fixtures/<route>/wide.header    raw H.265 parameter-set preamble
  fixtures/<route>/meta.json      provenance + hashes

Each .enc record is (little-endian):

    [u32 len][len bytes AU][u32 frameId][u32 encodeId][u64 sof][u64 eof][u32 flags]

  len       = AU byte length == the device's EncodeIdx `len` field (asserted at
              extraction time; a mismatch is a HARD error and the fixture is bad)
  AU bytes  = the access unit's VCL slices *including their Annex-B start codes*,
              parameter sets EXCLUDED (they live in the .header companion).
              This is exactly what the device puts in EncodeData.data.
  frameId   from qlog <cam>EncodeIdx
  encodeId  from qlog <cam>EncodeIdx
  sof       from qlog <cam>EncodeIdx timestampSof      (device clock, ns)
  eof       matching rlog <cam>CameraState timestampEof when present and >0,
            else sof + SYNTHETIC_EOF_OFFSET_NS (50 ms) and counted in meta.json
            under eof_source.<cam>.synthetic
  flags     from qlog <cam>EncodeIdx flags

The .header companion is the concatenation (with start codes) of every
parameter-set NAL (VPS=32 / SPS=33 / PPS=34) found in the elementary stream.
For the recorded routes here these all appear once, at the head of the file.

AU grouping rule (HEVC):
  * NALs with nal_type in {32,33,34} are parameter sets -> never part of an AU.
  * a NAL with nal_type < 32 is a VCL slice; a NEW access unit begins at a slice
    whose `first_slice_segment_in_pic_flag` == 1 (the MSB of the byte after the
    2-byte NAL header). Slices with the flag == 0 belong to the current AU.
  * AUs are aligned 1:1, in order, to the qlog EncodeIdx records.
"""
from __future__ import annotations

import hashlib
import json
import os
import struct
import time
from dataclasses import dataclass
from collections.abc import Iterator

SYNTHETIC_EOF_OFFSET_NS = 50_000_000  # 50 ms marker when no real eof exists
FORMAT_ID = "offload-replay-fixture/1"

PARAM_NAL_TYPES = frozenset({32, 33, 34})  # VPS, SPS, PPS
V4L2_BUF_FLAG_KEYFRAME = 8

CAMERAS = ("narrow", "wide")
ENC_NAME = {"narrow": "narrow.enc", "wide": "wide.enc"}
HDR_NAME = {"narrow": "narrow.header", "wide": "wide.header"}
# qlog / rlog service names
ENCODE_IDX_SVC = {"narrow": "narrowRoadEncodeIdx", "wide": "wideRoadEncodeIdx"}
CAMERA_STATE_SVC = {"narrow": "narrowRoadCameraState", "wide": "wideRoadCameraState"}
# ZMQ EncodeData service published by replayd
ENCODE_DATA_SVC = {"narrow": "narrowRoadEncodeData", "wide": "wideRoadEncodeData"}
FRAME_W = 1928
FRAME_H = 1208


# --- record I/O --------------------------------------------------------------

_REC_HEAD_TAIL = struct.Struct("<I")            # len
_REC_META = struct.Struct("<IIQQI")             # frameId, encodeId, sof, eof, flags


@dataclass(slots=True)
class FixtureRecord:
  """One access unit + its device metadata, as read back from a .enc file."""
  frame_id: int
  encode_id: int
  sof: int
  eof: int
  flags: int
  au: bytes

  @property
  def is_keyframe(self) -> bool:
    return bool(self.flags & V4L2_BUF_FLAG_KEYFRAME)


def write_record(fh, frame_id: int, encode_id: int, sof: int, eof: int, flags: int, au: bytes) -> int:
  """Append one record; returns bytes written."""
  fh.write(_REC_HEAD_TAIL.pack(len(au)))
  fh.write(au)
  fh.write(_REC_META.pack(frame_id, encode_id, sof, eof, flags))
  return 4 + len(au) + _REC_META.size


def read_records(path: str) -> Iterator[FixtureRecord]:
  """Stream records from a .enc file. Raises ValueError on a truncated record."""
  with open(path, "rb") as f:
    while True:
      head = f.read(4)
      if not head:
        return
      if len(head) != 4:
        raise ValueError(f"{path}: truncated record header")
      (n,) = _REC_HEAD_TAIL.unpack(head)
      au = f.read(n)
      if len(au) != n:
        raise ValueError(f"{path}: truncated AU payload ({len(au)}/{n})")
      tail = f.read(_REC_META.size)
      if len(tail) != _REC_META.size:
        raise ValueError(f"{path}: truncated record tail")
      fid, eid, sof, eof, flags = _REC_META.unpack(tail)
      yield FixtureRecord(fid, eid, sof, eof, flags, au)


def count_records(path: str) -> int:
  n = 0
  with open(path, "rb") as f:
    while True:
      head = f.read(4)
      if not head:
        return n
      (ln,) = _REC_HEAD_TAIL.unpack(head)
      f.seek(ln + _REC_META.size, os.SEEK_CUR)
      n += 1


def sha256_file(path: str, chunk: int = 1 << 20) -> str:
  h = hashlib.sha256()
  with open(path, "rb") as f:
    for blk in iter(lambda: f.read(chunk), b""):
      h.update(blk)
  return h.hexdigest()


def read_header(path: str) -> bytes:
  try:
    with open(path, "rb") as f:
      return f.read()
  except FileNotFoundError:
    return b""


# --- HEVC scanning / AU grouping --------------------------------------------

def scan_nals(data: bytes):
  """Return [(nal_type, abs_off, total_len_incl_sc, payload_off, payload_len)]."""
  n = len(data)
  out = []
  i = 0
  while i < n - 3:
    if data[i] == 0 and data[i + 1] == 0 and data[i + 2] == 1:
      sc = 3
    elif i < n - 4 and data[i] == 0 and data[i + 1] == 0 and data[i + 2] == 0 and data[i + 3] == 1:
      sc = 4
    else:
      i += 1
      continue
    s = i + sc
    j = i + sc
    while j < n - 3 and not (data[j] == 0 and data[j + 1] == 0 and data[j + 2] <= 1):
      j += 1
    if j >= n - 3:
      j = n
    out.append(((data[s] >> 1) & 0x3F, i, j - i, s, j - s))
    i = j
  return out


def split_stream(data: bytes):
  """Split an Annex-B elementary stream into (header_bytes, [au_bytes, ...]).

  header_bytes  = leading parameter-set NALs (VPS/SPS/PPS), with start codes.
  AUs           = one bytes object per access unit (VCL slices + start codes).
  """
  nals = scan_nals(data)
  header_parts: list[bytes] = []
  aus: list[bytes] = []
  cur: list[tuple[int, int]] = []   # (abs_off, total_len_incl_sc)

  for nt, off, tot, s, pl in nals:
    if nt in PARAM_NAL_TYPES:
      if cur:                       # parameters after a prior VCL -> flush AU
        aus.append(b"".join(data[o:o + t] for o, t in cur))
        cur = []
      header_parts.append(data[off:off + tot])
      continue
    # VCL slice
    fss = (data[s + 2] >> 7) & 1 if pl >= 3 else 1
    if fss == 1 and cur:
      aus.append(b"".join(data[o:o + t] for o, t in cur))
      cur = []
    cur.append((off, tot))
  if cur:
    aus.append(b"".join(data[o:o + t] for o, t in cur))
  return b"".join(header_parts), aus


# --- meta --------------------------------------------------------------------

def write_meta(path: str, meta: dict) -> None:
  tmp = path + ".tmp"
  with open(tmp, "w") as f:
    json.dump(meta, f, indent=2, sort_keys=True)
    f.write("\n")
  os.replace(tmp, path)


def read_meta(fixture_dir: str) -> dict:
  with open(os.path.join(fixture_dir, "meta.json")) as f:
    return json.load(f)


def utc_now() -> str:
  return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
