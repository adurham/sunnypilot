#!/usr/bin/env python3
"""fixtures.py — reader for the WS-B/WD-C replay fixture format.

Record layout (little-endian, no file header/trailer), repeated:
  [u32 nbytes][u32 frameId][u32 encodeId][u64 timestampSof][u64 timestampEof][u32 flags]
  followed by `nbytes` bytes of the access unit's Annex-B HEVC payload.

Fixed 32-byte header = struct "<IIIQQI".  See openpilot/offload/mac/fixtures_README.md
and openpilot/offload/mac/make_fixture.py (the writer).

`nbytes` counts only the AU payload (VPS/SPS/PPS are NOT part of a record; a
consumer that lacks stream config must be handed the parameter sets separately).
"""
from __future__ import annotations

import os
import struct
from dataclasses import dataclass
from functools import lru_cache

HDR = struct.Struct("<IIIQQI")   # 32 bytes
HDR_SIZE = HDR.size

# V4L2_BUF_FLAG_KEYFRAME, as stored in EncodeIndex.flags on the wire.
V4L2_BUF_FLAG_KEYFRAME = 8


@dataclass(slots=True)
class FixtureRecord:
  data: bytes            # Annex-B access unit (one picture)
  frame_id: int
  encode_id: int
  timestamp_sof: int     # device ns
  timestamp_eof: int     # device ns
  flags: int             # device encoder flags

  @property
  def is_keyframe(self) -> bool:
    return bool(self.flags & V4L2_BUF_FLAG_KEYFRAME)

  @property
  def nbytes(self) -> int:
    return len(self.data)


class FixtureFormatError(Exception):
  pass


def iter_fixture(path: str, max_frames: int | None = None):
  """Yield FixtureRecord objects from a .enc file, in file order."""
  n = 0
  with open(path, "rb") as f:
    while True:
      if max_frames is not None and n >= max_frames:
        return
      head = f.read(HDR_SIZE)
      if not head:
        return
      if len(head) < HDR_SIZE:
        raise FixtureFormatError(f"{path}: truncated header at record {n} (got {len(head)} of {HDR_SIZE} bytes)")
      nbytes, frame_id, encode_id, sof, eof, flags = HDR.unpack(head)
      data = f.read(nbytes)
      if len(data) != nbytes:
        raise FixtureFormatError(f"{path}: truncated payload at record {n} (got {len(data)} of {nbytes} bytes)")
      yield FixtureRecord(data, frame_id, encode_id, sof, eof, flags)
      n += 1


def read_fixture(path: str, max_frames: int | None = None) -> list[FixtureRecord]:
  records = list(iter_fixture(path, max_frames=max_frames))
  # Validate framing consumed the whole file (only meaningful on a full read).
  if max_frames is None:
    framed = sum(HDR_SIZE + r.nbytes for r in records)
    on_disk = os.path.getsize(path)
    if framed != on_disk:
      raise FixtureFormatError(f"{path}: framed size {framed} != on-disk {on_disk} (trailing/garbage bytes)")
  return records


def load_params_from_fixture_source(hevc_path: str) -> bytes:
  """Extract the leading VPS/SPS/PPS Annex-B bytes from a raw .hevc file.

  Returns the concatenated parameter-set NALs (with their start codes) from the
  head of the stream. Used to seed vtdec for fixtures, which do not carry params.
  """
  with open(hevc_path, "rb") as f:
    head = f.read(4096)
  out = bytearray()
  i = 0
  n = len(head)
  found = 0
  while i < n - 4 and found < 3:
    if head[i:i + 4] == b"\x00\x00\x00\x01":
      hl = 4
    elif head[i:i + 3] == b"\x00\x00\x01":
      hl = 3
    else:
      i += 1
      continue
    nal_type = (head[i + hl] >> 1) & 0x3F
    if nal_type in (32, 33, 34):
      # find next start code to delimit this NAL
      j = i + hl + 2
      while j < n - 4:
        if head[j:j + 4] == b"\x00\x00\x00\x01" or head[j:j + 3] == b"\x00\x00\x01":
          break
        j += 1
      else:
        j = n
      out += head[i:j]
      found += 1
      i = j
    else:
      i += hl
  if found < 3:
    raise FixtureFormatError(f"{hevc_path}: only found {found}/3 parameter sets at head")
  return bytes(out)


@lru_cache(maxsize=4)
def fixture_path_for(route_dir: str, cam: str) -> str:
  """Locate the .enc for a camera within a route fixtures dir."""
  return os.path.join(route_dir, f"{cam}.enc")
