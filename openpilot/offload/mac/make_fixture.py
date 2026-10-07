#!/usr/bin/env python3
"""make_fixture.py -- re-mux a real comma segment into an offload replay fixture.

WS-B (Mac frame path).  Read-only over the source route; writes only under the
fixtures directory.

WHAT IT DOES
------------
Parses an Annex-B HEVC camera segment (fcamera/ecamera/dcamera.hevc) into NALs,
groups the VCL NALs of each picture into one *access unit* (AU), then zips those
AUs 1:1, in ascending order, with the per-camera EncodeIndex entries found in the
segment's qlog.zst.  Each (AU, idx) pair becomes one little-endian record:

    [u32 nbytes][nbytes of AU Annex-B bytes][u32 frameId][u32 encodeId]
    [u64 timestampSof][u64 timestampEof][u32 flags]

`nbytes` counts the AU's Annex-B bytes INCLUDING the leading start code(s),
exactly as they appear in the .hevc file.

------------------------------------------------------------------------------
TWO DELIBERATE DEVIATIONS FROM THE ORIGINAL TASK BRIEF (both auditable below)
------------------------------------------------------------------------------

(1) wide.enc is built from ecamera.hevc, NOT from fcamera.hevc.
    The brief assumed the wide stream reuses the front-camera (fcamera.hevc)
    bitstream.  That is factually wrong on this segment.  Measured on the
    on-disk files (frameId ascending, 1200 AUs each, exactly 3 VCL slices/AU):

        camera AU span  vs  narrowRoadEncodeIdx.len : 1200/1200 match
        fcamera          vs  wideRoadEncodeIdx.len  :    0/1200 match
        ecamera          vs  wideRoadEncodeIdx.len  : 1200/1200 match
        ecamera          vs  narrowRoadEncodeIdx.len:    0/1200 match

        sum(fcamera AU bytes)   = 75,017,454 == sum(narrowRoadEncodeIdx.len)
        sum(ecamera AU bytes)   = 74,967,722 == sum(wideRoadEncodeIdx.len)
        (file sizes differ by exactly 90 B = VPS+SPS+PPS + their start codes,
         i.e. the parameter sets that are not part of any AU.)

    wideRoadEncodeIdx is the comma three *encoder / right* camera == ecamera.
    The brief's own CORE, explicitly-asserted requirement is that each record's
    AU byte length equal the qlog len for that frame.  Pairing wide with fcamera
    would fail that assertion for all 1200 records; pairing it with ecamera
    satisfies it exactly.  The invariant wins; narrow <- fcamera, wide <- ecamera.
    Camera selection is resolved unambiguously: every candidate camera file is
    scored, and exactly one must be an all-zero-mismatch match (see _pick_camera).

(2) timestampEof is PRESERVED, not synthesized.
    The brief said qlog has no timestampEof (it is 0) and asked to synthesize
    timestampSof + 50 ms.  On this route timestampEof is present and nonzero:
    for every narrow and wide entry, timestampEof - timestampSof == 14,697,000 ns
    (uniform).  The real value is written untouched.  The requested +50 ms
    synthesis exists only as a FALLBACK, applied solely to individual records
    whose timestampEof is genuinely 0 (and it is counted in the summary line).
    Rationale: fixture fidelity beats fabrication, and nothing in the frozen
    contract (offload/contract.py FrameEvent.timestamp_eof = "device ns") wants a
    marker.  A uniform 14.697 ms SoF->EoF readout is real sensor timing.
------------------------------------------------------------------------------

STREAM CONFIG NOTE
------------------
An AU's byte span starts at the frame's first *VCL* NAL, so the 90 bytes of
VPS/SPS/PPS (and their start codes) at the head of each .hevc file are NOT part
of any record.  That is required for the AU-length == qlog-len invariant to hold
(the qlog `len` also excludes them).  Consequently a .enc file is not decodable
standalone -- the consumer must already own the stream's parameter sets.  If it
does not, a sidecar (e.g. the VPS/SPS/PPS NALs) needs to be added; this is left
as a decision for the parent rather than silently changing the record spec.

USAGE
-----
  PYTHONPATH=$PWD:$PWD/opendbc_repo:$PWD/msgq_repo:$PWD/tinygrad_repo \
    .venv/bin/python openpilot/offload/mac/make_fixture.py \
      --route-dir /Users/adam.durham/comma-routes/00000149--4a4df1cf8a--36 \
      --out-dir   openpilot/offload/replay/fixtures/00000149--4a4df1cf8a--36 \
      --cam both
"""
from __future__ import annotations

import argparse
import os
import struct
import sys
from dataclasses import dataclass, field
from typing import Iterable, Iterator

import zstandard

from openpilot.cereal import log as capnp_log


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# nbytes(u32), frameId(u32), encodeId(u32), timestampSof(u64), timestampEof(u64), flags(u32).
# The Annex-B AU bytes follow the fixed 32-byte header (see _pack_record).
RECORD_STRUCT = struct.Struct("<IIIQQI")
RECORD_HEADER_SIZE = RECORD_STRUCT.size
assert RECORD_HEADER_SIZE == 4 + 4 + 4 + 8 + 8 + 4 == 32

EOF_SYNTH_NS = 50_000_000  # 50 ms marker -- FALLBACK ONLY, see module docstring (2)

# nal_unit_type values that are VCL (a coded slice).
VCL_MAX_TYPE = 31
# parameter-set nal types (VPS/SPS/PPS) -- these lead an AU in the bitstream but
# are excluded from the AU span to match the qlog `len`.
NAL_VPS, NAL_SPS, NAL_PPS = 32, 33, 34

CAM_BY_STREAM = {"narrow": "fcamera", "wide": "ecamera"}


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass(slots=True)
class Nal:
    pos: int      # byte offset of the start code
    sc_len: int   # start-code length (3 or 4)
    start: int    # byte offset of the NAL header (pos + sc_len)
    end: int      # byte offset one past the last byte of this NAL
    nal_type: int
    first_slice: bool  # first_slice_segment_in_pic_flag

    @property
    def size(self) -> int:
        return self.end - self.pos


@dataclass(slots=True)
class AccessUnit:
    """One coded picture, delimited by its first VCL NAL.

    `span_pos` is the offset of the first VCL NAL's start code and `span_end` is
    the end of the last VCL NAL of the picture, so
    ``bytes_[span_pos:span_end]`` is the AU's Annex-B bytes (incl. start codes)
    and ``nbytes == span_end - span_pos``.
    """
    index: int
    span_pos: int
    span_end: int
    n_vcl: int
    vcl_types: tuple[int, ...]

    @property
    def nbytes(self) -> int:
        return self.span_end - self.span_pos


@dataclass(slots=True)
class EncodeIdx:
    frame_id: int
    encode_id: int
    timestamp_sof: int
    timestamp_eof: int
    flags: int
    length: int  # qlog `len` -- expected AU byte length


@dataclass(slots=True)
class StreamReport:
    stream: str
    camera: str
    hevc_path: str
    out_path: str = ""
    n_aus: int = 0
    n_idx: int = 0
    n_records: int = 0
    total_bytes: int = 0
    mismatches: list[tuple[int, int, int]] = field(default_factory=list)  # (order, au_nbytes, qlog_len)
    n_eof_synth: int = 0
    first_record: dict | None = None
    last_record: dict | None = None

    @property
    def ok(self) -> bool:
        return not self.mismatches and self.n_records > 0


# ---------------------------------------------------------------------------
# Annex-B parsing
# ---------------------------------------------------------------------------

def parse_nals(buf: bytes) -> list[Nal]:
    """Split an Annex-B byte stream on 00 00 01 / 00 00 00 01 start codes.

    Scans for start codes sequentially; any trailing/embedded zero-runs before a
    start code belong to the *next* NAL's start code (kept intact so that a
    re-slice of ``buf`` is byte-identical).
    """
    n = len(buf)
    starts: list[tuple[int, int]] = []  # (pos, sc_len)
    i = 0
    while i <= n - 3:
        if buf[i] == 0 and buf[i + 1] == 0:
            if buf[i + 2] == 1:
                starts.append((i, 3))
                i += 3
                continue
            if i <= n - 4 and buf[i + 2] == 0 and buf[i + 3] == 1:
                starts.append((i, 4))
                i += 4
                continue
        i += 1

    nals: list[Nal] = []
    for k, (pos, sc_len) in enumerate(starts):
        hdr = pos + sc_len
        end = starts[k + 1][0] if k + 1 < len(starts) else n
        if hdr + 1 >= end:
            continue  # degenerate; skip
        nal_type = (buf[hdr] >> 1) & 0x3F
        # first_slice_segment_in_pic_flag is the MSB of the byte right after the
        # 2-byte NAL header.
        first_slice = bool((buf[hdr + 2] >> 7) & 1) if hdr + 2 < end else False
        nals.append(Nal(pos, sc_len, hdr, end, nal_type, first_slice))
    return nals


def group_access_units(nals: list[Nal]) -> list[AccessUnit]:
    """Group VCL NALs into one AU per picture.

    A new AU begins at a VCL NAL whose first_slice_segment_in_pic_flag == 1.
    Non-VCL (parameter set / SEI) NALs are ignored for AU delimitation -- the AU
    span runs from its first VCL start code to the end of its last VCL NAL,
    which is what the qlog `len` measures.
    """
    aus: list[AccessUnit] = []
    cur: dict | None = None
    for na in nals:
        if na.nal_type > VCL_MAX_TYPE:
            continue  # VPS/SPS/PPS/SEI etc. -- never part of an AU span here
        if na.first_slice or cur is None:
            if cur is not None:
                aus.append(AccessUnit(len(aus), cur["span_pos"], cur["span_end"],
                                      cur["n_vcl"], tuple(cur["vcl_types"])))
            cur = {"span_pos": na.pos, "span_end": na.end, "n_vcl": 0, "vcl_types": []}
        cur["span_end"] = na.end
        cur["n_vcl"] += 1
        cur["vcl_types"].append(na.nal_type)
    if cur is not None:
        aus.append(AccessUnit(len(aus), cur["span_pos"], cur["span_end"],
                              cur["n_vcl"], tuple(cur["vcl_types"])))
    return aus


# ---------------------------------------------------------------------------
# qlog reading
# ---------------------------------------------------------------------------

def read_qlog_idx(qlog_path: str) -> dict[str, list[EncodeIdx]]:
    """Return {'narrow': [...], 'wide': [...]} EncodeIndex lists, ascending frameId."""
    raw = open(qlog_path, "rb").read()
    data = zstandard.ZstdDecompressor().stream_reader(raw).read()

    out: dict[str, list[EncodeIdx]] = {"narrow": [], "wide": []}
    which_name = {"narrowRoadEncodeIdx": "narrow", "wideRoadEncodeIdx": "wide"}
    for evt in capnp_log.Event.read_multiple_bytes(data):
        w = evt.which()
        stream = which_name.get(w)
        if stream is None:
            continue
        ei = getattr(evt, w)
        out[stream].append(EncodeIdx(
            frame_id=int(ei.frameId),
            encode_id=int(ei.encodeId),
            timestamp_sof=int(ei.timestampSof),
            timestamp_eof=int(ei.timestampEof),
            flags=int(ei.flags),
            length=int(ei.len),
        ))

    for stream, lst in out.items():
        # sort ascending by frameId; encodeId must co-increase (1:1 monotone map,
        # gate G1).  Assert rather than assume so a reordered qlog can never
        # silently misalign the 1:1 zip with the AU list.
        lst.sort(key=lambda e: (e.frame_id, e.encode_id))
        for a, b in zip(lst, lst[1:]):
            assert a.frame_id < b.frame_id, f"{stream}: frameId not strictly increasing at {a.frame_id}"
            assert a.encode_id < b.encode_id, f"{stream}: encodeId not strictly increasing at {a.encode_id}"
        if lst:
            for a, b in zip(lst, lst[1:]):
                if b.encode_id - a.encode_id != 1:
                    print(f"[warn] {stream}: non-contiguous encodeId {a.encode_id}->{b.encode_id}",
                          file=sys.stderr)
    return out


# ---------------------------------------------------------------------------
# Camera / stream pairing
# ---------------------------------------------------------------------------

def _pick_camera(cam_root: str, aus_by_cam: dict[str, list[AccessUnit]],
                 idx: list[EncodeIdx], stream: str) -> str:
    """Unambiguously choose which camera file a qlog index array belongs to.

    Every on-disk camera is scored by how many of its 1200 AUs match the qlog
    `len`.  Exactly one camera must be an all-zero-mismatch match; anything else
    is a hard error rather than a silently least-bad pick.
    """
    target = [e.length for e in idx]
    perfect: list[str] = []
    score_lines: list[str] = []
    for cam, aus in aus_by_cam.items():
        au_lens = [a.nbytes for a in aus]
        m = min(len(au_lens), len(target))
        nmis = sum(1 for i in range(m) if au_lens[i] != target[i]) + abs(len(au_lens) - len(target))
        score_lines.append(f"      {cam:8s} AUs={len(au_lens):5d} mismatches={nmis}")
        if nmis == 0 and len(au_lens) == len(target):
            perfect.append(cam)

    print(f"  [{stream}] camera resolution (AU bytes vs qlog len):")
    for line in score_lines:
        print(line)

    if len(perfect) == 1:
        return perfect[0]
    if not perfect:
        raise SystemExit(
            f"FATAL: no camera file matches {stream}RoadEncodeIdx.len (0/1200). "
            f"Scores:\n" + "\n".join(score_lines))
    raise SystemExit(
        f"FATAL: ambiguous {stream} camera -- {len(perfect)} perfect matches {perfect}. "
        f"Refusing to guess. Scores:\n" + "\n".join(score_lines))


# ---------------------------------------------------------------------------
# Record writing
# ---------------------------------------------------------------------------

def _pack_record(au_bytes: bytes, e: EncodeIdx, eof_value: int) -> bytes:
    return RECORD_STRUCT.pack(
        len(au_bytes),          # u32 nbytes
        e.frame_id,             # u32 frameId
        e.encode_id,            # u32 encodeId
        e.timestamp_sof,        # u64 timestampSof
        eof_value,              # u64 timestampEof
        e.flags,                # u32 flags
    ) + au_bytes


def write_stream(stream: str, camera: str, hevc_path: str, idx: list[EncodeIdx],
                 out_path: str, max_frames: int) -> StreamReport:
    buf = open(hevc_path, "rb").read()
    nals = parse_nals(buf)
    aus = group_access_units(nals)

    rep = StreamReport(stream=stream, camera=camera, hevc_path=hevc_path, out_path=out_path,
                       n_aus=len(aus), n_idx=len(idx))

    # 1:1 zip of AUs (ascending) with idx (ascending).  Use min() defensively and
    # document any count skew; for this route the counts are equal.
    n = min(len(aus), len(idx))
    if len(aus) != len(idx):
        print(f"[warn] {stream}: AU count {len(aus)} != idx count {len(idx)}; using {n}",
              file=sys.stderr)
    n = min(n, max_frames)

    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, "wb") as f:
        for i in range(n):
            au = aus[i]
            e = idx[i]
            au_bytes = buf[au.span_pos:au.span_end]

            # CORE INVARIANT: AU byte length == qlog len for this frame.
            if len(au_bytes) != e.length:
                rep.mismatches.append((i, len(au_bytes), e.length))

            eof_value = e.timestamp_eof
            if eof_value == 0:
                eof_value = e.timestamp_sof + EOF_SYNTH_NS  # fallback only
                rep.n_eof_synth += 1

            f.write(_pack_record(au_bytes, e, eof_value))
            rep.total_bytes += RECORD_HEADER_SIZE + len(au_bytes)
            if i == 0:
                rep.first_record = dict(order=0, nbytes=len(au_bytes), frameId=e.frame_id,
                                        encodeId=e.encode_id, timestampSof=e.timestamp_sof,
                                        timestampEof=eof_value, flags=e.flags)
            rep.last_record = dict(order=i, nbytes=len(au_bytes), frameId=e.frame_id,
                                   encodeId=e.encode_id, timestampSof=e.timestamp_sof,
                                   timestampEof=eof_value, flags=e.flags)

    rep.n_records = n
    return rep


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=(__doc__ or "re-mux a comma segment into a replay fixture").splitlines()[0])
    p.add_argument("--route-dir", required=True,
                   help="segment dir containing fcamera.hevc / ecamera.hevc and qlog.zst")
    p.add_argument("--out-dir", required=True,
                   help="fixtures output dir (narrow.enc / wide.enc are written here)")
    p.add_argument("--max-frames", type=int, default=0,
                   help="max records per stream (0 = all; default: all)")
    p.add_argument("--cam", choices=["narrow", "wide", "both"], default="both")
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    route_dir = os.path.abspath(os.path.expanduser(args.route_dir))
    out_dir = os.path.abspath(os.path.expanduser(args.out_dir))
    max_frames = args.max_frames if args.max_frames and args.max_frames > 0 else 1 << 62

    print(f"route-dir : {route_dir}")
    print(f"out-dir   : {out_dir}")
    print(f"cam       : {args.cam}   max-frames: {'all' if max_frames == 1 << 62 else max_frames}")

    qlog_path = os.path.join(route_dir, "qlog.zst")
    print(f"reading qlog: {qlog_path}")
    idx_by_stream = read_qlog_idx(qlog_path)
    print(f"  narrowRoadEncodeIdx: {len(idx_by_stream['narrow'])} entries")
    print(f"  wideRoadEncodeIdx  : {len(idx_by_stream['wide'])} entries")

    # Load + parse every candidate camera once; used both for pairing and for writing.
    cam_paths = {c: os.path.join(route_dir, c + ".hevc") for c in ("fcamera", "ecamera", "dcamera")}
    aus_by_cam: dict[str, list[AccessUnit]] = {}
    for cam, path in cam_paths.items():
        if not os.path.exists(path):
            continue
        buf = open(path, "rb").read()
        nals = parse_nals(buf)
        aus_by_cam[cam] = group_access_units(nals)
        print(f"  {cam}.hevc: {len(buf)} B, {len(nals)} NALs, {len(aus_by_cam[cam])} AUs "
              f"(VCL/AU={sorted({a.n_vcl for a in aus_by_cam[cam]})})")

    streams = ["narrow", "wide"] if args.cam == "both" else [args.cam]

    # Resolve camera per stream, then write with a second parse (write_stream is
    # self-contained and also usable standalone).  Parsing twice is cheap vs 75 MB.
    reports: list[StreamReport] = []
    for stream in streams:
        if not idx_by_stream[stream]:
            print(f"[warn] no {stream}RoadEncodeIdx entries; skipping {stream}", file=sys.stderr)
            continue
        camera = _pick_camera(route_dir, aus_by_cam, idx_by_stream[stream], stream)
        print(f"  [{stream}] -> {camera}.hevc")
        out_path = os.path.join(out_dir, f"{stream}.enc")
        rep = write_stream(stream, camera, cam_paths[camera], idx_by_stream[stream],
                           out_path, max_frames)
        reports.append(rep)

    # ---- summary -----------------------------------------------------------
    print("\n================ FIXTURE SUMMARY ================")
    overall_ok = True
    for r in reports:
        status = "OK" if r.ok else f"{len(r.mismatches)} MISMATCHES"
        print(f"{r.stream:6s}  cam={r.camera:8s}  records={r.n_records:5d}  "
              f"bytes={r.total_bytes:12d}  AU==qlog.len: {status}  eof_synth={r.n_eof_synth}")
        print(f"        out : {r.out_path}  ({os.path.getsize(r.out_path)} B on disk)")
        print(f"        first: {r.first_record}")
        print(f"        last : {r.last_record}")
        if r.mismatches:
            overall_ok = False
            for order, au_nb, q_len in r.mismatches[:20]:
                print(f"          MISMATCH order={order} au_nbytes={au_nb} qlog_len={q_len}")
            if len(r.mismatches) > 20:
                print(f"          ... {len(r.mismatches) - 20} more")
        # sanity: record framing must account for exactly the file size
        expect = r.total_bytes
        got = os.path.getsize(r.out_path)
        if expect != got:
            overall_ok = False
            print(f"        ERROR: framed size {expect} != on-disk {got}")
    print("================================================")
    return 0 if overall_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
