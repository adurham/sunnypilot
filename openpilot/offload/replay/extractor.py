#!/usr/bin/env python3
"""WS-C extractor — build replay fixtures from a recorded openpilot route.

Reads a route directory (~/comma-routes/<route>/) containing rlog.zst, qlog.zst,
fcamera.hevc (road) and ecamera.hevc (wide) and writes, per camera, a `.enc`
record file in the frozen format (see openpilot/offload/replay/fixture.py),
a `.header` parameter-set preamble, and one `meta.json` for the route.

Fixtures are written OUTSIDE the repo (default
~/.hermes/cache/scratch/car-features/offload/replay-fixtures/<route>/) so the
git tree stays clean.

Usage (from the worktree root):
  PYTHONPATH=$PWD:$PWD/opendbc_repo:$PWD/msgq_repo:$PWD/tinygrad_repo \\
    .venv/bin/python -m openpilot.offload.replay.extractor <route_dir> [<route_dir> ...]
  ... --out-dir <dir>  --force  --limit-seconds 600

Exit codes: 0 ok, 1 bad fixture (hard error), 2 usage/IO error.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

from openpilot.tools.lib.logreader import LogReader

from openpilot.offload.replay import fixture as fx

DEFAULT_OUT_DIR = os.path.expanduser("~/.hermes/cache/scratch/car-features/offload/replay-fixtures")
CAM_HEVC = {"narrow": "fcamera.hevc", "wide": "ecamera.hevc"}


class FixtureError(Exception):
  pass


def _route_name(route_dir: str) -> str:
  return os.path.basename(os.path.normpath(route_dir))


def _load_encode_idx(route_dir: str, cam: str):
  """Return [(frameId, encodeId, sof, eof, flags, len)] from qlog <cam>EncodeIdx."""
  svc = fx.ENCODE_IDX_SVC[cam]
  rows = []
  qlog = os.path.join(route_dir, "qlog.zst")
  if not os.path.exists(qlog):
    raise FixtureError(f"{route_dir}: missing qlog.zst")
  for msg in LogReader(qlog):
    if msg.which() != svc:
      continue
    raw = getattr(msg, svc)
    d = getattr(raw, "idx", raw)      # EncodeData wraps EncodeIndex; *EncodeIdx IS EncodeIndex
    f = d.to_dict()
    rows.append((int(f["frameId"]), int(f["encodeId"]), int(f["timestampSof"]),
                 int(f["timestampEof"]), int(f["flags"]), int(f["len"])))
  return rows


def _load_camera_state_eof(route_dir: str, cam: str) -> dict[int, int]:
  """frameId -> real timestampEof from the rlog (0 / missing => no real eof)."""
  svc = fx.CAMERA_STATE_SVC[cam]
  out: dict[int, int] = {}
  rlog = os.path.join(route_dir, "rlog.zst")
  if not os.path.exists(rlog):
    return out
  for msg in LogReader(rlog):
    if msg.which() != svc:
      continue
    d = getattr(msg, svc)
    eof = int(d.timestampEof)
    if eof > 0:
      out[int(d.frameId)] = eof
  return out


def _extract_camera(route_dirs: list[str], cam: str, out_dir: str) -> dict:
  """Build one .enc for `cam` from one or more consecutive route dirs (concatenated)."""
  enc_path = os.path.join(out_dir, fx.ENC_NAME[cam])
  hdr_path = os.path.join(out_dir, fx.HDR_NAME[cam])
  partial = enc_path + ".partial"

  n_real = n_syn = 0
  written = 0
  idx_all: list[tuple[int, int, int, int, int, int]] = []
  headers: list[bytes] = []
  n_aus_total = 0

  with open(partial, "wb") as f:
    for route_dir in route_dirs:
      hevc = os.path.join(route_dir, CAM_HEVC[cam])
      if not os.path.exists(hevc):
        raise FixtureError(f"{route_dir}: missing {CAM_HEVC[cam]}")
      idx = _load_encode_idx(route_dir, cam)
      eof_map = _load_camera_state_eof(route_dir, cam)
      data = open(hevc, "rb").read()
      header, aus = fx.split_stream(data)
      headers.append(header)

      if len(aus) != len(idx):
        raise FixtureError(
          f"{_route_name(route_dir)}/{cam}: AU count {len(aus)} != EncodeIdx records {len(idx)} "
          "(stream/record desync — bad fixture)")

      for i, (fid, eid, sof, eof_q, flags, ln) in enumerate(idx):
        au = aus[i]
        if len(au) != ln:
          raise FixtureError(
            f"{_route_name(route_dir)}/{cam}: record {i} (frameId {fid} encodeId {eid}) "
            f"AU len {len(au)} != EncodeIdx len {ln} — AU grouping/record misalignment (bad fixture)")
        eof = eof_map.get(fid, 0)
        if eof > 0:
          n_real += 1
        else:
          eof = sof + fx.SYNTHETIC_EOF_OFFSET_NS
          n_syn += 1
        written += fx.write_record(f, fid, eid, sof, eof, flags, au)
      idx_all.extend(idx)
      n_aus_total += len(aus)

  os.replace(partial, enc_path)
  with open(hdr_path, "wb") as f:
    f.write(b"".join(headers))

  return {
    "enc": enc_path, "header": hdr_path,
    "n_frames": len(idx_all), "bytes": written, "n_aus": n_aus_total,
    "first_frame_id": idx_all[0][0] if idx_all else None,
    "last_frame_id": idx_all[-1][0] if idx_all else None,
    "first_encode_id": idx_all[0][1] if idx_all else None,
    "last_encode_id": idx_all[-1][1] if idx_all else None,
    "encode_id_contiguous": all(idx_all[i + 1][1] == idx_all[i][1] + 1 for i in range(len(idx_all) - 1)),
    "keyframes": sum(1 for r in idx_all if r[4] & fx.V4L2_BUF_FLAG_KEYFRAME),
    "eof_source": {"real": n_real, "synthetic": n_syn},
    "sha256": fx.sha256_file(enc_path),
    "sha256_header": fx.sha256_file(hdr_path),
  }


def extract_route(route_dirs: list[str], out_dir: str, name: str | None = None) -> dict:
  """Extract a fixture from one route dir, or concatenate several consecutive ones."""
  if isinstance(route_dirs, str):
    route_dirs = [route_dirs]
  route_dirs = [os.path.abspath(os.path.expanduser(d)) for d in route_dirs]
  for d in route_dirs:
    if not os.path.isdir(d):
      raise FixtureError(f"not a directory: {d}")
  route = name or _route_name(route_dirs[0])
  dest = os.path.join(out_dir, route)

  if os.path.exists(os.path.join(dest, "meta.json")):
    raise FixtureError(f"{dest}: fixture exists (use --force to rebuild)")

  os.makedirs(dest, exist_ok=True)
  t0 = time.time()
  cams = {cam: _extract_camera(route_dirs, cam, dest) for cam in fx.CAMERAS}
  all_cont = all(cams[cam]["encode_id_contiguous"] for cam in fx.CAMERAS)

  meta = {
    "format": fx.FORMAT_ID,
    "route": route,
    "route_dirs": route_dirs,
    "segments": [_route_name(d) for d in route_dirs],
    "extracted_utc": fx.utc_now(),
    "extract_seconds": round(time.time() - t0, 3),
    "n_frames": {cam: cams[cam]["n_frames"] for cam in fx.CAMERAS},
    "first_frame_id": {cam: cams[cam]["first_frame_id"] for cam in fx.CAMERAS},
    "last_frame_id": {cam: cams[cam]["last_frame_id"] for cam in fx.CAMERAS},
    "first_encode_id": {cam: cams[cam]["first_encode_id"] for cam in fx.CAMERAS},
    "last_encode_id": {cam: cams[cam]["last_encode_id"] for cam in fx.CAMERAS},
    "encode_id_contiguous": {cam: cams[cam]["encode_id_contiguous"] for cam in fx.CAMERAS},
    "keyframes": {cam: cams[cam]["keyframes"] for cam in fx.CAMERAS},
    "eof_source": {cam: cams[cam]["eof_source"] for cam in fx.CAMERAS},
    "sha256": {cam: cams[cam]["sha256"] for cam in fx.CAMERAS},
    "sha256_header": {cam: cams[cam]["sha256_header"] for cam in fx.CAMERAS},
    "bytes": {cam: cams[cam]["bytes"] for cam in fx.CAMERAS},
    "files": {cam: {"enc": fx.ENC_NAME[cam], "header": fx.HDR_NAME[cam]} for cam in fx.CAMERAS},
  }
  fx.write_meta(os.path.join(dest, "meta.json"), meta)
  meta["_all_encode_id_contiguous"] = all_cont
  return meta


def main(argv=None) -> int:
  ap = argparse.ArgumentParser(description="Build replay fixtures from recorded routes")
  ap.add_argument("routes", nargs="+", help="route dirs; multiple dirs are concatenated in order")
  ap.add_argument("--out-dir", default=DEFAULT_OUT_DIR)
  ap.add_argument("--force", action="store_true", help="rebuild an existing fixture")
  ap.add_argument("--name", default=None, help="fixture name (default: first route dir's basename)")
  args = ap.parse_args(argv)

  try:
    meta = extract_route(args.routes, args.out_dir, name=args.name)
  except FixtureError as e:
    print(f"[extractor] BAD FIXTURE: {e}", file=sys.stderr)
    return 1
  except Exception as e:  # noqa: BLE001 — surface any IO/parse failure clearly
    print(f"[extractor] ERROR: {e!r}", file=sys.stderr)
    return 2

  ns = meta["n_frames"]
  print(f"[extractor] OK {meta['route']}  segs={len(meta['segments'])} "
        f"narrow={ns['narrow']} wide={ns['wide']}  "
        f"eof real/synth narrow={meta['eof_source']['narrow']['real']}/"
        f"{meta['eof_source']['narrow']['synthetic']} "
        f"wide={meta['eof_source']['wide']['real']}/{meta['eof_source']['wide']['synthetic']}  "
        f"encodeId_contiguous={meta['_all_encode_id_contiguous']}  "
        f"-> {os.path.join(args.out_dir, meta['route'])}")
  return 0


if __name__ == "__main__":
  sys.exit(main())
