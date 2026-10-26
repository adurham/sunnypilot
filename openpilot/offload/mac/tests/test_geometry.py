#!/usr/bin/env python3
"""WS-B OPEN-C1 geometry check: vtdec emits DEVICE NV12 geometry.

Runs the built `vtdec` binary's `--params <raw .hevc> --format geom` self-check and
asserts the printed (width, height, stride, y_height, uv_height, uv_offset, size) is
byte-identical to `openpilot.system.camerad.cameras.nv12_info.get_nv12_info(w, h)` —
i.e. the same geometry camerad allocates on-device with create_buffers_with_sizes.

This closes the source half of OPEN-C1 (the fork's modeld_v2 reads frames at stride
2048 / uv_offset 2490368 / size 4804608 for 1928x1208; the Mac path must match).

Run (from the worktree root), or via pytest (skips if the route/binary are absent):
  PYTHONPATH=$PWD:$PWD/opendbc_repo:$PWD/msgq_repo:$PWD/tinygrad_repo \\
    .venv/bin/python -m pytest openpilot/offload/mac/tests/test_geometry.py -q
"""
from __future__ import annotations

import json
import os
import subprocess

import pytest

from openpilot.system.camerad.cameras.nv12_info import get_nv12_info

REPO = "/Users/adam.durham/repos/sunnypilot-offload"
ROUTE = os.path.expanduser("~/comma-routes/00000149--4a4df1cf8a--36")
HEVC = os.path.join(ROUTE, "fcamera.hevc")
VTDEC = os.path.join(REPO, "openpilot/offload/mac/vtdec")

# device values for 1928x1208 (frozen by camerad/nv12_info.h VENUS_BUFFER_SIZE)
EXPECT_1928x1208 = {"stride": 2048, "y_height": 1216, "uv_height": 608, "size": 4804608,
                    "uv_offset": 2490368}


def _ready() -> bool:
  return os.path.exists(HEVC) and os.path.exists(VTDEC)


def run_geom(params_path: str) -> dict:
  r = subprocess.run([VTDEC, "--params", params_path, "--format", "geom"],
                     capture_output=True, text=True, timeout=120)
  assert r.returncode == 0, f"vtdec --format geom failed ({r.returncode}): {r.stderr}"
  return json.loads(r.stdout.strip().splitlines()[-1])


@pytest.mark.skipif(not _ready(), reason="real route or built vtdec missing")
def test_vtdec_geom_matches_nv12_info():
  g = run_geom(HEVC)
  stride, y_height, uv_height, size = get_nv12_info(int(g["width"]), int(g["height"]))
  # exact device geometry for the camera resolution
  assert g == {"width": 1928, "height": 1208, "stride": 2048, "uv_offset": 2490368,
               "y_height": 1216, "uv_height": 608, "size": 4804608}
  # and equal (field by field) to the python reference the fork's modeld_v2 imports
  assert (g["stride"], g["y_height"], g["uv_height"], g["size"]) == (stride, y_height, uv_height, size)
  assert g["uv_offset"] == stride * y_height


@pytest.mark.skipif(not _ready(), reason="real route or built vtdec missing")
def test_vtdec_geom_pinned_device_values():
  g = run_geom(HEVC)
  for k, v in EXPECT_1928x1208.items():
    assert g[k] == v, f"{k}: vtdec={g[k]} != device {v}"


if __name__ == "__main__":
  if not _ready():
    print("route or vtdec missing; nothing to do")
    raise SystemExit(0)
  g = run_geom(HEVC)
  stride, y_height, uv_height, size = get_nv12_info(int(g["width"]), int(g["height"]))
  print("vtdec geom   :", json.dumps(g))
  print("nv12_info    :", {"stride": stride, "y_height": y_height, "uv_height": uv_height, "size": size,
                            "uv_offset": stride * y_height})
  assert (g["stride"], g["y_height"], g["uv_height"], g["size"], g["uv_offset"]) == \
      (stride, y_height, uv_height, size, stride * y_height)
  print("GEOMETRY OK")
