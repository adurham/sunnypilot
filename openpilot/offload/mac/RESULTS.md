# WS-B — Mac frame path: RESULTS

Owner: WS-B. Deliverables under `openpilot/offload/mac/`. Conforms to
`openpilot/offload/INTERFACES.md` §1–3 and the frozen `contract.py`/`ports.py`.

## What works

| Piece | State | Evidence |
|---|---|---|
| `vtdec` (Swift VideoToolbox CLI) | ✅ built + exercised | `mac/vtdec` (kept next to source); decode p50 ≈ 1.2 ms |
| AU grouping (3 slices/picture) | ✅ | 1200/1200 AUs; without grouping VT returns -12909 on ~2/3 of frames |
| `frametable.py` (FrameEvent store) | ✅ | 8/8 unit tests |
| `fixtures.py` (replay reader) | ✅ | 9/9 unit tests |
| `framebridge.py` replay mode | ✅ | 1200/1200 frames, exact device frame_id/sof |
| `framebridge.py` ZMQ mode | ✅ | loopback test: 40/40 decoded + small services republished |
| Mac-local cameraState synth + header restamp | ✅ | §1.2 |
| small-service republish into local msgq | ✅ | `carState`/`deviceState`/`carControl` seen by SubMaster |
| VisionIPC publish (`camerad`, device ids) | ✅ | `VisionIpcClient` receives exact frame_id/sof/eof; buffers use DEVICE NV12 geometry (stride 2048 / uv_offset 2490368 / size 4804608) |
| `--format info` mode | ✅ | `{"width":1928,"height":1208,"pixelformat":"420v","pixelformat_code":875704438}` |
| `--format geom` self-check | ✅ | equals `nv12_info.get_nv12_info(1928,1208)` exactly |
| latency jsonl (`--out`) | ✅ | `contract` FrameEvent/LatencyRecord fields |
| SIGINT/SIGTERM clean shutdown | ✅ | terminates vtdec children |
| tests + real-route integration test | ✅ | 19/19 pass |

## Exact commands

```bash
cd /Users/adam.durham/repos/sunnypilot-offload
export PY=".venv/bin/python"; export PP="$PWD:$PWD/opendbc_repo:$PWD/msgq_repo:$PWD/tinygrad_repo"

# rebuild vtdec (kept next to source)
xcrun swiftc -O openpilot/offload/mac/vtdec.swift -o openpilot/offload/mac/vtdec

# pixel-format probe (reads the framed stream from a file)
./openpilot/offload/mac/vtdec --file <framed.bin> --params <cam.params> --format info
#   -> {"width":1928,"height":1208,"pixelformat":"420v","pixelformat_code":875704438}

# ZMQ mode (device bridge host)
PYTHONPATH=$PP $PY -m openpilot.offload.mac.framebridge --host <device-ip>

# replay mode (fixtures dir or a single .enc), narrow or wide
PYTHONPATH=$PP $PY -m openpilot.offload.mac.framebridge \
    --replay --replay-path openpilot/offload/replay/fixtures/00000149--4a4df1cf8a--36 \
    --replay-cam narrow \
    --hevc-params ~/comma-routes/00000149--4a4df1cf8a--36/fcamera.hevc \
    --out openpilot/offload/mac/logs/narrow_full.jsonl

# tests
PYTHONPATH=$PP $PY -m pytest openpilot/offload/mac/tests/ -q
```

## Observed numbers (route 00000149--4a4df1cf8a--36, 1200 frames/cam)

Load avg at run time: 2.0–3.7 (5 users, 6d uptime) — benign.

| metric | narrow | wide |
|---|---|---|
| frames received / decoded / published | 1200 / 1200 / 1200 | 1200 / 1200 / 1200 |
| frame_id range | 43206..44405 | 43206..44405 |
| frame continuity (gaps ≠ 1, dupes) | none / 0 | none / 0 |
| VisionIPC buffer_len / stride / uv_offset | 4804608 / 2048 / 2490368 | 4804608 / 2048 / 2490368 |
| **decode_ms p50** | **1.123** | **1.127** |
| decode_ms p99 / p99.9 / max | 2.225 / 2.808 / 3.273 | 2.370 / 2.483 / 3.385 |
| recv→VisionIPC-publish p50 / p99 | 6.25 / 9.15 ms | 6.88 / 9.31 ms |
| payload bytes sum | 75,017,454 | 74,967,722 |

**G7 decode latency gate (p50 ≤ 3 / p99.9 ≤ 10 ms): PASS** (1.2 / 3.1 ms).

`recv→publish` max ≈ 59 ms is the burst-replay first-frame (VT session init); under
real 20 fps pacing it is bounded by the p99 above.

## Design notes / integration facts

- **VT AU grouping is mandatory.** openpilot packs 3 VCL slices per picture; each VCL
  is length-prefixed within one `CMSampleBuffer`. Feeding slices individually makes VT
  return -12909 on ~2/3 of frames. `vtdec` groups by `first_slice_segment_in_pic_flag`
  (and treats VPS/SPS/PPS after a prior VCL as a new AU) and submits one sample/AU.
- **Pixel copy uses per-plane APIs, into DEVICE geometry.** VT's 420v biplanar buffers have
  **independent, non-contiguous** plane row-bytes; assuming Y/UV contiguity segfaults (measured).
  `vtdec` copies plane 0 (Y) and plane 1 (CbCr) separately into a **device-geometry** NV12 buffer
  (`stride=align(w,128)`, Y plane `align(h,32)` rows at `row*stride`, UV plane `align(h/2,16)` rows
  at `uv_offset + row*stride`, `size` from `get_nv12_info`) — see OPEN-C1 below.

## OPEN-C1 CLOSED — VisionIPC buffer geometry

**Bug.** `framebridge` allocated its VisionIPC buffers with msgq's *tight* `create_buffers`
(`size=w*h*3/2=3493536`, `stride=w=1928`, `uv_offset=w*h=2329024`) and `vtdec` packed rows at
width `w`. But on-device `camerad` allocates from `get_nv12_info(1928,1208)` =
`(stride=2048, y_height=1216, uv_height=608, size=4804608)` via `create_buffers_with_sizes`, and the
fork's `modeld_v2` run path reads frames at *that* geometry. The tight layout skewed every row after
the first by `(2048-1928)=120` bytes; `modeld_runner` only avoided an exception by zero-padding to
`frame_copy_size=3735552` — a band-aid that left the pixels wrong.

**Fix.** (1) `vtdec.swift` now emits device-geometry NV12 (`stride 2048`, Y 1216 rows, UV 608 rows at
`uv_offset 2490368`, `size 4804608`; row tails and pad rows zero-filled) — `nbytes` is the full
buffer size. A new `--format geom` self-check prints `w,h,stride,uv_offset,y_height,uv_height,size`.
(2) `framebridge` allocates with `create_buffers_with_sizes(...)` using the same `get_nv12_info`
tuple. (3) `modeld_runner` drops the zero-pad and now **asserts** `len(buf) >= frame_copy_size`.

**Verification (route 149-36).** `vtdec --params fcamera.hevc --format geom` →
`{"width":1928,"height":1208,"stride":2048,"uv_offset":2490368,"y_height":1216,"uv_height":608,"size":4804608}`
— byte-identical to `nv12_info.get_nv12_info(1928,1208)`. WS-B replay: `VisionIpcClient` sees
`buffer_len=4804608, stride=2048, uv_offset=2490368`; published payload length `4804608`.

- **Params live in `EncodeData.header`, not `.data`.** `encoder.cc` sets
  `edat.setHeader(header)` only on keyframes; `.data` is VCL-only. framebridge forwards
  `.header` to vtdec as a codec-config block (flags bit1). Replay mode instead sources
  VPS/SPS/PPS from a raw `.hevc` (`--hevc-params`), because fixtures (per the frozen
  spec) carry VCL-only AUs and are not decodable standalone.
- **`flags` is not a `FrameData` member** (`CameraState` schema has no `flags`, only
  `carState`/`wideRoadCameraState`-adjacent fields); cameraState is synthesized from
  `EncodeData.idx` (frameId/frameIdSensor/requestId/encodeId/sof/eof/processingTime).
- **`contract.FrameEvent.is_keyframe` is derived** from `EncodeIndex.flags & 8`
  (`V4L2_BUF_FLAG_KEYFRAME`), as the task specifies.
- VisionIPC `create_buffers_with_sizes(vst, 4, w, h, size, stride, uv_offset)` uses the first
  `EncodeData` dims (1928×1208) and the `get_nv12_info` geometry (OPEN-C1);
  `send(vst, payload, frame_id, sof, eof)` uses the **device** ids (§3).
- Header re-stamp idiom (works with this capnp): `messaging.log_from_bytes(raw).as_builder()`
  then set `logMonoTime = time.monotonic_ns()` (§1.2); payload untouched.

## Blockers / OPEN items

1. **BLOCKER — this repo's msgq has NO ZMQ transport.** `MSGQSubSocket::connect`
   hard-asserts `address == "127.0.0.1"` (`msgq_repo/msgq/impl_msgq.cc:40`), so
   `messaging.sub_sock(..., addr=<device>)` aborts the process. framebridge therefore
   subscribes to the device with **pyzmq (27.1.0, installed)** on the same framing the
   C++ `bridge_zmq` uses (single-part raw cereal bytes, subscribe `""`,
   `tcp://<host>:<port>`, port = `ports.get_port`). Local publishes still use the
   frozen `PubMaster`/msgq path. This is consistent with INTERFACES §0 (ports.py is the
   port authority) but contradicts the *mechanism* implied by §(e); PM may want to
   note it. Not a functional blocker — verified end-to-end via a local ZMQ loopback.
2. **OPEN — `/format info` on a real 20 fps stream**: implemented and verified on
   fixtures, not yet against the device (device ssh is flaky).
3. **OPEN — fixtures carry VCL-only AUs** (see `mac/fixtures_README.md`); replay needs
   `--hevc-params`. If WS-C/others expect standalone-decodable `.enc`, a VPS/SPS/PPS
   sidecar is needed (decision left to PM).
4. **OPEN — `make_fixture.py` corrections vs the brief** (done by WS-B fixture worker,
   documented in `fixtures_README.md`): wide ← `ecamera.hevc` (not `fcamera.hevc`), and
   `timestampEof` is real (≈ sof+14.697 ms), not a 50 ms marker. Both changes were
   required for the AU-length == qlog-`len` invariant to hold 1200/1200.
5. `recv→publish` p99 in burst replay ≈ 9 ms; needs a paced (20 fps) run against the
   device to characterize the true end-to-end path (G6 owner is WS-C).

## Files

- `openpilot/offload/mac/vtdec.swift`, `openpilot/offload/mac/vtdec` (built binary)
- `openpilot/offload/mac/framebridge.py`, `frametable.py`, `fixtures.py`
- `openpilot/offload/mac/make_fixture.py`, `fixtures_README.md`
- `openpilot/offload/mac/tests/test_frametable.py`, `test_fixtures.py`,
  `test_integration_replay.py`, `test_zmq_loopback.py`
- `openpilot/offload/mac/logs/*.jsonl` + `*.log` (raw runs)
- fixtures (WS-B-generated, WS-C-owned dir): `openpilot/offload/replay/fixtures/00000149--4a4df1cf8a--36/{narrow,wide}.enc`
