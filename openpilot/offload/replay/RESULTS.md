# WS-C — offline replay + metrics + gate harness: RESULTS

**Host:** macOS 27.0.1 (arm64, M4 Max); worktree `/Users/adam.durham/repos/sunnypilot-offload` @ branch `offload-mac`;
loadavg during the runs `2.0–2.5` (bracketed by `uptime` at each p1_gate invocation; no other GPU jobs).
**Date:** 2026-10-07 (local CDT). **Run root:** `. /` is the repo worktree.

Everything under `openpilot/offload/replay/` (WS-C owns it exclusively):
`fixture.py` (frozen format lib), `extractor.py`, `replayd.py`, `metrics.py`,
`modeld_runner.py`, `p1_gate.py`, `tests/test_replay.py`, `reports/<ts>/`.
Fixtures live OUTSIDE the repo: `~/.hermes/cache/scratch/car-features/offload/replay-fixtures/<name>/`.

## Verdict

**p1_gate exit 0 — all implemented gates PASS on both routes, end to end.**
WS-A (model stack) and WS-B (Mac frame path) had landed by run time, so nothing was stubbed;
the graceful-fail paths are implemented and unit-tested too.

## Frozen fixture format (as implemented)

`.enc` = concatenated records, little-endian:
`[u32 len][len bytes AU][u32 frameId][u32 encodeId][u64 sof][u64 eof][u32 flags]`.
`.header` = the stream's VPS/SPS/PPS (Annex-B, with start codes); `meta.json` = provenance +
sha256 of each `.enc`/`.header`, first/last frameId & encodeId, per-camera real/synthetic eof
counts, `encode_id_contiguous`.

HEVC NAL→AU grouping: one AU per coded picture, beginning at the first VCL slice whose
`first_slice_segment_in_pic_flag==1`; **`len` = Σ VCL slice bytes including their Annex-B start
codes, parameter sets excluded** — this is exactly what the device puts in `EncodeData.data`
and it reproduces the qlog `EncodeIndex.len` byte-for-byte (verified 1200/1200 on every route,
both cameras). `sof` from the qlog EncodeIdx; `eof` from the matching rlog `<cam>CameraState`
when present and >0, else `sof + 50_000_000` ns (counted under `eof_source.<cam>.synthetic`).
Any per-record `len` mismatch is a hard error (exit 1) and marks the fixture bad.

*Route A* `149-36` (single recorded segment, 1200 frames): eof real/synth **1198/2** per camera.
*Route B* `113-0-11` (12 consecutive segments concatenated = **14400 frames = 12.0 min**):
eof real/synth narrow **14384/16**, wide **14386/14**; frameId 11→14410, encodeId 0→14399,
**encodeId-contiguous=True** across all 12 segment joints. So Route B is the required
"≥10 min" route, built from the longest contiguous run of segments that carry both `.hevc` files
(only 3 of 62 routes have any hevc at all; `128--5` in the brief has qlog+rlog but no camera file,
so it is unusable as an input).

## p1_gate — the single evidence collector

`p1_gate.py` runs the whole pipeline per route: (a) `extractor`; (b) `replayd` publishes the wire
over local ZMQ with `--jitter-ms 10`; (c) WS-B `framebridge` (ZMQ SUB → `vtdec` → local VisionIPC,
Mac-local re-stamped `cameraState`) writes the `FrameEvent`/`LatencyRecord` jsonl; (d)
`replay/modeld_runner.py` imports the OFFLOAD-patched `sunnypilot.modeld_v2.modeld`, runs
`ModelState.run` over VisionIPC, and publishes `modelV2` into local msgq + one jsonl row per
frame; (e) `metrics.py` joins by frame_id and emits the gates. It also blasts the whole fixture
through `replayd --speed max` with no subscriber (G1wF) to prove the full fixture timeline.

### Command (exactly what produced `reports/20261007-042955/`)

```bash
cd /Users/adam.durham/repos/sunnypilot-offload
uptime   # record loadavg
PYTHONPATH=$PWD:$PWD/opendbc_repo:$PWD/msgq_repo:$PWD/tinygrad_repo \
  .venv/bin/python -m openpilot.offload.replay.p1_gate \
    --duration 40 --jitter-ms 10 --preflight-timeout 150
# -> overall PASS, exit 0, report: openpilot/offload/replay/reports/20261007-042955/report.md
```

### Gate results (route `149-36` / route `113-0-11`)

| gate | threshold | measured | verdict |
|---|---|---|---|
| G1 timeline | no dup/synthetic; encodeId↔frameId 1:1 monotone; Mac stream ⊆ source, gaps accounted | 770/1200 & 764/14400 frames replayed (40 s window), **drops=0**, dup=0, subseq=True, eid_mono/contig/1to1=True | **PASS** |
| G1w wire | every emitted record == fixture prefix, no dup | emitted=800/764, prefix_match=True | **PASS** |
| G1wF full wire | whole fixture through the wire, in order | 1200/1200, 14400/14400, dup=False | **PASS** |
| G2 counts | decoded == encoded-in-window; modelV2 == decoded (joined) | dec=770 (no_drops), mv2 765/765; dec=764 (no_drops), mv2 758/758 | **PASS** |
| G6 e2e (10 ms jitter) | p50 ≤ 75 / p99 ≤ 150 / p99.9 ≤ 250 ms | p50 **21.4/21.7**, p99 **33.0/32.4**, p99.9 **53.8/37.5** ms | **PASS** |
| G7 decode | p50 ≤ 3 / p99.9 ≤ 10 ms | p50 **2.25/2.31**, p99.9 **5.04/5.09** ms | **PASS** |

G6 = SOF→modelV2-published in the Mac clock domain (`modelv2_mac_ns − recv_mac_ns`; on loopback
`recv_mac_ns ≈` SOF, and §1.4 forbids subtracting across clocks). G7 uses the decoder's own
`decode_ms` (vtdec `decode_ns`); the recv→done round-trip (pipe + Python thread hop) is reported
alongside (`~5.5/6.1 ms`) but not gated.

### rlog baselines recomputed for the report table (report-only, route A)

`glass(SOF)→modelV2` p50 56.1 / p99 58.0; `glass(SOF)→carControl` p50 82.1 / p99 106.3;
`carControl→sendcan` p50 5.5 / p99 8.6; `glass(SOF)→sendcan` p50 86.6 / p99 111.7 ms.
Replay G6 p50 21.4 ms is *below* the stock 56 ms because the replay skips the device's
camera→encoder→loggerd path; structural comparison only, parity is never a gate.

## Graceful degradation (implemented + unit-tested, not stubbed)

* **WS-B missing** → `[p1_gate] WS-B missing: missing …/mac/framebridge.py`, report written, **exit 2**.
* **WS-A missing** (no `OFFLOAD` patch in `modeld_v2/modeld.py` or no compiled pkl under `offload/models/`)
  → `[p1_gate] WS-A missing: …`, report written, **exit 2**.

## Unit tests

`tests/test_replay.py` — **16 passed**: fixture reader round-trip & truncation, record size ==
spec, AU grouping on the real route `149-36` (1200/1200 len match, VPS/SPS/PPS preamble),
port anchors, metrics math (synthetic rows → exact p50/p99/p99.9, G1 dup/subsequence,
G2 joined-window, G7 threshold), and the p1_gate graceful-fail paths.

```bash
PYTHONPATH=$PWD:$PWD/opendbc_repo:$PWD/msgq_repo:$PWD/tinygrad_repo \
  .venv/bin/python -m pytest openpilot/offload/replay/tests/test_replay.py -q
# 16 passed
```

## OPEN items / caveats (honest)

* **OPEN-C1 — pixel layout.** WS-B's framebridge creates VisionIPC buffers at tight
  `1928*1208*3/2` but modeld copies `frame_copy_size` from stride 2048 (`~3.74 MB`). `modeld_runner`
  zero-pads the buffer to `frame_copy_size` so the run path executes; the NV12 stride/offset still
  differs, so pixel-exact numerics are **not** validated end to end — G4/G5 (numerics) and the
  fork-model G8 live in WS-A/§5 scope. Only structural gates (G1/G2/G6/G7) are claimed here.
* **First-IDR warmup.** The recorded routes start mid-GOP; vtdec needs the next IDR, so the first
  ~1.4 s (≈28 frames) fail to decode. G1/G2 measure inside the emitted / decoded window and count
  these as neither drops nor replayed frames; every replayed frame decodes (drops=0).
* **Transport drops under load.** framebridge's ZMQ path is device-faithful (`ZMQ_DONTWAIT`), so under
  a busy Mac it drops ~1 % of records at the socket (HWM); those are counted as `not_replayed`, never
  as frame drops. The full-fixture wire check (G1wF) is lossless because nothing else runs then.
* **`frameAge`/`unixTimestampNanos`** are not published by the offline runner (device-clock /
  device-wall quantities with no device clock present); they are not gates.
* **Route B naming.** Fixture B is named `113-0-11` (not `128-5`, which has no camera files).

## Reports

`openpilot/offload/replay/reports/20261007-042955/{report.md,summary.json}` — the passing run
(both routes). Earlier timestamped dirs are the iterative debugging runs; a `WS-B missing`
graceful-fail report dir is also present when that path was exercised.
