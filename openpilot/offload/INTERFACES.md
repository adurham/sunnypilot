# Offload interfaces — FROZEN CONTRACT (branch `offload-mac`)

Owner: PM agent. Consumers: WS-A (model stack), WS-B (Mac frame path), WS-C (replay+metrics),
WS-D (device side). **If you need a change here, edit nothing — append a `## CHANGES REQUESTED`
section at the bottom with the exact edit + reason; PM arbitrates between workstreams.**

## 0. Layout, ownership, and hard rules

| Path | Owner | Notes |
|---|---|---|
| `openpilot/offload/ports.py`, `contract.py`, `INTERFACES.md` | PM | FROZEN. Read-only for everyone else. |
| `openpilot/sunnypilot/modeld_v2/*` (patches only) | WS-A | Every hunk gated by env `OFFLOAD=1`; device behavior byte-identical when unset. |
| `openpilot/offload/models/` | WS-A | Compiled pkls + bench artifacts. |
| `openpilot/offload/mac/` | WS-B | `vtdec.swift`(+built `vtdec`), `framebridge.py`, `frametable.py`, `mac/tests/`. |
| `openpilot/offload/replay/` | WS-C | `replayd.py`, `metrics.py`, `p1_gate.py`, `replay/tests/`, reports. |
| `openpilot/offload/device/` | WS-D | `offloadd.py`, `bench/`, `check_device.sh`, `README.md`, `device/tests/`. |
| `openpilot/system/manager/process_config.py` | WS-D | Minimal gated entries only (P2-prep); verify registry layout first. |
| `FORK.md` | PM | No one else edits. |

Hard rules for ALL children:
- Worktree: `/Users/adam.durham/repos/sunnypilot-offload` (branch `offload-mac`). venv: `.venv` (symlinked).
  Run python as `PYTHONPATH=$PWD:$PWD/opendbc_repo:$PWD/msgq_repo:$PWD/tinygrad_repo .venv/bin/python ...`.
- **No git write operations** (no add/commit/checkout/stash). Read-only git is fine. PM commits.
- **No installs** (no pip/brew/network installs). stdlib + repo venv only. Fetching a model ONNX file is OK (WS-A).
- **No device writes, no reboots, no car interaction.** Device ssh is read-only and currently flaky (1Password agent).
- **Never kill processes you didn't start** — other sessions' subagents share this machine. GPU-heavy jobs:
  serialize with an flock on `/tmp/offload.gpu.lock`; check `uptime` before benchmarking and record loadavg.
- Write incremental results to files under your own subtree as you go; final answers ≤ 300 words + schema.

## 1. Wire semantics (normative — rationale in contract.py docstring)

1. ZMQ carries raw cereal Event bytes; device→Mac is never re-stamped; `logMonoTime` on the wire is DEVICE clock.
2. Mac-local republish (framebridge → Mac msgq): header `logMonoTime` = Mac monotonic ns; payload untouched.
3. Return path (WS-D offloadd, drive mode, P2 code-complete/P4-prep): freshness-gate on device clock; stale ⇒ drop (never republish); fresh ⇒ header = device monotonic now.
4. No clock sync anywhere. End-to-end age is always computed device-side as `now_device - timestamp_sof`.

## 2. Ports (from ports.py — run `python openpilot/offload/ports.py` to self-test)

Anchor set (live-verified): livestreamNarrowRoadEncodeData=8475, modelV2=58537, cameraOdometry=50972,
drivingModelData=49244, narrowRoadEncodeData=52737, wideRoadEncodeData=42305,
narrowRoadCameraState=20911, wideRoadCameraState=53095. Computed: `8023 + fnv1a(name) % (65535-8023)`.

## 3. Frame path contract (WS-B implements, WS-C/WS-A consume)

- `vtdec` CLIs: frame in `[u32 len][bytes][u64 pts][u32 flags]` → out `[u32 len][bytes][u64 pts][u64 decode_ns]`
  on stdout; exit non-zero + stderr on fatal. Keyframes flagged so the consumer can (re)init the session.
- Decoded pixel buffer handed to VisionIPC server `camerad` via `VisionIpcServer.send(...)` with
  **device frame_id, device timestamp_sof/eof** — never a local counter. Buffers use DEVICE NV12
  geometry via `create_buffers_with_sizes` (stride/y_height/uv_height/size from
  `system.camerad.cameras.nv12_info.get_nv12_info`), not msgq's tight `create_buffers` (OPEN-C1).
- `framebridge.py` publishes Mac-local `narrowRoadCameraState` (+`wideRoadCameraState`) synthesized from
  EncodeData idx (frameId/sof/eof/flags), header re-stamped per §1.2.
- FrameTable (WS-B) is the single source of truth for FrameEvent rows; exposes `append(evt)`,
  `frame(frame_id)`, `join_window(frame_id, timeout_ms)`; thread-safe; retention ≥ 600 frames.

## 4. modeld_v2 on Mac (WS-A)

- Frozen approach: fork `openpilot/sunnypilot/modeld_v2/` run path, patched ONLY under `OFFLOAD=1`:
  (a) DEV/WARP_DEV/QUEUE_DEV override → `METAL` on Darwin-arm64; (b) skip/neutralize realtime-priority and
  cpu-affinity calls (they raise on macOS); (c) COMMA_HARDWARE-dependent guards; (d) params access made
  injectable: seed `CarParams` bytes (from an rlog fixture or device CarParamsPersistent), tolerate
  `UnknownKeyName`; (e) VisionIPC server name from env `OFFLOAD_VIPC_SERVER` (default `camerad`);
  (f) no `qcom`/`kgsl`/GMMU references may execute on the Mac path.
- P1 model: STOCK supercombo ONNX (on disk: `~/.hermes/cache/scratch/car-features/offload/metal-model/driving_supercombo.onnx`,
  sha256 659727c4…) compiled through the FORK's `modeld_v2/compile_modeld.py` (`--model-type supercombo --model-size 512x256
  --camera-resolutions 1928x1208`) with `Device.DEFAULT=METAL`. Fork's active WMI-V9 split model = later swap, not P1.
- Fork-model Metal latency STOP condition: inference p50 > 20 ms or p99.9 > 40 ms at model input 512x256.

## 5. Gates (mechanical; `p1_gate.py` is the collector, exit 0/1, prints PASS/FAIL per gate)

- G1 frame timeline: no synthetic/dup frame_ids; gaps mirror source; encodeId↔frameId 1:1 monotone
  (proven on ≥2 routes ≥10 min; note first encodeId offset).
- G2 counts: decoded == encoded frames; modelV2 outputs == frames processed (per joined window).
- G3 determinism: double-run modelV2 rel ≤ 1e-6.
- G4 numerics: same pkl METAL vs CPU (fork runtime): median rel ≤ 1e-4, max ≤ 1e-2 (~600 frames sample).
- G5 decode: VT vs software reference (if importable) max|diff| ≤ 4, mean ≤ 0.5 (skip+warn if no reference).
- G6 e2e replay (10 ms synthetic jitter injected): SOF→modelV2-published p50 ≤ 75 / p99 ≤ 150 / p99.9 ≤ 250 ms.
- G7 decode latency: p50 ≤ 3 / p99.9 ≤ 10 ms.
- G8 fork-model inference: p50 ≤ 20 / p99.9 ≤ 40 ms.
Numeric parity with rlog `modelV2` is NEVER a gate (different model + lossy codec). Structural-only comparison.

## 6. Decision table (PM resolves; no user check-in)

| Trigger | Action |
|---|---|
| msgq/VisionIPC broken on macOS | (PASSED 2026-10-06 — fallback retired to doc-only) |
| Metal compile fails (unsupported op) | Report exact op+trace; do NOT rewrite model. Try CPU pkl for the pipeline, flag as OPEN-1. |
| Device bridge binary or pyzmq missing (ssh) | P2 device-presence work proceeds; flag OPEN-2; deploy decision goes to user. |
| fork-model p99.9 > 40 ms | Stop model work; report; PM decides stock-model-only MVP. |
| Any submodule pointer change in diff | Hard fail; revert. |
| Other sessions' GPU load perturbs bench | Record loadavg; rerun; if still noisy, mark bench "provisional". |

## 7. Active fallback / output arbitration (JOIN) — owner directive 2026-10-07; Jetlink-proven shape

**Invariant: exactly ONE process on the device publishes {modelV2, drivingModelData, cameraOdometry,
modelDataV2SP}. That process is the device modeld (modeld_v2).** The local model always runs and computes
every frame (unchanged); the Mac's outputs are consumed only when eligible. Absence/death of the offload
link NEVER removes model output — it only means the local output is published. ("Dead-man-switch the big
model": the big model must continuously prove liveness to be selected; the local model is the default
that never stops.)

- **Remote delivery (offloadd, WS-D):** Mac→device over ZMQ; offloadd forwards each remote message into
  device msgq under SHADOW names (contract.SHADOW_OUTPUT_SERVICES), header `logMonoTime` = device monotonic
  at receipt, payload untouched. Stale (>OFFLOAD_STALE_MS) or non-finite messages are dropped, never forwarded.
  Eligibility counters + behind/lost events logged; eligibility is signalled by presence/age of the shadow
  messages, not by an API.
- **SP pairing rule (added 2026-10-07):** `modelDataV2SP` carries no frameId and no timestamps
  (custom.capnp ModelDataV2SP), so it cannot be aged on its own. offloadd forwards it ONLY when paired
  within `OFFLOAD_SP_PAIR_MS` (default 25) of an aged-able service message that itself passed the freshness
  gate; an unpaired SP is dropped (`dropped_unpaired`, counted) — never forwarded blind. This is the same
  proximity rule the arbiter applies when binding SP to a frame set (OFFLOAD_ARB_SP_PROXIMITY_MS), so both
  ends agree on which frame an SP belongs to.
- **Arbitration (device modeld_v2, OFFLOAD-gated):** at each publish point, if remote mode is engaged and an
  eligible shadow output exists for the frame being published → publish the shadow's outputs under the real
  names (re-stamped to device-now). Else publish the locally computed outputs (today's path).
- **Eligibility:** shadow output for the target frame arrived within `OFFLOAD_ELIGIBLE_MS` (default 46) of the
  frame's reference time, and its outputs are finite.
- **Hysteresis (no per-frame flipping):** engage remote mode after `OFFLOAD_ENTER_N` (default 10 = 0.5 s)
  consecutive eligible frames; disengage after `OFFLOAD_EXIT_N` (default 3) consecutive ineligible.
- **Settling:** on engage, do not republish plans older than `OFFLOAD_SETTLE_N` (default 2) frames behind the
  latest locally computed frame; mirror Jetlink's "large model owns its settling frames" behavior in structure.
- **Cohesion:** all four messages published for a given frame come from ONE source (remote or local) — never
  mixed within a frame's output set. cameraOdometry travels with its frame's source.
- **Observability:** source switches + eligible/ineligible per-second counters via cloudlog (no schema change).
  OPEN (lagd P4): re-identify the delay model per active source; log the source for that analysis.
- **Failure semantics:** link dead / offloadd dead / Mac dead → eligibility 0 → local keeps publishing.
  No disengage may be caused by the offload system.

## CHANGES REQUESTED
(none — §7 (join/arbitration) was added 2026-10-07 by PM under owner directive "active fallback /
dead-man-switch the big model": local always publishes; remote selected only on continuous eligibility.)
