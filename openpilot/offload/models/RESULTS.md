# WS-A — Mac model stack: fork-flavor Metal supercombo pkl + bench + OFFLOAD modeld_v2 patches

**Host:** macOS 27.0.1 (arm64), tinygrad `Device.DEFAULT = METAL`. **Worktree:** `/Users/adam.durham/repos/sunnypilot-offload` @ `4247791681` (branch `offload-mac`). **Date:** 2026-10-07 (local CDT).

## Verdict (INTERFACES.md §4 / decision table)

**PASS — no OPEN-1.** The FORK compiler (`openpilot/sunnypilot/modeld_v2/compile_modeld.py`) compiled the stock
supercombo ONNX to a Metal pkl with **no unsupported-op fallback**. Bench is far inside the stop condition:

| gate | threshold | measured | verdict |
|---|---|---|---|
| inference p50 (G8) | ≤ 20 ms | **2.00 ms** | PASS |
| inference p99.9 (G8 / §4 stop) | ≤ 40 ms | **2.86 ms** | PASS |

→ Model work continues; no stock-only MVP needed.

## 1. Fork-flavor Metal pkl

**ONNX source (sha256 verified before compile):**
`driving_supercombo.onnx` — `659727c4d4839adc4992a254409a54259a8756a743f2d567bf5fdc6579f8009b` (60,881,999 B), matches the
task-supplied hash; no re-fetch needed.

**Compiler:** `openpilot/sunnypilot/modeld_v2/compile_modeld.py`
`--model-type supercombo --model-size 512x256 --camera-resolutions 1928x1208 --benchmark-runs 2`, run with cwd =
`openpilot/offload/models/` (the tool's tempfile is written in CWD). GPU serialized via `run_locked.py`
(fcntl.flock on `/tmp/offload.gpu.lock`; `flock(1)` is absent on macOS).

**Compile wall: 6 s** (exit 0). First capture 3344 ms (JIT+first kernel), then warm; pickle round-trip determinism
self-tests passed. Full log: `compile_fork.log`.

**Artifacts (chunked, load via `open_file_chunked`):**

| file | sha256 | bytes |
|---|---|---|
| `driving_supercombo_fork_metal.pkl.chunk01of02` | `cca606191aa3fd7de6cd9bd7c360f0f93a51817dc0c26be10b79f91d21cb7c27` | 47,185,920 |
| `driving_supercombo_fork_metal.pkl.chunk02of02` | `c46f1c40595e737729bbbffb0f3d003a686f6bbd81339dfbec642ea0c6310dde` | 25,562,742 |
| `driving_supercombo_fork_metal.pkl.chunkmanifest` | `d4735e3a265e16eee03f59718b9b5d03019c07d8b6c51f90da3a666eec13ab35` (`2`) | 1 |

Metadata: `input_devices = {'model': 'METAL'}`, `run_model` keyed `(1928, 1208)`,
input_shapes `{img:(1,12,128,256), big_img:(1,12,128,256), features_buffer:(1,24,512), desire_pulse:(1,25,8),
traffic_convention:(1,2), action_t:(1,2)}`, frame_skip = 4.

**Stock-flavor pkl (fallback, not needed — retained):**
`~/.hermes/cache/scratch/car-features/offload/metal-model/driving_metal.pkl.chunk01of02`
sha256 `da434f6598c763683f232c655945834558f00463e50ca1739cb8bf797fe84ebf`, chunk02 `dee5868b1ef4575bfed412436ec9619db31e52f02cb32bb21d8135007d031812`.

## 2. Bench (fork-flavor pkl)

Method: `load_oob(open_file_chunked(pkl))` → queues via the fork run path's own `make_input_queues` (frame_skip 4,
frame_copy_size 3,735,552) → `run_model(**queues)` timed as enqueue, then `Device.default.synchronize()`, wall =
enqueue+sync. 15 warmup, **200 measured runs**. Raw: `latency_fork_raw.csv`; summary: `latency_fork_results.json`.

| metric | enqueue ms | wall ms |
|---|---|---|
| p50 | 0.317 | **2.000** |
| p95 | 0.399 | **2.599** |
| p99 | 0.413 | **2.750** |
| p99.9 | 0.430 | **2.861** |
| p99.99 | 0.433 | 2.866 |
| max | 0.433 | 2.868 |
| min | 0.298 | 1.894 |
| mean | 0.329 | 2.052 |

Loadavg **before/after: 2.85 / 2.89 / 2.70** (identical — no drift, no contending GPU load). `uptime`:
`3:53 up 6 days, load averages: 2.85 2.89 2.70`; `uptime_before_bench.txt` ≈ `uptime_after_bench.txt`. Bench is
**not** provisional. (Loadavg sampler logged a single sample because the 200-run loop finishes in ~0.4 s; the
explicit before/after `uptime` captures bracket the run.)

20 Hz budget = 50 ms/frame → ≈ **25× headroom at p50, ≈ 17× at p99.9**.

## 3. modeld_v2 macOS patches (env-gated `OFFLOAD=1`)

All hunks in `openpilot/sunnypilot/modeld_v2/` only (verified: `git diff --name-only` touches nothing else;
no submodule pointer changes). `OFFLOAD` unset ⇒ every new branch is dead ⇒ device behavior byte-identical.
Full diff: `modeld_v2_offload.patch` (314 lines).

**`__init__.py` (was empty, now the gated shim — §4d/§4a foundation).** `libparams_c.dylib` is **not built** in this
worktree, so `import openpilot.common.params` raises `OSError: dlopen(...libparams_c.dylib)`; the whole run path
(`models.helpers`, `relc`, `livedelay.helpers`, stock `modeld`) imports through it. At `if OFFLOAD:` this module
injects a tolerant `Params` backend into `sys.modules['openpilot.common.params']` **before** any submodule imports
it, so `OFFLOAD` is fully injectable: `get` honors `OFFLOAD_<KEY>` env vars and `return_default`, storage-optional
`put/put_bool/remove`, plus `UnknownKeyName`/`ParamKeyFlag`/`ParamKeyType`. Unset ⇒ module body is a no-op.

**`modeld_base.py:16` (§4d).** `ModelStateBase.__init__` — under OFFLOAD, read `OFFLOAD_LAT_DELAY` or
`Params().get("LagdValueCache", return_default=True)` inside try/except, fall back to `0.1`.

**`modeld.py`:**
- `:42` (§4f) — `ChestnutState` eager `from openpilot.selfdrive.modeld.modeld import` is skipped on
  `OFFLOAD=1 and darwin` (that chain needs `usb1`, AMD/Chestnut device imports); `_get_chestnut_state_class()`
  returns a raising stub. Unset ⇒ eager import exactly as before.
- `:68` (§4e) — `OFFLOAD_VIPC_SERVER = os.environ.get('OFFLOAD_VIPC_SERVER', 'camerad')`.
- `:104` (§4f) — `_offload_chestnut_present()`: no USB/chestnut probe under OFFLOAD (`OFFLOAD_CHESTNUT` overrides).
- `:110` (§4d) — `_load_offload_car_params()`: seed `CarParams` from `OFFLOAD_CARPARAMS_PKL`, else
  `Params().get("CarParamsPersistent")`, else `get_demo_car_params()`.
- `:177` (§4a) — DEV/WARP_DEV/QUEUE_DEV = `METAL` on `OFFLOAD=1 and darwin` (env-overridable); else the original
  `COMMA_HARDWARE`-gated QCOM/AMD/CPU selection.
- `:384` (§4b) — `config_realtime_process(7, 54)` skipped under OFFLOAD on macOS (SCHED_FIFO/affinity unsupported).
- `:397,405,406` (§4e) — `VisionIpcClient.available_streams("camerad")` → `OFFLOAD_VIPC_SERVER`; the two client
  constructions likewise. Frame sync (25 ms/10 ms windows, frame_id from narrowRoadCameraState) untouched.
- `:474` (§4d) — main(): under OFFLOAD use `_load_offload_car_params()` instead of `params.get("CarParams", block=True)`.

**§4(ii) params** (CarParams, PlanplusControl, CameraOffset, Chestnut*, LagdValueCache/LagdToggle) are all served
by the shim (`Params.get`/`get_bool`) — the `Chestnut*` writes are no-ops, the `:465-467` block returns defaults.
**§4(iii)** frame-sync logic untouched (only the server name is configurable). **§4(v)** `_patch_tinygrad_fetch_fw`
reads `/lib/firmware` which does not exist on Mac — no-op, fine. **§4(vi)** no `qcom`/`kgsl`/GMMU execution on the
Mac path; `GMMU=0` is set at module top (no-op for Metal), and the stock chestnut modeld is never imported.

**Guard audit:** `verify_offload_guards.py` (AST) → 6 `if OFFLOAD` guards, all in the three files; output in
`offload_guards.txt`. `py_compile` clean. Fork unit tests under `OFFLOAD=1`: **110 passed, 2 skipped**
(`test_models`, `test_combined_pkl_loader`, `test_compile_modeld`, `test_camera_offset_helper`,
`test_parse_model_outputs`).

## 4. Offline run-path proof

WS-C `openpilot/offload/replay/` fixtures are **not present yet**, so `smoke_offline.py` drives a minimal local
stub frame source (no WS-C files touched): under `OFFLOAD=1`, import the patched `modeld_v2.modeld`, load the fork
pkl via `load_oob(open_file_chunked(...))`, build queues via `make_stock_input_queues`, then `ModelState.run(...)`
for **25 iterations** with a fixed deterministic uint8 frame buffer.

Result (`smoke_offline_results.json`) — **PASS**:
- DEV/WARP_DEV/QUEUE_DEV all `METAL`; combined_model_type `supercombo`; frame_skip 4; both vision inputs present.
- `all_finite = true`; `mean_nonzero_fraction = 0.988`; `plan` shape `[495]`, `absmax = 58.16` (plausible).
- Two fresh runs, same seed: `max_rel_diff = 0.0` ≤ 1e-6 ⇒ deterministic.

## Artifacts (this dir)

`driving_supercombo_fork_metal.pkl.chunk*` (+ manifest), `sha256sums.txt`, `compile_fork.log`,
`compile_wall_fork.txt`, `latency_fork_raw.csv` (200 rows), `latency_fork_results.json`,
`smoke_offline_results.json`, `offload_guards.txt`, `modeld_v2_offload.patch`, `bench_fork.py`, `smoke_offline.py`,
`verify_offload_guards.py`, `run_locked.py`, `uptime_*.txt`.
Scripts: `bench_fork.py`, `smoke_offline.py`, `verify_offload_guards.py`, `run_locked.py`.

## OPEN items

- **OPEN-A1 (info only):** `libparams_c.dylib` is absent in the worktree; the `OFFLOAD` shim works around it. If a
  later workstream needs the real Params backend, the native lib must be built — outside WS-A's no-install scope.
- **OPEN-A2:** camera-capture / 20 Hz scheduling / msgq IPC are out of scope here (WS-B/WS-C); the pkl carries a
  single camera resolution (1928x1208) as specified.

_(No CHANGES REQUESTED — the contract was implementable as written.)_
