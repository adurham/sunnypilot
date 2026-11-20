# RUNBOOK — Mac frame path: soak, sleep/App-Nap, and the OFFLOAD non-finite guard

Operational companion to `RESULTS.md`. Owns: how to run Mac soak/bench so the numbers
are trustworthy, and what the OFFLOAD-gated robustness changes do. All scripts live in
`openpilot/offload/mac/`.

## Sleep assertions & App Nap

**Why.** jetlink measured an in-process server throttled by App Nap at **p50 75 ms,
45% frames dropped**, where the *same server as a command-line process* ran at **36 ms**
(`~/repos/jetlink/JetlinkKit/Sources/JetlinkServer/Server.swift:530-545`;
`ProcessInfo.beginActivity([.userInitiatedAllowingIdleSystemSleep, .latencyCritical])`).
Their fix is a sleep/App-Nap assertion held only while a comma is connected.

**Our situation.** `framebridge.py` is a **plain CLI process**, not a bundled
`NSApplication`, so App Nap should not apply to it — but "should" is not evidence, and
idle sleep can still cut a long soak short. The wrapper is cheap insurance; use it for
**all soak/bench runs**.

**Use the wrapper** (`run_framebridge.sh`) instead of invoking `framebridge.py` directly.
It passes every argument through and, before `exec`, prints:

1. whether it is wrapping in `caffeinate` (item a),
2. `pmset -g assertions` — the system-wide sleep assertions, so the operator sees what
   was already asserting *before* we started (item b),
3. a one-line App-Nap / `NSAppSleepDisabled` note (item c).

```bash
cd /Users/adam.durham/repos/sunnypilot-offload
# ZMQ mode (device bridge), wrapped in `caffeinate -dims` by default:
openpilot/offload/mac/run_framebridge.sh --host 192.168.60.1 \
    --out openpilot/offload/mac/logs/soak_run.jsonl
# replay mode (fixtures), identical wrapper:
openpilot/offload/mac/run_framebridge.sh --replay \
    --replay-path openpilot/offload/replay/fixtures/00000149--4a4df1cf8a--36 \
    --replay-cam narrow --hevc-params ~/comma-routes/00000149--4a4df1cf8a--36/fcamera.hevc
```

`caffeinate -dims` asserts display / idle / disk / system sleep prevention for the life
of the child. Set `OFFLOAD_NO_CAFFEINATE=1` to skip the wrap (CI / already-asserted
environments); the script still prints the preamble. `PY=<interpreter>` overrides the
default `<repo>/.venv/bin/python`.

**Operator note.** If `pmset -g assertions` already shows a `caffeinate` assertion the
operator owns, ours is additive and harmless. A machine that legitimately sleeps
mid-soak invalidates the soak — check the wrapper preamble first.

## Soak evidence

**Why.** The device-side soak claim ("flat memory") rests on a per-PID PSS sampler reading
`/proc`. macOS has no `/proc`, so `bench_resources_mac.py` is the Mac counterpart: it
produces the *evidence* for a soak instead of an assertion.

**Run it bracketing a wrapped bridge run** (two shells, or background the sampler):

```bash
cd /Users/adam.durham/repos/sunnypilot-offload
# sampler: every 10 s for 30 min, jsonl + summary
.venv/bin/python openpilot/offload/mac/bench_resources_mac.py \
    --interval 10 --duration 1800 --label soak-149-36 \
    --out openpilot/offload/mac/logs/soak.jsonl
# then, in another shell:
openpilot/offload/mac/run_framebridge.sh --replay --replay-path <fixtures> --replay-cam narrow
```

**What it samples per tick** (`--interval`, default 10 s): per-PID RSS + %CPU for
`--names` patterns (default `framebridge,vtdec,modeld_runner,replayd,python`) or explicit
`--pids`; system memory via `memory_pressure` (falling back to `vm_stat` when that command
is absent); swap via `sysctl vm.swapusage` plus `vm_stat` swap/page in-out deltas;
compressor occupancy; and a one-shot `pmset -g therm` (non-sudo; best effort). stdlib +
subprocess only.

**Expected readings.** A steady-state soak should show, across the window:

- per-PID RSS **flat** (single-digit MB drift at most) and total RSS flat;
- **swap used** unchanged, and swap-in/out deltas near zero;
- **compressor** flat (it should not grow monotonically);
- no new `pmset -g therm` thermal/performance warning.

**How the summary flags a leak.** For every tracked series it fits a least-squares slope
(`MB/h`) and an R², plus a monotonic-with-flat-band check. A series is flagged
`leak_flagged` only when slope > `--leak-mb-per-hour` (default **5**) **and** R² ≥ 0.5
**and** the window ≥ `--min-window-s` (default **300 s**). The minimum window is the key
guard: a few MB of normal churn over seconds extrapolates to tens of thousands of MB/h, so
short runs report the slope but cannot flag. A flagged run prints
`** LEAK FLAGGED: <series> **` and names the series in `flagged_series` of the summary
JSON; the raw jsonl is one JSON object per sample for independent re-analysis.

Artifacts: `<out>` (jsonl) and `<out>.summary.json` (or `--summary-out`).

## Non-finite guard (OFFLOAD §4g, `modeld_v2`)

**Why.** jetlink's server holds hidden state and replaces it **only after an all-finite
frame**, reporting `NOT_FINITE` and keeping the last good state instead of tearing down
(`~/repos/jetlink/JetlinkKit/Sources/JetlinkServer/Queues.swift:122-125`). On the offload
Mac the run step must not die on a single bad frame.

**What we changed** (all hunks inside `if os.environ.get('OFFLOAD')` guards; `OFFLOAD`
unset ⇒ byte-identical device behavior):

- `openpilot/sunnypilot/modeld_v2/modeld.py:336-345` — supercombo branch: an offload host
  that sees a non-finite output preserves the last-good recurrent state (`prev_feat`),
  **publishes nothing** for that frame (absence ⇒ downstream staleness), logs once then
  rate-limits, and continues. The stock chestnut `raise RuntimeError("model output not
  finite")` still fires when `OFFLOAD` is unset (`:345`).
- `openpilot/sunnypilot/modeld_v2/modeld.py:352-355,364-367` — same last-good policy for
  the split/multi-policy vision and policy outputs.
- `openpilot/sunnypilot/modeld_v2/modeld.py:145-153` — `_note_nonfinite_offload()`: a
  rate-limited counter + `cloudlog.warning`.

This also closes a quiet hole: the `chestnut=False` Mac path previously had **no** finite
check at all, so a NaN silently poisoned `prev_feat` with no raise. The guard now covers
that case on the offload host only.

Unit-test the guard: `pytest openpilot/sunnypilot/modeld_v2/tests/test_offload_guard.py`
(asserts no exception escapes, `prev_feat` unchanged, a subsequent good frame advances
normally, and that OFFLOAD-unset still raises).

## Return path (Mac -> device) — `returnsend.py`

**What it is.** The Mac-side half of the return loop (INTERFACES §1, §7). The Mac-local
`modeld_v2` publishes its outputs into the **Mac-local** msgq under the real service names. A
stalled consumer must never be handed a backlog, so `returnsend.py` subscribes that msgq with a
**raw** socket, drops anything already older than `OFFLOAD_SEND_MAX_AGE_MS` (default **150 ms**,
env-overridable; `--max-age-ms`), and republishes the **raw Event bytes unchanged** over ZMQ to
the device, one PUB per service on `ports.get_port(name)` — the exact ports `offloadd` SUBs to.
It never re-stamps on the wire: `offloadd` re-stamps the header to device-now at receipt (§1.3).

**Run it (drive mode).** Mac binds, device connects:

```bash
cd /Users/adam.durham/repos/sunnypilot-offload
PYTHONPATH=$PWD:$PWD/opendbc_repo:$PWD/msgq_repo:$PWD/tinygrad_repo \
  .venv/bin/python -m openpilot.offload.mac.returnsend --bind 0.0.0.0 --stats
# on the device (read-only tree, from its root), once OffloadMode is set:
#   OFFLOAD_MODE=shadow python3 -m openpilot.offload.device.offloadd --connect-host <mac-ip>
```

`--bind` defaults to `0.0.0.0` so the device can reach the Mac over the tether
(`OFFLOAD_RETURN_BIND` overrides); use `127.0.0.1` for a same-machine test. `--services` restricts
the subset; `--quiet` silences stdout; `--stats`/`--stats-period` emit periodic counters.

**One-machine end-to-end test (no device).** Runs `returnsend` + `offloadd` as real subprocesses on
`127.0.0.1`, publishes fake modeld outputs with a recent SOF into the Mac msgq, then raw-subscribes
the shadow topics and asserts the full chain:

```bash
openpilot/offload/mac/return_path_test.sh          # exit 0 = chain proven
# equivalent pytest form:
PYTHONPATH=$PWD:$PWD/opendbc_repo:$PWD/msgq_repo:$PWD/tinygrad_repo \
  .venv/bin/python -m pytest openpilot/offload/mac/tests/test_returnsend.py -v
```

One machine is the correct model: here `offloadd`'s "device" clock **is** this machine's monotonic
clock, exactly as on a drive — both ends share one monotonic domain through the receipt re-stamp.

**Expected readings.**
- `returnsend` stdout (JSON lines): `forwarded` climbing with the message rate, `dropped_age == 0`
  (nothing is stale on a healthy Mac), `dropped_noreader == 0` once `offloadd` is connected
  (a brief nonzero at startup is the reader-monitor latch), `readers` per service `>= 1`.
- `offloadd` stdout: `forwarded` climbing; `dropped_stale == 0`, `dropped_error == 0`. When an
  offloadd SUB is down, `returnsend` keeps running (never blocks) and counts `dropped_noreader`.

**`modelDataV2SP` forwarding — SP pairing rule (resolved 2026-10-07).** SP has no `frameId` and no
`timestampSof`, so it can't be aged directly. `offloadd` forwards it only when it is **paired within
`OFFLOAD_SP_PAIR_MS` (25 ms, inclusive)** with a partner (`modelV2`/`cameraOdometry`/`drivingModelData`)
that itself passed the freshness + finite gate; otherwise it is dropped as `dropped_unpaired` and
never forwarded blind. Pairs are one-to-one (a partner is consumed) and use true drain-time stamps
(never the step-top clock, which can over-pair after a blocking camera update). The §7 arbiter binds
SP to a frame by the same 25 ms proximity rule. Success counter: `sp_paired`. `returnsend` ships all
four `RETURN_SERVICES` unchanged; the device end decides pairing.
