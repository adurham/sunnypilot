# §7 Output arbitration (active fallback / dead-man-switch the big model)

Device-side hook that lets **one** process — the device modeld_v2 — publish the Mac's (remote)
model outputs under the real service names `{modelV2, drivingModelData, cameraOdometry,
modelDataV2SP}`, but **only while the remote is continuously fresh**. Otherwise it publishes its
own locally computed outputs. The local model always runs and computes every frame; absence or
death of the offload link never removes model output (INTERFACES.md §7).

Owner: WS-A (`openpilot/sunnypilot/modeld_v2/**`). Sender of the shadow topics: WS-D
(`openpilot/offload/device/offloadd.py`). Codec/protocol: `openpilot/cereal/services.py`,
`openpilot/offload/contract.py`.

## Where it lives

| Piece | File |
|---|---|
| State machine + raw-msgq shadow reader | `openpilot/sunnypilot/modeld_v2/offload_arbiter.py` (`OffloadArbiter`, `make_arbiter`, `is_arbiter_enabled`) |
| modeld_v2 call surface (3 hunks, all inside `# OFFLOAD §7` comments) | `openpilot/sunnypilot/modeld_v2/modeld.py` |
| Tests | `openpilot/sunnypilot/modeld_v2/tests/test_offload_arbiter.py` |

modeld.py call sites:

- **construct once** — `modeld.py:498` `arbiter = make_arbiter(params)` (right after PubMaster/SubMaster).
  Returns `None` unless the gate below passes.
- **poll once per loop** — `modeld.py:569-571` `if arbiter is not None: arbiter.poll(time.monotonic_ns())`
  (just after `sm.update(0)`, non-blocking drain at 20 Hz).
- **publish point** — `modeld.py:677` `remote = arbiter.select(meta_main.frame_id, time.monotonic_ns())
  if arbiter is not None else None`; when `remote is not None` the four `pm.send(...)` calls below send the
  remote bytes, else the local `*_send` builders (unchanged names). Import at `modeld.py:64`.

## Transport (why raw sockets)

The four SHADOW msgq topics (`contract.SHADOW_OUTPUT_SERVICES` = `offloadModelV2`,
`offloadCameraOdometry`, `offloadDrivingModelData`, `offloadModelDataV2SP`) are registered in
`openpilot/cereal/services.py` but have **no `log.Event` union member**. A cereal `SubMaster` /
`new_message` cannot construct on them; the arbiter therefore reads them with **raw msgq
sockets** (`msgq.sub_sock(name, segment_size=SERVICE_LIST[name].queue_size)` → `receive()`), exactly
as device/tests/test_offloadd.py does. The wire payload's union member is the **real** one
(`modelV2` / `cameraOdometry` / `drivingModelData` / `modelDataV2SP`), so `log.Event.from_bytes`
decodes it normally. Live-verified on this Mac for all four names (sizes 224/152/96/48 B).

Frame binding:
- `modelV2`, `cameraOdometry`, `drivingModelData` carry `frameId`; a message is cached under its
  `frameId`.
- `modelDataV2SP` (`custom.ModelDataV2SP`) has **no** `frameId`; it is bound to a frame by
  **arrival proximity** — nearest `modelV2` arrival within `OFFLOAD_ARB_SP_PROXIMITY_MS` (default
  **20 ms**), matching the spec's "all four of a set arrive within a few ms". Documented choice.

## Gating (default off; device path byte-identical)

`is_arbiter_enabled()` is `True` iff:

1. env `OFFLOAD_ARBITRATION` is truthy (bench), **or**
2. `COMMA_HARDWARE` **and** `Params().get("OffloadMode")` (decoded, stripped, lowercased) == `'drive'`.

Params access is defensive: a missing key (`UnknownKeyName`) or any backend error ⇒ disabled.
When disabled, `make_arbiter()` returns `None`: **no sockets are opened, no shadow topic is read,
and the publish path is untouched** (the local path is byte-identical to today). `COMMA_HARDWARE`
is import-time `os.path.isfile('/AGNOS')`, so on the Mac condition (2) is always false.

Optional tunables: `OFFLOAD_ARB_SP_PROXIMITY_MS` (default 20).

## State machine

States: **DISENGAGED** (default) → **ENGAGED** (remote selected when eligible).

| field | meaning |
|---|---|
| `eligible_streak` | consecutive frames with a complete, in-window, settled set |
| `ineligible_streak` | consecutive ineligible frames |
| `settle_remaining` | frames left in the post-engage settling window |
| `engage_count` / `disengage_count` | transitions (sum = `switches`) |

Per frame being published locally at `frame_id = N`:

1. **Candidate** = newest cached frame `f <= N` that is *complete* and not already consumed
   (a frame is never republished twice).
2. **Eligible** iff a candidate exists, all four messages are cached, and
   `0 <= (now - set_arrival) <= OFFLOAD_ELIGIBLE_MS` (46 ms; `contract.ELIGIBLE_MS`).
3. **Settled** iff, while `settle_remaining > 0` right after an engage, `f >= N - OFFLOAD_SETTLE_N`
   (2); otherwise the frame is counted `settle_skipped` and the local output is published.
4. `eligible && settled` → `eligible_streak += 1`, `ineligible_streak = 0`; else the reverse.
5. **Engage** when DISENGAGED and `eligible_streak >= OFFLOAD_ENTER_N` (10, = 0.5 s): set ENGAGED,
   `settle_remaining = SETTLE_N`, log `engaged`. The engaging frame itself must also pass settling.
6. **Disengage** when ENGAGED and `ineligible_streak >= OFFLOAD_EXIT_N` (3): set DISENGAGED, log
   `disengaged`.
7. Publish remote iff `ENGAGED && eligible && settled` **and** a candidate exists; else local.

`decide()` is an alias of `select()`.

### Cohesion

**Never a mixed-source set within one frame.** A remote set is used only when *all four* messages
are present (`_FrameSet.has_ids()` + proximity-bound SP). If any one piece is missing — e.g.
`modelV2` shadow present but `cameraOdometry` shadow absent — that frame publishes the **local**
outputs for all four. `cameraOdometry` travels with its frame's source.

### Settling

Mirrors Jetlink's "the large model owns its settling frames": for the first frames after an
engage, only a set whose `frameId >= N - SETTLE_N` may be republished; an older-but-fresh set is
settle-skipped (local published for that frame) and counted.

### Cache

Frames are kept keyed by `frameId`: purged when older than `OFFLOAD_ARB_PURGE_S` (default 2 s) or
when more than `max_frames` (default 8) remain (newest kept). SP buffers are purged with the same
horizon.

## Restamp semantics

On engage the remote messages are published under the **real** names with **only** the cereal
`Event` header `logMonoTime` rewritten to device-now (`as_builder()` on the reader, one scalar
write, `to_bytes()`); payload fields are untouched. This matches offloadd's own re-stamp (§1.3)
and is unit-tested byte-for-byte: the emitted real-name bytes must differ from the inbound shadow
payload **only** in `logMonoTime`.

## Observability (cloudlog; no schema change)

Once per second: `eligible`, `ineligible`, `engaged`, `switches`, `settle_skipped`. Transitions log
`engaged` / `disengaged`. `OffloadArbiter.stats()` returns the same counters.

## Deviations from the spec text (code-safe variants)

- **modelDataV2SP frame binding** — the spec anticipated `modelDataV2SP` may lack `frameId`;
  confirmed (custom struct has none). Bound by the 20 ms arrival-proximity window to the modelV2
  of the set (documented above).
- **Settling comparison** — the spec says "`frameId >= current_local_frame_id - SETTLE_N`"; applied
  only within the post-engage settle window (default 2 frames), so it never blocks steady-state.
- **`decide()` name** — kept as an alias of `select()` because the hook is called at the modeld
  publish site.

## Open items

- **(a) `cameraOdometry.valid`** is computed on the Mac by its own drop accounting; this hook
  republishes the remote `valid` unchanged, so remote validity semantics must be **re-validated in
  bench shadow** against locationd's sensitivity history before drive-mode enablement.
- **(b) Mac-computed telemetry** (`frameAge`, `timestampEof`/`unixTimestampNanos`-style model
  timestamps) are forwarded as-is; they reflect Mac frame accounting, not device frame accounting.
- **(c) Per-source lagd re-identification is P4** — when the active source switches
  local↔remote, the delay model (`lagd`/`lat_delay`) is not re-identified here; the switch is
  logged so a P4 job can segment by source and re-fit.
