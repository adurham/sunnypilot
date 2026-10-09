# WS-D results — evidence

All commands run from the worktree root
(`/Users/adam.durham/repos/sunnypilot-offload`, branch `offload-mac`) with
`PYTHONPATH=$PWD:$PWD/opendbc_repo:$PWD/msgq_repo:$PWD/tinygrad_repo .venv/bin/python ...`.
Machine: macOS arm64, Python 3.12.13. No device writes; `msgq` resolves to the sibling
editable build at `/Users/adam.durham/repos/sunnypilot/msgq_repo`.

## 1. Device audit (read-only) — BLOCKED by ssh auth, retried 3×

```
$ ssh -o BatchMode=yes -o ConnectTimeout=8 comma@comma-b203ed6e.local 'echo SSH_OK'
sign_and_send_pubkey: signing failed for RSA "Personal Macbook" from agent: communication with agent failed
comma@comma-b203ed6e.local: Permission denied (publickey).
exit=255      # retried 3× over ~2 min: exit 124/255 each time
```

Device is up and reachable — `ping comma-b203ed6e.local` → `64 bytes ... time=6.656 ms`
(192.168.86.24) — but ssh fails because the **1Password ssh-agent is not answering** (no
local key files). No ssh command could be executed, so nothing on the device was read.

`check_device.sh` behaves correctly when ssh fails:

```
$ bash check_device.sh nobody@203.0.113.7 --no-retry
== offload device audit ==
host: nobody@203.0.113.7   (Wed Oct  7 04:00:03 CDT 2026)

FAIL connectivity               ssh failed after retries — load your ssh key (1Password), check mDNS

SUMMARY: PASS=0 WARN=0 FAIL=1  -> NOT AUDITED
exit=1
```

### Device checks — ASSUMED from this session's facts (not re-verified live)

AGNOS Ubuntu 24.04 / kernel 4.9.103; device python `/usr/local/venv/bin/python3` (3.12,
capnp+pyzmq+zstandard); cereal bridge binary at
`/data/openpilot/openpilot/cereal/messaging/bridge`; `iperf3` present; `/data` ~90% full;
`OffloadMode` absent; device runs prebuilt tree, branch `dev`.

### Device-python port dump — validated locally against the frozen scheme

The port-dump heredoc embedded in `check_device.sh` was executed locally against the same
`cereal/services.py` it targets on the device (identical file), proving the output shape:

```
FWD    narrowRoadEncodeData           port=52737  present=True
FWD    wideRoadEncodeData             port=42305  present=True
FWD    narrowRoadCameraState          port=20911  present=True
FWD    wideRoadCameraState            port=53095  present=True
FWD    carState                       port=9041   present=True
FWD    deviceState                    port=17667  present=True
FWD    carControl                     port=63225  present=True
FWD    extrinsicsCalibration          port=32584  present=True
FWD    driverMonitoringState          port=31369  present=True
FWD    lateralDelay                   port=34210  present=True
FWD    modelV2                        port=58537  present=True
FWD    drivingModelData               port=49244  present=True
FWD    cameraOdometry                 port=50972  present=True
FWD    modelDataV2SP                  port=12568  present=True
RET    modelV2  port=58537   RET cameraOdometry  port=50972
RET    drivingModelData  port=49244   RET modelDataV2SP  port=12568
MISSING: []          # every FORWARD + RETURN service is present on the device schema
```

## 2. offloadd unit tests (offline, no device) — 13/13 PASS

> Reframe 2026-10-07: the suite is now **27** tests (these 13 + `test_holdpolicy.py`); see §5.
> SP pairing rule 2026-10-07: now **47** (`openpilot/offload/device`); see §8.

```
$ PYTHONPATH=... .venv/bin/python -m pytest openpilot/offload/device/tests/test_offloadd.py -v
collected 13 items
test_enabled_mode ................................................ PASSED
test_read_offload_mode_tolerates_missing_key .................... PASSED
test_sof_for_service_prefers_frameid_map ....................... PASSED
test_sof_for_service_none_when_unageable ....................... PASSED
test_restamp_only_changes_header ................................ PASSED
test_restamp_payload_byte_identical ............................. PASSED
test_restamp_all_return_services_roundtrip ...................... PASSED
test_fresh_message_is_restamped_and_published ................... PASSED
test_stale_message_is_dropped_not_republished ................... PASSED
test_no_sof_is_dropped .......................................... PASSED
test_gap_watchdog_fires_then_clears ............................. PASSED
test_wrong_service_on_port_is_not_republished ................... PASSED
test_stats_json_shape ........................................... PASSED
============================== 13 passed in 2.24s ==============================
```

The tests use a real localhost ZMQ PUB/SUB pair (with a socket-monitor ACCEPTED barrier) plus
the in-process cereal msgq, and inject the device clock via `step(now_ns=...)`, so
freshness/stale/gap semantics are deterministic. `test_restamp_payload_byte_identical`
proves byte-level: normalizing the re-stamped header back reproduces the original bytes
exactly → payload fields untouched, header changed.

## 3. Lint (fork tools; `lint.sh` is git-tracked-only and WS-D cannot `git add`, so its
   individual checks are run directly on the added files)

```
$ ruff check openpilot/offload/device/ openpilot/system/manager/process_config.py   -> All checks passed
$ check_indentation.py  (all device .py + process_config.py)                          -> ok
$ check_shebang_format.sh           (check_device.sh + bench/*.sh)                    -> ok
$ check_shebang_scripts_are_executable.py (same)                                      -> ok
$ check_nomerge_comments.sh         (offloadd.py, process_config.py)                  -> ok
$ codespell ... --ignore-words=.codespellignore                                       -> ok
```

Manager gate logic verified in isolation (AST-extracted, executed with a stub Params):
absent param → False; `UnknownKeyName` → False; `b"shadow"`/`"drive"`/`" Drive "` → True;
`b"off"`/empty/undecodable → False. Both new procs (`offload_bridge`, `offloadd`) are
`and_(iscar, offload_mode_any)`-gated.

## 4. Known environment caveats

- **`test_manager.py` cannot collect on macOS** (`OSError: dlopen(... libparams_c.dylib)`),
  including at baseline before my edit — `libparams_c` is a device/CI-built native lib. The
  manager change was therefore validated by ruff + AST-extracted gate-logic execution, not by
  running the fork's manager test.
- **`openpilot/common/params.py` is not importable locally** (same missing dylib), so the
  `OffloadMode` gate in `offloadd.main()` is only exercisable on-device; the pure
  `enabled_mode()`/`read_offload_mode()` helpers (which the daemon routes through) are unit
  tested.
- **`submodules` are empty in this worktree**, but `msgq` resolves via the `openpilot`
  editable `.pth` to the sibling `sunnypilot/msgq_repo` build; `openpilot.cereal.messaging`
  and `log` import and operate in-process.

## 5. Jetlink borrows — HOLD/BEHIND/LOST eligibility machine + device bench tooling

Ported from `jetlink/openpilot/model_state.py` (`_note_hold`, `HOLD_FRAME=0.046`) and
`docs/link-protocol.md` "Late replies", **reframed to our single-publisher architecture**
(INTERFACES §7, 2026-10-07): the device's LOCAL model keeps computing AND publishing every frame;
`offloadd` only receives the remote (Mac) outputs, freshness-validates them, and forwards each
eligible one into device msgq under SHADOW service names. Feature flag `OFFLOAD_HOLDPOLICY=1`
(or `OffloadMode='holds'`, or `--hold-policy`); **default off preserves the pre-port behavior
exactly** (`published` kept as a compat alias of the new `forwarded`).

### State-machine mapping (theirs → ours)

| Jetlink (comma side) | OURS (device side) |
|---|---|
| `HOLD_FRAME=0.046` from the frame's warp start | `OFFLOAD_HOLD_MS=46` from the frame's first arrival at offloadd |
| held → republish previous frame's output | a HOLD means "no eligible remote output for this frame"; **nothing is published** |
| `HOLDS_IN_A_ROW=5` | `OFFLOAD_HOLDS_IN_A_ROW=5` |
| `HOLDS_ALLOWED=20` within `HOLD_WINDOW=10 s` | `OFFLOAD_HOLDS_ALLOWED=20` within `OFFLOAD_HOLD_WINDOW=10 s` |
| `behind` → hand the drive to the small model | `behind` **event** + `remote_eligible=False`; arbitration (modeld_v2) decides |
| `PROVING_FRAMES=20` / `SETTLING_FRAMES=3` | `OFFLOAD_PROVING_FRAMES` / `OFFLOAD_SETTLING_FRAMES` |
| lost: no answer for 0.2 s | `OFFLOAD_LOST_MS=200`: no Mac message at all → `lost` event, reset; recover on next msg |

**Reframe note:** absence of a remote message NEVER removes model output — the local model always
publishes the real `modelV2`/`cameraOdometry`/`drivingModelData`/`modelDataV2SP`. The state
machine's output is an ELIGIBILITY signal (`remote_eligible`) + stdout events (`hold`, `behind`,
`lost`, `recovered`), not a stop-publishing action; a `behind` frame still forwards later eligible
messages. The correctness rule carried over — "never replace state with a non-finite/failed
output" — becomes **never forward a stale or non-finite message**: a message older than
`OFFLOAD_STALE_MS` at receipt is dropped, and a message with any NaN/Inf float is dropped.

### Unit tests — 27/27 PASS (2× stable)

```
$ PYTHONPATH=... .venv/bin/python -m pytest openpilot/offload/device/tests/ -q
27 passed in 9.42s      # test_offloadd.py 14 + test_holdpolicy.py 13
```

`test_holdpolicy.py`: HOLD fires at 46 ms (not at 45); HOLD makes the remote ineligible and
forwards nothing; a remote output is forwarded under the shadow name with the header re-stamped
and the payload intact; stale- and non-finite-message drop; 5-consecutive → `behind` + ineligible
(and a later message still forwards); >20-in-window → `behind`; LOST at 200 ms + recovery;
default-off leaves the policy inert. `test_offloadd.py` gained
`test_nonfinite_message_is_dropped_not_forwarded` and now asserts forwarding under
`offloadModelV2` (raw msgq), not a real modelV2.

### Cereal service-name validation — REQUIRED before this can run on the device

Reported for the join task (the §7 shadow names must be registered the same way on both sides):
1. **`openpilot/cereal/services.py`** — the four shadow names were NOT in `SERVICE_LIST`, so
   `pub_sock`/`sub_sock` sized them `segment_size=0`. **Fixed here**: added `offloadModelV2`
   (BIG), `offloadCameraOdometry` (SMALL), `offloadDrivingModelData` (SMALL), `offloadModelDataV2SP`
   (BIG), matching `contract.SHADOW_OUTPUT_SERVICES`. Registration is mandatory: an unregistered
   topic with a >0 message **ABORTS the process** (`msgq_msg_send` assertion `3 * total_msg_size
   <= q->size`), proven on this Mac. `offloadd` reads the names from `contract.SHADOW_OUTPUT_SERVICES`
   and asserts its map matches, so a contract change breaks the daemon loudly, not silently.
2. **`openpilot/cereal/log.capnp`** — the names are NOT members of `log.Event`'s union. That is a
   **hard blocker for cereal *consumption***: `SubMaster([...])` calls `new_message(name)` →
   `dat.init(name)` → `KjException: struct has no such member; name = offloadModelV2`. `offloadd`'s
   **WRITE path is unaffected** — it forwards raw Event bytes with `msgq.pub_sock(name, size)` /
   `send`, never `Event.init(service)` — and the tests read them back on the **raw msgq topic**
   (`msgq.sub_sock`) and then decode with `log.Event.from_bytes` + `which()` (which works, since
   the payload union member is the real `modelV2`/etc.). But the **modeld_v2 join side needs a
   typed read of the shadow topic**, so a capnp member must be added per shadow name (e.g.
   `offloadModelV2 @<id> :ModelDataV2;` — same payload type), regenerated, and `services.py` kept
   in sync. `openpilot/cereal/log.capnp` is **frozen and PM-owned**; this is a CHANGES REQUESTED
   item for PM (the §7 text "python-cereal only (no capnp change)" is accurate for `offloadd`
   but not yet for a typed consumer).

### Cost measurement (guards the join-task budget)

A recursive finite-scan of a synthetic modelV2 (6208 B, `position`/`velocity`/`acceleration`/
`laneLines` floats as `to_dict` emits them) measured **0.024 ms/msg** on the bench Mac — the
non-finite guard is affordable at 20 Hz (two orders of magnitude under one frame).

## 6. P3/P4 measurement: on-device shadow cost (Jetlink W2 warning)

Jetlink **tried per-frame shadow and removed it** (2026-10-05): shadow frames cost the comma
**~9 ms of every frame** while a window stayed shut, starving the driver-monitoring model under
selfdrived's frequency floor (`joining.py:28-34`, COMPARISON-COMMS.md W2). Our design is additive
(no device fork, encoderd/cereal tap), so the mechanism differs — but the lesson is direct: **any
on-device shadow validation must budget its per-frame cost deliberately**, and the
"local always publishes + remote eligible when fresh" reframe is exactly the always-on variant
they found unsafe. Flag as a **measurement for P3/P4**: instrument the added per-frame cost of the
shadow path (bridge + any join-side comparison) with `bench/bench_resources.py` alongside the P3
soak, and do not assume it is free. This is a stronger reason to gate return-path/arbitration work
behind `OffloadMode` and to keep the local model authoritative.

## 7. Arbitration hook (§7) — device modeld_v2 consumes the shadow topics

The consumer of this daemon's SHADOW output is implemented in WS-A:
`openpilot/sunnypilot/modeld_v2/offload_arbiter.py` (state machine + raw-msgq reader) wired at
three call sites in `openpilot/sunnypilot/modeld_v2/modeld.py` (`:498` construct, `:569` poll,
`:677` select). Full spec, gating, cohesion/settling and open items: `openpilot/offload/ARBITRATION.md`.
Default off: enabled only by `OFFLOAD_ARBITRATION` (bench) or (device: `COMMA_HARDWARE` and
`OffloadMode == 'drive'`); otherwise modeld_v2's publish path is byte-identical. It reads the four
shadow topics with **raw msgq sockets** (no `log.Event` union member exists), publishes the remote
set under the real names only when ENGAGED and an eligible+settled complete set exists, re-stamping
only the header `logMonoTime` to device-now, and otherwise publishes its own local outputs.

## 8. SP pairing rule (INTERFACES §7, added 2026-10-07) — `modelDataV2SP` forwarding

**Gap closed.** `modelDataV2SP` (`custom.capnp ModelDataV2SP`: `laneTurnDirection` + 2 bools) has
no `frameId` and no timestamps, so offloadd could not age it and dropped it as `dropped_no_sof`.
But the arbiter's candidate set requires all four pieces (`has_ids() and _resolve_sp(fs) is not
None`), so without SP forwarding the remote path could never engage. offloadd now forwards SP under
a **pairing rule**, still never blind.

| Rule | Behavior (`offloadd.py`) |
|---|---|
| Buffering | SP is NOT aged. Validated once (right service / finite / decodable), then held in a **single-slot** pending buffer as (raw bytes, receipt ns). |
| Pair | Forwarded (shadow `offloadModelDataV2SP`, header restamped at forward time, payload byte-identical) **only** if a message that passed freshness+finite (modelV2 / cameraOdometry / drivingModelData) was received within `OFFLOAD_SP_PAIR_MS` (default 25, **inclusive**) of the SP. Either may arrive first. |
| No partner | Window elapsed with no partner → dropped, `dropped_unpaired += 1`. SP shadow topic stays silent. |
| Stale / non-finite partner | Dropped before it can anchor anything → its nearby SP goes unpaired. |
| One-to-one | A partner is consumed by the SP it pairs with; a newer pending SP supersedes an older one (older counted `dropped_unpaired`). One fresh message cannot launder a stream of SPs. |
| Counters | New `dropped_unpaired` (replaces `dropped_no_sof` for SP) and `sp_paired`, both in the stats JSON. `dropped_no_sof` is unchanged for any other service without an age source. |
| Env | `OFFLOAD_SP_PAIR_MS` (default 25; garbage → default). |

**Receipt stamps are true drain times, not the step clock.** The step stamps `now_ns` at the TOP of
`step()`, before the camera `SubMaster.update` that paces the loop and can block up to
`recv_timeout_ms` (100 ms). Pairing proximity therefore uses a fresh monotonic read taken when each
message is drained (`_recv_ns`; the injected `now_ns` in tests). Using the step stamp would let a
long camera block in the *later* step silently over-pair an SP with a partner that is really tens of
ms away — the unsafe direction, invisible downstream (the arbiter cannot tell). A review (consult)
flagged exactly this; `test_long_camera_block_in_later_step_does_not_overpair_sp` pins it. While a
pairing is in flight (pending SP, or a fresh unclaimed partner) the camera poll is shortened to
`SP_PAIR_POLL_MS` (5 ms) so a straggler straddling two drains still lands inside the window
(`test_sp_straddling_two_steps_still_pairs_on_true_stamps`). Residual limit: ZMQ gives no arrival
timestamp, so "receipt" is the moment the message is drained from the SUB socket.

**Deviations from the §7 text (all stricter, none looser):**
1. The pair/forward/drop decision runs once per `step()` AFTER the whole socket drain (not inside the
   per-message handler), so it does not depend on which per-service socket is drained first.
2. Pairing is one-to-one (partner consumed; newer SP supersedes older). The literal text would
   forward both SPs when two frames' messages are drained in one step (a backlog).
3. `sp_paired` counter added (observability for the new path; not in the §7 text).

**Tests** — `openpilot/offload/device`: **47 passed** (was 27). New/flipped:
`test_sp_alone_without_partner_is_dropped_unpaired` (c), `test_sp_before_modelv2_within_window_pairs_both`
(a), `test_sp_and_modelv2_same_drain_either_send_order_pairs[True/False]` (a, order-irrelevant),
`test_sp_after_modelv2_within_window_pairs_both` (b), `test_stale_modelv2_with_nearby_sp_forwards_neither`
(d), `test_nonfinite_modelv2_with_nearby_sp_forwards_neither`, `test_full_pair_frameid_preserved_and_sp_payload_byte_identical`
(e), `test_sp_after_partner_window_boundary[25/26 ms]`, `test_sp_burst_single_slot_only_newest_pairs`,
`test_one_anchor_pairs_at_most_one_sp`, `test_no_fresh_partner_never_forwards_sp` (the "no fresh
partner ⇒ not forwarded" safety property), `test_wrong_service_on_sp_port_is_not_buffered`,
`test_sp_pair_ms_default_and_env`, the two true-stamp tests above, `test_camera_poll_shortens_only_while_sp_pairing_in_flight`,
and in `test_holdpolicy.py` the pair / lone-SP cases under the hold policy. `test_stats_json_shape`
now also asserts `dropped_unpaired` and `sp_paired`.

**Integration test needs a PM edit.** `mac/tests/test_returnsend.py::test_full_return_chain_into_shadow_topics`
(the target of the frozen `mac/return_path_test.sh`, which itself only delegates to that pytest id
and hard-codes no counts) pins the OLD behavior — `fwd_delta == 3`, `noseof_delta == 1`,
`"offloadModelDataV2SP" not in msgs` — and now fails with `expected 3 forwards for the round, got 4`.
A ready-to-apply flipped version (proven 3× against the final daemon, and `patch --dry-run` clean
against the frozen file) is at `~/.hermes/cache/scratch/sp-pair/test_returnsend_sp_pairing.patch`;
it also fixes the test helper `_drain_shadow(want_frame=...)`, which silently discarded the SP because
`_frame_id()` is `None` for a frameId-less message.


