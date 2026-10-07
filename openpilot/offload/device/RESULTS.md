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
