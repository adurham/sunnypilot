# WS-C — uplink byte-integrity harness: EVIDENCE

**Host:** macOS 27.0.1 (arm64, M4 Max, 36 GB); worktree `/Users/adam.durham/repos/sunnypilot-offload`
@ branch `offload-mac`. **Date:** 2026-10-07 (local CDT). Loadavg during the runs 2.5–4.2 (`uptime`
bracketed each run). All offline on the Mac — no device, no network.

This closes the gap called out as **COMPARISON-COMMS R11 / Top-5 borrow #1**: *"we have ZERO
byte-integrity evidence for the path we actually ship on."* The idea is borrowed from jetlink's
`scripts/comma/jetlink_usb_integrity.py` (digest + frame/offset stamp + forced-interruption stress +
a zero-replays assertion) and **reimplemented against OUR path**. None of their USB-vendor specifics
(gadget, `JLNK` framing, ep0/lending, 16 KB padding) is ported.

## What is actually being proven

The uplink is `replayd` (binds a ZMQ PUB per service on the frozen `ports.py` scheme, publishes
`EncodeData` + cameraState + small services) → `framebridge` (pyzmq SUB, decodes, republishes into
local msgq). This harness proves, **byte-for-byte**, that:

1. every `EncodeData` framebridge *receives* carries exactly the fixture's access-unit bytes
   (`sha256` match vs the fixture record for that `(frame_id, encode_id)`);
2. `frame_id` **and** `encode_id` are **strictly increasing** — zero duplicates and zero
   regressions — across the **whole run**, *including across consumer reconnects*;
3. received vs expected is counted and every never-received frame is classified;
4. a replayed frame (an id pair the fixture never emitted, or a rewound sequence) is caught.

## Components

| File | Role |
|---|---|
| `replay/integrity.py` | `expected <fixture_dir> --out` → per-record `{cam,frame_id,encode_id,sha256,len}`; `check <expected.jsonl> <fb.jsonl>` → verify + classify; `report` = check + one summary line. Exit 1 on ANY mismatch/dup/replay/regression. |
| `mac/framebridge.py` | **new opt-in `--digest-out <path>`**: appends per received `EncodeData` `{kind,cam,frame_id,encode_id,sha256(data),len,recv_mac_ns}`. Flag absent ⇒ **today's behavior, byte-for-byte** (the emit call no-ops). |
| `replay/integrity_run.sh` | end-to-end: expected digests → `replayd` (ZMQ PUB) + `framebridge --digest-out` → checker. `--stress N` kills+restarts framebridge N times mid-run while replayd keeps publishing; all restarts append to the **same** digest jsonl, so the post-reconnect stream is checked in one pass. |
| `replay/tests/test_integrity.py` | unit tests (9) with small synthetic fixtures: round-trip, injected corrupt byte, duplicated row, replayed seq, clean accept, CLI exit codes, `--digest-out` default path. |

## Exact commands

```bash
cd /Users/adam.durham/repos/sunnypilot-offload
uptime   # record loadavg

# (a) fixture digests (ground truth). 113-x17 = 17 contiguous segments = 20400 frames/cam.
PYTHONPATH=$PWD:$PWD/opendbc_repo:$PWD/msgq_repo:$PWD/tinygrad_repo \
  .venv/bin/python -m openpilot.offload.replay.integrity expected \
    ~/.hermes/cache/scratch/car-features/offload/replay-fixtures/113-x17 \
    --out /tmp/expected.jsonl

# (b) CLEAN RUN — 20400 unique frames/cam, full coverage:
bash openpilot/offload/replay/integrity_run.sh --fixture 113-x17 --speed 2

# (c) STRESS RUN — same fixture, 5 mid-run framebridge restarts:
bash openpilot/offload/replay/integrity_run.sh \
  --fixture 113-x17 --speed 2 --stress 5 --stress-interval 60

# (d) manual check (what the script runs internally):
PYTHONPATH=$PWD:$PWD/opendbc_repo:$PWD/msgq_repo:$PWD/tinygrad_repo \
  .venv/bin/python -m openpilot.offload.replay.integrity check \
    <run>/expected.jsonl <run>/digest.jsonl --json <run>/integrity.json

# unit tests
PYTHONPATH=$PWD:$PWD/opendbc_repo:$PWD/msgq_repo:$PWD/tinygrad_repo \
  .venv/bin/python -m pytest openpilot/offload/replay/tests/test_integrity.py -q   # 9 passed
```

The `113-x17` fixture was built once (17 consecutive `00000113--8cc572ac6e--{0..16}` segments,
`encodeId_contiguous=True`, 20400 frames/cam, extracted 2026-10-07) with:

```bash
PYTHONPATH=$PWD:$PWD/opendbc_repo:$PWD/msgq_repo:$PWD/tinygrad_repo \
  .venv/bin/python -m openpilot.offload.replay.extractor \
    ~/comma-routes/00000113--8cc572ac6e--{0..16} --name 113-x17
```

It lives outside the repo (`replay-fixtures/` is git-ignored) and is reused from then on.

## Results table

Run root: `~/.hermes/cache/scratch/car-features/offload/integrity/<ts>-<fixture>/`
(kept — it is the evidence; `integrity.json` is the machine-readable result).

| run | fixture | frames/cam | speed | restarts | expected | received | missing | **mismatch** | **dup** | **replay** | regression | digest match rate | verdict |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| **CLEAN (full coverage)** | `113-x17` (17 seg) | **20400** | 2 | 0 | 40800 | **40800** | **0** | **0** | **0** | **0** | **0** | **1.000000** | **PASS** |
| **STRESS (5 restarts)** | `113-x17` | **20400** | 2 | **5** | 40800 | 39580 | 1220 | **0** | **0** | **0** | **0** | **1.000000** | **PASS** |
| stress smoke | `149-36` (1 seg) | 1200 | 4 | 2 | 2400 | 1518 | 882 | 0 | 0 | 0 | 0 | 1.0 | PASS |

Run dirs: clean `20261007-091731-113-x17`, stress `20261007-092650-113-x17`.

**Clean run `integrity.json` totals:**
`{"status":"PASS","expected":40800,"received":40800,"missing":0,"mismatch":0,"duplicate":0,"replay":0,"regression":0,"digest_ok":40800,"digest_match_rate":1.0}`

**Stress run `integrity.json` totals:**
`{"status":"PASS","expected":40800,"received":39580,"missing":1220,"mismatch":0,"duplicate":0,"replay":0,"regression":0,"digest_ok":39580,"digest_match_rate":1.0}`

**Digest match rate: 1.000000 on both** — every single received frame's payload digest equals the
fixture's digest for that `(frame_id, encode_id)`. Zero corrupted bytes out of 80400 received
frames across the two runs.

The clean run is **40800/40800 = 100 % coverage, zero missing** — an end-to-end lossless pass of
20400 unique frames per camera. The stress run trades coverage (5 reconnect windows) for the
restart assertion below; both keep `mismatch=dup=replay=regression=0`.

### Reconnect-stress result (the interesting one)

`--stress 5 --stress-interval 60` killed and restarted framebridge **5 times** mid-run (60 s apart)
while replayd kept publishing. The framebridge log shows **6 instances** (`[framebridge] VisionIPC
server 'camerad' listening` ×6, `Killing old publisher: visionipc_camerad_*` on each restart) and 6
`summary:` lines; each instance's `received` count lands in the **same** `digest.jsonl`:

```
narrow per instance: 2338 + 2399 + 2401 + 2402 + 2399 + 7851 = 19790   (== checker "received" narrow)
digest recv_mac_ns span: 510.0 s of continuous publishing
narrow frame_id strictly_increasing=True
framebridge log: 0 MultiplePublishersError / Traceback / Exception
```

So the post-reconnect stream has **zero duplicated frameIds, zero regressions, zero digest
mismatches**, and only the reconnect-boundary gaps show up — as **missing** counts
(`missing_frame_ids` in `integrity.json` are contiguous runs, i.e. the ~3 s framebridge rebuild
window per restart), which are counted and **not** failed. This is exactly the required assertion:
*kill+restart the consumer N times mid-run, then assert the post-reconnect stream has zero
duplicated frameIds and zero digest mismatches for received frames; missing at the boundary is
acceptable and counted.*

## 100,000-frame soak (documented command)

For a 100k-frame soak, reuse the same fixture with `--loop` at `--speed max` so replayd blasts the
1021 s device span as fast as the socket allows. `--loop` re-emits the same ids, so the checker will
(by design) flag the second pass as **duplicates/replays** — which is the *correct* behavior of an
integrity checker and is itself a positive test of the duplicate/replay detectors:

```bash
# soak: 100k+ frames; the 2nd/3rd passes exercise the dup/replay detectors
PYTHONPATH=$PWD:$PWD/opendbc_repo:$PWD/msgq_repo:$PWD/tinygrad_repo \
  .venv/bin/python -m openpilot.offload.replay.replayd \
    ~/.hermes/cache/scratch/car-features/offload/replay-fixtures/113-x17 \
    --fixture-dir ~/.hermes/cache/scratch/car-features/offload/replay-fixtures/113-x17 \
    --host 127.0.0.1 --speed max --loop --pub-log /tmp/soak_pub.jsonl &
# framebridge --digest-out /tmp/soak_digest.jsonl  (as integrity_run.sh does)
# checker: expected vs /tmp/soak_digest.jsonl
```

For a **lossless** 100k soak (all unique frames, zero loop-induced dups) extract a longer fixture
instead — route `113` has 77 contiguous segments (~92 400 frames/cam); extend `--name 113-x77`
over `--{0..76}` and run `integrity_run.sh --fixture 113-x77 --speed 2` (no `--loop`).

## Honesty / limitations

* **Missing ≠ corruption.** The clean run had **zero missing** (lossless). The stress run missed 1220
  (3 %) — those are the five reconnect-boundary windows, not device frame loss. Both are counted and
  kept out of the pass/fail decision (a reconnecting consumer must be allowed to skip the boundary);
  `--strict-missing` turns missing into a failure when you want an end-to-end lossless assertion.
* **Receiver startup race.** A framebridge started immediately after a previous one exits can hit a
  pre-existing msgq `MultiplePublishersError` on `<cam>RoadCameraState` (stale publisher in the shm
  segment), which kills only *that* camera's SUB thread. `integrity_run.sh` now waits 8 s after
  processes clear to let the segments settle; two earlier iterations lost a full narrow stream to
  this. This is a WS-B/msgq property, not the digest path.
* **Throughput ceiling is the Mac receiver**, not the link: the digest rate is one `sha256` per
  received frame plus framebridge's decode of both cameras. The wire is not the bottleneck here.
* The digest covers `EncodeData.data` (the AU bytes) **only** — matching exactly what the device
  puts on the wire (`fixture.py` AU == qlog `EncodeIndex.len`). Parameter sets in `.header` are
  replayed separately and are not part of the digest (they are not in `.data`).
* `recv_mac_ns` is Mac monotonic ns; the per-instance boundary is recoverable from the framebridge
  `summary:` lines when you need to attribute a missing run to a specific reconnect.

## Pointer

`replay/RESULTS.md` carries a one-line pointer to this file.
