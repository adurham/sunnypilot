# WS-D — device side (offload target)

Owner: WS-D. Everything under `openpilot/offload/device/`. Conforms to the FROZEN
[`../INTERFACES.md`](../INTERFACES.md) and [`../contract.py`](../contract.py); the only
non-device file touched is a minimal, param-gated pair of entries in
`openpilot/system/manager/process_config.py` (P2-prep).

```
device/
  offloadd.py            device-side return-path forwarder (P2 code-complete, P4-prep) + HOLD/BEHIND/LOST
  check_device.sh        read-only ssh audit (run from the Mac)
  bench/                 user-executed bring-up / smoke / tuning / latency / resources + P3 checklist
  tests/test_offloadd.py     offline unit tests (Mac; no device needed)
  tests/test_holdpolicy.py   offline tests for the HOLD/BEHIND/LOST eligibility machine
  README.md  RESULTS.md
```

## Architecture as-built

```
 DEVICE (/data/openpilot)                                   MAC
 ┌──────────────────────────┐                         ┌───────────────────────────┐
 │ camerad/loggerd → msgq    │  ZMQ PUB  *:fnv1a(name)  │ WS-B framebridge           │
 │ offload_bridge (./bridge) ├────────────────────────►│  vtdec → VisionIPC         │
 │   = msgq→zmq, ALL services│   (FORWARD_* on connect- │  Mac msgq republish        │
 │                          │    list selects the set) │ WS-A modeld_v2 (METAL)     │
 │ LOCAL modeld → msgq      │                          │  modelV2/cameraOdometry/   │
 │   (the real modelV2/…)   │  ZMQ PUB (Mac binds)     │  drivingModelData/         │
 │ msgq ◄── offloadd ◄───────┼──────────────────────────┤  modelDataV2SP (Mac PUB)   │
 │   offload* SHADOW names  │  RETURN_SERVICES         │                            │
 │   header→device now (§1.3)│                          │                            │
 └──────────────────────────┘                         └───────────────────────────┘
```

- **Forward (device→Mac):** `./bridge` re-publishes every msgq service on
  `tcp://*:8023 + fnv1a(name) % (65535-8023)` (ports per `../ports.py`). The Mac SUB's
  connect-list effectively selects the service set.
- **Return (Mac→device):** `offloadd` SUBs to the Mac's PUBs, freshness-gates on the **device**
  clock, re-stamps the cereal header `logMonoTime` to device-now, and forwards into the **local**
  msgq — under the **SHADOW** service names `offloadModelV2` / `offloadCameraOdometry` /
  `offloadDrivingModelData` / `offloadModelDataV2SP`, **never** the real names. No clock sync
  anywhere (§1.4).
- **Single publisher (reframe 2026-10-07, INTERFACES §7):** the device's **local** model keeps
  computing AND publishing the real `modelV2`/`cameraOdometry`/`drivingModelData`/`modelDataV2SP`
  every frame; the remote output is a *shadow*. `offloadd` never publishes a real service name
  and never removes one. The local-vs-remote choice (arbitration, hysteresis, settling) lives in
  the device `modeld_v2`, not in `offloadd`.
- **Echo prevention:** the Mac must **not** subscribe to `modelV2` (etc.) on the device ZMQ
  while it is republishing the same services back — that is an infinite echo. Keep the return
  PUB on the Mac and the forward SUB list disjoint from `RETURN_SERVICES`.

## offloadd operational rules (read before enabling)

1. **Single publisher.** `offloadd` forwards only under the shadow names; the real services are
   the local model's. Never add a second publisher of a real service name.
2. **Do NOT live-stream during offload.** `livestream*` encode data shares the encode path and
   the USB link budget; it perturbs the frame timing gates. Kill/disable livestream for the
   whole offload session.
3. **Freshness is `now_device - timestamp_sof`** (SOF, not EOF), threshold `OFFLOAD_STALE_MS`
   (default 300 ms). Stale ⇒ drop, **never forward**. A message with any non-finite payload float
   is likewise dropped.
4. **Absence of a remote NEVER removes model output.** When no eligible remote output exists for
   a frame, the shadow topics simply go quiet; the local model still publishes. The shadow
   going quiet is the remote's staleness signal **for the modeld_v2 arbitration**, not for
   `controlsd`.
5. **HOLD/BEHIND/LOST eligibility (default OFF).** `OFFLOAD_HOLDPOLICY=1` (or `OffloadMode='holds'`
   or `--hold-policy`) runs the ported Jetlink timing machine: per-frame 46 ms HOLD →
   `remote_eligible=False`; 5-in-a-row or >20-in-10 s → a `behind` event; 200 ms of no Mac
   message → a `lost` event; recover on the next message. Its output is an eligibility signal +
   stdout events, not a stop-publishing action. Default off preserves the pre-port behavior.
6. **OffloadMode gates everything.** `offloadd` is inert unless the device param `OffloadMode` ∈
   {`shadow`,`drive`,`holds`}. With the param absent (default) nothing offload-related starts.
7. **Stats go to stdout only** (JSONL: `forwarded`, `dropped_stale`, `dropped_nonfinite`,
   `dropped_unpaired`, `sp_paired`, …). Bench mode may append `/tmp/offloadd_bench.jsonl`.
   **No `/data` writes** — `/data` is ~90% full.
8. **`modelDataV2SP` is forwarded only when paired (INTERFACES §7 "SP pairing rule").** It carries
   no `frameId` and no timestamps (`custom.ModelDataV2SP`), so it cannot be aged on its own. It is
   held in a single-slot buffer and forwarded **only** if a message that passed the freshness +
   finite gate (`modelV2` / `cameraOdometry` / `drivingModelData`) was received within
   `OFFLOAD_SP_PAIR_MS` (default **25** ms) of it, either side first. Otherwise it is dropped and
   counted `dropped_unpaired` — never forwarded blind. A stale- or non-finite-dropped partner never
   anchors a pair; pairing is one-to-one (a newer SP supersedes an older pending one, also counted
   `dropped_unpaired`). `dropped_no_sof` still counts any *other* service that lacks an age source.
   Without this the arbiter (modeld_v2) can never see a complete 4-piece remote set, so the remote
   path could never engage.

### Shadow service names — a capnp prerequisite (INTERFACES §7)

The four shadow names are registered in `openpilot/cereal/services.py` (queue sizing) **and must
be added as members of the `log.Event` union in `openpilot/cereal/log.capnp`** before a cereal
`SubMaster` can read them (writing on the raw msgq topic already works). `log.capnp` is frozen /
PM-owned; see `RESULTS.md` §5 for the exact failure and the CHANGES REQUESTED item.

## Tether bring-up (USER-GATED)

Use the **AUX** USB-C port (USB 3.1 Gen2, UDC `a600000.dwc3`), never OBD-C/panda.

```bash
bash openpilot/offload/device/bench/01_enable_tether.sh comma@comma-b203ed6e.local   # AdbEnabled=1 + start adbd
bash openpilot/offload/device/bench/02_mac_side.sh    comma@comma-b203ed6e.local       # find link, ping, iperf3
bash openpilot/offload/device/bench/05_device_tuning.sh comma@comma-b203ed6e.local --apply  # sysctls + FFS log off
bash openpilot/offload/device/bench/06_latency_roundtrip.sh comma@comma-b203ed6e.local 2000   # return-path p99/p99.9
bash openpilot/offload/device/bench/03_live_smoke.sh  169.254.x.y 30                    # framebridge smoke (needs WS-B)
bash openpilot/offload/device/bench/05_device_tuning.sh comma@comma-b203ed6e.local --revert   # restore sysctls
bash openpilot/offload/device/bench/04_disable_tether.sh comma@comma-b203ed6e.local    # AdbEnabled=0 + stop adbd
```

`/usr/comma/set_adb.sh` builds the NCM+FunctionFS gadget (configfs under
`/config/usb_gadget/g1`, then `echo a600000.dwc3 > .../UDC`). On the device, configfs + F_FS +
NCM are built-in; **F_UVC is not built** and there are no loadable modules. The `adbd` systemd
unit is gated on `AdbEnabled=1`.

### Bridge bind limitation (documented wish)

`./bridge` binds `tcp://*:<port>` (all interfaces) — **the binary has no bind-ip flag**
(confirmed in `openpilot/cereal/messaging/bridge_zmq.cc`: `full_endpoint = "tcp://*:"`). To
restrict exposure to the tether interface only, use an OS firewall (e.g. nftables `iifname
"usb0" accept; ... drop` on the offload ports) — not a bridge flag. Tracked as OPEN.

## Manual start (if the manager edit is not used on the target)

```bash
# on the device (read-only tree at /data/openpilot; run from its root)
./openpilot/cereal/messaging/bridge &                                   # forward path
OFFLOAD_MODE=shadow /usr/local/venv/bin/python3 -m openpilot.offload.device.offloadd --connect-host <mac-ip>
```

## Failure modes

| Mode | Detected by | Daemon action | System effect |
|---|---|---|---|
| link death | ZMQ recv timeout budget exhausted | stop republishing; open gap | modelV2 staleness → soft-disable |
| bridge death (device) | local camera state stalls | SOF map ages out → all drops | gap → soft-disable |
| Mac pipeline stall/lag | return frame SOF age > stale_ms | drop + count stale | no fresh modelV2 → soft-disable |
| segment boundary | cameraOdometry frameId reset/gap | drop unmatched; keep publishing | brief stale window; loggerd unaffected |
| unpaired `modelDataV2SP` | no fresh aged-able msg within `OFFLOAD_SP_PAIR_MS` (25 ms) | hold pending; drop + count `dropped_unpaired` | SP shadow quiet → arbiter sees an incomplete set → local publishes that frame |
| encoder GOP gap | no return msg for a frame id | nothing to publish for it | handled upstream (WS-B) |
| ZMQ HWM drops | SUB silently drops | counted via recv gaps | gap watchdog covers the symptom |
| echo loop | Mac SUB sees its own republish | MUST NOT HAPPEN | operational rule 1 |

## User-gated steps (require an explicit human action)

- Tether: `01_enable_tether.sh` (param `AdbEnabled=1` + `systemctl start adbd`) and
  `04_disable_tether.sh`.
- Enabling offload: writing param `OffloadMode` ∈ {`shadow`,`drive`} (absent by default).
- Any deploy/reboot of the device tree (PM/owner; not WS-D).
