# WS-D — device side (offload target)

Owner: WS-D. Everything under `openpilot/offload/device/`. Conforms to the FROZEN
[`../INTERFACES.md`](../INTERFACES.md) and [`../contract.py`](../contract.py); the only
non-device file touched is a minimal, param-gated pair of entries in
`openpilot/system/manager/process_config.py` (P2-prep).

```
device/
  offloadd.py            device-side return-path republisher (P2 code-complete, P4-prep)
  check_device.sh        read-only ssh audit (run from the Mac)
  bench/                 user-executed tether bring-up / smoke / teardown + P3 checklist
  tests/test_offloadd.py offline unit tests (Mac; no device needed)
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
 │ msgq ◄── offloadd ◄───────┼──────────────────────────┤  modelV2/cameraOdometry/   │
 │   republish RETURN_SERVICES│  ZMQ PUB (Mac binds)     │  drivingModelData/         │
 │   header→device now (§1.3) │  RETURN_SERVICES         │  modelDataV2SP (Mac PUB)   │
 └──────────────────────────┘                         └───────────────────────────┘
```

- **Forward (device→Mac):** `./bridge` re-publishes every msgq service on
  `tcp://*:8023 + fnv1a(name) % (65535-8023)` (ports per `../ports.py`). The Mac SUB's
  connect-list effectively selects the service set.
- **Return (Mac→device, drive mode):** `offloadd` SUBs to the Mac's PUBs, freshness-gates on
  the **device** clock, re-stamps the cereal header `logMonoTime` to device-now, and injects
  into the **local** msgq. No clock sync anywhere (§1.4).
- **Echo prevention:** the Mac must **not** subscribe to `modelV2` (etc.) on the device ZMQ
  while it is republishing the same services back — that is an infinite echo. Keep the return
  PUB on the Mac and the forward SUB list disjoint from `RETURN_SERVICES`.

## offloadd operational rules (read before enabling)

1. **Echo prevention (above).** The Mac's forward SUB list must exclude the services it
   republishes.
2. **Do NOT live-stream during offload.** `livestream*` encode data shares the encode path and
   the USB link budget; it perturbs the frame timing gates. Kill/disable livestream for the
   whole offload session.
3. **Freshness is `now_device - timestamp_sof`** (SOF, not EOF), threshold `OFFLOAD_STALE_MS`
   (default 300 ms). Stale ⇒ drop, never republish.
4. **Absence is the signal.** On a `OFFLOAD_GAP_MS` (default 400 ms) gap with no valid
   republish, `offloadd` publishes *nothing* — `controlsd`'s existing SubMaster alive/valid
   checks then soft-disable. Do not add a synthetic "stale" message.
5. **OffloadMode gates everything.** `offloadd` is inert unless the device param `OffloadMode`
   ∈ {`shadow`,`drive`}. The manager entries are gated the same way; with the param absent
   (default) nothing offload-related starts. Set it only when you mean it.
6. **Stats go to stdout only** (JSONL). Bench mode may append `/tmp/offloadd_bench.jsonl`.
   **No `/data` writes** — `/data` is ~90% full on the target device.

## Tether bring-up (USER-GATED)

Use the **AUX** USB-C port (USB 3.1 Gen2, UDC `a600000.dwc3`), never OBD-C/panda.

```bash
bash openpilot/offload/device/bench/01_enable_tether.sh comma@comma-b203ed6e.local   # AdbEnabled=1 + start adbd
bash openpilot/offload/device/bench/02_mac_side.sh    comma@comma-b203ed6e.local       # find link, ping, iperf3
bash openpilot/offload/device/bench/03_live_smoke.sh  169.254.x.y 30                    # framebridge smoke (needs WS-B)
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
| encoder GOP gap | no return msg for a frame id | nothing to publish for it | handled upstream (WS-B) |
| ZMQ HWM drops | SUB silently drops | counted via recv gaps | gap watchdog covers the symptom |
| echo loop | Mac SUB sees its own republish | MUST NOT HAPPEN | operational rule 1 |

## User-gated steps (require an explicit human action)

- Tether: `01_enable_tether.sh` (param `AdbEnabled=1` + `systemctl start adbd`) and
  `04_disable_tether.sh`.
- Enabling offload: writing param `OffloadMode` ∈ {`shadow`,`drive`} (absent by default).
- Any deploy/reboot of the device tree (PM/owner; not WS-D).
