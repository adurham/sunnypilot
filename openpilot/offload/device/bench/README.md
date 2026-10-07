# WS-D bench — device tether bring-up, smoke, teardown, and the P3 checklist

All scripts here are **user-executed**. They change device state and are NOT run by any
workstream or CI. Read each script's header comment before running it. Everything lives
under `openpilot/offload/device/bench/`.

| Script | Side | Purpose | Gated? |
|---|---|---|---|
| `01_enable_tether.sh` | Mac → device | set `AdbEnabled=1`, start `adbd` (NCM+FunctionFS gadget on UDC `a600000.dwc3`) | **YES** |
| `02_mac_side.sh` | Mac ↔ device | find the tether link, show both link-local IPs, ping, iperf3 | **YES** (starts a one-shot iperf3 server) |
| `03_live_smoke.sh` | Mac | run `openpilot/offload/mac/framebridge.py` against the device; print frame counts + decode p50/p99 | no (read-only on device) |
| `04_disable_tether.sh` | Mac → device | reverse of 01: stop `adbd`, `AdbEnabled=0` | **YES** |

Order: `01 → 02 → 03 → … → 04`. Use the AUX USB-C port (USB 3.1 Gen2), never OBD-C.

Device facts this bench assumes (re-verify with `check_device.sh`):
AGNOS Ubuntu 24.04 / kernel 4.9.103; configfs + F_FS + NCM built-in, **F_UVC not built**,
no loadable modules; device python `/usr/local/venv/bin/python3` (3.12, capnp+pyzmq+zstandard);
cereal bridge binary `/data/openpilot/openpilot/cereal/messaging/bridge`; `iperf3` present;
`/data` ~90% full.

---

## P3 checklist

Run order matters. Do the `/data` check before anything that could write; the tether and
offload path write nothing to `/data` but confirming headroom first is cheap insurance.

### Before you start
- [ ] **`/data` headroom**: `ssh <host> 'df -h /data'` → aim for ≥ 10% free (it is ~90% full). Offload does not
      write `/data`, but loggerd/uploaders still do; do not start with a full disk.
- [ ] **AC power both ends** + `caffeinate -dimsu` on the Mac (a 30-min soak must not sleep).
- [ ] **Cable in AUX USB-C** (UDC `a600000.dwc3`), not OBD-C/panda.
- [ ] Confirm **no dashboard/livestream during offload** (see README operational rules).

### P3-1 — Tether up (01) + link
- [ ] `AdbEnabled == 1`, `systemctl is-active adbd == active`, `usb0` exists on device.
- [ ] Mac shows a new interface with **`169.254.x.x/16` autoconf** (not DHCP).
- [ ] **`ping` to the device link-local < 1 ms** RTT.
- [ ] **`iperf3`** device-server / Mac-client throughput comfortably above the video rate.

### P3-2 — Bridge + frame path (03)
- [ ] `narrowRoadEncodeData` sustained throughput **~10–12 Mbps** for the run.
- [ ] Frame counts ≈ duration × 20 fps per camera; no synthetic/dup `frame_id` (gate G1).
- [ ] **join latency ≤ 2 s** (encodeId↔frameId join window, WS-B FrameTable `join_window`).
- [ ] Decode latency **p50 ≤ 3 ms / p99.9 ≤ 10 ms** (gate G7).

### P3-3 — 30-minute soak
- [ ] **Zero crashes** on the Mac pipeline and the device bridge over 30 min.
- [ ] **Flat memory** on both ends (no monotonic RSS growth).
- [ ] **encode → Mac-arrival p50 15–25 ms**.
- [ ] **Mac pipeline p50 30–60 ms** (SOF → modelV2 published on the Mac).
- [ ] **Added-vs-device-baseline p50 ≤ 60 ms** (Mac path cost over the device's own modeld).

### P3-4 — Teardown (04)
- [ ] `adbd` stopped; `AdbEnabled` cleared; `usb0` gone. `/data` unchanged (no writes).

---

## Expected readings (quick reference)

| Reading | Expected | Notes |
|---|---|---|
| link-local | `169.254.x.x/16`, autoconf | both ends |
| tether ping RTT | **< 1 ms** | direct link |
| narrowRoadEncodeData | **~10–12 Mbps** sustained | 1928x1208 HEVC (fullHEVC, 10 Mbps cap) |
| join window | **≤ 2 s** | FrameTable |
| decode | p50 ≤ 3 / p99.9 ≤ 10 ms | gate G7 |
| encode→Mac-arrival | **p50 15–25 ms** | 30-min soak |
| Mac pipeline | **p50 30–60 ms** | SOF→modelV2 |
| added vs baseline | **≤ 60 ms p50** | Mac overhead |
| /data | ≥ 10% free | check first |

## Decision table (INTERFACES §6 — for reference)

| Trigger | Action |
|---|---|
| msgq/VisionIPC broken on macOS | PASSED 2026-10-06 — fallback retired |
| Metal compile fails (unsupported op) | report op+trace; try CPU pkl; flag OPEN-1 |
| Device bridge/pyzmq missing | P2 presence work proceeds; flag OPEN-2; deploy gate = user |
| fork-model p99.9 > 40 ms | stop model work; PM decides stock-model MVP |
| submodule pointer change in diff | hard fail; revert |
| other sessions' GPU load perturbs bench | record loadavg; rerun; mark "provisional" |

If a bench run is perturbed by other GPU load, check `uptime`, record loadavg, rerun once, and
mark the result **provisional** if still noisy.
