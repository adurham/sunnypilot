# FORK.md — adurham/sunnypilot

This fork of [sunnypilot/sunnypilot](https://github.com/sunnypilot/sunnypilot) is the
**deployment path for the owner's 2022 Hyundai Elantra N** (comma 3X, dongle b203ed6e).
It carries a small set of fork-local changes on top of upstream and is kept in sync
automatically.

**Convention (same as the hermes-agent fork):** every divergence from upstream gets a
dated entry below, in the SAME commit as the code change. Entries state symptom/why,
files, verification, and whether the change is fork-local or an upstream-PR candidate.
Newest entries at the top. This file is the single source of truth for what is different
from upstream and why — it is what keeps syncs debuggable and prevents silent loss.

---

## Deploy architecture (read this first)

- **`main`** = source of truth. Upstream `master` + the fork commits below. NOT runnable
  on the device (no compiled artifacts / models / capnp-gen).
- **`dev`** = the DEPLOYABLE prebuilt branch. Built automatically by
  `.github/workflows/sync-upstream.yaml`: upstream's prebuilt tree (`dev`, contains the
  `prebuilt` marker, all compiled artifacts + models) overlaid with the fork's source
  diff, with capnp C++ bindings regenerated when the schema changed.
- **The comma tracks `dev`**: device `origin` = this fork, `UpdaterTargetBranch=dev`.
  Flow: push `main` → CI rebuilds `dev` → device updater fetches `dev` → activates on
  the next device restart (reboot or car power cycle).
- Deploy is therefore triggered by **pushing `main`**. Do not push `main` unless a deploy
  is intended (CI rebuilds `dev`, and the device will pick it up). Never point the device
  at `main` — it would attempt an on-device source build (no `prebuilt` marker there).
- Rollback ref: `deploy-elantra-n` (mirror ref), plus the device's own git history.

---

## Fork divergence table

| # | Change | Kind | Upstream status |
|---|--------|------|-----------------|
| 1 | Road type classification (mapd → LiveMapDataSP.roadType) | feature | fork-local; PR candidate |
| 2 | HUD road-type shield indicator (onroad UI) | feature | fork-local; PR candidate |
| 3 | Elantra N vehicle specs override (mass, steerRatio) | car-specific | intentionally fork-local |
| 4 | Automated upstream sync + prebuilt rebuild workflow | tooling | intentionally fork-local |
| 5 | Prebuilt CI commit-identity fix | fix | fork-local (CI-only) |
| 6 | Cargo allowance preserved in the specs override | fix | fork-local |
| 7 | Audio boot-race survival (micd/soundd lifecycle) | fix | fork-local; upstream-PR candidate |
| 8 | locationd transient-invalidity hysteresis | fix | fork-local; upstream-PR candidate (cf. openpilot #38929) |
| 9 | Road-type classifier speed-threshold boundaries | fix | fork-local (part of the feature) |
| 10 | Hyundai comma-pedal (gas interceptor) longitudinal, non-SCC — INERT until bench-validated | feature (safety C + car) | fork-local; branch `hyundai-pedal-long`, NOT on main |
| 11 | Hyundai pedal: REMAPPED CAN IDs (0x700/0x701, owner's custom pedal firmware) alongside standard 0x200/0x201 | feature (safety C + car) | fork-local; amends #10, same branch |
| 12 | Hyundai pedal: low-speed TAKE CONTROL alert gated on longitudinal engagement | fix (selfdrived car events) | fork-local; amends #10 |
| 13 | Hyundai FCA11 (0x38D) brake-injection TEST safety mode — **TEST-GATED, NOT FOR ROAD USE**, parked/standstill only. **v2:** test TX list = CLU11 cancel-only + `0x38D`; camera LKAS11/LFAHDA keep forwarding | research (safety C only) | fork-local; patch `0002` (v2), opendbc branch `fca11-brake-test-v2`, NOT on main's default path |
| 14 | Hyundai pedal SCE-latch fix: fault-clearing zero frame + escalation to accFaulted, and bursted CLU11 cancel in pedal mode — **offline-tested only, not road-validated** | fix (car layer, Python only) | fork-local; amends #10; patch `0003`, opendbc branch `hyundai-pedal-sce-fix` |
| 15 | Hyundai pedal tune from route 123: gas cap 0.15 → 0.30 + matching **panda raw ceiling** (A 1110 / B 559), PEDAL_SCALE 0.36, refit hold offset, 0.75/s rise slew — **offline-tested only** | tune + safety C | fork-local; amends #10; patch `0004`, opendbc branch `hyundai-pedal-tune` |
| 16 | Hyundai pedal: **zero CLU11 TX** in pedal mode, factory-cruise **MAIN lockout**, pause/resume re-engage on a deliberate press, **no auto-resume** — supersedes #14's bursted cancel — **offline-tested only** | fix + feature (safety C + car) | fork-local; amends #10; patch `0005`, opendbc branch `hyundai-pedal-buttons` |
| 17 | Adaptive follow distance (road type × speed + throttle-only closing margin), param `AdaptiveFollowDistance` default OFF; Elantra N `wheelSpeedFactor` 1.0125 (GPS-measured) — **offline-tested only** | feature + car-specific | fork-local; branch `planner-tune`, integrated in `integration-2` |
| 18 | Hyundai pedal buttons v2 (route 128): pause/resume = openpilot long on/off at **any speed incl. standstill**, gas no longer blocks the press, panda grant independent of `controls_allowed`; audible SET/RES-below-25-mph and factory-MAIN-lockout alerts; SET at current speed; launch limit 12 % at standstill → full cap by 25 mph; hold-offset table refit — **offline-tested only** | fix + feature (safety C + car + selfdrived) | fork-local; amends #16; patch `0006`, opendbc branch `pedal-buttons-v2` |
| 19 | Hyundai FCA11 **ROLLING** brake test, TEST-ONLY bit 128 on top of bit 64: capped 0.10 g decel only in a 5-35 km/h D window, latching cut, 1.2 s cap, auto camera hand-back — **TEST-GATED, NOT FOR ROAD USE, inert unless armed by the runner** | research (safety C only) | fork-local; amends #13; patch `0007`, opendbc branch `fca11-rolling` |
| 20 | Hyundai pedal gas cap 0.30 → **0.35** (owner-approved) + panda raw ceiling A 1218 / B 612 — **offline-tested only** | tune + safety C | fork-local; amends #15; patch `0008` |
| 21 | Hyundai pedal **timed factory-cruise CANCEL** backstop: if factory cruise goes ACTIVE (EMS16 `CRUISE_LAMP_S`) under engaged openpilot long, panda itself sends CLU11 button-4 frames right after cluster CLU11 frames (4×3 max, 15 ms rate cap, 2 s give-up); CLU11 stays out of the USB TX list; openpilot long is still dropped by the MAIN lockout and alerts **"Cruise Fault"** if factory cruise is still on after 2.5 s. Should rarely fire. **ECM acceptance unproven until the road test** — **offline-tested only** | safety net (safety C + car) | fork-local; amends #16/#18; patch `0009`, opendbc branch `timed-cancel` (fbc82f9c) → `integration-3` |
| 22 | Hyundai pedal **speed-scheduled gain** (0.10 → 0.36 over 0-22 m/s) + hold table in command units, **jerk-based** rise limit (2.0 m/s³ × gain) instead of the fixed 0.75/s — identical to the old law at ≥ 22 m/s; cap 0.35, launch ceiling and panda ceiling unchanged (**no firmware change**) — **offline-tested only** | tune (car layer, Python only) | fork-local; amends #15/#20; patch `0011`, opendbc branch `pedal-tune-12ef` |
| 23 | SCC fixes (fork subclasses, upstream SCC files untouched): vision output floored at MIN_V, no ENTERING below 9 m/s, already-turning → TURNING; map path ignored when > 25 m away or mapd unmatched, `scc_map_diag` cloudlog diagnostics; **throttle-only guard** releases an SCC coast with no lateral corroboration after 2 s. Adaptive follow highway factor 1.2 → 1.1 — **offline-tested only** | fix + safety net (planner) | fork-local (`fork/scc.py`); SCC bug fixes are upstream-PR candidates |
| 24 | **Automatic read-only ESC UDS read** at every ignition (card, inside openpilot's own fingerprint window, panda still in ELM327; no firmware change): 70 identification/variant-coding DIDs once per ESC firmware, DTCs (0x19 02) once per day, only in Park at 0 km/h, at most once per ignition → `/data/esc-uds/*.json` + `esc_uds_read` rlog event. Plus route data: `scc_map_path` event logs the MapTargetVelocities list (was only in /dev/shm) | diagnostics + route data | fork-local (`fork/esc_diag.py`, `fork/scc.py`) |
| 25 | Hyundai FCA11 rolling test **Warn gating**: in the rolling mode (bit 128, armed only by the runner) a `CF_VSM_Warn` > 0 FCA11 frame is policed like actuation — same 5-35 km/h D window, latched cut and fresh-input rules, and it shares the 1.2 s episode clock. **Inert unless bit 128 is armed**; parked mode and normal driving byte-for-byte unchanged — **TEST-GATED, offline-tested only** | safety C (tightening) | fork-local; amends #19; patch `0012`, opendbc branch `fca11-warn-0012` |
| 26 | ESC read **standstill fix** (amends #24): "stationary" = Park + fewer than 2 wheels above 12 LSB (0.375 km/h, opendbc's Hyundai STANDSTILL_THRESHOLD) and no wheel above 96 LSB, instead of all wheels exactly 0 (parked single-wheel noise skipped the read on the car). Same definition for the pre-check and every per-frame check; a pre-check skip no longer uses up the ignition (≤ 3 tries per ignition, ≤ 3 s settle wait in Park) | diagnostics | fork-local (`fork/esc_diag.py`) |
| 27 | ESC **0x27 seed probe + no-op 0x2E write** (adds to #24/#26, same fingerprint window): enabled only by `/data/esc-probe-0027/state.json` `{"probe_enabled": true}`, at most once per ignition — read 0x0103, **enter extended session (10 03 — the vendor's own write flow, decoded GIT VariantCodingTable)**, request the 0x27 seed (0x27 sub 0x01 ONLY; sendKey is never sent), then write the just-read bytes straight back to 0x0103 with 0x2E (no-op by construction: the allowed payload MUST equal the step-1 read-back), and re-read. Answers "does the write path need security access at all". **Phase 3 update:** a `phase` key selects 1 (pre-write seed), 2 (vendor-exact, no pre-write seed) or **3 = the discriminating battery** (6-address `27 01` seed sweep across ESC/CLU/TCU/EPS/CAM/CR, default- AND extended-session no-op `2E`, `29 01` auth + `31 01` routine probes, and **exactly ONE** `27 02` identity-key attempt mechanically pinned to the step-2 ESC seed). **Phase 4 update:** `phase: 4` (+ `key_mode` = identity2 (default) / identity4 / identity8 / algo / hex) completes the unlock — extended-session seed -> **ONE** `27 02` sendKey with the RESOLVED candidate key (2/4-byte single frame, 8-byte ISO-TP multi-frame; single-attempt + candidate-pinned in `guard_frame`), and ONLY on `67 02` the no-op `2E` + re-read. Park + standstill only, mux restored in `finally` → `/data/esc-probe-0027/*.json` + `esc_probe_0027` rlog event | diagnostics | fork-local (`fork/esc_probe_0027.py`; **algo-mode update:** `key_mode: "algo"` + optional `algo` = `"27100".."27400"` selects a recovered G-scan CalKeyAlgorithm_* from `fork/esc_probe_seedkey.py`, candidate recorded as `algo`/`key_bytes`, UNVERIFIED vs hardware) |
| 28 | Hyundai FCA11 rolling test **decel cap 0.10 g → 0.30 g** for the dose-response (scaling) test, **rolling mode only** (bits 64\|128, armed only by the runner); window, 1.2 s clock, latched cut, freshness, camera hand-back, HBA/StopReq block and check_relay unchanged; parked mode keeps 0.10 g; with the bits unset no decel is transmittable — **TEST-GATED, offline-tested only** | safety C (test-mode widening) | fork-local; amends #19/#25; patch `0013`, opendbc branch `fca11-scale-0013` |
| 29 | **Set-speed easing** (personality-dependent): the planner's cruise candidate chases an eased speed that ramps toward a raised set speed / SLA limit / released SCC target (relaxed 0.55→0.33, standard 0.75→0.40, aggressive 1.2→0.8 m/s per s over 10→29 m/s, leashed to 1.5 s of ramp ahead of vEgo), down immediate, never below vEgo, pass-through while not in control, launches from a stop un-eased. **Merge gate:** un-eased (upstream) while the car is ≥ 15 mph below a highway-class speed (roadType highway/interstate, a map / car speed limit ≥ 55 mph, or a ≥ 55 mph map limit ≤ 500 m ahead — the ahead limit overrides urban; capped at the set speed, 3 mph speed hysteresis + 0.5 s ref-boundary debounce), or, with no map / car limit at all, ≥ 25 mph below a ≥ 55 mph set speed. Car-limit (cluster sign) staleness is known and documented (benign: gate only suppresses easing). Lead (MPC) and e2e candidates untouched — **offline-tested + closed-loop sim only** | feel (planner) | fork-local (`fork/setspeed_ease.py`) |
| 30 | **Cruise preferences are never deleted when openpilot longitudinal is transiently unavailable**: ExperimentalMode, DynamicExperimentalControl, CustomAccIncrementsEnabled, SmartCruiseControlVision, SmartCruiseControlMap are kept in Params no matter what the current ignition's CarParams say (the pedal interceptor being disarmed used to look exactly like a stock-ACC car and the upstream cleanup paths deleted them permanently). The UI still disables/greys the toggles and shows the STORED value while greyed, and the runtime consumers stay gated on longitudinal being active (verified). Enabling **lateral maneuver mode or joystick debug mode no longer overwrites the stored ExperimentalMode** — experimental mode is suppressed at runtime (`fork/cruise_prefs.experimental_active`) while either is on, so the preference is untouched by those developer toggles — **offline-tested only; no firmware change** | fix (car + UI + selfdrived) | fork-local (`fork/cruise_prefs.py`) |
| 31 | Hyundai **FCA11 LONGITUDINAL braking** — production (toggle-gated) FCA11 (0x38D) decel through the ESC **on top of the comma pedal**, openpilot ENGAGED. Param `HyundaiFca11Brake` **default OFF**; when ON (and the pedal armed) the card sets panda bit 256 `FCA11_LONG` and `CP_SP.fca11Brake`. The car layer mirrors the freshest bus-2 camera 0x38D byte-for-byte and overrides ONLY the brake fields, hard-zeroing gas while braking; panda independently polices: `controls_allowed` AND `heartbeat_engaged`, gear D, no pedal, every wheel > 9 km/h (no ceiling), inputs fresh ≤ 100 ms, `CR_VSM_DecCmd` ≤ 30 (0.30 g), growth ≤ 0.04 g/camera period, a 2.5 s actuation budget + 3 s cooldown, latched cut on driver brake/gas/gear≠D/pedal fault, and same-frame camera hand-back. No auto-resume after a cut. **Toggle OFF = byte-for-byte today's pedal-only behavior.** NOT FOR ROAD USE until reviewed + the drive-mode gate (open risk R1) — **offline-tested only (safety + car unit + firmware)** | feature (safety C + car + planner limits + selfdrived alert + UI) | fork-local; patch `0014`, opendbc branch `fca11-long` (08adce5c), firmware `67c1f99e`; NOT on main |
| 32 | ESC 0x27 probe **phase 5 — READ-ONLY capability battery** (amends #27, same fingerprint window): `state.json` `{"phase": 5}` runs ONE parked, read-only ignition — a FIXED frame list (0xF100 canary, 0x0103 read, `10 03`, three `27 01` seed samples, ten 2-byte sub-probes `27 03/05/07/09/0B/0D/0F/11/41/61`, seven bare 1-byte service probes `23/29/31/34/35/36/37`, 0x0103 re-read, and two extra-address `10 03` peeks at 0x770/0x7A0) that records every response/NRC/timeout/latency and adds seed-stability + fp_canary + value_start/value_end summary fields. **NO `27 02`, NO `2E`, no multi-frame TX** — the guards admit only the exact frames above | diagnostics | fork-local (`fork/esc_probe_0027.py`); phases 1-4 byte-identical |

---

## Entries

### keep-cruise-prefs: never delete the owner's cruise settings when openpilot longitudinal is transiently unavailable — 2026-10-05 (offline-tested; no firmware change)

> **Driver notes:** your cruise settings stay put. DEC (Dynamic Experimental Control), the Custom ACC
> increment switch, Smart Cruise Control vision/map and Experimental mode are no longer wiped just
> because openpilot longitudinal happens to be off for one ignition (e.g. the pedal interceptor
> disarmed for a parked test). While a feature is unavailable its toggle is greyed out and still shows
> what you saved, and it works again the moment longitudinal is back — you never have to re-set it.

- **Why:** on 2026-10-05 the interceptor was disarmed (``HyundaiGasInterceptor`` off) for a parked test.
  With ``CP.openpilotLongitudinalControl`` False and ``CP_SP.pcmCruiseSpeed`` True the car looked exactly
  like a plain stock-ACC car, and upstream's cleanup paths DELETED the longitudinal-dependent prefs. The
  condition was transient (interceptor re-armed the next ignition) but the deletion was permanent: DEC
  and the custom-increment **enable switch** stayed gone — the stored 5 mph increment VALUES survived, so
  the set-speed buttons silently stepped 1 mph — and SCC vision/map + Experimental mode had to be
  re-toggled by hand.
- **What:** ``openpilot/sunnypilot/fork/cruise_prefs.py``: ``preserved(key)`` (the five keys),
  ``remove_unless_preserved(params, key)``, ``sync_toggle(params, key, item)``, and the runtime gates
  ``maneuver_mode_active(params)`` / ``experimental_active(params, openpilot_longitudinal)``. Six
  upstream-style cleanup sites switch from ``params.remove(key)`` to the guard (one line each):
  ``selfdrive/car/interfaces.py`` ``_cleanup_unsupported_params`` (DEC/CustomAcc/SCC-V/SCC-M),
  ``selfdrive/ui/sunnypilot/ui_state.py`` ``_enforce_constraints`` (ExperimentalMode + DEC, and
  CustomAcc/SCC-V/SCC-M), ``selfdrive/ui/sunnypilot/layouts/settings/cruise.py`` (disables + shows the
  stored value; no ``set_state(False)`` clobber), ``selfdrive/selfdrived/selfdrived.py``,
  ``selfdrive/ui/layouts/settings/toggles.py`` and ``selfdrive/ui/mici/layouts/settings/toggles.py``
  (ExperimentalMode). Only the five keys are exempt — every other cleanup (ICBM,
  AlphaLongitudinalEnabled, NNLC, LateralJerk, angle-steering, BSM) is unchanged.
- **Developer toggles no longer wipe ExperimentalMode:** enabling lateral maneuver mode previously
  wrote ``put_bool("ExperimentalMode", False)`` in both developer layouts (Qt ``selfdrive/ui/layouts/
  settings/developer.py`` and mici ``selfdrive/ui/mici/layouts/settings/developer.py``), permanently
  clobbering one of the five preserved keys from a path unrelated to longitudinal availability. Those
  two writes are gone; lateral maneuver mode and joystick debug mode now force experimental OFF at
  runtime instead (``experimental_active`` = stored param + longitudinal available + no maneuver mode),
  so toggling either on and off leaves the stored preference exactly as the owner set it. The mici
  ExperimentalMode button is greyed (matching the Qt path) rather than hidden, and still shows the
  stored value.
- **Consumers stay gated (why keeping the value is safe while unavailable):** DEC/experimental mode act
  only when ``selfdriveState.experimentalMode`` is set, which card/selfdrived compute as
  ``param and CP.openpilotLongitudinalControl``; SCC-V/SCC-M targets are only arbitrated when
  ``carControl.longActive`` is true, which controlsd computes as
  ``... and (CP.openpilotLongitudinalControl or not CP_SP.pcmCruiseSpeed)``; the custom-increment apply
  path (``VCruiseHelperSP.update_v_cruise_delta``) is only reached from the non-PCM-cruise branch of
  ``VCruiseHelper.update_v_cruise``. Each gate is pinned by a test
  (``fork/tests/test_cruise_prefs.py::TestUpstreamGatePins``) so an upstream rename fails loudly instead of
  silently un-gating a consumer.
- **Verification:** fork suite **228 passed** (19 new in ``fork/tests/test_cruise_prefs.py`` + 27 subtests);
  the touched upstream suites (``sunnypilot/selfdrive/car/tests``, ``dec/tests``,
  ``smart_cruise_control/tests``, ``sunnylink/tests/test_settings_changes.py``) **134 passed, 31 skipped**;
  ruff clean. Mutation: ``car-features/keep-cruise-prefs-mutation.py`` **16/16 killed** (predicate on/off,
  key-list drops, wrapper always/never/reversed delegating, ``sync_toggle`` clobber, and each of the six
  call sites reverted to ``params.remove``). ``test_cruise_speed.py::TestCruiseSpeed`` fails 12/18 here —
  pre-existing on the fork (the setspeed-ease feature vs the Maneuver plant), identical on the unmodified
  worktree, not caused by this change.
- **Merge note:** intentionally fork-local (``fork/`` only; the six edits are one-line each). No firmware
  build. The underlying behaviour is a reasonable upstream-PR candidate (transient capability loss should
  not destroy persisted settings), but as a fork guard it needs no capnp/schema change.

### fca11-long: production FCA11 longitudinal braking (toggle-gated, default OFF) — 2026-10-06 (offline-tested; one firmware build; NOT road-run)

> **Driver notes (only when `HyundaiFca11Brake` is turned ON).** sunnypilot can add light braking (up to ~0.30 g,
> comfortable ACC strength, about a third of a hard stop) on top of the comma pedal by sending the car's own FCA11
> forward-collision message, so it no longer only lifts off for a lead — it can actually slow for one. The dash shows
> the car's collision-braking notification while it brakes (no chime at these levels). It hands the stop back to the
> driver below 12 km/h and at every standstill: **you still brake for the stop.** It only adds braking when openpilot
> long is engaged (the pedal on) and never fights you — your brake or gas cancels it immediately, and after any cancel
> it stays off until you deliberately pause/resume. **Default OFF; keep it off until reviewed for your car.**

- **Why:** the on-car scaling run `roll-20261005T232357Z` proved the FCA11 variant-B shape (Prefill=1, Warn=3,
> `CR_VSM_DecCmd`=g×100, `FCA_CmdAct`=1, `DecCmdAct`=0) brakes this car through the ESC (0.74 / 1.28 / 2.03 m/s² for
> 0.10 / 0.20 / 0.30 g, ≈ ⅔ linear; nothing below ~8-10 km/h). This turns that research path into a production,
> toggle-gated actuator while keeping the pedal-only car byte-for-byte unchanged when the toggle is OFF.
- **What (opendbc branch `fca11-long`, patched by `0014-hyundai-fca11-long-brake.patch`:**
  - `opendbc/sunnypilot/car/hyundai/fca11_long.py`: `Fca11LongBrake` (window/onset/gain/rate/mirror) +
    `build_fca11_frame` (byte-exact camera mirror: overrides ONLY Prefill, Warn, `CR_VSM_DecCmd`, `Fca_CmdAct`,
    `CF_VSM_DecCmdAct`, alive+1, CRC; every other byte copied, incl. the always-1 undefined byte4 bit7) +
    `camera_requesting` (hand-back detect).
  - `gas_interceptor.py`: `create_gas_command` calls the brake first and **hard-zeroes gas while braking**.
  - `carstate_ext.py` / `carstate.py`: CS inputs (`fca11_cam_frame` from a raw-frame-capturing bus-2 `CANParser`,
    `fca11_now_nanos`, `interceptor_state`); `CANParser.capture_addrs/captured` (opt-in, zero cost when off).
  - `interface.py` (Hyundai): `get_pid_accel_limits` override — brake authority to −2.0 m/s² above 12 km/h, coast-only
    below; `interfaces.py`: `_initialize_hyundai_gas_interceptor` arms `CP_SP.fca11Brake` + safety bit **256** from the
    `HyundaiFca11Brake` param (defensive: any non-`"1"` value, missing/stale key → OFF).
  - `structs.py` + `custom.capnp`: `CarParamsSP.fca11Brake @6`; `params_keys.h`: `HyundaiFca11Brake` BOOL `"0"`.
  - `sunnypilot/selfdrive/car/car_specific.py` + `selfdrived/events.py` + `camera/custom.capnp`: WARNING-only
    `fca11BrakeLowSpeed` hand-over alert below 14 km/h while long is engaged; UI toggle in the Hyundai vehicle settings.
  - panda `safety/modes/hyundai.h` (`hyundai_fca11_long_*`) + `hyundai_common.h` (bit 256): the full gate/window/cap/
    rate/budget/cooldown/cut/camera-hand-back above; `check_relay`/HBA/StopReq/DecCmdAct unchanged; legacy + CAN-FD
    force the bit off.
- **Verification (this pickup, 2026-10-06):** safety suite **4439 passed / 780 skipped / 0 failed** (new
  `TestHyundaiFca11LongSafety` class: **289 passed / 16 skipped**; unchanged counts for the frozen test modes); car
  layer `test_fca11_long.py` **23 passed**, `test_gas_interceptor.py` **135 passed**. Firmware mutations 19/19
  non-equivalent killed + 1 equivalent (argued in the script). Closed-loop sim: see `fca11-long-integration.md` §4.
  MISRA cppcheck **rc 0**. Firmware rebuilt from a clean export **and** the CI-path tree (pristine f95f996f + 0001-0014,
  rehearsal tree matches the branch tree `bae30226`): sha256 **`67c1f99e1f85da9a43d3000efb194e3aa718fdd0a1e68663a4a1caf4132dd190`**
  (113820 B, `DEV-74a0adce-DEBUG`); controls reproduce the deployed `d0f5396c` and `65d3692f`.
- **Gotchas documented in code:** the window's gas-freshness input is **EMS16 (0x260)**, not the pedal sensor; the
  cooldown must not fire from a fresh arm (guard `ts_act_end != 0`); the panda gates **all** 0x38D on
  `controls_allowed`+heartbeat (there is no passive carrier we send — the camera supplies idle frames). `fca11Brake` has
  zero C++ consumers today; the overlay CI regenerates capnp on the schema change.
- **Merge note:** fork-local; NOT for the car until the independent review + the drive-mode gate (open risk R1 in
  `fca11-long-integration.md`) — the feature does not yet enforce "OP long only in Eco/Normal/Sport".

### setspeed-ease: personality-dependent easing toward a raised set speed — 2026-10-05 (offline-tested + closed-loop sim; NOT road-run; no firmware change)

> **Driver notes:** raising the set speed (button, Speed Limit Assist, long-press-down to current speed) or a lead pulling
> away no longer gets the full 1.0-1.2 m/s² (2.0 in experimental) at once. The car eases up to the new speed, and the
> personality (distance button) now sets how fast: **relaxed** gentle (~0.35-0.55 m/s²), **standard** moderate
> (~0.40-0.75), **aggressive** about like before. Lowering the set speed still takes effect immediately. Launches from a
> stop are unchanged. Braking for a lead is unchanged. **Merging onto a highway is NOT eased**: while the car is well
> below a highway-class speed (on-ramp, catching up to highway traffic) it accelerates exactly as before, in every
> personality, and the gentle ramp takes over only for the last ~15 mph.

- **Why:** `car-features/drive-133-134-report.md` §A ("robotic" chase of a raised target; personality-independent
  `A_CRUISE_MAX` / `ACCEL_MAX` step).
- **What:** `openpilot/sunnypilot/fork/setspeed_ease.py` `SetSpeedEase`. One hook in `LongitudinalPlannerSP.update_targets`.
  The returned cruise speed is eased, while `longitudinalPlanSP.vTarget` stays raw. Upstream `longitudinal_planner.py`,
  `get_cruise_accel` and `cruise.py` are untouched. Rules: ramp at `rate(v, personality)`; leash
  `v_ease ≤ vEgo + 1.5·rate`; `min(vEgo, target) ≤ v_ease ≤ target` (down immediate, eased request ∈
  [min(0, upstream), upstream]); pass-through + re-arm at vEgo while not engaged/overriding; launch latch below 3 m/s
  until within 1 m/s of target or plan < 0.3 m/s² for 1 s; capnp personality enum normalized (a raw dict lookup silently
  fell back to standard); an unknown personality value (one cereal adds later) or one that does not cast to int uses
  the **standard** table (never the un-eased one). `setspeed_ease` cloudlog event on each ramp / merge edge.
- **Merge gate (`MergeGate`, `merge_reference`)** — easing is for discretionary speed-ups only. Highway evidence, from
  signals the fork already has (no capnp change): `liveMapDataSP.roadType` highway/interstate (its limit, else 55 mph),
  a valid map limit ≥ 55 mph, `carStateSP.speedLimit` (cluster / camera) ≥ 55 mph, or a valid map limit ≥ 55 mph within
  500 m ahead (on-ramps are usually unnamed OSM links with no limit). 55 mph =
  `road_type_classifier.HIGHWAY_SPEED_THRESHOLD`. Reference = min(highway speed, set speed); merging while
  vEgo < ref − 15 mph (3 mph hysteresis). A ≥ 55 mph limit ≤ 500 m ahead deliberately **overrides** `roadType=urban`
  (approaching a highway from a surface street / frontage road *is* the on-ramp case), so "positive non-highway evidence
  → never a merge" holds for a map / car limit < 55 mph, and for `urban` only when there is no highway-class limit ahead.
  No data at all → fail safe: merging while vEgo < set − 25 mph if the set speed is ≥ 55 mph. The gate keys off the
  SET speed (`v_cruise`), not the SCC/SLA-arbitrated target, so a curve on the ramp does not end the merge; while
  merging the arbitrated target is passed through unchanged. On exit the leash hands over at vEgo + 1.5·rate. A
  `MERGE_REF_HOLD_S` (0.5 s) debounce on the reference boundary keeps a flapping `roadType` / limit-valid from toggling
  the gate tick-to-tick (holding keeps the gate up = suppresses easing = upstream behaviour, the benign direction).
- **`carStateSP.speedLimit` staleness (known, benign):** the cluster / camera sign is a *last-seen* display value read
  live from CAN; `CarStateSP` has only a bare `Float32` (no validity bit, no timestamp), so a stale ≥ 55 mph reading can
  keep the gate active on a normal road (> 15 mph below a highway-class set speed). The effect is benign by
  construction — the gate only *suppresses easing* (restores exact upstream behaviour) and never adds acceleration or
  weakens braking. A freshness guard was investigated and rejected: no age / valid signal exists to build one from, and
  an evidence-veto version would suppress the gate on the real 134 on-ramp merge (its `roadType` reads `urban` at merge
  start), breaking the "never suppress a real merge" rule. Real remediation is upstream (validity / age on
  `CarStateSP.speedLimit`).
- **Drive-mode hook:** personality is the only input; Eco/Normal/Sport → relaxed/standard/aggressive can feed
  `selfdriveState.personality` once the mode signal is decoded.
- **Verification:** fork tests 187 passed on the branch (52 new incl. end-to-end through the real `LongitudinalPlanner` and a
  `TestUpstreamHooks` pin of `get_cruise_accel` / `update_targets` signatures and the planner call sites). Mutation
  58/58 killed (`car-features/setspeed-ease-mutation.py`, incl. 27 merge-gate / hook / enum-cast mutants). Sim: 12f
  on-ramp time to 20 m/s identical to upstream in every personality (was +13.5 s relaxed / +6.2 s standard before the
  gate); +10 mph bumps on normal roads unchanged from the un-gated ease; aggressive+e2e cut-in min gap up to 2.6 m smaller
  than upstream e2e (still >= upstream ACC). Closed-loop pedal sim (v3 DCT plant, shipped 0011 pedal law,
  real planner/LongControl): see `car-features/setspeed-ease-report.md`.

### esc-probe-0027: ESC 0x27 seed probe + no-op 0x2E write of the current 0x0103 — 2026-10-05 (offline-tested; source only, no firmware/opendbc change)

- **Why:** does WRITING the ESC's variant-coding DID 0x0103 need UDS security access (0x27) at all? The "one-minute
  probe" planned in `car-features/esc-probe-0027-report.md`: read 0x0103 as it is, ask for the 0x27 seed, then write
  those exact bytes back. Writing a value onto itself is a no-op for the ESC's configuration; the informative part is
  the reply (refused seed + refused write vs. refused seed + ACCEPTED write vs. an all-zero "already unlocked" seed).
- **Where:** `openpilot/sunnypilot/fork/esc_probe_0027.py`, one call in `openpilot/selfdrive/car/card.py` immediately
  after `run_esc_diag_from_card` (still before `FirmwareQueryDone`, same ELM327 fingerprint window). **No panda
  firmware change, no opendbc change.**
- **Enabled by a file:** `/data/esc-probe-0027/state.json` with `{"probe_enabled": true}`; missing file/flag = inert
  (`skip: not enabled`). `touch /data/esc-probe-0027/DISABLE` forces off. At most ONE probe per ignition
  (`done_ignition` written once the standstill pre-check passes, before the multiplexer).
- **Sequence (vendor-confirmed):** `0x22 0x0103` (extended-session retry once if refused with NRC 0x31/0x7F) →
  `10 03` enter extended session (SKIPPED if the read's retry already entered it; any answer continues, true silence
  aborts before the write) → `0x27 0x01` requestSeed only (**sendKey 0x02 is never sent**) → `0x2E 0x0103 <read-back>`
  (no-op) → `0x22 0x0103` re-read. The read→session→write order mirrors the vendor tool itself (see the follow-up
  bullet below).
- **Safety, mechanically enforced:** `guard_service` allows exactly {0x22, 0x3E, 0x10 sub 0x03, 0x27 sub 0x01, 0x2E DID
  0x0103}; `guard_frame` admits only those single frames + one flow-control frame, and a 0x2E frame is admitted only
  when its 4 data bytes equal the step-1 read-back. The write is attempted ONLY if step 1 returned exactly
  `62 01 03 <4 bytes>`; a refused seed with a NEGATIVE answer still attempts the no-op write (that contrast is the
  point), but a seed that gets NO response aborts without writing. Park + standstill via the imported
  `esc_diag.VehicleGate` / `wheels_moving` (one standstill definition in the tree), re-checked before every TX and
  while waiting. Budget 15 s; multiplexer off in `finally` (SIGTERM too) and verified from pandaStates; every error is
  recorded in the result JSON, never raised into card.
- **Output:** `/data/esc-probe-0027/<UTC>-result.json` (read value, seed response verbatim, write request/response,
  re-read, TX log, duration, aborted/error, mux_restored; 60 files max) + an `esc_probe_0027` cloudlog event (rlog).
- **Tests:** `fork/tests/test_esc_probe_0027.py` (42, fake ESC through the real `run()` path). Mutation proof:
  `car-features/auto-esc-mutation.py` ESC_WT=the probe worktree **47/47 killed** (7 probe mutants: 0x27 sub 0x02
  admitted, 0x2E DID 0x0104 admitted, constant payload instead of the read-back, gate check removed before TX, write
  attempted when the read was refused, **10 03 session step removed**, **write attempted when the session was
  silent**). Demo: `car-features/esc-probe-0027-offline-demo.out`.
- **Vendor-flow alignment (2026-10-05, update 2):** the vendor's own `VariantCodingTable_HY.git.xml` (decoded; the
  CN7N/"Elantra N" ESC block, `<Security SecuritySupported="0" .../>`) exposes the authoritative write as
  `<Backup requestvalue="07D103220103">` (read) → `<Input requestvalue="07D1021003">` (**10 03 extended session**) →
  `<Input requestvalue="07D1072E0103$…$">` (2E 0103 write) — the tool sends **no 0x27**. The probe now mirrors that
  exact order at the write (READ → SESSION → WRITE) while still asking for the 0x27 seed, so its reply remains the
  informative part. Report: `car-features/esc-probe-0027-report.md` "Update 2 — vendor-flow alignment".
- **Phase 2 (2026-10-05, update 3):** phase 1 ran on the car (`esc-software/probe-results/20261005T214537Z-result.json`):
  read OK, `10 03` OK, the `27 01` seed came back POSITIVE with an 8-byte static-looking value (`5AB05AB0` x4), the no-op
  `2E` write was REFUSED (`7F 2E 33` = securityAccessDenied), re-read unchanged, mux restored, no errors. Because that
  refusal cannot separate "the write needs an unlock" from "phase 1's OWN pre-write seed confounded it" from "the
  Hyundai/Kia Security Gateway blocks write services (same NRC 0x33)", the probe now takes a `phase` key in
  `state.json`: **phase 2** repeats the write in the EXACT vendor order with **NO pre-write seed** (READ → SESSION →
  WRITE → re-read) and then sends ONE `27 01` **after** the write as a data point (recorded as `seed_post`; silence or a
  refusal there never aborts). `phase` defaults to 1 (missing/`None`/0 → 1); anything else is inert (`skip: unknown
  phase`). Tests: `fork/tests/test_esc_probe_0027.py` 48 (+6 phase-2); mutation **49/49** killed (2 new: pre-write seed
  sent in phase 2, post-write sample skipped). Report: "Update 3 — phase 2 (vendor-exact, no pre-write seed)".
- **Phase 3 — the battery (2026-10-05, update 4):** phase 2 ran on the car
  (`esc-software/probe-results/20261005T221227Z-result.json`): the vendor-exact order ALSO got `7F 2E 33` on the no-op
  write and the post-write `27 01` again returned a fresh 8-byte seed — so phase 1's own seed was not the confounder.
  Phase 3 (`state.json` `{"phase": 3}`, 30 s budget) discriminates the survivors in ONE ignition: `22 0103` read → a
  **6-address `27 01` seed sweep** (ESC 0x7D1, CLU 0x7C6, TCU 0x7E1, EPS 0x7D4, CAM 0x7C4, CR 0x7B7; 300 ms each —
  uniform seeds from no-security modules hint at a gateway answering on their behalf) → no-op `2E` in the **DEFAULT**
  session → `29 01` (Authentication) → `31 01 0000` (RoutineControl: `7F 31 31` = module session logic answered,
  `33` = same blanket filter) → re-read → `10 03` → no-op `2E` in **EXTENDED** (`33` in both sessions = blanket
  filter) → **exactly ONE** `27 02` sendKey with the IDENTITY key = the exact step-2 ESC seed (ISO-TP multi-frame;
  guard admits it only as the first attempt with the pinned key — any other key/second attempt raises) → only on
  `67 02`, a final `10 03` + no-op `2E` + re-read. Per-frame receive latencies are recorded (`frames_t_ms[]`) to
  expose gateway-vs-module timing. Tests: 59 (+11 phase-3); mutation **53/53** killed (4 new). Report:
  "Update 4 — phase 3 battery".
- **Phase 4 — sendKey (2026-10-05, update 5):** phase 3 ran on the car
  (`esc-software/probe-results/20261005T232115Z-result.json`) and in the DEFAULT session EVERY probe was refused —
  `27 01` -> `7F 27 7F`, `29 01` -> `7F 29 7F`, `31 01 0000` -> `7F 31 33`, no-op `2E` -> `7F 2E 33` in BOTH default and
  extended — so the seed is **session-gated** (it only answers in extended) and phase 3 never reached the key.
  Phase 4 (`state.json` `{"phase": 4, "key_mode": ...}`, 30 s budget) completes the unlock: `22 0103` read ->
  `10 03` extended session (**REQUIRED positive `50 03`**; silence or negative -> abort) -> `27 01` seed (**REQUIRED
  positive `67 01` with >=2 bytes**; else abort) -> ~500 ms vendor delay -> resolve the candidate key (identity2 =
  seed[:2], identity4 = seed[:4], identity8 = the seed, `algo` = `fork/esc_probe_seedkey.key_for(seed, state["algo"])`
  (missing/failing module or unknown algo -> recorded abort; a zero-byte seed the vendor path bails on -> recorded
  abort, no frame), `hex` = `state["key_hex"]`; ONLY lengths 2/4/8) -> **ONE** `27 02` sendKey (2/4-byte:
  single frame `04/06 27 02 K...`; 8-byte: ISO-TP `10 0A 27 02 K1..K4` + `21 K5..K8`) -> only on `67 02` the no-op `2E`
  + re-read. `key_mode` defaults to `identity2`; a malformed one is inert (`skip: unknown key_mode`). Mechanically
  single-attempt and candidate-pinned: `guard_frame(..., key=...)` admits a `27 02` only in phase 4, only as the first
  key (`key_attempts == 0`), and only when its bytes equal the resolved candidate (re-checked at the single TX site);
  phase 3's own identity-key path is untouched. Tests: 82 (+23 phase-4); mutation **56/56** killed (3 new). Report:
  "Update 5 — phase 4 (sendKey)".
- **Algo mode — recovered G-scan key algorithms (2026-10-05, update 6):** the phase-4 `algo` mode is now live with
  selectable candidates. New `openpilot/sunnypilot/fork/esc_probe_seedkey.py` is a byte-identical port of the recovered
  G-scan2 (GIT) `CalKeyAlgorithm_*` functions (static disassembly of `Gs2_SpecialFunc.exe`): `bitsum` (8-bit reversal),
  the 0xC503 family (`cal_27100` LFSR16 = the vendor-exact default, `cal_26300` mod, `cal_26400` XOR), `cal_26700`
  (LFSR32 0x3BCC14C8), `cal_26800` (table-XOR), `cal_27400` (mul/add), and `key_for(seed, algo)` (`None` -> `27100`;
  unknown algo -> `ValueError`) + `candidates(seed)`. The probe reads an optional `algo` string from `state.json` and
  records the selector (`algo`) and the computed candidate (`key_bytes`) in the result JSON + summary/cloudlog BEFORE the
  `27 02`. **Every output is UNVERIFIED against hardware** (no ground-truth seed->key pair exists in the image; a wrong
  key is expected to draw NRC 0x35). `python -m openpilot.sunnypilot.fork.esc_probe_seedkey` self-test passes. Tests:
  86 (+4 algo); mutation **58/58** killed (2 new). Report: "Update 6 — algo mode wired".
- **8-byte key constructions — algo8/algo8p/repeat8/hex8 (2026-10-05, update 7):** the car proved `27 02` wants an
  8-byte key (a 4-byte key drew `7F 27 13` incorrect-length; the 8-byte seed-as-key drew `7F 27 35` invalidKey), so
  phase-4 gains five modes that resolve to exactly 8 wire bytes: `algo8` = the algo key repeated to fill (2-byte -> x4,
  4-byte -> x2), `algo8w` = the algo key's first two bytes repeated to fill (`[lo,hi]x4` — the seed's own wire shape),
  `algo8p` = the algo key + zero padding, `repeat8` = `seed[:2]*4`, `hex8` = exactly 8 bytes of
  `state["key_hex"]` (any other length aborts). Same `guard_frame` single-attempt + candidate-pin; `algo8`/`algo8w`/`algo8p`
  share `algo`'s import/zero-bail abort semantics. Tests: 95 (+9); mutation **60/60** killed (anchor retargeted only). Report:
  "Update 7 — 8-byte key constructions".
- **Phase 5 — READ-ONLY capability battery (2026-10-06, update 8):** phase 4 iterates key VALUES; phase 5 maps what the
  ESC will even TALK ABOUT, parked, without ever reaching a key or a write. `state.json` `{"phase": 5}` (40 s budget)
  runs a FIXED, read-only frame list in one ignition and records every frame's request/response/NRC/timeout/latency
  (never aborting on a refusal): `22 F1 00` (the 0xF100 functional-read canary) -> `22 01 03` (value_start) -> `10 03`
  -> `27 01` (seed1) -> `27 01` (seed2) -> the ten 2-byte sub-probes `27 03/05/07/09/0B/0D/0F/11/41/61` -> the seven bare
  1-byte service probes `23/29/31/34/35/36/37` -> `27 01` (seed3) -> `22 01 03` (value_end) -> a single `10 03` to
  request addr 0x770 (listen 0x778) and to 0x7A0 (listen 0x7A8). Summary adds `seed1`/`seed2`/`seed3` hex,
  `seed_stable_12`, `seed_stable_all`, `fp_canary_hex`, `value_start`, `value_end`. Guards (raised BEFORE any frame is
  built): only the exact frames above; `27` only sub `01` + the exact ten sub-probes (NO `27 02` EVER); `10` only sub
  `03`, extra request addresses ONLY {0x770, 0x7A0} and ONLY for the `10 03` frame; `22` only DIDs {F100, 0103};
  services 23/29/31/34/35/36/37 only as the bare 1-byte frame; **NO `2E`, no `31` sub `01`, no `29` sub `01`, and NO
  multi-frame sends at all** (RX multi-frame is fine); phases 1-4 byte-identical; once per ignition. Tests: **112**
  (+17 phase-5); mutation **64/64** killed (4 new: admits 27 02, admits 2E, drops sub_2761, allows `10 83`). Report:
  "Update 8 — phase 5 (read-only capability battery)".
- **Merge note:** intentionally fork-local; diagnostic only (removed or left inert once the question is answered).

### esc-standstill-fix: ESC read standstill definition + pre-check retry — 2026-10-05 (offline-tested; no firmware change)

- **Why:** first on-car run (2026-10-05, parked, ignition on) wrote `/data/esc-uds/20261005T130925Z-skipped.json`:
  `precheck: moving (wheel speeds [0.0, 0.0, 0.0, 0.03125])`. One LSB on one wheel = sensor noise. The pre-check also
  had already saved `last_ignition`, so it would not retry until the next ignition. Analysis:
  `car-features/esc-read-and-probe-fixes.md`.
- **Log evidence** (396 rlog segments, bus 0, every WHL_SPD11 frame; `car-features/_standstill_eval.py`):
  - Settled in Park (≥ 3 s after the last non-P gear frame), 215,942 frames: 8.4 % have a nonzero wheel. A single
    wheel reads up to 59 LSB, but **2+ wheels above 8 LSB at once: 0 frames.** The old rule (`any != 0`) is false in
    18,114 of them, in runs up to 407 frames (8 s).
  - Genuine motion, 131 starts from an all-zero frame: **"2+ wheels > 12 LSB" fires on all 131**, within ≤ 42 frames
    (0.84 s), after ≤ 2.5 cm of travel.
- **Change (`fork/esc_diag.py`):**
  - `wheels_moving(raw)`: ≥ 2 wheels > 12 LSB, or any wheel > 96 LSB (3 km/h). Computed on raw integers, not floats.
    `VehicleGate.violation()` uses it, so the pre-check, the check before every TX and the check after every receive
    share one definition.
  - **In Park only,** the pre-check waits up to 3 s (`PRECHECK_WAIT_S`) for a "moving" car to settle. Out of Park it
    still skips at once.
  - A pre-check skip increments `precheck_skips{key,n}` (saved before the wait) and does **not** set
    `last_ignition`. `plan()` refuses after `MAX_PRECHECK_SKIPS` = 3 skips in one ignition. `last_ignition` is
    written only once the pre-check passed, still before the multiplexer or any TX.
  - **Retry within an ignition** happens only when card runs again in the same ignition (card restart). There is no
    background retry loop: the read sits in card's fingerprint window and cannot block startup longer than
    3 s + 20 s budget.
- **Tests:** `test_esc_diag.py` 30 → 40. New tests cover: the on-car frame, logged noise shapes, 2-wheel creep, the
  rule boundaries, the TX-site check alone, mid-read creep and noise, the bounded settle wait, a skip not using up
  the ignition, and the per-ignition skip bound. Mutation proof: `car-features/auto-esc-mutation.py` **40/40 killed**
  (10 new mutants for this change).

### fca11-scale-0013: opendbc 0013 FCA11 rolling-test decel cap 0.30 g — 2026-10-05 (offline-tested; one firmware build; NOT road-run)

- **Why:** the 2026-10-05 rolling A/B run (`roll-20261005T190845Z`) made the ESC brake (FCA_ACK 0→1, BrakeLight 0→1,
  −0.39 / −0.81 m/s² extra) but both passes used 0.10 g, so it cannot say whether the braking scales with the command.
- **What:** `HYUNDAI_FCA11_ROLL_MAX_DEC 30` used as the decel cap only when `hyundai_fca11_rolling_test` (64\|128);
  parked (64 alone) keeps `HYUNDAI_FCA11_TEST_MAX_DEC 10`; normal driving still blocks every decel. Python mirror
  `HYUNDAI_FCA11_ROLL_MAX_DEC` in `values.py`.
- **opendbc:** `fca11-scale-0013` @ `97ae2133` on `fca11-warn-0012` @ `935e5ed3`. Patch sha256 `845dec6c62515eae4b7bdd7b9fca54b4df4bde299246e658475f32d868b72d2f`.
- **Verification:** 0001-0013 apply clean on pristine `f95f996f` (13/13, 0 fuzz), tree `1f8824a5` == branch tip.
  Hyundai safety tests: exactly +20 PASSED vs 0012, no outcome changed. Mutation 22/22 non-equivalent killed (2 argued equivalent). MISRA 0 findings.
  Firmware `panda_h7.bin.signed` sha256 `0d5e138ff516e7d4620964518f874ba3d05edd7e4fc46d524ec35b68dc350513`
  (109,352 B, export ×2 == CI series; controls reproduce `d0f5396c` and `65d3692f`).
- **Report:** `car-features/fca11-scaling-prep.md`.

### fca11-warn-0012: opendbc 0012 FCA11 rolling-test Warn gating — 2026-10-05 (offline-tested; one firmware build; NOT road-run)

- **Why:** the rolling brake test's next variants send `CF_VSM_Warn` (dash FCW) with and before the decel. In the
  rolling mode the window/clock applied only to *actuation* frames, so while armed a Warn-only frame (Warn 1-3, no
  actuation bits) passed at any speed, with a pedal pressed or after a latched cut. The fix existed only as opendbc
  `fca11-rolling-warn` @ `1546f434` on the OLD base `f265f992`; its firmware (`824039ad…`) lacks buttons-v3, the 0.35
  ceiling and the timed factory-cruise cancel, so flashing it would have downgraded the daily-driver safety. This
  carries the one commit onto the deployed stack as patch `0012` instead.
- **What (`opendbc/safety/modes/hyundai.h`, rolling branch of the FCA11 TX check only):** `gated = actuation ||
  CF_VSM_Warn != 0` replaces `actuation` in the window check, the 1.2 s clock check and the clock start/stop. A Warn
  lead-in followed by actuation is ONE episode (1.2 s total). Nothing outside `if (hyundai_fca11_rolling_test)` changed:
  normal driving (no bit 64), the parked test mode (64 without 128) and every pedal/cruise path are untouched.
- **opendbc:** `fca11-warn-0012` @ `935e5ed3` = `1546f434` cherry-picked onto `pedal-tune-12ef` @ `f5fc1256` (clean,
  identical `git patch-id`; one test comment renumbered 0011 → 0012). Patch
  `0012-hyundai-fca11-rolling-warn-gate.patch` (`git format-patch`, submodule-relative) sha256
  `2d2d0ef24eab299f8e01eff855fea2235c3008c42b28b10aac547d35e10fcb39`.
- **Verification:** 0001-0012 apply clean on pristine `f95f996f` (12/12, 0 fuzz/offset), tree `620a3fd7` == branch tip.
  `test_gas_interceptor` + `test_hyundai` + `test_release_build`: **2432 passed, 321 skipped, 2125 subtests** (base
  f5fc1256: 2416 / 321 / 2079); per-test outcome diff vs base = exactly the 16 new Warn tests added as PASSED, no
  other test changed outcome, no new skips. Mutations **9/9 killed** (`car-features/fca11-rolling-warn-mutation.py
  fca11-warn-0012`). MISRA/cppcheck 2.21.0: 0 findings.
- **Firmware:** panda `74a0adce`, arm-none-eabi 13.2.1, CI recipe → `panda_h7.bin.signed` sha256
  **`65d3692f5bfc6c0850a1a779b0b4590374f439de4c41267d44a9c17c27c7db4e`**, 109,340 B, `DEV-74a0adce-DEBUG`, 0
  warnings; two clean `git archive` builds and the patch-series (CI-path) tree build byte-identical; control
  `f5fc1256` reproduces the deployed `d0f5396c…`. **Merging this changes the firmware → one panda reflash.**
- **Report:** `car-features/fca11-warn-0012-report.md`.

### auto-esc-read: automatic read-only ESC UDS read + MapTargetVelocities in the rlog — 2026-10-04 (offline-tested; no firmware change)

- **Why:** `car-features/auto-esc-read-brief.md` / `auto-esc-read-report.md`. The FCA11/AEB work needs the ESC's
  identification + variant-coding DIDs and stored DTCs. The SCC-Map analyses had to fake the map turn path because
  `MapTargetVelocities` only lives in `/dev/shm` params.
- **ESC read (`openpilot/sunnypilot/fork/esc_diag.py`, one call in `selfdrive/car/card.py`):** runs after `get_car()` and
  before `FirmwareQueryDone`. The panda is still in ELM327 safety then (openpilot's own fw query talks to this ESC
  in that window every boot: `abs` 0x7D1 -> 0x7D9, bus 1 with OBD multiplexing). **No panda firmware change.**
  - **Read-only:** only single frames 0x22 / 0x3E / 0x10 03 / 0x19 02 FF + one fixed flow-control frame, to 0x7D1 on
    bus 1. Two guards: per service before a frame is built, and per frame at the single `can_send` site.
  - **Stationary only:** gear P (LVR12) and the wheels at standstill (WHL_SPD11; **was** all four exactly 0, see
    esc-standstill-fix), from bus-0 frames < 0.25 s old.
    Checked before every frame and after every receive; any violation aborts at once.
  - **Startup cost and multiplexer:** openpilot cannot engage yet in this window, and the pedal gets no commands. The
    OBD multiplexer is switched back off in `finally` (SIGTERM too) and verified from pandaStates. Budget 20 s;
    stops after 4 requests in a row with no answer. Errors never propagate.
  - **Schedule:** the full DID read runs once per ESC firmware version (max 3 attempts; skipped once a complete read
    exists). DTCs are read once per UTC day. `last_ignition` is saved once the pre-check has passed, still before any
    send; a pre-check skip does not consume the ignition (**amended by esc-standstill-fix:** ≤ 3 tries per ignition,
    ≤ 3 s settle wait in Park, immediate skip out of Park).
  - **Output:** `/data/esc-uds/<UTC>-{full,dtc,fulldtc,skipped}.json` (raw request/response hex, NRCs, TX log; 60 files
    max) + `esc_uds_read` cloudlog event (rlog). Disable: `touch /data/esc-uds/DISABLE`.
- **Route data (`fork/scc.py`):** `scc_map_path` cloudlog event = the MapTargetVelocities list (lat/lon/v, ≤ 400
  points). Logged only when its content changes and at most once per 2 s, engaged or not.
- **Tests:** `fork/tests/test_esc_diag.py` (30, fake car through the real `run()` path) + 2 new in `test_scc.py`.
  Mutation proof: `car-features/auto-esc-mutation.py`, **30/30 mutants killed** (one per hard rule).
- **Not done / open:** N drive mode is NOT broadcast on C-CAN (`car-features/n-drive-mode-decode.md`); it needs a
  labeled capture. First on-car run will show whether every DID fits the fixed flow-control frame (BS 0 = no limit).

### pedal-tune-12ef: drive 12e/12f fixes — opendbc 0011 pedal gain/jerk + SCC fixes + throttle-only SCC guard + follow 1.1 — 2026-10-04 (offline-tested, NOT road-validated; no firmware change)

> **Driver notes (read before the next drive):**
> - **Pull-aways are gentler.** Below ~22 m/s (50 mph) openpilot uses much less pedal per m/s² asked (the car is 2-3×
>   stronger there than on the highway). The 25 mph "surge" should be gone. The cost is about 1.5 s longer to reach
>   35-45 mph on a cruise-only launch. Engaging while rolling ramps the throttle in over ~1 s at 25 mph (was 0.5 s).
>   From 50 mph up nothing changes.
> - **Launching into a turn keeps accelerating.** The vision turn controller no longer engages below 20 mph, and it never
>   asks for less than 12 mph.
> - **The phantom on-ramp slowdown is fixed.** The map turn controller ignores its path when mapd is unmatched or the
>   path is > 25 m away. As a backstop, any turn-controller coast with no curve visible (no lateral accel, none
>   predicted) is cancelled after 2 s.
> - **Interstate follow is a bit closer**: ~1.8 s behind a car at 65 mph (was ~2.0 s). Town is unchanged.
> - **Nothing about brakes/gas-pressed/panda changed.** openpilot still cannot brake.

- **Why:** `car-features/drive-12e-12f-report.md` (routes 0000012e / 0000012f).
- **0011 (opendbc, `gas_interceptor.py`):** `cmd = pedal_scale(v)·a + hold_cmd(v)`.
  - `PEDAL_SCALE_V` [0.10, 0.10, 0.12, 0.14, 0.165, 0.20, 0.25, 0.36] at [0, 3, 6, 9, 12, 15, 18, 22] m/s; ~10-20 % under the
    fitted 1/G.
  - `HOLD_CMD_V` [0.04, 0.08, 0.085, 0.095, then 0.36 × (0.34, 0.43, 0.49, 0.56)] at [0..35 step 5] m/s.
  - Rise limit `2.0 m/s³ × pedal_scale(v)` per second (0.20/s at 0, 0.33/s at 12 m/s, 0.72/s from 22 m/s). Decreases
    stay immediate.
  - Bit-identical to the 0010 law at ≥ 22 m/s (test). `MAX_INTERCEPTOR_GAS` 0.35, `LOW_SPEED_MAX_GAS`, panda
    `HYUNDAI_GAS_INTERCEPTOR_MAX_GAS_A/_B` and the gas-pressed threshold are untouched. Python only, so **no panda reflash**.
- **SCC (`openpilot/sunnypilot/fork/scc.py`):** `ForkSCCVision` / `ForkSCCMap` / `ForkSmartCruiseControl` subclass the
  upstream controllers, so the upstream files are unchanged. The single hook is in
  `sunnypilot/selfdrive/controls/lib/longitudinal_planner.py` (`self.scc = ForkSmartCruiseControl(is_throttle_only(...))`).
  `fork/tests/test_scc.py::TestUpstreamHooks` pins the overridden upstream members.
  - Vision: output = max(upstream, MIN_V) while active; no ENTERING below 9 m/s; ENTERING with current lat ≥ 1.0 →
    TURNING.
  - Map: path rejected if nearest point > 25 m, or mapd unmatched (roadType unknown, no speed limit, no name). The
    rejection is immediate while not turning and needs 1 s while turning (a match blip does not snap back to cruise).
  - Diagnostics: cloudlog `scc_map_diag` at 1 Hz while engaged plus on state/gate change (points, nearest m/idx,
    target lat/lon/m, road info, reason). Cloudlog was chosen to avoid a capnp change.
  - Throttle-only guard (Hyundai + interceptor only). Time accumulates while an SCC output is below vEgo − 0.5, the
    plan is coasting (< −0.05) and there is no curve evidence (current lat < 1.0 and predicted lat < 1.3). After 2 s
    the SCC targets are released (V_CRUISE_UNSET) and stay latched until curve evidence appears, the raw SCC target
    reaches ≥ vEgo + 1 or long disengages. Cloudlog `scc_throttle_only_guard` on each edge.
  - Guard arming keys off `CP.brand == "hyundai"` (opendbc has no brand enum/constant; the folder-name string is the
    contract). `test_adaptive_follow.py::TestThrottleOnlyRealCarParams` builds the real `HYUNDAI_ELANTRA_2022_NON_SCC`
    interface with the pedal enabled and pins `is_throttle_only() is True` (plus a real Toyota interceptor at False), so
    an upstream brand rename fails the suite instead of silently disarming the guard on the owner's car.
  - Sizing: on 12e/12f the only uncorroborated SCC coast > 0.7 s is the 12f @699 phantom (5.4 s); legit slowdowns,
    including the 9.5 s 12f @1349 curve, were corroborated from their start.
- **Adaptive follow:** `HIGHWAY_FACTOR_V` [1.0, 1.2] → [1.0, 1.1] (gap at 29 m/s 1.95 → 1.80 s). Unknown road stays 1.1.
- **Verification (post-review fixes):** opendbc 0001-0011 on pristine f95f996f: 11/11 clean, 0 fuzz, tree == branch tip
  (patch 0011 sha256 `d2b3c19e509455f95e1283426d00fd55c6282635058e02efe3830fb204bcd9e9`). `test_gas_interceptor` +
  `test_hyundai` + `test_release_build` **145 passed, 2 skipped, 388 subtests**. openpilot pedal/events/fork/SCC/mapd/
  longcontrol **172 passed** (63 fork+SCC incl. the 2 new real-CarParams guard tests, 32 mapd, 2 longcontrol, 25 pedal,
  50 events). Mutations **33/33 killed** (9 pedal/jerk, 4 vision, 8 map, 7 guard, 1 hook, 2 follow, plus the reviewer's
  3). Launch sim through the shipped code: max(aEgo − request) at
  6-14 m/s +0.44 → +0.03, peak 1.59 → 1.28 m/s².
- **Report:** `car-features/drive-12ef-fixes-report.md`.

### buttons-v3: opendbc 0010 pedal buttons v3 — 2026-10-04 (one firmware build; offline-tested + independently reviewed GO, NOT road-validated)

- opendbc `buttons-v3` @ 0a19ee8a (on 2258c35a); patch `0010-hyundai-pedal-buttons-v3.patch`; panda firmware sha256 `d0f5396c55a1d5a0b34cb193172a12dd7890d6d2ad65e97fc1b6cca9fba98a1b` (109336 B, two clean `git archive` builds byte-identical; reviewer rebuilt it independently).
- Wheel buttons (CF_Clu_CruiseSwState up=1, down=2, pause/resume=4, from routes 127/128):
  - pause/resume is the ONLY on/off for openpilot long, at any speed incl. standstill; deliberate press only, NO auto-resume (panda + openpilot lockstep).
  - up/down ONLY change the set speed (±1 mph short, +5 long-up), engaged or not; they never engage or disengage (panda grant removed in pedal mode).
  - long press down (0.5 s) sets the set speed to the current speed, never engages.
- `minEnableSpeed = -1` (no floor); the 25 mph refusal beep/alert and "Press Set to Engage" removed in pedal mode. The launch limit (12% at standstill -> 35% cap by 25 mph) stays; it caps throttle, it is not an engage floor. openpilot still cannot brake.
- Driver notes: pause/resume engages at the stored set speed (or current speed rounded to mph if none yet; 5 mph from a stop). A stored speed far BELOW current speed coasts down; a stored speed far ABOVE (e.g. 65 from the highway, resumed at a city light) accelerates toward it, launch-limited. Check the grey MAX number or long-press down first.
- Verification: opendbc 2408 passed; openpilot 99 passed; mutations 60/61 killed (+6 reviewer mutants killed; the survivor is the known equivalent); 0001-0010 apply clean on pristine f95f996f; MISRA 0 findings. Review: car-features/buttons-v3-review.md.

### integration-3: integration-2 + opendbc 0009 timed factory-cruise cancel — 2026-10-04 (one firmware build; offline-tested, NOT road-validated)

> **Driver notes (read before the next drive):** everything in the integration-2 notes below still applies.
> - **New backstop, should rarely fire.** If the factory cruise ever becomes *active* while openpilot long is engaged
>   (the only case seen in the logs is pressing CC/MAIN while engaged), openpilot long drops immediately (as before) and
>   panda tries to cancel the factory cruise itself with a short pause/resume press on the cluster's behalf.
> - **If the factory cruise is still on 2.5 s later you get an audible "Cruise Fault"** (immediate disable) and the
>   alert stays while factory cruise is on. Then **press CC off or brake** — the factory cruise is in control.
> - **Whether the ECM accepts panda's cancel is unproven until the road test.** Do not rely on it; keep CC/MAIN off.

- **What:** opendbc **`0009`** = `timed-cancel` fbc82f9c (written on pedal-buttons-v2 24ba04bb) cherry-picked onto
  `integration-2` cc592f1c as opendbc branch `integration-3` (032ded0d; final 2258c35a after the 3b review fixes), plus one integration test. The car-features
  file `0007-hyundai-timed-factory-cruise-cancel.patch` is renumbered `0009` and regenerated against cc592f1c.
  `0001`-`0008` unchanged. No superproject code change: `accFaulted` already maps to the audible immediate disable.
- **Conflicts:** none (auto-merge in `hyundai.h`, `hyundai_common.h`, `test_hyundai.py`, `test_gas_interceptor.py`).
  Audited: the cancel is gated on `hyundai_gas_interceptor`, which `hyundai_common_init` forces false with bit 64 (and
  so 64+128) → inert in both FCA11 test modes; it runs before the v2 MAIN/active lockout on the same EMS16 frame (the
  lockout still drops long); it never touches the pedal command path or the 0.35 ceiling; pedal TX lists still have
  no CLU11. New test `test_test_bit_overrides_timed_factory_cancel` (64 and 64+128, with pedal, both dialects):
  0 self-TX, with a pedal-only positive control of 12.
- **Test harness:** libsafety gains a recording `can_send`/`can_set_checksum` and `set_heartbeat_engaged`
  (reset in `init_tests`); the full targeted suites pass with it.
- **Verification:** CI-style rehearsal from the committed patch blobs (fresh clone of `f95f996f`, sorted glob, plain
  `git apply --verbose`): 9/9, no fuzz, tree `f312b599` == `integration-3` tip. `test_gas_interceptor` + `test_hyundai`
  + `test_release_build` **2374 passed**; whole `opendbc/safety/tests` **8252 passed** (harness change breaks nothing);
  `test_pedal_pause_resume` + `test_car_specific_events` + `fork/tests` **54 passed**. Mutations **29/30** killed (all
  integration-2 mutants + CLU11 TX outside the trigger ×2, TX without a cluster frame, rate cap removed, parity not
  recomputed, TX in FCA11 test mode, cancel after the lockout, lockout removed, Cruise Fault removed); the one survivor
  is integration-2's documented equivalent. MISRA (cppcheck 2.21.0): 0 findings. Panda `74a0adce` DEBUG build:
  `panda_h7.bin.signed` sha256 `53700ef3…23eb` (109316 B) was the pre-review build; the shipped integration-3b firmware is `49bff2e0…52ea` (109444 B), built from two clean `git archive` exports of 2258c35a, byte-identical.
- **Correction to integration-2 below:** its recorded firmware sha `1d654e95…` is NOT tree `94f3d9f6`; it reproduces
  exactly as `94f3d9f6` with the "pedal ceiling removed" mutant applied (the build shared the rehearsal tree with a
  concurrent mutation run). The correct integration-2 build is `4c7514a7…d80a` (108700 B). Never flash `1d654e95`.
- **Deploy note:** CHANGES THE FIRMWARE (one reflash). Report: `car-features/integration-3-report.md`.

### integration-2: opendbc 0006 pedal buttons v2 + 0007 FCA11 rolling test + 0008 cap 0.35, with adaptive follow — 2026-10-04 (one firmware build; offline-tested, NOT road-validated)

> **Driver notes (read before the next drive):**
> - **Keep the factory CC/MAIN button OFF.** openpilot long never needs it. If it gets armed, openpilot long turns off
>   and shows "Factory Cruise Armed: openpilot Long Off"; SET/RES then drive the *factory* cruise, which openpilot can't
>   cancel. Press CC off (or brake).
> - **Pause/resume = openpilot long on/off.** Pressed while engaged: off. Pressed while off (foot off the brake): on, at
>   the previous set speed, **at any speed including a standstill**, also with the foot on the gas (it takes over when
>   you lift). It **re-engages only on a deliberate press**: releasing the brake, waiting or speed recovering never
>   turns it on.
> - **openpilot cannot brake. You brake for every stop** (stopped car, red light, the DCT creeps on its own). From a
>   stop the pedal is limited to 12 % rising to the full cap by 25 mph; any brake press turns long off immediately.
> - **SET/RES** still need 25 mph+; below that they now refuse with a sound and point at pause/resume. SET uses the
>   current speed (no more jump to 65 in experimental mode).
> - **Throttle cap 30 % → 35 %** of pedal travel (hard-limited in panda), for on-ramps.
> - **The FCA11 rolling brake-test bit (128) is test-only and inert** unless the parked runner arms it (sp 200 = 8|64|128).
>   Nothing in normal driving sets it.
> - Adaptive follow distance is behind `AdaptiveFollowDistance` (default OFF); the wheel-speed correction (1.0125) is
>   always on for the Elantra N.

- **What:** opendbc series is now `0001` → `0002` v2 → `0003` → `0004` → `0005` → **`0006`** (pedal buttons v2,
  `pedal-buttons-v2` 24ba04bb) → **`0007`** (FCA11 rolling, `fca11-rolling` d2c8497f) → **`0008`** (cap 0.35). Built
  as opendbc branch `integration-2` on `f265f992`: 7607261b (cherry-pick 24ba04bb, clean), 4951ec91 (cherry-pick
  d2c8497f + 1 integration test), cc592f1c (cap). Tree `94f3d9f6`. Superproject side: pedal-buttons-v2 a80ec7360c and
  planner-tune 52df330beb (both clean on 08f100c21e). `0001`-`0005` unchanged; the two car-features files both named
  `0006` are now `0006` (buttons v2, body identical) and `0007` (rolling).
- **Conflicts:** one, `test_hyundai.py` imports (rolling's speed constants vs v2's `PAUSE_RELEASE_SAMPLES`), kept both.
  `hyundai.h`/`hyundai_common.h` auto-merged; audited: no logic interacts.
- **Test modes still override the pedal:** `hyundai_common_init` forces `hyundai_gas_interceptor = false` whenever bit
  64 is armed, and rolling (128) is only honoured on top of an armed 64. So with 64 or 64+128 none of the pedal
  button/launch/hold/ceiling code runs; TX is CLU11 cancel-only + `0x38D` (no 0x340/0x485, no pedal IDs); the pedal
  TX lists still have no CLU11. New test `test_rolling_bits_override_pedal_buttons_v2` (64+128+pedal, both dialects):
  no v2 pause/resume grant (gas held, standstill, stale grant), no pedal command, EMS16 still latches the rolling cut,
  dynamic camera hand-back intact.
- **0008 (cap 0.35):** `MAX_INTERCEPTOR_GAS` 0.35; panda `HYUNDAI_GAS_INTERCEPTOR_MAX_GAS_A/_B` from the packer exactly
  as 0004: **A 1218 / B 612** (was 1110 / 559) = 35.0 % of track-A travel; B on the measured line B = 0.494·A + 10.5
  (612.3). `LOW_SPEED_MAX_GAS` ends at the cap at 25 mph (the SET/RES floor, 11.18 m/s).
- **Verification:** CI-style rehearsal (fresh clone of `f95f996f`, sorted glob, plain `git apply --verbose`): 8/8, no
  fuzz, tree `94f3d9f6` == `integration-2` tip. `test_gas_interceptor` + `test_hyundai` + `test_release_build`
  **2263 passed**; `test_pedal_pause_resume` + `test_car_specific_events` + `fork/tests` **54 passed**. Mutations 20/21
  killed (auto-resume panda/car, ceiling removed, cap reverted, stale ceiling, launch limit removed, CLU11 back in
  either pedal list, rolling ceiling/driver cut/hand-backs removed, 0x340 re-blocked in both test lists, test bits not
  overriding the pedal, pause machine in test mode, adaptive-follow hook and wheelSpeedFactor dropped); one equivalent
  survivor (128 without 64), its paired mutant killed. MISRA (cppcheck 2.21.0, `test_misra.sh` flags): 0 findings.
  Panda `74a0adce` DEBUG build: `panda_h7.bin.signed` sha256 **`4c7514a7…d80a`** (108700 B). *Corrected 2026-10-04: the
  originally recorded `1d654e95…9d6d55` (108668 B) was built from the rehearsal tree while a mutation ("pedal ceiling
  removed") was applied; never flash it.* Rolling/parked runners checked against the firmware constants: match, unchanged.
- **Deploy note:** CHANGES THE FIRMWARE (one reflash). `params_keys.h` changed → libparams rebuild on sync.
- Report: `car-features/integration-2-report.md`.

### feature: adaptive follow distance + Elantra N wheel-speed factor — 2026-10-03 (offline-tested, NOT road-validated)

- **Why:** drive report `car-features/drive-128-planner-report.md` (routes 00000127 + 00000128). The car followed at
  a median **1.0 s** at 26-32 m/s on the interstate (truck ahead) vs the MPC's 1.73 s target, and the cluster read
  2-3 mph over the set speed. Root causes are mostly in the pedal layer (hold-offset table is 0.1-0.25 m/s^2 too high
  above 22 m/s; the fix is in the report, for the opendbc patch series). This entry holds the planner/car-param part.
- **wheelSpeedFactor (fork/vehicle_specs.py):** `HYUNDAI_ELANTRA_2022_NON_SCC` → 1.0125. Over 46 km, GPS doppler/vEgo
  = 1.0125 and GPS-position distance/vEgo distance = 1.0131, flat over 12-36 m/s, so the car ran 1.25 % (0.9 mph at
  70) above the set speed in true terms. CarState reads `CP.wheelSpeedFactor` live and shares card's CP object, so
  the override reaches vEgoRaw (test drives the real Hyundai CarInterface). Tire-dependent: re-measure after tire
  changes.
- **Adaptive follow (fork/adaptive_follow.py, param `AdaptiveFollowDistance`, BOOL default 0):**
  `T = T_FOLLOW(personality) × factor(roadType, vEgo) + closing_margin`. factor = 1.0 urban (never shorter: the car
  cannot brake), 1.0→1.2 between 20 and 29 m/s on highway/interstate, 1.0→1.1 on unknown. The closing margin applies
  only to throttle-only cars (Hyundai + `enableGasInterceptor`) with a slower lead: the extra distance to cancel the
  closing speed by coasting (measured 0.27-0.42 m/s^2, derated) instead of the MPC's 2.5 m/s^2 `COMFORT_BRAKE`, capped
  at 0.6 s. Output is rate-limited (+1.0 s/s, −0.2 s/s). Hook: `LongitudinalMpc.update(..., t_follow=None)` (the
  time gap is already runtime param p[4], so OCP/codegen are unchanged); `LongitudinalPlannerSP.get_t_follow`
  computes it. Param off, or unknown to a stale libparams → returns None → stock T_FOLLOW, byte-identical behaviour.
  modelV2 leads carry no class/size, so trucks are not singled out.
- **Files:** `openpilot/sunnypilot/fork/{adaptive_follow.py,vehicle_specs.py}`,
  `openpilot/sunnypilot/fork/tests/{test_adaptive_follow.py,test_vehicle_specs.py}`,
  `openpilot/selfdrive/controls/lib/{longitudinal_planner.py,longitudinal_mpc_lib/long_mpc.py}`,
  `openpilot/sunnypilot/selfdrive/controls/lib/longitudinal_planner.py`, `openpilot/common/params_keys.h`
  (new key → **libparams rebuild** on sync, already gated by the CI params_keys.h check).
- **Verification:** `fork/tests` 24 passed, including end to end through the real LongitudinalPlanner + acados MPC
  (Plant): the MPC receives the override, and the steady gap grows by the expected +10 % T·v. Two mutants were killed
  (hook dropped, wheelSpeedFactor dropped). Closed-loop replay of the three logged highway follows, through the real
  planner/MPC with a fitted pedal plant (`car-features/planner128/replay_follow.py`), reproduces the logged gap (sim
  1.13/1.22 s vs logged 1.20/0.98 s). Upstream `test_following_distance.py` fails 15/18 on an UNMODIFIED tree on this
  host: Plant's Honda CP has `openpilotLongitudinalControl=False`, so the planner resets every step. This is
  pre-existing and not caused by this change.

### integration: opendbc series 0002 v2 + 0004 + 0005 — 2026-10-03 (one firmware build; offline-tested, NOT road-validated)

> **Driver notes (read before the next drive):**
> - **Do NOT arm the factory cruise MAIN button.** It is no longer needed to clear "Enable Adaptive Cruise to Engage".
>   Pedal-long is available with MAIN off.
> - **MAIN armed = pedal-long locked out.** Panda drops longitudinal and refuses every engage while the MAIN lamp is on;
>   openpilot shows "Adaptive Cruise Disabled" (`wrongCruiseMode`). Switch MAIN off to get pedal-long back. Lateral
>   (MADS) is unaffected either way.
> - **Pause/resume (the CANCEL position, `CF_Clu_CruiseSwState` 4) re-engages only on a deliberate press:** pressed while
>   engaged it disengages (as before). Pressed while disengaged, it resumes at the previous set speed, only if brake and
>   gas stayed released for the whole press plus a 60 ms release debounce, MAIN is off and speed is above the engage
>   minimum. **There is no auto-resume**: releasing the brake, a timer or speed recovering never re-engages.
> - **Throttle authority roughly doubles** (cap 15 % → 30 % pedal travel, hard-limited in panda). Expect firmer pulls
>   uphill and on engage (slewed at 0.75/s). The brake or your foot on the gas still disengages/overrides immediately.
> - **openpilot no longer sends any CLU11 in pedal mode** (the cause of the pedal SCE latches), so it can't cancel a
>   factory cruise. That is exactly why MAIN must stay off.

- **What:** the opendbc patch series is now `0001` (pedal) → `0002` **v2** (FCA11 brake test) → `0003` (SCE fix) →
  `0004` (tune) → `0005` (buttons). It was built as one opendbc branch `pedal-integration` (`f265f992`, tree
  `38d8bf84`): 00d48a2f + cherry-picks of ab370764, 2634c2e2, 55e0a7f1, f286ba76, ae903fd5. `0001` and `0003` are
  byte-identical to before. `0002` is regenerated as two commits (v1 + v2 fix) on `00d48a2f`.
- **Conflicts (both "keep both"):** (1) `0004` × `0002`: the `HYUNDAI_GAS_INTERCEPTOR_MAX_GAS_A/_B` defines and
  `HYUNDAI_FCA11_TEST_MAX_DEC` landed at the same spot in `hyundai.h` (0004 was cut without 0002). (2) `0005` × `0002`:
  the `hyundai_fca11_brake_test` and `hyundai_factory_main_on`/pause-state declarations landed at the same spot in
  `hyundai_common.h`. No logic conflicted.
- **FCA11 test mode (bit 64) still overrides pedal mode:** `hyundai_common_init` forces `hyundai_gas_interceptor =
  false` when the test is armed, and every 0005 path (MAIN lockout in the EMS16 RX, `pedal_check`, the pause state
  machine, the SET/RES brake block) is gated on `hyundai_gas_interceptor`, so test mode keeps the stock non-SCC
  semantics. Its TX list is still exactly CLU11 (cancel only) + `0x38D` check_relay. 0005's pedal TX lists (LKAS11 +
  LFAHDA + pedal command, no CLU11) and 0004's ceiling coexist in `hyundai.h`. A new test,
  `test_test_bit_overrides_pedal_buttons` (in 0005), pins it with both bits set: MAIN is not a lockout, CLU11 cancel
  is TX-able and pause/resume grants nothing.
- **Supersedes:** #14's FIX 2 (bursted CLU11 cancel) is removed by 0005 (no CLU11 in pedal mode at all). #14's FIX 1
  (the clear frame + escalation) stays.
- **Files:** `openpilot/sunnypilot/fork/patches/0002-hyundai-fca11-brake-test.patch` (replaced with v2),
  `0004-hyundai-pedal-tune.patch`, `0005-hyundai-pedal-buttons.patch` (new);
  `openpilot/sunnypilot/selfdrive/car/tests/test_pedal_pause_resume.py` (new, from `hyundai-pedal-buttons` ca790223d0).
  Reports: `car-features/integration-report.md`, `cruise-buttons-report.md`, `drive-123-report.md`,
  `pedal-tune-verification.md`, `fca11-prereqs.md`.
- **Verification:** CI-style rehearsal (fresh clone of the pin `f95f996f`, sorted glob, plain `git apply --verbose`
  from inside the submodule): 5/5 applied, no fuzz, tree `38d8bf84` == integrated branch tip. Results on that tree:
  `test_gas_interceptor.py` + `test_hyundai.py` + `test_release_build.py` **2063 passed**;
  `test_pedal_pause_resume.py` + `test_car_specific_events.py` **22 passed**. Cross-feature mutations (auto-resume,
  ceiling removed/raised, cap reverted, FCA11 0x340/0x485 re-blocked, CLU11 back in either pedal list, test bit no
  longer overriding pedal mode, MAIN lockout/pause machine leaking into test mode, test TX list swapped) were all
  killed. Panda `74a0adce` DEBUG build from the rehearsal tree: `panda_h7.bin.signed` sha256 `4bac8d02…28e4f5` (106400 B,
  `DEV-74a0adce-DEBUG`). The control build of master's series reproduces `61b3165d…`.
- **Deploy note:** this CHANGES THE FIRMWARE (0004/0005 are safety C). It means one device reflash on the next sync.
  `cppcheck`/MISRA was not run locally.

### fix: Hyundai pedal SCE-latch recovery + bursted cruise cancel — 2026-10-03 (amends #10; offline-tested, NOT road-validated)

- **Why:** route `0000011c` forensics (`car-features/field-drive-11c-report.md`). During commanded engagements the pedal
  latched FAULT_SCE (state 3, i.e. passthrough) and stayed latched. Commanded `ENABLE=1` frames never clear a pedal fault;
  only an `ENABLE=0` all-zero frame does. The pedal applied only **6.56 s of 55.1 s commanded (11.9 %)**: the
  "surge then shut off". The trigger is our own CLU11 (`0x4F1`) CANCEL. It went out every 10 ms frame while factory cruise
  was on, colliding with the cluster's 50 Hz CLU11 and producing panda bit1Errors and bus-offs. **43/43** SCE onsets and
  **1282/1282** bus-0 errors fell inside cancel streams; there were 0 outside them.
- **FIX 1 — clear frame (`gas_interceptor.py: PedalFaultMonitor`):** while commanding (`active` and gas > 0), if the
  pedal's broadcast STATE is nonzero (any fault, 1-6), that 50 Hz send slot carries ONE `ENABLE=0` zero frame instead
  of the command. The counter stays continuous and the CRC is valid. It costs no torque because a faulted pedal is
  already in passthrough. Rate limit: at most one clear per 60 ms (`PEDAL_CLEAR_HOLDOFF_FRAMES = 6`; the measured
  clear round trip is 16-26 ms, plus one CarState tick). Panda safety needs no change, since a zero `GAS_COMMAND` is
  always allowed.
- **Escalation:** if a fault stays visible while commanding for **> 0.5 s**, or there are **> 8 clears in 2 s**, then
  `CarState.accFaulted = True`. That is the standard `accFaulted` event, an IMMEDIATE_DISABLE "Cruise Fault" alert, so
  the driver knows throttle authority is gone. It is held until longitudinal disengages and re-armed when not commanding.
  A fault that the clears can't recover hits the count path at 480 ms. Chosen over a new sunnypilot event:
  `accFaulted` already exists end to end (car_events → events.py alert) and needs no cereal/event-enum change. Its
  "restart the car" wording is stronger than necessary (re-engage works once the pedal is back to state 0).
- **Plumbing (no schema change):** `CarStateExt` owns the `PedalFaultMonitor` next to `interceptor_state`.
  `CarInterfaceBase.apply` passes **that same CarState instance** as `CS` to `CarController.update` →
  `create_gas_command`, so the controller reads `CS.interceptor_state` and steps `CS.pedal_fault_monitor`.
  `CarStateExt.update` reads `escalated` into `accFaulted` on the next frame (one 10 ms tick of latency). A new
  `custom.capnp CarStateSP` field was rejected for three reasons: card would only republish data that never leaves the
  card process; it would force a capnp C++ regen on sync (`SCHEMA_CHANGED`); and it would split the change across the
  superproject and the patch series. Everything is inert unless `CP_SP.enableGasInterceptor`: `create_gas_command`
  returns early otherwise, and the STATE parse and escalation are both inside the interceptor block.
- **FIX 2 — bursted cancel (`car/hyundai/carcontroller.py`, gas-interceptor mode only):** after the existing 10-frame
  `CANCEL_BUTTON_DELAY_FRAMES`, CLU11 CANCEL is now sent as **5-frame bursts with pauses rotating 150/200/250 ms**
  (20 % duty, 75-frame cycle) instead of every frame. The pause rotates so a fixed period can't phase-lock against the
  cluster's 20 ms schedule. Cancel semantics are unchanged: frames go out only while `CC.cruiseControl.cancel` and stop
  the frame it drops. Stock (`pcmCruise`) Hyundai CAN cars keep the continuous stream (a test pins this). The CAN FD
  path is untouched. `cancel_counter`/`last_button_frame` are not shared with ICBM (it receives `last_button_frame`
  by value and never sees `cancel_counter`).
  - Cancel latency on 11c with the continuous stream was 0.04-12.4 s. The fast cases (0.04-0.13 s) needed 5-13 frames;
    the long tails (3.7 / 9.3 / 12.4 s) ran at 25-78 bus errors/s. So the stream is not latency-efficient: the tail
    looks collision/phase-driven, not frame-count-driven. A 5-frame burst covers the fast cases. Whether it lands
    better or worse in the tails is **unmeasured**.
  - **Watch on the next drive:** factory-cruise-on → off latency per cancel, and per-burst bus-0 `totalErrorCnt` /
    `busOffCnt` deltas. If a clean 5-frame burst sometimes doesn't cancel (the cluster/EMS may debounce on hold
    duration), lengthen the burst; don't shorten the pause.
- **Files (opendbc, via patch `openpilot/sunnypilot/fork/patches/0003-hyundai-pedal-sce-fix.patch`, opendbc commit
  `55e0a7f1` on branch `hyundai-pedal-sce-fix`, parent `00d48a2f`):** `car/hyundai/carcontroller.py`,
  `sunnypilot/car/hyundai/{gas_interceptor,carstate_ext}.py`, `sunnypilot/car/hyundai/tests/test_gas_interceptor.py`.
  There is no file overlap with `0002`. **Python only, no safety C**, so the panda firmware binary should be unchanged.
  Note: the sync workflow still REBUILDS firmware because the patch-set hash changed (expected to reproduce the
  0001+0002 bytes, `61b3165d…`; not built locally).
- **Verification:** `test_gas_interceptor.py` **81 passed** (+17 new: monitor policy, end-to-end through `CarInterface.update`/`apply`
  for both ID dialects, non-interceptor inertness, cancel bursts). `safety/tests/test_hyundai.py` **1762 passed**
  (unchanged). Series rehearsal: pristine `f95f996f` + `0001` + `0002` + `0003`, all plain `git apply` exit 0 with no
  fuzz → tree `f1621b30…`. Both suites on that tree: **1998 passed**. Mutation-checked: removing the rate limit → 5
  failures; removing the clear-frame substitution → 2; removing the accFaulted escalation → 1; restoring the continuous
  cancel → 1. Route-11c replay sim (`car-features/fd11c-sce-sim.py`, 200 seeds, uses the real `PedalFaultMonitor`):
  recorded 12.0 % applied; modeled no-fix 10.6 % (sanity check); **FIX 1 97.3 %** (min 96.2), with 0.23 escalations
  per run; FIX 1 + FIX 2 99.5 %, which *assumes* onset rate scales with cancel duty.
- **NOT proven:** road effectiveness, how the clear frame behaves under the user's custom-ID pedal firmware at speed
  (evidence: 3/3 on-road latches and ~40 uncommanded blips, all cleared by zero frames), the ECU response to a 20 ms
  clear dip, and FIX 2's effect on bus errors and cancel latency. Acceptance on the next drive: pedal-applied/commanded
  ≥ 90 %, no `accFaulted` except under real faults, and criteria A-D still pass.

### research: Hyundai FCA11 brake-injection TEST safety mode — 2026-10-03 (TEST-GATED, NOT FOR ROAD USE)

- **What:** panda safety-mode bit for a PARKED, STANDSTILL test of FCA11 (`0x38D`, forward camera → ESC) brake-message
  injection ("software AEB" research) on the non-SCC Elantra N. New `safety_param_sp` bit
  `HYUNDAI_PARAM_SP_FCA11_BRAKE_TEST = 64` (Python `HyundaiSafetyFlagsSP.FCA11_BRAKE_TEST`). **Safety C only — no car-layer
  code sets the bit**; nothing changes on the car unless something explicitly sends `safetyParamSP |= 64`.
- **Armed only when:** bit && non-SCC && ICE (not EV/HEV/FCEV, not camera-SCC) && `!hyundai_longitudinal` — the same
  predicate as the gas interceptor, shared in `hyundai_common_init`. While armed, the bit **overrides** the gas interceptor
  (no `0x200`/`0x700` TX). Forced off in `hyundaiLegacy` and `hyundaiCanfd`.
- **When armed (v1; v2 2026-10-03 narrows it to CLU11 cancel-only + `0x38D`, see the integration entry):** TX list = common Hyundai set + `{0x38D, 0, 8, .check_relay = true}`, so the stock camera FCA11 is
  **no longer forwarded** bus 2 → 0 (only one FCA11 source on the car bus). Relay-fault safe on this car: full route logs
  show `0x38D` received ONLY on bus 2 (src 2/128). `stock_ecu_check` only trips when a `check_relay` address is
  received on the TX entry's bus (0). tx_hook for `0x38D` allows `CR_VSM_DecCmd <= HYUNDAI_FCA11_TEST_MAX_DEC` (10 raw
  = 0.10 g; C == Python `opendbc/sunnypilot/car/hyundai/values.py`, enforced by a boundary test), plus `CF_VSM_Prefill`,
  `FCA_CmdAct` and `CF_VSM_DecCmdAct`. **Always blocked:** decel above the cap, `CF_VSM_HBACmd != 0`, `FCA_StopReq`, and
  ANY actuation (decel/prefill/act bits) while `vehicle_moving`. Does not depend on `controls_allowed`, since openpilot is
  disengaged during the test.
- **Bit unset:** FCA11 handling is byte-identical to before (the only rule is still "block `CR_VSM_DecCmd != 0`,
  `FCA_CmdAct`, `CF_VSM_DecCmdAct`"; prefill/HBA/StopReq alone pass, as today — pinned by `test_fca11_rule_pinned`).
  In non-SCC mode without the bit, `0x38D` is not in the TX allowlist at all.
- **Caution for the test itself:** while armed, the ESC receives NO camera FCA11 (the camera's real AEB path is cut).
  It is parked-only for that reason too. The FCA11 checksum (`CR_FCA_ChkSum`, byte 7) is CRC8-J1850 init 0xFD
  xor 0xDF (`opendbc/car/hyundai/hyundaican.py: hyundai_checksum`), NOT a byte sum. Panda does not verify it; the sender
  must compute it.
- **Files (opendbc, via patch `openpilot/sunnypilot/fork/patches/0002-hyundai-fca11-brake-test.patch`, opendbc commit
  `ab370764` on branch `fca11-brake-test`, parent `00d48a2f`):** `safety/modes/hyundai.h`, `hyundai_common.h`,
  `hyundai_canfd.h`, `sunnypilot/car/hyundai/values.py`, `safety/tests/test_hyundai.py`, `safety/tests/common.py`
  (`test_tx_hook_on_wrong_safety_mode`: added to the HKG shared-FCA11 group).
- **Verification:** `test_hyundai.py` 1917 passed (base 1762; +155 new); full `safety/tests/` + `test_gas_interceptor.py`
  7972 passed; `test_misra.sh` PASS (the `00d48a2f` base FAILED it on a `knownConditionTrueFalse` in the pedal gate,
  fixed here by the shared predicate + inline suppression). Mutation-checked: removing the cap / HBA / StopReq / moving
  checks, `check_relay=false`, or dropping the gating terms each fails a dedicated test. Rehearsal: pristine opendbc
  `f95f996f` + `0001` + `0002` → tree `9b87a144…` == branch tip tree; panda `74a0adce` built `DEV-74a0adce-DEBUG`,
  `panda_h7.bin.signed` sha256 `61b3165d…e83672` (0001-only build: `64b46c30…`, the deployed one).
- **Deploy note:** adding `0002` to the patches dir CHANGES THE FIRMWARE on the next sync (bit-inert, but a new binary =
  one device reflash). Inert unless bit 64 is set.

### feature: Hyundai comma-pedal (gas interceptor) longitudinal — 2026-10-02 (branch `hyundai-pedal-long`, not deployed)

- **What:** accelerator-only openpilot longitudinal for the non-SCC Elantra N through a comma-pedal-type interceptor
  (TX `0x200` GAS_COMMAND, RX `0x201` GAS_SENSOR). Modeled on the merged Toyota/Honda interceptor pattern, but as a
  **separate panda mode** (`hyundai_gas_interceptor`), NOT `hyundai_longitudinal` (that selects the SCC11/12/13/14 +
  FCA + radar-UDS TX allowlist). TX = base Hyundai set + `0x200` only. Engage = falling edge of SET/RES, exit = CANCEL
  / brake (same button logic as HKG long); the factory conventional cruise is tracked only so openpilot may cancel it.
  Works on a RELEASE panda build (no `ALLOW_DEBUG` dependency; the clamp at `hyundai_common.h` is untouched).
- **Inert by construction — two gates, both required:** (1) capability: `0x201` seen on bus 0 at fingerprinting sets
  `HyundaiFlagsSP.GAS_INTERCEPTOR_DETECTED` (non-SCC ICE only); (2) opt-in: param `HyundaiGasInterceptor=1`. Only then:
  `CP_SP.enableGasInterceptor`, `openpilotLongitudinalControl=True`, `pcmCruise=False`, `minEnableSpeed=25 mph`,
  `safetyParamSP |= GAS_INTERCEPTOR (16)`. Otherwise CarParams are byte-identical to today (unit-tested). The car shows
  0 frames of 0x200/0x201 (141 segments), but the pedal IS on the bus on remapped IDs (0x701), see entry #11.
- **Driver-supervisory contract (there is NO brake actuator):** accel ≥ 0 → throttle via the pedal (capped);
  accel < 0 → lift/coast, engine braking only, never service brakes; no stop-and-go (no engage below 25 mph, pedal
  command cut below ~20.5 mph with a "TAKE CONTROL" warning); downhill the car WILL exceed set speed; it will NOT keep
  distance to a braking lead. **The driver must brake.** This is not ACC.
- **Constants:** `HYUNDAI_GAS_INTERCEPTOR_THRESHOLD = 420` (bench-measured, see below; C `safety/modes/hyundai.h` ==
  Python `sunnypilot/car/hyundai/gas_interceptor.py`, a test parses the C value). Still TBD-BENCH-ROAD:
  `MAX_INTERCEPTOR_GAS = 0.15` (15% travel = mild accel; conservative start, may be raised after road validation);
  `PEDAL_SCALE`/hold-speed offset; `minEnableSpeed`. The cap is deliberately NOT forced below the RX threshold: the
  comma pedal firmware reports the DRIVER's raw ADC on 0x201 and applies max(driver, cmd) only at its DAC
  (`panda c076a9f2^:board/pedal/main.c`), so openpilot cannot self-latch gas_pressed. Bench gate: foot off, ramp the
  command to the cap, GAS_SENSOR must not move (if it does: firmware differs → stop).
- **Bench results (2026-10-03, owner's car parked; CAN chain proven working):** the two pedal tracks relate as
  **B = 0.49413·A + 10.45** (fit over 342,290 real driving pairs; command-range fit n=233,520, median |residual| 3.6).
  Driver sweep: rest A=465 B=241, full press A=2616 B=1284. The ECU flagged its own gas-pressed at commanded A=620 (~7%
  travel); the pedal accepted every coherent commanded pair without fault.
  - **Scaling fix:** the DBC GAS_COMMAND/GAS_COMMAND2 scaling had been copied from Toyota (`(0.159375,-75.555)` /
    `(0.159375,-151.111)` ⇒ B_raw = A_raw + 475), a pair this car never produces: at the 0.15 cap it sent B=1188 where
    the car expects ~400 (3.3×), an APS-correlation-fault risk. Now (512 and 1792): A `(0.11855,-55.126)` → raw
    465..2616 (linear rest..full), B `(0.239915,-57.632)` → raw 240..1303 (= the fitted relation applied to A; the 1303
    vs measured 1284 full-press gap is the fit's endpoint residual). Physical 0..255 = travel fraction. At the cap:
    A=788, B=400. `test_dbc_mapping_matches_bench_measurement` pins the raw output at 0/0.05/0.15/0.25/1.0 and the A/B
    relation (±3) for both dialects; the old Toyota scaling and a one-digit gain flip both fail it.
  - **Threshold basis:** rest average (465+241)/2 = 353, noise ±12 → 420 (rest + 67 ≈ 4% travel, below the ECU's ~7%).
- **Pedal health:** `0x201` RX check enforces the pedal crc8 checksum AND 4-bit counter at 50 Hz (stricter than
  upstream Toyota/Honda): dead / frozen / corrupt pedal → controls_allowed=false in panda and canValid=false in openpilot.
- **DBC:** pedal messages live in a NEW `dbc/generator/hyundai/hyundai_gas_interceptor.dbc` (own parser on bus 0 +
  own packer). NOT appended to `hyundai_can.dbc`: `BO_ 512` is EMS20 there and CANDefine binds `VAL_` by address, so a
  second `BO_ 512` breaks FCEV gear parsing (verified). Generated DBCs are built in-memory at import (gitignored), no
  checked-in artifact.
- **Side effects when active:** ICBM is disabled and its param REMOVED (`openpilot/sunnypilot/selfdrive/car/interfaces.py`
  `_cleanup_unsupported_params`, because `openpilotLongitudinalControl=True`); panda main-cruise toggling is not used
  (acc_main_on stays = factory cruise main lamp); MADS lateral unaffected.
- **Deploy (BLOCKED today):** needs (a) panda firmware rebuild (safety C changed; overlay CI drops submodule changes
  and compiles no C), (b) libparams rebuild for the new `HyundaiGasInterceptor` key in `common/params_keys.h` (until
  then the Python reads it defensively and the feature stays off), (c) no capnp change. `can.h` untouched.
- **Files:** opendbc: `safety/modes/hyundai.h`, `hyundai_common.h`, `hyundai_canfd.h`, `safety/tests/test_hyundai.py`,
  `car/hyundai/{interface,carstate,carcontroller}.py`, `sunnypilot/car/hyundai/{gas_interceptor,carstate_ext,values}.py`,
  `sunnypilot/car/interfaces.py`, `dbc/generator/hyundai/hyundai_gas_interceptor.dbc`,
  `sunnypilot/car/hyundai/tests/test_gas_interceptor.py`. superproject: `openpilot/common/params_keys.h`,
  `openpilot/sunnypilot/selfdrive/car/{interfaces,car_specific}.py`, this file.
- **Verification:** safety tests debug build (test_hyundai + test_hyundai_canfd) and the pedal class against a
  RELEASE-built libsafety; mutation checks (C threshold 500→501, re-enabling EMS16 gas) are caught; car-layer tests.
  Details: `~/.hermes/cache/scratch/car-features/pedal-implementation.md`.
- **Open items:** road validation of the TBD-BENCH-ROAD values (cap, PEDAL_SCALE, hold offset, minEnableSpeed); disengage on pedal STATE fault (parsed, not acted
  on); persistent "no brakes" on-road notice needs a new `EventNameSP` (capnp regen) → follow-up; settings UI toggle
  for `HyundaiGasInterceptor`; CI panda rebuild path.
- **Merge note:** fork-local; do not merge to `main` until bench-validated and the panda-rebuild deploy path exists.

### feature: Hyundai pedal REMAPPED CAN IDs (0x700 TX / 0x701 RX) — 2026-10-02 (amends #10; branch `hyundai-pedal-long`)

- **Why:** the owner's pedal runs CUSTOM firmware whose CAN IDs are moved +0x500, chosen years ago to avoid the
  0x200 = EMS20 address in the Hyundai DBC. Pedal TX (GAS_SENSOR) = `0x701`, commands-in (GAS_COMMAND) = presumed `0x700`.
  #10 only knew 0x200/0x201, so it would never have detected this pedal.
- **Evidence (real route logs, 4 drives):** 0x701 frames match the pedal firmware's exact crc8 (poly 0xD5, init 0xFF,
  bytes 4..0, stored in byte 5) in 400/400 frames, ~0% for every other address; payload = two 16-bit ADC tracks at ~2:1
  that follow the driver's foot (mean 621 with carState.gasPressed vs 472 released); STATE nibble is constantly 5
  (FAULT_TIMEOUT = no valid command ever received); the 4-bit counter steps +1; ~97-101 Hz; on bus 0 (mirrored on 2).
  No Hyundai DBC defines 0x700/0x701, and nothing has ever sent them in sendcan. **0x700 for commands is inferred**
  from the +0x500 pattern and NOT observed (nothing commands the pedal today); bench-verify first: send a zero command on
  0x700 and STATE must drop 5 → 0.
- **What:** both dialects, exactly one active, selected per drive from the fingerprint:
  - DBC: `BO_ 1792 GAS_COMMAND_R` / `BO_ 1793 GAS_SENSOR_R` added to `hyundai_gas_interceptor.dbc`. The signals are
    identical to 512/513 (a test enforces this), and 0x700/0x701 are absent from `hyundai_can` (also tested).
  - Detection: `0x701` on bus 0 sets the new capability flag `HyundaiFlagsSP.GAS_INTERCEPTOR_REMAPPED_DETECTED` (2**12);
    `0x201` still sets `GAS_INTERCEPTOR_DETECTED` (2**11).
  - Selection (`sunnypilot/car/interfaces.py`): optional param `HyundaiGasInterceptorIDSet` = `auto` (default) /
    `standard` / `remapped`. `auto` uses whatever was seen, and prefers standard if both were. A forced dialect whose
    sensor was NOT seen leaves the feature OFF. The remapped choice sets `safetyParamSP |= GAS_INTERCEPTOR_REMAPPED (32)`,
    which is the single source of truth: panda and the car layer (`get_interceptor_ids`) both derive the IDs from it.
  - Safety C: `HYUNDAI_PARAM_SP_GAS_INTERCEPTOR_REMAPPED = 32` (it is only honored together with GAS_INTERCEPTOR).
    TX allowlist = base + `{0x700,0,6}` INSTEAD OF `0x200`. The RX check is the same strict crc+counter 50 Hz check on
    `0x701` INSTEAD OF `0x201`. The interceptor tx_hook guard and gas_pressed follow the active ID. The other dialect's
    command is rejected and its sensor is ignored (tested).
  - Car layer: carstate subscribes only the active sensor (50 Hz, bus 0); carstate_ext reads it; the carcontroller packs
    the active command (`create_gas_interceptor_command(..., msg_name)`, defaults unchanged for Toyota/Honda).
- **RX-check rate note:** the remapped pedal transmits at ~100 Hz, versus the ~49 Hz in the comma firmware source. The
  50 Hz check still passes (the lag check is only a minimum-rate check), and counter +1 per frame is what the firmware does.
- **Inertness unchanged:** with no pedal, or with the param off, CarParams stay byte-identical to today for every ID-set
  value (tested). With the opt-in and the remapped pedal present, the feature WOULD activate on the owner's car, so the
  #10 TBD-BENCH gates still apply before `HyundaiGasInterceptor=1`.
- **Deploy implications:** zero physical work (no reflash, no cable change). The pedal is already on bus 0. The same
  blockers as #10 still apply: a panda firmware rebuild (the safety C changed again), and a libparams rebuild for
  `HyundaiGasInterceptor` + `HyundaiGasInterceptorIDSet`. Both keys are read defensively; a missing ID-set key means
  `auto`. The deploy patch `sunnypilot/fork/patches/0001-hyundai-gas-interceptor.patch` was regenerated.
- **Verification:** safety tests cover both dialects × LDA (debug and RELEASE libsafety), car-layer tests cover
  detection, selection, end-to-end gasPressed and the TX id, and 8 dialect-selection mutations are each caught.
  Details: `~/.hermes/cache/scratch/car-features/pedal-remap-implementation.md`.

### fix: Hyundai pedal low-speed takeover alert only while engaged — 2026-10-03 (amends #10)

- **Bug:** `CarSpecificEventsSP.update()` added `manualRestart` ("TAKE CONTROL / Resume Driving Manually") whenever
  `enableGasInterceptor` was set and `vEgo < minEnableSpeed - GAS_CUT_HYSTERESIS` (~20 mph), with no engagement
  check. With the feature merely ARMED, the alert fired while the driver was driving manually at low speed.
- **Fix:** `update()` takes a new `long_active` argument, and `manualRestart` now requires it. selfdrived passes
  `self.sm['carControl'].longActive`, the same carControl it already gives to `car_events.update`. That is the
  previous frame's carControl, so the alert can lag by one 100 Hz frame. `belowEngageSpeed` is unchanged: it still
  fires only on a `buttonEnable` attempt below `minEnableSpeed`.
- **Files:** `openpilot/sunnypilot/selfdrive/car/car_specific.py`, `openpilot/selfdrive/selfdrived/selfdrived.py`.
  No opendbc / patch-series change.
- **Verification:** new `openpilot/sunnypilot/selfdrive/car/tests/test_car_specific_events.py` (13 cases: armed but not
  engaged, engaged below and above the cut, belowEngageSpeed only on buttonEnable, feature off). Run with the
  `0001-hyundai-gas-interceptor.patch` series applied to opendbc, because the submodule checkout lacks
  `opendbc.sunnypilot.car.hyundai.gas_interceptor`. Removing the gate makes the not-engaged cases fail (mutation caught).

### fix: road-type classifier speed-threshold boundaries — 2026-10-02

- **Symptom:** the owner never saw the HUD road-type indicator on the car. Replaying the
  two pulled drives (8,415 real `liveMapDataSP` samples) proved the feature was running
  and classifying — but three speed thresholds were off-by-a-hair against their own
  stated intent: `HIGHWAY_SPEED_THRESHOLD = 24.6` ("~55 mph"; actual 55 mph = 24.5872
  m/s → true 55-mph roads could never classify highway), `URBAN_SPEED_CEILING = 20.1`
  ("~45 mph"; actual = 20.1168 → the most common US arterial limit fell through to
  `unknown`; 1,251 of the real samples), `HIGHWAY_NAME_SPEED_THRESHOLD = 22.3`
  ("~50 mph"; actual = 22.352 → named 50-mph routes missed the highway branch).
- **Fix:** express all three thresholds as `N * MPH` (55 / 50 / 45 mph) so the
  comparison is inclusive at the posted limit, and change the urban check from
  `< ceiling` to `<= ceiling` (the inclusive value lands exactly on the boundary).
  Logic otherwise unchanged; the 45–55 mph unnamed "ambiguous" zone is preserved.
- **Files:** `openpilot/sunnypilot/mapd/lib/road_type_classifier.py`; boundary tests
  added in `openpilot/sunnypilot/mapd/tests/test_road_type_classifier.py` (8 new pinned
  tests — the original tests never pinned the boundary values, which is why this slipped).
- **Verification:** 24/24 tests pass, ruff clean. Real-data replay (both drives): 1,251
  samples move `unknown → urban` (the 45-mph arterials); 50-mph unnamed stays `unknown`
  (652, ambiguous zone intact); classifier output vs the device's logged values shifts
  from 8,415/8,415 to 7,164/8,415 — exactly the intended fix delta, and itself proof the
  device was running the old thresholds verbatim.
- **Merge note:** fork-local fix to the fork-local feature; PR-candidate alongside it.

### fix: locationd transient single-frame invalidity — 2026-10-01

- **Symptom:** a single skipped camera frame makes modeld publish one `cameraOdometry`
  with `valid=False` (dropped-frame accounting in the modeld publish path). locationd's
  `sm.all_valid()` gate had no hysteresis on the validity side, so that ~50 ms blip
  flipped `deviceMotion.inputsOK=False` for one frame → `locationdTemporaryError`
  (NO_ENTRY + SOFT_DISABLE; full-screen "TAKE CONTROL" that lasted 2.0 s even though
  inputs recovered on the next poll). Occurred twice in ~35 min on 2026-10-01 (route 114
  segs 18/28). MADS lateral assist stayed active throughout (the alarm was louder than
  the problem — but on a stock build the same event would drop steering for ~2 s).
  Known upstream: openpilot #38505 (open), PR #38929 (open, same class).
- **Fix:** `debounce_all_valid()` in `locationd.py` — tolerate a few consecutive invalid
  cameraOdometry polls (`ALL_VALID_INVALID_LIMIT = 3` → ~150 ms at 20 Hz) before
  declaring inputs invalid; any valid poll resets the counter. The counter starts AT the
  limit so startup behavior is unchanged (a service that has never arrived still reads
  bad from the first poll). Mirrors the file's existing `INPUT_INVALID_LIMIT` counter
  style.
- **Files:** `openpilot/selfdrive/locationd/locationd.py`,
  `openpilot/selfdrive/locationd/test/test_locationd.py` (new).
- **Verification:** 8 unit tests (limit−1 tolerated / limit flips / valid resets /
  alternating never accumulates / sustained clears / startup-seeded reads bad), ruff
  clean.
- **Merge note:** upstream-PR candidate. When upstream's own fix merges, take upstream's.

### fix: audio boot-race survival (micd/soundd) — 2026-10-01

- **Symptom:** cold-booting the device with the car ON raced audio bring-up. micd/soundd
  spent a fixed ~30 s `@retry` budget against a device that wasn't ready (`ALSA: No
  soundcards found` at t≈2.8 s; codec/ADSP ready ≈t=8 s), crashed on exhaustion, and —
  because the manager never restarts crashed non-daemons — stayed dead for the whole
  drive. selfdrived then reported `process_not_running {soundd, micd}` → SOFT_DISABLE +
  NO_ENTRY (blocks engagement). Car-OFF boots were immune.
- **Fix:** audio unavailability is an operational state, not an exception.
  `open_audio_stream()` (micd.py) retries at a low rate indefinitely, logs the REAL
  PortAudio error (rate-limited), starts the stream (**sounddevice streams open
  stopped** — the original `with stream:` did the start; without an explicit `start()`
  the daemons open streams that never play and loop "inactive"), and closes partial
  streams on failed starts. micd: acquire → publish → close + re-acquire on loss (0.5 s
  cooldown bounds retry rate). soundd: same via `AudioStreamSlot` (lock-guarded single
  slot, background acquire; the 20 Hz alert loop never stops or blocks; the old
  `assert stream.active` crash path is gone). `common/utils.retry` now chains the
  underlying cause (`raise ... from e`).
- **Files:** `openpilot/system/micd.py`, `openpilot/selfdrive/ui/soundd.py`,
  `openpilot/common/utils.py`, `openpilot/system/tests/test_micd.py`,
  `openpilot/selfdrive/ui/tests/test_soundd.py`.
- **Verification:** 8 + 9 unit tests, ruff clean; two independent review rounds
  (must-fixes applied: slot race, close() lifecycle, acquiring-flag reset, acquire-path
  tests; re-review verdict: approve).
- **Merge note:** upstream-PR candidate (generic fix). Conflicts possible with upstream
  micd/soundd work — preserve the property that the daemons never exit on audio
  unavailability.

### feature: road type classification + HUD indicator — 2026-10-01

- **What:** `LiveMapDataSP.roadType` enum field (custom.capnp) + classifier in
  `openpilot/sunnypilot/mapd/lib/road_type_classifier.py` (name/speed based: interstate /
  highway / urban / unknown), wired through `base_map_data.py` / `osm_map_data.py`; HUD
  shield icon in `openpilot/selfdrive/ui/sunnypilot/onroad/road_type.py` +
  `hud_renderer.py` hook. Commits `17380f6ddd`, `d332b9a840`.
- **Note:** pure Python + one capnp field; no C++ consumer (deploy via overlay keeps
  working without a recompile; capnp-gen still regenerated by CI).
- **Verification status:** VERIFIED 2026-10-02. Log replay (both drives, 8,415 samples)
  reproduced the device's roadType output exactly; the HUD wiring was reviewed (renders
  for any non-unknown value); and the local replay rig recorded the UI drawing the
  interstate shield at the Calumet Expressway stretch of route 114 (shield pixels at the
  expected position, correct #BF2033/#003399 colors). The owner hadn't seen it because
  the classifier's 45/55 mph thresholds misfiled common arterials (fixed — see entry
  above) and most driving was surface streets. Proof artifacts:
  `~/.hermes/cache/scratch/comma-route-114/ui-record.mp4` + `shield-zoom-4x.png`.
- **Merge note:** fork-local feature; PR-candidate if it proves out.

### ci: automated upstream sync + prebuilt rebuild — 2026-10-01

- **What:** `.github/workflows/sync-upstream.yaml`. Triggers: push to `main`, daily cron
  09:17 UTC, manual dispatch. Merges upstream's prebuilt `dev`'s recorded "master commit"
  into main; overlays the fork diff onto a fresh clone of upstream's prebuilt dev tree;
  regenerates capnp when the schema changed; force-pushes `dev`; verifies both refs.
  Commits `49a2233f09`, `223e90224c` (identity fix).
- **Gotchas (in workflow comments):** prebuilt clone is a separate repo — commit needs
  explicit `-c user.name/-c user.email`; capnp needs its BIN_DIR on PATH (plugin skew);
  shallow checkout + unshallow-main instead of depth 0.
- **Merge note:** intentionally fork-local tooling.

### fork: Elantra N vehicle specs + cargo-allowance fix — 2026-10-01

- **Why:** the car fingerprints as `HYUNDAI_ELANTRA_2022_NON_SCC`, which upstream
  configures with inherited base-Elantra specs (2800 lb, steerRatio 12.9). The N is a
  different car: ~3296 lb curb, 12.2:1 rack.
- **What:** `openpilot/sunnypilot/fork/vehicle_specs.py` — `FORK_VEHICLE_SPECS` keyed by
  fingerprint; `apply_fork_vehicle_specs()` wired via `set_car_specific_params()`
  (runs BEFORE CarParams is persisted, so every consumer sees the same values).
  Commits `973ddde741`, `d73288046f`.
- **Gotcha documented in code:** upstream convention is curb + `STD_CARGO_KG` (136 kg
  assumed payload) added in `get_params()`; the hook runs AFTER that, so the override
  must re-add the allowance (`mass + STD_CARGO_KG`) or the car is modeled 136 kg light.
  (This also explained the old "mystery" reading of 1406 kg = 2800 lb + 136 kg.)
  Fixed in `d73288046f`; regression test pins the allowance.
- **Verification:** end-to-end through the real `set_car_specific_params` (2800→3296 lb,
  12.9→12.2; other platforms untouched); unit tests.
- **Merge note:** intentionally fork-local (car-specific numbers for this owner).
  Upstream alternative if ever wanted: PR the two fields into the existing platform.

---

## Sync mechanics notes

- Upstream base this fork currently tracks: `a5f44653d7` (upstream master) / `0b2c431d`
  (upstream prebuilt dev, "built from" a5f44653d7).
- The sync workflow merges upstream into `main` when upstream advances; conflicts are
  resolved once as normal git merges (fork-only files — e.g. `openpilot/sunnypilot/fork/`,
  this file — should never conflict; keep fork-only code in fork-only paths when possible).
- Device state as of 2026-10-02: commit `6611e937` ("sunnypilot prebuilt (fork main)"),
  branch `dev`, origin = this fork.
