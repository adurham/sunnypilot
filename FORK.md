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

---

## Entries

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
