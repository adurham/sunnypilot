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

---

## Entries

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
- **Verification status:** the owner has NOT observed the indicator on-road yet.
  Verification against pulled route logs + HUD wiring is in progress — see scratch
  reports `comma-route-114/roadtype-log-evidence.md` and `roadtype-hud-wiring.md`.
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
