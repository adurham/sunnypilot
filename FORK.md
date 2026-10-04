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

---

## Entries

### integration-3: integration-2 + opendbc 0009 timed factory-cruise cancel — 2026-10-04 (one firmware build; offline-tested, NOT road-validated)

> **Driver notes (read before the next drive):** everything in the integration-2 notes below still applies.
> - **New backstop, should rarely fire.** If the factory cruise ever becomes *active* while openpilot long is engaged
>   (the only case seen in the logs is pressing CC/MAIN while engaged), openpilot long drops immediately (as before) and
>   panda tries to cancel the factory cruise itself with a short pause/resume press on the cluster's behalf.
> - **If the factory cruise is still on 2.5 s later you get an audible "Cruise Fault"** (immediate disable) and the
>   alert stays while factory cruise is on. Then **press CC off or brake** — the factory cruise is in control.
> - **Whether the ECM accepts panda's cancel is unproven until the road test.** Do not rely on it; keep CC/MAIN off.

- **What:** opendbc **`0009`** = `timed-cancel` fbc82f9c (written on pedal-buttons-v2 24ba04bb) cherry-picked onto
  `integration-2` cc592f1c as opendbc branch `integration-3` (032ded0d), plus one integration test. The car-features
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
  `panda_h7.bin.signed` sha256 `53700ef3…23eb` (109316 B), rebuild and a clean `git archive` build byte-identical.
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
  Panda `74a0adce` DEBUG build: `panda_h7.bin.signed` sha256 `1d654e95…9d6d55` (108668 B), rebuild byte-identical.
  Rolling/parked runners checked against the firmware constants: match, unchanged.
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
