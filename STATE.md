# Lane A — 17c blip smoothing — STATE

## CP1 worktree + md5 (done)
- `git worktree add -b fix/17c-blip-smoothing /Users/adam.durham/comma-sim/trees/fix17c-blip 096fbddc0a` → HEAD 096fbddc0a.
- md5 in worktree (opendbc_repo/opendbc/sunnypilot/car/hyundai/):
  gas_interceptor.py a274db98b901cc261f9d92153270ea7f ✔, fca11_long.py 087111a1b4027c62d176615bb9e41d55 ✔,
  fca11_shadow_log.py 08ae7c83fd80f681a7e9d3f87bec2566 ✔ — base correct.
- Next: read OWNER-NOTES, REPORT §2 Note 4, gas_interceptor.py, tests, patch 0006.

## CP2 code read (done)
- Shipping form: `opendbc_repo/` in this commit is a FLATTENED, already-patched tree; the fork ships opendbc changes as
  `openpilot/sunnypilot/fork/patches/NNNN-*.patch` (CI `git apply`, LC_ALL=C order). Series: 0001-0003,0005-0009 (no 0004;
  panda/0004 shares the number space). 0006 = over-speed release (gas_interceptor.py only, NO tests shipped).
  → Plan: edit the flattened file + emit patch `0010-fork-hyundai-gas-blip-smoothing.patch` (deployed→fixed diff, incl. tests).
  Lane B STATE also eyes 0010 — collision to be resolved at integration (flagged).
- Baseline test run (unmodified base): `test_gas_interceptor.py + test_gas_interceptor_personality.py` → **159 passed, 2 FAILED**
  (TestFca11LongControllerIntegration::test_mirror_frame_bytes / test_personality_reaches_gas_command: pins 20 LSB, base
  0009 measured inverse gives 16). Pre-existing; the re-pin exists only in ~/repos/sunnypilot-cycle1 (never shipped).
- The route-16a TestOverSpeedRelease class (8 tests) the brief calls "existing" is NOT in the base test file — it lives only in
  ~/repos/sunnypilot-cycle1. Plan: port it verbatim as the regression set (plus adjust the helper-boundary test for hysteresis).
- Mechanism verified at gas_interceptor.py: over_speed_release L248-258 (strict >, no hysteresis); rate_limit_pedal_command
  L261-263 (min(gas, last+rate*DT): instant fall); get_pedal_rate_up L216-225 (0.72/s ≥22 m/s); call site L403-421.

## CP3 design + implementation (done, iterating on evidence)
- Frame-level replay harness (fallback per rules §4; closed-loop attempt pending): ~/comma-sim/wsSTOP/fixes-17c/laneA/
  extract_17c.py (17c.npz, 490 710 carState rows) + replay.py (REAL GasInterceptorCarController.create_gas_command,
  logged CC.longActive/accel, vEgo, vCruise, personality; FCA11 stubbed to the LOGGED 0x38D actuating flag).
  Fidelity (pre-fix tree vs logged 0x700): exact raw 84.1 %, |d|<=2 92.4 %, |d|<=10 98.65 %, ENABLE match 99.96 %.
  Census through the same blip definition: logged 87 pulses / replay-pre 90 (6.5/min) — reproduces.
- Findings vs brief: (a) 19:40:17 (largest blip) is a GAS-OVERRIDE HAND-BACK (driver pedal released 18973.75, longActive
  rises 18973.78), not a release re-entry; (b) over-speed release chatter is real but small: 6 release->re-entry episodes
  (5 shorter than 1 s, all at 19:57:51-54 hovering set+0.45..0.51); (c) 71 of 77 v>15 zero->re-entry episodes are law clips
  (ask < -hold/scale) — p50 zero duration 1.84 s, these are planner-asked coasts.
- Design: (1) release latch: entry = unchanged over_speed_release; hold while v > set + 0.0 and ask < 0.5;
  (2) soft recovery below hold: rise limited to hold(v)/1.0 s while gas_last < hold, ask < 0.5, v >= 25 mph, not launching.
- Variant A (shipping candidate) replay: pulses 90 -> 83, p90 0.159 -> 0.110, ENABLE up 15 -> 10, slots with step>0.010
  268 -> 90; 19:57:51-54 window ENABLE edges 12 -> 1; 19:40:17 0->0.223 in 0.3 s -> 0->0.183 in 1.0 s.
- Variant B (evidence only, soft rate also above hold): pulses 85, step>0.010 slots 0 — NOT shipped (would slow lead/merge
  response above hold; outside brief scope).

## CP4 tests (done)
- Ported route-16a `TestOverSpeedRelease` (8 tests, only existed in ~/repos/sunnypilot-cycle1, never shipped) + new
  `TestBlipSmoothing17c` (8 tests) into test_gas_interceptor.py.
- Fail-before/pass-after (git stash of gas_interceptor.py ONLY; md5 back to deployed a274db98…):
  `-k "Blip or OverSpeed"` → before: **6 failed, 11 passed**; after: **17 passed** (16 + 1 boundary param).
  Fail-before: chatter_at_release_boundary, chatter_through_set_and_boundary_bounded, reentry_from_release_is_soft_and_monotonic,
  reentry_after_law_clip_is_soft, route_16a_setdown_still_releases_promptly_and_coasts_to_set,
  latch_inert_on_positive_deliberate_ask_and_disengage. Pass both (no-regression guards): deliberate_ask_keeps_jerk_rate,
  launch_and_low_speed_unchanged.
- Regression: `test_gas_interceptor.py + test_gas_interceptor_personality.py + test_fca11_shadow.py` → **283 passed, 2 failed**;
  the 2 failures are PRE-EXISTING on the untouched base (TestFca11LongControllerIntegration::test_mirror_frame_bytes,
  ::test_personality_reaches_gas_command — pin 20 LSB, base 0009 inverse gives 16; re-pin only in sunnypilot-cycle1). Base: 159 passed/2 failed on the first two files.
- ruff: 6 findings, all pre-existing (base also 6; I removed one blank-line finding I introduced).

## CP5 replay / closed loop (done)
- Frame replay (laneA/replay.py, real create_gas_command): logged 87 / pre 90 / post 83 pulses; p90 0.159→0.110; ENABLE up
  15→10; slots with up-step >0.010 268→90; 19:57:51-54 ENABLE edges 12→1; 19:40:17 0→0.223 in 0.3 s → 0→0.183 in 1.0 s.
- Closed loop: EXISTING wsH harness (simdrive.py, unmodified) via laneA/cl/cl_run.py, which only supplies CS.out.vCruise
  (harness CarState double predates 0006 and crashes on the DEPLOYED build without it). sp-freeze planner + odbc pre/post
  (git archive HEAD vs worktree), --module freeze --arm off --pers logged --flag-path seam --shakedown, 27 17c windows
  (new windows/0000017c.json generated with the harness's gen_windows_7d.py):
  pulses 41→27 (3.3→2.2/min), max up-step/50 ms 0.073→0.043 (p99 0.0228→0.0091), zero↔live 72→66, v-set mean −0.230→−0.223,
  |v-set|>1 frac 0.207→0.213, |jerk| p99 1.64→1.56, FCA11 eps 74→70, refused 0/0; sim lead collisions: same 6 windows in both
  (harness lead re-base artifacts at pre — none introduced); min dRel per window never worse post.
- Reviewer (consult): no safety regression; flagged PI windup (N/A: ask==aTarget on 99.3 % of v>5 rows — Hyundai PI ki≈0 here),
  and that soft recovery covers the whole sub-hold band, not only post-release (by design — that is where 84 of 87 pulses live;
  stated in report as a choice).

## CP6 packaging (done)
- Patch `openpilot/sunnypilot/fork/patches/0010-fork-hyundai-gas-blip-smoothing.patch` = git diff of the two opendbc files
  (relative to opendbc_repo/); `git apply` onto a clean archive of base opendbc_repo reproduces both files byte-identically
  (md5 6a5be287…). check_patch_numbers.py rc=0. Lane B took 0011 (its STATE.md). FORK.md row 93 (DRAFT).
