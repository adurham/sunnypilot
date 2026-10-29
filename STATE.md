# STATE — integration branch `fix/17c-integrated` (BRIEF I1)

Worktree `/Users/adam.durham/comma-sim/trees/fix17c-integrated`, branch `fix/17c-integrated` off
`096fbddc0ae4a7dac09ee60768fcbdffd48b3aca`. The main clone is READ-ONLY; the other lanes' worktrees are untouched.
Merge order A -> B -> C. This file is the integration log; the per-lane records are kept verbatim below.



---

# Lane A record (from `fix/17c-blip-smoothing` @ 684ae094d4, kept verbatim)

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

---

# Lane B record (from `fix/17c-standstill-hold` @ 577406fe62, kept verbatim)

# STATE — lane B (fix/17c-standstill-hold)

## CP1 — worktree + md5 (2026-10-29)
- `git worktree add -b fix/17c-standstill-hold ~/comma-sim/trees/fix17c-hold 096fbddc0a` — OK, HEAD 096fbddc0a.
- md5 inside worktree (opendbc_repo/opendbc/sunnypilot/car/hyundai/):
  - gas_interceptor.py   a274db98b901cc261f9d92153270ea7f  == deployed ✔
  - fca11_long.py        087111a1b4027c62d176615bb9e41d55  == deployed ✔
  - fca11_shadow_log.py  08ae7c83fd80f681a7e9d3f87bec2566  == deployed ✔
- Read REPORT.md §2 Note 1, logs/cyl-pres.txt. `logs/pipe*` named in brief DOES NOT EXIST (no such files in drive-17c/logs).
- Patch series: opendbc_repo/ in this commit is a flattened tree (no submodule); fork patches at
  openpilot/sunnypilot/fork/patches/0001..0009 (no 0004). Next free: 0010 (lanes A/C may also claim it — flagged).
- Next: map fca11_long.py hold paths.

## CP2 — law-path map (fca11_long.py @ 096fbddc, md5 087111a1…)
- Actuating instance: `self.feel = FCA11_FEEL_ENABLED(True) and enabled and not affine_gain` (:1238); affine_gain is
  forced False in Shadow (:1136) and the 17c drive ran Shadow (header note "wire = OLD-LAW"). => the WIRE ran the
  **FEEL path**, `_update_law` :2396 -> `_feel_level` :1832. The SHAPED path (`_shape_level` :1722, target = cap in band)
  is ONLY the private V3 shadow core (logged as v3_*), never on the wire.
- Hold-band branch that owns the deployed hold: `_feel_level` :1840-1856:
    dec = clamp_dec_cmd(dec_cmd_from_accel(min(accel,-0.5)), personality)   # :1848  tier clamp #1
    dec = max(dec, min(fca11_floor_lsb(v), tier cap))                       # :1851  (meas_inverse) 8 LSB band floor
    dec = clamp_dec_cmd(min(dec, last_sent+RATE_STEP), personality)         # :1852  tier clamp #2 (would clip a raised hold)
  Band latch `_feel_update_band` :1824 (ENTER <=3 km/h or CS.standstill, EXIT >=5 km/h), updated every frame at :2324.
- Why 20: at standstill LongControl's stopping ramp drives the ask to -2.0 (= FCA11_A_MAX_MSS2 PID clip); the deployed
  request map gives round(2.0/0.67/9.81*100)=30 -> tier clamp -> 20 (standard). Verified on stop-2 shadow rows
  (mono 16650-16676): ask -0.97 -> -2.0 at -1/s, cmd 8,11,14,17,20 then 20 x 891 frames; standstill flips False
  16664.0-16664.8 during the creep (v peak 0.165 m/s) — so a standstill-ONLY gate would DROP the hold mid-creep.
- Alerts: `_alerts` :1360 (hold_lost v>2 km/h, no_response moving>5 km/h) read dec_cmd only — no level coupling.
- Panda (hyundai.h :1115/:1154): only CR_VSM_DecCmd > 30 and > last_accepted+4 are refused — 30 is the hard gate.
- Evidence check: 891 production episodes across all 7 pulls on disk: dec_cmd_max never > 20 (aggressive never driven
  in production FCA11-long). 30 LSB = 0.30 g WAS exercised only in the rolling test-mode scaling runs (0.6 s pulses
  ~31 km/h, 2.03 m/s^2 extra delivered). Never at standstill.
- Baseline `test_fca11_long.py` on the UNMODIFIED base: **7 failed / 171 passed** (+44 subtests) — pre-existing (0009's
  measured inverse makes the moving -2.0 ask land at 16, tests pin the tier cap). List in .baseline-test_fca11_long.txt.
- Patch numbering: lane A claims 0010 (fix17c-blip STATE.md). Lane B will use **0011** to avoid the collision.
- Next: design (latched standstill hold floor in the FEEL hold branch).

## CP3 — design (consulted reference model; ruling adopted with one deviation, see report)
- Feel hold branch only. Latch `_feel_standstill_hold`: SET when the band branch runs with CS.standstill; CLEARED on band
  EXIT (>=5 km/h, unchanged), `_feel_reset` (every episode open, longActive off, disable, cal hand-off).
- While latched both band tier clamps use ceiling max(tier cap, FCA11_STANDSTILL_HOLD_LSB=30); limiter unchanged.
- All tiers -> 30 at standstill. Kill switch FCA11_STANDSTILL_HOLD_ENABLED. Accessor clamps to panda gate 30.
- Reviewer suggested a 0.3-0.5 s standstill-persistence arm; NOT needed: on every 17c stop the ask is still -0.85 at the
  first standstill frame and the planner ramp (-1 m/s^3) is what climbs the request, so 20->30 lands 0.5-1.1 s AFTER
  standstill at +1 LSB/sent frame (replay below). A cold -2.0 at standstill still ramps at the panda +4/sent.
- Reviewer's refusal-desync concern checked: observe_echo (:1537) already zeroes _dec_last_sent on a refused actuating
  echo -> ramp restarts at 4 and climbs back (pinned by a new test).

## CP4 — implementation (fca11_long.py md5 087111a1… -> 31385a85…)
- +FCA11_STANDSTILL_HOLD_ENABLED/_LSB + fca11_standstill_hold_lsb(); +standstill_hold flag/_feel_standstill_hold latch;
  `_feel_band_clamp`; `_feel_level(..., standstill=)`; call site passes CS.out.standstill.

## CP5 — tests
- New: test_fca11_long.py::Test17cStandstillHold (13 tests).
  Fail-before (source stashed, md5 087111a1 verified): 9 failed / 4 passed. Pass-after: 13 passed.
- test_fca11_long.py full: base 7 failed/171 passed -> 7 failed/184 passed (+44 subtests both); failing set IDENTICAL
  (pre-existing, 0009 measured inverse vs tier-cap pins).
- test_fca11_feel.py: base 6 failed/24 -> fix (unedited test) 7 failed/23: `test_hold_band_runs_the_deployed_law_byte_identical`
  pinned feel-band == deployed-band at standstill with ask -2.0 — exactly the intended change. Test amended to run the
  identity with the switch OFF everywhere and ON everywhere except (standstill AND ask past the cap). After: 6 failed/24
  (same 6 pre-existing). Amended test FAILS on the deployed source (KeyError-free AttributeError on the new switch).
- Unchanged pass/fail counts base vs fix: feel_fix 17f/18p, crawl_dwell 12p, shadow 108p, shaping_param 9p,
  cal_level 10p, gas_interceptor 2f/142p, gas_interceptor_personality 17p.

## CP6 — frame-level replay (real Fca11LongBrake, logged 17c shadow inputs incl. real camera bytes)
- Script: ~/comma-sim/wsSTOP/fixes-17c/laneB-replay/replay_stop2.py; outputs stop2-replay.txt, whole-drive-replay.txt.
- Stop 2 (mono 16654-16672.4, warm-start 16640): deployed replay == logged wire 921/921. 17c differs on 888 frames,
  all with the latch set; 20 -> 30 between 16654.63 and 16655.23 (+1 LSB per sent frame), 30 through the creep
  (16662.8-16665.0) incl. the standstill=False frames; release at 16672.38 identical (0).
- Whole shadow .5 (25 517 send rows): deployed replay == wire 98.77%; all 315 mismatches are MOVING (0 in band, 0 at
  standstill: 1-frame 4-vs-0 opens + two runs of 5-vs-4/7-vs-4 at 15306/15626 — lead-state the shadow does not carry);
  17c differs on 4 412 rows, 0 outside the latch. Shadow .4: 0 differences. .2/.jsonl carry no frame rows.
- Closed-loop harness (wsH simdrive + plant_v4): CANNOT reproduce the code path's purpose — plant2/plant_v4 floor v at 0
  with a<0 clamp (`if self.v <= 0 and a_ss < 0: a_ss = 0`), i.e. no DCT creep model and no static brake hold. It would
  only show 20 vs 30 on the wire with zero motion either way. Not run; frame-level replay is the accepted fallback.

## CP7 — packaging
- Patch openpilot/sunnypilot/fork/patches/0011-fork-hyundai-fca11-standstill-hold.patch (fca11_long.py + 2 tests).
  Applied with plain `git apply` to the deployed opendbc tree: result md5 31385a85… == worktree; tests identical.
  check_patch_numbers.py exit 0; test_patch_number_guard.py 9 passed.
- FORK.md draft row 93.
- Next: commit, push-if-auth, report.
- CP8: committed 577406fe62; pushed to adurham/fix/17c-standstill-hold (see ls-remote).

---

# Integration checkpoints

## I-CP0 — worktree + base proof
- `git worktree add -b fix/17c-integrated .../fix17c-integrated 096fbddc0a` — OK.
- md5 inside the worktree == EVIDENCE hashes: gas_interceptor.py a274db98…, fca11_long.py 087111a1…,
  fca11_shadow_log.py 08ae7c83… — base correct.
- Merge A (684ae094d4) clean. Merge B (577406fe62) conflicted on FORK.md (A and B both drafted row 93) and STATE.md
  (both created it) — resolved: A = row 93, B = row 94, both lane STATE records kept below.
