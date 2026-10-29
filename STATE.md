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
