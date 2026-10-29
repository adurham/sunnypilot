# STATE — lane C (fix/17c-planner-commitment)

## CP1 worktree + md5 (done)
- `git worktree add -b fix/17c-planner-commitment ~/comma-sim/trees/fix17c-planner 096fbddc0a` → HEAD 096fbddc0a.
- md5 gas_interceptor.py a274db98…f7 ✔, fca11_long.py 087111a1…41d55 ✔, fca11_shadow_log.py 08ae7c83…bec2566 ✔ (all match deployed).
- Next: read REPORT §2 Notes 2+3, §6; map planner layering; inspect prior branches.

## CP2 planner layering + prior branches (done)
- Published aTarget = longitudinal_planner.py:278-280 `min(candidates)` over (MPC lead0/1, cruise a_cruise, e2e) then clip.
  e2e candidate chain (deployed): modelV2.action.desiredAcceleration (:253) -> update_e2e_gate raise-only (:256, cycle-1..3
  far-lead gate, STILL IN THE DEPLOYED BASE incl. +0.4 cap) -> G4 turn limit min() (:265) -> AccelBoost (:272) -> min().
  Fork SP layer (sunnypilot/.../longitudinal_planner.py update_targets) only picks v_cruise (cruise/SCC-V/SCC-M/SLA) — no aTarget shaping.
  Shipping form: planner is plain Python in the superproject (NOT the opendbc patch series; patches/ = opendbc only).
- Prior branches (none is an ancestor of 096fbddc; base is a squashed prebuilt):
  cycle1-early-brake = gate + lead-decel veto + median ENTER — ALREADY IN BASE (base has more: closing veto, latch, cap).
  cycle2-alerts-setspeed = closing-rate veto (in base) + fca11 0006-0009 (opendbc, not planner).
  gate-delete = DELETES the gate (FORK row 93) + passthrough checker — NOT in base (deployed still runs the gate).
  esc-standstill-fix = old ESC/pedal work, no planner shaping relevant.
  -> None contains stop-approach / commitment shaping. Reuse: the passthrough-invariant + checker idea (gate-delete), the
     "lift = aTarget - model_raw - boost" measurement, and the census-before-mechanism rule.
- Extract: ~/comma-sim/wsSTOP/fixes-17c/laneC/extract17c.py -> r17c.npz (96,712 plan rows 15280-20120, 100% model-paired).
- Window (a) mono 16884-16903: no lead after 16889.6 (brief lead-less claim true only after 16889.6; 16885-16889 had a flickering
  far lead 60-100 m). ask -0.13..-0.5 to 17 m/s, then -0.6..-0.9 to 12.9, -1.0..-1.7 to 7.4; driver brake at ~16903.6.
- Window (b) real location mono 17546-17557 (19:15:48 wall): vRel noisy -1.0..-2.9 @ ~100 m for ~7 s, then -4.2 -> -9.6 -> -12.3
  in ~1.6 s (17553.9-17555.5). Ask -0.59 -> -1.04 -> -1.43 -> -1.71 tracks that collapse with ~0.5 s lag.
  Model's own 10-s trajectory min speed was already 10-13 m/s (from 21.5) during the "flat" part.
- Next: open-loop real-planner replay through wsH simdrive --mode ol_planner (MPC candidate not logged) ; design doc.

## CP3 harness bring-up on this build (done)
- Existing wsH simdrive.py (UNMODIFIED) runs against this worktree via ~/comma-sim/wsSTOP/fixes-17c/laneC/run_simdrive.py
  (launcher adapters, plumbing only: aarch64 Cython .so placeholder -> harness mpc_shim; DEC readout absent in schema;
  personality_scale absent; selfdriveState.experimentalMode absent; map-oracle signature (#2064); CS.out.vCruise default 0 =
  over_speed_release inert in sim).
- Fidelity (ol_planner, base): win (a) 0000017c-s28-16884 RMSE(aT_sim - aT_log) 0.000 (max 0.003); win (b) 0000017c-s38-17540
  RMSE 0.029 (max 0.31). Harness is faithful for the planner on 17c.
- Census (offline prototype, census.py / census_kin.py): (i) no-lead commit -> 12 episodes, 9 STOP / 3 DRIVER_BRAKE / 0 NO_STOP;
  with lead allowed 40 eps, 16 NO_STOP -> lead-present excluded. (ii) kinematic lead candidate -> 26 eps, 15 NO_STOP, starts
  AFTER the logged ramp in window (b) -> zero true positives -> NOT shipped (design doc only).
- Second opinion (consult) applied: shape after gate+boost, plan's own v0, lead debounce (not a hard edge), no speed fade in
  the final approach, soft dead-band, absolute vmin condition, shadow fields for census.
- Next: implement (i) behind a param flag (default off) + shadow log fields; tests; full-route ol_planner A/B + closed-loop.

## CP4 implementation + unit tests (done)
- New: openpilot/sunnypilot/fork/stop_commit.py (StopCommit, commit_accel, stop_weight); wired in
  selfdrive/controls/lib/longitudinal_planner.py on the FINAL e2e candidate (after AccelBoost, before min()).
  Param StopApproachCommit registered in common/params_keys.h (BOOL, default "0"). Mac libparams rebuilt from this tree
  (wsP build_libparams.sh) -> laneC/libparams/libparams_c.dylib sha256 5a79bbd1...; round-trip put_bool(block=True) OK.
  (Host gotcha: non-blocking put_bool is async; an immediate get_bool reads False — use block=True in sims.)
- Tests: openpilot/sunnypilot/fork/tests/test_stop_commit.py 20 passed; fail-before (source moved away) = collection
  error (ImportError) -> all 20 fail; pass-after 20 passed.
## CP5 replay, open loop (done)
- wsH simdrive --mode ol_planner, all 27 engaged windows of 17c (3326.9 s), flag OFF vs ON through the real Params:
  fidelity OFF vs logged aT RMSE 0.0320 over 66,550 rows. Changed rows 582 (0.87 %), POSITIVE changes 0.
  Episodes 6: 3 STOP, 3 DRIVER_BRAKE, 0 NO_STOP. 23/27 windows byte-identical (max|d| 0.0000).
- Window (a) 0000017c-s28-16884 (rem = logged distance to the driver's standstill): v<=17: ask -0.61 -> -0.97 (req -1.07);
  v<=15: -0.77 -> -1.14 (req -1.11); v<=13: -0.88 -> -1.17 (req -1.22); at takeover v7.39 rem 14.4 m: -1.68 both (req -1.89).
- Window (b) 0000017c-s38-17540: 0 changed rows (lead present -> never arms) — (ii) not implemented, by design.
- Next: closed-loop A/B (running), base-source equivalence run, design doc, FORK row, commit.

## CP6 closed loop + equivalence + tests (done)
- Equivalence: flag-OFF branch vs UNMODIFIED 096fbddc source (planner + params_keys stashed), same harness, ol_planner,
  27 windows: 0 differences over 69,190 rows (aTarget, v, ask, gas, held_lsb, src, a) + every sent frame (equiv.py).
- Closed loop (plant v3) ON vs OFF: 23/27 windows identical. Changed: s28-16884 (counterfactual planner standstill
  x 298.9 ON vs 389.0 OFF; logged driver standstill 307.6; OFF arm already +47 m vs log at press -> delta only),
  s22-16571 (4524.8 vs 4539.1; logged 4528.1), s04-15473 (OFF arm sim-collision in tail at 162 s after running 42 m
  ahead of the log; ON stops), s72-19569 (<=0.04). Main-row valid fraction 0.29-0.43 on changed windows (plant-limited).
- Over-speed-release precondition (sim vCruise=0 adapter): 74 long-active rows on the whole drive, 0 inside any changed span.
- Tests (pytest_shim.py = harness mpc_shim for the aarch64 .so, no repo file touched):
  test_stop_commit.py 20 passed; fail-before = ImportError (20 fail). test_stop_commit_planner.py 4 passed; fail-before
  (planner wiring stashed) 1 failed / 3 passed (the 3 equivalence tests pass by design on both).
  Regression set: test_stop_commit + test_stop_commit_planner + test_e2e_far_lead_gate + test_adaptive_follow +
  test_param_reachability: 104 passed, 2 failed — both 2 fail IDENTICALLY on the unmodified base (pre-existing:
  check_param_reachability wants 'fca11Shadow' read in opendbc; real-CP Elantra interceptor test) -> not this change.
  ruff: clean on all touched files. check_patch_numbers: rc 0 (no patch touched). check_param_reachability --tree . : ok.
- Docs: LANE-C-DESIGN.md (design + evidence), FORK.md draft row 94 (lanes A/B both took 93 -> renumber at integration).
- Next: commit, push to adurham if auth works, final report.

## CP7 commit + push (done)
- Commit b639f18c17 on fix/17c-planner-commitment; pushed to adurham (ls-remote = b639f18c1780d584d2ee65aa319c943def674429).
- Untracked local-only: openpilot/common/libparams_c.dylib (gitignored; Mac build of this tree's registry).
