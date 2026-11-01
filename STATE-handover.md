# STATE — lane A (fix/blip-handover): gas-interceptor hand-over seed

Brief: `~/comma-sim/wsSTOP/drive-185/LANE-A-BRIEF-handover.md`
Worktree: `~/comma-sim/trees/blip-handover`, branch `fix/blip-handover`, base `db6861e6af`.

## CP1 — recon (done)

- Worktree clean at `db6861e6af`; `opendbc_repo` is a flattened tree whose own HEAD == `db6861e6af` (same commit id), no submodule.
- Defect site confirmed: `opendbc_repo/opendbc/sunnypilot/car/hyundai/gas_interceptor.py::GasInterceptorCarController.create_gas_command`,
  rise limiter `rate_limit_pedal_command(pedal_target, self.gas, ...)` with `self.gas = 0.` while inactive
  (comment "0 while inactive, so every engagement ramps from 0").
- Relevant helpers (unchanged by this lane): `get_hold_command(v, personality)` (personality-invariant physics FF),
  `get_personality_ceiling(v, personality)`, `get_pedal_rate_up(v, launching)`, `LAUNCH_RAMP_V_MAX = 3.0`,
  `PEDAL_REENTRY_V_MIN = LOW_SPEED_FULL_GAS_SPEED` (25 mph) soft-recovery gate.
- Patch series: `openpilot/sunnypilot/fork/patches/0001..0014`; 7 of them mention `gas_interceptor.py`
  (0002 creates it; 0003/0006/0010/0012/0013 modify it). They are a SEQUENTIAL series (0003 has `--- a/opendbc/.../gas_interceptor.py`
  as an existing file → it cannot apply to the pristine pin alone). Last patch in the series: `0014-...-two-finger-tap`.
- Environment note: venv `~/repos/sunnypilot/.venv` has an editable `opendbc` pointing at `~/repos/sunnypilot/opendbc_repo`,
  so the worktree copy must be forced first on `PYTHONPATH`.

## CP2 — change in `gas_interceptor.py` (done, uncommitted)

`GasInterceptorCarController`, one place, exactly the brief's rule:
- `__init__`: new `self._was_active = False`.
- `create_gas_command`: `hand_over = active and not self._was_active` right after `active = CC.longActive` (plus the
  route-185 measured table as a comment block).
- In the law branch, immediately BEFORE `rate_limit_pedal_command(...)`:
  `if hand_over and not self.gas_launching: self.gas = min(pedal_target, get_hold_command(v, personality), get_personality_ceiling(v, personality))`
  (applies to both the delivery-law and the deployed-law target, since both converge on the one `pedal_target`).
- `self._was_active = active` at the very end of the call (after the send slots, on every return path).
- Unchanged: every inactive frame still `self.gas = 0.` + the all-zero ENABLE=0 send; the over-speed release / FCA11
  brake / cal hold / cam-brake paths return before the seed, so they stay exactly 0 (their frames do CONSUME the
  hand-over - see "brief vs code" in the report); the launch latch still skips the seed.
- Old block comment "0 while inactive, so every engagement ramps from 0" replaced with the seed description + the
  route-185 numbers.

## CP3 — tests appended (`TestHandover185`, 5 tests) (done)

`opendbc_repo/opendbc/sunnypilot/car/hyundai/tests/test_gas_interceptor.py`, new class `TestHandover185` at EOF:
test_handover_seeds_at_hold_9ms, test_handover_negative_ask_not_seeded, test_handover_launch_unchanged,
test_handover_after_disengage, test_inactive_still_sends_zero (real path: CarInterface.update -> apply -> 0x700).
3 EXISTING tests pinned the 0-start and had to be updated (they encode the defect): their open-loop models now start
at the same seed - TestPedalFaultClearEndToEnd::test_engage_ramps_then_lift_is_immediate,
TestBlipSmoothing17c::test_deliberate_ask_keeps_jerk_rate, TestBlipSmoothing17c::test_launch_and_low_speed_unchanged.

Run (two-line rule: PYTHONPATH line, then the pytest line):
  PYTHONPATH=/Users/adam.durham/comma-sim/trees/blip-handover/opendbc_repo
  /Users/adam.durham/repos/sunnypilot/.venv/bin/python3 -m pytest opendbc_repo/opendbc/sunnypilot/car/hyundai/tests/test_gas_interceptor.py -q
Result: **2 failed, 188 passed** - the 2 are PRE-EXISTING on base db6861e6af (baseline before any edit: 2 failed,
183 passed): TestFca11LongControllerIntegration::test_mirror_frame_bytes and ::test_personality_reaches_gas_command
(both "assert 16 == 20" in the FCA11 decel ramp, unrelated to the pedal law).

## CP4 — fail-before (test 6, done)

Seed block removed from `gas_interceptor.py` (tests kept), file restored afterwards (md5 `db679a9a0f2da1fafe92911a32cfcec3`
before and after).

  PYTHONPATH=/Users/adam.durham/comma-sim/trees/blip-handover/opendbc_repo
  /Users/adam.durham/repos/sunnypilot/.venv/bin/python3 -m pytest opendbc_repo/opendbc/sunnypilot/car/hyundai/tests/test_gas_interceptor.py -q -k Handover185

REAL OUTPUT (seed removed):
```
E     AssertionError: (0.0028000000000000004, 0.084, 0.0028000000000000004)      # test 1: gas[0], seed, step
E     assert 0.084 <= 0.0028000000000000004
E     AssertionError: (0.0008900000000000002, 0.08900000000000001, 0.0033000000000000004)  # test 4: gas[0], seed, step
E     assert 0.08900000000000001 <= 0.0008900000000000002
FAILED ...::TestHandover185::test_handover_seeds_at_hold_9ms
FAILED ...::TestHandover185::test_handover_after_disengage
2 failed, 3 passed, 185 deselected in 0.20s
```
=> exactly tests 1 and 4 fail; tests 2 (negative ask), 3 (launch) and 5 (inactive zero) pass. Full file with the seed
removed: `7 failed, 183 passed` = the 2 above + the 3 pre-existing-pin tests whose models assume the seed + the 2
pre-existing base failures.
With the seed restored: `2 failed, 188 passed` (only the 2 pre-existing base failures).

## CP5 — patch series (done, per the owner's correction: NEW patch, no earlier patch regenerated)

  openpilot/sunnypilot/fork/patches/0015-fork-hyundai-gas-handover-seed.patch   (267 lines, 2 files)

Generated exactly like 0014: `git diff --relative` from `opendbc_repo/` (that is why the paths are `a/opendbc/...`,
`index <10hex>..<10hex> 100644`, no commit header), scoped to the two files this lane touches.

Verification (scratch copy of the opendbc tree at HEAD = post-0014, i.e. the deployed build, which is what 0015 is a
diff against):
```
git archive HEAD opendbc_repo | tar -x -C /tmp/scratch185                      # post-0014 tree
cd /tmp/scratch185/opendbc_repo && git apply --check -p1 .../0015-...patch     # rc=0 (clean, "Checking patch" x2)
git apply -p1 .../0015-...patch
md5 opendbc/sunnypilot/car/hyundai/gas_interceptor.py                 -> db679a9a0f2da1fafe92911a32cfcec3
md5 <worktree>/opendbc/sunnypilot/car/hyundai/gas_interceptor.py      -> db679a9a0f2da1fafe92911a32cfcec3   (identical)
md5 opendbc/sunnypilot/car/hyundai/tests/test_gas_interceptor.py      -> a57de84db734640cdf9c90379fe1447c
md5 <worktree>/.../tests/test_gas_interceptor.py                      -> a57de84db734640cdf9c90379fe1447c   (identical)
```
`openpilot/sunnypilot/fork/tests/check_patch_numbers.py` -> rc=0 (14 numbered patches, prefixes unique; there is no
0004, so 0015 is the 14th).
Also checked: `git apply --check` of `0001-...` on the post-0014 tree FAILS ("patch does not apply") - the series is
sequential (each patch is relative to the state left by its predecessors), it is NOT a set of pin-relative diffs.

## CP6 — safety (done)

  PYTHONPATH=/Users/adam.durham/comma-sim/trees/blip-handover/opendbc_repo
  /Users/adam.durham/repos/sunnypilot/.venv/bin/python3 -m pytest opendbc_repo/opendbc/safety/tests/test_hyundai.py -q
=> **36 failed, 3075 passed, 384 skipped, 3110 subtests passed** (7.6 s)
All 36 are `test_ignore_frequency_check` (one per parametrised class) and they fail with
`AttributeError: get_ignore_frequency_check_count` from the prebuilt `tests/libsafety` cffi module - a stale compiled
libsafety in this tree, not a rule regression. A/B proof: the SAME command against a pristine `git archive HEAD` copy of
opendbc_repo (no lane changes) gives the identical **36 failed, 3075 passed, 384 skipped, 3110 subtests passed**.
Interceptor-specific subset (`-k Interceptor`, includes TestHyundaiNonSCCGasInterceptorSafety + TestHyundaiFca11LongSafety):
**4 failed (same test_ignore_frequency_check), 516 passed, 93 skipped, 302 subtests passed**. No separate
gas-interceptor safety test file exists (`tests/gas_interceptor_common.py` is the shared base class).

Panda rules that touch a GAS_COMMAND frame (safety/modes/hyundai.h:1277+, safety/longitudinal.h:31):
1. non-zero command requires `get_longitudinal_allowed()` (i.e. controls_allowed - and NOT gas_pressed/brake state);
2. magnitude: track A <= 1218, track B <= 612 (= MAX_INTERCEPTOR_GAS 0.35);
3. FCA11-long throttle exclusivity / cam-brake: non-zero refused while braking (zero frames always accepted);
4. address/dialect allowlist + counter/crc (unchanged).
=> **No rule is specific to the first frame after `controls_allowed`** (no slew/rate gate, no first-frame special case).
The seed's first frame is hold_cmd(v) <= 0.16 pedal (A ~810 / B ~400 raw at 25-30 m/s, less lower down), well under the
0.35 ceiling, so the seeded frame is accepted exactly where the old ~0 first frame was.

## Not verified (do not read as done)
- No on-car / log-replay validation of the new first frame (the table in the brief was not re-derived from route 185
  logs here - no log access in this lane).
- The FCA11 `test_mirror_frame_bytes` / `test_personality_reaches_gas_command` failures are pre-existing on base
  (`assert 16 == 20` in the FCA11 decel ramp) and were NOT investigated or fixed by this lane.
- The pristine upstream opendbc pin itself is not present in this worktree (opendbc_repo is a flattened directory in the
  superproject), so "applies cleanly to that pin" was verified at the post-0014 state CI reaches, not at the pin.

## CP7 — commit + evidence (done)

Committed on `fix/blip-handover` only: **427b5325f1** (base db6861e6af), 4 files / +573 −14:
```
 STATE-handover.md                                  | 127 ++++++++++
 .../sunnypilot/car/hyundai/gas_interceptor.py      |  42 +++-
 .../car/hyundai/tests/test_gas_interceptor.py      | 151 +++++++++++-
 .../0015-fork-hyundai-gas-handover-seed.patch      | 267 +++++++++++++++++++++
```

### Where the brief contradicted the code
1. **"the patch that owns it"** - `grep -l gas_interceptor.py` returns SEVEN patches (0002 creates the file,
   0003/0006/0010/0012/0013 modify it). The series is sequential, not pin-relative: `git apply --check 0001` on a
   post-0014 tree fails ("patch does not apply"), because 0003 already assumes 0002's file. So regenerating any one
   patch in place would break the chain; per the owner's correction this lane ADDS `0015`, a diff of the two touched
   files from the post-0014 state.
2. **Test 1's expected value.** The brief's own code rule feeds the seed through `rate_limit_pedal_command`, i.e. the
   first frame is `min(pedal_target, seed + rate_up*DT)`, NOT `min(law, hold, ceiling)`. At v=9 m/s / ask +0.4 the seed
   is 0.084 but the first frame is 0.0868 (one 0.0028 step of the unchanged rise limiter). Implemented the rule as
   written; the test pins `seed <= cmd <= seed + step` and `cmd > 10 * step` (pre-fix 0.0028).
3. **"append to the test file" is not enough.** Three existing tests in that file pin the 0-start as the EXPECTED
   behaviour and go red on the change: `test_engage_ramps_then_lift_is_immediate`,
   `test_deliberate_ask_keeps_jerk_rate`, `test_launch_and_low_speed_unchanged`. Their open-loop models were updated to
   start at the same seed (behaviour above the seed otherwise unchanged); no other existing test needed touching.
4. **The two-line test-command rule with the worked example is not in the brief text.** Used the two-line form:
   a `PYTHONPATH=.../opendbc_repo` line, then the `.../python3 -m pytest ...` line (the venv's editable opendbc
   otherwise wins and the worktree copy would not be under test).
5. **"applies cleanly to the pristine pin"** is unverifiable in this worktree: `opendbc_repo/` is a flattened directory
   in the superproject, no pristine pin exists here. Verified at the post-0014 state that CI reaches immediately before
   0015 (CI applies the series in `LC_ALL=C` filename order, `.github/workflows/sync-upstream.yaml`).
6. **Design edge the brief did not cover:** `hand_over` is consumed on the first ACTIVE frame even if that frame is one
   of the zeroed paths (FCA11 brake / cal hold / cam brake / over-speed release), which is what "reset to False on any
   inactive frame" implies - so if an engagement's first frame is a brake frame, the following pedal ramp still starts
   from 0. That is the conservative direction (it can only ever command LESS pedal) and is outside the brief's tests.

### Unverified
- No route-185 log replay / on-car validation of the new first frame (no log access in this lane; the measured table
  was taken from the brief, not re-derived).
- The 2 pre-existing failures in `test_gas_interceptor.py` (FCA11 `test_mirror_frame_bytes`,
  `test_personality_reaches_gas_command`, both `assert 16 == 20`) predate this lane and were not investigated.
- The 36 `test_ignore_frequency_check` safety failures are a stale prebuilt `libsafety` in this tree (identical on a
  pristine HEAD copy); the safety suite was NOT rebuilt here.
