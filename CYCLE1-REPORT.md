# CYCLE 1 — far-lead e2e gate (planner only)

Branch `cycle1-early-brake` from `adurham/main` `5e162a6786`. Not pushed, not deployed. Gain, onset
thresholds, release, panda and firmware are untouched. The one brake-law change is the crawl dwell escape (opendbc patch
`0005`, section "Under-brake fix" below).

## Change

`openpilot/selfdrive/controls/lib/longitudinal_planner.py`: `LongitudinalPlanner.update_e2e_gate()` is the brief's
pseudocode verbatim, with the thresholds as module constants `E2E_GATE_*`. It is applied to the **raw**
`modelV2.action.desiredAcceleration` **before** the G4 turn limit. After it, the turn limit → AccelBoost → `min()`
chain runs unchanged.

- `d_des = STOP_DISTANCE + t_follow * v_ego`:
  - `STOP_DISTANCE` is imported from `long_mpc` (6.0 m).
  - `t_follow` is the value passed to `mpc.update()`: the adaptive-follow value when that param is on (it was on in
    167/168), otherwise `get_T_FOLLOW(personality)`.
  - At equal speeds this equals the MPC's `get_safe_obstacle_distance(v, t_follow) − get_stopped_equivalence_factor(v)`
    (unit-tested).
- Gate state (`e2e_gate`, `e2e_gate_cnt`) is held on the planner. A new planner starts un-gated.
- Revert: delete the `update_e2e_gate(...)` call in `update()`. The method and constants are then dead code.

## Acceptance (numbers verbatim from `check_e2e_gate_logs.py`, run 2026-10-23; output in `~/comma-sim/wsSTOP/feel-167-168/cycle1-gate/report.json`)

| # | Criterion | 167 | 168 | Result |
|---|---|---|---|---|
| 1 | Frames where the gate lowers the plan | **0** / 33255 (new ≥ old on 33255) | **0** / 32250 (new ≥ old on 32250) | **PASS** |
| 2a | `shouldStop` changed | 0 | 0 | **PASS** (structural, see note) |
| 2b | First ask ≤ −0.5 ≥ 5 s before standstill, 15 engaged stops | 7/9 (old 7/9) | 3/6 (old 3/6) | **10/15 new = 10/15 old. FAIL as written; not caused by the gate** |
| 3 | Lead-present e2e-sourced onsets with the gate active (the 41 = phantom-hunt 4c episodes where e2e set the ask, 13 + 28) | 11/13 | 20/28 | **31/41 = 75.6 %, FAIL (< 90 %)** |
| 4 | Pull-away follow frames (engaged, lead vRel > 0, MPC > +0.3) where e2e no longer wins the min | 1492/2073 = 72.0 % | 5752/6620 = 86.9 % | **pooled 7244/8693 = 83.3 % PASS; 167 alone FAIL** |
| 5 | Frames with `aTarget < −0.5`, reduction | engaged 2262→2019 = **−10.7 %**; all 4153→3871 = −6.8 % | engaged 2780→2711 = **−2.5 %**; all 3727→3658 = −1.9 % | **FAIL (≥ 40 % required)** |

Thresholds were not tuned. These are the brief's values as written.

Other numbers from the same run:
- Gate-active frames: 4357 on 167 (3478 engaged), 11949 on 168 (10504 engaged).
- Frames where e2e changed: 4110 / 11258. Frames where the plan changed: 3911 / 10825.
- Changed frames that are no-lead, model `shouldStop`, v ≤ 3, or old source mpc/cruise: **0** on both routes.

### Notes per criterion

- **2a.** `output_should_stop` is computed from model `shouldStop`, `should_stop(v, mpc)` and `should_stop(v, cruise)`.
  None of these reads the e2e accel the gate edits, so it cannot change. The gate also never acts while model
  `shouldStop` is set (0 frames).
- **2b.** The 5 stops under 5 s were already under 5 s before the change: 167 15:45:28 is 7.3 s, but 16:07:23 = 1.91,
  16:13:28 = 2.24, 168 16:40:58 = 3.03, 16:43:10 = 1.85, and 16:45:45 has no ask ≤ −0.5 at all.
  - The gate left 12/15 unchanged.
  - It moved 3 later: 167 15:54:28 11.30→10.95 s, 167 16:00:38 28.51→28.16 s, 168 17:15:17 22.17→21.22 s.
  - No stop crossed the 5 s line. Window: the 30 s before standstill, cut at the previous standstill.
- **3. Misses (10):**
  - 4 failed the 10-frame enter count only: 167 15:57:31, 15:57:52; 168 16:37:53, 17:11:40. The last of these has no
    lead in the episode table.
  - The other 6 failed an enter condition because the lead was really near or the MPC was braking:
    - 16:53:12 and 16:53:16: gap 1.02×, MPC −0.35.
    - 16:56:14 and 16:56:42: MPC −0.86.
    - 17:01:35: gap 0.82×, TTC 9.4 s.
    - 17:02:34: gap 1.07×.
  - Of the 41 onsets, 34 now have a plan > −0.3 at the onset frame. That count includes 3 missed onsets whose logged
    ask was already above −0.3 at that frame. 23 of the 41 episodes no longer reach −0.3 at any point.
  - **Ambiguity:** "lead-present e2e-sourced" read literally over ALL phantom-hunt planner episodes gives 93 onsets
    (41 on 167 alone), with the gate active at 42/93 = 45 %. The brief's "41 total" matches the 4c set (13 + 28), so
    that set is the headline.
- **5. Why so little:** most remaining engaged `aTarget < −0.5` frames are outside this gate's scope:
  - 167: source already mpc/cruise 964; MPC < −0.1 792; v ≤ 3 123; no lead 119; gap ≤ 1.2× 9; vRel ≤ −3 7;
    enter count 5.
  - 168: 1048 / 970 / 99 / 560 / 15 / 16 / 3 in the same categories.
  - A 40 % cut would require changing the MPC/cruise candidates, the no-lead e2e frames or the stop approaches. The
    design forbids all three.

## Harness fidelity / limits

The replay is open-loop: inputs come from each logged frame, and the car's response to a changed plan is not
simulated.
- The MPC candidate rebuilt from `longitudinalPlan.speeds/accels` matches `aTarget` on lead0 frames: p99 |Δ| 4e-6 /
  5e-6.
- The e2e chain (turn limit + boost) rebuilt from the logs matches on e2e frames: p99 2e-4 / 9e-6, max 0.46 / 0.056.
  The gate's effect is therefore applied as a delta on the logged aTarget.
- The cruise candidate is not logged. It is rebuilt (p90 |Δ| 0.06 / 0.05 on cruise frames) and floored at the logged
  aTarget wherever e2e won.
- Every plan matched its modelV2 by `modelMonoTime` (0 unmatched).
- Not simulated: AccelBoost eligibility changes, and the downstream FCA11 response (episode suppression).

## Tests

- New file `openpilot/sunnypilot/fork/tests/test_e2e_far_lead_gate.py`: **17 passed**. It covers:
  - 10-frame enter count, counter reset, stay band, immediate exit on each of the 7 conditions;
  - raise-only on 20k randomized frames and inside the real `update()`;
  - no-lead, shouldStop and v < 3 bit-exact against a gate-disabled planner run in lockstep;
  - a synthetic cut-in drops the gate the same frame, and so does each cut-in signature alone;
  - 1.2× / TTC 13 s does not gate;
  - the turn limit still binds after the gate;
  - `d_des` equals the MPC distance and uses the MPC's `t_follow`.
- Existing `fork/tests` + `selfdrive/controls/tests`, run serially: **490 passed, 18 failed, 9 errors**. The failure
  set is byte-identical with the planner file reverted to `5e162a6786`, so these are pre-existing environment failures:
  - `test_following_distance` 7;
  - `test_latcontrol` 5, `test_latcontrol_torque_buffer` 1;
  - `test_cruise_prefs` 2;
  - `test_cal_consent_big_ui` 9 errors.
- `test_leads.py` (card process replay) times out in the worktree **on the base commit as well**, so it is excluded. It
  passes in the main checkout.
- Environment:
  - Worktree `~/repos/sunnypilot-cycle1`, using the shared `.venv`.
  - `PYTHONPATH` = worktree + worktree `opendbc_repo` + the main checkout's `msgq_repo` / `rednose_repo` /
    `tinygrad_repo`.
  - `opendbc_repo` is a detached worktree at main's pin `e3f1487d` with the fork's 3 opendbc patches applied
    (uncommitted, as CI does).
  - Built artifacts were **copied** from `~/repos/sunnypilot`, not rebuilt: `openpilot/cereal/gen`, the cereal `.a`
    files and `services.h`, `openpilot/common/libparams_c.dylib`, and the long-MPC `c_generated_code` +
    `acados_ocp_long.json`. All are gitignored.

## Not verified

- Closed-loop behaviour and the on-car feel.
- FCA11 episode counts under the new plan.
- Behaviour on routes other than 167/168.
- Any acceptance number for the AccelBoost interaction when the gate changes boost eligibility.

# Under-brake fix — FCA11 crawl dwell escape (opendbc patch `0005`)

## Defect

Route 167 seg 9, 15:50:36.69–39.34 CDT: the car crept from 1.13 to 1.49 m/s toward a lead closing 6.4 → 3.4 m with
no brake for 2.65 s. The ask stayed ≤ −0.5 for 74 frames, lifted to −0.459 for about 53 frames, then stayed ≤ −0.5 for
68 frames (14 of them ≤ −0.60). Above the 3 km/h hold band a new episode needs 20 frames ≤ −0.60, an ask ≤ −1.2, or the
1.0 s dwell (100 consecutive frames ≤ −0.5). None was met (evidence: `wsSTOP/feel-167-168/underbrake/VERDICT.md`).

## Change

`opendbc/sunnypilot/car/hyundai/fca11_long.py`, feel path only: a second, strict counter `_feel_dwell_slow` counts the
same qualifying frames as the dwell (ok ∧ ask ≤ −0.5) only while vEgo < `FCA11_FEEL_DWELL_SLOW_MPS` (2.0 m/s). Any other
frame resets it. It opens an episode at `FCA11_FEEL_DWELL_SLOW_FRAMES` (70 = 0.7 s). Because the counter is strict, a car
slowing through 2 m/s cannot open off frames it counted at speed. At or above 2 m/s the 100-frame dwell is unchanged.
Gain, release (−0.30), min-on, bypass, coast-first, camera hand-back, panda and planner are untouched. The counter is
cleared on a longActive drop and on a feature disable, the same as `_feel_dwell`.

## Variant replay

Method: an open-loop frame replay of the validated gate (`underbrake/gate_replay.py`, which matches 135/155 real brake
starts within 60 ms), changing only the dwell counter. Scripts and JSON are in `wsSTOP/feel-167-168/underbrake-fix/`
(`variant_replay.py`, `sweep.py`, `final_table.json`). The replay asserts it reproduces `gate_replay.sim` exactly for
the deployed configuration. Column definitions:
- "standstills perturbed": the 27 approach windows (t_stop − 30 s … t_stop + hold) where the brake command differs.
- "changed openings": every episode opening, route-wide, that the deployed gate does not have.
- "standalone": a new brake run that does not overlap any deployed run.

| variant | seg 9 opens (15:…) | standstills perturbed /27 | changed openings | of them > 2 m/s (max) | standalone pulses (speed) | added brake-on: stop windows / route-wide |
|---|---|---|---|---|---|---|
| A: DWELL 100 → 70, all speeds | 50:37.99 | 11 | 38 | 36 (26.4 m/s) | 1.31, **12.32, 4.76** m/s | 4.73 s / 14.71 s |
| B: leak −2 per above frame | not covered | 12 | 0 | 0 | none | 19.54 s / 101.42 s |
| B: leak −3 per above frame | not covered | 12 | 0 | 0 | none | 15.59 s / 71.61 s |
| B: reset after 2 consecutive above | not covered | 10 | 0 | 0 | none | 0.13 s / 0.12 s |
| B: reset after 3 consecutive above | not covered | 10 | 0 | 0 | none | 0.26 s / 0.24 s |
| B′: freeze in (−0.50, −0.45], all speeds | 50:38.88 | 2 | 4 | 3 (11.2 m/s) | 1.46 m/s | 1.19 s / 1.92 s |
| B′: freeze in (−0.50, −0.45], v < 2 m/s | 50:38.88 | 1 | 1 | 0 | 1.46 m/s | 0.42 s / 0.42 s |
| **Landed: dwell 70 while v < 2 m/s (strict)** | **50:37.99** | **3** | **2** | **0 (1.39 m/s)** | 1.31 m/s (seg 9 itself) | 1.82 s / 1.52 s |

The landed variant's 3 windows are 167-s9 (+1.22 s) and 167-s25 16:07:01 / 16:07:23 (+0.30 s each). The s25 entries are
one window counted twice, so this is 2 distinct approaches. The s25 opening is at 16:06:59.89, 1.39 m/s, ask −0.58 and
deepening. The deployed gate opens 0.33 s later on the same ask, so this is not a new pulse. No variant loses brake-on
time (0 s lost everywhere). Scoping the landed variant to 1.5 m/s gives identical results. A non-strict scoped variant
(the speed checked only at the open) gives identical numbers on these drives, but it is weaker in principle, so the strict
form was landed.

## Selection

- **Brief A is rejected by the at-speed rule.** 36 of its 38 changed openings are above 2 m/s (up to 26 m/s), and it adds
  standalone pulses at 12.3 and 4.8 m/s. That is the owner's early-braking-at-speed complaint.
- **Brief B (dropout tolerance) cannot fix this defect.** The lift is a 53-frame shallow dip, not a dropout:
  - A leak of 2 or 3 per frame drains the count. Counting 2–3 frames before a reset still resets inside the dip.
  - With 100 frames needed, the earliest any dropout-tolerant counter can open is about 15:50:38.23, after the brief's
    15:50:38 target.
  - The leak forms also keep the dwell latched after the ask rises above −0.30, so episodes run past the release rule
    (+72 to +101 s). That is a release change and out of scope.
- **The freeze-in-band B′ scoped below 2 m/s is the narrowest variant (1 window), but it opens at 15:50:38.88.** That is
  0.46 s before the driver braked. The VERDICT already judged an open 0.35 s before the driver too late, so B′ does not
  fix the defect.
- **The landed variant is A's constant, scoped to the crawl regime.** It meets the coverage bar (1.35 s before the driver)
  and passes the reject rule. Strictly, it departs from the brief's two literal variants. A reference-model review agreed
  with this pick and asked that the deviation be stated, which it is here.

## Verification

- **Real class on the logged stream:** the patched `Fca11LongBrake` was driven with the logged 167 15:50:20–39.34
  carControl/carState (`underbrake-fix/real_class_seg9.py`). It opens at **15:50:37.99, 1.31 m/s**. With the escape
  disabled it does not open.
- **Tests:** new `test_fca11_crawl_dwell.py` has 12 tests:
  - opens at exactly 70 frames;
  - 100 frames at 2.0, 2.5, 5, 12 and 25 m/s;
  - strictness: frames counted at speed are ignored, and crossing 2 m/s resets;
  - a lift above −0.5 resets, and the gas/`ok` veto plus the hold-off resets;
  - the seg-9 ask shape opens while crawling, and the deployed gate does not open on it;
  - the episode holds through the −0.48 lift;
  - FEEL off: the counter never runs;
  - disengage and feature-disable both reset.
- **Suite:** `opendbc/sunnypilot/car/hyundai/tests` + `opendbc/car/hyundai/tests` (pytest) went from 718 passed to
  **730 passed**, with 2 skipped. The skips are pre-existing Santa Cruz parametrized subtests ("Skip checking Santa Cruz
  for its parts", "No part numbers for car model"), with the same count on the unpatched series.
- **Mutation:** **14/14 killed**, run in a scratch copy with `PYTHONDONTWRITEBYTECODE=1` and `__pycache__` cleared. The
  mutants:
  - the speed limit set to 2.5 and to 1.5, and `<` changed to `<=`;
  - 69 and 71 frames, and `>=` changed to `>`;
  - the speed scope dropped, `ok` dropped, `feel` dropped;
  - a non-strict hold, the onset hook removed, the threshold set to the release value;
  - no reset on a longActive drop, no reset on disable.
- **Series:**
  - Rehearsal: a pristine clone at pin `e3f1487d`, then the `LC_ALL=C`-sorted `patches/*.patch` (`0001`, `0002`, `0003`,
    `0005`) applied with plain `git apply`: 4/4 applied. The resulting tree is `diff -r` identical to the worktree's
    `opendbc_repo`. `fca11_long.py` sha256 is `c8308e69…` and the test file is `a9d0e9bd…`.
  - `0005` refuses to apply alone on a pristine pin, because it depends on `0002`.
  - `check_patch_numbers` passes: 4 series + 1 panda, unique (`0004` is the panda patch).

## Not verified (under-brake fix)

- Closed-loop behavior: the replay is open-loop, and the planner would re-plan after an earlier brake.
- On-car feel at the crawl open: soft ramp from 4 LSB, the measured plant is about 1 m/s² at 4–5 LSB.
- Routes other than 167/168.
- Interaction with the camera hand-back and the one-slot release-pending delay, which the replay does not model (the
  real-class run uses a quiet camera).
- Nothing pushed, built into a prebuilt, or deployed.


# Lead-deceleration veto (BRIEF G, after the gate audit)

The audit (`~/comma-sim/wsSTOP/feel-167-168/gate-audit/AUDIT.md`) failed the gate's safety rule. At 168 17:14:55 the
lead braked at about −1.0 to −1.7 m/s², and the gate discarded a −0.73 e2e ask for about 1.3 s.

## Change

- `update_e2e_gate()` adds `lead.aLeadK > E2E_GATE_ENTER_ALEAD` (−0.5) to **enter** and
  `lead.aLeadK > E2E_GATE_STAY_ALEAD` (−0.8) to **stay**.
- Nothing else changed: raise-only, the 10-frame delay, gap/TTC/vRel/MPC terms, turn-limit placement, law `0005`, gain.
- **Sign convention, from the code:** `radarState.leadOne.aLeadK` is the lead's **absolute** accel in m/s².
  Negative means the lead is braking.
  - For a radar track it is `radard.Track.aLeadK = kf.x[ACCEL]`, the KF1D accel state on `vLead`.
  - For a vision lead it is `get_RadarState_from_vision`'s `modelV2.leadsV3[0].a[0]`.
  - The MPC uses the same value (`long_mpc.process_lead`: `a_lead = lead.aLeadK`).
- **Caveat: on 167/168 every lead frame is vision-only** (`radar` = False, `radarTrackId` = −1 on 100 % of lead
  frames). So the veto runs on the model's lead-accel estimate, not a radar Kalman state.
  - This is still the signal the follow MPC uses.
  - It agrees with the audit's independent series: p99 |Δ| 0.08 / 0.09 m/s².
- No `vRel` derivative and no TTC-shrink term were added.
- Harness: `check_e2e_gate_logs.py` now extracts `leadOne.aLeadK` per plan frame (new column `l_alead`; the cache is
  re-extracted when the column set changes) and passes it to the real gate.
- Audit re-run: `~/comma-sim/wsSTOP/feel-167-168/lead-veto/audit_veto.py` → `veto_audit.json`, `veto_bad_frames.json`.
  - It reuses the committed `extract_route`/`replay` and the audit's LS2/Q2 estimators unchanged.
  - Variants patch the two constants in-process. `noveto` reproduces the audit (0 lowered, 26 frames ≤ −1.0, set B
    209/217).

## Measured trade-off (pooled 167 + 168, open-loop)

Column notes:
- Bad aLeadK: gate active ∧ aLeadK ≤ −1.0 ∧ TTC shrinking.
- LS2 / LS2&Q2: the audit's −1.5 rule.
- Set B: the audit's engaged ∧ e2e ∧ old-enter-predicate frames with aTarget < −0.5 that were raised.
- Onsets: out of the 33 addressable lead-present e2e 4c onsets.

| variant (enter/stay) | bad aLeadK ≤ −1.0 | LS2 / LS2&Q2 ≤ −1.5 | lowered | set B raised | onsets | stops first-ask unchanged |
|---|---|---|---|---|---|---|
| noveto | 26 | 35 / 17 | 0 | 209/217 = 96.3 % | 30/33 | 12/15, lost 0 |
| veto_-0.5/-0.8 | 0 | 13 / 8 | 0 | 146/217 = 67.3 % | 29/33 | 14/15, lost 0 |
| enter-0.3 | 0 | 13 / 8 | 0 | 136/217 = 62.7 % | 29/33 | 14/15, lost 0 |
| enter-0.7 | 0 | 13 / 8 | 0 | 161/217 = 74.2 % | 29/33 | 14/15, lost 0 |
| stay-0.6 | 0 | 7 / 3 | 0 | 134/217 = 61.8 % | 27/33 | 15/15, lost 0 |
| stay-1.0 | 0 | 17 / 8 | 0 | 152/217 = 70.0 % | 29/33 | 13/15, lost 0 |
| loosest_-0.7/-1.0 | 0 | 17 / 8 | 0 | 167/217 = 77.0 % | 29/33 | 13/15, lost 0 |

## Acceptance (committed −0.5 / −0.8)

| # | Criterion | Result |
|---|---|---|
| 1 | Gate-active ∧ aLeadK ≤ −1.0 ∧ TTC shrinking | **0** (was 26). Gate-active aLeadK minimum −0.79 / −0.76. **PASS** |
| 2 | Plan lowered, pooled | **0** of 65,505; e2e lowered 0. **PASS** |
| 3 | Set B raise fraction (≥ 80 %) | **146/217 = 67.3 %** (was 96.3 %). **FAIL.** Best allowed setting −0.7/−1.0: 76.96 %, still FAIL |
| 4 | Onset coverage (33) | **29/33** (was 30/33). The new miss is 167 16:15:16. A single-frame vision aLeadK spike of −1.10 at 16:15:16.18 dropped the gate, which was still inside its 10-frame re-entry at the onset. aLeadK was +0.04 on the next frame. The vision aLeadK has such 1-frame spikes, and the veto acts on them by design: it errs toward keeping the brake ask |
| 5 | Stops | `shouldStop` changed 0; gate on model-shouldStop frames 0. Standstill plan changes 0. No stop lost its brake. **14/15** first-ask unchanged vs the base build (was 12/15). The one change is 167 16:00:38: 28.51 → 28.16 s, the same as before the veto. **PASS** |
| 6 | Unit tests | `test_e2e_far_lead_gate.py` **25 passed, 0 skipped** (was 17). 8 of them fail with the veto reverted. **PASS** |

**Why #3 fails.** Of the 63 set-B frames that lost their raise:
- 12 had aLeadK ≤ −0.8 (stay veto).
- 24 were in the −0.8 … −0.5 band, during an entry being blocked.
- 27 had aLeadK > −0.5 on the frame itself, still inside the 10-frame re-entry after a veto earlier in the episode.
- 31 of the 63 have LS2 ≤ −1.0.

Most of what is lost is the lead actually slowing: 168 17:14:55 is 13 of these frames, 167 15:56:40 is 25. This is
the trade the veto exists for.

Set B restricted to frames that also pass the new enter term is 131/164 = 79.9 %.

**Inside the brief's bounds (±0.2 per threshold), #3 cannot be met without giving up #1's intent.** Relaxing #3, or
re-defining set B to exclude braking-lead frames, is the brief owner's call. It was not changed here.

**17:14:55 (168), replayed:**
- The gate drops at 17:14:55.33 (aLeadK −0.81), and from then on aTarget equals the logged plan (−0.47 → −0.68).
- Without the veto it held +0.29 → −0.14 until 17:14:56.58.
- That is about 1.25 s of e2e braking restored.

**Remaining LS2 ≤ −1.5 frames (13, aLeadK all > −0.8). These are the audit's noise / identity-switch episodes:**
- 167 16:05:10.30: not engaged.
- 168 16:39:25.04–25.29: aLeadK −0.74 … −0.60, raw −0.50 → +0.39.
- 168 16:59:54.84–55.29.
- 168 17:00:01.59.
- 168 17:09:38.53.

Of these, three episodes are LS2&Q2 (8 frames): 16:39:25, 16:59:54, 17:09:38. 167 16:15:16–17 (aLeadK −1.28) and
168 17:14:55 are gone.

## Tests (veto)

- `test_e2e_far_lead_gate.py`: 25 passed, no skips. New tests:
  - sign / hysteresis order;
  - a hard-braking lead (−0.5 … −3.0) blocks entry and resets the enter count;
  - −0.8 / −1.0 / −1.7 drop the stay on the same frame, with the ask passed through;
  - the −0.6 stay band;
  - a mildly braking lead (−0.3) still gates;
  - the 168 17:14:55 shape from the logged values;
  - a planner-level lockstep check: a braking 4× lead is bit-exact with ungated, with a non-vacuous control.
- The exit-is-immediate and 20k randomized raise-only tests now include aLeadK.
- `fork/tests` + `selfdrive/controls/tests` (excluding `test_leads.py`): **515 passed, 2 skipped, 18 failed, 9 errors**.
  The failures are the same pre-existing environment set as above: following_distance 7, latcontrol 5 + 1,
  cruise_prefs 2, cal_consent_big_ui 9 errors.
- `ruff`: clean.

## Not verified (veto)

- Closed loop. On-car feel.
- Radar-track leads: this car's leads are vision-only, so the radar-Kalman `aLeadK` path is untested on logs.
- Routes other than 167/168.
- Nothing pushed or deployed.
