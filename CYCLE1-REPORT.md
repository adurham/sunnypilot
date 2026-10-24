# CYCLE 1 — far-lead e2e gate (planner only)

Branch `cycle1-early-brake` from `adurham/main` `5e162a6786`. Not pushed, not deployed. Brake law, gain, onset
thresholds, panda and firmware are untouched.

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
  - Of the 41 onsets, 34 now have a plan > −0.3 at the onset frame. That count includes 4 missed onsets whose logged
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
