# Lane C — planner stop-approach commitment (route 17c, Notes 2 + 3) — design + evidence

Branch `fix/17c-planner-commitment` on `096fbddc0a` (deployed build; md5 of the three deployed opendbc files verified).
NOT deployed. Ships behaviour (i) behind a default-OFF param. Behaviour (ii) is NOT shipped: the census found zero true
positives (see §4).

## 1. Planner layering (where the published `aTarget` is shaped) — `openpilot/selfdrive/controls/lib/longitudinal_planner.py`

| step | line (this branch) | what |
|---|---|---|
| MPC lead candidate | 251 `get_accel_from_plan(...)` | lead0/lead1 obstacle MPC (t_follow from fork adaptive_follow) |
| e2e raw | 254 `modelV2.action.desiredAcceleration` | the vision model's instantaneous ask |
| far-lead gate | 258 `update_e2e_gate` | cycle-1..3 gate (raise-only, +0.4 cap) — **still in the deployed base**; `gate-delete` never shipped |
| G4 turn limit | 266–268 | `min(e2e, turn budget)` |
| cruise | 270 `get_cruise_accel` | set-speed candidate (fork setspeed_ease feeds v_cruise) |
| AccelBoost | 274 | ≤ +0.2 |
| **NEW: StopCommit** | 275–281 | `e2e + delta`, `delta <= 0`, param `StopApproachCommit` |
| arbitration | 283–289 | `min(candidates)`, clip → published `aTarget` |

`LongitudinalPlannerSP.update_targets` (sunnypilot layer) only selects `v_cruise` (cruise/SCC-V/SCC-M/SLA). It does not
shape aTarget. The planner is superproject Python, so the opendbc patch series is not its shipping form, and no patch
file is involved.

## 2. Prior branches (none is an ancestor of the base, which is a squashed prebuilt)

- `cycle1-early-brake` (median+floor ENTER on the gate veto) and `cycle2-alerts-setspeed` (closing-rate veto): **their
  planner content is already IN the base**, which additionally has the rebound latch and the +0.4 comfort cap. Nothing
  further to reuse there for stop commitment.
- `gate-delete`: deletes the gate and adds a pass-through invariant + post-drive checker. Not in the base. Reused the
  *method*, not the code: census before mechanism, plan/model pairing by `modelMonoTime`, and the lift measurement. The
  deletion itself is a separate owner decision and out of scope ("forward-fix only").
- `esc-standstill-fix`: ESC/pedal work, not planner shaping. Nothing to reuse.

None of them contains stop-approach or commitment shaping.

## 3. Behaviour (i) — no-lead stop-approach commitment (`openpilot/sunnypilot/fork/stop_commit.py`)

**Root cause (measured).** On e2e-owned no-lead stops, the published ask is the model's instantaneous ask, and that ask
eases into the stop. On the same frames the model's own 10 s trajectory had already planned the stop. Window (a)
(mono 16884–16903): from 17 m/s the plan's v_min was 3–5 m/s (ratio 0.22–0.4), while the ask sat at −0.4…−0.6.

**Shape.** `commit = min_k (v_k² − v_0²) / (2·dx_k)` over the plan's own points (t ≥ 1 s, dx > 1 m). This is the
constant decel that meets the plan's own speed at the plan's own position, i.e. the model's stop-point ask, taken early.
It introduces no new distance estimate, and the stopping margin stays the model's. Since `delta <= 0` and the final
min() is unchanged, the planner can never brake later than today (0 positive changes over 66,550 replayed rows).

- weight = plan-shape predicate: `interp(v_min/v_0, [0.25, 0.5], [1, 0]) * interp(v_min, [3, 6], [1, 0])`. A 15→6 m/s
  curve gives 0.
- arm (entry only): long active, no gas, v ≥ 5 m/s, no lead (leadOne/leadTwo) for ≥ 1.0 s, and gap `commit − e2e < −0.3`.
  Disarm: gap > −0.1, or weight 0. A lead appearing mid-stop does not release the commitment; the MPC can still go
  deeper via min().
- `delta` follows `weight·min(0, gap)` at a bounded rate (1.0 m/s³ deepening, 1.5 release). The trigger drives a ramp,
  never a veto. Gas or long-off resets it at once.
- Applied to the FINAL e2e candidate, after gate, turn limit and boost, so nothing downstream can raise it back.

**Rollback shape.** Param `StopApproachCommit` (registered in `params_keys.h`, BOOL, default "0"). OFF = identity. The
proof that OFF is identical to the deployed base is in §5. Full rollback = delete the 9 planner lines, the module and
the key.

## 4. Behaviour (ii) — progressive deepening when the lead tightens: NOT shipped (zero true positives)

The brief says vRel ramped −1.0 → −9.3 over ~9 s while the ask sat flat. The logged window that matches that
description (mono 17546–17557; gap ~100 → 79 m) reads differently:

- 17546–17553.7: vRel is noisy at −1…−2.9 m/s at ~95–109 m (TTC 35–100 s). Ask −0.36…−0.45, with a 1.5 s ease to
  −0.05…−0.16 around 17550.
- 17553.8–17555.4: the vision lead's **vLead collapses 20 → 8–11 m/s in ~1.6 s with dRel still ~100 m** (aLeadK −1.0 →
  −1.7). This is a step in the lead estimate, not a progressive tightening.
- The ask follows within ~0.5 s: −0.59 → −0.81 → −1.00 → −1.15 → −1.43 → −1.71 over 2.5 s. e2e leads until the MPC
  takes over at −1.29.

So there was no gradual vRel ramp for the planner to follow. "Flat then step" is the lead estimate's own shape.
Candidate tested: a kinematic lead requirement, rate-limited (`census_kin.py`). It produced 26 episodes over the drive:
15 NO_STOP, 9 STOP, 2 driver brake. Several turned a +0.4 ask negative on aLeadK noise, and in window (b) it starts
*after* the logged ramp (17554.3). Per the planner-gate-arc rule (count true positives first), that is no mechanism.
Not even default-off code is shipped.

**True-positive definition for future drives** (written now so the decision can be falsified): an e2e- or MPC-owned
frame run of ≥ 1 s where the ask is ≥ 0.3 m/s² shallower than the kinematic requirement of a lead with TTC < 20 s, the
lead track has been continuous (no dropout) for ≥ 2 s, and the run ends in a driver brake within 8 s. Count on 17c: 0.

## 5. Evidence (route 17c only; all runs = the EXISTING wsH harness `simdrive.py`, file unmodified)

Launcher `~/comma-sim/wsSTOP/fixes-17c/laneC/run_simdrive.py` adapts plumbing only. The harness predates this build's
upstream sync: aarch64 Cython .so → the harness's own mpc_shim; no `dec`, `experimentalMode` or `personality_scale` in
this build; map-oracle signature (#2064); `CS.out.vCruise` defaults to 0 so over_speed_release is inert, and the logged
precondition count in every changed span is 0; registry-absent keys are skipped. The flag goes through the real Params
(rebuilt Mac libparams from this tree's `params_keys.h`, `put_bool(block=True)`).

| check | result |
|---|---|
| harness fidelity (ol_planner, flag OFF vs logged aTarget, 27 windows / 3326.9 s) | RMSE **0.032** over 66,550 rows; window (a) 0.000, window (b) 0.029 |
| **equivalence: flag OFF vs unmodified 096fbddc source** (planner + key stashed, same harness) | **0 differences** over 69,190 rows (aTarget, v, ask, gas, held LSB, src, a) **and every sent CAN frame** |
| open-loop ON vs OFF | 582 / 66,550 rows changed (0.87 %); **0 positive changes**; 23/27 windows byte-identical |
| open-loop episodes | 6: 3 STOP, 3 DRIVER_BRAKE, **0 NO_STOP** |
| closed-loop ON vs OFF (plant v3) | 23/27 windows byte-identical; changed: s04-15473, s22-16571, s28-16884, s72-19569 (≤ 0.04) |

**(a) No-lead approach, window 0000017c-s28-16884 (driver brake at 7.39 m/s, logged standstill 14.4 m later).**
Open loop (ask vs the required-to-stop decel `−v²/2·rem` to the driver's own standstill point):

| first v ≤ | rem (m) | required | ask today | ask with (i) |
|---|---|---|---|---|
| 17 m/s | 135 | −1.07 | −0.61 | **−0.97** |
| 15 m/s | 101 | −1.11 | −0.77 | **−1.14** |
| 13 m/s | 69 | −1.22 | −0.88 | **−1.17** |
| 11 m/s | 43 | −1.40 | −1.20 | −1.30 |
| takeover 7.39 | 14.4 | −1.89 | −1.68 | −1.68 |

The deepening arrives from 17 m/s and tracks the required curve within ~0.1. At the takeover speed, the ask would have
been −1.68 in both arms: by then the model's own ask has caught up with its plan, the gap has closed, and (i) has
released. So the improvement is earlier commitment, not a deeper final ask. Closed loop (counterfactual tail: the
planner keeps driving and the driver does not brake; this is not a driver model): the planner's own standstill is
**x = 298.9 m with (i) vs 389.0 m without** (logged driver standstill 307.6 m). The OFF sim baseline was already 47 m
ahead of the log at the press, so only the delta (−90 m) is meaningful. The absolute position is plant-limited.

**(b) Lead tightening, window 0000017c-s38-17540:** 0 rows changed in open and closed loop. A lead was present, so (i)
never arms, and (ii) is not shipped (§4).

**(c) Negative controls:** steady following s51-18272 (601 s, lead 100 %), s63-19033, s65-19159, s41-17714, s67-19229
and s72-19533 are byte-identical. The 30-min engaged leg's last window s72-19569 changes 2 rows by ≤ 0.04, at 20052.7
(no lead, an exit slowdown ending in a driver brake). The engaged occupied stops are unchanged: stop 2 (16653) sits in
s22-16571, whose only changed rows are the 16850 no-lead episode; stops 3 and 4 are in s28-16913 and s36-17371
(byte-identical). Every launch from a hold is in an unchanged span. Max |Δ| on the 23 unchanged windows: **0.000**.

**Episodes and their logged outcome (open loop):** 16850 (s22, no lead, 22 m/s → STOP, ask −1.42 → −1.98 at 20.7 m/s;
closed-loop standstill 14 m earlier); 16891.9 (window a); 15624.5 (s04, 21.5 m/s, no lead at arming, a far lead flickers
in at 111–125 m mid-stop, driver brake at +13.2 s; ask −0.38 → −1.25 at 15.3 m/s); 18102.2 (s43, STOP); two < 0.4 s
blips ≤ 0.11.

Smoothness: ramps stay ≤ 1.0 m/s³ by construction (unit-tested). Closed loop s04: max |jerk| of aTarget falls 31.8 →
7.0 m/s³, because the OFF arm's late −3.5 step behind a close lead does not happen with (i).

## 6. Risks / open questions

1. **One route.** The curve and turn false-positive guard (v_min ≤ 6 m/s absolute) has 0 NO_STOP episodes on 17c, but
   that is a single drive. Keep the flag OFF until a multi-route census, run with this same harness on the 15x/16x
   drives, shows no turn or slowdown episodes.
2. **Firmer earlier braking at empty lights.** This is the owner's own ask (Note 2), but it is a feel change under the
   "smoothness for normal" directive: up to −0.5…−0.9 extra at ≤ 1 m/s³. The delivered decel depends on the FCA11 law
   (other lanes).
3. **The model's stop point is trusted.** Report §2 shows the model aiming ~9 m past where the driver stopped (#5). (i)
   commits to the model's point earlier; it does not move it. If the model's point is late, the stop is still late, just
   less so.
4. **Closed loop is plant-limited** (valid fraction 0.29–0.43 on the changed windows). Report deltas only.
5. **No UI toggle.** The param has to be set by hand. A shadow-log field (commit/delta per frame) would give the field
   census without behaviour change, but it needs a capnp field, so it is not done.
