# Lane FB — driver-feedback marker (17c) — STATE
Worktree `/Users/adam.durham/comma-sim/trees/fb-marker`, branch `fix/17c-feedback-marker`, base `a472aa6105`.
(`STATE.md` at this root is the integration log already committed on this base — this lane's record lives here.)

## CP1 — worktree + base verification (done)
- `cd ~/repos/sunnypilot && git worktree add -b fix/17c-feedback-marker ~/comma-sim/trees/fb-marker a472aa6105`
  → HEAD `a472aa6105` ("sunnypilot prebuilt (fork 17c manual) — dynamic brake …").
- md5 in `opendbc_repo/opendbc/sunnypilot/car/hyundai/`:
  - `gas_interceptor.py`    `60b5ccfa22e6018f989516702b35886c`
  - `fca11_long.py`         `dc4653247a05156d902fc57ebaed94d0`
  - `fca11_shadow_log.py`   `08ae7c83fd80f681a7e9d3f87bec2566`
- **Do NOT equal the three hashes in EVIDENCE-AND-RULES** (`a274db98…`, `087111a1…`, `08ae7c83…`). Checked why instead of
  stopping: `git show 096fbddc0a:opendbc_repo/…/<file> | md5` gives exactly those three — i.e. the rules doc pins
  **build 1** (the commit it was written against), not deployed build 2. The read-only deploy tree
  `~/comma-sim/trees/fix17c-deploy` (checked out at `a472aa6105`) yields the same three hashes as my worktree, all
  three identical → **base correct for the brief's base `a472aa6105`** (= deployed per `DEPLOYED.md`); rules-doc hash
  row stale by one build. Not a STOP condition.
- Remote `adurham` present (git@github.com:adurham/sunnypilot.git). Branch only, never `main`.

## CP2 — code read (done) — the facts that drive the design
- **Trigger.** `userBookmark` + `bookmarkButton` are each published once **per press**, back-to-back, in the same UI
  call (`openpilot/selfdrive/ui/layouts/main.py:119-122`; mici layout identical). So a press = a PAIR of messages
  microseconds apart → counting both services naively counts 2 per press and every single press would read "bad".
  A pair-dedupe guard is mandatory.
- **Who can see them.** `card` (`selfdrive/car/card.py:77`) subscribes `pandaStates carControl onroadEvents carControlSP
  longitudinalPlanSP` — not the bookmark services. `selfdrived` (`selfdrive/selfdrived/selfdrived.py:110`) subscribes
  `userBookmark` ONLY. No existing subscriber sees both halves of the pair.
- **Who can write.** The FCA11 writer is the law's: `Fca11LongBrake.__init__` creates `self.shadow_log` (Shadow) /
  `self._dyn_log` (dynamic release) as a `ShadowLogger` on `fca11-shadow.jsonl` (`fca11_long.py:1478-1505`, "one writer
  per path, never two"). The law is built once in `GasInterceptorCarController.__init__` (`gas_interceptor.py:576`),
  reachable from card as `self.CI.CC.fca11_brake` (`opendbc/car/interfaces.py:144` sets `self.CC`).
- **Snapshot.** At press time the car layer holds everything: `CS` **is** the `CarStateExt` (`class CarState(…,
  CarStateExt)` in `car/hyundai/carstate.py:32`), so `CS.fca11_clutch_tq_nm` / `CS.fca11_lead_one` are in-process;
  `CS.out.*` for aEgo/vEgo/standstill/pedals; the law for its own level (`_dec_last_sent`, `_rel_sent`) and
  `_sh_episode_id`; `personality` is an argument of the law's update/create_gas_command.
- **Existing driver-facing chain (reused, not invented).** controller writes `CS.fca11_*` in `CI.apply()` → next frame
  `CarStateExt.update()` copies it to `ret_sp.<field>` ("1-frame pattern", `carstate_ext.py:275-286`) →
  `sunnypilot/selfdrive/car/car_specific.py:66-88` raises `EventNameSP.<x>` from `CS_SP` →
  `sunnypilot/selfdrive/selfdrived/events.py` maps it to an alert. FCA11 ordinals in use: 30..35 → new ones from 36.
- **Shipping form.** `opendbc_repo/` is the flattened, already-patched tree; opendbc changes ship as
  `openpilot/sunnypilot/fork/patches/NNNN-*.patch` (incl. tests — see 0012). Series ends at `0012` →
  `check_patch_numbers.py` will confirm the next free number.
- **No local real-signal check for press spacing.** Only route cache here is `~/comma-routes-baseline/00000167--…`
  (97 segments); scanning every rlog for `userBookmark`/`bookmarkButton` events found **0**. Route 181 (7 pairs) is not
  on this Mac. The pair spacing is therefore taken from the UI source; the dedupe guard is defensive (0.25 s) and is
  correct whether the two messages land in one `sm.update()` window or straddle a tick.
- **What the data shows the owner does** (`wsSTOP/drive-17c/REPORT.md`): 15 presses on 17c; `19:13:21 (bookmark ×2)`
  "sits exactly on a gas-press transition during plan braking", and `19:18:14 + 19:18:35` are two marks 21 s apart. The
  review already could not tell a double-press from two singles — the gap this lane closes.

## CP3 — design (done) — `DESIGN-driver-mark.md`
- **D1 — trigger point = `card`.** Add `userBookmark` + `bookmarkButton` to card's SubMaster: 2 service names, no new
  process, no new service, no new param. `selfdrived` was the alternative (it already subscribes `userBookmark`) and is
  rejected: it sees only half the pair, and it CANNOT write the record — the writer is the law's, inside card, and a
  second `ShadowLogger` on the same path from another process would double-rotate the generation chain and interleave
  two headers ("one writer per path, never two").
- **D2 — typing is deferred, never corrected.** An isolated press is only *known* isolated after 2.0 s, so the record
  and the confirmation are emitted at burst close, with the **snapshot taken at press time**. A "good" mark therefore
  appears ~2 s after the press; a "bad" burst appears at the second press (+1 frame). No record is ever rewritten.
- **D3 — confirmation reuses the existing CarStateSP chain** (law → `CS.fca11_mark` → `ret_sp.driverMark` →
  `car_specific` → `EventNameSP.driverMarkGood/Bad` → 2 s alert). Cannot appear outside selfdrived's normal alert
  flow, separate ordinals from every FCA11 safety alert, and code 0 = silent.
- **D4 — no new param; the marker rides `HyundaiFca11Brake`** (1 on the car). Every snapshot field is the law's own
  state; with no law object there is no marker (documented in the module).
- **D5 — level window = 500 controller frames (5 s @100 Hz), one sample per SENT frame**, matching the k="b" trace;
  `lsb_frames == 0` ⇒ min/max/mean `null` (fork convention: null, never 0).

## CP4 — implementation (done)
Files (opendbc, shipped as patch `0013`):
- NEW `opendbc/sunnypilot/car/hyundai/driver_mark.py` — pure typing state machine (`MarkTyping`), `level_window_stats`,
  constants (`MARK_GAP_S` 2.0, `MARK_PAIR_DEDUP_S` 0.25, `MARK_LEVEL_WINDOW_FRAMES` 500, `MARK_ALERT_FRAMES` 200),
  `MARK_SNAPSHOT_FIELDS`.
- `fca11_long.py` — `_lsb_hist` deque + `_note_sent_level` at both SENT sites; `frame_now`; `mark_snapshot(CS, now_nanos,
  frame=None, engaged, long_active)`; `on_driver_mark(verdict, source, snapshot)` (THE write path, lazy writer that
  shares `shadow_log`/`_dyn_log`, never a second writer); `mark_alert_code(frame)`; frame-local latching in `update()`.
- `gas_interceptor.py` — `CS.fca11_mark = mark_alert_code(frame)` (1-frame pattern).
- `carstate_ext.py` — `self.fca11_mark = 0` + `ret_sp.driverMark`.
- `car/structs.py` — `CarStateSP.driverMark: int`.
Files (superproject):
- NEW `openpilot/sunnypilot/fork/driver_mark_hook.py` — `DriverMarkHook.step(sm, CI, CS, now_nanos)` (press dedupe,
  snapshot buffering, verdict flush), `law_of(CI)`.
- `selfdrive/car/card.py` — 2 subscriptions, hook construction, one call in `state_update` after the REPLAY clock line.
- `sunnypilot/selfdrive/car/car_specific.py` — raises `driverMarkBad`/`driverMarkGood` from `CS_SP.driverMark`.
- `sunnypilot/selfdrive/selfdrived/events.py` — 2 s "Marked GOOD"/"Marked BAD".
- `cereal/custom.capnp` — `CarStateSP.driverMark @20`, `EventNameSP.driverMarkGood @36`, `driverMarkBad @37`.
- `FORK.md` — row 98 (DRAFT).

## CP5 — tests (done)
- New `opendbc_repo/.../tests/test_driver_mark.py` (20) + `openpilot/sunnypilot/fork/tests/test_driver_mark_hook.py` (12).
- **Fail-before** (tracked sources stashed, new modules kept so the failure is behavioural not a collection error):
  car layer **10 failed / 10 passed** (the 10 passing are the pure typing/level-window units), glue **11 failed / 1 passed**.
  Full-source stashed (both new modules gone too): collection ImportError on both files.
- **Pass-after: 20 passed + 12 passed.**
- Regression set (test_fca11_long, test_fca11_feel, test_fca11_feel_fix, test_fca11_shadow, test_gas_interceptor,
  test_gas_interceptor_personality, test_fca11_clutch, test_fca11_crawl_dwell) run on BOTH trees:
  **585 passed / 32 failed on each, and the 32 failing node ids are IDENTICAL** (all pre-existing).
- Fork guards: `test_car_specific_events.py` + `test_capnp_parity.py` + `test_param_reachability.py` +
  `test_patch_number_guard.py` + `test_fca11_shadow_hook.py` → **86 passed**. `check_capnp_parity --tree .` ok
  (CarStateSP 21/21 fields). `check_param_reachability --tree .` ok. `check_patch_numbers` rc 0 (12 numbered patches).
- ruff on all 11 touched files: **7 findings, exactly the 7 the base already has** (0 new).
- Fixed during the run: my `CS_SP.driverMark` attribute access broke 53 existing `test_car_specific_events` doubles that
  build `CarStateSP` as a `SimpleNamespace` → switched to `getattr(CS_SP, "driverMark", 0)` (the fork's own defensive
  pattern, used in `carstate_ext` for the same field).

## CP6 — byte-level "no behaviour change" proof (done)
- `~/.hermes/cache/scratch/law_bytecheck.py`, run in two processes against the deployed tree and this one:
  deployed law `sha256=6761cef094ce87e2ea7d475a43ae44315c2caf99919082d164d03f0929d59890` on BOTH;
  dynamic-release `sha256=7a58d4c970477b55cc0ffdc97aed792b1c74306126079935bc493c0c283c9c62` on BOTH (539 frames,
  onset/hold/release/ramp/gas-cut/driver-brake/standstill/lead/personality sweep).
- Plus in-suite: a mark fired mid-drive leaves every `CanData` byte identical (test_marks_never_change_a_byte_of_the_law_output).
- Replay: the closed-loop wsH harness cannot reproduce this code path (it needs a UI press; the harness has no UI and no
  bookmark publisher) — frame-level drive of the REAL law + the REAL hook is the accepted fallback and is what the 32
  tests are. No real-signal replay available: route 181 is not on this Mac and route 167 carries 0 bookmark events.

## CP7 — packaging (done)
- Patch `openpilot/sunnypilot/fork/patches/0013-fork-hyundai-driver-mark-feedback.patch` (660 lines, 6 files incl. tests).
  `git apply --check` on a clean copy of the DEPLOYED opendbc tree: OK; after apply all six files md5-identical to this
  worktree.
- FORK.md row **98** (DRAFT), next free number confirmed by `check_patch_numbers.py` (series now 0001-0003, 0005-0013).

## CP8 — commit + push (done)
- Commit `35b62c6072` on `fix/17c-feedback-marker` (14 files, +1579/-2). Pushed to `adurham` only:
  `git ls-remote adurham refs/heads/fix/17c-feedback-marker` = `35b62c607231fc9b2fc36e99948dbafece4c29ad` == local HEAD.
  `main` untouched; the deployed worktree left git-clean (only the `.pytest_cache` my test runs created was removed);
  no device contact of any kind.
- Untracked lane documents (not committed, same as the other lanes): `DESIGN-driver-mark.md`, `STATE-marker.md`.
