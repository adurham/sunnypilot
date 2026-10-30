# DESIGN — driver-feedback marker (k="m") for the FCA11 fork

Lane `fix/17c-feedback-marker`, base `a472aa6105` (the deployed build). Repo-only.

## 1. Problem

The owner marks felt events with the on-screen bookmark button. What lands in the log is a bare `userBookmark` +
`bookmarkButton` pair with no verdict, no state, and no confirmation — so the review has to guess which of the 15
presses on route 17c meant "this was good" and which meant "this was bad" (`19:13:21` is literally `bookmark ×2` in
the review's own table and was read as one event). PLAN-NEXT codes the gesture: **1 press = good, 2 rapid presses =
bad**. The star button was tried and abandoned because its CAN signal reaches none of the logged buses.

## 2. Where it runs, and why there

| Concern | Needs | Where it lives |
|---|---|---|
| see the presses | `userBookmark` + `bookmarkButton` | **`card`** — new subscriptions (2 names) |
| state snapshot | `CS` (the `CarStateExt` itself), law level, personality, `ep` | `card` + the law object it already owns |
| write `k="m"` into `fca11-shadow.jsonl` | the ONE writer for that path | the law (`Fca11LongBrake`), which owns `shadow_log`/`_dyn_log` |
| 2 s on-screen confirmation | selfdrived's alert flow | law → `CS.fca11_mark` → `CarStateSP.driverMark` → `car_specific` → `EventNameSP` |

`selfdrived` was the tempting alternative (it already subscribes `userBookmark`), and it is **rejected**:
(a) it sees only half of the pair; (b) it cannot write the record without opening a *second* `ShadowLogger` on
`fca11-shadow.jsonl`, and two writers on one path each rotate the generation chain at construction and interleave
headers — the fork's rule is explicit: one writer per path, never two. `card` is the smallest existing process that
can both see the trigger and own the write; it needs no new process, no new service, and no new param.

## 3. Trigger and typing

`userBookmark` and `bookmarkButton` are published **once each per press**, back-to-back, inside one UI call. So:

- a press = `sm.updated['userBookmark'] or sm.updated['bookmarkButton']` on a tick, deduped against the previous
  counted press by `MARK_PAIR_DEDUP_S = 0.25 s` (a pair straddling a tick boundary must not count twice, while the
  shortest meaningful second press in this lane is 1.0 s);
- typing (`driver_mark.MarkTyping`, pure, monotonic-ns clock):
  - a press opens a burst, or extends the open burst if it lands within `MARK_GAP_S = 2.0` of the previous press;
  - a press further than 2.0 s away closes the previous burst first;
  - a burst closes when 2.0 s pass with no press;
  - verdict = `"bad"` if the burst holds ≥ 2 presses, else `"good"`; **every** press of a burst gets that verdict,
    including the first — the burst is the message.

**Deferred, not corrected.** The verdict of an isolated press is only knowable 2.0 s later, so the record is written at
burst close and the snapshot is frozen **at press time** (`mark_snapshot()` runs on the press frame; the buffered dict
is handed to `on_driver_mark()` when the burst finalizes). Consequences, deliberately accepted:
- single press → 1 record `"good"`, ~2.0 s after the press; confirmation appears then too;
- double press 1.0 s apart → 2 records `"bad"`, written at the second press (~1 s after the first, +1 frame for the
  CarStateSP hop);
- the stream is never rewritten, so a reviewer reading it in time order can never see a verdict flip.

`source` names the mechanism and the burst shape: `"bookmark_single"` / `"bookmark_double"`.

## 4. The record (`k="m"`)

One JSON object per line, `sort_keys`, same file and same writer as the FCA11 shadow / `k="b"` trace (schema 2).

| field | meaning |
|---|---|
| `k` | `"m"` |
| `verdict` | `"good"` \| `"bad"` |
| `source` | `"bookmark_single"` \| `"bookmark_double"` |
| `t_wall` | press instant, wall clock (s) |
| `t_mono_ns` | press instant, card's `now_nanos` (log monotonic in replay) |
| `frame`, `ep` | controller frame of the press, V3 episode id (`_sh_episode_id`; the same id the `k="b"`/`k="f"` records carry) |
| `lsb_cmd` | the level we had COMMANDED (`_dec_last_sent`, 0 = nothing on the wire) — "our commanded level" |
| `lsb_sent_now` | the level of the frame the law computed this frame (0 = none) |
| `ask_mps2` | the planner ask the law consumed (clipped to `FCA11_A_MAX_MSS2`, as in the `k="f"` trace) |
| `a_ego_mps2`, `v_ego_mps` | delivered acceleration, speed |
| `standstill`, `gas_pressed`, `brake_pressed` | pedal/standstill state |
| `personality` | raw `LongitudinalPersonality` |
| `clutch_tq_nm` | `CS.fca11_clutch_tq_nm` (TCU_DCT13 0x162; `null` until that signal lands) |
| `lead_present`, `lead_drel_m`, `lead_vrel_mps` | `CS.fca11_lead_one` (`vRel = vLead − vEgo`; **null, never 0**, when absent) |
| `lsb_min`, `lsb_max`, `lsb_mean`, `lsb_frames` | our own level over the last `MARK_LEVEL_WINDOW_FRAMES = 500` controller frames (5 s @ 100 Hz), one sample per SENT frame; all `null` (frames 0) if we sent nothing in the window |
| `engaged`, `long_active`, `armed` | context, so a mark can be read without joining to `carState` |

`armed` = the law exists and `HyundaiFca11Brake` is on (the marker only exists with the law — see §7).

## 5. Confirmation

`on_driver_mark()` arms `self._mark_alert = 1|2` for `MARK_ALERT_FRAMES = 200` frames (2 s @ 100 Hz). The controller
publishes it on the same 1-frame chain every other FCA11 flag uses:

    CS.fca11_mark = 1|2|0   (GasInterceptorCarController.create_gas_command)
    → CarStateSP.driverMark (carstate_ext.update, @20 :UInt8)
    → car_specific.py raises EventNameSP.driverMarkGood (@36) / driverMarkBad (@37)
    → events.py: NormalPermanentAlert("Marked GOOD"/"Marked BAD", duration=2.0)

Ordinals 36/37 sit above every FCA11 safety event (`fca11Unavailable` 30, `superviseStop` 31, `stopComplete` 32,
`brakeNow` 33, `cameraBraking` 34, `divergence` 35) so nothing can collide, and the code is 0 unless a burst just
finalized — an idle drive raises no alert at all.

## 6. Invariants kept

- **No mark → no behaviour change.** No subscription change alters any send; the `CanData` output of the law is a pure
  function of its inputs and nothing on the mark path is read by the send path (the ring append is a deque push).
  Proven by a byte-level `CanData` comparison of the base law vs the modified law over a synthetic drive (§tests) plus
  the untouched existing suites.
- **Idle → no I/O.** Nothing is written and no file is opened until the first mark: when neither Shadow nor dynamic
  release already opened the stream, the marker's `ShadowLogger` is created lazily on the first `k="m"` write.
- **Log dir disabled → inert.** `ShadowLogger` is already inert when the dir resolves to `None` (`enabled` False,
  `write()` returns immediately); the marker inherits that, and `on_driver_mark` is wrapped so a broken log can never
  take `card` down.

## 7. Limits (stated, not hidden)

- **The marker requires the law** (`HyundaiFca11Brake`). Every snapshot field is the law's own state, and a record of
  "our commanded level" without the law is meaningless. With the param off, `self.CI.CC.fca11_brake` has no marker path
  and no `k="m"` record exists. On the car the param is 1.
- **The good-verdict confirmation is ~2 s late** — inherent to distinguishing "one press" from "the first of two".
- Only the bookmark pair triggers this lane. `on_driver_mark(verdict, source, snapshot)` is the single write path, so a
  future wheel-button trigger is a caller change, not a law change.
