# Fork settings — one table, one model (2026-11-10)

## Ten-line summary

1. The Hyundai page now has **one master choice**: Stock, Comma pedal, or Pedal + FCA11.
2. Everything else is a modifier on that choice, grouped under Longitudinal, Following and stopping,
   FCA11 braking, and Pedal.
3. The old pedal and FCA11-braking switches are gone from the page; the car still reads the same
   internal keys, so nothing about the wire changes.
4. `HyundaiFca11AffineGain` is renamed to **`HyundaiFca11Shaping`**; the old key is read as a
   fallback and stays readable for at least one release.
5. Migration is one write: your stored pedal=1 / brake=1 becomes mode 2, so the first ignition after
   the update behaves exactly as before.
6. No default changed. The two rail switches `AdaptiveFollowDistance` and `NoCloseOnLead` were
   already default **ON**; the brief's "all fork keys default OFF" was wrong.
7. Every row now says in one plain sentence what it does to the car; no patch numbers, no LSB or
   capnp jargon.
8. A row whose path is not selected is greyed, not silently live; every car-value row is offroad
   only.
9. Shadow logging and the unproven stop-hold clutch are behind **Show experimental**; either is
   shown automatically while its stored value is ON.
10. All 184 params on build `744a2de4` are registered, and every fork key has a reader; there are
    no dead keys and no orphan device keys. Tests, reachability and capnp parity are green; the
    same 8 environment failures as base.

This is the place to look up a fork key. The header keeps the registration
(type/default/flags) and a pointer here; its inline comments are local notes, not the reference.

Scope: every fork-owned key the Hyundai vehicle page can show or the car stores, plus the three
longitudinal rail switches that share that page. `OffloadMode` belongs to the Mac-offload lane and
is out of scope.

## The model

`HyundaiLongitudinalMode` (INT, default **0**) owns the two master switches that used to be eleven
peer booleans' prerequisites:

| mode | label | pedal (`HyundaiGasInterceptor`) | FCA11 braking (`HyundaiFca11Brake`) |
|---|---|---|---|
| 0 | Stock | 0 | 0 |
| 1 | Comma pedal | 1 | 0 |
| 2 | Pedal + FCA11 | 1 | 1 |

Everything else is a **modifier** on the selected path, or an **experimental/diagnostic** row that
is hidden behind the page's "Show experimental" reveal. `HyundaiGasInterceptorIDSet` stays a safety
selector with **no row**: it is what refuses to command a pedal the car cannot hear.

### Mapping from today's keys

| today | after 2026-11-10 | why |
|---|---|---|
| `HyundaiGasInterceptor` | folded into `HyundaiLongitudinalMode` (modes 1/2); legacy key still fed to the car layer | one master selector instead of a hidden prerequisite |
| `HyundaiFca11Brake` | folded into `HyundaiLongitudinalMode` (mode 2); legacy key still fed | same |
| `HyundaiFca11AffineGain` | renamed to `HyundaiFca11Shaping`; old key read as fallback and kept registered | the name was a historical misnomer (it selects the shaping law, not an affine gain) |
| `HyundaiFca11Shadow` | diagnostic row, hidden behind "Show experimental" (auto-shown while its own value is ON) | log-only; not a peer driving switch |
| `HyundaiFca11StopHoldClutch` | experimental row, hidden behind the reveal (auto-shown while ON) | never proven on the car |
| `HyundaiFca11DynamicBrakes` | FCA11 braking group switch | genuine modifier of the brake request |
| `HyundaiPedalDeliveryLaw` | Pedal group switch | genuine pedal modifier |
| `HyundaiGasRiseLag` | Pedal group switch | genuine pedal modifier |
| `DriveModePersonality` | Longitudinal group switch (live, no restart) | genuine live modifier |
| `HyundaiLongitudinalTuning` | "Stock Acceleration Tuning", listed only in Stock mode | inert while the pedal is on; dead as configured on this car |
| `AdaptiveFollowDistance` | "Following and stopping" group switch | longitudinal rail, not Hyundai-specific |
| `NoCloseOnLead` | "Following and stopping" group switch | same |
| `StopApproachCommit` | "Following and stopping" group switch | same |

### Migration (nothing silently changes a default)

`openpilot/sunnypilot/fork/hyundai_settings.py` is the shim and the single source of truth behind
`initialize_params()`.

* `HyundaiLongitudinalMode` is written **once** when it is absent and either legacy key exists:
  `(pedal, brake)` → `(0,0)=Stock`, `(1,0)=Pedal`, `(1,1)=Pedal+FCA11`. The owner's car
  (`pedal=1, brake=1`) migrates to mode 2, so the first ignition after the deploy behaves exactly
  as before. A fresh device has neither legacy key and keeps mode 0, which is the old default pair
  `(0,0)`. `(0,1)` is the old inert combination (braking without the pedal never actuated
  anything) and maps to Stock, with the legacy brake value left untouched and readable.
* `HyundaiFca11Shaping` is written **once** from `HyundaiFca11AffineGain` when the new key is
  absent. Both default OFF; the owner's `1` migrates to `1`.
* Neither key is ever overwritten once present. Once the new key exists it is **authoritative**;
  this is the one deliberate semantic change, named here rather than hidden: a later external write
  of a legacy key (an old backup restore, an SSH set) no longer re-enables a path the owner did not
  select. The legacy keys stay registered and readable for at least one release, so a rollback to a
  pre-0046 build still finds the values it wrote.
* The car layer still receives `HyundaiGasInterceptor`, `HyundaiFca11Brake`, and
  `HyundaiFca11AffineGain` in `params_list`; the values are the resolved model. That is why this
  consolidation needs **no opendbc, panda, capnp, or firmware change**, and why the wire is
  byte-identical for the same resolved state.

## Inventory (every key accounted for)

Read sites are the deployed build `744a2de4` (`dev`, 2026-11-07) for `opendbc/...` paths; superproject line numbers are this branch (the model moved in 2026-11-10). The superproject feed is `openpilot/sunnypilot/selfdrive/car/interfaces.py::initialize_params`;
`opendbc/...` means `opendbc_repo/opendbc/...` in the prebuilt tree.

| key | type / default | new UI | production reader(s) | what it changes on the car | takes effect | inert when | status |
|---|---|---|---|---|---|---|---|
| `HyundaiLongitudinalMode` | INT `"0"` | Longitudinal Mode (Stock / Comma pedal / Pedal + FCA11) | `fork/hyundai_settings.py:101` (`resolve_mode`) → `initialize_params:115` | selects the pedal/FCA11 pair below; no direct car-layer reader | next ignition | never | **new master** |
| `HyundaiGasInterceptor` | BOOL `"0"` | no row (legacy, still fed) | `initialize_params:149`; `opendbc/sunnypilot/car/interfaces.py:133` | arms the whole comma-pedal stack: `enableGasInterceptor`, op long, `pcmCruise=False`, `minEnableSpeed=-1`, `GAS_INTERCEPTOR` safety bit | next ignition | mode 0 (fed value); a stale libparams leaves it off | legacy of mode 1/2 |
| `HyundaiGasInterceptorIDSet` | STRING `"auto"` | no row | `initialize_params:154`; `opendbc/sunnypilot/car/interfaces.py:136` | pedal CAN dialect: `auto` / `standard` (0x200/0x201) / `remapped` (0x700/0x701); a forced dialect not seen leaves the feature off | next ignition | pedal not armed | safety selector; on this car the only live value is `auto`→remapped |
| `HyundaiFca11Brake` | BOOL `"0"` | no row (legacy, still fed) | `initialize_params:160`; `opendbc/sunnypilot/car/interfaces.py:167` | arms the FCA11 brake stack: `CP_SP.fca11Brake`, `FCA11_LONG` safety bit 256 | next ignition | mode ≠ 2 (fed value) | legacy of mode 2 |
| `HyundaiFca11Shaping` | BOOL `"0"` | Smooth FCA11 Braking | `fork/hyundai_settings.py:126` (`resolve_shaping`) → `initialize_params:167` | opt-in brake-shaping law (continuous when Dynamic Brakes is also ON; the 0043 shaped law otherwise) | next ignition | mode ≠ 2, or Shadow ON | **renamed** from `HyundaiFca11AffineGain` |
| `HyundaiFca11AffineGain` | BOOL `"0"` | no row (legacy fallback) | `fork/hyundai_settings.py:126` (`resolve_shaping`); `initialize_params:167` | same reader path as Shaping via the fallback; the car layer still receives this key name | next ignition | mode ≠ 2, or Shadow ON | legacy, read for ≥ 1 release |
| `HyundaiFca11Shadow` | BOOL `"0"` | Log What Shaping Would Do, behind Show experimental | `initialize_params:173`; `opendbc/sunnypilot/car/interfaces.py:164` | wire stays the OLD law (forces shaping/dynamic/stop-hold off); computes + logs the 0043 shaped law to `/data/fca11-log/fca11-shadow.jsonl`, never sends | next ignition | pedal not armed | diagnostic; at the car's values ON ≡ shaping OFF byte-for-byte |
| `HyundaiFca11DynamicBrakes` | BOOL `"0"` | Smooth Brake Release | `initialize_params:180`; `opendbc/sunnypilot/car/interfaces.py:190` | makes the continuous law possible (with Shaping), adds the 18x release ramp / rate limit / soft release / `k="b"` trace | next ignition | mode ≠ 2, or Shadow ON | genuine modifier; not independent of Shaping for the continuous law |
| `HyundaiFca11StopHoldClutch` | BOOL `"0"` | ESC Hold At A Stop, behind Show experimental | `initialize_params:191`; `opendbc/sunnypilot/car/interfaces.py:196`; `fca11_long.py::_stop_hold_ok` | adds `CF_VSM_HBACmd=1 + FCA_StopReq=1` to the 0x38D frame during a standstill hold | next ignition | mode ≠ 2, Shadow ON, or not at a real hold | experimental; 0 candidate frames on the 2026-11-09 drives |
| `HyundaiPedalDeliveryLaw` | BOOL `"0"` | Light Braking Response | `initialize_params:197`; `opendbc/sunnypilot/car/interfaces.py:204`; `gas_interceptor.py:735` | measured-inverse pedal law for light negative asks + one-sided ask filter; NORMAL drive mode only | next ignition | pedal not armed | genuine modifier |
| `HyundaiGasRiseLag` | BOOL `"0"` | Smooth Gas Application | `initialize_params:203`; `opendbc/sunnypilot/car/interfaces.py:214`; `gas_interceptor.py:738` | rise-only first-order lag (τ 0.45 s) on the accel ask; falls pass through | next ignition | pedal not armed | genuine modifier |
| `DriveModePersonality` | BOOL `"0"` | Drive Mode Personality | `fork/drive_mode_personality.py:37`; `selfdrived.py:150,292,676` | ECO→relaxed, NORMAL→standard, SPORT→aggressive; N/N-Custom blocks openpilot longitudinal; lateral untouched | **live** (selfdrived params thread, 0.1 s) | never | genuine live switch |
| `HyundaiLongitudinalTuning` | INT `"0"` | Stock Acceleration Tuning (Stock only) | `initialize_params:121` → `opendbc/sunnypilot/car/interfaces.py:253` | `CP.longitudinalActuatorDelay` via the tuning config; the tuned accel is published but never sent on a pedal car | next ignition | pedal armed (`scc_long` False; this car's DEFAULT delay is already 0.50 s) | **dead as configured** on this car; not dead code |
| `AdaptiveFollowDistance` | BOOL **`"1"`** | Single Follow Gap | `fork/adaptive_follow.py:38,143`; planner `:38` | one continuous follow gap for every personality (1.45 s town → ~1.19 s at 72 mph) | next ignition / planner start | never | genuine rail switch; **default is 1, not 0** |
| `NoCloseOnLead` | BOOL **`"1"`** | Don't Speed Up To A Lead | `fork/no_close.py:54,129`; planner `:136` | caps a positive ask while the gap is closing; only removes acceleration | next ignition / planner start | never | genuine rail switch; **default is 1, not 0** |
| `StopApproachCommit` | BOOL `"0"` | Commit To The Stop | `fork/stop_commit.py:64,205`; planner `:135` | deepens the final e2e candidate toward the model's own stop point; only ever deepens | next ignition / planner start | never | genuine rail switch; ON on the car |

`Show experimental` is UI-only: no param, no car effect.

### Does anything actually read it? Any dead keys?

**No fork key is dead.** Every row above has at least one production reader, and every reader is
wired to a registered key in the deployed build. The nuances:

* `HyundaiLongitudinalTuning` is **dead as configured on this car**: the CAN wire is byte-identical
  for 0/1/2 (the tuned accel is published, not sent, because `scc_long` is False with the pedal),
  and this car's DEFAULT tuning delay is already the 0.50 s the interceptor sets. It is not dead
  code and stays for other Hyundai cars; the new page lists it only in Stock mode.
* `HyundaiGasInterceptorIDSet` has exactly one live value on this car (`auto` → remapped); forcing
  `standard` is a kill switch, because 0x201 was never in the fingerprint. It is a safety guard,
  not a feature, so it keeps no row.
* `HyundaiFca11StopHoldClutch` is implemented and wired but unreachable on the two recorded drives:
  the lane needs a standstill where the camera is not braking **and** a negative ask at v≈0; the
  census found 152 / 45 such ticks and all had a positive ask. Experimental, not dead.
* `HyundaiFca11Shadow` is a diagnostic. At the car's values, ON is byte-identical to shaping OFF
  (26364 / 9272 pedal frames and the 10 / 30 level-1 0x38D sends vanish), which is why it is behind
  the reveal rather than a peer switch.

### Device vs. build (`744a2de4`)

`GATE-C-2026-11-10-INITDATA-PARAMS.txt` (both 2026-11-09 routes) reports
`gitCommit=744a2de46c5e6bb025ad021deaa5ed1c3edc8bee`, `gitBranch=dev`:
**all 184 initData keys are registered in that build's `params_keys.h`; 0 device keys are
unregistered, and 0 fork keys are absent from the device.** `HyundaiGasRiseLag=1` is present (it
was absent from the 2026-11-06 snapshot). The 2026-11-06 "`HyundaiFca11DynamicBrakes` does not
resolve" note was a **stale local checkout**, not a device orphan: the source checkout's
`opendbc` submodule at `f95f996f` does not contain the fork's FCA11 modules; the deployed prebuilt
tree does. `AdaptiveFollowDistance` and `NoCloseOnLead` are both **1** on the car; the task brief's
"all fork keys default OFF" is wrong for those two.

## UI

The page is grouped, top to bottom:

1. **Longitudinal** — mode selector; Drive Mode Personality; Stock Acceleration Tuning (Stock only).
2. **Following and stopping** — Single Follow Gap; Don't Speed Up To A Lead; Commit To The Stop.
3. **FCA11 braking** — Smooth FCA11 Braking; Smooth Brake Release.
4. **Pedal** — Light Braking Response; Smooth Gas Application.
5. **Experimental** — Show experimental; ESC Hold At A Stop; Log What Shaping Would Do.

Each row has one plain-English sentence (no patch numbers, no LSB/capnp/`scc_long` jargon). Rows
whose path is not selected stay listed but **greyed** (`set_enabled(False)`) with a sentence saying
what they need; every row that writes a car value is offroad-only. Experimental rows are hidden
until the reveal, except while their own stored value is ON (a backup restore must not leave a live
lane with no switch in sight). The owner's test — read the page and know what each switch does to
the car — is the acceptance test for this section.

## Verification

| check | result |
|---|---|
| `check_param_reachability.py --tree . --opendbc <patched opendbc>` | rc=0; 14 fork-owned keys used, 0 unregistered; 13 `CarParamsSP` fields read, 0 unwritten; 0 feed gaps |
| `check_capnp_parity.py` against the patched opendbc (`structs.py` with 13/22/7 fields) | rc=0; all fields present in `custom.capnp` |
| compiled `libparams_c.so` rebuilt from the new `params_keys.h` | `Params().check_key("HyundaiLongitudinalMode")` and `...("HyundaiFca11Shaping")` accept, defaults 0/False |
| `test_hyundai_settings.py` + `test_fca11_settings.py` | 33 passed (migration, precedence, feed parity, row rules, wiring) |
| remaining fork suites (acados-dependent files excluded: no `acados_ocp_solver_pyx` in the source checkout) | 8 failed / 548 passed — **the same 8 failing names as the base run** (base: 8 failed / 556 passed; the −8 passed is the retired pre-0046 page suite, replaced by 33 new tests). All 8 are `ModuleNotFoundError: acados_ocp_solver_pyx`: `test_cruise_prefs.py::TestConsumersInert` (2), `test_no_close.py::TestWiring::test_applied_after_candidate_min` (1), `test_scc.py` `TestUpstreamHooks::test_planner_uses_fork_scc` + `TestEndToEndPlanner` (4) (5). |

No deploy, no flash, no param write on the car, no default change, no driving decision.
