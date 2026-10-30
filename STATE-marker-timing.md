# STATE — marker timing tests (BRIEF-FEEDBACK-marker-timing-tests.md)

lane: ~/comma-sim/trees/fb-marker, branch fix/17c-feedback-marker, head 35b62c6072, base a472aa6105

## Baseline (before any change)
- opendbc_repo/.../tests/test_driver_mark.py ......... 20 passed
- openpilot/sunnypilot/fork/tests/test_driver_mark_hook.py ... 12 passed
- NOTE the brief's glue invocation is wrong: `PYTHONPATH=... env -u PYTHONPATH python` unsets what it set
  (and the venv has an editable opendbc → ~/repos/sunnypilot/opendbc_repo, so the worktree copy must shadow it).
  Working form: `PYTHONPATH="$PWD/opendbc_repo:$PWD" ~/repos/sunnypilot/.venv/bin/python -m pytest <glue> -q`

## Findings from reading the lane
- driver_mark.py is ABSENT at base a472aa6105 (whole lane added at head) → "fail-before at base" is a
  collection error, not a behaviour failure. The meaningful fail-before is head with the pair guard
  neutralised (MARK_PAIR_DEDUP_S = 0.0).
- typing machine: press() drops any message < MARK_PAIR_DEDUP_S (0.25 s, strict <) after the last COUNTED
  press; poll() closes at strictly > MARK_GAP_S (2.0 s). _finalize: n>=2 → "bad" for EVERY press, else "good".
  So item 5 (burst of 3 = all bad) matches the code — no xfail needed.
- write path: Fca11LongBrake.on_driver_mark → _mark_writer() opens lazily (default law has no shadow/dyn
  writer) → ShadowLogger disabled (dir "0"/uncreatable) is inert; on_driver_mark swallows every exception and
  arms the confirmation BEFORE writing. So item 6 is satisfiable without a code change.
- expected code changes for items 1-6: none.

## Plan
- car layer: TestRealPressSpacing (real 4-message press, guard boundary both sides, pinned constants,
  1.0 s bad gesture, 49.79 s, burst of 3) + TestWriterUnavailable (uncreatable dir).
- glue: per-service ticks (real sub-ms gaps, both service orders), bad gesture + BAD confirmation,
  49.79 s, burst of 3, disabled dir, uncreatable dir. Name MARK_GAP_S in the existing 5 s test.

## Done (this run)
- tests-only change: 239 insertions, 0 deletions, 0 lines of behaviour code touched.
- AFTER: car layer 20 -> 29 passed; glue 12 -> 19 passed. Neighbour test_fca11_shadow_hook.py: 5 passed.
- fail-before (guard neutralised, MARK_PAIR_DEDUP_S = 0.0 on head) -> 5 failed / 24 passed (car) and
  6 failed / 13 passed (glue). Node ids in the report.
- fail-before at base a472aa6105 (only reachable as a collection error: driver_mark.py / driver_mark_hook.py
  do not exist there) -> ERROR both files, 1 error each.
- item 5: implementation types EVERY press of a 3-press burst bad (n=3) -> asserted directly, no xfail.
- items 1-6 needed NO code change. Real-data numbers pinned: 4 messages/press (rlog+qlog duplication),
  intra-press gap <= 0.106 ms, min inter-press gap 49.79 s, bad gesture ~1.0 s, guard 0.25 s (~2360x).
- committed + pushed to adurham/fix/17c-feedback-marker. No deploy. DESIGN-driver-mark.md / STATE-marker.md
  left untracked (as the previous lane commit did).
