"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

17c FEEDBACK: the card-side glue for the driver-feedback marker (k="m").

WHY IT LIVES IN `card` AND NOT IN `selfdrived`
The trigger is the on-screen bookmark button, which the UI publishes as a PAIR of messages per press (`userBookmark` +
`bookmarkButton`, see openpilot/selfdrive/ui/layouts/main.py `_on_bookmark_clicked`). Three things are needed at once:
the presses, the state snapshot, and the ONE writer for `fca11-shadow.jsonl` (owned by the FCA11 law, inside `card`).
`selfdrived` already subscribes `userBookmark` but (a) sees only half the pair and (b) cannot write the record without
opening a second ShadowLogger on the same path, which would rotate the generation chain twice and interleave two
headers - the fork's rule is one writer per path, never two. `card` already owns the law, the CarState and the CS_SP
publication, so the trigger is two extra subscription names here and nothing new anywhere else: no new process, no new
service, no new param.

HOW IT RUNS (every card frame, 100 Hz, before state_publish)
0. WHICH GESTURE: the bookmark swipe publishes the two services back to back (one bookmark press); the two-finger tap
   publishes ``bookmarkButton`` alone (see openpilot/sunnypilot/fork/mark_gesture.py). The typing rules read that
   difference, so the hook only has to forward the service name;
1. a press message (either service) -> MarkTyping.press(service), which drops the other half of the pair (0.25 s guard)
   and returns a sequence id for a REAL press;
2. the snapshot for that press is frozen immediately (`law.mark_snapshot`) and buffered - it must describe the PRESS,
   not the moment the verdict is known;
3. MarkTyping.poll() closes a burst once 2.0 s pass with no further press and returns (seq, verdict, n_presses); the
   buffered snapshot is handed to the single write path, `law.on_driver_mark(verdict, source, snapshot)`, which writes
   the k="m" record and arms the 2 s on-screen confirmation.

Typing is DEFERRED, never corrected: a single press is only known to be single after the gap, so its record and its
confirmation appear ~2 s after the press, and no record is ever rewritten.
"""
from opendbc.sunnypilot.car.hyundai import driver_mark

# the services the UI can publish per press (together they are ONE bookmark press; bookmarkButton ALONE is the
# two-finger tap). The canonical tuple lives with the typing rules so the UI adapter and this hook cannot drift apart.
MARK_SERVICES = driver_mark.MARK_SERVICES


def law_of(CI):
  """The FCA11 law object of this car interface, or None (mock/non-Hyundai/a car without the gas interceptor)."""
  return getattr(getattr(CI, "CC", None), "fca11_brake", None)


class DriverMarkHook:
  """Per-frame press counting + snapshot buffering. Owns no I/O and no writer: the law does the writing."""

  def __init__(self):
    self._typing = driver_mark.MarkTyping()
    self._snaps: dict[int, dict] = {}
    self.presses = 0     # diagnostics: real presses counted (pair halves excluded)
    self.marks = 0       # diagnostics: verdicts handed to the write path

  def step(self, sm, CI, CS, now_nanos: int) -> None:
    law = law_of(CI)
    if law is None:
      return   # no law object -> nothing owns a k="m" writer and every snapshot field would be meaningless
    # both halves are fed in: which SERVICE arrived is what types the gesture (a lone bookmarkButton = the two-finger tap)
    for service in MARK_SERVICES:
      if not sm.updated[service]:
        continue
      seq = self._typing.press(int(now_nanos), service)
      if seq is not None:
        self.presses += 1
        cc = sm["carControl"]
        # frozen AT THE PRESS: the driver is marking what he just felt, not what the car will be doing 2 s later
        self._snaps[seq] = law.mark_snapshot(CS, int(now_nanos), engaged=bool(cc.enabled),
                                            long_active=bool(cc.longActive))
    for seq, verdict, _n_presses, source in self._typing.poll(int(now_nanos)):
      snap = self._snaps.pop(seq, None)
      if snap is None:
        continue
      law.on_driver_mark(verdict, source, snap)
      self.marks += 1
