"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

17c FEEDBACK: the BAD gesture is a TWO-FINGER TAP anywhere on the driving screen.

WHY THIS EXISTS AT ALL
Build 3 typed the verdict from the bookmark PRESS COUNT: one press = good, a second press within 2 s = bad. The owner
drove it and rejected the gesture in one line - "I can't take my eyes off the road for 2 seconds to tap 1 second apart".
Counting and timing are not available to a driver; a verdict has to be ONE action.

THE SPLIT
- GOOD keeps the gesture he already owns: the bookmark swipe in from the right edge (upstream `BookmarkIcon`), unchanged.
- BAD is new: tap the screen with two fingers. One action, no counting, no waiting, and no need to look at anything.

WHY IT IS FREE
`MAX_TOUCH_SLOTS = 2` (openpilot/system/ui/lib/application.py) and NO widget in the tree enables multi-touch - the
framework drops every event whose slot is not 0 unless a widget asks for them, so nothing in openpilot reacts to a
second finger today. A single-finger tap keeps its upstream meaning (the road view's tap -> scroll to home), so the
gesture cannot be confused with navigation.

WHY THE LOGIC LIVES HERE (and not in the widget)
This module is PURE: no pyray, no I/O, no widget state, so the whole gesture - including the per-finger bookkeeping that
is easy to get wrong - is tested anywhere, in the same pytest process as everything else. The widget adapter
(openpilot/selfdrive/ui/sunnypilot/mici/onroad/bookmark_icon.py) only feeds events in and publishes the verdict; it is
pinned by the wiring test.

ONE RULE THAT LOOKS LIKE A DETAIL: travel is measured PER FINGER, from where THAT finger landed. The two fingers of an
ordinary tap land 50-150 px apart, so measuring "distance from where the session started" would read every real
two-finger tap as a 100 px swipe and silently refuse to call it a tap.
"""

# A two-finger tap needs two fingers. (Three also counts: the gesture is "more than one".)
TWO_FINGER_TOUCHES = 2
# A tap does not travel: both fingers stay put. Wide enough for a thumb roll, far below the 150 px bookmark swipe.
TAP_MAX_PX = 40
# A tap is short. A hand resting on the screen is not a tap.
TAP_MAX_S = 0.6


def is_bad_mark(touches: int, duration_s: float, travel_px: float) -> bool:
  """True when a whole touch session is the BAD gesture: two or more fingers, short, and stationary.

  ``touches`` is how many presses the session saw (one per finger), ``duration_s`` spans the first press to the last
  release, and ``travel_px`` is the furthest ANY finger moved from where IT landed.
  """
  return (touches >= TWO_FINGER_TOUCHES
          and duration_s <= TAP_MAX_S
          and travel_px <= TAP_MAX_PX)


class TouchSession:
  """One touch session's bookkeeping: how many fingers, how long, how far each finger moved.

  The widget feeds it the four things a session can do - a finger lands, drags, lifts, and eventually all fingers are
  up - and asks ``is_bad_mark()`` once, at the end. Pure: no pyray, so this class is driven directly by tests.
  """

  def __init__(self):
    self.reset()

  def reset(self) -> None:
    self._slots: dict[int, tuple[float, float]] = {}   # slot -> where that finger landed
    self._travel: dict[int, float] = {}                # slot -> that finger's own furthest distance
    self._touches = 0
    self._start_t = 0.0
    self._end_t = 0.0

  @property
  def active(self) -> bool:
    return bool(self._slots)

  @property
  def touches(self) -> int:
    return self._touches

  @property
  def duration_s(self) -> float:
    return self._end_t - self._start_t

  @property
  def travel_px(self) -> float:
    return max(self._travel.values(), default=0.0)

  def press(self, slot: int, x: float, y: float, t: float) -> None:
    """A finger lands at (x, y) at time t. The first finger of a session starts its clock."""
    if not self._slots:                                # no finger down: this press STARTS a new session
      self.reset()                                     # (the previous session's numbers were already read)
      self._start_t = float(t)
    self._slots[slot] = (float(x), float(y))
    self._travel[slot] = 0.0
    self._touches += 1
    self._end_t = float(t)

  def move(self, slot: int, x: float, y: float, t: float) -> None:
    """A finger already on the screen drags to (x, y)."""
    start = self._slots.get(slot)
    if start is None:
      return
    self._travel[slot] = max(self._travel[slot], abs(float(x) - start[0]), abs(float(y) - start[1]))
    self._end_t = float(t)

  def release(self, slot: int, x: float, y: float, t: float) -> bool:
    """A finger lifts. Returns True when the LAST finger has lifted, i.e. the session is complete."""
    self.move(slot, x, y, t)
    self._slots.pop(slot, None)
    self._end_t = float(t)
    return not self._slots

  def is_bad_mark(self) -> bool:
    """The verdict for the session that just ended. Read only after ``release`` returned True."""
    return is_bad_mark(self._touches, self.duration_s, self.travel_px)
