"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Fork (adurham): the mici bookmark icon with the driver-feedback BAD gesture (fork-local; upstream is untouched).

Upstream ``BookmarkIcon`` fires the bookmark callback on a left swipe in from the right edge. This subclass keeps that
path exactly as it is and ADDS the second gesture the owner asked for, which must be ONE action with no counting and no
timing: a TWO-FINGER TAP anywhere on the driving screen = BAD. Why a two-finger tap, and the numbers that decide it,
live in ``openpilot/sunnypilot/fork/mark_gesture.py`` - this class is only the adapter that feeds real mouse events to
``mark_gesture.TouchSession`` (pure, tested) and publishes the verdict through the callback the layout handed the view.

FOUR THINGS THIS ADAPTER GETS RIGHT ON PURPOSE
- Every slot goes to the session, but ONLY slot 0 goes to upstream's handler: the base class was written single-touch
  and reacts to any slot, so a second finger could otherwise reset its drag origin or fire a bookmark on the wrong
  finger's release.
- The session is fed ALWAYS, the callback is gated on ``ui_state.started``. Feeding only while onroad would leave a
  finger recorded as down forever when the driver changes screens mid-touch (the widget stops being rendered).
- ``_interacting`` is set as soon as a session has TWO fingers and on every event of it, because the base view calls
  ``interacting()`` (one-shot, read-and-clear) on EACH release: without this the first finger's release would be read as
  a tap on the road view (scroll to home) and the second release would be read right after the flag was consumed.
- The icon's rect is the WHOLE on-road view (the view renders the icon with its own rect), so "tap the screen" really is
  the hit area; and ``_multi_touch`` must be on or the framework drops every event whose slot is not 0.
"""
from openpilot.selfdrive.ui.mici.onroad.augmented_road_view import BookmarkIcon
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.sunnypilot.fork import mark_gesture


class BookmarkIconSP(BookmarkIcon):
  """Upstream bookmark swipe, plus the two-finger tap that marks BAD."""

  def __init__(self, bookmark_callback, bad_mark_callback=None):
    super().__init__(bookmark_callback)
    self._bad_mark_callback = bad_mark_callback
    self._multi_touch = True          # without this the framework drops every slot but 0, i.e. the whole gesture
    self._session = mark_gesture.TouchSession()

  def _handle_mouse_event(self, mouse_event) -> None:
    if mouse_event.slot == 0:
      super()._handle_mouse_event(mouse_event)          # upstream's swipe, exactly as before

    pos, slot, t = mouse_event.pos, mouse_event.slot, mouse_event.t
    if mouse_event.left_pressed:
      self._session.press(slot, pos.x, pos.y, t)
    elif mouse_event.left_down:
      self._session.move(slot, pos.x, pos.y, t)
    elif not mouse_event.left_released:
      return

    if self._session.touches >= mark_gesture.TWO_FINGER_TOUCHES:
      self._interacting = True                          # consume the view's own tap (scroll to home) for this gesture

    if mouse_event.left_released and self._session.release(slot, pos.x, pos.y, t):
      if self._bad_mark_callback is not None and ui_state.started and self._session.is_bad_mark():
        self._bad_mark_callback()
