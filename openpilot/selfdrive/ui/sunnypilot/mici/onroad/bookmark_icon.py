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

Notes that are easy to get wrong, kept next to the code that depends on them:
- the icon's rect is the WHOLE on-road view (the view renders the icon with its own rect), so "tap the screen" really is
  the hit area;
- events arrive per touch SLOT, which is why the widget turns multi-touch on: the base class silently drops every event
  whose slot is not 0, so the second finger of the gesture would otherwise never arrive;
- offroad the same guard the upstream handler applies (``ui_state.started``) applies here too.
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
    # upstream first: the bookmark swipe and its own guard behave exactly as before
    super()._handle_mouse_event(mouse_event)
    if self._bad_mark_callback is None or not ui_state.started:
      return

    pos, slot, t = mouse_event.pos, mouse_event.slot, mouse_event.t
    if mouse_event.left_pressed:
      self._session.press(slot, pos.x, pos.y, t)
    elif mouse_event.left_down:
      self._session.move(slot, pos.x, pos.y, t)
    elif mouse_event.left_released and self._session.release(slot, pos.x, pos.y, t):
      if self._session.is_bad_mark():
        # Consume the release: the view would otherwise read the same tap as its own click (scroll to home) and the
        # driver would lose the road view while marking. Same one-shot guard the FCA11 consent widget uses.
        self._interacting = True
        self._bad_mark_callback()
