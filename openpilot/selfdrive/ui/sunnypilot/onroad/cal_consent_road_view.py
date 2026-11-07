"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Fork (adurham): BIG-UI (tici / tizi) on-road view for the FCA11-long CAL driver-consent prompt
(opendbc patches 0033 / 0034).

WHY THIS FILE EXISTS
--------------------
The 0033 consent prompt was wired ONLY into the mici (small-screen) UI tree
(``sunnypilot/mici/onroad/augmented_road_view.py``, swapped in by ``mici/layouts/main.py``). But
``ui.py`` runs ``MainLayout`` (the big UI) whenever ``gui_app.big_ui()`` -- i.e. on every ``tici`` /
``tizi`` device, which is the owner's comma 3X. So the prompt could never appear on this car. This
module is the big-UI twin of the mici swap: a subclass of the big-UI ``AugmentedRoadView`` that ADDS the
same ``CalConsentWidget`` (imported from the mici package, unchanged -- it has no mici-specific
dependencies). It is swapped in by ``layouts/main.py`` behind ``if gui_app.sunnypilot_ui():`` next to the
existing ``SettingsLayoutSP`` / ``HomeLayoutSP`` swaps; upstream ``onroad/augmented_road_view.py`` is
untouched.

TOUCH SEMANTICS (differs from mici on purpose)
----------------------------------------------
The big-UI base view acts on mouse PRESS (it toggles the sidebar via the click callback in
``_handle_mouse_press``; ``_handle_mouse_release`` is a no-op), whereas the mici view acts on RELEASE.
So the guard sits on ``_handle_mouse_press`` here. Ordering makes this exact: ``Widget.render`` runs
``_render`` (which renders -- and so lets the prompt process -- the same frame's mouse events) BEFORE the
parent view's own ``_process_mouse_events``, so a press that started on a prompt button has already set
``interacting()`` when the parent's press handler runs. ``interacting()`` is consume-once; we consume it
only here. The big-UI view has no bookmark one-shot (the bookmark lives in the sidebar), so there is
nothing else to preserve; the base's own HUD guard (``user_interacting()``) still runs via ``super()``.

When NO rep is pending the widget is not visible, so ``Widget.render`` returns before drawing or
processing any touch: the view behaves byte-for-byte like upstream (the only extra work is a 5 Hz
best-effort ``open()`` of ``pending.json``).
"""
from openpilot.selfdrive.ui.onroad.augmented_road_view import AugmentedRoadView
from openpilot.selfdrive.ui.sunnypilot.mici.onroad.cal_consent_widget import CalConsentWidget
from openpilot.selfdrive.ui.ui_state import ui_state


class AugmentedRoadViewCal(AugmentedRoadView):
  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self._cal_consent = CalConsentWidget()
    # Only take part in rendering / touch when a rep is genuinely pending (or briefly, "firing...").
    self._cal_consent.set_visible(lambda: self._cal_consent.is_prompting())

  def _handle_mouse_press(self, mouse_pos) -> None:
    # A tap on the consent prompt must NOT also toggle the sidebar (the base acts on press).
    if self._cal_consent.interacting():
      return
    super()._handle_mouse_press(mouse_pos)

  def _render(self, rect) -> None:
    super()._render(rect)
    if not ui_state.started:
      return
    # Throttled, best-effort file poll; then render inside the camera content area (never over the
    # sidebar or the border). Pure draw calls -- never blocks.
    self._cal_consent.update()
    self._cal_consent.render(self._content_rect)
