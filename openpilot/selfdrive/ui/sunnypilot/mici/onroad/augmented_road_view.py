"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Fork (adurham): sunnypilot mici on-road view for FCA11-long CAL driver consent (opendbc patch 0033).

The fork does NOT edit upstream files where a SP subclass swap exists (see
``augmented_road_view.py:22-24`` and ``layouts/main.py:14-16``). The base class here IS upstream
``AugmentedRoadView``; this subclass only ADDS the consent prompt widget. It is swapped in by
``openpilot/selfdrive/ui/mici/layouts/main.py`` behind ``if gui_app.sunnypilot_ui():``.

It must be visually invisible and must not interfere with anything the base view does when NO rep is
pending: the prompt widget processes no touch events at all when hidden (it is set not-visible), and
on release it is guarded exactly like the bookmark swipe (``augmented_road_view.py:182-184``).
"""
import pyray as rl

from openpilot.selfdrive.ui.mici.onroad.augmented_road_view import AugmentedRoadView
from openpilot.selfdrive.ui.sunnypilot.mici.onroad.cal_consent_widget import CalConsentWidget
from openpilot.selfdrive.ui.ui_state import ui_state


class AugmentedRoadViewSP(AugmentedRoadView):
  def __init__(self, bookmark_callback=None, *args, **kwargs):
    super().__init__(bookmark_callback, *args, **kwargs)
    self._cal_consent = CalConsentWidget()
    # Only take part in rendering / touch when a rep is genuinely pending (or briefly, "firing...").
    self._cal_consent.set_visible(lambda: self._cal_consent.is_prompting())

  def _handle_mouse_release(self, mouse_pos) -> None:
    # A tap on the consent prompt must NOT also trigger the base view's click (scroll-to-home).
    # Consume first; if it was ours, swallow the release. Otherwise fall through to the base, which
    # applies its OWN bookmark guard (we deliberately do NOT consume the bookmark flag here -- it is
    # one-shot and the base owns it).
    if self._cal_consent.interacting():
      return
    super()._handle_mouse_release(mouse_pos)

  def _render(self, _) -> None:
    super()._render(_)
    if not ui_state.started:
      return
    # Throttled, best-effort file poll; then render inside the camera content area (so the prompt
    # never covers the side panel). Pure draw calls -- never blocks.
    self._cal_consent.update()
    self._cal_consent.render(self._content_rect)
