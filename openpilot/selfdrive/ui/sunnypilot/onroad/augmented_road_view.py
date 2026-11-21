"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl
from openpilot.cereal import log
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.ui.ui_state import UIStatus, ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.selfdrive.ui.sunnypilot.onroad.confidence_meter import ConfidenceMeter
from openpilot.selfdrive.ui.sunnypilot.onroad.hold_timer import HoldTimerOverlay

BORDER_COLORS_SP = {
  UIStatus.LAT_ONLY: rl.Color(0x00, 0xC8, 0xC8, 0xFF),  # Cyan for lateral-only state
  UIStatus.LONG_ONLY: rl.Color(0x96, 0x1C, 0xA8, 0xFF),  # Purple for longitudinal-only state
}


class AugmentedRoadViewSP:
  def __init__(self):
    self._fade_texture = gui_app.texture("icons_mici/onroad/onroad_fade.png")
    self._fade_alpha_filter = FirstOrderFilter(0, 0.1, 1 / gui_app.target_fps)
    # comma 3X additions: the confidence dot (the mici confidence ball) and the
    # standstill hold timer that has to stay visible while an alert is up.
    self._confidence_meter = ConfidenceMeter()
    self._hold_timer = HoldTimerOverlay()
    self._top_overlay_errors_logged: set[str] = set()

  def update_fade_out_bottom_overlay(self, _content_rect):
    # Fade out bottom of overlays for looks (only when engaged)
    fade_alpha = self._fade_alpha_filter.update(ui_state.status != UIStatus.DISENGAGED)
    if ui_state.torque_bar and fade_alpha > 1e-2:
      # Scale the fade texture to the content rect
      rl.draw_texture_pro(self._fade_texture,
                          rl.Rectangle(0, 0, self._fade_texture.width, self._fade_texture.height),
                          _content_rect, rl.Vector2(0, 0), 0.0,
                          rl.Color(255, 255, 255, int(255 * fade_alpha)))

  def render_top_overlays(self, content_rect: rl.Rectangle) -> None:
    """Draw the 3X additions that must sit ON TOP of the alert layer.

    Called by AugmentedRoadView._render at the end of its overlay stack (after the
    HUD, the alert renderer and the driver-state renderer), so neither element can
    be covered by an alert box or a full-screen banner.

    * The confidence dot (the comma 4's confidence ball) on the right edge - off
      while a FULL alert is up: an urgent takeover screen is not the place for it,
      and mici's own full alert covers the ball the same way.
    * The standstill hold timer as a small pill, ONLY while an alert is up.  With
      no alert the existing "STOPPED" circle already shows it, so this never
      duplicates the timer.

    Each element is drawn under its OWN guard (review fix, 2026-11-20): these run every frame in the process that also
    draws the alert layer, and the pill runs through every openpilot hold (an alert is up from the dead stop), so a
    raise here must never end the UI process. A failing element skips that frame, its first failure is logged once, and
    it is tried again on the next frame. Only these two calls are guarded - the alert renderer and the HUD are not.
    """
    if not ui_state.started:
      return

    alert_size = ui_state.sm['selfdriveState'].alertSize
    if ui_state.confidence_meter and alert_size != log.SelfdriveState.AlertSize.full:
      try:
        self._confidence_meter.update()
        self._confidence_meter.render(content_rect)
      except Exception:
        self._top_overlay_failed("confidence meter")

    if (ui_state.standstill_timer and alert_size != log.SelfdriveState.AlertSize.none
            and ui_state.sm['carState'].standstill):
      try:
        self._hold_timer.render(content_rect, self._hud_renderer.standstill_elapsed_time)
      except Exception:
        self._top_overlay_failed("hold timer")

  def _top_overlay_failed(self, element: str) -> None:
    """Log the first failure of one top-overlay element, with its traceback; never raises."""
    if element in self._top_overlay_errors_logged:
      return
    self._top_overlay_errors_logged.add(element)
    try:
      cloudlog.exception("3X top overlay failed (%s): skipped this frame, retried every frame, logged once", element)
    except Exception:
      pass
