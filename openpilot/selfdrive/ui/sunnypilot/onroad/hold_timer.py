"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The standstill hold timer, drawn ALONGSIDE the alert on the comma 3X (tici).

Why this exists.  The standstill timer (Settings > Display > "Enable Standstill
Timer") is drawn by CircularAlertsRenderer as the big "STOPPED 1:30" circle on the
right of the 3X screen.  That whole layer is suppressed whenever ANY selfdrive
alert is up (`_allow_e2e_alerts` requires `selfdriveState.alertSize == none`).
During an openpilot (FCA11) hold an alert is up from the dead stop to the end:
the small "stop complete - hold brake pedal" (fca11StopComplete) from +0.06 s,
then the 0043 countdown ("Stopped - press the brake" at 1:00, "Press the brake
now" at 1:30, the full-screen "Brakes release in 10 seconds" at 1:50, "BRAKES
RELEASED" at 2:00).  Measured on the owner's 2026-11-19/20 drives: an alert on
screen for the whole of every engaged hold (no alert-free stretch over 1.0 s), so
the circle was never visible while openpilot held the car - only on stops the
driver held himself.  (Review correction, 2026-11-20: the first write-up said the
circle vanished only from 1:00.)

This renderer puts the same clock in its own small pill in the TOP-RIGHT corner
whenever an alert is up, drawn AFTER the alert renderer (see
AugmentedRoadViewSP.render_top_overlays), so it shows for the whole hold and
survives even the full-screen banners.  The top-right corner is the one region no
3X alert or HUD element occupies (alert text starts 200 px down; the MAX box and
speed are top-left / top-centre; the developer UI's right column starts 260 px
down).

Display only: it reads the clock CircularAlertsRenderer already keeps and never
touches the countdown itself, the brake law, alert text, priorities or timing.
That clock adds 1/target_fps per rendered onroad frame (a frame count, not a
wall clock); on the owner's holds the UI rendered without gaps (longest 125 ms)
and the clock read 59.95-60.25 s when the controller's own sent-frame countdown
reached its 60 s tier, i.e. it tracks the release to within a fraction of a
second.  It stands still while the onroad view is not rendered (e.g. settings
open), and it is the time since carState.standstill, not the controller's
counter - so on a stop openpilot is not holding it can pass 2:00 with nothing
released.
"""
import pyray as rl

from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.lib.multilang import tr

# 0043-CD: the hold is released at 120 s (6000 sent frames at 50 Hz).  The pill's
# progress bar is scaled to it; the seconds keep counting past it exactly as the
# existing circle does (a standstill can outlive a hold - e.g. the driver's foot
# on the brake - and that is still a valid standstill time).
HOLD_TIMER_TOTAL_S = 120.0

# Colour thresholds: the fork's own countdown tiers (60 s notice, 90 s audible).
HOLD_TIMER_AMBER_S = 60.0
HOLD_TIMER_RED_S = 90.0

PILL_WIDTH = 340
PILL_HEIGHT = 132
PILL_MARGIN = 40


def format_hold_time(seconds: float) -> str:
  """m:ss for a non-negative elapsed time; minutes are not capped."""
  s = max(0, int(seconds))
  return f"{s // 60:d}:{s % 60:02d}"


def hold_timer_color(seconds: float) -> rl.Color:
  """White before the 1:00 notice, amber to 1:30, red from there on."""
  if seconds >= HOLD_TIMER_RED_S:
    return rl.Color(255, 64, 64, 255)
  if seconds >= HOLD_TIMER_AMBER_S:
    return rl.Color(255, 175, 3, 255)
  return rl.WHITE


def hold_progress(seconds: float) -> float:
  """Fraction of the 2:00 hold already spent, clamped to [0, 1]."""
  return max(0.0, min(1.0, seconds / HOLD_TIMER_TOTAL_S))


def pill_rect(rect: rl.Rectangle) -> rl.Rectangle:
  """The pill's rectangle: top-right corner of the onroad content rect."""
  return rl.Rectangle(rect.x + rect.width - PILL_MARGIN - PILL_WIDTH,
                      rect.y + PILL_MARGIN,
                      PILL_WIDTH, PILL_HEIGHT)


class HoldTimerOverlay:
  """Draws the standstill clock as a small pill while an alert is on screen."""

  def __init__(self):
    self._font_bold = gui_app.font(FontWeight.BOLD)
    self._font_semi_bold = gui_app.font(FontWeight.SEMI_BOLD)

  def render(self, rect: rl.Rectangle, elapsed_s: float) -> None:
    box = pill_rect(rect)
    color = hold_timer_color(elapsed_s)

    rl.draw_rectangle_rounded(box, 0.28, 10, rl.Color(0, 0, 0, 190))
    rl.draw_rectangle_rounded_lines_ex(box, 0.28, 10, 4, rl.Color(255, 255, 255, 75))

    rl.draw_text_ex(self._font_semi_bold, tr("STOPPED"), rl.Vector2(box.x + 24, box.y + 14), 30, 0,
                    rl.Color(166, 166, 166, 255))
    rl.draw_text_ex(self._font_bold, format_hold_time(elapsed_s), rl.Vector2(box.x + 24, box.y + 44), 64, 0, color)

    # Progress towards the 2:00 release.
    bar = rl.Rectangle(box.x + 24, box.y + box.height - 26, box.width - 48, 8)
    rl.draw_rectangle_rounded(bar, 0.5, 8, rl.Color(255, 255, 255, 46))
    filled = rl.Rectangle(bar.x, bar.y, bar.width * hold_progress(elapsed_s), bar.height)
    if filled.width > 1:
      rl.draw_rectangle_rounded(filled, 0.5, 8, color)
