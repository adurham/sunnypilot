"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The comma 4 (mici) confidence ball, ported to the comma 3X (tici) onroad screen.

The mici UI draws a single dot on the right edge of its side panel
(openpilot/selfdrive/ui/mici/onroad/confidence_ball.py): it travels from the bottom
(no confidence) to the top (full confidence), and its colour follows the driving
state and the model's own disengage predictions.  The 3X screen has no side panel,
so the same dot is drawn as a thin vertical track on the right edge of the road
view (the circular alerts end ~110 px short of the edge, and the alert band never
reaches it). When the developer UI's right column is on, that column owns the
edge, so the track moves to the column's left (dot_center_inset).

Meaning, unchanged from mici:
  * full engagement: confidence = (1 - max(brakeDisengageProbs)) *
                                  (1 - max(steerOverrideProbs))
    i.e. the model's belief that the driver will NOT have to take over.
  * lateral-only / longitudinal-only: confidence = 1 - the worst probability that
    the ACTIVE half gets overridden (steer for lat-only, brake for long-only).
  * disengaged: the dot sits at the bottom in the disengaged grey.
"""
import math

import pyray as rl

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.ui.sunnypilot.onroad.developer_ui import DeveloperUiState
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app

# The lateral-only / longitudinal-only dot colours: the SAME values as
# BORDER_COLORS_SP (openpilot/selfdrive/ui/sunnypilot/onroad/augmented_road_view.py),
# duplicated here only to avoid an import cycle (that module imports this one).
LAT_ONLY_COLOR = rl.Color(0x00, 0xC8, 0xC8, 0xFF)
LONG_ONLY_COLOR = rl.Color(0x96, 0x1C, 0xA8, 0xFF)

# The two mici colour-zone thresholds, kept identical so the two screens agree.
CONFIDENCE_HIGH = 0.5
CONFIDENCE_LOW = 0.2

# Geometry on the 3X (2160x1080).  RADIUS 30 is the mici dot's 24 px scaled to the
# 3X's 4.5x-larger screen (~5.5 % of the height, the same proportion as on mici).
DOT_RADIUS = 30
EDGE_INSET = 40          # dot centre from the right content edge
TRACK_TOP_FRAC = 0.28    # track top / bottom as a fraction of the content height
TRACK_BOTTOM_FRAC = 0.72

# Review fix (2026-11-20): the developer UI's RIGHT column (Settings > Display > Developer UI = Right or Both;
# developer_ui/__init__.py _draw_right_dev_ui: container_width 184, its own UI_BORDER_SIZE 20, x = right - 184 - 2*20)
# covers right-224 .. right-12 over the whole height of this track, so a dot at right-40 sat on its labels, values and
# units. With that column on, the track moves to the column's left, like CircularAlertsRenderer moves left for it.
DEV_UI_RIGHT_COLUMN_WIDTH = 184 + 2 * 20   # right-edge strip the column owns (pinned to developer_ui by a test)
DEV_UI_CLEARANCE = 60                      # dot centre to the column's left edge: the 44 px masking ring + 16 px gap


def dot_center_inset(developer_ui) -> float:
  """Distance from the content rect's right edge to the dot's centre, for the developer-UI mode in use."""
  if developer_ui in (DeveloperUiState.RIGHT, DeveloperUiState.BOTH):
    return DEV_UI_RIGHT_COLUMN_WIDTH + DEV_UI_CLEARANCE
  return EDGE_INSET


def disengage_confidence(brake_probs, steer_probs) -> float:
  """Confidence in [0, 1] from modelV2's disengage predictions.

  Pure, so the meaning is testable without a window: 1.0 means the model predicts
  no takeover at all, 0.0 means it predicts one right now.  An empty list is
  treated as "no prediction" -> 0.0 (fail towards showing low confidence, never
  towards inventing a high one).
  """
  brake = max(brake_probs) if len(brake_probs) else 1.0
  steer = max(steer_probs) if len(steer_probs) else 1.0
  return max(0.0, min(1.0, (1.0 - brake) * (1.0 - steer)))


def confidence_tier(confidence: float) -> str:
  """The mici colour zone for a confidence value: 'high' / 'medium' / 'low'."""
  if confidence > CONFIDENCE_HIGH:
    return "high"
  if confidence > CONFIDENCE_LOW:
    return "medium"
  return "low"


def confidence_colors(tier: str) -> tuple[rl.Color, rl.Color]:
  """The mici gradient pair (top, bottom) for a colour zone."""
  if tier == "high":
    return rl.Color(0, 255, 204, 255), rl.Color(0, 255, 38, 255)
  if tier == "medium":
    return rl.Color(255, 200, 0, 255), rl.Color(255, 115, 0, 255)
  return rl.Color(255, 0, 21, 255), rl.Color(255, 0, 89, 255)


def dot_y(confidence: float, top: float, bottom: float) -> float:
  """Where the dot's CENTRE sits for ``confidence``: bottom at 0, top at 1."""
  c = max(0.0, min(1.0, confidence))
  return bottom - c * (bottom - top)


def draw_gradient_dot(center_x: float, center_y: float, radius: int,
                      top: rl.Color, bottom: rl.Color) -> None:
  """A round dot with a vertical gradient, the same construction mici uses."""
  rl.draw_rectangle_gradient_v(int(center_x - radius), int(center_y - radius),
                               radius * 2, radius * 2, top, bottom)
  outer_radius = math.ceil(radius * math.sqrt(2)) + 1
  rl.draw_ring(rl.Vector2(int(center_x), int(center_y)), radius, outer_radius,
               0.0, 360.0, 20, rl.BLACK)


class ConfidenceMeter:
  """Draws the mici confidence dot on the 3X right edge, in the top overlay."""

  def __init__(self, demo: bool = False):
    self._demo = demo
    self._filter = FirstOrderFilter(-0.5, 0.5, 1 / gui_app.target_fps)

  @property
  def confidence(self) -> float:
    return max(0.0, min(1.0, self._filter.x))

  def _target(self) -> float:
    status = ui_state.status
    if status == UIStatus.DISENGAGED:
      return -0.5
    if status in (UIStatus.LAT_ONLY, UIStatus.LONG_ONLY):
      probs = self._status_probs()
      return 1.0 - max(probs or [1.0])
    preds = ui_state.sm['modelV2'].meta.disengagePredictions
    return disengage_confidence(preds.brakeDisengageProbs, preds.steerOverrideProbs)

  @staticmethod
  def _status_probs():
    preds = ui_state.sm['modelV2'].meta.disengagePredictions
    if ui_state.status == UIStatus.LAT_ONLY:
      return preds.steerOverrideProbs
    return preds.brakeDisengageProbs

  def update(self) -> None:
    if not self._demo:
      self._filter.update(self._target())

  def _colors(self) -> tuple[rl.Color, rl.Color]:
    status = ui_state.status
    if status == UIStatus.ENGAGED or self._demo:
      return confidence_colors(confidence_tier(self.confidence))
    if status in (UIStatus.LAT_ONLY, UIStatus.LONG_ONLY):
      c = LAT_ONLY_COLOR if status == UIStatus.LAT_ONLY else LONG_ONLY_COLOR
      return c, c
    if status == UIStatus.OVERRIDE:
      return rl.Color(255, 255, 255, 255), rl.Color(82, 82, 82, 255)
    return rl.Color(50, 50, 50, 255), rl.Color(13, 13, 13, 255)

  def render(self, rect: rl.Rectangle) -> None:
    """Draw the dot.  ``rect`` is the onroad content rect (the road view area)."""
    top_color, bottom_color = self._colors()

    top = rect.y + rect.height * TRACK_TOP_FRAC
    bottom = rect.y + rect.height * TRACK_BOTTOM_FRAC

    # A faint track so the dot's position reads as a scale, not a stray blob.
    x = rect.x + rect.width - dot_center_inset(ui_state.developer_ui)
    rl.draw_rectangle_rounded(rl.Rectangle(x - 3, top, 6, bottom - top), 0.5, 8,
                              rl.Color(255, 255, 255, 38))

    y = dot_y(self.confidence, top, bottom)
    draw_gradient_dot(x, y, DOT_RADIUS, top_color, bottom_color)
