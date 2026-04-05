import pyray as rl

from cereal import custom
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.widgets import Widget

RoadType = custom.LiveMapDataSP.RoadType

# Icon size
ICON_W = 56
ICON_H = 56


def _shield_points(cx: float, cy: float, w: float, h: float) -> list[rl.Vector2]:
  """Shield shape vertices — wider at top, pointed at bottom."""
  hw, hh = w / 2, h / 2
  return [
    rl.Vector2(cx - hw * 0.85, cy - hh),        # top-left
    rl.Vector2(cx + hw * 0.85, cy - hh),        # top-right
    rl.Vector2(cx + hw, cy - hh * 0.4),          # right shoulder
    rl.Vector2(cx + hw * 0.7, cy + hh * 0.45),   # right lower
    rl.Vector2(cx, cy + hh),                      # bottom point
    rl.Vector2(cx - hw * 0.7, cy + hh * 0.45),   # left lower
    rl.Vector2(cx - hw, cy - hh * 0.4),           # left shoulder
  ]


def _fill_polygon(points: list[rl.Vector2], color: rl.Color):
  """Fill convex polygon via triangle fan from centroid."""
  n = len(points)
  if n < 3:
    return
  cx = sum(p.x for p in points) / n
  cy = sum(p.y for p in points) / n
  c = rl.Vector2(cx, cy)
  for i in range(n):
    rl.draw_triangle(c, points[(i + 1) % n], points[i], color)


def _outline_polygon(points: list[rl.Vector2], thickness: float, color: rl.Color):
  """Draw polygon outline."""
  n = len(points)
  for i in range(n):
    rl.draw_line_ex(points[i], points[(i + 1) % n], thickness, color)


def _draw_street_sign(cx: float, cy: float):
  """Green street sign — rounded rect with white double border."""
  w, h = ICON_W, ICON_H * 0.55
  outer = rl.Rectangle(cx - w / 2, cy - h / 2, w, h)
  rl.draw_rectangle_rounded(outer, 0.25, 10, rl.Color(0x00, 0x6B, 0x3F, 0xDD))
  rl.draw_rectangle_rounded_lines_ex(outer, 0.25, 10, 2.5, rl.Color(0xFF, 0xFF, 0xFF, 0xDD))
  inset = 4
  inner = rl.Rectangle(cx - w / 2 + inset, cy - h / 2 + inset, w - inset * 2, h - inset * 2)
  rl.draw_rectangle_rounded_lines_ex(inner, 0.2, 10, 1.5, rl.Color(0xFF, 0xFF, 0xFF, 0xAA))


def _draw_us_route_shield(cx: float, cy: float):
  """US Route shield — white shield with black border."""
  pts = _shield_points(cx, cy, ICON_W * 0.9, ICON_H * 0.9)
  _fill_polygon(pts, rl.Color(0xFF, 0xFF, 0xFF, 0xDD))
  _outline_polygon(pts, 3.0, rl.Color(0x00, 0x00, 0x00, 0xDD))


def _draw_interstate_shield(cx: float, cy: float):
  """Interstate shield — blue body with red top banner."""
  w, h = ICON_W, ICON_H
  pts = _shield_points(cx, cy, w * 0.95, h * 0.95)

  # Blue body
  _fill_polygon(pts, rl.Color(0x00, 0x33, 0x99, 0xDD))

  # Red top banner
  banner_h = h * 0.28
  top_y = cy - h * 0.95 / 2
  banner = rl.Rectangle(cx - w / 2, top_y, w, banner_h)
  rl.begin_scissor_mode(int(banner.x), int(banner.y), int(banner.width), int(banner.height))
  _fill_polygon(pts, rl.Color(0xBF, 0x20, 0x33, 0xDD))
  rl.end_scissor_mode()

  # White border
  _outline_polygon(pts, 2.5, rl.Color(0xFF, 0xFF, 0xFF, 0xDD))


class RoadTypeRenderer(Widget):
  def __init__(self):
    super().__init__()
    self.road_type = RoadType.unknown

  def update(self):
    sm = ui_state.sm
    if sm.recv_frame["carState"] < ui_state.started_frame:
      return

    if sm.updated["liveMapDataSP"]:
      self.road_type = sm["liveMapDataSP"].roadType

  def _render(self, rect: rl.Rectangle):
    if self.road_type == RoadType.unknown:
      return

    # Position: top-left area, offset from top
    cx = rect.x + 90
    cy = rect.y + 40

    if self.road_type == RoadType.urban:
      _draw_street_sign(cx, cy)
    elif self.road_type == RoadType.highway:
      _draw_us_route_shield(cx, cy)
    elif self.road_type == RoadType.interstate:
      _draw_interstate_shield(cx, cy)
