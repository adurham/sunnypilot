"""fork: the comma 3X onroad additions of 2026-11-20 -- the confidence dot and the
standstill hold timer that stays visible while an alert is up.

The two screens are pure display: nothing here touches the brake law, the 2:00
countdown, alert text, priorities or timing. What these tests pin:

  * the confidence number means what the mici ball means: the product of "the
    model does not expect a brake takeover" and "it does not expect a steer
    override", with an empty prediction list failing towards LOW confidence;
  * the colour zones and the dot's bottom-to-top travel are the mici ones;
  * the hold clock formats exactly the way the existing "STOPPED" circle does
    (m:ss, no cap), its amber/red thresholds sit on the fork's own 60/90 s
    countdown tiers, and the progress bar saturates at 2:00;
  * the pill sits in the top-right corner, clear of the full-alert title band
    (which starts 200 px down), so a full-screen banner cannot cover it;
  * the WIRING is ordered: the base 3X view calls the SP top overlay AFTER the
    alert renderer (a full-screen alert is drawn first, then the pill on top),
    behind `gui_app.sunnypilot_ui()`.
"""
import ast
import os
import unittest
from pathlib import Path

os.environ.setdefault("SCALE", "1.0")  # no auto-scale probing of a monitor in tests

REPO = Path(__file__).resolve().parents[4]
BASE_VIEW = REPO / "openpilot" / "selfdrive" / "ui" / "onroad" / "augmented_road_view.py"

from openpilot.selfdrive.ui.sunnypilot.onroad.confidence_meter import (  # noqa: E402
  CONFIDENCE_HIGH, CONFIDENCE_LOW, confidence_tier, disengage_confidence, dot_y,
)
from openpilot.selfdrive.ui.sunnypilot.onroad.hold_timer import (  # noqa: E402
  HOLD_TIMER_AMBER_S, HOLD_TIMER_RED_S, HOLD_TIMER_TOTAL_S, PILL_HEIGHT, PILL_MARGIN, PILL_WIDTH,
  format_hold_time, hold_progress, hold_timer_color, pill_rect,
)


class TestConfidence(unittest.TestCase):
  def test_no_predicted_takeover_is_full_confidence(self):
    self.assertAlmostEqual(disengage_confidence([0.0] * 5, [0.0] * 5), 1.0)

  def test_product_of_the_two_worst_predictions(self):
    # the WORST (max) sample of each list drives it, exactly like the mici ball
    self.assertAlmostEqual(disengage_confidence([0.0, 0.5, 0.1], [0.2, 0.0]), 0.5 * 0.8)

  def test_certain_takeover_is_zero(self):
    self.assertAlmostEqual(disengage_confidence([1.0], [0.0]), 0.0)

  def test_empty_predictions_fail_towards_low_confidence(self):
    # an empty list is NOT evidence of confidence: never invent a high dot
    self.assertAlmostEqual(disengage_confidence([], []), 0.0)
    self.assertAlmostEqual(disengage_confidence([], [0.5]), 0.0)
    self.assertAlmostEqual(disengage_confidence([0.5], []), 0.0)

  def test_result_is_clamped_to_the_unit_interval(self):
    self.assertAlmostEqual(disengage_confidence([-1.0], [-1.0]), 1.0)
    self.assertAlmostEqual(disengage_confidence([2.0], [0.0]), 0.0)

  def test_colour_zones_are_the_mici_thresholds(self):
    self.assertEqual(confidence_tier(CONFIDENCE_HIGH + 0.01), "high")
    self.assertEqual(confidence_tier(CONFIDENCE_HIGH), "medium")      # strict >
    self.assertEqual(confidence_tier(CONFIDENCE_LOW + 0.01), "medium")
    self.assertEqual(confidence_tier(CONFIDENCE_LOW), "low")
    self.assertEqual(confidence_tier(0.0), "low")

  def test_dot_travels_bottom_to_top(self):
    top, bottom = 100.0, 500.0
    self.assertAlmostEqual(dot_y(0.0, top, bottom), bottom)
    self.assertAlmostEqual(dot_y(1.0, top, bottom), top)
    self.assertAlmostEqual(dot_y(0.5, top, bottom), (top + bottom) / 2)
    # out-of-range values clamp instead of drawing off the track
    self.assertAlmostEqual(dot_y(-1.0, top, bottom), bottom)
    self.assertAlmostEqual(dot_y(2.0, top, bottom), top)


class TestHoldTimer(unittest.TestCase):
  def test_clock_is_the_same_m_ss_the_stopped_circle_shows(self):
    for seconds, expected in ((0, "0:00"), (10, "0:10"), (60, "1:00"), (90, "1:30"),
                              (110, "1:50"), (120, "2:00"), (3661, "61:01")):
      self.assertEqual(format_hold_time(seconds), expected)

  def test_clock_truncates_and_never_goes_negative(self):
    self.assertEqual(format_hold_time(59.99), "0:59")
    self.assertEqual(format_hold_time(-3.0), "0:00")

  def test_progress_is_scaled_to_the_2_minute_release(self):
    self.assertEqual(HOLD_TIMER_TOTAL_S, 120.0)
    self.assertAlmostEqual(hold_progress(0.0), 0.0)
    self.assertAlmostEqual(hold_progress(60.0), 0.5)
    self.assertAlmostEqual(hold_progress(120.0), 1.0)
    self.assertAlmostEqual(hold_progress(600.0), 1.0)   # saturates, never overflows the bar

  def test_colour_follows_the_fork_countdown_tiers(self):
    self.assertEqual(HOLD_TIMER_AMBER_S, 60.0)
    self.assertEqual(HOLD_TIMER_RED_S, 90.0)
    def rgb(c):
      return tuple(c)[:3] if isinstance(c, tuple) else (c.r, c.g, c.b)

    self.assertEqual(rgb(hold_timer_color(59.9)), (255, 255, 255))
    self.assertEqual(rgb(hold_timer_color(60.0)), (255, 175, 3))
    self.assertEqual(rgb(hold_timer_color(90.0)), (255, 64, 64))
    self.assertEqual(rgb(hold_timer_color(600.0)), (255, 64, 64))

  def test_pill_sits_in_the_top_right_corner_and_clears_the_alert_title_band(self):
    class Rect:
      def __init__(self, x, y, width, height):
        self.x, self.y, self.width, self.height = x, y, width, height

    rect = Rect(20.0, 20.0, 2120.0, 1040.0)
    box = pill_rect(rect)
    self.assertAlmostEqual(box.x + box.width, rect.x + rect.width - PILL_MARGIN)
    self.assertAlmostEqual(box.y, rect.y + PILL_MARGIN)
    self.assertEqual((box.width, box.height), (PILL_WIDTH, PILL_HEIGHT))
    # Full alerts draw their title at y + 200 (base AlertRenderer._draw_text);
    # the pill has to end above that so it never lands on the title.
    self.assertLess(box.y + box.height, rect.y + 200)


class TestWiring(unittest.TestCase):
  """The overlay must be drawn LAST, or a full-screen alert covers the timer."""

  def test_base_view_calls_the_sp_overlay_after_the_alert_renderer(self):
    tree = ast.parse(BASE_VIEW.read_text())
    render = None
    for node in ast.walk(tree):
      if isinstance(node, ast.FunctionDef) and node.name == "_render":
        render = node
    self.assertIsNotNone(render, "AugmentedRoadView._render not found")
    seg = ast.get_source_segment(BASE_VIEW.read_text(), render)

    alert_call = seg.index("self.alert_renderer.render(")
    overlay_call = seg.index("AugmentedRoadViewSP.render_top_overlays(self, self._content_rect)")
    self.assertGreater(overlay_call, alert_call,
                       "the top overlay must be drawn AFTER the alert renderer")

    # and it must be behind the sunnypilot-UI guard, like the existing SP hooks
    guard = seg.index("if gui_app.sunnypilot_ui():")
    self.assertLess(guard, overlay_call)
    self.assertGreater(overlay_call, guard)
    self.assertLess(overlay_call - guard, 120, "overlay call should sit inside the sunnypilot_ui() block")

  def test_hold_timer_and_confidence_are_wired_into_the_sp_mixin(self):
    sp_view = (REPO / "openpilot" / "selfdrive" / "ui" / "sunnypilot" / "onroad" / "augmented_road_view.py").read_text()
    self.assertIn("render_top_overlays", sp_view)
    self.assertIn("self._hold_timer.render(", sp_view)
    self.assertIn("self._confidence_meter.render(", sp_view)
    # the timer pill is gated on an alert being up (otherwise the circle shows it)
    self.assertIn("AlertSize.none", sp_view)
    self.assertIn("ui_state.standstill_timer", sp_view)

  def test_confidence_has_a_toggle_and_defaults_on(self):
    keys = (REPO / "openpilot" / "common" / "params_keys.h").read_text()
    self.assertIn('{"ConfidenceMeter", {PERSISTENT | BACKUP, BOOL, "1"}}', keys)
    visuals = (REPO / "openpilot" / "selfdrive" / "ui" / "sunnypilot" / "layouts" / "settings" / "visuals.py").read_text()
    self.assertIn('"ConfidenceMeter": (', visuals)
    ui_state = (REPO / "openpilot" / "selfdrive" / "ui" / "sunnypilot" / "ui_state.py").read_text()
    self.assertIn('self.params.get_bool("ConfidenceMeter")', ui_state)


if __name__ == "__main__":
  unittest.main()
