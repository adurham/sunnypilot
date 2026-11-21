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
import math
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

  def test_the_new_key_is_under_the_compiled_libparams_guard(self):
    # review fix: the UI reads ConfidenceMeter while it builds Settings > Display at startup, so a stale libparams_c.so
    # (no such key) would stop the UI from starting; test_param_reachability checks every FORK_KEY_PREFIXES key against
    # the REAL compiled registry, and CI runs it on the prebuilt tree
    from openpilot.sunnypilot.fork.tests import check_param_reachability as CPR
    self.assertTrue("ConfidenceMeter".startswith(CPR.FORK_KEY_PREFIXES))


# --- review fixes (2026-11-20) ------------------------------------------------------------------------------------------
from openpilot.selfdrive.ui.sunnypilot.onroad.confidence_meter import (  # noqa: E402
  DEV_UI_RIGHT_COLUMN_WIDTH, DOT_RADIUS, EDGE_INSET, dot_center_inset,
)
from openpilot.selfdrive.ui.sunnypilot.onroad.developer_ui import DeveloperUiState  # noqa: E402

DEV_UI = REPO / "openpilot" / "selfdrive" / "ui" / "sunnypilot" / "onroad" / "developer_ui" / "__init__.py"
CIRCULAR = REPO / "openpilot" / "selfdrive" / "ui" / "sunnypilot" / "onroad" / "circular_alerts.py"
RING = math.ceil(DOT_RADIUS * math.sqrt(2)) + 1     # the black masking ring draw_gradient_dot draws around the dot


def _local_int(path, func, name):
  """the int literal assigned to `name` inside function `func` of `path` (pins our copy of another file's geometry)."""
  for node in ast.walk(ast.parse(path.read_text())):
    if isinstance(node, ast.FunctionDef) and node.name == func:
      for sub in ast.walk(node):
        if isinstance(sub, ast.Assign) and any(isinstance(t, ast.Name) and t.id == name for t in sub.targets):
          return ast.literal_eval(sub.value)
  raise AssertionError(f"{name} not found in {func}")


class TestDotClearsTheDeveloperColumn(unittest.TestCase):
  """With Developer UI = Right/Both (the owner runs Both) the dot must not sit on the column's labels and units."""

  def test_column_geometry_is_the_developer_ui_one(self):
    width = _local_int(DEV_UI, "_draw_right_dev_ui", "container_width")
    border = _local_int(DEV_UI, "_draw_right_dev_ui", "UI_BORDER_SIZE")
    self.assertEqual(DEV_UI_RIGHT_COLUMN_WIDTH, width + 2 * border)   # x = right - width - 2*border

  def test_track_moves_left_of_the_column_only_when_it_is_shown(self):
    self.assertEqual(dot_center_inset(DeveloperUiState.OFF), EDGE_INSET)
    self.assertEqual(dot_center_inset(DeveloperUiState.BOTTOM), EDGE_INSET)
    for mode in (DeveloperUiState.RIGHT, DeveloperUiState.BOTH):
      # the ring's right edge stays >= 16 px left of the column's left edge
      self.assertGreaterEqual(dot_center_inset(mode) - RING, DEV_UI_RIGHT_COLUMN_WIDTH + 16)

  def test_without_the_column_the_dot_still_clears_the_stopped_circle(self):
    # CircularAlertsRenderer: centre x = right - 250 - adj - 3*UI_BORDER_SIZE(30), radius 250, adj 100 without the column
    circle_right_edge_inset = 250 + 100 + 3 * 30 - 250
    self.assertGreater(circle_right_edge_inset - (EDGE_INSET + RING), 100)
    self.assertIn("dev_ui_width_adjustment = 180 if ui_state.developer_ui in (DeveloperUiState.RIGHT, DeveloperUiState.BOTH) "
                  "else 100", CIRCULAR.read_text())


class TestTopOverlayGuard(unittest.TestCase):
  """A raise in either new element skips that frame only: render_top_overlays never raises, each element logs its first
  failure once, and both draw again on the next frame (retry, not a latch)."""

  def setUp(self):
    from openpilot.cereal import messaging
    import openpilot.selfdrive.ui.sunnypilot.onroad.augmented_road_view as sp_view_mod
    from openpilot.selfdrive.ui.ui_state import ui_state
    self.mod, self.ui_state = sp_view_mod, ui_state
    self._saved = {k: getattr(ui_state, k, None) for k in ("sm", "started", "confidence_meter", "standstill_timer")}
    sd = messaging.new_message("selfdriveState")
    sd.selfdriveState.alertSize = "small"           # "stop complete - hold brake pedal" is up during every hold
    cs = messaging.new_message("carState")
    cs.carState.standstill = True
    setattr(ui_state, "sm", {"selfdriveState": sd.selfdriveState.as_reader(), "carState": cs.carState.as_reader()})
    ui_state.started, ui_state.confidence_meter, ui_state.standstill_timer = True, True, True
    self.logged = []
    self._orig_log = sp_view_mod.cloudlog
    setattr(sp_view_mod, "cloudlog", type("L", (), {"exception": lambda _s, *a, **k: self.logged.append(a[1])})())

  def tearDown(self):
    setattr(self.mod, "cloudlog", self._orig_log)
    for k, v in self._saved.items():
      setattr(self.ui_state, k, v)

  def _view(self, fail_frames):
    drawn = []

    class Flaky:
      def __init__(self, name):
        self.name, self.calls = name, 0

      def update(self):
        pass

      def render(self, *_a):
        self.calls += 1
        if self.calls in fail_frames:
          raise RuntimeError(f"{self.name} injected failure")
        drawn.append((self.name, self.calls))

    view = object.__new__(self.mod.AugmentedRoadViewSP)    # no textures/fonts: only the overlay path is under test
    view._confidence_meter, view._hold_timer = Flaky("dot"), Flaky("pill")
    view._hud_renderer = type("H", (), {"standstill_elapsed_time": 12.0})()
    view._top_overlay_errors_logged = set()
    return view, drawn

  def test_a_raise_skips_one_frame_logs_once_and_the_next_frame_draws(self):
    view, drawn = self._view(fail_frames={1, 3})
    for _ in range(4):
      view.render_top_overlays(None)                     # must never raise
    self.assertEqual(drawn, [("dot", 2), ("pill", 2), ("dot", 4), ("pill", 4)])
    self.assertEqual(sorted(self.logged), ["confidence meter", "hold timer"])   # once per element, not per frame

  def test_one_element_failing_does_not_stop_the_other(self):
    view, drawn = self._view(fail_frames={1})
    view._hold_timer.calls = 1                           # the pill is past its failing frame: only the dot raises
    view.render_top_overlays(None)
    self.assertEqual(drawn, [("pill", 2)])
    self.assertEqual(self.logged, ["confidence meter"])


if __name__ == "__main__":
  unittest.main()
