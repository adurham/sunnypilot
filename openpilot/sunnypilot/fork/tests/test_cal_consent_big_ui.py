"""fork: the FCA11-cal driver-consent prompt on the BIG UI (tici / tizi -- the owner's comma 3X).

The 0033 prompt was wired only into the mici UI, which never runs on a tizi (``gui_app.big_ui()``), so it could never
appear on this car. These tests pin the big-UI swap:

  * static: ``layouts/main.py`` swaps ``AugmentedRoadViewCal`` in behind ``gui_app.sunnypilot_ui()``;
  * the swapped view constructs with the (unchanged) ``CalConsentWidget``;
  * NO pending file -> the widget is invisible and touch-INERT: a press where the pill would be still reaches the base
    view (sidebar toggle), no consent file is ever written, nothing is drawn;
  * a synthetic ``pending.json`` -> the pill becomes visible and DRAWS, with no actuation (no ``consent.json``) until
    a real tap; a tap on TAP TO FIRE writes a bound ``accept: true`` and does NOT also toggle the sidebar;
    the dismiss target writes ``accept: false``;
  * ``ui_state.started`` False -> nothing polled, nothing drawn.

Runs headless (hidden raylib window); no camera, no device. Each test drives the REAL ``Widget.render`` path with
synthetic ``MouseEvent``s, so the press/release ordering between the prompt and the base view is the production one.
"""
import ast
import functools
import json
import os
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

import pyray as rl

REPO = Path(__file__).resolve().parents[4]
MAIN = REPO / "openpilot" / "selfdrive" / "ui" / "layouts" / "main.py"
W, H = 2160, 1080   # tici / tizi screen


class TestBigUiSwapStatic(unittest.TestCase):
  def test_main_layout_swaps_cal_view_behind_sunnypilot_ui(self):
    tree = ast.parse(MAIN.read_text())
    found = False
    for node in tree.body:
      if isinstance(node, ast.If) and ast.unparse(node.test) == "gui_app.sunnypilot_ui()":
        for imp in node.body:
          if isinstance(imp, ast.ImportFrom) and imp.module == "openpilot.selfdrive.ui.sunnypilot.onroad.cal_consent_road_view":
            found = any(a.name == "AugmentedRoadViewCal" and a.asname == "AugmentedRoadView" for a in imp.names)
    self.assertTrue(found, "layouts/main.py must swap AugmentedRoadViewCal in as AugmentedRoadView behind sunnypilot_ui()")


class TestBigUiCalConsent(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    from openpilot.common.prefix import OpenpilotPrefix
    cls.prefix = OpenpilotPrefix()   # isolated params dir: defaults only (IsMetric False == the owner's car)
    cls.prefix.__enter__()
    rl.set_config_flags(rl.ConfigFlags.FLAG_WINDOW_HIDDEN)
    from openpilot.system.ui.lib.application import gui_app
    from openpilot.selfdrive.ui.ui_state import ui_state
    cls.gui_app = gui_app
    gui_app.init_window("cal-consent-big-ui-test")
    ui_state.update_params()   # what ui.py's loop does before the first frame (populates the SP toggles)
    cls.target = rl.load_render_texture(W, H)

  @classmethod
  def tearDownClass(cls):
    rl.unload_render_texture(cls.target)
    cls.gui_app.close()   # close the GL window now, not at interpreter shutdown
    cls.prefix.__exit__(None, None, None)

  def setUp(self):
    from openpilot.selfdrive.ui.ui_state import ui_state
    from openpilot.selfdrive.ui.sunnypilot.onroad import cal_consent_road_view as mod
    from openpilot.selfdrive.ui.sunnypilot.mici.onroad.cal_consent_widget import CalConsentWidget
    self.ui_state = ui_state
    self._prev_started = ui_state.started
    ui_state.started = True
    self.tmp = tempfile.TemporaryDirectory()
    self.dir = self.tmp.name
    # construct through the REAL constructor; only the widget's cal dir is redirected
    with mock.patch.object(mod, "CalConsentWidget", functools.partial(CalConsentWidget, cal_dir=self.dir)):
      self.view = mod.AugmentedRoadViewCal()
    self.clicks = 0

    def _click():
      self.clicks += 1
    self.view.set_click_callback(_click)
    self.gui_app._mouse_events = []

  def tearDown(self):
    self.gui_app._mouse_events = []
    self.ui_state.started = self._prev_started
    self.view.close()
    # Upstream CameraView.__del__ calls close() again, and the view stays referenced by ui_state's offroad-transition
    # callback list until interpreter shutdown -- by then the GL context is gone and rl.unload_shader() SIGABRTs
    # (reproducible on the UNPATCHED view, out/base_shutdown_abort_probe.py). Drop the already-unloaded handle.
    self.view.shader = None
    self.tmp.cleanup()

  # ------------------------------------------------------------------ helpers
  def _frame(self, events=()):
    from openpilot.system.ui.lib.application import MouseEvent, MousePos
    self.gui_app._mouse_events = [MouseEvent(MousePos(x, y), 0, kind == "press", kind == "release", kind == "press",
                                             time.monotonic()) for kind, x, y in events]
    self.view._cal_consent._last_poll = -1e9   # defeat the 5 Hz throttle so each frame re-reads the file
    rl.begin_texture_mode(self.target)
    rl.clear_background(rl.BLACK)
    self.view.render(rl.Rectangle(0, 0, W, H))
    rl.end_texture_mode()
    self.gui_app._mouse_events = []

  def _pixels_in(self, r: rl.Rectangle) -> int:
    """Count non-black-ish pixels inside rect r of the last frame (render textures are y-flipped)."""
    img = rl.load_image_from_texture(self.target.texture)
    try:
      n = 0
      for y in range(int(r.y) + 8, int(r.y + r.height) - 8, 6):
        for x in range(int(r.x) + 8, int(r.x + r.width) - 8, 12):
          c = rl.get_image_color(img, x, H - 1 - y)
          n += (c.r + c.g + c.b) > 150
      return n
    finally:
      rl.unload_image(img)

  def _write_pending(self, **over):
    doc = {"pending": True, "rep_id": "r2-step-24-60", "level_lsb": 24, "level_g": 0.24, "band_kph": 60.0,
           "seq": 7, "plan_id": "cal-test", "ts": round(time.time(), 3)}  # noqa: TID251 - wall clock, as cal_mode writes it
    doc.update(over)
    with open(os.path.join(self.dir, "pending.json"), "w") as f:
      json.dump(doc, f)

  def _consent(self):
    p = os.path.join(self.dir, "consent.json")
    if not os.path.exists(p):
      return None
    with open(p) as f:
      return json.load(f)

  def _center(self, r):
    return r.x + r.width / 2, r.y + r.height / 2

  # ------------------------------------------------------------------ tests
  def test_constructs_with_widget(self):
    from openpilot.selfdrive.ui.sunnypilot.mici.onroad.cal_consent_widget import CalConsentWidget
    from openpilot.selfdrive.ui.onroad.augmented_road_view import AugmentedRoadView
    self.assertIsInstance(self.view, AugmentedRoadView)
    self.assertIsInstance(self.view._cal_consent, CalConsentWidget)
    self.assertEqual(self.view._cal_consent.dir, self.dir)
    self.assertFalse(self.view._cal_consent.is_visible)

  def test_no_pending_is_invisible_and_touch_inert(self):
    self._frame()
    w = self.view._cal_consent
    self.assertFalse(w.is_prompting())
    self.assertFalse(w.is_visible)
    pill = w._pill_rect()   # where the pill WOULD be (widget rect = the content rect)
    tx, ty = self._center(w._tap_rect())
    self._frame([("press", tx, ty)])
    self._frame([("release", tx, ty)])
    self.assertEqual(self.clicks, 1, "with nothing pending a press on the pill area must reach the base view (sidebar)")
    self.assertIsNone(self._consent(), "an idle prompt must never write consent.json")
    self.assertFalse(w.interacting())
    self.assertEqual(self._pixels_in(pill), 0, "nothing may be drawn where the pill would be")

  def test_malformed_or_not_pending_stays_hidden(self):
    for body in ("{not json", "[]", json.dumps({"pending": False, "rep_id": "x"})):
      with open(os.path.join(self.dir, "pending.json"), "w") as f:
        f.write(body)
      self._frame()
      self.assertFalse(self.view._cal_consent.is_prompting(), body)
    self.assertIsNone(self._consent())

  def test_pending_draws_pill_without_actuation(self):
    self._write_pending()
    self._frame()
    w = self.view._cal_consent
    self.assertTrue(w.is_prompting())
    self.assertTrue(w.is_visible)
    self.assertGreater(self._pixels_in(w._tap_rect()), 20, "the TAP TO FIRE button must actually be drawn")
    # several frames with no touch: still pending, never a consent write
    for _ in range(5):
      self._frame()
    self.assertTrue(w.is_prompting())
    self.assertIsNone(self._consent(), "a pending prompt alone must never actuate")
    self.assertEqual(self.clicks, 0)

  def test_tap_to_fire_writes_bound_accept_and_does_not_toggle_sidebar(self):
    self._write_pending()
    self._frame()
    tx, ty = self._center(self.view._cal_consent._tap_rect())
    self._frame([("press", tx, ty)])
    self.assertEqual(self.clicks, 0, "a press on the prompt must not also toggle the sidebar")
    self.assertIsNone(self._consent(), "consent is written on RELEASE inside the button, not on press")
    self._frame([("release", tx, ty)])
    doc = self._consent()
    self.assertIsNotNone(doc)
    self.assertIs(doc["accept"], True)
    self.assertEqual((doc["rep_id"], doc["plan_id"], doc["seq"]), ("r2-step-24-60", "cal-test", 7))
    self.assertEqual(self.clicks, 0)
    # a later, unrelated press elsewhere still reaches the base view (the guard is consume-once)
    self._frame([("press", 300, 300)])
    self.assertEqual(self.clicks, 1)

  def test_dismiss_writes_reject(self):
    self._write_pending()
    self._frame()
    dx, dy = self._center(self.view._cal_consent._dismiss_rect())
    self._frame([("press", dx, dy)])
    self._frame([("release", dx, dy)])
    doc = self._consent()
    self.assertIsNotNone(doc)
    self.assertIs(doc["accept"], False)
    self.assertEqual(self.clicks, 0)
    self.assertFalse(self.view._cal_consent.is_prompting())

  def test_press_elsewhere_while_pending_still_reaches_base(self):
    self._write_pending()
    self._frame()
    self._frame([("press", 300, 300)])
    self._frame([("release", 300, 300)])
    self.assertEqual(self.clicks, 1)
    self.assertIsNone(self._consent())

  def test_every_drawn_char_has_a_real_glyph(self):
    """raylib draws a missing codepoint as '?': the deployed pill showed '?' for its dismiss mark and arrows."""
    from openpilot.selfdrive.ui.sunnypilot.mici.onroad import cal_consent_widget as wmod
    drawn = []
    real = rl.draw_text_ex

    def spy(font, text, *a):
      drawn.append((font, text))
      return real(font, text, *a)
    self._write_pending()
    with mock.patch.object(wmod.rl, "draw_text_ex", spy):
      self._frame()
    self.assertTrue(any("CAL REP READY" in t for _, t in drawn))
    missing = {(t, ch) for font, t in drawn for ch in t
               if not ch.isspace() and font.glyphs[rl.get_glyph_index(font, ord(ch))].value != ord(ch)}
    self.assertEqual(missing, set(), "characters with no glyph in the loaded font (render as '?')")

  def test_not_started_does_not_poll_or_draw(self):
    self._write_pending()
    self.ui_state.started = False
    self._frame()
    self.assertFalse(self.view._cal_consent.is_prompting())


if __name__ == "__main__":
  unittest.main()
