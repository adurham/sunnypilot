"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

17c FEEDBACK: the wiring net for the two-finger BAD gesture.

The gesture only works if FOUR things stay connected, and three of them live in files no unit test can execute on the
Mac (raylib widgets + the app loop). A silent break in any link would look exactly like "the driver's marks are
missing", which is the failure this whole lane exists to prevent, so the chain is asserted here:

  layout  _on_bad_mark  --publishes-->  bookmarkButton ALONE
  layout  ...           --passes---->  AugmentedRoadView(bad_mark_callback=...)
  view    AugmentedRoadViewSP --installs--> BookmarkIconSP(bad_mark_callback)
  icon    BookmarkIconSP --feeds---->  mark_gesture.is_bad_mark
  typing  lone bookmarkButton --becomes--> verdict bad, source "touch2"

Static source assertions are deliberate: they catch an unwiring, not a behaviour change (behaviour is covered by
test_mark_gesture.py, test_driver_mark.py and test_driver_mark_hook.py).
"""
import ast
import pathlib
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[4]          # the superproject root (this file: openpilot/sunnypilot/fork/tests/)


def _src(rel: str) -> str:
  return (ROOT / rel).read_text()


def _init_args(rel: str, class_name: str) -> list[str]:
  """The parameter names of a class's ``__init__``, read from the source with AST (never imported: importing a mici UI
  module initialises raylib, which segfaults in a test process that already loaded msgq)."""
  for node in ast.walk(ast.parse(_src(rel))):
    if isinstance(node, ast.ClassDef) and node.name == class_name:
      for fn in node.body:
        if isinstance(fn, ast.FunctionDef) and fn.name == "__init__":
          names = [a.arg for a in fn.args.args] + [a.arg for a in fn.args.kwonlyargs]
          names += [a.arg for a in (fn.args.vararg, fn.args.kwarg) if a is not None]
          return names
  raise AssertionError(f"{class_name}.__init__ not found in {rel}")


class TestBadMarkWiring(unittest.TestCase):
  def test_the_layout_publishes_the_button_half_alone_for_a_bad_mark(self):
    src = _src("openpilot/selfdrive/ui/mici/layouts/main.py")
    self.assertIn("def _on_bad_mark(self)", src, "the layout must own the BAD gesture's publisher")
    body = src.split("def _on_bad_mark(self)", 1)[1].split("\n  def ", 1)[0]
    self.assertIn("messaging.new_message('bookmarkButton'", body)
    self.assertIn("self._pm.send('bookmarkButton'", body)
    self.assertNotIn("messaging.new_message('userBookmark'", body,
                     "a lone bookmarkButton IS the bad verdict: publishing the bookmark half too would make it GOOD")
    self.assertNotIn("self._pm.send('userBookmark'", body)

  def test_the_layout_hands_the_callback_to_the_road_view(self):
    src = _src("openpilot/selfdrive/ui/mici/layouts/main.py")
    self.assertIn("bad_mark_callback=self._on_bad_mark", src)

  def test_the_base_view_carries_the_callback(self):
    src = _src("openpilot/selfdrive/ui/mici/onroad/augmented_road_view.py")
    self.assertIn("bad_mark_callback=None", src)
    self.assertIn("self._bad_mark_callback = bad_mark_callback", src)

  def test_the_sunnypilot_view_installs_the_gesture_icon(self):
    src = _src("openpilot/selfdrive/ui/sunnypilot/mici/onroad/augmented_road_view.py")
    self.assertIn("from openpilot.selfdrive.ui.sunnypilot.mici.onroad.bookmark_icon import BookmarkIconSP", src)
    self.assertIn("self._bookmark_icon = BookmarkIconSP(bookmark_callback, bad_mark_callback)", src)

  def test_the_icon_feeds_the_pure_rule_and_guards_offroad(self):
    src = _src("openpilot/selfdrive/ui/sunnypilot/mici/onroad/bookmark_icon.py")
    self.assertIn("mark_gesture.TouchSession()", src, "the bookkeeping itself lives in the pure module")
    self.assertIn("self._session.is_bad_mark()", src)
    self.assertIn("ui_state.started", src, "no gestures offroad: the same guard the upstream handler applies")
    self.assertIn("self._multi_touch = True", src, "without multi-touch the second finger never reaches the widget")
    self.assertIn("self._interacting = True", src, "the release must be consumed or the view also scrolls home")
    for flag in ("mouse_event.left_pressed", "mouse_event.left_down", "mouse_event.left_released"):
      self.assertIn(flag, src, f"the session must see {flag}")

  def test_the_hook_types_by_service_not_by_count(self):
    src = _src("openpilot/sunnypilot/fork/driver_mark_hook.py")
    self.assertIn("MARK_SERVICES = driver_mark.MARK_SERVICES", src,
                  "the hook and the UI adapter must share one service list")
    self.assertIn("self._typing.press(int(now_nanos), service)", src)
    self.assertNotIn("driver_mark.mark_source(", src, "the source now comes from the typing, which knows the gesture")

  def test_the_views_keep_their_arguments_in_the_right_slots(self):
    # The base view takes (bookmark_callback, bad_mark_callback, stream_type) and the sunnypilot view forwards all three,
    # so a later reorder of either signature would put a callback where the stream type belongs. Parsed, not imported:
    # importing this module initialises raylib, which is not safe in a test process that already loaded msgq.
    self.assertEqual(["self", "bookmark_callback", "bad_mark_callback", "stream_type"],
                     _init_args("openpilot/selfdrive/ui/mici/onroad/augmented_road_view.py", "AugmentedRoadView"))
    self.assertEqual(["self", "bookmark_callback", "bad_mark_callback", "args", "kwargs"],
                     _init_args("openpilot/selfdrive/ui/sunnypilot/mici/onroad/augmented_road_view.py",
                                "AugmentedRoadViewSP"))
