"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

17c FEEDBACK: the BAD gesture's decision rule (two-finger tap = one action, no counting, no timing).

The rule is pure (no pyray), so it is tested here directly; the widget that feeds it real mouse events is covered by
test_mark_wiring.py plus the on-device check.
"""
import unittest

from openpilot.sunnypilot.fork import mark_gesture as mg


class TestBadMarkGesture(unittest.TestCase):
  def test_two_fingers_tapping_is_a_bad_mark(self):
    self.assertTrue(mg.is_bad_mark(2, 0.12, 3.0))

  def test_three_fingers_are_also_a_bad_mark(self):
    self.assertTrue(mg.is_bad_mark(3, 0.2, 0.0), "the gesture is 'more than one finger'")

  def test_one_finger_is_not(self):
    self.assertFalse(mg.is_bad_mark(1, 0.1, 2.0), "a single tap keeps its upstream meaning (scroll to home)")

  def test_a_two_finger_SWIPE_is_not_a_tap(self):
    self.assertFalse(mg.is_bad_mark(2, 0.3, 200.0), "travelling that far is the bookmark swipe, not a tap")

  def test_a_resting_hand_is_not_a_tap(self):
    self.assertFalse(mg.is_bad_mark(2, 1.5, 1.0), "a hand left on the screen is not a verdict")

  def test_the_limits_are_inclusive(self):
    self.assertTrue(mg.is_bad_mark(2, mg.TAP_MAX_S, mg.TAP_MAX_PX), "exactly at the limits still counts")
    self.assertFalse(mg.is_bad_mark(2, mg.TAP_MAX_S + 0.01, 0.0))
    self.assertFalse(mg.is_bad_mark(2, 0.1, mg.TAP_MAX_PX + 0.01))

  def test_the_numbers_are_pinned_and_far_below_the_bookmark_swipe(self):
    # the UI disables scrolling / arms the bookmark at PEEK_THRESHOLD = 150 px (upstream augmented_road_view.py), so a
    # tap threshold has to sit far below it or a deliberate swipe could be read as a tap.
    self.assertEqual(2, mg.TWO_FINGER_TOUCHES)
    self.assertEqual(40, mg.TAP_MAX_PX)
    self.assertEqual(0.6, mg.TAP_MAX_S)
    self.assertLess(mg.TAP_MAX_PX, 150 / 2, "a tap must not reach anywhere near the swipe threshold")
    self.assertLess(mg.TAP_MAX_S, 1.0, "MARK_GAP_S is 2.0 s: a tap can never be mistaken for a burst's second press")


class TestTouchSession(unittest.TestCase):
  """The session bookkeeping the widget feeds: fingers landing, dragging and lifting, per SLOT.

  This is where the gesture is actually decided, and it is pure - so the two-finger tap is proven here rather than only
  on the device. (A first cut of the widget measured travel from the SESSION's start point, which read an ordinary
  two-finger tap - fingers 50-150 px apart by nature - as a 100 px swipe: `test_fingers_landing_apart_is_still_a_tap`
  is the regression guard for exactly that.)
  """

  def setUp(self):
    self.s = mg.TouchSession()
    self.t = 1000.0

  def _advance(self, dt):
    self.t += dt

  def _two_finger_tap(self, dt=0.12, apart=150.0):
    self.s.press(0, 900.0, 500.0, self.t)
    self._advance(0.02)
    self.s.press(1, 900.0 + apart, 500.0, self.t)
    self._advance(dt)
    self.assertFalse(self.s.release(1, 900.0 + apart, 500.0, self.t), "one finger still down: not over yet")
    self.assertTrue(self.s.release(0, 900.0, 500.0, self.t), "the last finger lifts -> the session is complete")
    return self.s.is_bad_mark()

  def test_two_finger_tap_is_bad(self):
    self.assertTrue(self._two_finger_tap())

  def test_fingers_landing_apart_is_still_a_tap(self):
    self.assertTrue(self._two_finger_tap(apart=170.0), "travel is per finger, not from the session's first finger")

  def test_one_finger_tap_is_not_bad(self):
    self.s.press(0, 900.0, 500.0, self.t)
    self._advance(0.12)
    self.assertTrue(self.s.release(0, 900.0, 500.0, self.t))
    self.assertFalse(self.s.is_bad_mark())

  def test_a_two_finger_swipe_is_not_bad(self):
    self.s.press(0, 900.0, 500.0, self.t)
    self.s.press(1, 1050.0, 500.0, self.t)
    self._advance(0.05)
    self.s.move(0, 700.0, 500.0, self.t)          # 200 px: past the bookmark swipe threshold
    self.s.move(1, 850.0, 500.0, self.t)
    self._advance(0.05)
    self.s.release(1, 850.0, 500.0, self.t)
    self.s.release(0, 700.0, 500.0, self.t)
    self.assertFalse(self.s.is_bad_mark())

  def test_two_fingers_resting_is_not_bad(self):
    self.s.press(0, 900.0, 500.0, self.t)
    self.s.press(1, 1050.0, 500.0, self.t)
    self._advance(1.5)
    self.s.release(1, 1050.0, 500.0, self.t)
    self.s.release(0, 900.0, 500.0, self.t)
    self.assertFalse(self.s.is_bad_mark())

  def test_a_third_finger_still_counts(self):
    self.assertTrue(self._two_finger_tap() and mg.is_bad_mark(3, 0.1, 0.0))

  def test_a_new_session_after_a_completed_one_starts_clean(self):
    self.assertTrue(self._two_finger_tap())
    self._advance(3.0)
    self.assertTrue(self._two_finger_tap(), "every gesture is its own verdict - nothing to count, nothing to time")
    self.assertEqual(2, self.s.touches, "the counters are per session, not cumulative")

  def test_a_drag_of_one_finger_is_ignored(self):
    self.s.move(0, 700.0, 500.0, self.t)          # a move for a finger that never landed: no session, no crash
    self.assertEqual(0, self.s.touches)
    self.assertFalse(self.s.active)

  def test_a_missed_release_does_not_kill_the_gesture(self):
    """The widget does not see every release (another screen, a dropped event, ui_state flipping mid-touch). A single
    stale finger must not leave the gesture dead until the process restarts."""
    self.s.press(0, 900.0, 500.0, self.t)         # a finger that never lifts
    self._advance(30.0)                           # long past any tap
    self.assertTrue(self._two_finger_tap(), "the next gesture must still count")

  def test_the_same_slot_landing_twice_starts_over(self):
    self.s.press(0, 900.0, 500.0, self.t)
    self._advance(0.05)
    self.s.press(0, 900.0, 500.0, self.t)         # a fresh landing on a slot already recorded as down
    self.assertEqual(1, self.s.touches, "a re-press of the same slot is a new session, not a second finger")
    self._advance(0.05)
    self.assertTrue(self.s.release(0, 900.0, 500.0, self.t))
    self.assertFalse(self.s.is_bad_mark())
