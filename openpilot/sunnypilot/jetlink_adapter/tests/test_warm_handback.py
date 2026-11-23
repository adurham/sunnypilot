"""
The warm hand-back, opted into by this fork's adapter (FORK.md row 156).

A hand-back is the moment the phone's large model stops driving and the comma's small model takes over. jetlink used
to reset the small model there ("cold"), and its first 4 frames then planned from an empty history; on the drive of
2026-11-21 that jumped the steering command by 0.40 to 0.65 m/s^2 at three hand-backs with steering active. With the
adapter's optional warm_handback() answering True, jetlink (joining.py, from 6a40fe4) runs the small model unpublished
from the frame after the large model's first held frame, so a hand-back finds it "warm" and does not reset it.

Pinned here, against the jetlink checked out in jetlink_repo:
- the member exists in the pinned jetlink's interface, as an optional ModelSide member, and jetlink's joining model
  asks it (a jetlink without it would ignore the adapter's answer);
- the adapter answers True, and jetlink reads that answer as on;
- with the link off (JetlinkLink 0 or unset, the default) the warm hand-back changes nothing: no joining model is ever
  built, so nothing asks; test_default_off.py holds every hook to "as if jetlink were absent", and here the same is
  checked with the opt-in in place, the adapter's own warm_handback() spied on.
Pure Python: no native params library, no model.
"""
import inspect
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from jetlink.openpilot import interface, joining

from openpilot.sunnypilot import jetlink_adapter
from openpilot.sunnypilot.jetlink_adapter.tests.test_default_off import DefaultOff

Adapter = jetlink_adapter._adapter_class()


class TestThePinnedJetlinkHasIt(unittest.TestCase):
  def test_the_interface_member_exists_and_is_optional(self):
    self.assertIn('warm_handback', interface.members(interface.ModelSide))
    self.assertIn('warm_handback', interface.optional(interface.ModelSide))
    # a method taking nothing but self, as the adapter's does
    self.assertEqual(list(interface.members(interface.ModelSide)['warm_handback'].parameters), [])

  def test_the_joining_model_takes_it_and_caps_it(self):
    self.assertIn('warm_handback', inspect.signature(joining.JoiningModelState).parameters)
    self.assertIs(inspect.signature(joining.JoiningModelState).parameters['warm_handback'].default, False)
    self.assertTrue(callable(getattr(joining, 'warm_handback', None)))
    # the cap the review asked for (F1): at most 60 warm frames in any 10 s, counted across warm-ups
    self.assertEqual(joining.WARM_MAX, 60)
    self.assertEqual(joining.HOLD_WINDOW, 10.0)

  def test_jetlinks_stock_adapter_leaves_it_off(self):
    # off unless a fork opts in: the stock adapter jetlink ships has no warm_handback
    from jetlink.openpilot.stock import Stock
    self.assertFalse(hasattr(Stock, 'warm_handback'))


class TestTheAdapterOptsIn(unittest.TestCase):
  def test_the_adapter_answers_true(self):
    self.assertIs(Adapter().warm_handback(), True)
    self.assertIn('warm_handback', vars(Adapter))

  def test_it_still_conforms_to_jetlinks_interface(self):
    self.assertEqual(interface.conformance(Adapter(), interface.Openpilot), [])

  def test_jetlink_reads_the_answer_as_on_and_says_so(self):
    log = mock.MagicMock()
    self.assertIs(joining.warm_handback(Adapter(), log), True)
    log.warning.assert_called_once()
    self.assertIn('kept warm for a hand-back', log.warning.call_args[0][0])
    log.exception.assert_not_called()
    log.error.assert_not_called()

  def test_jetlinks_reading_is_off_for_anything_but_true(self):
    # the contract the opt-in rests on: no member, a raise, or any answer but True is off
    log = mock.MagicMock()

    class Raises:
      def warm_handback(self):
        raise RuntimeError('adapter broke')

    class Truthy:
      def warm_handback(self):
        return 1

    self.assertIs(joining.warm_handback(object(), log), False)
    self.assertIs(joining.warm_handback(Raises(), log), False)
    self.assertIs(joining.warm_handback(Truthy(), log), False)


class TestLinkOffIsUnchanged(DefaultOff, unittest.TestCase):
  """The hooks with the link off and the opt-in in place, as test_default_off.TestWithJetlink checks them: every hook
  still answers as if jetlink were absent, and nothing asks the adapter about the warm hand-back."""

  def setUp(self):
    super().setUp()
    spy = mock.patch.object(Adapter, 'warm_handback', autospec=True, return_value=True)
    self.asked = spy.start()
    self.addCleanup(spy.stop)

  def test_no_setting_is_off(self):
    super().test_no_setting_is_off()
    self.asked.assert_not_called()

  def test_zero_is_off(self):
    super().test_zero_is_off()
    self.asked.assert_not_called()

  def test_the_real_hooks_are_used_with_the_link_off(self):
    from jetlink.openpilot.hooks import Hooks
    self.assertIsInstance(self.hooks(), Hooks)
    self.check_inert()
    self.asked.assert_not_called()

  def test_the_joining_model_is_only_built_with_the_link_on(self):
    # attach() is the only way to the joining model, and so to the question; with the link off it hands modeld its own
    # model back, and jetlink's join() is never reached
    model = object()
    with mock.patch.object(joining, 'join', side_effect=AssertionError('join() with the link off')) as join:
      for value in (None, '0'):
        if value is not None:
          self.write_link(value)
        jetlink_adapter._hooks = None
        self.assertFalse(jetlink_adapter.prepare(False))
        self.assertIs(jetlink_adapter.attach(model, object(), 1928, 1208), model)
      join.assert_not_called()
    self.asked.assert_not_called()


if __name__ == '__main__':
  unittest.main()
