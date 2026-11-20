"""
Jetlink is DEFAULT OFF on this fork: with the JetlinkLink param at 0 (or never written), every hook the manager,
modelds, selfdrived and hardwared call must answer as if jetlink did not exist, so the driving path is as it was.
Checked with jetlink present (the real hooks, the link off), with it absent, and with it unusable. Pure Python.
"""
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

from openpilot.sunnypilot import jetlink_adapter


class DefaultOff:
  """The hooks with the link off. Subclasses set up the world they run in."""

  def hooks(self):
    return jetlink_adapter.hooks()

  def setUp(self):
    self._tmp = tempfile.TemporaryDirectory()
    self.addCleanup(self._tmp.cleanup)
    patcher = mock.patch.dict('os.environ', {'PARAMS_ROOT': self._tmp.name, 'OPENPILOT_PREFIX': 'd'})
    patcher.start()
    self.addCleanup(patcher.stop)
    self.params_dir = Path(self._tmp.name) / 'd'
    self.params_dir.mkdir()
    jetlink_adapter._hooks = None
    self.addCleanup(setattr, jetlink_adapter, '_hooks', None)

  def write_link(self, value):
    (self.params_dir / jetlink_adapter.KEYS.link).write_text(value)

  def check_inert(self):
    model, small = object(), object()
    for started in (False, True):
      self.assertFalse(jetlink_adapter.should_run(started, None, None))
    self.assertFalse(jetlink_adapter.prepare(False))
    self.assertFalse(jetlink_adapter.prepare(True))
    self.assertIs(jetlink_adapter.attach(model, small, 1928, 1208), model)    # the model modeld built runs, unchanged
    self.assertFalse(jetlink_adapter.handed_over(model))                      # no warmup restart
    self.assertEqual(jetlink_adapter.state(model), 'none')
    self.assertIsNone(jetlink_adapter.reason())                               # no alert
    # jetlink's shutdown_step latches once it says yes ("True once it may, and from then on"), so the no comes first
    self.assertFalse(jetlink_adapter.shutdown_step(False, ''))                # no power-off is invented
    self.assertTrue(jetlink_adapter.shutdown_step(True, 'offroad'))           # a wanted power-off goes ahead at once
    swaps = jetlink_adapter.swap_events(0.01)
    if swaps is not None:   # the real hooks hand a tracker with no events behind it
      # every transition selfdrived can feed it: no flag, and never a settling that would forgive a comm/localizer fault
      for fresh, big, in_control in [(f, b, c) for f in (True, False) for b in (True, False) for c in (True, False)] * 3:
        flags = swaps.update(fresh, big, in_control)
        self.assertFalse(any(flags), (fresh, big, in_control))
        self.assertFalse(swaps.settling)

  def test_no_setting_is_off(self):
    self.assertFalse(jetlink_adapter.link_on())
    self.check_inert()

  def test_zero_is_off(self):
    self.write_link('0')
    self.assertFalse(jetlink_adapter.link_on())
    self.check_inert()


class TestWithJetlink(DefaultOff, unittest.TestCase):
  def test_the_real_hooks_are_used_with_the_link_off(self):
    from jetlink.openpilot.hooks import Hooks
    self.assertIsInstance(self.hooks(), Hooks)


class TestWithoutJetlink(DefaultOff, unittest.TestCase):
  def setUp(self):
    super().setUp()
    # `import jetlink` finds nothing: no checkout, as on a device whose submodule is not fetched
    patcher = mock.patch.dict(sys.modules, {'jetlink': None, 'jetlink.openpilot': None, 'jetlink.openpilot.hooks': None})
    patcher.start()
    self.addCleanup(patcher.stop)

  def test_no_checkout_is_the_link_off_quietly(self):
    self.assertIsInstance(self.hooks(), jetlink_adapter._Off)
    self.assertEqual(self.hooks().why, '')

  def test_someone_who_turned_the_link_on_is_told_why_when_it_cannot_run(self):
    self.write_link('2')
    jetlink_adapter._hooks = jetlink_adapter._Off('jetlink package too old for this build')
    self.assertEqual(jetlink_adapter.reason(), 'jetlink package too old for this build')
    # and the drive is unaffected: still the small model, still a power-off that goes ahead
    model = object()
    self.assertIs(jetlink_adapter.attach(model, None, 1, 1), model)
    self.assertFalse(jetlink_adapter.prepare(True))
    self.assertTrue(jetlink_adapter.shutdown_step(True, ''))
    self.assertFalse(jetlink_adapter.should_run(True, None, None))


class TestUnusableJetlink(DefaultOff, unittest.TestCase):
  def test_another_api_is_the_link_off_with_a_reason(self):
    with mock.patch.object(jetlink_adapter, 'API', 99), mock.patch.object(jetlink_adapter, '_cloudlog',
                                                                         return_value=mock.MagicMock()):
      self.write_link('2')
      hooks = self.hooks()
      self.assertIsInstance(hooks, jetlink_adapter._Off)
      self.assertIn('99', jetlink_adapter.reason())
      self.check_reasonless_drive()

  def check_reasonless_drive(self):
    model = object()
    self.assertIs(jetlink_adapter.attach(model, None, 1, 1), model)
    self.assertFalse(jetlink_adapter.prepare(True))
    self.assertTrue(jetlink_adapter.shutdown_step(True, ''))


class TestTheHooksNeverRaise(unittest.TestCase):
  def test_an_adapter_that_throws_is_the_link_off(self):
    from jetlink.openpilot.hooks import Hooks

    def boom():
      raise RuntimeError('adapter broke')

    with tempfile.TemporaryDirectory() as root, mock.patch.dict('os.environ', {'PARAMS_ROOT': root, 'OPENPILOT_PREFIX': 'd'}):
      Path(root, 'd').mkdir()
      Path(root, 'd', jetlink_adapter.KEYS.link).write_text('2')
      hooks = Hooks(jetlink_adapter.API, boom, link_on=jetlink_adapter.link_on, log=lambda: mock.MagicMock())
      model = SimpleNamespace()
      self.assertFalse(hooks.prepare(False))
      self.assertIs(hooks.attach(model, None, 1, 1), model)
      self.assertTrue(hooks.shutdown_step(True, ''))


if __name__ == '__main__':
  unittest.main()
