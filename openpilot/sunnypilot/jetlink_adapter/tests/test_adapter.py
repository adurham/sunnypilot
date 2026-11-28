"""
Derived from zoompilot's jetlink_adapter tests (Copyright (c) 2026-, Zeph Leggett, MIT License).

The adapter against this fork: jetlink tests its side against a fake and installs no openpilot, so whatever jetlink
relies on of this fork is pinned here, where a sync that moves it fails. Pure Python: no native params library.
"""
import ast
import json
import re
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import numpy as np
import jetlink.openpilot as jl
from jetlink.openpilot.interface import OwnerConfig, load_adapter
from jetlink.openpilot.testing import check_adapter

from openpilot.sunnypilot import jetlink_adapter
from openpilot.sunnypilot.jetlink_adapter import KEYS

OPENPILOT = Path(__file__).resolve().parents[3]  # .../openpilot
Adapter = jetlink_adapter._adapter_class()
# what the resident owner must never import: swaglog pulls numpy, capnp and zmq in to publish a log line and costs
# 28 MB, and params imports swaglog. Measured on the comma: the owner is 10 MB, the daemon that imported the world
# was 47.5 MB
HEAVY = ('numpy', 'capnp', 'zmq', 'cereal', 'msgq', 'tinygrad', 'openpilot.cereal', 'openpilot.common.params',
         'openpilot.common.swaglog')


class TestTheInterface(unittest.TestCase):
  def test_it_is_jetlinks_adapter_module(self):
    # every side of the interface, a top level of the standard library, and an owner_config() that imports nothing
    # heavy: everything the owner imports runs for the whole drive
    self.assertEqual(check_adapter(jetlink_adapter, heavy=HEAVY), [])

  def test_the_api_and_the_setting_are_jetlinks(self):
    self.assertEqual(jetlink_adapter.API, jl.API)
    self.assertEqual(jetlink_adapter.MODES, jl.MODES)

  def test_it_is_an_adapter_module(self):
    self.assertIsInstance(load_adapter(jetlink_adapter.__name__), Adapter)
    config = jetlink_adapter.owner_config()
    self.assertIsInstance(config, OwnerConfig)
    self.assertEqual(config.adapter, jetlink_adapter.__name__)
    self.assertEqual(config.keys, Adapter().keys)


class TestParams(unittest.TestCase):
  def test_the_directory_follows_params_root_and_the_prefix(self):
    with tempfile.TemporaryDirectory() as root:
      for prefix in ('d', 'bench'):
        with mock.patch.dict('os.environ', {'PARAMS_ROOT': root, 'OPENPILOT_PREFIX': prefix}):
          self.assertEqual(jetlink_adapter._params_dir(), Path(root) / prefix)
          self.assertEqual(Adapter().params_dir(), Path(root) / prefix)

  def test_the_link_setting_is_off_unless_set(self):
    with tempfile.TemporaryDirectory() as root, mock.patch.dict('os.environ', {'PARAMS_ROOT': root, 'OPENPILOT_PREFIX': 'd'}):
      link = Path(root) / 'd' / KEYS.link
      self.assertFalse(jetlink_adapter.link_on())   # no file: the default
      link.parent.mkdir()
      for text, on in (('0', False), ('', False), ('junk', False), ('1', True), ('2', True), ('3', False)):
        link.write_text(text)
        self.assertEqual(jetlink_adapter.link_on(), on, text)

  def test_every_key_is_declared_in_params_keys_h_with_its_type_and_default_off(self):
    header = (OPENPILOT / 'common' / 'params_keys.h').read_text()
    declared = {m.group(1): (m.group(2), m.group(3)) for m in
                re.finditer(r'\{"(\w+)",\s*\{([^,}]+),\s*(\w+)', header)}
    want = {KEYS.link: ('PERSISTENT | BACKUP', 'INT'), KEYS.progress: ('CLEAR_ON_MANAGER_START', 'JSON'),
            KEYS.spec: ('PERSISTENT', 'JSON'), KEYS.charge_phone: ('PERSISTENT', 'BOOL'),
            'Offroad_JetlinkUnavailable': ('CLEAR_ON_MANAGER_START', 'JSON')}
    for key, (flags, kind) in want.items():
      self.assertIn(key, declared, key)
      self.assertEqual((declared[key][0].strip(), declared[key][1]), (flags, kind), key)
    # the setting defaults to 0 (off) and the phone does not charge by default
    self.assertIn('{"JetlinkLink", {PERSISTENT | BACKUP, INT, "0"}}', header)
    self.assertIn('{"JetlinkChargePhone", {PERSISTENT, BOOL, "0"}}', header)

  def test_the_offroad_alert_exists_and_takes_the_reason(self):
    alerts = json.loads((OPENPILOT / 'selfdrive' / 'selfdrived' / 'alerts_offroad.json').read_text())
    self.assertEqual(alerts['Offroad_JetlinkUnavailable']['text'], 'Jetlink unavailable: %1')


class TestTheDevice(unittest.TestCase):
  def test_in_control_is_enabled_or_mads_enabled(self):
    def sm(enabled, mads):
      return {'carControl': SimpleNamespace(enabled=enabled), 'carControlSP': SimpleNamespace(mads=SimpleNamespace(enabled=mads))}
    op = Adapter()
    self.assertEqual([op.engaged(sm(e, m)) for e, m in ((False, False), (True, False), (False, True), (True, True))],
                     [False, True, True, True])
    self.assertIn('carControlSP', Adapter.IN_CONTROL_SERVICES)

  def test_the_chestnut_ids_are_the_hardware_modules(self):
    from openpilot.common.hardware.usb import CHESTNUT_ROM_USB_IDS, CHESTNUT_USB_IDS
    self.assertEqual(Adapter.CHESTNUT_IDS, frozenset(CHESTNUT_USB_IDS + CHESTNUT_ROM_USB_IDS))

  def test_a_cable_is_a_chestnut_guess_only_while_jetlink_is_off(self):
    with mock.patch.object(jetlink_adapter, 'link_on', return_value=False):
      self.assertTrue(jetlink_adapter.chestnut_cable(True))
      self.assertFalse(jetlink_adapter.chestnut_cable(False))
    with mock.patch.object(jetlink_adapter, 'link_on', return_value=True):
      self.assertFalse(jetlink_adapter.chestnut_cable(True))   # a Jetson or an iPhone on the port, not a chestnut

  def test_blendful_rides_on_the_stock_action(self):
    stock_calls = []

    def stock(model_output, prev_action, lat_t, long_t, v_ego):
      stock_calls.append(v_ego)
      return SimpleNamespace(desiredCurvature=0.01)

    action = jetlink_adapter._with_blendful(stock, 0.3)
    self.assertAlmostEqual(action({}, None, 0., 0., 10.).desiredCurvature, 0.01)                       # no callback: stock
    self.assertAlmostEqual(action({}, None, 0., 0., 10., lambda c: 0.002).desiredCurvature, 0.012)    # corrected
    self.assertAlmostEqual(action({}, None, 0., 0., 0.1, lambda c: 0.002).desiredCurvature, 0.01)      # at a stop: not
    self.assertEqual(len(stock_calls), 3)


class TestTheActionRule(unittest.TestCase):
  """A swap to the Jetlink model must not be a step in what is commanded: in modeld_v2 the joining model's action is
  held to modeld_v2's own ModelState.get_action_from_model, read from its source (importing it costs tinygrad)."""

  @staticmethod
  def v2_method():
    from collections.abc import Callable
    from openpilot.cereal import log
    from openpilot.selfdrive.controls.lib.drive_helpers import get_accel_from_plan, smooth_value
    from openpilot.sunnypilot.modeld_v2.constants import Plan
    from openpilot.sunnypilot.modeld_v2.fill_model_msg import get_curvature_from_output
    tree = ast.parse((OPENPILOT / 'sunnypilot' / 'modeld_v2' / 'modeld.py').read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'ModelState')
    fn = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'get_action_from_model')
    ns = {'np': np, 'log': log, 'Callable': Callable, 'Plan': Plan, 'get_accel_from_plan': get_accel_from_plan,
          'smooth_value': smooth_value, 'get_curvature_from_output': get_curvature_from_output}
    exec(compile(ast.Module([fn], []), 'modeld_v2.ModelState.get_action_from_model', 'exec'), ns)
    return ns['get_action_from_model']

  def outputs(self):
    rng = np.random.default_rng(7)
    yield {'action': np.array([[0.12, -0.6]])}
    yield {'action': np.array([[-0.3, 0.4]])}
    yield {'plan': rng.normal(size=(1, 33, 15)).astype(np.float32)}

  def test_it_is_modeld_v2s_action_at_every_speed(self):
    from openpilot.cereal import log
    from openpilot.sunnypilot.modeld_v2.constants import ModelConstants
    method = self.v2_method()
    with mock.patch.object(jetlink_adapter, '_smoothing', return_value=(0.0, 0.3, 0.3)):
      ours = jetlink_adapter._fork_action_function()
    # modeld_v2's ModelState as the loop holds the joining model to: the stock smoothing a joining model reports
    shim = SimpleNamespace(constants=ModelConstants, PLANPLUS_CONTROL=1.0, mlsim=False, generation=12,
                           LAT_SMOOTH_SECONDS=0.0, LONG_SMOOTH_SECONDS=0.3, MIN_LAT_CONTROL_SPEED=0.3)
    prev = log.ModelDataV2.Action(desiredCurvature=0.02, desiredAcceleration=0.1)
    compared = 0
    for out in self.outputs():
      for v_ego in (0.0, 0.2, 0.3, 0.5, 1.0, 1.5, 2.5, 4.0, 10.0, 30.0):
        for blendful in (None, lambda c: 0.002 * np.sign(c)):
          want = method(shim, out, prev, 0.3, 0.4, v_ego, blendful)
          got = ours(out, prev, 0.3, 0.4, v_ego, blendful)
          self.assertAlmostEqual(got.desiredCurvature, want.desiredCurvature, 9, (v_ego, out.keys()))
          self.assertAlmostEqual(got.desiredAcceleration, want.desiredAcceleration, 9, (v_ego, out.keys()))
          self.assertEqual(got.shouldStop, want.shouldStop, (v_ego, out.keys()))
          compared += 1
    self.assertEqual(compared, 60)

  def test_stock_modelds_big_model_is_left_stock(self):
    # in stock modeld there is no modeld_v2 rule to match: manager's launcher runs it, so __main__ is manager.
    # The second clause keys on sys.modules, so isolate the test from any modeld_v2 import another test in this
    # process has already made.
    saved = sys.modules.pop('openpilot.sunnypilot.modeld_v2.modeld', None)
    if saved is not None:
      self.addCleanup(sys.modules.__setitem__, 'openpilot.sunnypilot.modeld_v2.modeld', saved)
    with mock.patch.dict(sys.modules, {'__main__': SimpleNamespace(__file__='/data/openpilot/openpilot/system/manager/manager.py')}):
      self.assertFalse(jetlink_adapter._in_modeld_v2())
    with mock.patch.dict(sys.modules, {'__main__': SimpleNamespace(__file__='/data/openpilot/openpilot/sunnypilot/modeld_v2/modeld.py')}):
      self.assertTrue(jetlink_adapter._in_modeld_v2())


if __name__ == '__main__':
  unittest.main()
