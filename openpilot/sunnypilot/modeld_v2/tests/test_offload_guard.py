"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import os

import numpy as np

import openpilot.sunnypilot.modeld_v2.modeld as modeld_module
from openpilot.sunnypilot.modeld_v2.tests import helpers as tests_helpers
from openpilot.sunnypilot.modeld_v2.tests.helpers import (
  DummyModel, DummyBundle, CAM_W, CAM_H, FakeRunModel, write_supercombo_run_pkl,
)
from openpilot.common.test import OpenpilotTestCase

tmp_path = tests_helpers.tmp_path
patch_modeld = tests_helpers.patch_modeld

# Minimal supercombo layout: only hidden_state, so parse_outputs is a no-op and
# the guard logic (finite check + prev_feat feedback) is what is exercised.
GUARD_SLICES = {'hidden_state': slice(0, 512)}
OUT_LEN = 512
HIDDEN = GUARD_SLICES['hidden_state']


def _good_flat():
  """Deterministic all-finite 1062-vector (any finite values parse fine)."""
  return (np.arange(OUT_LEN, dtype=np.float32) * 1e-3)


def _bad_flat():
  """Same vector with a single NaN inside the plan slice -> NOT_FINITE."""
  f = _good_flat()
  f[10] = np.nan
  return f


class TestOffloadNonFiniteGuard(OpenpilotTestCase):
  """OFFLOAD §4g: an all-finite failure must not tear down the offload host.

  Reference: jetlink keeps the last good hidden state and reports NOT_FINITE
  instead of raising (JetlinkKit/Sources/JetlinkServer/Queues.swift:122-125).
  """

  def _make_state(self, tmp_path, monkeypatch, patch_modeld, *, offload=True,
                  output_slices=None):
    from openpilot.common.hardware import hw
    write_supercombo_run_pkl(tmp_path, output_slices=GUARD_SLICES)
    bundle = DummyBundle(
      models=[DummyModel('supercombo', 'driving_test_tinygrad.pkl')], is_20hz=False)
    patch_modeld(bundle)
    monkeypatch.setattr(hw.Paths, 'model_root', staticmethod(lambda: str(tmp_path)))
    if offload:
      # force the OFFLOAD branch but keep the queues on CPU (no Metal dependency)
      os.environ['OFFLOAD'] = '1'
      os.environ['OFFLOAD_DEV'] = 'CPU'
      os.environ['OFFLOAD_WARP_DEV'] = 'CPU'
    else:
      os.environ.pop('OFFLOAD', None)
    return modeld_module.ModelState(cam_w=CAM_W, cam_h=CAM_H)

  def _bufs_and_inputs(self, state):
    frame = (np.arange(state.frame_copy_size, dtype=np.uint8) % 251)
    bufs = {name: frame.tobytes() for name in state.vision_input_names}
    transforms = {name: np.eye(3, dtype=np.float32) for name in state.vision_input_names}
    vec_desire = np.zeros(state.constants.DESIRE_LEN, dtype=np.float32)
    vec_desire[3] = 1.0
    inputs = {state.desire_key: vec_desire,
              'traffic_convention': np.array([0.0, 1.0], dtype=np.float32)}
    if 'action_t' in state.numpy_inputs:
      inputs['action_t'] = np.array([0.15, 0.25], dtype=np.float32)
    return bufs, transforms, inputs

  def test_nonfinite_preserves_last_good_state_and_continues(self, tmp_path, monkeypatch, patch_modeld):
    state = self._make_state(tmp_path, monkeypatch, patch_modeld)
    assert state.is_run_model and state._combined_model_type == 'supercombo'

    # one good frame first, so prev_feat holds a known good value
    state.run_model = FakeRunModel([_good_flat()])
    bufs, transforms, inputs = self._bufs_and_inputs(state)
    out = state.run(bufs, transforms, inputs)
    assert out is not None
    good_state = state.numpy_inputs['prev_feat'].copy()
    np.testing.assert_allclose(np.asarray(good_state).ravel(),
                               _good_flat()[HIDDEN], rtol=1e-6, atol=1e-6)

    # a non-finite frame: no exception, nothing published, last-good state kept
    before_count = modeld_module._NONFINITE_OFFLOAD_STATE['count']
    state.run_model = FakeRunModel([_bad_flat()])
    out = state.run(bufs, transforms, inputs)  # must NOT raise
    assert out is None
    assert modeld_module._NONFINITE_OFFLOAD_STATE['count'] == before_count + 1
    np.testing.assert_array_equal(state.numpy_inputs['prev_feat'], good_state)

    # a subsequent good frame runs normally and advances the hidden state
    state.run_model = FakeRunModel([_good_flat() * 2.0])
    out = state.run(bufs, transforms, inputs)
    assert out is not None
    np.testing.assert_allclose(np.asarray(state.numpy_inputs['prev_feat']).ravel(),
                               (_good_flat() * 2.0)[HIDDEN], rtol=1e-6, atol=1e-6)

  def test_nonfinite_is_rate_limited(self, tmp_path, monkeypatch, patch_modeld):
    state = self._make_state(tmp_path, monkeypatch, patch_modeld)
    bufs, transforms, inputs = self._bufs_and_inputs(state)
    state.run_model = FakeRunModel([_bad_flat()])
    start = modeld_module._NONFINITE_OFFLOAD_STATE['count']
    for _ in range(3):
      assert state.run(bufs, transforms, inputs) is None
    # counter increments every frame; the log is what is rate-limited
    assert modeld_module._NONFINITE_OFFLOAD_STATE['count'] == start + 3

  def test_device_behavior_raises_when_offload_unset(self, tmp_path, monkeypatch, patch_modeld):
    """Byte-identical device contract: OFFLOAD unset => the stock chestnut raise fires."""
    state = self._make_state(tmp_path, monkeypatch, patch_modeld, offload=False)
    # chestnut models are the only ones that carry the finite check; force it
    monkeypatch.setattr(state, 'chestnut', True)
    state.run_model = FakeRunModel([_bad_flat()])
    bufs, transforms, inputs = self._bufs_and_inputs(state)
    with self.assertRaisesRegex(RuntimeError, "not finite"):
      state.run(bufs, transforms, inputs)
