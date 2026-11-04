"""
Tests for the fork's no-accelerate-toward-a-lead cap (sunnypilot/fork/no_close.py, param NoCloseOnLead).

Owner's note after the 2026-11-01 drive: "it still accelerates towards a car that comes into its view to try and get TO
the follow distance, it never needs to do that."
"""
import math

import numpy as np

from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.fork import no_close as nc


class FakeParams:
  def __init__(self, enabled=True, unknown=False):
    self.enabled = enabled
    self.unknown = unknown

  def get_bool(self, key):
    assert key == nc.PARAM
    if self.unknown:
      raise KeyError(key)
    return self.enabled


class TestCapNoClose(OpenpilotTestCase):
  def test_closing_lead_caps_positive_ask_to_zero(self):
    for a in (0.1, 0.5, 1.2):
      with self.subTest(accel=a):
        self.assertEqual(nc.cap_no_close(a, 25., True, 22., closing=True), 0.)

  def test_never_deepens_a_brake(self):
    for a in (-0.2, -1.5, -3.0):
      with self.subTest(accel=a):
        self.assertEqual(nc.cap_no_close(a, 25., True, 22., closing=True), a)

  def test_matched_or_opening_gap_untouched(self):
    self.assertEqual(nc.cap_no_close(0.8, 25., True, 25., closing=False), 0.8)
    self.assertEqual(nc.cap_no_close(0.8, 25., True, 28., closing=False), 0.8)

  def test_no_lead_is_inert(self):
    self.assertEqual(nc.cap_no_close(0.8, 25., False, 22., closing=True), 0.8)

  def test_disabled_is_inert(self):
    self.assertEqual(nc.cap_no_close(0.8, 25., True, 22., closing=True, enabled=False), 0.8)

  def test_unusable_lead_speed_is_inert(self):
    for v in (None, float("nan"), -5.):
      with self.subTest(v_lead=v):
        self.assertEqual(nc.cap_no_close(0.8, 25., True, v, closing=True), 0.8)


class TestNoCloseCap(OpenpilotTestCase):
  def test_disabled_is_inert(self):
    c = nc.NoCloseCap(params=FakeParams(enabled=False))
    for _ in range(20):
      self.assertEqual(c.update(0.9, 25., True, 20.), 0.9)

  def test_stale_libparams_is_inert(self):
    c = nc.NoCloseCap(params=FakeParams(unknown=True))
    self.assertEqual(c.update(0.9, 25., True, 20.), 0.9)

  def test_full_cap_at_or_past_enter(self):
    for v_rel in (nc.ENTER, nc.ENTER + 0.01, 1.5, 5.):
      with self.subTest(v_rel=v_rel):
        c = nc.NoCloseCap(params=FakeParams())
        self.assertEqual(c.update(0.9, 25., True, 25. - v_rel), 0.)

  def test_no_cap_at_or_below_exit(self):
    for v_rel in (nc.EXIT, nc.EXIT - 0.01, 0., -2.):
      with self.subTest(v_rel=v_rel):
        c = nc.NoCloseCap(params=FakeParams())
        for _ in range(5):
          self.assertEqual(c.update(0.9, 25., True, 25. - v_rel), 0.9)
        self.assertFalse(c.closing)

  def test_linear_inside_the_band(self):
    mid = (nc.ENTER + nc.EXIT) / 2.
    c = nc.NoCloseCap(params=FakeParams())
    self.assertAlmostEqual(c.update(0.8, 25., True, 25. - mid), 0.4, places=6)
    self.assertTrue(c.closing)

  def test_release_needs_the_dwell(self):
    """Full cap, then the gap matches: ONE frame below EXIT stays capped; RELEASE_DWELL_S of it releases fully."""
    c = nc.NoCloseCap(params=FakeParams())
    self.assertEqual(c.update(0.9, 25., True, 21.), 0.)
    n = int(round(nc.RELEASE_DWELL_S / c.dt))
    outs = [c.update(0.9, 25., True, 25. - nc.EXIT + 0.01) for _ in range(n)]
    self.assertEqual(outs[0], 0.)    # ONE matched frame does not release
    self.assertEqual(outs[-2], 0.)   # the capped frame is still in the window...
    self.assertEqual(outs[-1], 0.9)  # ...and ages out after exactly RELEASE_DWELL_S below EXIT (as the old dwell)
    self.assertFalse(c.closing)

  def test_tighten_is_immediate(self):
    """Pulling throttle while closing must never be delayed: the first frame at/past ENTER is fully capped, even
    straight after a long uncapped stretch, and every rise of the closing speed lowers the output on that frame."""
    c = nc.NoCloseCap(params=FakeParams())
    for _ in range(50):
      self.assertEqual(c.update(0.9, 25., True, 25.), 0.9)
    self.assertEqual(c.update(0.9, 25., True, 25. - nc.ENTER), 0.)
    c = nc.NoCloseCap(params=FakeParams())
    prev = c.update(0.9, 25., True, 25. - nc.EXIT)
    for v_rel in np.linspace(nc.EXIT + 0.02, nc.ENTER, 10):
      out = c.update(0.9, 25., True, 25. - v_rel)
      self.assertAlmostEqual(out, 0.9 * nc.no_close_factor(v_rel), places=9)  # no lag: exactly the raw factor
      self.assertLess(out, prev)
      prev = out

  def test_negative_or_zero_ask_untouched(self):
    """Coast/brake passes through bit-for-bit in every state: closing, inside the band, held by the dwell."""
    c = nc.NoCloseCap(params=FakeParams())
    for v_rel in (3., 0.5, 0.4, 0.1, 0.1, 0.1, -1.):
      for a in (0., -0.01, -0.7, -3.5):
        self.assertEqual(c.update(a, 25., True, 25. - v_rel), a)

  def test_continuity_for_smooth_closing_speed(self):
    """The route-191 cliff: a smoothly rising closing speed may not drop the ask by more than accel*dv/(ENTER-EXIT)
    per frame (no binary step anywhere), and a smoothly falling one may only bring throttle back after the dwell."""
    c = nc.NoCloseCap(dt=0.05, params=FakeParams())
    a = 0.4
    dv = 0.02   # m/s per 50 ms frame = 0.4 m/s^2 of relative decel - a brisk but ordinary closure
    bound = a * dv / (nc.ENTER - nc.EXIT) + 1e-9
    v_rels = list(np.arange(-0.5, 1.2, dv)) + list(np.arange(1.2, -0.5, -dv))
    out = [c.update(a, 20., True, 20. - v) for v in v_rels]
    falls = [p - q for p, q in zip(out, out[1:])]
    self.assertLessEqual(max(falls), bound)
    self.assertEqual(min(out), 0.)          # still reaches the full cap
    self.assertEqual(out[-1], a)            # and fully releases
    rises = [q - p for p, q in zip(out, out[1:])]
    self.assertLessEqual(max(rises), bound)  # release is continuous too (the dwell shifts it, it doesn't step it)

  def test_unusable_lead_speed_frame_does_not_release(self):
    c = nc.NoCloseCap(params=FakeParams())
    self.assertEqual(c.update(0.9, 25., True, 21.), 0.)
    self.assertEqual(c.update(0.9, 25., True, -5.), 0.9)   # unusable frame: inert for that frame (as before)...
    self.assertEqual(c.update(0.9, 25., True, 25.), 0.)    # ...but it did not flush the dwell window

  def test_lead_disappearing_releases(self):
    c = nc.NoCloseCap(params=FakeParams())
    self.assertEqual(c.update(0.9, 25., True, 20.), 0.)
    self.assertFalse(c.closing is False and False)  # state must reset below
    self.assertEqual(c.update(0.9, 25., False, 20.), 0.9)
    self.assertFalse(c.closing)

  def test_zero_speed_leads_do_not_read_as_closing_at_standstill(self):
    # ego stopped behind a stopped lead: v_rel = 0, no cap
    c = nc.NoCloseCap(params=FakeParams())
    self.assertEqual(c.update(1.0, 0., True, 0.), 1.0)

  def test_param_is_reread(self):
    p = FakeParams(enabled=False)
    c = nc.NoCloseCap(params=p)
    self.assertEqual(c.update(0.9, 25., True, 20.), 0.9)
    p.enabled = True
    out = [c.update(0.9, 25., True, 20.) for _ in range(int(1. / c.dt) + 1)]
    self.assertEqual(out[-1], 0.)


class TestWiring(OpenpilotTestCase):
  """The cap must run on the FINAL ask, after the candidate min(), or a candidate could raise it back."""

  def test_applied_after_candidate_min(self):
    import inspect
    from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
    src = inspect.getsource(LongitudinalPlanner.update)
    i_min = src.index("min(candidates")
    i_cap = src.index("self.no_close.update")
    self.assertGreater(i_cap, i_min)


def test_lead_hovering_at_the_threshold_cannot_square_wave():
  """Route 191: 6 blips per engaged minute, the ask flipping 0.400 <-> 0.000 - a lead sitting at the release threshold."""
  c = nc.NoCloseCap(dt=0.05, params=FakeParams(enabled=True))
  c.enabled = True
  out = []
  for i in range(200):
    v_rel = 0.62 if i % 4 == 0 else 0.22          # radar noise straddling the old 0.3/0.1 band
    out.append(c.update(0.4, 20., True, 20. - v_rel))
  flips = sum(1 for a, b in zip(out, out[1:]) if abs(a - b) > 0.1)
  assert flips <= 2, f"pedal command flipped {flips} times in 10 s"
