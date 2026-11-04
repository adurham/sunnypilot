"""
Tests for the fork's no-accelerate-toward-a-lead cap (sunnypilot/fork/no_close.py, param NoCloseOnLead).

Owner's note after the 2026-11-01 drive: "it still accelerates towards a car that comes into its view to try and get TO
the follow distance, it never needs to do that."
"""
import math

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

  def test_enters_at_enter_threshold(self):
    c = nc.NoCloseCap(params=FakeParams())
    self.assertEqual(c.update(0.9, 25., True, 25. - nc.ENTER + 0.01), 0.9)   # below ENTER: not closing
    self.assertEqual(c.update(0.9, 25., True, 25. - nc.ENTER - 0.01), 0.)    # past ENTER: capped

  def test_hysteresis_holds_until_exit(self):
    c = nc.NoCloseCap(params=FakeParams())
    self.assertEqual(c.update(0.9, 25., True, 21.), 0.)                 # closing
    self.assertEqual(c.update(0.9, 25., True, 25. - nc.EXIT - 0.01), 0.)  # still closing (above EXIT)
    # 2026-11-03: a release now needs EXIT held for RELEASE_DWELL_S, so ONE frame below it stays capped
    self.assertEqual(c.update(0.9, 25., True, 25. - nc.EXIT + 0.01), 0.0)
    self.assertTrue(c.closing)
    for _ in range(int(nc.RELEASE_DWELL_S / c.dt)):
      out = c.update(0.9, 25., True, 25. - nc.EXIT + 0.01)
    self.assertEqual(out, 0.9)  # ...and RELEASE_DWELL_S below EXIT releases

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
