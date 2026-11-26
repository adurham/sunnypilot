"""
Tests for the optional earlier-planner-braking offset (sunnypilot/fork/planner_earlier.py).

The change is intentionally tiny and contained: when the default-OFF ``PlannerEarlierBraking`` param is ON, an extra
0.5 s of follow time is added to the MPC/gate distance. The result is a lead/stop-approach plan that asks to slow
earlier, so the car spends more of the first part of the slowdown on engine braking before the FCA11 friction-brake
gate (-0.5 m/s^2) opens. The FCA11 map, panda, engine-braking physics, ACCEL_MIN/MAX, AccelBoost, NoClose cap,
standstill-launch hold, stop-commit, min-gap floor and SCC/SLA arbitration are untouched.
"""
import numpy as np

from openpilot.cereal import log
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import get_T_FOLLOW
from openpilot.sunnypilot.fork import planner_earlier as pe_mod
from openpilot.sunnypilot.fork.adaptive_follow import AdaptiveFollow
from openpilot.sunnypilot.fork.tests.test_lead_closing import DT, make_planner, planner_sm


class FakeParamsOff:
  def get_bool(self, key):
    return False


class FakeParamsOn:
  def get_bool(self, key):
    return key == pe_mod.PARAM


class TestPlannerEarlierBraking(OpenpilotTestCase):
  def test_default_is_off(self):
    """Without the param set, PlannerEarlierBraking returns the base t_follow unchanged."""
    p = pe_mod.PlannerEarlierBraking(params=FakeParamsOff())
    for tf in (1.25, 1.45, 1.75, 2.05):
      self.assertEqual(p.adjust(tf), tf)
    self.assertIsNone(p.adjust(None))

  def test_when_on_adds_extra_time_gap(self):
    """When the param is ON, the offset is added to any finite base t_follow; None stays None."""
    p = pe_mod.PlannerEarlierBraking(params=FakeParamsOn())
    self.assertTrue(p.enabled)
    for tf in (1.25, 1.45, 1.75, 2.05):
      self.assertAlmostEqual(p.adjust(tf), tf + pe_mod.EXTRA_T_FOLLOW, places=6)
    self.assertIsNone(p.adjust(None))

  def _base_t_follow(self, p, sm, v_ego):
    base = p.get_t_follow(sm, v_ego)
    return get_T_FOLLOW(log.LongitudinalPersonality.standard) if base is None else base

  def test_get_effective_t_follow_is_identity_when_off(self):
    """The LongitudinalPlannerSP hook is identity when the param is OFF."""
    p = make_planner()
    p.planner_earlier = pe_mod.PlannerEarlierBraking(params=FakeParamsOff())
    sm = planner_sm(15., 0.0, d_rel=60., v_rel=-1.)
    base = self._base_t_follow(p, sm, 15.)
    effective = p.get_effective_t_follow(sm, 15.)
    self.assertAlmostEqual(effective, base, places=6)

  def test_get_effective_t_follow_adds_offset_when_on(self):
    """The hook adds the offset when the param is ON."""
    p = make_planner()
    p.planner_earlier = pe_mod.PlannerEarlierBraking(params=FakeParamsOn())
    sm = planner_sm(15., 0.0, d_rel=60., v_rel=-1.)
    base = self._base_t_follow(p, sm, 15.)
    effective = p.get_effective_t_follow(sm, 15.)
    self.assertAlmostEqual(effective, base + pe_mod.EXTRA_T_FOLLOW, places=6)

  def test_stock_personality_t_follow_is_used_when_adaptive_off(self):
    """When AdaptiveFollowDistance is OFF, the hook still applies the offset to the stock personality T_FOLLOW."""
    p = make_planner()
    p.adaptive_follow = AdaptiveFollow(True, params=FakeParamsOff())  # disabled adaptive follow
    p.planner_earlier = pe_mod.PlannerEarlierBraking(params=FakeParamsOn())
    sm = planner_sm(15., 0.0, d_rel=60., v_rel=-1.)
    effective = p.get_effective_t_follow(sm, 15.)
    self.assertAlmostEqual(effective, get_T_FOLLOW(log.LongitudinalPersonality.standard) + pe_mod.EXTRA_T_FOLLOW, places=6)

  def test_braking_lead_ask_goes_negative_earlier(self):
    """A lead braking from a matched highway gap: with the offset the published ask goes negative sooner."""
    v_ego = 20.
    d0 = 30.   # ~1.5 s headway at matched speed
    a_lead = -1.5
    n = 120

    def drive(planner):
      asks = []
      for i in range(n):
        # lead brakes at 20 m/s; ego initially matched, then the gap closes
        sm = planner_sm(v_ego, desired_accel=0.0,
                        d_rel=d0 + 0.5 * a_lead * (i * DT) ** 2,
                        v_rel=a_lead * i * DT,
                        v_cruise_kph=110.)
        planner.update(sm)
        asks.append(float(planner.output_a_target))
      return np.array(asks)

    off = drive(make_planner())
    p_on = make_planner()
    p_on.planner_earlier = pe_mod.PlannerEarlierBraking(params=FakeParamsOn())
    on = drive(p_on)

    first_neg_off = next(i for i, a in enumerate(off) if a < -0.01) * DT
    first_neg_on = next(i for i, a in enumerate(on) if a < -0.01) * DT
    self.assertLess(first_neg_on, first_neg_off,
                    f"first negative ask must move earlier (off={first_neg_off:.3f}s, on={first_neg_on:.3f}s)")

  def test_braking_lead_crosses_friction_gate_earlier(self):
    """The -0.5 m/s^2 friction-brake threshold should be crossed earlier when the offset is ON."""
    v_ego = 20.
    d0 = 30.
    a_lead = -1.5
    n = 120

    def drive(planner):
      asks = []
      for i in range(n):
        sm = planner_sm(v_ego, desired_accel=0.0,
                        d_rel=d0 + 0.5 * a_lead * (i * DT) ** 2,
                        v_rel=a_lead * i * DT,
                        v_cruise_kph=110.)
        planner.update(sm)
        asks.append(float(planner.output_a_target))
      return np.array(asks)

    off = drive(make_planner())
    p_on = make_planner()
    p_on.planner_earlier = pe_mod.PlannerEarlierBraking(params=FakeParamsOn())
    on = drive(p_on)

    gate = -0.5
    cross_off = next(i for i, a in enumerate(off) if a <= gate) * DT
    cross_on = next(i for i, a in enumerate(on) if a <= gate) * DT
    self.assertLess(cross_on, cross_off,
                    f"-0.5 gate crossing must move earlier (off={cross_off:.3f}s, on={cross_on:.3f}s)")
    self.assertLess(cross_on - cross_off, 0.)

  def test_no_lead_plan_is_unchanged(self):
    """Without a lead the MPC time gap has no effect: the published plan is identical ON vs OFF."""
    p_off = make_planner()
    p_on = make_planner()
    p_on.planner_earlier = pe_mod.PlannerEarlierBraking(params=FakeParamsOn())
    asks_off, asks_on = [], []
    for i in range(60):
      sm = planner_sm(17., desired_accel=0.3, d_rel=0., v_rel=0., lead_present=False)
      p_off.update(sm)
      p_on.update(sm)
      asks_off.append(float(p_off.output_a_target))
      asks_on.append(float(p_on.output_a_target))
    np.testing.assert_array_equal(asks_on, asks_off)

  def test_pulling_away_lead_plan_is_unchanged(self):
    """A lead pulling away is not a slowdown: the offset must not change the plan."""
    p_off = make_planner()
    p_on = make_planner()
    p_on.planner_earlier = pe_mod.PlannerEarlierBraking(params=FakeParamsOn())
    asks_off, asks_on = [], []
    for i in range(80):
      sm = planner_sm(17., desired_accel=0.5, d_rel=60. + 1.0 * i * DT, v_rel=+1.0)
      p_off.update(sm)
      p_on.update(sm)
      asks_off.append(float(p_off.output_a_target))
      asks_on.append(float(p_on.output_a_target))
    np.testing.assert_array_equal(asks_on, asks_off)

  def test_only_lead_follow_braking_episodes_change(self):
    """Sanity: on a fixed-gap matched-lead cruise, the offset has no effect until the lead starts braking."""
    p_off = make_planner()
    p_on = make_planner()
    p_on.planner_earlier = pe_mod.PlannerEarlierBraking(params=FakeParamsOn())
    diffs = []
    for i in range(100):
      # matched lead, then it starts braking at frame 50
      if i < 50:
        sm = planner_sm(17., desired_accel=0.3, d_rel=35., v_rel=0.)
      else:
        t = (i - 50) * DT
        sm = planner_sm(17., desired_accel=0.0, d_rel=35. - 0.5 * t * t, v_rel=-1.0 * t)
      p_off.update(sm)
      p_on.update(sm)
      diffs.append(float(p_on.output_a_target - p_off.output_a_target))
    diffs = np.array(diffs)
    # no difference while matched
    self.assertTrue(np.all(diffs[:50] == 0.0))
    # difference appears only once the lead is braking and the plan asks to slow
    self.assertTrue(np.any(diffs[50:] < 0.0))
