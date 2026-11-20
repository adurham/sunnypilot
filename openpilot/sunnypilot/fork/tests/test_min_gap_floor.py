"""
Tests for the fork's low-speed minimum-gap brake floor, v2 (sunnypilot/fork/min_gap_floor.py, param MinGapBrakeFloor).

The fixture frames are the REAL ones from route 000001ac, segment 59 (2026-11-19, I-80/94 stop-and-go behind a flat-rear
RV trailer): the gap collapse at 59:40.32-59:40.96 and the range triplet 4.6 -> 8.9 -> 3.1 m inside 0.3 s that made v1
chatter in closed loop.

Every rework item of the 2026-11-20 review has at least one test here:
  R1 release dwell / no spike release / no short re-arm      TestReleaseDwell, TestPostReleaseHold
  R2 post-release hold caps the ask at zero throttle          TestPostReleaseHold
  R3 v < 1 m/s enters the hold, no 1.0/1.5 limit cycle        TestReleaseDwell, TestPostReleaseHold
  R4 dropout bridge: drop the brake, hold the throttle        TestDropoutBridge
  R5 launch grace starts below 1 m/s; does not mask a hit     TestArming
  R7 the hold ask is zero throttle and NOT a brake            TestAsksAndGeometry
"""
import os

import numpy as np

from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.fork import min_gap_floor as mg


class FakeParams:
  def __init__(self, enabled=True, unknown=False):
    self.enabled = enabled
    self.unknown = unknown

  def get_bool(self, key):
    assert key == mg.PARAM
    if self.unknown:
      raise KeyError(key)
    return self.enabled


# --- real frames, route 000001ac seg 59 (elapsed t, radarState.leadOne.dRel)
GAP_COLLAPSE_59_40 = [(3580.319, 2.965), (3580.362, 2.681), (3580.414, 2.539), (3580.474, 2.806), (3580.521, 2.642),
                      (3580.570, 2.130), (3580.613, 1.908), (3580.670, 1.836), (3580.719, 1.854), (3580.763, 1.722)]
# the range triplet the v1 review quotes: 4.6 -> 8.9 -> 3.1 m inside 0.3 s (real range noise at these ranges)
RANGE_TRIPLET = [4.638, 7.469, 6.386, 8.895, 8.505, 9.190, 8.395, 6.625, 5.098, 4.867, 4.371, 3.855, 3.100]


def make(enabled=True, dt=0.05):
  return mg.MinGapFloor(params=FakeParams(enabled=enabled), dt=dt)


class Ctx:
  """One-frame stepper with the previous call's inputs as defaults."""

  def __init__(self, f, **_kw):
    self.f = f
    self.kw = dict(ask=0.0, v=5.0, active=True, gas=False, present=True, d=2.0, prob=1.0, stand=False)
    self.kw.update(_kw)
    self.out = 0.0

  def push(self, n=1, **kw):
    self.kw.update(kw)
    for _ in range(n):
      self.out = self.f.update(self.kw["ask"], self.kw["v"], self.kw["active"], self.kw["gas"],
                               self.kw["present"], self.kw["d"], self.kw["prob"], self.kw["stand"])
    return self.out

  def arm(self, n=30, d=2.0, v=5.0, ask=0.0, **kw):
    """Feed n frames of a close lead and return the last ask (the floor arms inside this)."""
    return self.push(n, d=d, v=v, ask=ask, **kw)


class TestAsksAndGeometry(OpenpilotTestCase):
  def test_entry_exit_recover_bands_are_ordered(self):
    # entry 0.5 s / 3 m, exit 0.8 s / 5 m, recovery 1.0 s / 6 m: the release and recovery bands are always wider
    self.assertAlmostEqual(mg.enter_gap(5.0), 3.0)
    self.assertAlmostEqual(mg.enter_gap(8.0), 4.0)
    self.assertAlmostEqual(mg.exit_gap(5.0), 5.0)
    self.assertAlmostEqual(mg.exit_gap(8.0), 6.4)
    self.assertAlmostEqual(mg.recover_gap(3.0), 6.0)
    self.assertAlmostEqual(mg.recover_gap(8.0), 8.0)
    for v in np.arange(0.0, 15.0, 0.25):
      self.assertGreater(mg.exit_gap(v), mg.enter_gap(v))
      self.assertGreaterEqual(mg.recover_gap(v), mg.exit_gap(v))

  def test_hold_ask_is_zero_throttle_and_not_a_brake(self):
    """R7: the hold ask is pinned between two physical thresholds of the DEPLOYED car layer.

    - Above the FCA11 onset (-0.5): the hold must not command a brake.
    - Below -coast(v) for every v in the band: the gas law's continuous negative branch computes
      pedal = hold(v) * max(0, 1 - |ask|/coast(v)), so |ask| >= coast forces exactly 0 pedal (coast(10 m/s) = 0.29).
      That is what makes the hold a real zero-throttle state on the deployed build, independent of any pedal-fit branch.
    """
    onset = -0.5                       # opendbc/sunnypilot/car/hyundai/fca11_long.py: FCA11_BRAKE_ONSET_ACCEL
    self.assertGreater(mg.A_HOLD, onset)
    coast = lambda v: 0.365 * (1.0 - np.exp(-v / 6.4))   # fca11_cont_coast
    for v in np.arange(1.0, 10.01, 0.1):
      self.assertLess(mg.A_HOLD, -float(coast(v)), msg=f"v={v:.2f}: the hold would leave pedal live")
    self.assertLessEqual(mg.A_FLOOR, onset)              # the brake ask is at/below the onset

  def test_floor_ask_never_raises_in_any_state(self):
    for a in np.arange(-3.5, 2.01, 0.05):
      self.assertEqual(mg.floor_ask(float(a), mg.OFF), float(a))
      self.assertEqual(mg.floor_ask(float(a), mg.BRAKE), min(float(a), mg.A_FLOOR))
      for s in (mg.HOLD, mg.BRIDGE):
        self.assertEqual(mg.floor_ask(float(a), s), min(float(a), mg.A_HOLD))
        self.assertLessEqual(mg.floor_ask(float(a), s), float(a))


class TestOffAndFailsafe(OpenpilotTestCase):
  def test_off_is_byte_identical(self):
    """OFF-invariance at the rule level: with the param at 0 the output IS the input on every frame of a 500-frame walk."""
    f = make(enabled=False)
    rng = np.random.default_rng(7)
    ask = 0.0
    for _ in range(500):
      ask = float(np.clip(ask + rng.normal(0.0, 0.2), -3.5, 2.0))
      v = float(np.clip(5.0 + rng.normal(0.0, 1.0), 0.0, 12.0))
      d = float(np.clip(2.5 + rng.normal(0.0, 1.5), 0.3, 20.0))
      self.assertEqual(f.update(ask, v, True, False, True, d, 1.0, False), ask)
    self.assertEqual(f.state, mg.OFF)

  def test_unknown_key_stays_off(self):
    f = mg.MinGapFloor(params=FakeParams(enabled=True, unknown=True))
    c = Ctx(f)
    self.assertEqual(c.arm(30, d=1.5, ask=0.4), 0.4)
    self.assertEqual(f.state, mg.OFF)

  def test_driver_gas_is_off(self):
    f = make()
    c = Ctx(f)
    self.assertEqual(c.arm(30, d=1.5, gas=True), 0.0)
    self.assertEqual(f.state, mg.OFF)

  def test_planner_reset_is_off(self):
    f = make()
    c = Ctx(f)
    self.assertEqual(c.arm(30, d=1.5, active=False), 0.0)
    self.assertEqual(f.state, mg.OFF)
    # and an already-armed floor is released by a reset
    self.assertEqual(c.arm(30, d=1.5, active=True), -1.0)
    self.assertEqual(c.push(1, active=False), 0.0)
    self.assertEqual(f.state, mg.OFF)

  def test_standstill_is_off(self):
    f = make()
    c = Ctx(f)
    self.assertEqual(c.arm(30, d=1.2, v=1.8, stand=True), 0.0)
    self.assertEqual(f.state, mg.OFF)

  def test_above_v_max_never_frees_the_ask(self):
    """Leaving the top of the band must release the BRAKE but must not hand the pedal back (release-into-accel)."""
    f = make()
    c = Ctx(f)
    self.assertEqual(c.arm(30, d=2.0, v=9.5, ask=0.0), -1.0)
    self.assertEqual(c.push(1, v=10.5, ask=0.8), mg.A_HOLD)
    self.assertEqual(f.state, mg.HOLD)
    self.assertEqual(c.push(20, v=10.5, ask=0.8), mg.A_HOLD)   # still capped: no positive ask

  def test_hold_ends_at_a_standstill(self):
    """R3 boundary: the hold is a rolling-car state; a stopped car is the launch logic's, with a fresh grace."""
    f = make()
    c = Ctx(f)
    c.arm(30, d=2.0, v=5.0)
    c.push(1, v=0.9)                                           # -> HOLD
    self.assertEqual(f.state, mg.HOLD)
    self.assertEqual(c.push(1, v=0.4, d=2.0, ask=0.5), 0.5)    # stopped: the hold ends
    self.assertEqual(f.state, mg.OFF)
    self.assertAlmostEqual(f.launch_hold, mg.LAUNCH_GRACE_S)


class TestArming(OpenpilotTestCase):
  def test_no_entry_below_v_min(self):
    for v in (0.0, 0.4, 1.0, 1.49):
      f = make()
      c = Ctx(f)
      self.assertEqual(c.arm(40, d=1.0, v=v), 0.0, msg=f"v={v}")
      self.assertEqual(f.state, mg.OFF)

  def test_launch_from_a_short_queue_stop_is_not_braked(self):
    """Standstill pre-warms the median, so a normal pull-away from a 2.5 m queue stop must be protected by the grace."""
    f = make()
    c = Ctx(f)
    self.assertEqual(c.push(20, d=2.5, v=0.0, stand=True), 0.0)
    self.assertAlmostEqual(f.launch_hold, mg.LAUNCH_GRACE_S)
    for i in range(39):
      v = min(3.0, 0.15 * (i + 1))
      self.assertEqual(c.push(1, d=2.5, v=v), 0.0, msg=f"frame {i} v={v:.2f}")

  def test_crawl_below_one_then_launch_is_not_braked(self):
    """R5: the grace starts at v < 1 m/s, not only at a true standstill - a crawl at 0.8 m/s that never stops, then a
    launch at a 2.6 m gap, must not brake."""
    f = make()
    c = Ctx(f)
    self.assertEqual(c.push(40, d=2.6, v=0.8), 0.0)          # 2 s of crawl, no standstill flag
    self.assertAlmostEqual(f.launch_hold, mg.LAUNCH_GRACE_S)
    # launch: 0.8 -> 3.0 m/s over 1.6 s with the gap opening 2.6 -> 4.0 m (a normal pull-away)
    for i in range(32):
      v = 0.8 + 2.2 * (i + 1) / 32.0
      d = 2.6 + 1.4 * (i + 1) / 32.0
      self.assertEqual(c.push(1, d=d, v=v), 0.0, msg=f"frame {i} v={v:.2f} d={d:.2f}")

  def test_the_grace_does_not_mask_a_real_close_approach(self):
    """R5, the opposite hole: the two real episodes never had v < 1 m/s in the minute before the approach. A car that has
    been doing 3.3-5.3 m/s with no stop is armed on the median crossing."""
    f = make()
    c = Ctx(f)
    self.assertEqual(c.push(60, d=5.5, v=3.3), 0.0)           # 3 s of open road
    self.assertAlmostEqual(f.launch_hold, 0.0)
    out = c.push(10, d=1.77, v=3.3)                            # the 49:14 shape: 3.3 m/s, gap collapses to 1.77 m
    self.assertEqual(f.state, mg.BRAKE)
    self.assertEqual(out, -1.0)

  def test_one_noisy_first_frame_cannot_arm(self):
    f = make()
    c = Ctx(f)
    self.assertEqual(c.push(1, d=1.0, v=5.0), 0.0)             # a single close frame
    self.assertEqual(f.state, mg.OFF)
    self.assertEqual(c.push(mg.MIN_SAMPLES - 2, d=4.5, v=5.0), 0.0)
    self.assertEqual(c.push(5, d=1.2, v=5.0), -1.0)


class TestReleaseDwell(OpenpilotTestCase):
  def _armed(self):
    f = make()
    c = Ctx(f)
    self.assertEqual(c.arm(30, d=2.0, v=5.0), -1.0)
    self.assertEqual(f.state, mg.BRAKE)
    return f, c

  def _to_exit(self, f, c, target=5.6, n=60):
    """Ramp the gap up until the MEDIAN is above the exit band (5.0 m at 5 m/s) and the dwell has started."""
    for i in range(n):
      c.push(1, d=float(np.interp(i, [0, n - 1], [2.0, target])))
      if f.gap_med >= mg.exit_gap(5.0):
        return
    self.fail("the median never crossed the exit band")

  def test_release_requires_one_second_above_the_exit_band(self):
    """R1: the release needs the MEDIAN above the exit band CONTINUOUSLY for ~1 s - not merely to cross it."""
    f, c = self._armed()
    self._to_exit(f, c)
    self.assertEqual(f.state, mg.BRAKE)
    e0 = f.exit_t                                            # the crossing frame already counted
    k = int(round((mg.EXIT_DWELL_S - e0) / 0.05)) - 1        # the frames that do NOT yet reach the dwell
    self.assertGreaterEqual(k, 15)                           # it really does wait about a second
    for n in range(k):
      self.assertEqual(c.push(1, d=5.6), -1.0, msg=f"frame {n}")
      self.assertEqual(f.state, mg.BRAKE, msg=f"frame {n}")
    self.assertEqual(c.push(1, d=5.6), mg.A_HOLD)            # releases into the hold, not into the ask
    self.assertEqual(f.state, mg.HOLD)

  def test_the_real_range_triplet_cannot_release(self):
    """R1: 4.6 -> 8.9 -> 3.1 m inside 0.3 s (the logged excursion). The 0.5 s median cannot follow it and the dwell needs
    the median above the band for a whole second, so the brake stays on."""
    f, c = self._armed()
    for d in RANGE_TRIPLET:
      self.assertEqual(c.push(1, d=d), -1.0)
      self.assertEqual(f.state, mg.BRAKE)
    self.assertEqual(c.push(20, d=3.2), -1.0)
    self.assertEqual(f.state, mg.BRAKE)

  def test_a_dip_back_inside_restarts_the_dwell(self):
    f, c = self._armed()
    self._to_exit(f, c)
    c.push(10, d=5.6)
    self.assertGreater(f.exit_t, 0.2)
    c.push(10, d=3.5)                                          # the median is back inside the band
    self.assertEqual(f.exit_t, 0.0)
    self.assertEqual(f.state, mg.BRAKE)
    self.assertEqual(c.push(1, d=5.6), -1.0)                   # and the dwell starts from zero again

  def test_v_below_one_enters_the_hold_not_the_ask(self):
    """R3: v1 freed the ask at v < 1 m/s. v2 holds it at zero throttle."""
    f, c = self._armed()
    self.assertEqual(c.push(1, v=0.9, d=2.0, ask=0.6), mg.A_HOLD)
    self.assertEqual(f.state, mg.HOLD)

  def test_the_hold_never_returns_a_positive_ask(self):
    """R2: in the first second after any release the ask is <= 0 even if the planner asks for throttle."""
    f, c = self._armed()
    self._to_exit(f, c)
    n = 0
    while f.state == mg.BRAKE:
      c.push(1, d=5.6, ask=1.2)
      n += 1
      self.assertLess(n, 40)
    self.assertEqual(f.state, mg.HOLD)
    for _ in range(20):
      self.assertLessEqual(c.push(1, d=5.6, ask=1.2), 0.0)
      self.assertEqual(f.state, mg.HOLD)

  def test_hold_ends_only_after_the_recovery_dwell(self):
    """R2: recovery is the HIGHER band (6 m / 1.0 s headway), held for a second."""
    f, c = self._armed()
    self._to_exit(f, c)
    while f.state == mg.BRAKE:
      c.push(1, d=5.6, ask=1.2)
    self.assertEqual(f.state, mg.HOLD)
    for i in range(60):                                        # ramp just above the recovery band (6.0 m) and hold it
      c.push(1, d=float(np.interp(i, [0, 59], [5.6, 6.3])), ask=1.2)
      if f.gap_med >= mg.recover_gap(5.0):
        break
    self.assertEqual(f.state, mg.HOLD)
    n = 0
    outs = []
    while f.state == mg.HOLD:
      outs.append(c.push(1, d=6.3, ask=1.2))
      n += 1
      self.assertLess(n, 60)
    self.assertGreaterEqual(n - 1, 15)                         # the recovery dwell was real
    for o in outs[:-1]:
      self.assertEqual(o, mg.A_HOLD)                           # capped right up to the release
    self.assertEqual(outs[-1], 1.2)                            # freed only here
    self.assertEqual(f.state, mg.OFF)


class TestPostReleaseHold(OpenpilotTestCase):
  def test_rearm_is_blocked_for_five_seconds(self):
    """R1's anti-chatter bar: no arm -> release -> arm cycle shorter than 5 s. The hold caps the ask at -0.45 throughout,
    so the cooldown never hands the pedal back."""
    f = make()
    c = Ctx(f)
    c.arm(30, d=2.0, v=5.0)
    for i in range(60):
      c.push(1, d=float(np.interp(i, [0, 59], [2.0, 5.6])))
      if f.state != mg.BRAKE:
        break
    if f.state == mg.BRAKE:
      for i in range(40):
        c.push(1, d=5.6)
        if f.state != mg.BRAKE:
          break
    self.assertEqual(f.state, mg.HOLD)
    t_release = f.rearm_t
    # the gap collapses again straight away (a real second approach, well inside the cooldown)
    out = c.push(40, d=1.6, ask=1.0)
    self.assertEqual(f.state, mg.HOLD)                         # not re-armed: cooldown
    self.assertEqual(out, mg.A_HOLD)                           # and never throttle
    self.assertGreater(f.rearm_t, 0.0)
    # after the cooldown the second approach brakes
    out = c.push(int(round(f.rearm_t / 0.05)) + 2, d=1.6, ask=1.0)
    self.assertEqual(f.state, mg.BRAKE)
    self.assertEqual(out, -1.0)
    self.assertAlmostEqual(t_release, mg.REARM_COOLDOWN_S, delta=1.5)

  def test_no_limit_cycle_between_1_and_1_5(self):
    """R3: v oscillating across the 1.0 / 1.5 m/s fences at a 2 m gap must not produce CHATTER: no arm -> release ->
    arm inside 5 s, and the ask is never positive."""
    f = make()
    c = Ctx(f)
    c.push(40, d=3.0, v=1.4)                                   # rolling slowly, no standstill
    arms, releases, prev = [], [], f.state
    for i in range(200):                                       # 10 s of 0.8 <-> 1.6 m/s
      v = 1.6 if (i // 10) % 2 == 0 else 0.8
      c.push(1, d=2.0, v=v)
      if f.state == mg.BRAKE and prev != mg.BRAKE:
        arms.append(i)
      if f.state != mg.BRAKE and prev == mg.BRAKE:
        releases.append(i)
      prev = f.state
      self.assertLessEqual(c.out, 0.0)
    self.assertGreaterEqual(len(arms), 1)                      # it does engage at 1.6 m/s with a 2 m gap...
    for a in arms[1:]:                                         # ...but never sooner than 5 s after a release
      self.assertTrue(any(r <= a and (a - r) * 0.05 >= mg.REARM_COOLDOWN_S - 1e-9 for r in releases),
                      msg=f"re-arm at frame {a} inside the cooldown")


class TestDropoutBridge(OpenpilotTestCase):
  def test_dropout_drops_the_brake_holds_zero_throttle_and_keeps_the_buffer(self):
    """R4: a lead dropout releases the BRAKE (never brake a vanished lead) but keeps the throttle at zero for the bridge,
    keeps the median buffer, and resumes on the first re-acquired frame - no 0.25 s re-warm."""
    f = make()
    c = Ctx(f)
    c.arm(30, d=2.0, v=5.0)
    self.assertEqual(len(f._buf), mg.N_MED)
    out = c.push(10, present=False, ask=0.7)                   # 0.5 s of dropout
    self.assertEqual(out, mg.A_HOLD)
    self.assertEqual(f.state, mg.BRIDGE)
    self.assertEqual(len(f._buf), mg.N_MED)                    # buffer survived
    out = c.push(1, present=True, d=2.1, ask=0.7)
    self.assertEqual(f.state, mg.BRAKE)
    self.assertEqual(out, -1.0)                                # resumes on the first frame back

  def test_bridge_expires_into_off(self):
    f = make()
    c = Ctx(f)
    c.arm(30, d=2.0, v=5.0)
    c.push(int(round(mg.LEAD_BRIDGE_S / 0.05)) - 1, present=False, ask=0.7)
    self.assertEqual(f.state, mg.BRIDGE)                       # still inside the bridge
    self.assertEqual(c.push(2, present=False, ask=0.7), 0.7)   # the bridge expired: the ask is freed
    self.assertEqual(f.state, mg.OFF)
    self.assertEqual(len(f._buf), 0)
    self.assertEqual(c.push(1, present=True, d=2.0, ask=0.0), 0.0)  # a re-acquired lead must re-earn its samples
    self.assertEqual(f.state, mg.OFF)

  def test_untrusted_lead_bridges_instead_of_braking(self):
    f = make()
    c = Ctx(f)
    c.arm(30, d=2.0, v=5.0)
    out = c.push(4, prob=mg.MIN_LEAD_PROB - 0.1, ask=0.7)
    self.assertEqual(out, mg.A_HOLD)
    self.assertEqual(f.state, mg.BRIDGE)
    out = c.push(1, prob=1.0, d=2.0, ask=0.7)
    self.assertEqual(out, -1.0)
    self.assertEqual(f.state, mg.BRAKE)

  def test_bridge_from_the_hold(self):
    f = make()
    c = Ctx(f)
    c.arm(30, d=2.0, v=5.0)
    c.push(1, v=0.9)                                           # -> HOLD
    self.assertEqual(f.state, mg.HOLD)
    self.assertEqual(c.push(3, present=False), mg.A_HOLD)
    self.assertEqual(f.state, mg.BRIDGE)
    self.assertEqual(c.push(int(round(mg.LEAD_BRIDGE_S / 0.05)) + 2, present=False), 0.0)
    self.assertEqual(f.state, mg.OFF)

  def test_a_dropout_in_the_hold_cannot_re_brake(self):
    """A dropout inside the HOLD is not a resume: the release already happened, so the lead coming back must return to
    the hold, not re-arm the brake a third of a second after the release (the R1 chatter cycle)."""
    f = make()
    c = Ctx(f)
    c.arm(30, d=2.0, v=5.0)
    c.push(1, v=0.9)                                           # -> HOLD
    self.assertEqual(f.state, mg.HOLD)
    self.assertEqual(c.push(6, present=False), mg.A_HOLD)       # 0.3 s dropout inside the hold
    self.assertEqual(f.state, mg.BRIDGE)
    self.assertEqual(c.push(1, present=True, d=2.0, ask=0.5), mg.A_HOLD)   # lead back: still the hold
    self.assertEqual(f.state, mg.HOLD)
    for _ in range(40):                                        # and it stays the hold through the cooldown
      self.assertLessEqual(c.push(1, d=2.0, ask=1.0), 0.0)


  def test_a_lead_that_returns_far_starts_the_cooldown(self):
    """A dropout during the release dwell, with the lead returning beyond the exit band, is a RELEASE, not a resume: the
    hold takes over and the 5 s cooldown bars a re-brake even if the gap then collapses again."""
    f = make()
    c = Ctx(f)
    c.arm(30, d=2.0, v=5.0)
    for i in range(40):                                        # ramp into the exit band and start the dwell
      c.push(1, d=float(np.interp(i, [0, 39], [2.0, 5.6])))
    self.assertEqual(f.state, mg.BRAKE)
    self.assertGreater(f.gap_med, mg.exit_gap(5.0))
    self.assertGreater(f.exit_t, 0.0)
    c.push(6, present=False)                                   # 0.3 s dropout, mid-dwell
    self.assertEqual(f.state, mg.BRIDGE)
    out = c.push(1, present=True, d=5.8, ask=1.0)               # lead back beyond the exit band
    self.assertEqual(f.state, mg.HOLD)
    self.assertEqual(out, mg.A_HOLD)
    self.assertEqual(c.push(10, d=2.0, ask=1.0), mg.A_HOLD)     # gap collapses again: still the hold (cooldown)
    self.assertEqual(f.state, mg.HOLD)


class TestWiring(OpenpilotTestCase):
  def test_the_planner_reads_lead_model_prob_not_radar_state(self):
    """v1 wired `sm['radarState'].modelProb`; cereal has no such field (modelProb is on LeadData), so the first frame the
    planner ran would have raised AttributeError. Pin the correct source (read as text: importing the planner needs the
    acados generated code, which a source checkout does not have)."""
    p = os.path.join(os.path.dirname(__file__), "..", "..", "..", "selfdrive", "controls", "lib",
                     "longitudinal_planner.py")
    src = open(os.path.normpath(p)).read()
    self.assertNotIn("radarState'].modelProb", src)
    self.assertIn("getattr(_lead, 'modelProb'", src)

  def test_module_constants_match_the_deployed_brake_interface(self):
    self.assertAlmostEqual(mg.A_FLOOR, -1.0)
    self.assertAlmostEqual(mg.A_HOLD, -0.45)
    self.assertGreaterEqual(mg.REARM_COOLDOWN_S, 5.0)
    self.assertGreaterEqual(mg.EXIT_DWELL_S, 1.0)
    self.assertGreaterEqual(mg.RECOVER_DWELL_S, 1.0)
    self.assertGreaterEqual(mg.LEAD_BRIDGE_S, 1.0)
