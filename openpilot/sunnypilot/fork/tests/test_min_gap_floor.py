"""
Tests for the fork's low-speed minimum-gap brake floor (sunnypilot/fork/min_gap_floor.py, param MinGapBrakeFloor).

The fixture frames are the REAL ones from route 000001ac, segment 59 (2026-11-19, I-80/94 stop-and-go behind a flat-rear
RV trailer): the vRel spike burst at 59:35.16-59:35.72 and the gap collapse at 59:40.32-59:40.96.
"""
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


# --- real frames, route 000001ac seg 59 (elapsed t, radarState.leadOne.dRel, radarState.leadOne.vRel)
VREL_SPIKE_59_35 = [
  (3575.164, 4.638, 3.356), (3575.221, 7.469, 13.704), (3575.264, 6.386, 10.234),
  (3575.322, 8.895, 15.008), (3575.374, 8.505, 10.211), (3575.413, 9.190, 12.098),
  (3575.465, 8.395, 10.252), (3575.514, 6.625, 7.463), (3575.573, 5.098, 3.162),
  (3575.616, 4.867, 1.818), (3575.672, 4.371, 1.443), (3575.720, 3.855, 0.574),
]
# the next frames, after the jump: the gap is now 3.1-3.5 m while vRel is STILL positive (the trailer read as pulling away)
VREL_SPIKE_LATE_59_35 = [(3575.770, 3.787, 0.523), (3575.815, 3.700, 0.501), (3575.864, 3.472, 1.142),
                         (3575.920, 3.393, 1.598), (3575.970, 5.313, 7.053)]
GAP_COLLAPSE_59_40 = [(3580.319, 2.965), (3580.362, 2.681), (3580.414, 2.539), (3580.474, 2.806), (3580.521, 2.642),
                      (3580.570, 2.130), (3580.613, 1.908), (3580.670, 1.836), (3580.719, 1.854), (3580.763, 1.722)]


def make(enabled=True, dt=0.05):
  return mg.MinGapFloor(params=FakeParams(enabled=enabled), dt=dt)


def feed(f, n, d_rel, v_ego, ask=0.0, **kw):
  """Push n frames of the same (gap, speed) and return the last ask."""
  active = kw.get("active", True)
  gas = kw.get("gas_pressed", False)
  present = kw.get("lead_present", True)
  prob = kw.get("lead_prob", 1.0)
  stand = kw.get("standstill", False)
  out = ask
  for _ in range(n):
    out = f.update(ask, v_ego, active, gas, present, d_rel, prob, stand)
  return out


class TestGapGeometry(OpenpilotTestCase):
  def test_entry_and_exit_bands(self):
    # entry 0.5 s / 3 m, exit 0.8 s / 5 m; the exit band is always the wider one
    self.assertAlmostEqual(mg.enter_gap(5.0), 3.0)
    self.assertAlmostEqual(mg.enter_gap(8.0), 4.0)
    self.assertAlmostEqual(mg.exit_gap(5.0), 5.0)
    self.assertAlmostEqual(mg.exit_gap(8.0), 6.4)
    for v in np.arange(0.0, 15.0, 0.25):
      self.assertGreater(mg.exit_gap(v), mg.enter_gap(v))

  def test_floor_ask_never_raises(self):
    for a in np.arange(-3.5, 2.01, 0.05):
      self.assertEqual(mg.floor_ask(float(a), True), min(float(a), -1.0))
      self.assertEqual(mg.floor_ask(float(a), False), float(a))


class TestOffAndFailsafe(OpenpilotTestCase):
  def test_off_is_byte_identical(self):
    """OFF-invariance at the rule level: with the param at 0 the output IS the input on every frame of a 500-frame walk."""
    f = make(enabled=False)
    rng = np.random.default_rng(7)
    ask = 0.0
    for i in range(500):
      ask = float(np.clip(ask + rng.normal(0.0, 0.2), -3.5, 2.0))
      v = float(np.clip(5.0 + rng.normal(0.0, 1.0), 0.0, 12.0))
      d = float(np.clip(2.5 + rng.normal(0.0, 1.5), 0.3, 20.0))
      self.assertEqual(f.update(ask, v, True, False, True, d, 1.0, False), ask)
    self.assertFalse(f.on)

  def test_unknown_key_stays_off(self):
    f = mg.MinGapFloor(params=FakeParams(enabled=True, unknown=True))
    self.assertEqual(feed(f, 20, 1.5, 5.0, ask=0.4), 0.4)

  def test_lost_lead_releases_and_clears(self):
    f = make()
    self.assertEqual(feed(f, 10, 2.0, 5.0), -1.0)
    self.assertTrue(f.on)
    # lead drops for 3 frames: OFF immediately, median buffer cleared
    for _ in range(3):
      self.assertEqual(f.update(0.0, 5.0, True, False, False, 0.0, 1.0, False), 0.0)
    self.assertFalse(f.on)
    self.assertEqual(len(f._buf), 0)
    # a re-acquired lead needs its own 0.25 s of samples before it may ADD a brake
    self.assertEqual(feed(f, mg.MIN_SAMPLES - 1, 1.2, 5.0), 0.0)
    self.assertEqual(feed(f, 1, 1.2, 5.0), -1.0)

  def test_low_model_prob_is_off(self):
    f = make()
    self.assertEqual(feed(f, 30, 1.5, 5.0, lead_prob=mg.MIN_LEAD_PROB - 0.01), 0.0)
    self.assertFalse(f.on)
    # the same geometry with a trusted track does fire, so the guard is what held it off
    self.assertEqual(feed(f, mg.MIN_SAMPLES, 1.5, 5.0, lead_prob=1.0), -1.0)

  def test_driver_gas_is_off(self):
    f = make()
    self.assertEqual(feed(f, 30, 1.5, 5.0, gas_pressed=True), 0.0)

  def test_planner_reset_is_off(self):
    f = make()
    self.assertEqual(feed(f, 30, 1.5, 5.0, active=False), 0.0)
    self.assertFalse(f.on)

  def test_standstill_is_off(self):
    f = make()
    self.assertEqual(feed(f, 30, 1.2, 1.8, standstill=True), 0.0)


class TestArmingEnvelope(OpenpilotTestCase):
  def test_no_entry_below_v_min_launch_is_untouched(self):
    """A launch from rest is owned by the standstill/launch logic: 0-1.5 m/s never arms, at any gap."""
    for v in (0.0, 0.4, 1.0, 1.49):
      f = make()
      self.assertEqual(feed(f, 40, 1.0, v), 0.0, msg=f"v={v}")

  def test_a_launch_from_a_short_queue_gap_is_not_braked(self):
    """The review's catch: the median buffer is pre-warmed during a standstill, so a normal pull-away from a 2.5 m queue
    stop would arm the floor as soon as v passed 1.5 m/s. Standing still must start the 2 s launch grace."""
    f = make()
    self.assertEqual(feed(f, 20, 2.5, 0.0, standstill=True), 0.0)   # 1 s held at a normal queue gap
    self.assertAlmostEqual(f.launch_hold, mg.LAUNCH_GRACE_S)
    out = 0.0
    for i in range(39):                                            # the 2 s grace: 39 frames of a pull-away, v 0.15 -> 3.0 m/s
      v = min(3.0, 0.075 * (i + 1) * 2)
      out = f.update(0.0, v, True, False, True, 2.5, 1.0, False)
      self.assertEqual(out, 0.0, msg=f"frame {i} v={v:.2f}")
    # grace spent and the gap still short (a lead that is NOT pulling away): now it arms
    self.assertEqual(f.update(0.0, 3.0, True, False, True, 2.5, 1.0, False), -1.0)

  def test_no_entry_above_v_max(self):
    f = make()
    self.assertEqual(feed(f, 40, 1.0, mg.V_MAX + 0.5), 0.0)

  def test_hysteresis_holds_through_the_noise_band(self):
    """Inside the 3-5 m band the state must not flip frame to frame - the reason the exit band is wider than the entry."""
    f = make()
    self.assertEqual(feed(f, 10, 2.2, 4.0), -1.0)          # armed at 2.2 m
    rng = np.random.default_rng(11)
    for _ in range(200):                                   # jitter 3.0-4.9 m, the vision noise band at this range
      d = float(np.clip(3.9 + rng.normal(0.0, 0.55), 2.9, 4.9))
      self.assertEqual(f.update(0.0, 4.0, True, False, True, d, 1.0, False), -1.0, msg=f"d={d:.2f}")
    # only the WIDER exit band releases - and only once the 0.5 s MEDIAN is across it (one frame cannot move it)
    self.assertEqual(feed(f, 5, 5.02, 4.0), -1.0)    # 5 of 10 samples: median still ~4.5 m
    self.assertEqual(feed(f, 1, 5.02, 4.0), 0.0)     # 6 of 10: median is now 5.02 m
    self.assertFalse(f.on)

  def test_release_below_v_release(self):
    f = make()
    self.assertEqual(feed(f, 10, 1.5, 3.0), -1.0)
    self.assertEqual(feed(f, 1, 1.5, mg.V_RELEASE - 0.05), 0.0)


class TestTheRealFrames(OpenpilotTestCase):
  def test_the_59_40_gap_collapse_arms(self):
    """The frame sequence that produced Adam's 59:45 brake: the 0.5 s median is 2.33 m at v 5.3 m/s, i.e. 0.44 s headway."""
    f = make()
    out = 0.0
    for i, (t, d) in enumerate(GAP_COLLAPSE_59_40):
      out = f.update(0.0, 5.28, True, False, True, d, 1.0, False)
      if i + 1 >= mg.MIN_SAMPLES:
        self.assertEqual(out, -1.0, msg=f"frame {i} d={d}")
    self.assertAlmostEqual(f.gap_med, float(np.median([d for _, d in GAP_COLLAPSE_59_40])), places=6)
    self.assertAlmostEqual(f.gap_med, 2.3345, places=4)
    self.assertAlmostEqual(out, mg.A_FLOOR)

  def test_the_59_35_vrel_spikes_neither_fire_nor_disarm(self):
    """The vision failure itself: dRel jumps 4.6 -> 8.9 m with vRel +13.7/+15.0 while the car is closing.

    Two claims, both from the same real frames: (a) at 4-9 m the floor stays off - it is not a highway rule; (b) once the
    gap is genuinely 3.3-3.8 m with vRel STILL positive (the trailer "pulling away"), the floor arms anyway, because it
    never reads vRel. The vRel value is passed in and must make no difference to the decision.
    """
    f = make()
    for t, d, vrel in VREL_SPIKE_59_35:
      # vRel is not an input to the rule at all: the call is identical whatever it says
      self.assertEqual(f.update(0.0, 3.8, True, False, True, d, 1.0, False), 0.0, msg=f"t={t} d={d} vRel={vrel}")
    self.assertFalse(f.on)
    # the gap now sits at 3.4-3.8 m, vRel still positive: median 3.70 m < enter_gap(3.8) = 3.0? no -> 3.7 > 3.0, still off
    out = 0.0
    for t, d, vrel in VREL_SPIKE_LATE_59_35:
      out = f.update(0.0, 3.8, True, False, True, d, 1.0, False)
    self.assertGreater(f.gap_med, mg.enter_gap(3.8))
    self.assertEqual(out, 0.0)
    # and the same vRel burst around a genuinely short gap arms: gap wins, vRel is irrelevant
    f2 = make()
    self.assertEqual(feed(f2, mg.N_MED, 2.2, 3.8), -1.0)

  def test_a_vrel_claim_cannot_change_the_output(self):
    """There is no path from vRel to the ask, so a bad lead speed cannot disarm the floor.

    Structural, not statistical: the rule has no vRel input at all. (The replay round that made this a brake-path change
    is what proves the behaviour end to end; this pins the shape that makes it possible.)
    """
    import inspect
    params = {p.lower() for p in inspect.signature(mg.MinGapFloor.update).parameters}
    self.assertFalse({p for p in params if p in ("v_rel", "vrel", "v_lead", "vlead", "closing")})
    self.assertEqual(set(inspect.signature(mg.floor_ask).parameters), {"accel", "on"})


class TestWiring(OpenpilotTestCase):
  def test_planner_applies_it_after_no_close_and_before_the_launch_debounce(self):
    """The floor must be the LAST word on the ask before the standstill debounce.

    Read as TEXT, not imported: importing longitudinal_planner pulls the acados long-MPC c_generated_code, which is not
    built in a source checkout on this host (the known environment gap, not a land defect).
    """
    import pathlib
    root = pathlib.Path(__file__).resolve().parents[4]
    src = (root / "openpilot/selfdrive/controls/lib/longitudinal_planner.py").read_text()
    i_nc = src.index("self.no_close.update")
    i_floor = src.index("self.min_gap_floor.update")
    i_sl = src.index("self.standstill_launch.update")
    self.assertLess(i_nc, i_floor)
    self.assertLess(i_floor, i_sl)
    self.assertIn("self.min_gap_floor = MinGapFloor()", src)
    self.assertIn("float(sm['radarState'].modelProb)", src)
