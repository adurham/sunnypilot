"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

0050: under CAL the planner term is computed from the RAW ask, so a COASTING planner contributes 0 and the SCRIPTED
level is what goes on the wire. Before this the -0.5 onset clamp mapped through the 0.67 inverse to 8, so every
scripted level <= 8 was degenerate to 8 LSB on the wire (wsSTOP/m57 MEASUREMENT-5-7.md section 0.3). Pins:
  * the coasting-planner ladder 4/5/6/7/8 -> the EXACT scripted level (the falsifiable contrast vs 0049);
  * a soft planner ask (release band, -0.5 .. -0.3) under cal is not a hidden 8 LSB;
  * a release-band ask at/just below -0.5 under cal carries the scripted level, NOT the clamp floor (M8a/M8b);
  * a planner that IS braking (accel <= FCA11_BRAKE_ONSET_ACCEL) still wins over the scripted level;
  * the affine+cal branch takes the same raw ask (wire-neutral at levels >= CAL_MIN_LSB);
  * with cal OFF the planner path is byte-for-byte the pre-0050 law (the cal_on gate);
  * the panda contract with cal on (Warn 3, <= +RATE_STEP over the last SENT frame, <= 30, one passive close);
  * the scope (no plan + a coasting planner -> no frames at all).

NOTE on the safety wording: `max()` means "cal only ADDS braking" when the planner is braking (ask at or below
FCA11_BRAKE_ONSET_ACCEL). Inside the release band (-0.5 < accel <= -0.3) the scripted level is AUTHORITATIVE: a
coasting/soft planner term decays to 0 there (dec_cmd_from_accel(-0.49) = 0), so the cal frame may carry LESS than
the 0049 release-band floor (8 LSB) -- it can undercut production there. cal frames only; bounded and disclosed
(MEASUREMENT-5-7.md section 4.1; cal-fix REPORT sections 5-6).
"""
import unittest
from tempfile import TemporaryDirectory

from opendbc.car import structs
from opendbc.sunnypilot.car.hyundai import cal_mode as cm
from opendbc.sunnypilot.car.hyundai import fca11_long as fl
from opendbc.sunnypilot.car.hyundai.tests.test_fca11_long import _CalCS, _cal_plan, _cam, _write_plan

STEP = fl.HYUNDAI_FCA11_LONG_RATE_STEP


class _FeelSwitch:
  """Force FCA11_FEEL_ENABLED (read once per instance at construction), restoring the module value."""

  def __init__(self, on):
    self.on = on

  def __enter__(self):
    self.prev = fl.FCA11_FEEL_ENABLED
    fl.FCA11_FEEL_ENABLED = self.on
    return self

  def __exit__(self, *a):
    fl.FCA11_FEEL_ENABLED = self.prev


def _mk(d, fca11=True, affine=False, feel=None):
  cp_sp = structs.CarParamsSP(fca11Brake=fca11, enableGasInterceptor=True, fca11AffineGain=affine)
  with _FeelSwitch(fl.FCA11_FEEL_ENABLED if feel is None else feel):
    b = fl.Fca11LongBrake(structs.CarParams(), cp_sp)
  b.log.enabled = False
  if fca11:
    b.cal_seq = cm.CalSequencer(d)
    b.cal_seq.mark_session(1)
    b.cal_seq.set_phases(("steps",))
  return b


def _cc(accel=0.0):
  cc = structs.CarControl()
  cc.enabled = cc.longActive = True
  cc.actuators.accel = accel
  return cc


def _drive(b, cs, accel, frames):
  """Step `frames` 100 Hz frames at a fixed planner ask; return [(frame, dec, warn, cmdact, deccmdact, prefill)]."""
  out = []
  for f in range(frames):
    for s in b.update(_cc(accel), cs, f):
      dat = s.dat
      out.append((f, fl._get_bits(dat, 8, 8), fl._get_bits(dat, 3, 2),
                  fl._get_bits(dat, 20, 1), fl._get_bits(dat, 31, 1), fl._get_bits(dat, 0, 1)))
  return out


def _cal_run(d, level, accel=0.0, v_kph=35.0, frames=520, hold_s=3.0, affine=False, feel=None):
  """One plan with a single scripted rep; returns (brake, [(frame, dec, ...)] of every sent 0x38D)."""
  _write_plan(d, _cal_plan([(v_kph, level, hold_s)]))
  b = _mk(d, affine=affine, feel=feel)
  cs = _CalCS(v_ego=v_kph / 3.6, cam=_cam())
  return b, _drive(b, cs, accel, frames)


def _hold_frames(got, hold_s=3.0):
  """The sent-frame tuples inside the CAL HOLD window only. Beyond the hold the (unchanged) production path
  re-takes the same planner ask and ramps, so tests that pin the SCRIPTED plateau must look only here."""
  last = int(hold_s * 100)
  return [g for g in got if g[0] < last]


class TestCalLevelDelivery(unittest.TestCase):
  """A scripted cal hold must put the SCRIPTED level on the wire (the ladder 4..8), not a hidden 8 LSB."""

  def test_coasting_planner_delivers_levels_4_to_8(self):
    """The core falsifiable contrast. Planner coasting (accel 0.0); the cal frame's first sent level is the panda
    onset step 4, then a flat plateau at EXACTLY the scripted level. On the pre-0050 tree levels 4..7 plateau at
    8 (the -0.5 onset clamp through the 0.67 inverse), so this test fails there for 4/5/6/7 and passes with the fix."""
    with TemporaryDirectory() as d:
      for level in (4, 5, 6, 7, 8):
        _, got = _cal_run(d, level, accel=0.0)
        nonzero = [g[1] for g in got if g[1] > 0]
        self.assertTrue(nonzero, f"L={level}: cal must actuate")
        self.assertEqual(4, nonzero[0], f"L={level}: first SENT frame is the panda onset step")
        self.assertEqual({level}, set(nonzero[1:]), f"L={level}: plateau is the SCRIPTED level, not a hidden 8")
        self.assertEqual(level, max(nonzero), f"L={level}")

  def test_coasting_planner_delivers_higher_levels(self):
    """Above the onset the ladder still holds: ramp of <= +RATE_STEP per SENT frame, then flat at the level."""
    with TemporaryDirectory() as d:
      for level in (12, 25):
        _, got = _cal_run(d, level, accel=0.0)
        nonzero = [g[1] for g in got if g[1] > 0]
        self.assertEqual(4, nonzero[0], f"L={level}: first SENT frame is the onset step")
        self.assertEqual(level, max(nonzero), f"L={level}")
        self.assertTrue(all(v == level for v in nonzero[-20:]), f"L={level}: steady tail is flat at the level")

  def test_soft_planner_ask_under_cal_is_not_a_hidden_eight(self):
    """A planner ask in the release band (-0.5 .. -0.3, above the onset) must not inject the 8 LSB onset floor: at
    -0.2 the raw ask contributes dec_cmd_from_accel(-0.2) = 0 (coast territory, above the onset -> 0, NOT 3), so
    max() leaves the scripted level untouched. On the pre-0050 tree the clamped ask is -0.5 -> 8 -> the wire
    carries 8."""
    with TemporaryDirectory() as d:
      _, got = _cal_run(d, 5, accel=-0.2)
      nonzero = [g[1] for g in got if g[1] > 0]
      self.assertEqual(5, max(nonzero), "the scripted level, not the release-band 8")
      self.assertEqual({5}, set(nonzero[1:]))

  def test_release_band_at_onset_floor_carries_scripted_level(self):
    """F2 pin (kills the reviewer's survivor M8b: restrict the raw ask to accel > -0.45): a planner ask inside the
    release band at/just below the -0.5 onset (here -0.49) still has raw dec_cmd_from_accel(-0.49) = 0, so the
    scripted level is authoritative. Checked in the CAL HOLD window (frames < hold*100). The clamp-back mutant sends
    the 8 LSB release floor here -> this test fails (wire verified: tree -> {4}, M8b -> {8}).

    (The reviewer's other survivor, M8a `accel > FCA11_BRAKE_ONSET_ACCEL`, is an EQUIVALENT mutant, not a gap: with
    cal_on the authored raw ask and `accel` are only ever read when accel <= ONSET -- above the onset the law term is
    the same 0 either way -- so no wire test can distinguish it. Verified by a sweep over accel in [-2.0, 0.2].)
    On the pre-0050 tree the clamped ask is -0.5 -> 8 -> the wire carries 8, so this also fails there."""
    with TemporaryDirectory() as d:
      _, got = _cal_run(d, 4, accel=-0.49)
      nz = [g[1] for g in _hold_frames(got) if g[1] > 0]
      self.assertTrue(nz, "L=4 at -0.49: cal must actuate")
      self.assertEqual(4, nz[0], "first SENT frame is the onset step")
      self.assertEqual({4}, set(nz[1:]), "the scripted level, not the clamp floor")
      self.assertEqual(4, max(nz), "the cal frame may sit below the 8 LSB release-band floor")

  def test_release_band_plateau_is_flat_at_the_scripted_level(self):
    """F2 pin (release band, above the onset): at -0.4 the raw ask is 0, so the whole cal hold is flat at the
    scripted level. On the pre-0050 tree the clamped -0.5 -> 8 makes the hold 8 (falsifiable contrast)."""
    with TemporaryDirectory() as d:
      _, got = _cal_run(d, 5, accel=-0.4)
      nz = [g[1] for g in _hold_frames(got) if g[1] > 0]
      self.assertEqual(5, max(nz), "the scripted level, not the release-band floor")
      self.assertEqual({5}, set(nz[1:]))

  def test_planner_braking_under_cal_still_wins(self):
    """cal only ADDS braking WHEN the planner is braking: a HARD planner ask (accel <= ONSET) is a no-op for the
    onset clamp, so the mapping is unchanged and max() lets the planner win (here the personality tier cap, well
    above the scripted level). This is the regime where the "cal only ADDS braking" claim holds; inside the release
    band it does not (see test_release_band_at_onset_floor_carries_scripted_level)."""
    cap = fl.personality_max_dec(fl.PERSONALITY_STANDARD)
    with TemporaryDirectory() as d:
      _, got = _cal_run(d, 5, accel=-2.0)
      nonzero = [g[1] for g in got if g[1] > 0]
      self.assertEqual(cap, max(nonzero), "the planner ask dominates the scripted level")
      self.assertGreaterEqual(max(nonzero), fl.dec_cmd_from_accel(fl.FCA11_BRAKE_ONSET_ACCEL))

  def test_affine_plus_cal_takes_the_same_raw_ask(self):
    """The affine branch is changed identically. It is wire-neutral at levels >= CAL_MIN_LSB (the affine onset term
    is 1 LSB, below any scripted level) but must not regress: the scripted level still goes out with a coasting
    planner."""
    with TemporaryDirectory() as d:
      _, got = _cal_run(d, 5, accel=0.0, affine=True)
      nonzero = [g[1] for g in got if g[1] > 0]
      self.assertEqual(5, max(nonzero))
      self.assertEqual({5}, set(nonzero[1:]))

  def test_cal_contract_preserved_with_cal_on(self):
    """Panda-side authority is unchanged: Warn 3 and CmdAct 1 on every actuating frame, DecCmdAct 0, <= +RATE_STEP
    over the last SENT frame, <= 30 (the panda gate), and exactly one passive frame closes the episode."""
    with TemporaryDirectory() as d:
      for level in (4, 5, 7, 8, 25):
        _, got = _cal_run(d, level, accel=0.0)
        last = 0
        for f, dec, warn, cmdact, deccmdact, prefill in got:
          if dec > 0:
            self.assertEqual(3, warn, (level, f))
            self.assertEqual(1, cmdact, (level, f))
            self.assertEqual(0, deccmdact, (level, f))
            self.assertEqual(1, prefill, (level, f))
            self.assertLessEqual(dec, last + STEP, (level, f, dec, last))
            self.assertLessEqual(dec, fl.HYUNDAI_FCA11_LONG_MAX_DEC, (level, f))
            last = dec
          else:
            self.assertEqual(0, warn, (level, f))
            last = 0
        self.assertEqual(1, sum(1 for g in got if g[1] == 0), f"L={level}: exactly one passive close")


class TestCalOffIsUnchanged(unittest.TestCase):
  """The change is gated on cal_on: with cal OFF the planner path is the pre-0050 law byte-for-byte."""

  def test_planner_path_with_feel_off_is_unchanged(self):
    """OLD-LAW planner path (feel off, affine off, cal off = no plan). Open on a hard ask, then hold a RELEASE-BAND
    ask (-0.4): the pre-0050 law floors the ask at the onset (-0.5 -> 8 LSB, the release hysteresis), and this must
    stay 8. An UNGATED `accel` raw ask would emit dec_cmd_from_accel(-0.4) = 6 and drop the floor."""
    with TemporaryDirectory() as d:
      b = _mk(d, fca11=True, feel=False)
      b.cal_seq = None                                  # no cal at all
      cs = _CalCS(v_ego=35 / 3.6, cam=_cam())
      deep = _drive(b, cs, -2.0, 60)                    # hard ask: episode opens on frame 1 and ramps
      self.assertEqual([4], [g[1] for g in deep if g[1] > 0][:1])
      shallow = _drive(b, cs, -0.4, 40)                 # release band: holding, the ask is floored at the onset
      levels = [g[1] for g in shallow if g[1] > 0]
      self.assertTrue(levels, "the release-band ask must still hold the episode open")
      self.assertEqual({8}, set(levels), "the onset floor (8) is unchanged with cal off")

  def test_no_plan_no_cal_frames(self):
    """Scope: a coasting planner with NO plan emits nothing (the change never creates frames)."""
    with TemporaryDirectory() as d:
      b = _mk(d, fca11=True)
      b.cal_seq = None
      cs = _CalCS(v_ego=35 / 3.6, cam=_cam())
      self.assertEqual([], _drive(b, cs, 0.0, 200))


if __name__ == "__main__":
  unittest.main()
