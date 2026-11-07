"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

PLANT-CURVE PLATEAU instrumentation (fca11_log.PlateauTracker, fed from Fca11LongBrake._log_tick): every braking episode
records its sustained constant-SENT-level windows (>= 0.6 s, v > 4 m/s, no pedal) with the steady-part mean v / aEgo,
so ordinary driving measures the brake plant curve - including the never-observed > 20 LSB region. Instrumentation only:
the last class proves the emitted 0x38D stream is byte-identical with and without it.
"""
import json
import os
import tempfile
import unittest
from unittest import mock

from opendbc.car import structs
from opendbc.sunnypilot.car.hyundai import fca11_log as lg
from opendbc.sunnypilot.car.hyundai import fca11_long as fl
from opendbc.sunnypilot.car.hyundai.tests.test_fca11_long import _CS, _cam, _cc, _cp_sp, _decs, _drive_scenario


def _feed(t, frames, lsb, v=20.0, a=-1.6, gas=False, brake=False, step=2):
  for f in frames[::step]:
    t.sample(f, lsb, v, a, gas, brake, step)


class TestPlateauTracker(unittest.TestCase):
  """The pure accumulator, at the 50 Hz send cadence the controller feeds it."""

  def test_one_steady_window(self):
    t = lg.PlateauTracker()
    _feed(t, range(0, 100), 26)                      # 1.0 s at 26 LSB (samples 0..98)
    p = t.record()
    self.assertEqual(1, len(p))
    self.assertEqual(26, p[0]["lsb"])
    self.assertEqual(0.98, p[0]["dur_s"])
    self.assertEqual(20.0, p[0]["v_mps"])
    self.assertEqual(-1.6, p[0]["a_mss2"])
    self.assertEqual(35, p[0]["n"])                  # frames 30..98 step 2: the first 0.3 s is skipped

  def test_settle_part_is_excluded_from_the_mean(self):
    t = lg.PlateauTracker()
    for f in range(0, 100, 2):                       # transient -0.5 in the first 0.3 s, steady -2.0 after
      t.sample(f, 26, 20.0, -0.5 if f < 30 else -2.0, False, False, 2)
    self.assertEqual(-2.0, t.record()[0]["a_mss2"])

  def test_short_window_dropped(self):
    t = lg.PlateauTracker()
    _feed(t, range(0, 58), 26)                       # 0.56 s < 0.6 s
    self.assertEqual([], t.record())

  def test_pedal_press_breaks_the_window(self):
    for kw in (dict(gas=True), dict(brake=True)):
      t = lg.PlateauTracker()
      _feed(t, range(0, 50), 26)                     # 0.5 s
      _feed(t, range(50, 54), 26, **kw)              # pedal
      _feed(t, range(54, 104), 26)                   # 0.5 s again
      self.assertEqual([], t.record(), kw)

  def test_level_change_breaks_the_window(self):
    t = lg.PlateauTracker()
    _feed(t, range(0, 50), 26)
    _feed(t, range(50, 52), 25)
    _feed(t, range(52, 102), 26)
    self.assertEqual([], t.record())

  def test_low_speed_and_send_gap_break_the_window(self):
    t = lg.PlateauTracker()
    _feed(t, range(0, 200), 26, v=4.0)               # not > 4 m/s
    self.assertEqual([], t.record())
    t = lg.PlateauTracker()
    _feed(t, range(0, 50), 26)
    _feed(t, range(60, 110), 26)                     # a 10-frame hole (no actuating frame) splits it
    self.assertEqual([], t.record())

  def test_repeats_merge_and_count_is_capped(self):
    t = lg.PlateauTracker()
    f = 0
    for lsb in list(range(4, 4 + lg.PLATEAU_MAX + 3)) + [4]:   # 15 distinct levels, then 4 again
      _feed(t, range(f, f + 80), lsb, a=-0.1 * lsb)
      f += 80
    p = t.record()
    self.assertEqual(lg.PLATEAU_MAX, len(p))
    self.assertEqual(3, t.dropped)
    self.assertEqual(4, p[0]["lsb"])
    self.assertEqual(2, p[0]["runs"])                # the repeat of 4 merged into the first entry
    self.assertEqual(-0.4, p[0]["a_mss2"])
    self.assertEqual([4 + i for i in range(lg.PLATEAU_MAX)], [x["lsb"] for x in p])


class _Drive:
  def __init__(self, d, cp_sp=None):
    self.dir = d
    self.b = fl.Fca11LongBrake(structs.CarParams(), cp_sp or _cp_sp())
    self.b.log = fl.fca11_log.EpisodeLogger(d)
    self.f = 0
    self.levels: list[int] = []

  def run(self, n, ask, **cs):
    for _ in range(n):
      self.levels += _decs(self.b.update(_cc(ask), _CS(cam=_cam(), **cs), self.f))
      self.f += 1

  def records(self):
    path = os.path.join(self.dir, "episodes.jsonl")
    if not os.path.exists(path):
      return []
    lines = open(path).read().splitlines()
    return [json.loads(x) for x in lines]


class TestPlateauEpisodeRecord(unittest.TestCase):
  """End to end through Fca11LongBrake.update -> _log_tick -> episodes.jsonl."""

  def test_steady_26_lsb_at_20_mps(self):
    with tempfile.TemporaryDirectory() as d:
      dr = _Drive(d)
      dr.run(300, -3.0, v_ego=20.0, a_ego=-1.9)      # urgent lane: ramps 4..24 then holds 26
      dr.run(2, 0.5, v_ego=20.0, brake=True)         # driver brake ends the episode
      self.assertEqual(26, max(dr.levels))
      self.assertGreaterEqual(dr.levels.count(26), 140)   # ~2.9 s of 26 LSB on the wire
      recs = dr.records()
      self.assertEqual(1, len(recs))
      p = recs[0]["plateaus"]
      self.assertEqual(1, len(p), p)
      self.assertEqual(26, p[0]["lsb"])
      self.assertEqual(20.0, p[0]["v_mps"])
      self.assertEqual(-1.9, p[0]["a_mss2"])
      self.assertGreaterEqual(p[0]["dur_s"], 2.5)
      self.assertEqual(0, recs[0]["plateaus_dropped"])
      # the original schema is intact and the line is one bounded line
      for k in ("dec_cmd_min", "dec_cmd_max", "dec_cmd_last", "aego_min_mss2", "dur_s", "v_kph_start", "end_reason",
                "refused", "refused_panda", "reached_stop", "stop_kph"):
        self.assertIn(k, recs[0])
      self.assertEqual(1, len(open(os.path.join(d, "episodes.jsonl")).read().splitlines()))

  def test_pedal_press_before_0p6_s_yields_none(self):
    with tempfile.TemporaryDirectory() as d:
      dr = _Drive(d)
      dr.run(60, -3.0, v_ego=20.0, a_ego=-1.9)       # ramp + < 0.6 s at 26
      dr.run(2, -3.0, v_ego=20.0, brake=True)        # brake press: the episode (and window) ends
      recs = dr.records()
      self.assertEqual(1, len(recs))
      self.assertEqual([], recs[0]["plateaus"])

  def test_no_qualifying_window_is_an_empty_list(self):
    with tempfile.TemporaryDirectory() as d:
      dr = _Drive(d)
      dr.run(300, -3.0, v_ego=3.0, a_ego=-1.0)       # long and steady but below the 4 m/s gate
      dr.run(2, 0.5, v_ego=3.0, brake=True)
      recs = dr.records()
      self.assertEqual(1, len(recs))
      self.assertEqual([], recs[0]["plateaus"])

  def test_level_change_mid_window_yields_none_for_either_level(self):
    with tempfile.TemporaryDirectory() as d:
      dr = _Drive(d)
      dr.run(50, -3.0, v_ego=20.0, a_ego=-1.9)       # ramp, then 26 for ~0.4 s
      dr.run(50, -1.5, v_ego=20.0, a_ego=-1.2)       # level steps down: the 26 window ends < 0.6 s
      dr.run(2, 0.5, v_ego=20.0, brake=True)
      recs = dr.records()
      self.assertEqual(1, len(recs))
      self.assertNotIn(26, [p["lsb"] for p in recs[0]["plateaus"]])

  def test_shadow_instance_episode_log_carries_plateaus(self):
    """In Shadow the wire instance (OLD-LAW) writes the SAME episodes.jsonl through the same _log_tick, so it carries the
    plateaus too; the V3 core's episode log stays disabled, and no extra file appears (FCA11_LOG_DIR is the one dir
    both the episode log and the shadow stream resolve to)."""
    cp_sp = _cp_sp()
    cp_sp.fca11Shadow = True
    with tempfile.TemporaryDirectory() as d, mock.patch.dict(os.environ, {"FCA11_LOG_DIR": d}):
      dr = _Drive(d, cp_sp)
      self.assertTrue(dr.b.shadow)
      self.assertFalse(dr.b._shadow_v3.log.enabled)
      for _ in range(300):
        dr.run(1, -3.0, v_ego=20.0, a_ego=-1.9)
        dr.b.v3_post_send()
      dr.run(2, 0.5, v_ego=20.0, brake=True)
      dr.b.v3_post_send()
      recs = dr.records()
      self.assertEqual(1, len(recs))
      self.assertEqual([26], [p["lsb"] for p in recs[0]["plateaus"]])
      self.assertEqual(-1.9, recs[0]["plateaus"][0]["a_mss2"])
      # exactly the two pre-existing streams: the episode log and the shadow trace - no new plateau file
      self.assertEqual({"episodes.jsonl", "fca11-shadow.jsonl"}, set(os.listdir(d)))


def _identity_stream(make_log):
  """A 3 s urgent 26 LSB hold at 20 m/s, then the 600-frame identity scenario from test_fca11_long; returns raw frames."""
  b = fl.Fca11LongBrake(structs.CarParams(), _cp_sp())
  b.log = make_log()
  out = []
  drive = structs.CarState.GearShifter.drive
  for f in range(950):
    if f < 300:     # 3 s urgent-lane hold at 20 m/s (26 LSB), then an ask-driven release, then the identity scenario
      ask, v, ss, brake, gas, gear, fresh = -3.0, 20.0, False, False, False, drive, True
    elif f < 350:
      ask, v, ss, brake, gas, gear, fresh = 0.5, 20.0, False, False, False, drive, True
    else:
      ask, v, ss, brake, gas, gear, fresh = _drive_scenario(f - 350)
    cs = _CS(v_ego=v, gear=gear, brake=brake, gas=gas, standstill=ss, cam=_cam(),
             now=0 if fresh else int(0.2 * 1e9), a_ego=-1.7)   # the Test18xOffIsTheDeployedLaw harness (stale = +0.2 s)
    out += [(s.address, bytes(s.dat), s.src) for s in b.update(_cc(ask), cs, f)]
  return out


class TestPlateauCannotPerturbTheLaw(unittest.TestCase):

  def test_emitted_frames_byte_identical(self):
    with tempfile.TemporaryDirectory() as d:
      with_inst = _identity_stream(lambda: fl.fca11_log.EpisodeLogger(d))
      recs = [json.loads(x) for x in open(os.path.join(d, "episodes.jsonl")).read().splitlines()]
      self.assertTrue(any(26 in [p["lsb"] for p in r["plateaus"]] for r in recs), "the 26 LSB hold was logged")
    no_log = _identity_stream(lambda: fl.fca11_log.EpisodeLogger("0"))
    with tempfile.TemporaryDirectory() as d, mock.patch.object(lg.EpisodeLogger, "note_sent", lambda *a, **k: None):
      no_inst = _identity_stream(lambda: fl.fca11_log.EpisodeLogger(d))

    def _boom(*a, **k):
      raise RuntimeError("tracker fault")
    with tempfile.TemporaryDirectory() as d, mock.patch.object(lg.PlateauTracker, "sample", _boom), \
         mock.patch.object(lg.PlateauTracker, "record", _boom):
      faulty = _identity_stream(lambda: fl.fca11_log.EpisodeLogger(d))
      recs = [json.loads(x) for x in open(os.path.join(d, "episodes.jsonl")).read().splitlines()]
      self.assertTrue(recs and all(r["plateaus"] == [] for r in recs), "a tracker fault costs only the plateau list")
    self.assertGreater(len(with_inst), 100)
    self.assertTrue(any(fl._get_bits(dat, 8, 8) == 26 for _, dat, _ in with_inst))
    self.assertEqual(no_inst, with_inst)
    self.assertEqual(no_log, with_inst)
    self.assertEqual(faulty, with_inst)


if __name__ == "__main__":
  unittest.main()
