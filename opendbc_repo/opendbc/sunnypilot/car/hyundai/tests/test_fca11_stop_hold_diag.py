"""0039: the stop-hold clutch lane answers "why was the pair never sent" from the LOG, and the frame the car produces at
the route-195 hold inputs is the frame that was on the wire (byte for byte).

WHAT HAPPENED (route 195, 2026-11-05). 4,234 SENT 0x38D frames that drive, 399 of them at level 30 (CR_VSM_DecCmd), and
ZERO carrying the pair: `Fca11LongBrake._stop_hold_ok` returned False on its FIRST clause, `not self.stop_hold_clutch`,
because `CP_SP.fca11StopHoldClutch` was never set - the param key was missing from the feed that builds the params list
(fixed in the superproject's initialize_params, patch-independent). Every input the gate reads was satisfied in the log:
vEgo 0.015-0.066 m/s, carState.standstill True, fastest wheel 0.00-0.31 km/h, TCS13 FCA_ACK = 1 (100 %/93 %), StandStill =
1 (100 %), level 30. Nothing in the log said WHICH clause refused.

These tests pin (a) the per-frame reason code, (b) that the reason reaches the episode line, (c) the exact wire bytes of
one of those held frames, rebuilt from its own camera frame, and (d) that only the pair bits differ once the lane is on.
"""
import os
import tempfile
import unittest

from opendbc.car import structs
from opendbc.sunnypilot.car.hyundai import fca11_long as fl
from opendbc.sunnypilot.car.hyundai import fca11_log
from opendbc.sunnypilot.car.hyundai.tests.test_fca11_long import _CS, _cc, _get_ci, _real_camera_frame
from opendbc.sunnypilot.car.interfaces import setup_interfaces

# Route 195, segment 195-2, t = 9851.314 / 9851.322 (logMonoTime): the camera's own 0x38D on bus 2 and the mirror frame the
# host sent 8 ms later, at vEgo 0.02 m/s, standstill, fastest wheel 0.00 km/h, TCS13 FCA_ACK = 1, StandStill = 1.
LOGGED_CAM = bytes.fromhex("00000800d07ffe94")     # CR_FCA_Alive = 10, DecCmd 0, Prefill 0, Warn 0
LOGGED_SENT = bytes.fromhex("191e1800d87ffed7")    # Prefill 1, DecCmd 30, Warn 3, CmdAct 1, DecCmdAct 0, HBA 0, StopReq 0


def _cp_sp(stop_hold):
  cp_sp = structs.CarParamsSP()
  cp_sp.fca11Brake = True
  cp_sp.enableGasInterceptor = True
  cp_sp.fca11StopHoldClutch = stop_hold
  return cp_sp


def _logged_cs(**over):
  """The CS the car layer published at those frames: every gate input satisfied (route-195 values)."""
  cs = _CS(v_ego=over.pop("v_ego", 0.02), standstill=over.pop("standstill", True), a_ego=-1.5)
  cs.fca11_wheel_max_kph = over.pop("wheel_max", 0.0)
  cs.fca11_esc_ack = over.pop("ack", True)
  cs.fca11_esc_standstill = over.pop("esc_standstill", True)
  assert not over, over
  return cs


def _m(level=30, prefill=True, warn=3, cmd_act=True, dec_cmd_act=False, stop_hold=False):
  return fl.Fca11MirrorFrame(dec_cmd=level, prefill=prefill, warn=warn, cmd_act=cmd_act, dec_cmd_act=dec_cmd_act,
                              stop_hold=stop_hold)


class TestBlocker(unittest.TestCase):
  """The reason code is the gate itself: `_stop_hold_ok` is `blocker == ""`, so it cannot drift."""

  def _b(self, stop_hold=True):
    return fl.Fca11LongBrake(structs.CarParams(), _cp_sp(stop_hold))

  def test_the_route_195_first_clause_is_named(self):
    """THE FIELD FAILURE: the lane off (CP_SP field never set) + every logged input satisfied + the exact held frame."""
    b = self._b(False)
    self.assertFalse(b.stop_hold_clutch)
    self.assertEqual("lane_off", b._stop_hold_blocker(_logged_cs(), _m(), False))
    self.assertFalse(b._stop_hold_ok(_logged_cs(), _m(), False))
    self.assertFalse(b._stop_hold_step(_logged_cs(), _m(), False))
    self.assertEqual({"lane_off": 1}, b.stop_hold_blockers)

  def test_all_inputs_satisfied_is_no_blocker(self):
    b = self._b()
    self.assertEqual("", b._stop_hold_blocker(_logged_cs(), _m(), False))
    self.assertTrue(b._stop_hold_ok(_logged_cs(), _m(), False))

  def test_each_clause_names_itself(self):
    """One mutation per term: the reason must name the term that failed (a name that never fires is not a diagnostic)."""
    b = self._b()
    cases = {
      "cal": (_logged_cs(), _m(), True),
      "shape": (_logged_cs(), _m(warn=2), False),
      "level": (_logged_cs(), _m(level=fl.FCA11_STOP_HOLD_MIN_LSB - 1), False),
      "standstill": (_logged_cs(standstill=False), _m(), False),
      "vego": (_logged_cs(v_ego=fl.FCA11_STOP_HOLD_V_MAX_MPS + 0.1), _m(), False),
      "wheel": (_logged_cs(wheel_max=fl.FCA11_STOP_HOLD_WHEEL_MAX_KPH + 0.1), _m(), False),
      "esc_ack": (_logged_cs(ack=False), _m(), False),
      "esc_standstill": (_logged_cs(esc_standstill=False), _m(), False),
    }
    for want, (cs, m, cal) in cases.items():
      with self.subTest(clause=want):
        self.assertEqual(want, b._stop_hold_blocker(cs, m, cal))
        self.assertFalse(b._stop_hold_ok(cs, m, cal))
    cs = _logged_cs()
    del cs.fca11_wheel_max_kph
    self.assertEqual("wheel_unpublished", b._stop_hold_blocker(cs, _m(), False))

  def test_spent_is_named_only_after_a_pair_was_sent(self):
    """The once-per-standstill latch: the reason is `spent` (not a data clause) once the lane used its shot and the ESC
    then stopped acknowledging at the same light."""
    b = self._b()
    self.assertTrue(b._stop_hold_step(_logged_cs(), _m(), False))          # pair rides this frame
    self.assertFalse(b._stop_hold_step(_logged_cs(ack=False), _m(), False))  # ESC stops acknowledging, still stopped
    self.assertEqual("", b._stop_hold_blocker(_logged_cs(), _m(), False), "every DATA clause holds again...")
    self.assertFalse(b._stop_hold_step(_logged_cs(), _m(), False))           # ...but the shot is spent
    self.assertEqual(1, b.stop_hold_blockers.get("spent"), "and the reason says so")
    b._stop_hold_step(_logged_cs(standstill=False), _m(), False)            # leaving standstill re-arms
    self.assertFalse(b._stop_hold_spent)


class TestWireBytes(unittest.TestCase):
  """Byte-level: the frame the car produced at that instant, rebuilt from its own camera frame."""

  def test_unpaired_reconstruction_reproduces_the_logged_frame(self):
    out = fl.build_fca11_frame(LOGGED_CAM, 11, _m())
    self.assertEqual(LOGGED_SENT, out)
    self.assertEqual((0, 0), (fl._get_bits(out, 1, 2), fl._get_bits(out, 21, 1)))

  def test_pair_changes_only_the_two_fields(self):
    base = fl.build_fca11_frame(LOGGED_CAM, 11, _m())
    held = fl.build_fca11_frame(LOGGED_CAM, 11, _m(stop_hold=True))
    self.assertEqual((1, 1), (fl._get_bits(held, 1, 2), fl._get_bits(held, 21, 1)))
    # every field the mirror owns is identical; only HBA / StopReq / the checksum byte differ
    for start, n in ((0, 1), (3, 2), (8, 8), (20, 1), (31, 1), (35, 4)):
      self.assertEqual(fl._get_bits(base, start, n), fl._get_bits(held, start, n), (start, n))
    self.assertNotEqual(base[7], held[7], "the checksum must follow the pair")


class TestEpisodeLine(unittest.TestCase):

  def _open_log(self):
    log = fca11_log.EpisodeLogger(tempfile.mkdtemp(prefix="fca11-0039-"))
    log.begin(0, 0.0, 0.0)
    log.note_frame(0, 30, 0.0, 0.0, -1.5, True, fl.FCA11_STOP_KPH)
    return log

  def test_census_is_on_the_episode_line(self):
    log = self._open_log()
    rec = log.episode_record(1, "release", fl.FCA11_STOP_KPH, 0, 12, {"lane_off": 7, "esc_ack": 3})
    self.assertEqual(12, rec["stop_hold_frames"])
    self.assertEqual({"esc_ack": 3, "lane_off": 7}, rec["stop_hold_blockers"])

  def test_additive_only(self):
    """The 0039 keys are additive: an older caller (no kwargs) still gets the full line, with zeros."""
    log = self._open_log()
    rec = log.episode_record(1, "release", fl.FCA11_STOP_KPH)
    self.assertEqual(0, rec["stop_hold_frames"])
    self.assertEqual({}, rec["stop_hold_blockers"])
    for k in ("frames", "dec_cmd_max", "hold_frames", "refused", "refused_panda", "plateaus", "stop_kph"):
      self.assertIn(k, rec)


class TestTheFieldStateFailsThroughTheRealPlumbing(unittest.TestCase):
  """The end-to-end signature, using the opendbc writer with the list the FIXED superproject now feeds: with the key the
  pair appears; with the key stripped (what card.py fed before the fix) it never does."""

  def _plist(self, with_key):
    pl = [{"HyundaiGasInterceptor": True}, {"HyundaiFca11Brake": True}]
    if with_key:
      pl.append({"HyundaiFca11StopHoldClutch": True})
    return pl

  def _law(self, with_key):
    CI, CP, CP_SP = _get_ci(None, None)
    setup_interfaces(CI, CP, CP_SP, self._plist(with_key))
    CS = _logged_cs()
    CS.fca11_cam_frame = (0, LOGGED_CAM)
    b = fl.Fca11LongBrake(CP, CP_SP)
    return b, CS

  def test_on_sends_the_pair_off_sends_none(self):
    b, CS = self._law(True)
    self.assertTrue(b.stop_hold_clutch)
    out = []
    for f in range(20):
      CS.fca11_cam_frame = (f * 10_000_000, LOGGED_CAM)
      CS.fca11_now_nanos = f * 10_000_000
      out += [bytes(s.dat) for s in b.update(_cc(-2.0), CS, f) if s.address == fl.FCA11_ADDR]
    self.assertTrue(out, "precondition: the law is holding")
    self.assertTrue(any((fl._get_bits(d, 1, 2), fl._get_bits(d, 21, 1)) == (1, 1) for d in out))
    self.assertGreater(b.stop_hold_frames, 0)

    b2, CS2 = self._law(False)
    self.assertFalse(b2.stop_hold_clutch)
    out2 = []
    for f in range(20):
      CS2.fca11_cam_frame = (f * 10_000_000, LOGGED_CAM)
      CS2.fca11_now_nanos = f * 10_000_000
      out2 += [bytes(s.dat) for s in b2.update(_cc(-2.0), CS2, f) if s.address == fl.FCA11_ADDR]
    self.assertTrue(out2, "precondition: the law is holding here too")
    self.assertFalse([d for d in out2 if (fl._get_bits(d, 1, 2), fl._get_bits(d, 21, 1)) == (1, 1)])
    self.assertEqual(0, b2.stop_hold_frames)
    self.assertEqual({"lane_off": len(out2)}, b2.stop_hold_blockers)


if __name__ == "__main__":
  unittest.main()
