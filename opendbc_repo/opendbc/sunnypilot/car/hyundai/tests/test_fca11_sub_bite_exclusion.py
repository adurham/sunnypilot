"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

0038-excl (lane F): the SUB-BITE brake frame must never be SENT alongside throttle.

opendbc/safety/modes/hyundai.h arms `host_brake_active_ts` on ANY actuating 0x38D (`CR_VSM_DecCmd != 0 || FCA_CmdAct
|| CF_VSM_DecCmdAct || CF_VSM_Prefill`, i.e. from 1 LSB) and refuses every non-zero GAS_COMMAND for the next
HYUNDAI_FCA11_LONG_BRAKE_THROTTLE_EXCL_US (100 ms). The pedal frame and the brake frame share the 50 Hz slot and the
pedal frame is appended first, so every actuating frame costs the pedal stream the frame 20 ms later. Below the
measured 4 LSB bite the brake does nothing physical while the throttle is held, and the comma pedal turns the missing
frames into a full lift-off: 102 ms without an applied frame -> FAULT_TIMEOUT -> DAC = the DRIVER's pedal only, latched
until an ENABLE=0 all-zero frame. That is the dip the owner calls flutter.

These tests drive the REAL host car layer (Elantra-N non-SCC, gas interceptor + FCA11 long) into the REAL panda TX hook
(libsafety, the same `TestHyundaiFca11LongSafety` fixture the safety tests use) and assert, per scenario, what the
0x38D stream carries, what the panda accepts, and whether the pedal command dips.
"""
import unittest

from opendbc.car import structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.hyundai.values import CAR
from opendbc.sunnypilot.car.hyundai import fca11_long as fl
from opendbc.sunnypilot.car.hyundai import gas_interceptor as gi
from opendbc.sunnypilot.car.interfaces import setup_interfaces
from opendbc.sunnypilot.car.hyundai.tests.test_gas_interceptor import (BASE_FINGERPRINT, STD, _Fca11CS,
                                                                       _fca11_cam_frame)

CAR_UNDER_TEST = CAR.HYUNDAI_ELANTRA_2022_NON_SCC
DT_US = 10_000          # 100 Hz controller frame
GHOST = (30.0, -25.0)   # a vision ghost: 30 m ahead, closing at 25 m/s


def _ci(cont: bool = True, fca11: bool = True):
  """The car layer this car runs: gas interceptor + FCA11 long + affine gain + dynamic brakes."""
  fl.FCA11_CONT_ENABLED = cont          # module constant, read once at construction (this is the ON/OFF switch)
  fingerprint = {i: {} for i in range(8)}
  fingerprint[0] = {**BASE_FINGERPRINT, **STD}
  CarInterface = interfaces[CAR_UNDER_TEST]
  CP = CarInterface.get_params(CAR_UNDER_TEST, fingerprint, [], alpha_long=False, is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, CAR_UNDER_TEST, fingerprint, [], alpha_long=False, is_release_sp=False,
                                     docs=False)
  setup_interfaces(CarInterface, CP, CP_SP, [
    {"HyundaiGasInterceptor": True}, {"HyundaiFca11Brake": fca11},
    {"HyundaiFca11AffineGain": True}, {"HyundaiFca11DynamicBrakes": True},
  ])
  return CarInterface(CP, CP_SP)


def _cc(accel):
  c = structs.CarControl()
  c.enabled = c.latActive = c.longActive = True
  c.actuators.accel = accel
  return c


class _Replay:
  """The host at 100 Hz into the real panda TX hook, frame by frame, in the order the host appends the frames."""

  def __init__(self, cont: bool = True, v: float = 20.0):
    self.ci = _ci(cont)
    import opendbc.safety.tests.test_hyundai as th
    self.tc = th.TestHyundaiFca11LongSafety_0("test_needs_controls_and_heartbeat")
    self.tc.setUp()
    self.cs = _Fca11CS(v_ego=v, cam=(0, _fca11_cam_frame(alive=5)))
    self.t = 0
    self.cmd_addr = int(self.ci.CC.interceptor_ids.command_addr)
    self.tc._open_window()

  def _refresh(self):
    tc = self.tc
    tc._rx(tc._user_brake_msg(False))
    tc._rx(tc._user_gas_msg(0))
    tc._rx(tc._interceptor_user_gas(0))
    tc._rx(tc._gear_msg(5))
    tc._rx(tc._wheel_msg(tc.IN_WINDOW_RAW))

  def run(self, ask: float, lead_frames, frames_before: int = 40, driver_brake_at: int | None = None):
    from opendbc.safety.tests.libsafety import libsafety_py
    rows = []
    plan = [None] * frames_before + list(lead_frames) + [None] * 40
    for f, lead in enumerate(plan):
      self.cs.fca11_lead_drel_m, self.cs.fca11_lead_vrel_mps = lead if lead is not None else (None, None)
      self.cs.fca11_lead_one = object() if lead is not None else None
      self.cs.fca11_now_nanos = f * DT_US * 1000
      self.cs.interceptor_state = 0
      sends = self.ci.CC.create_gas_command(_cc(ask), self.cs, f, 0)
      gas = next((bytes(d) for a, d, _s in sends if a == self.cmd_addr), None)
      brake = [bytes(d) for a, d, _s in sends if a == fl.FCA11_ADDR]
      self.t += DT_US
      self.tc.safety.set_timer(2_000_000 + self.t)
      self._refresh()
      if driver_brake_at is not None and f == driver_brake_at:
        self.tc._rx(self.tc._user_brake_msg(True))
      gas_ok = None if gas is None else bool(self.tc._tx(libsafety_py.make_CANPacket(self.cmd_addr, 0, gas)))
      brake_ok = [bool(self.tc._tx(libsafety_py.make_CANPacket(fl.FCA11_ADDR, 0, d))) for d in brake]
      rows.append(dict(f=f, gas=float(self.ci.CC.gas),
                       gas_en=bool((gas[4] >> 7) & 1) if gas is not None else None,
                       gas_word=((gas[0] << 8) | gas[1]) if gas is not None else None,
                       gas_sent=gas is not None, gas_ok=gas_ok,
                       lvl=brake[0][1] if brake else 0, brake_ok=brake_ok[0] if brake_ok else None,
                       muted=int(getattr(self.ci.CC.fca11_brake, "sub_bite_muted", 0)),
                       hold=bool(getattr(self.ci.CC.fca11_brake, "gas_hold", False))))
    return rows


def _refused(rows):
  return [(r["f"], r["lvl"]) for r in rows if r["gas_sent"] and not r["gas_ok"]]


def _actuating(rows, lo=1, hi=3):
  return [r for r in rows if r["lvl"] and lo <= r["lvl"] <= hi]


class TestSubBiteNeverRidesWithThrottle(unittest.TestCase):
  """The law's side: below the bite the frame is withheld; at/above the bite nothing changed; OFF is inert."""

  def tearDown(self):
    fl.FCA11_CONT_ENABLED = True

  def test_sub_bite_frame_is_withheld_while_the_pedal_commands_gas(self):
    for mf in (1, 2, 3, 5):
      with self.subTest(model_frames=mf):
        rows = _Replay().run(-0.20, [GHOST] * (5 * mf))
        if mf == 1:
          # 0040: a single 20 ms model frame no longer opens a sub-bite episode AT ALL - the wire quantiser's one-LSB
          # band means the slew cannot present a 1 LSB level to it (pre-0040 this frame was opened and then had to be
          # withheld). That is the stronger outcome, and it is pinned here rather than skipped: no brake frame at all.
          self.assertEqual(rows[-1]["muted"], 0)
          self.assertEqual(_actuating(rows), [], "a 1-frame phantom put a 1-3 LSB frame on the wire")
        else:
          # the ghost really did open an episode below the bite (otherwise the test proves nothing)
          self.assertGreater(rows[-1]["muted"], 0, "no sub-bite frame was ever withheld - the scenario missed the bite")
        # every frame that went out with a level carried exactly zero throttle
        for r in rows:
          if r["gas_sent"] and r["gas"] > 0.:
            self.assertEqual(0, r["lvl"], f"frame {r['f']}: level {r['lvl']} sent with gas {r['gas']:.3f}")
        # and the pedal command never dipped: a vision phantom must not touch the throttle
        self.assertGreater(min(r["gas"] for r in rows), 0., "the phantom cut the pedal")

  def test_sub_bite_frame_is_kept_when_the_pedal_is_at_true_zero(self):
    # a light brake that rides the coast: at 20 m/s the -0.65 ask is just past coast(20) = 0.349 m/s^2, so the law
    # sends 2-3 LSB and the pedal is ENABLE=0 exactly (true coast) - the one regime where a sub-bite frame is legal
    rows = _Replay().run(-0.65, [None] * 25)
    sub = _actuating(rows)
    self.assertTrue(sub, "no 1-3 LSB frame went out over the coast - the coast case is not exercised")
    self.assertTrue(all(not r["gas_en"] for r in rows if r["gas_sent"]), "the pedal was not at true zero on a coast")
    self.assertEqual(0, rows[-1]["muted"], "a coast frame was withheld - light braking over coast must be untouched")

  def test_a_real_bite_still_cuts_the_throttle(self):
    rows = _Replay().run(-0.20, [GHOST] * 50)
    bite = [r for r in rows if r["lvl"] >= fl.FCA11_FEEL_FIRST_LSB]
    self.assertTrue(bite, "the 500 ms phantom never reached the measured bite")
    self.assertTrue(all(r["gas"] == 0. for r in bite), "a >= 4 LSB frame did not force the throttle to exactly 0")

  def test_off_path_never_withholds_a_frame(self):
    rows = _Replay(cont=False).run(-0.20, [GHOST] * 25)
    self.assertEqual(0, rows[-1]["muted"], "the OFF path withheld a frame")
    self.assertTrue(all(r["lvl"] == 0 or r["gas"] == 0. for r in rows))


class TestNoPedalFrameIsRefused(unittest.TestCase):
  """The panda's side, through the real TX hook: the refused-frame storm is gone and the pedal never dips."""

  def tearDown(self):
    fl.FCA11_CONT_ENABLED = True

  def test_phantom_refuses_no_pedal_frame(self):
    for mf in (1, 2, 3, 5, 10):
      with self.subTest(model_frames=mf):
        rows = _Replay().run(-0.20, [GHOST] * (5 * mf))
        self.assertEqual([], _refused(rows), f"{mf} model frames: the panda refused pedal frames")
        self.assertTrue(all(r["brake_ok"] for r in rows if r["brake_ok"] is not None), "a 0x38D frame was refused")

  def test_closing_lead_with_the_planner_at_zero_refuses_no_pedal_frame(self):
    rows = _Replay().run(-0.02, [GHOST] * 25)
    self.assertEqual([], _refused(rows), "a closing-lead sub-bite brake refused pedal frames")
    self.assertGreater(min(r["gas"] for r in rows), 0., "the closing-lead brake cut the pedal")

  def test_light_brake_over_the_coast_refuses_no_pedal_frame(self):
    rows = _Replay().run(-0.65, [None] * 25)
    self.assertTrue(_actuating(rows), "the coast case sent no 1-3 LSB frame - nothing to compare")
    self.assertEqual([], _refused(rows))
    self.assertTrue(all(r["brake_ok"] for r in rows if r["brake_ok"] is not None))

  def test_a_real_bite_is_accepted_and_cuts_the_pedal(self):
    rows = _Replay().run(-0.20, [GHOST] * 50)
    self.assertEqual([], _refused(rows), "a real bite refuses pedal frames - it must cut them by COMMAND, not refusal")
    self.assertTrue(any(r["lvl"] >= fl.FCA11_FEEL_FIRST_LSB and r["gas"] == 0. for r in rows))

  def test_the_panda_window_mirror_is_armed_only_by_frames_that_went_out(self):
    # a sub-bite phantom arms nothing (no refusal possible), a >= 4 LSB frame arms 110 ms (> the panda's 100 ms)
    re_ = _Replay()
    rows = re_.run(-0.20, [GHOST] * 50)
    arm = [r["f"] for r in rows if r["hold"]]
    self.assertTrue(arm, "the bite never armed the gas window")
    self.assertGreaterEqual(max(arm) - min(arm), 10, "the armed window is shorter than the panda's exclusion window")


if __name__ == "__main__":
  unittest.main()
