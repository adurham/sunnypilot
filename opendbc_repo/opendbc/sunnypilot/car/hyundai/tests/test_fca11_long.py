"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

FCA11 long braking (HyundaiFca11Brake, default OFF) car-layer unit tests:
  - the pure window / gain / mirror-frame logic (fca11_long.py);
  - the mirror frame against bytes in the real camera layout (override fields only, alive+1, CRC, undefined bits kept);
  - the capnp CarParamsSP field round-trip and the interface plumbing (toggle on/off + stale-libparams);
  - the planner PID accel limits per speed (the interface override);
  - the alert emission in car_specific (via the selfdrived path).
"""
import copy
import json
import os
import tempfile
import time
import unittest

from opendbc.car import structs
from opendbc.car.hyundai.hyundaican import hyundai_checksum
from opendbc.car.car_helpers import interfaces
from opendbc.car.hyundai.values import CAR
from opendbc.sunnypilot.car.hyundai import fca11_long as fl
from opendbc.sunnypilot.car.hyundai.values import (HyundaiSafetyFlagsSP, HYUNDAI_FCA11_LONG_MAX_DEC)
from opendbc.sunnypilot.car.interfaces import setup_interfaces

CAR_UNDER_TEST = CAR.HYUNDAI_ELANTRA_2022_NON_SCC
BASE_FINGERPRINT = {0x260: 8, 0x371: 8, 0x386: 8, 0x394: 8, 0x251: 8, 0x4F1: 4, 0x340: 8}
STD = {0x201: 6}


def _get_ci(param: bool | None, fca11: str | bytes | None = None):
  fingerprint = {i: {} for i in range(8)}
  fingerprint[0] = {**BASE_FINGERPRINT, **STD}
  CarInterface = interfaces[CAR_UNDER_TEST]
  CP = CarInterface.get_params(CAR_UNDER_TEST, fingerprint, [], alpha_long=False, is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, CAR_UNDER_TEST, fingerprint, [], alpha_long=False, is_release_sp=False, docs=False)
  params_list: list = []
  if param is not None:
    params_list.append({"HyundaiGasInterceptor": param})
  if fca11 is not None:
    params_list.append({"HyundaiFca11Brake": fca11})
  setup_interfaces(CarInterface, CP, CP_SP, params_list)
  return CarInterface, CP, CP_SP


def _real_camera_frame(dec=0, warn=0, alive=5, prefill=0, cmd_act=0, dec_cmd_act=0, hba=0, stop_req=0, undefined_high=True):
  """A frame in the exact layout of the camera's 0x38D, including the always-set undefined byte4 bit7."""
  b = bytearray(8)
  def setb(start, length, val):
    for i in range(length):
      pos = start + i
      b[pos // 8] |= ((int(val) >> i) & 1) << (pos % 8)
  setb(0, 1, prefill)
  setb(1, 2, hba)
  setb(3, 2, warn)
  setb(8, 8, dec)
  setb(18, 2, 2)          # FCA_Status
  setb(20, 1, cmd_act)
  setb(21, 1, stop_req)
  setb(22, 3, 1)          # FCA_DrvSetStatus
  setb(31, 1, dec_cmd_act)
  setb(32, 3, 0)          # FCA_Failinfo
  setb(35, 4, alive)
  setb(39, 9, 255)        # FCA_RelativeVelocity
  setb(48, 8, 254)        # FCA_TimetoCollision
  if undefined_high:
    b[4] |= 0x80          # byte4 bit7 = 1 (10549/10549 logged frames)
  b[7] = hyundai_checksum(bytes(b[:7]))
  return bytes(b)


class TestPureLogic(unittest.TestCase):

  def test_dec_cmd_from_accel_gain(self):
    # inverse of the measured 0.67 gain; -0.5 is the onset, above it coasts
    self.assertEqual(0, fl.dec_cmd_from_accel(0.0))
    self.assertEqual(0, fl.dec_cmd_from_accel(-0.4))
    self.assertEqual(0, fl.dec_cmd_from_accel(fl.FCA11_BRAKE_ONSET_ACCEL + 1e-6))
    self.assertGreaterEqual(fl.dec_cmd_from_accel(fl.FCA11_BRAKE_ONSET_ACCEL), 1)
    # -2.0 m/s^2 -> 0.30 g raw (clamped cap)
    self.assertEqual(HYUNDAI_FCA11_LONG_MAX_DEC, fl.dec_cmd_from_accel(-2.0))
    # monotone in magnitude
    vals = [fl.dec_cmd_from_accel(-a) for a in (0.6, 0.8, 1.0, 1.5, 2.0)]
    self.assertEqual(vals, sorted(vals))

  def test_clamp_dec_cmd(self):
    # default personality is STANDARD (cap 20); the tier cap is applied, floored at 1
    self.assertEqual(1, fl.clamp_dec_cmd(0))
    self.assertEqual(1, fl.clamp_dec_cmd(-5))
    self.assertEqual(fl.PERSONALITY_MAX_DEC[fl.PERSONALITY_STANDARD], fl.clamp_dec_cmd(255))
    self.assertEqual(7, fl.clamp_dec_cmd(7))
    # explicit tiers
    self.assertEqual(fl.PERSONALITY_MAX_DEC[fl.PERSONALITY_AGGRESSIVE], fl.clamp_dec_cmd(255, fl.PERSONALITY_AGGRESSIVE))
    self.assertEqual(fl.PERSONALITY_MAX_DEC[fl.PERSONALITY_RELAXED], fl.clamp_dec_cmd(255, fl.PERSONALITY_RELAXED))

  def test_constants_mirror_c(self):
    # 0040: the speed floors are DELETED (panda HYUNDAI_FCA11_LONG_MIN_SPEED and the car layer's
    # HYUNDAI_FCA11_LONG_MIN_SPEED_KPH / FCA11_BRAKE_MIN_KPH). There is no speed gate left at either layer.
    self.assertEqual(HYUNDAI_FCA11_LONG_MAX_DEC, 30)
    for gone in ("FCA11_BRAKE_MIN_KPH", "FCA11_ALERT_MARGIN_KPH"):
      self.assertFalse(hasattr(fl, gone), gone)
    from opendbc.sunnypilot.car.hyundai import values as vals
    self.assertFalse(hasattr(vals, "HYUNDAI_FCA11_LONG_MIN_SPEED_KPH"))
    # the three owner-approved replacement alerts exist
    self.assertEqual(fl.FCA11_SUPERVISE_KPH, 15.)
    self.assertEqual(fl.FCA11_STOP_KPH, 0.5)
    self.assertEqual(fl.FCA11_NORESPONSE_FACTOR, 0.4)
    self.assertEqual(fl.FCA11_NORESPONSE_MS, 500.)
    self.assertEqual(fl.FCA11_NORESPONSE_MIN_LSB, 8)
    self.assertEqual(fl.FCA11_HOLD_LOST_KPH, 2.)


class TestMirrorFrame(unittest.TestCase):

  def _fields(self, dat):
    return {'prefill': fl._get_bits(dat, 0, 1), 'warn': fl._get_bits(dat, 3, 2), 'dec': fl._get_bits(dat, 8, 8),
            'cmd_act': fl._get_bits(dat, 20, 1), 'dec_cmd_act': fl._get_bits(dat, 31, 1), 'alive': fl._get_bits(dat, 35, 4)}

  def test_overrides_only_brake_fields_alive_crc(self):
    cam = _real_camera_frame(dec=0, alive=5, undefined_high=True)
    m = fl.Fca11MirrorFrame(dec_cmd=17, prefill=True, warn=3, cmd_act=True, dec_cmd_act=False)
    out = fl.build_fca11_frame(cam, 6, m)
    f = self._fields(out)
    self.assertEqual(1, f['prefill'])
    self.assertEqual(3, f['warn'])
    self.assertEqual(17, f['dec'])
    self.assertEqual(1, f['cmd_act'])
    self.assertEqual(0, f['dec_cmd_act'])
    self.assertEqual(6, f['alive'])
    self.assertEqual(hyundai_checksum(out[:7]), out[7])
    # bytes the override must never touch: byte 5 (RelVel low) and byte 6 (TimetoCollision) are byte-for-byte the
    # camera's; the undefined byte4 bit7 is preserved. (Bytes 0/1/2/3/4 hold the overridden fields.)
    self.assertEqual(cam[5], out[5])
    self.assertEqual(cam[6], out[6])
    self.assertEqual(cam[4] & 0x80, 0x80)          # undefined bit kept
    self.assertEqual(out[4] & 0x80, 0x80)
    self.assertEqual(cam[2] & 0x0C, out[2] & 0x0C)  # FCA_Status (bits 18-19) preserved

  def test_alive_wraps_from_15(self):
    m = fl.Fca11MirrorFrame(dec_cmd=1, prefill=True, warn=3, cmd_act=True, dec_cmd_act=False)
    out = fl.build_fca11_frame(_real_camera_frame(alive=15), 0, m)
    self.assertEqual(0, fl._get_bits(out, 35, 4))

  def test_bad_frame_length_raises(self):
    m = fl.Fca11MirrorFrame(dec_cmd=1, prefill=True, warn=3, cmd_act=True, dec_cmd_act=False)
    with self.assertRaises(ValueError):
      fl.build_fca11_frame(b"\x00\x01", 1, m)
    with self.assertRaises(ValueError):
      fl.build_fca11_frame(None, 1, m)

  def test_camera_requesting(self):
    self.assertFalse(fl.camera_requesting(_real_camera_frame()))
    self.assertTrue(fl.camera_requesting(_real_camera_frame(warn=1)))
    self.assertTrue(fl.camera_requesting(_real_camera_frame(warn=2)))
    self.assertTrue(fl.camera_requesting(_real_camera_frame(dec=1)))
    self.assertTrue(fl.camera_requesting(_real_camera_frame(prefill=1)))
    self.assertTrue(fl.camera_requesting(_real_camera_frame(cmd_act=1)))
    self.assertTrue(fl.camera_requesting(_real_camera_frame(dec_cmd_act=1)))
    self.assertTrue(fl.camera_requesting(_real_camera_frame(hba=1)))
    self.assertTrue(fl.camera_requesting(_real_camera_frame(stop_req=1)))
    self.assertFalse(fl.camera_requesting(b""))


class _CS:
  """Minimal CarState double: the only attributes Fca11LongBrake reads."""
  def __init__(self, v_ego=20.0, gear=structs.CarState.GearShifter.drive, brake=False, gas=False,
               state=0, cam=None, now=0, lead_closing=False, a_ego=0.0, standstill=False, wheel_min_kph=None):
    self.out = structs.CarState()
    self.out.vEgo = v_ego
    self.out.gearShifter = gear
    self.out.brakePressed = brake
    self.out.gasPressed = gas
    self.out.aEgo = a_ego
    self.out.standstill = standstill
    self.interceptor_state = state
    self.fca11_cam_frame = cam
    self.fca11_now_nanos = now
    self.fca11_lead_closing = lead_closing
    self.fca11_wheel_min_kph = wheel_min_kph
    self.fca11_echoes = []


def _cam(v=0.0):
  return (0, _real_camera_frame(alive=3))


class TestController(unittest.TestCase):

  def _cc(self, accel=0.0, engaged=True, long_active=True):
    cc = structs.CarControl()
    cc.enabled = engaged
    cc.longActive = long_active
    cc.actuators.accel = accel
    return cc

  def _brake(self, cp_sp):
    return fl.Fca11LongBrake(structs.CarParams(), cp_sp)

  def _cp_sp(self, fca11=True, pedal=True):
    cp_sp = structs.CarParamsSP()
    cp_sp.fca11Brake = fca11
    cp_sp.enableGasInterceptor = pedal
    return cp_sp

  def test_disabled_is_inert(self):
    for fca11, pedal in ((False, True), (True, False), (False, False)):
      b = self._brake(self._cp_sp(fca11, pedal))
      self.assertEqual([], b.update(self._cc(-2.0), _CS(cam=_cam()), 0))
      self.assertFalse(b.braking)

  def test_brakes_on_request_and_hard_zero_intent(self):
    b = self._brake(self._cp_sp())
    # G5: a hard ask (<= FCA11_ONSET_BYPASS_ACCEL = -1.2) bypasses the onset debounce and brakes on
    # the very first frame (no added latency to a genuine hard demand).
    b.update(self._cc(-2.0), _CS(cam=_cam()), 0)
    self.assertTrue(b.braking)
    # G5: inside the min-on window an ask that rises above the release threshold KEEPS braking
    # (anti-flap) and the emitted command is floored at the onset value - never a 0 (a 0 mid-episode
    # would be an immediate release, defeating the hysteresis).
    sends = b.update(self._cc(-0.2), _CS(cam=_cam()), 0)
    self.assertTrue(b.braking)
    self.assertNotEqual([], sends)
    self.assertGreaterEqual(fl._get_bits(sends[0].dat, 8, 8), 1)
    # after the min-on window elapses, an ask still above -0.30 releases.
    for f in range(2, fl.FCA11_MIN_ON_FRAMES + 6):
      b.update(self._cc(-0.2), _CS(cam=_cam()), f)
    self.assertFalse(b.braking)
    # a WEAK fresh ask (above the release band, below the hard-ask bypass): brakes only after the
    # full 0.2 s onset debounce.
    b2 = self._brake(self._cp_sp())
    for f in range(fl.FCA11_ONSET_HOLD_FRAMES - 1):
      b2.update(self._cc(-0.7), _CS(cam=_cam()), f)
      self.assertFalse(b2.braking, f)
    b2.update(self._cc(-0.7), _CS(cam=_cam()), fl.FCA11_ONSET_HOLD_FRAMES - 1)
    self.assertTrue(b2.braking)

  def test_onset_debounce_kills_single_frame_ask(self):
    # a one-frame ask (above the hard-ask bypass) must not open an episode on its own...
    b = self._brake(self._cp_sp())
    b.update(self._cc(-0.7), _CS(cam=_cam()), 0)
    self.assertFalse(b.braking)
    # ...and a single frame does not survive, so the counter resets on the next coast frame.
    b.update(self._cc(-0.2), _CS(cam=_cam()), 1)
    self.assertFalse(b.braking)
    self.assertEqual(0, b._ask_frames)

  def test_min_on_never_overrides_safety(self):
    # G5 invariant: min-on / onset state can never keep (or open) braking when a safety veto fires.
    # (a) driver brake -> the cut latch sets blocked_until_resume -> release on the SAME frame.
    b = self._brake(self._cp_sp())
    b.update(self._cc(-2.0), _CS(cam=_cam()), 0)
    self.assertTrue(b.braking)
    sends = b.update(self._cc(-2.0), _CS(cam=_cam(), brake=True), 1)
    self.assertTrue(b.blocked_until_resume)
    self.assertFalse(b.braking)
    self.assertEqual([], sends)
    # (b) 0040: the speed floor is GONE - dropping to a crawl no longer releases; the brake holds DOWN THROUGH ZERO.
    b2 = self._brake(self._cp_sp())
    b2.update(self._cc(-2.0), _CS(cam=_cam()), 0)
    self.assertTrue(b2.braking)
    b2.update(self._cc(-2.0), _CS(v_ego=fl.FCA11_STOP_KPH / 3.6, cam=_cam()), 1)
    self.assertTrue(b2.braking, "0040: a crawl/standstill must KEEP braking (no floor)")
    # (c) a stale camera frame releases immediately, even inside the min-on window.
    b3 = self._brake(self._cp_sp())
    b3.update(self._cc(-2.0), _CS(cam=_cam()), 0)
    self.assertTrue(b3.braking)
    b3.update(self._cc(-2.0), _CS(cam=(0, _real_camera_frame()), now=int(0.2 * 1e9)), 1)
    self.assertFalse(b3.braking)

  def test_every_immediate_release_condition_still_fires_mid_episode(self):
    """Full safety-path audit: each veto must release on the SAME frame, inside the 0.6 s min-on window
    (the anti-flap state must never hold a brake against a safety condition)."""
    def engaged_ctrl():
      b = self._brake(self._cp_sp())
      b.update(self._cc(-2.0), _CS(cam=_cam()), 0)     # hard ask opens immediately
      assert b.braking
      return b

    # gas pressed -> per-frame release (0038: no latch)
    b = engaged_ctrl()
    self.assertFalse(b.update(self._cc(-2.0), _CS(cam=_cam(), gas=True), 1))
    self.assertFalse(b.blocked_until_resume, "0038: gas releases per-frame without latching")
    # gear not D -> latch + release
    b = engaged_ctrl()
    self.assertFalse(b.update(self._cc(-2.0), _CS(cam=_cam(), gear=structs.CarState.GearShifter.sport), 1))
    self.assertTrue(b.blocked_until_resume)
    # pedal fault -> release (no latch; the window just closes)
    for st in (1, 2, 3, 6):
      b = engaged_ctrl()
      self.assertFalse(b.update(self._cc(-2.0), _CS(cam=_cam(), state=st), 1), st)
    # camera owns -> latch + release
    b = engaged_ctrl()
    self.assertFalse(b.update(self._cc(-2.0), _CS(cam=(0, _real_camera_frame(warn=2))), 1))
    self.assertTrue(b.blocked_until_resume)
    # no camera frame -> release
    b = engaged_ctrl()
    self.assertFalse(b.update(self._cc(-2.0), _CS(cam=None), 1))
    # disengaged / longActive off -> release
    b = engaged_ctrl()
    self.assertFalse(b.update(self._cc(-2.0, long_active=False), _CS(cam=_cam()), 1))
    # toggle off mid-episode -> release (inert)
    b = fl.Fca11LongBrake(structs.CarParams(), self._cp_sp(fca11=False))
    self.assertFalse(b.braking)

  def test_hysteresis_band_never_commands_zero_while_engaged(self):
    # while engaged, an ask anywhere in (onset, release] (or even above onset, during min-on) must
    # command a raw CR_VSM_DecCmd >= 1, so the ESC never sees a mid-episode release.
    b = self._brake(self._cp_sp())
    b.update(self._cc(-2.0), _CS(cam=_cam()), 0)
    self.assertTrue(b.braking)
    for accel in (-0.50, -0.40, -0.30, -0.10, 0.0, 0.5):
      sends = b.update(self._cc(accel), _CS(cam=_cam()), 0)
      self.assertTrue(b.braking, accel)
      self.assertNotEqual([], sends, accel)
      self.assertGreaterEqual(fl._get_bits(sends[0].dat, 8, 8), 1, accel)

  def test_no_send_below_floor(self):
    # 0040: RENAMED BEHAVIOUR. There is no floor: a request at/near a standstill is now SERVED, not refused.
    # (The falsifiable contrast test `test_floor_removed_now_sends_through_zero` asserts the full matrix; this keeps a
    # single, direct assertion that the old "no send below the floor" path is gone.)
    b = self._brake(self._cp_sp())
    sends = b.update(self._cc(-2.0), _CS(v_ego=0.0, cam=_cam()), 0)
    self.assertEqual(1, len(sends))
    self.assertTrue(b.braking)
    self.assertGreaterEqual(fl._get_bits(sends[0].dat, 8, 8), 1)

  def test_no_send_without_fresh_camera(self):
    b = self._brake(self._cp_sp())
    # no camera frame at all
    self.assertEqual([], b.update(self._cc(-2.0), _CS(cam=None), 0))
    # stale frame (age > 100 ms)
    stale = (0, _real_camera_frame())
    cs = _CS(cam=stale, now=int(0.2 * 1e9))
    self.assertEqual([], b.update(self._cc(-2.0), cs, 0))

  def test_send_rate_every_other_frame(self):
    b = self._brake(self._cp_sp())
    s0 = b.update(self._cc(-2.0), _CS(cam=_cam()), 0)
    s1 = b.update(self._cc(-2.0), _CS(cam=_cam()), 1)
    self.assertEqual(1, len(s0))
    self.assertEqual(0, len(s1))
    self.assertEqual(0x38D, s0[0].address)
    self.assertEqual(0, s0[0].src)

  def test_rate_limit_growth(self):
    # growth is capped at RATE_STEP per camera period and finally lands on the PERSONALITY's tier cap
    for p, cap in ((fl.PERSONALITY_AGGRESSIVE, fl.PERSONALITY_MAX_DEC[fl.PERSONALITY_AGGRESSIVE]),
                   (fl.PERSONALITY_STANDARD, fl.PERSONALITY_MAX_DEC[fl.PERSONALITY_STANDARD]),
                   (fl.PERSONALITY_RELAXED, fl.PERSONALITY_MAX_DEC[fl.PERSONALITY_RELAXED])):
      b = self._brake(self._cp_sp())
      sends = []
      for f in range(0, 20, 2):
        sends += b.update(self._cc(-2.0), _CS(cam=_cam()), f, p)
      decs = [fl._get_bits(s.dat, 8, 8) for s in sends]
      self.assertEqual(decs, sorted(decs))
      for a, c in zip(decs, decs[1:]):
        self.assertLessEqual(c - a, fl.HYUNDAI_FCA11_LONG_RATE_STEP)
      self.assertEqual(cap, decs[-1], p)

  def test_blocked_until_resume_latch_on_cut(self):
    b = self._brake(self._cp_sp())
    b.update(self._cc(-2.0), _CS(cam=_cam()), 0)
    # driver brake -> latch; no more braking even after the pedal/window clear
    b.update(self._cc(-2.0), _CS(cam=_cam(), brake=True), 2)
    self.assertTrue(b.blocked_until_resume)
    b.update(self._cc(-2.0), _CS(cam=_cam()), 4)
    self.assertFalse(b.braking)
    # disengage -> re-engage clears it
    b.update(self._cc(-2.0, engaged=False), _CS(cam=_cam()), 6)
    b.update(self._cc(-2.0, engaged=True), _CS(cam=_cam()), 8)
    self.assertFalse(b.blocked_until_resume)
    # 0035: the cut ENDED the episode with its passive frame (frame 2); there is NO cooldown, so the very
    # next send slot after the resume brakes again.
    self.assertNotEqual([], b.update(self._cc(-2.0), _CS(cam=_cam()), 10))
    self.assertTrue(b.braking)

  def test_passive_release_frame_on_every_episode_end(self):
    """G7 (N1): every braking episode must end with exactly ONE passive 0x38D (the panda needs it to
    close its FCA11-long episode; camera frames do not pass the TX hook)."""
    b = self._brake(self._cp_sp())
    # a hard ask opens an episode immediately (the -1.2 bypass)
    for f in range(0, 4):
      b.update(self._cc(-2.0), _CS(cam=_cam()), f)
    self.assertTrue(b.braking)
    # release: the ask rises above -0.30; once the 0.6 s min-on has elapsed the episode ends on a
    # frame that must carry the passive mirror frame (all brake fields 0, Warn 0).
    released = None
    for f in range(4, 400):
      s = b.update(self._cc(-0.2), _CS(cam=_cam()), f)
      if not b.braking:
        released = (f, s); break
    self.assertIsNotNone(released, "episode never released")
    f, s = released
    if not s:  # the passive frame is on the 50 Hz slot; the release frame may have been an odd one
      s = b.update(self._cc(-0.2), _CS(cam=_cam()), f + 1)
    passive = [x for x in s if int(x.address) == fl.FCA11_ADDR and fl._get_bits(x.dat, 8, 8) == 0]
    self.assertEqual(1, len(passive), "exactly one passive frame must close the episode")
    p = passive[0].dat
    self.assertEqual(0, fl._get_bits(p, 0, 1), "Prefill")     # the camera's idle shape
    self.assertEqual(0, fl._get_bits(p, 3, 2), "Warn")
    self.assertEqual(0, fl._get_bits(p, 20, 1), "CmdAct")
    self.assertEqual(0, fl._get_bits(p, 31, 1), "DecCmdAct")
    self.assertEqual(hyundai_checksum(p[:7]), p[7])
    # and while engaged the stream is ONLY actuating frames (never a stray passive)
    b2 = self._brake(self._cp_sp())
    for f in range(0, 6):
      for x in b2.update(self._cc(-2.0), _CS(cam=_cam()), f):
        self.assertGreaterEqual(fl._get_bits(x.dat, 8, 8), 1)

  # *** 0038: EXPLICIT ZERO on override (the load-bearing escape) ***

  def test_override_sends_an_explicit_zero_frame_not_omission(self):
    """0038 (criterion 4, load-bearing): when braking is suppressed by the driver's gas, the transmitted frame must
    carry EXPLICIT ZERO brake fields (DecCmd 0, Prefill 0, Warn 0, CmdAct 0, DecCmdAct 0) - never a stale value and
    never merely STOPPED (omission). The escape from a hold is not 'driver gas' by itself: it is OUR explicit zero
    reaching the ESC. The mirror is always sourced from a fresh camera frame and the zero is written into it, so the
    ESC sees the camera's idle shape with our fields cleared."""
    b = self._brake(self._cp_sp())
    b.update(self._cc(-2.0), _CS(cam=_cam()), 0)               # brake episode opens at a high level
    self.assertTrue(b.braking)
    t = 2
    for f in range(2, 40):                                     # keep the ask up; force a 50 Hz slot boundary
      out = b.update(self._cc(-2.0), _CS(gas=True, cam=_cam()), f)   # gas: override this frame
      if out:
        self.assertEqual(1, len(out), f)
        dat = out[0].dat
        self.assertEqual(fl.FCA11_ADDR, out[0].address)
        self.assertEqual(0, fl._get_bits(dat, 8, 8), "CR_VSM_DecCmd must be an EXPLICIT 0")
        self.assertEqual(0, fl._get_bits(dat, 0, 1), "CF_VSM_Prefill 0")
        self.assertEqual(0, fl._get_bits(dat, 3, 2), "CF_VSM_Warn 0")
        self.assertEqual(0, fl._get_bits(dat, 20, 1), "FCA_CmdAct 0")
        self.assertEqual(0, fl._get_bits(dat, 31, 1), "CF_VSM_DecCmdAct 0")
        self.assertEqual(hyundai_checksum(dat[:7]), dat[7], "fresh CRC over the zeroed fields")
        t = f
      break
    else:
      self.fail("no passive zero frame was emitted on the gas override")

  def test_holdoff_and_panda_constants_match(self):
    """0038: the car layer's frame-based gas hold-off must equal the panda's microsecond hold-off (the two layers
    must agree on the same 0.5 s: 100 Hz controller frames)."""
    from opendbc.sunnypilot.car.hyundai.values import HYUNDAI_FCA11_LONG_GAS_HOLDOFF_US
    self.assertEqual(HYUNDAI_FCA11_LONG_GAS_HOLDOFF_US, fl.GAS_HOLDOFF_FRAMES * 10000)

  # *** 0035 (F7): no duration budget, no cooldown in the car layer ***

  def _withheld(self, b, frames, accel=-2.0, start=0, cs=None):
    """Drive a sustained ask for `frames` controller frames; return (withheld, sent) where `withheld` counts the
    frames on which the class was NOT braking, or (on a 50 Hz send slot) did not emit an ACTUATING 0x38D, while
    every gate held - i.e. the frames the car layer withheld by itself."""
    withheld, sent = 0, 0
    for f in range(start, start + frames):
      out = b.update(self._cc(accel), cs or _CS(cam=_cam()), f)
      act = [x for x in out if fl._get_bits(x.dat, 8, 8) > 0]
      sent += len(act)
      if not b.braking or (f % fl.FCA11_SEND_EVERY == 0 and not act):
        withheld += 1
    return withheld, sent

  def test_f7_continuous_braking_is_never_withheld(self):
    """F7: a sustained hard ask brakes for 30 s (3000 frames, 12x the old 2.5 s budget) with ZERO withheld frames
    and an actuating frame on every 50 Hz slot. RED on 0034: the budget mirror stopped commanding at frame 251
    and held braking off for the following 3 s."""
    b = self._brake(self._cp_sp())
    withheld, sent = self._withheld(b, 3000)
    self.assertEqual(0, withheld)
    self.assertEqual(1500, sent)

  def test_f7_multi_episode_profiles_are_never_withheld(self):
    """F7: back-to-back episodes, each longer than the old budget, separated by gaps all SHORTER than the old 3 s
    cooldown: every frame of every episode brakes (0 withheld), and every episode ends with exactly one passive
    close frame. RED on 0034 (the budget edge + cooldown)."""
    b = self._brake(self._cp_sp())
    f = 0
    total_withheld = 0
    for gap in (4, 50, 150, 298):
      withheld, _ = self._withheld(b, 400, start=f)
      total_withheld += withheld
      f += 400
      # release: ask rises above -0.30 -> the episode ends on this frame (min-on long elapsed)
      got = []
      for g in range(f, f + gap):
        got += b.update(self._cc(0.0), _CS(cam=_cam()), g)
        self.assertFalse(b.braking, g)
      self.assertEqual(1, len([x for x in got if fl._get_bits(x.dat, 8, 8) == 0]), f"close frame, gap {gap}")
      f += gap
    withheld, _ = self._withheld(b, 400, start=f)
    self.assertEqual(0, total_withheld + withheld)

  def test_f7_close_frame_ordering_holds_a_rebrake_at_most_one_slot(self):
    """F7: the ONE remaining car-layer hold-off is the close-frame ordering. Release on an ODD frame (no send slot),
    re-ask on the very next frame: the owed passive frame goes out first on the even slot and the re-brake starts on
    the next frame - at most one 20 ms send slot, never a clock."""
    b = self._brake(self._cp_sp())
    self._withheld(b, 101)                                  # brake through frame 100
    self.assertEqual([], b.update(self._cc(0.0), _CS(cam=_cam()), 101))   # odd release: nothing sent yet
    self.assertFalse(b.braking)
    out = b.update(self._cc(-2.0), _CS(cam=_cam()), 102)   # re-ask on the next (even) slot
    self.assertEqual([0], [fl._get_bits(x.dat, 8, 8) for x in out], "the close frame goes out first")
    withheld, _ = self._withheld(b, 200, start=103)
    self.assertEqual(0, withheld, "and the re-brake is never withheld after it")

  def test_f7_no_budget_or_cooldown_reference_remains(self):
    """F7: the mirror is gone, not just bypassed: no constant, no state, no published mirror."""
    for name in ("FCA11_BUDGET_FRAMES", "FCA11_COOLDOWN_FRAMES", "FCA11_COOLDOWN_MARGIN_FRAMES"):
      self.assertFalse(hasattr(fl, name), name)
    b = self._brake(self._cp_sp())
    for attr in ("_in_budget", "_budget_start", "_budget_end"):
      self.assertFalse(hasattr(b, attr), attr)
    for mod in (fl, cm):
      with open(mod.__file__) as f:
        src = f.read()
      for token in ("FCA11_BUDGET_FRAMES", "FCA11_COOLDOWN_FRAMES", "CAL_BUDGET_FRAMES", "CAL_COOLDOWN_FRAMES",
                    "_in_budget", "_budget_end", "set_budget_mirror", "budget_can_arm", "_fca11_budget_predicate"):
        self.assertNotIn(token, src, (mod.__name__, token))

  def test_off_path_emits_no_passive_frame(self):
    """G7: the OFF path (fca11Brake False -> enabled False) is inert: no mirror frame of any kind."""
    b = self._brake(self._cp_sp(fca11=False))
    for f in range(0, 10):
      self.assertEqual([], b.update(self._cc(-2.0), _CS(cam=_cam()), f), f)
      self.assertFalse(b.braking, f)

  def test_camera_request_latches(self):
    b = self._brake(self._cp_sp())
    cam_req = (0, _real_camera_frame(warn=2))
    b.update(self._cc(-2.0), _CS(cam=cam_req), 0)
    self.assertTrue(b.blocked_until_resume)

  def test_gear_gate(self):
    b = self._brake(self._cp_sp())
    sends = b.update(self._cc(-2.0), _CS(gear=structs.CarState.GearShifter.sport, cam=_cam()), 0)
    self.assertEqual([], sends)
    self.assertTrue(b.blocked_until_resume)

  def test_pedal_fault_blocks(self):
    for st in (1, 2, 3, 6):
      b = self._brake(self._cp_sp())
      self.assertEqual([], b.update(self._cc(-2.0), _CS(state=st, cam=_cam()), 0), st)
    for st in (0, 4, 5):
      b = self._brake(self._cp_sp())
      self.assertNotEqual([], b.update(self._cc(-2.0), _CS(state=st, cam=_cam()), 0), st)

  def test_alert_gate(self):
    # 0040: the old below-12 hand-over alert is REPLACED by "supervise stop": steady while we are braking below
    # FCA11_SUPERVISE_KPH (15 km/h). No lead gate any more (the floor it warned about is gone).
    b = self._brake(self._cp_sp())
    b.update(self._cc(-2.0), _CS(v_ego=13.0 / 3.6, cam=_cam()), 0)   # opens an episode at 13 km/h
    self.assertTrue(b.braking)
    self.assertTrue(b.supervise_stop)
    self.assertTrue(b.low_speed_alert)                              # retained alias
    b.update(self._cc(-2.0), _CS(v_ego=20.0 / 3.6, cam=_cam()), 1)
    self.assertTrue(b.braking)
    self.assertFalse(b.supervise_stop)
    b.update(self._cc(-2.0), _CS(v_ego=13.0 / 3.6, cam=_cam()), 2)
    self.assertTrue(b.supervise_stop)
    # not braking -> no supervise alert even at a crawl (the alert is conditioned on a commanded brake)
    b2 = self._brake(self._cp_sp())
    b2.update(self._cc(0.5), _CS(v_ego=5.0 / 3.6, cam=_cam()), 0)
    self.assertFalse(b2.braking)
    self.assertFalse(b2.supervise_stop)

  # *** 0040: stop-to-zero, no hold cap, the three alerts, and the per-episode logger ***

  def test_commands_through_zero_and_holds_with_no_cap(self):
    """0040 REQUIREMENT: below ~0.5 km/h the car layer keeps commanding the hold; it never drops to a passive frame
    merely because the speed is low, and there is NO self-imposed hold cap. At a full stop the command keeps flowing
    and stop_complete (the hand-off alert) is up."""
    b = self._brake(self._cp_sp())
    s = b.update(self._cc(-2.0), _CS(v_ego=0.0, cam=_cam(), standstill=True), 0)
    self.assertEqual(1, len(s), "a standstill request must still emit an actuating frame")
    self.assertTrue(b.braking)
    self.assertTrue(b.stop_complete)
    self.assertGreaterEqual(fl._get_bits(s[0].dat, 8, 8), 1)
    # 30 s of a standstill hold: the car layer withholds NOTHING (no hold cap)
    withheld, sent = self._withheld(b, 3000, accel=-2.0,
                                    cs=_CS(v_ego=0.0, cam=_cam(), standstill=True, a_ego=-2.0))
    self.assertEqual(0, withheld, "no self-imposed hold cap: not one frame withheld in 30 s at a stop")
    self.assertGreater(sent, 1000)

  def test_stop_complete_only_at_a_stop_and_only_while_holding(self):
    b = self._brake(self._cp_sp())
    b.update(self._cc(-2.0), _CS(v_ego=10.0 / 3.6, cam=_cam()), 0)   # moving
    self.assertFalse(b.stop_complete)
    b.update(self._cc(-2.0), _CS(v_ego=0.2 / 3.6, cam=_cam()), 1)    # at a stop (<= 0.5 km/h)
    self.assertTrue(b.braking)
    self.assertTrue(b.stop_complete)
    # standstill flag alone also counts (all wheels <= 0.375 km/h)
    b2 = self._brake(self._cp_sp())
    b2.update(self._cc(-2.0), _CS(v_ego=0.8 / 3.6, cam=_cam(), standstill=True), 0)
    self.assertTrue(b2.stop_complete)

  def test_brake_now_on_esc_non_response(self):
    """0040: RED persistent BRAKE NOW when the MEASURED decel is < 0.4x expected for 500 ms while commanding >= 8 LSB;
    clears ONLY on driver brake; a genuinely delivered decel never trips it. 0041: no_response is additionally gated
    on MOVING > 5 km/h and a command SUSTAINED for the ESC lag (0.62 s), so the test pre-seeds BOTH the measurement
    timer and the command-stable gate."""
    import time as _t
    # (a) a real delivered decel does NOT trip it, even after > 500 ms
    b = self._brake(self._cp_sp())
    b.update(self._cc(-2.0), _CS(v_ego=20.0 / 3.6, cam=_cam(), a_ego=-1.0), 0)
    b.update(self._cc(-2.0), _CS(v_ego=20.0 / 3.6, cam=_cam(), a_ego=-1.0), 2)   # ramps >= 8 LSB
    self.assertGreaterEqual(b._dec_last_sent, fl.FCA11_NORESPONSE_MIN_LSB)
    b._noresp_since = _t.monotonic() - 1.0
    b._noresp_cmd_since = _t.monotonic() - 1.0
    b.update(self._cc(-2.0), _CS(v_ego=20.0 / 3.6, cam=_cam(), a_ego=-5.0), 4)   # measured decel >> expected
    self.assertFalse(b.brake_now, "responding ESC: no alert")
    # (b) non-response for > 500 ms while commanding >= 8 LSB -> RED
    b2 = self._brake(self._cp_sp())
    b2.update(self._cc(-2.0), _CS(v_ego=20.0 / 3.6, cam=_cam(), a_ego=0.0), 0)
    b2.update(self._cc(-2.0), _CS(v_ego=20.0 / 3.6, cam=_cam(), a_ego=0.0), 2)
    self.assertTrue(b2.braking)
    self.assertFalse(b2.brake_now, "under 500 ms of non-response: not yet")
    b2._noresp_since = _t.monotonic() - 1.0
    b2._noresp_cmd_since = _t.monotonic() - 1.0
    b2.update(self._cc(-2.0), _CS(v_ego=20.0 / 3.6, cam=_cam(), a_ego=0.0), 4)
    self.assertTrue(b2.brake_now)
    self.assertEqual("no_response", b2.brake_now_reason)
    # persistent: a good frame alone does NOT clear it (within the 2 s self-heal window)
    b2.update(self._cc(-2.0), _CS(v_ego=20.0 / 3.6, cam=_cam(), a_ego=-5.0), 6)
    self.assertTrue(b2.brake_now, "persistent: a good frame alone does not clear it")
    # clears on driver brake
    b2.update(self._cc(-2.0), _CS(v_ego=20.0 / 3.6, cam=_cam(), brake=True), 8)
    self.assertFalse(b2.brake_now)

  def test_brake_now_on_hold_lost(self):
    """0040: RED persistent BRAKE NOW when the car moves (v > 2 km/h) with no gas while we are holding a stop."""
    b = self._brake(self._cp_sp())
    b.update(self._cc(-2.0), _CS(v_ego=0.0, cam=_cam(), standstill=True, a_ego=-2.0), 0)   # establish the hold
    self.assertTrue(b.braking)
    self.assertFalse(b.brake_now)
    b.update(self._cc(-2.0), _CS(v_ego=3.0 / 3.6, cam=_cam(), a_ego=-2.0), 2)               # crept to 3 km/h, no gas
    self.assertTrue(b.brake_now)
    self.assertEqual("hold_lost", b.brake_now_reason)
    b.update(self._cc(-2.0), _CS(v_ego=3.0 / 3.6, cam=_cam(), brake=True), 4)              # clears on driver brake
    self.assertFalse(b.brake_now)

  def test_episode_logger_writes_one_compact_record(self):
    """0040: after a drive the log ALONE must answer 'did the ESC accept sub-12 km/h commands, did it hold, for how
    long'. One compact JSON line per episode; the record carries the commanded DecCmd min/max/last, the slowest-wheel
    speeds, reached_stop + hold_frames, the delivered aEgo, and the refusal histogram."""
    import json
    import tempfile
    with tempfile.TemporaryDirectory() as d:
      b = self._brake(self._cp_sp())
      b.log = fl.fca11_log.EpisodeLogger(d)
      b.update(self._cc(-2.0), _CS(v_ego=20.0 / 3.6, cam=_cam(), a_ego=-1.0, wheel_min_kph=20.0), 0)
      b.update(self._cc(-2.0), _CS(v_ego=0.0, cam=_cam(), a_ego=-2.0, standstill=True, wheel_min_kph=0.0), 2)
      b.update(self._cc(-2.0), _CS(v_ego=0.0, cam=_cam(), brake=True, standstill=True), 4)   # ends the episode
      path = os.path.join(d, "episodes.jsonl")
      self.assertTrue(os.path.exists(path), "the episode log file must exist")
      lines = open(path).read().strip().splitlines()
      self.assertEqual(1, len(lines), "exactly one line per episode")
      rec = json.loads(lines[0])
      self.assertEqual("episode", rec["event"])
      self.assertEqual(2, rec["frames"])
      self.assertGreaterEqual(rec["dec_cmd_max"], 1)
      self.assertLessEqual(rec["dec_cmd_max"], fl.HYUNDAI_FCA11_LONG_MAX_DEC)
      self.assertEqual(20.0, rec["wheel_kph_start"])
      self.assertEqual(0.0, rec["wheel_kph_min"])
      self.assertTrue(rec["reached_stop"])
      self.assertGreaterEqual(rec["hold_frames"], 1)
      self.assertIn("refused", rec)
      self.assertIn("refused_panda", rec)
      self.assertEqual(0.5, rec["stop_kph"])
      # the log is inert when disabled (an inert EpisodeLogger writes nothing)
      b2 = self._brake(self._cp_sp())
      b2.log = fl.fca11_log.EpisodeLogger("0")
      self.assertFalse(b2.log.enabled)
      b2.update(self._cc(-2.0), _CS(v_ego=0.0, cam=_cam()), 0)
      b2.update(self._cc(0.5), _CS(v_ego=0.0, cam=_cam(), brake=True), 2)
      self.assertEqual([], os.listdir(d) if False else [x for x in os.listdir(d) if x != "episodes.jsonl"])

  def test_logger_counts_panda_refusals(self):
    """0040: the firmware's OWN refusal (pandad echo src 192 of an actuating 0x38D) is the ground truth on whether the
    ESC was ever asked; the episode record counts them."""
    import json
    import tempfile
    with tempfile.TemporaryDirectory() as d:
      b = self._brake(self._cp_sp())
      b.log = fl.fca11_log.EpisodeLogger(d)
      b.update(self._cc(-2.0), _CS(v_ego=20.0 / 3.6, cam=_cam()), 0)
      act = bytes([0] * 7 + [0])
      # two rejected actuating echoes this tick
      b.observe_echo([(192, act[:1] + bytes([9]) + act[2:]), (192, act[:1] + bytes([9]) + act[2:])])
      b.update(self._cc(0.5), _CS(v_ego=20.0 / 3.6, cam=_cam(), brake=True), 2)   # release -> flush
      rec = json.loads(open(os.path.join(d, "episodes.jsonl")).read().strip().splitlines()[0])
      self.assertEqual(2, rec["refused_panda"])


class TestG8FrameContract(unittest.TestCase):
  """G8 (R7-A/C/D): the car->panda frame contract, tested at the frame level against the panda's rules.

  R7-A: the rate reference advances only on frames actually SENT, so every sent frame grows <= +RATE_STEP
        over the previous sent frame (what the panda's hyundai.h:940-941 rate limit checks).
  R7-C (0035): the passive close frame goes out on the first send slot after an odd release, and a re-brake right
        after it is accepted at once (there is no cooldown any more).
  R7-D: the longActive-off exit owes the passive frame too (the panda's act_active survives a disengage).
  """

  def _brake(self, cp_sp=None):
    if cp_sp is None:
      cp_sp = structs.CarParamsSP()
      cp_sp.fca11Brake = True
      cp_sp.enableGasInterceptor = True
    return fl.Fca11LongBrake(structs.CarParams(), cp_sp)

  def _cc(self, accel=0.0, engaged=True, long_active=True):
    cc = structs.CarControl()
    cc.enabled = engaged
    cc.longActive = long_active
    cc.actuators.accel = accel
    return cc

  def test_r7a_rate_reference_advances_only_on_sent_frames(self):
    """R7-A: a sustained 100 Hz hard ask must emit a ramp whose SENT-frame steps are <= RATE_STEP.

    Before G8 the reference advanced on the odd (unsent) frames too, so the sent ramp stepped +2*RATE_STEP
    and the panda (which allows +RATE_STEP over the last ACCEPTED frame) refused every frame after the first.
    """
    b = self._brake()
    decs = []
    for f in range(0, 40):
      for s in b.update(self._cc(-2.0), _CS(cam=_cam()), f, fl.PERSONALITY_AGGRESSIVE):
        decs.append(fl._get_bits(s.dat, 8, 8))
    self.assertGreaterEqual(len(decs), 8, "expected a ramp of sent frames")
    # every consecutive SENT frame grows at most RATE_STEP (+4): the panda's per-accepted-frame gate
    for a, c in zip(decs, decs[1:]):
      self.assertLessEqual(c - a, fl.HYUNDAI_FCA11_LONG_RATE_STEP, (a, c))
    # the whole ramp 0 -> cap in <= (cap / RATE_STEP) sent frames (160 ms), matching the panda comment
    self.assertEqual(fl.HYUNDAI_FCA11_LONG_RATE_STEP, decs[0], "first sent frame is capped at RATE_STEP")
    self.assertEqual(fl.PERSONALITY_MAX_DEC[fl.PERSONALITY_AGGRESSIVE], decs[-1])

  def test_r7c_passive_on_next_slot_then_immediate_rebrake(self):
    """R7-C (0035): odd release -> the passive goes out one frame later on the 50 Hz slot; the very next send slot
    may brake again (no cooldown, no margin)."""
    b = self._brake()
    for f in range(0, 61):
      b.update(self._cc(-2.0), _CS(cam=_cam()), f)
    self.assertTrue(b.braking)
    f_rel = 61  # odd: not a 50 Hz send slot
    s_rel = b.update(self._cc(-0.2), _CS(cam=_cam()), f_rel)
    self.assertFalse(b.braking, "min-on elapsed, ask above release -> releases on the odd frame")
    self.assertEqual([], s_rel, "odd release frame emits nothing (passive is on the 50 Hz slot)")
    f_pas = f_rel + 1
    s_pas = b.update(self._cc(-0.2), _CS(cam=_cam()), f_pas)
    self.assertEqual(1, len([x for x in s_pas if fl._get_bits(x.dat, 8, 8) == 0]), "passive on the next send slot")
    s_new = b.update(self._cc(-2.0), _CS(cam=_cam()), f_pas + 2)
    self.assertTrue(b.braking, "re-brake on the next send slot is commanded (0035: no cooldown)")
    self.assertEqual([fl.HYUNDAI_FCA11_LONG_RATE_STEP], [fl._get_bits(x.dat, 8, 8) for x in s_new],
                     "and restarts at the onset step, exactly what the panda's onset rule allows")

  def test_r7d_longactive_off_owes_and_emits_the_passive_frame(self):
    """R7-D: a mid-episode longActive drop must still close the episode with a passive frame."""
    b = self._brake()
    for f in range(0, 4):
      b.update(self._cc(-2.0), _CS(cam=_cam()), f)
    self.assertTrue(b.braking)
    # longActive drops mid-episode: no frame may go out, but a passive frame is OWED
    s = b.update(self._cc(-2.0, long_active=False), _CS(cam=_cam()), 4)
    self.assertEqual([], s)
    self.assertFalse(b.braking)
    self.assertTrue(b._release_pending, "the longActive-off exit owes the passive frame")
    # another longActive-off frame keeps owing (no re-arm)
    self.assertEqual([], b.update(self._cc(-2.0, long_active=False), _CS(cam=_cam()), 5))
    self.assertTrue(b._release_pending)
    # the first frame back with longActive emits the passive frame before any new actuation
    got = []
    for f in range(6, 12):
      for x in b.update(self._cc(-2.0), _CS(cam=_cam()), f):
        got.append((f, fl._get_bits(x.dat, 8, 8), fl._get_bits(x.dat, 3, 2)))
    passive = [g for g in got if g[1] == 0]
    self.assertEqual(1, len(passive), f"exactly one passive frame must close the dropped episode: {got}")
    self.assertEqual(0, passive[0][2], "Warn 0 on the passive frame")
    self.assertFalse(b._release_pending)
    # and no ACTUATING frame may leave before the passive one
    self.assertTrue(all(g[1] == 0 for g in got[:1]), "passive first, actuation after")

  def test_invariant_every_opened_episode_ends_with_one_passive_frame(self):
    """Invariant: any episode that OPENS must end with exactly one passive 0x38D on every exit path where
    the panda's episode state persists (brake / gas / gear / camera-owns / staleness / longActive-off).
    The only path that sends none is the DISABLED path, where the panda re-arms on hyundai_init - checked
    separately at the end."""
    cam_req = (0, _real_camera_frame(warn=2))

    def exits(f):
      return {
        "brake": _CS(cam=_cam(), brake=True),
        "gas": _CS(cam=_cam(), gas=True),
        "gear": _CS(cam=_cam(), gear=structs.CarState.GearShifter.sport),
        "camera_owns": _CS(cam=cam_req),
        "stale": _CS(cam=(0, _real_camera_frame()), now=int(0.2 * 1e9)),
      }

    for name, cs in exits(_cam()).items():
      for exit_f in (1, 2):  # odd and even exit frames: the passive must appear either way
        b = self._brake()
        b.update(self._cc(-2.0), _CS(cam=_cam()), 0)   # hard ask opens an episode immediately
        self.assertTrue(b.braking, name)
        got = []
        for f in range(exit_f, exit_f + 4):
          for x in b.update(self._cc(-2.0), cs, f):
            got.append((f, fl._get_bits(x.dat, 8, 8)))
        passive = [g for g in got if g[1] == 0]
        self.assertEqual(1, len(passive), f"{name}@{exit_f}: exactly one passive frame, got {got}")

    # longActive-off: same invariant, delivered on the first frame back
    for exit_f in (1, 2):
      b = self._brake()
      b.update(self._cc(-2.0), _CS(cam=_cam()), 0)
      b.update(self._cc(-2.0, long_active=False), _CS(cam=_cam()), exit_f)
      self.assertTrue(b._release_pending)
      got = []
      for f in range(exit_f + 1, exit_f + 5):
        for x in b.update(self._cc(-2.0), _CS(cam=_cam()), f):
          got.append((f, fl._get_bits(x.dat, 8, 8)))
      self.assertEqual(1, len([g for g in got if g[1] == 0]),
                       f"longActive-off@{exit_f}: exactly one passive frame, got {got}")

    # the DISABLED path is the one exception: inert, no passive frame (the panda re-arms on hyundai_init)
    off = structs.CarParamsSP()
    off.fca11Brake = False
    off.enableGasInterceptor = True
    b = self._brake(off)
    for f in range(0, 6):
      self.assertEqual([], b.update(self._cc(-2.0), _CS(cam=_cam()), f), f)
      self.assertFalse(b.braking, f)


class TestG9bDriverCutRearm(unittest.TestCase):
  """G9b (drive-14): the panda's driver-cut latch re-arms on the resume edge but clears on the FIRST CLEAN frame
  after the grant. The car layer must agree, or it claims a brake the panda will refuse (or hides one it allows).

  Python sees the *derived* signals only (brakePressed / gasPressed / gearShifter / camera), so a held pedal that
  falls below threshold appears as the SAME frame shape as a release: the clean frame is the first frame whose
  driver_cut is False. The mirror is therefore "clear on the first frame the car layer sees as clean, after the
  resume edge", which is exactly what the panda does with its live inputs.
  """

  def _brake(self, cp_sp):
    return fl.Fca11LongBrake(structs.CarParams(), cp_sp)

  def _cp_sp(self):
    cp_sp = structs.CarParamsSP()
    cp_sp.fca11Brake = True
    cp_sp.enableGasInterceptor = True
    return cp_sp

  def _cc(self, accel=-2.0, engaged=True, long_active=True):
    cc = structs.CarControl()
    cc.enabled = engaged
    cc.longActive = long_active
    cc.actuators.accel = accel
    return cc

  def test_gas_escape_then_a_new_brake_event_is_served(self):
    """0038 (the owner's exact scenario at the car layer): engage -> brake -> driver presses gas -> releases ->
    keeps driving -> a NEW brake request seconds later must be SENT, with NO disengage/re-engage (no new resume
    edge). Before 0038 the gas press latched _driver_cut and the car layer never sent another actuating frame for
    the rest of the engagement, so the next stop was unbraked. FALSIFIABLE: on the pre-0038 tree the final
    assertNotEqual([]) fails."""
    b = self._brake(self._cp_sp())
    b.update(self._cc(-2.0), _CS(cam=_cam()), 0)               # brake episode opens (hard ask)
    self.assertTrue(b.braking)
    b.update(self._cc(-2.0), _CS(gas=True, cam=_cam()), 2)     # driver escapes with the gas: per-frame release
    self.assertFalse(b.braking)
    self.assertFalse(b.blocked_until_resume, "gas never latches on the car layer either")
    # the pedal releases and the driver cruises for a while (well past the 0.5 s hold-off = 50 frames)
    b.update(self._cc(-0.2), _CS(gas=False, cam=_cam()), 3)    # release edge stamped here
    for f in range(4, 200):
      b.update(self._cc(-0.2), _CS(gas=False, cam=_cam()), f)
    self.assertFalse(b.blocked_until_resume)
    # a NEW brake request arrives, still the SAME engagement, no new resume edge
    sends = b.update(self._cc(-2.0), _CS(gas=False, cam=_cam()), 200)
    self.assertNotEqual([], sends, "a new brake event after a gas escape must be sent")
    self.assertTrue(b.braking)

  def test_gas_holdoff_blocks_for_50_frames_then_allows(self):
    """0038: the only post-release delay is the ~0.5 s (50-frame) hold-off, and it clears on ELAPSED TIME alone
    (no resume edge). Inside it fca11_ok is False; at the edge it becomes True and a new request then sends."""
    b = self._brake(self._cp_sp())
    b.update(self._cc(-2.0), _CS(gas=True, cam=_cam()), 0)     # gas (no prior press): release edge on frame 1
    b.update(self._cc(-2.0), _CS(gas=False, cam=_cam()), 1)
    self.assertEqual(1, b._gas_release_frame)
    for f in range(2, 1 + fl.GAS_HOLDOFF_FRAMES):              # frames 2 .. 50: still inside the hold-off
      self.assertFalse(b._gas_holdoff_ok(f), f)
      self.assertEqual([], b.update(self._cc(-2.0), _CS(gas=False, cam=_cam()), f), f)
    self.assertTrue(b._gas_holdoff_ok(1 + fl.GAS_HOLDOFF_FRAMES))
    # a new request now sends (allow for the 50 Hz slot + onset debounce)
    sent = []
    for f in range(2 + fl.GAS_HOLDOFF_FRAMES, 2 + fl.GAS_HOLDOFF_FRAMES + 60):
      sent += b.update(self._cc(-2.0), _CS(gas=False, cam=_cam()), f)
    self.assertNotEqual([], sent, "past the hold-off a new request is served")

  def test_engage_with_gas_held_is_refused_per_frame(self):
    """0038 required regression: engage with the gas STILL over threshold and KEEP it held -> every actuating
    frame stays refused (live per-frame gas), but no latch is set, so releasing + the hold-off restores braking."""
    b = self._brake(self._cp_sp())
    b.update(self._cc(engaged=False), _CS(gas=True, cam=_cam()), 0)
    b.update(self._cc(engaged=True), _CS(gas=True, cam=_cam()), 2)
    self.assertFalse(b.blocked_until_resume)
    for f in range(3, 12):
      self.assertEqual([], b.update(self._cc(-2.0), _CS(gas=True, cam=_cam()), f), f)
    # released: hold-off then served
    b.update(self._cc(-2.0), _CS(gas=False, cam=_cam()), 12)
    sends = b.update(self._cc(-2.0), _CS(gas=False, cam=_cam()), 12 + fl.GAS_HOLDOFF_FRAMES)
    self.assertNotEqual([], sends)

  def test_gear_override_rearms_on_resume(self):
    """Gear out of D latched before the engage clears on the first clean frame after the resume (G9 T1 parity)."""
    b = self._brake(self._cp_sp())
    b.update(self._cc(engaged=False), _CS(gear=structs.CarState.GearShifter.sport, cam=_cam()), 0)
    self.assertTrue(b._driver_cut)
    b.update(self._cc(engaged=True), _CS(gear=structs.CarState.GearShifter.sport, cam=_cam()), 2)
    self.assertTrue(b._driver_cut)                                   # grant frame still out of D
    sends = b.update(self._cc(accel=-2.0), _CS(gear=structs.CarState.GearShifter.drive, cam=_cam()), 4)
    self.assertFalse(b.blocked_until_resume)
    self.assertNotEqual([], sends)

  def test_camera_cut_is_hard_not_rearmed(self):
    """The camera hand-back must NOT be re-armed by a clean frame (separate _hard_cut, mirroring the panda's
    hard-latched hyundai_fca11_long_cut)."""
    b = self._brake(self._cp_sp())
    cam_req = (0, _real_camera_frame(warn=2))
    b.update(self._cc(engaged=False), _CS(cam=cam_req), 0)
    b.update(self._cc(engaged=True), _CS(cam=cam_req), 2)
    b.update(self._cc(engaged=True), _CS(cam=cam_req), 4)            # still a clean *driver* frame
    self.assertTrue(b._hard_cut)
    self.assertTrue(b.blocked_until_resume)
    # a fresh clean camera frame on a later tick still does not clear the hard latch
    b.update(self._cc(engaged=True), _CS(cam=_cam()), 6)
    self.assertTrue(b.blocked_until_resume)

  def test_gas_release_serves_the_next_request_without_any_edge(self):
    """0038: a gas press while ENGAGED, controls never dropping, must NOT bar actuation afterwards. Once the pedal
    releases and the hold-off elapses (no disengage/re-engage, no resume edge), a fresh request is served."""
    b = self._brake(self._cp_sp())
    b.update(self._cc(), _CS(cam=_cam()), 0)
    self.assertTrue(b.braking)
    b.update(self._cc(), _CS(gas=True, cam=_cam()), 2)
    self.assertFalse(b.blocked_until_resume)
    # the release is stamped on frame 3; hold off for the next 50 frames, then serve a fresh request
    b.update(self._cc(-2.0), _CS(gas=False, cam=_cam()), 3)
    for f in range(4, 4 + fl.GAS_HOLDOFF_FRAMES):
      self.assertEqual([], b.update(self._cc(-2.0), _CS(gas=False, cam=_cam()), f), f)
    sent = []
    for f in range(4 + fl.GAS_HOLDOFF_FRAMES, 4 + fl.GAS_HOLDOFF_FRAMES + 60):
      sent += b.update(self._cc(-2.0), _CS(gas=False, cam=_cam()), f)
    self.assertNotEqual([], sent)

  def test_disengage_abandons_a_pending_rearm(self):
    """An armed re-arm that is never consumed by a clean engaged frame is dropped on the next disengage: a cut
    held across a disengage must not slip through on a later clean frame without a fresh edge. (0038: uses the
    GEAR latch, a real driver latch - gas no longer latches.)"""
    b = self._brake(self._cp_sp())
    b.update(self._cc(engaged=False), _CS(gear=structs.CarState.GearShifter.sport, cam=_cam()), 0)
    b.update(self._cc(engaged=True), _CS(gear=structs.CarState.GearShifter.sport, cam=_cam()), 2)   # edge armed, gear out -> not consumed
    self.assertTrue(b._rearm_pending)
    b.update(self._cc(engaged=False), _CS(gear=structs.CarState.GearShifter.sport, cam=_cam()), 4)  # disengage: abandon
    self.assertFalse(b._rearm_pending)
    self.assertTrue(b._driver_cut)
    b.update(self._cc(engaged=False), _CS(gear=structs.CarState.GearShifter.drive, cam=_cam()), 6)
    self.assertTrue(b._driver_cut)                                  # clean but disengaged -> stays latched


class TestPersonalityStrength(unittest.TestCase):
  """Personality-indexed FCA11-long decel cap: relaxed 0.12 g / standard 0.20 g / aggressive 0.30 g (today's cap, max).

  Personality is cereal log.LongitudinalPersonality (0 = aggressive, 1 = standard, 2 = relaxed). The cap is a strict
  subset of the panda firmware gate (HYUNDAI_FCA11_LONG_MAX_DEC), which is personality-agnostic -> no firmware change.
  """

  RE = fl.PERSONALITY_RELAXED
  ST = fl.PERSONALITY_STANDARD
  AG = fl.PERSONALITY_AGGRESSIVE

  def test_tier_table_is_g(self):
    # raw CR_VSM_DecCmd is 0.01 g/LSB
    self.assertEqual(0.12, fl.PERSONALITY_MAX_DEC[self.RE] / 100)
    self.assertEqual(0.20, fl.PERSONALITY_MAX_DEC[self.ST] / 100)
    self.assertEqual(0.30, fl.PERSONALITY_MAX_DEC[self.AG] / 100)

  def test_every_tier_within_firmware_gate(self):
    # invariant: the whole table must fit the panda's personality-agnostic gate (else firmware would reject frames)
    self.assertLessEqual(max(fl.PERSONALITY_MAX_DEC.values()), HYUNDAI_FCA11_LONG_MAX_DEC)
    for p in (self.RE, self.ST, self.AG):
      self.assertLessEqual(fl.personality_max_dec(p), HYUNDAI_FCA11_LONG_MAX_DEC)

  def test_ordering_and_aggressive_is_legacy(self):
    self.assertLess(fl.personality_max_dec(self.RE), fl.personality_max_dec(self.ST))
    self.assertLess(fl.personality_max_dec(self.ST), fl.personality_max_dec(self.AG))
    # aggressive == the pre-personality cap (regression anchor); standard is the middle
    self.assertEqual(HYUNDAI_FCA11_LONG_MAX_DEC, fl.personality_max_dec(self.AG))
    self.assertEqual(20, fl.personality_max_dec(self.ST))

  def test_unknown_and_junk_fall_back_to_standard(self):
    for bad in (3, 7, -1, 99, 255, None, 'bogus', b'1'):
      self.assertEqual(fl.PERSONALITY_MAX_DEC[self.ST], fl.personality_max_dec(bad), bad)
    # a capnp _DynamicEnum-like object carrying .raw is accepted
    class _E:
      raw = 0  # aggressive
    self.assertEqual(fl.PERSONALITY_MAX_DEC[self.AG], fl.personality_max_dec(_E()))

  def test_clamp_at_each_tier(self):
    for p, cap in ((self.RE, 12), (self.ST, 20), (self.AG, 30)):
      self.assertEqual(cap, fl.clamp_dec_cmd(255, p), p)   # requests above the cap clamp down
      self.assertEqual(cap, fl.clamp_dec_cmd(cap, p), p)   # at the cap stays
      self.assertEqual(7, fl.clamp_dec_cmd(7, p), p)       # below the cap untouched
      self.assertEqual(1, fl.clamp_dec_cmd(0, p), p)       # raw floor
      self.assertEqual(1, fl.clamp_dec_cmd(-5, p), p)

  def test_default_personality_is_standard(self):
    # a direct call with no personality (sim / any caller that forgets) is STANDARD, never aggressive
    self.assertEqual(fl.clamp_dec_cmd(255), fl.clamp_dec_cmd(255, self.ST))
    self.assertEqual(20, fl.clamp_dec_cmd(255))

  def test_scale_then_clamp_each_tier(self):
    # the same -2.0 m/s^2 ask maps through the gain inverse to a raw cap per tier (the strength scaling felt)
    for p, cap in ((self.RE, 12), (self.ST, 20), (self.AG, 30)):
      self.assertEqual(cap, fl.clamp_dec_cmd(fl.dec_cmd_from_accel(-2.0), p), p)
    # a -0.6 m/s^2 ask is a small raw value below every tier cap: identical for all three
    small = fl.dec_cmd_from_accel(-0.6)
    self.assertLess(small, 12)
    for p in (self.RE, self.ST, self.AG):
      self.assertEqual(small, fl.clamp_dec_cmd(small, p), p)

  # ---- controller level: the emitted CR_VSM_DecCmd lands on the tier cap ----

  def _emit(self, personality, accel=-2.0, frames=40):
    cp_sp = structs.CarParamsSP()
    cp_sp.fca11Brake = True
    cp_sp.enableGasInterceptor = True
    b = fl.Fca11LongBrake(structs.CarParams(), cp_sp)
    cc = structs.CarControl()
    cc.enabled = cc.longActive = True
    cc.actuators.accel = accel
    sends = []
    for f in range(0, frames, 2):
      sends += b.update(cc, _CS(cam=_cam()), f, personality)
    return [fl._get_bits(s.dat, 8, 8) for s in sends]

  def test_emitted_frame_cap_per_tier(self):
    for p, cap in ((self.RE, 12), (self.ST, 20), (self.AG, 30)):
      decs = self._emit(p)
      self.assertTrue(decs, p)
      self.assertEqual(cap, max(decs), p)              # a strong request lands exactly on the tier cap
      self.assertLessEqual(max(decs), HYUNDAI_FCA11_LONG_MAX_DEC, p)

  def test_strong_request_scales_by_personality(self):
    caps = {p: max(self._emit(p)) for p in (self.RE, self.ST, self.AG)}
    self.assertLess(caps[self.RE], caps[self.ST])
    self.assertLess(caps[self.ST], caps[self.AG])

  def test_off_inert_every_personality(self):
    # toggle OFF must be byte-for-byte inert for any personality value (incl. out-of-range) and the kwarg-omitted path
    for p in (self.RE, self.ST, self.AG, 255):
      cp_sp = structs.CarParamsSP()
      cp_sp.fca11Brake = False
      cp_sp.enableGasInterceptor = True
      b = fl.Fca11LongBrake(structs.CarParams(), cp_sp)
      cc = structs.CarControl()
      cc.enabled = cc.longActive = True
      cc.actuators.accel = -2.0
      self.assertEqual([], b.update(cc, _CS(cam=_cam()), 0, p), p)
      self.assertFalse(b.braking, p)
      self.assertEqual(0, b._dec_last_sent, p)         # no state mutation
      self.assertEqual([], b.update(cc, _CS(cam=_cam()), 2), p)  # kwarg omitted -> default standard, still inert


class TestInterfacePlumbing(unittest.TestCase):

  def test_toggle_off_is_inert(self):
    _, CP_ref, CP_SP_ref = _get_ci(True, None)
    for val in (None, "0", "", "bogus", b"0"):
      _, CP, CP_SP = _get_ci(True, val)
      self.assertFalse(CP_SP.fca11Brake, val)
      self.assertFalse(CP_SP.safetyParam & HyundaiSafetyFlagsSP.FCA11_LONG, val)
      # everything else identical to the pedal-only baseline
      self.assertEqual(CP_SP_ref.safetyParam, CP_SP.safetyParam, val)
      self.assertEqual(CP_SP_ref.enableGasInterceptor, CP_SP.enableGasInterceptor)
      self.assertEqual(CP.to_bytes(), CP_ref.to_bytes(), val)

  def test_toggle_on_sets_flag_and_bit(self):
    _, CP, CP_SP = _get_ci(True, "1")
    self.assertTrue(CP_SP.fca11Brake)
    self.assertTrue(CP_SP.safetyParam & HyundaiSafetyFlagsSP.FCA11_LONG)
    # pedal bits untouched
    self.assertTrue(CP_SP.safetyParam & HyundaiSafetyFlagsSP.GAS_INTERCEPTOR)

  def test_toggle_on_without_pedal_is_ignored(self):
    _, CP, CP_SP = _get_ci(None, "1")
    self.assertFalse(CP_SP.fca11Brake)
    self.assertFalse(CP_SP.safetyParam & HyundaiSafetyFlagsSP.FCA11_LONG)

  def test_planner_pid_limits(self):
    CarInterface = interfaces[CAR_UNDER_TEST]
    _, CP, CP_SP = _get_ci(True, "1")
    # 0040: the planner floor is DELETED - brake authority to -2.0 at ANY speed, down to and through zero.
    for v in (20.0, 3.0, 0.5, 0.0):   # 20 m/s, 10.8 km/h, 1.8 km/h, standstill
      lo, hi = CarInterface.get_pid_accel_limits(CP, CP_SP, v, v)
      self.assertEqual(-fl.FCA11_A_MAX_MSS2, lo, v)
    # toggle off: today's stock limits, unchanged at any speed
    _, CP2, CP_SP2 = _get_ci(True, "0")
    for v in (20.0, 3.0, 0.0):
      self.assertEqual((-3.5, 2.0), CarInterface.get_pid_accel_limits(CP2, CP_SP2, v, v))

  def test_capnp_field_present_default_off(self):
    # the CarParamsSP struct carries the toggle and defaults OFF (the capnp field fca11Brake is regenerated by CI)
    self.assertIn("fca11Brake", structs.CarParamsSP.__dataclass_fields__)
    self.assertFalse(structs.CarParamsSP().fca11Brake)


# ============================================================================================
# 0029 CAL command mode (patch 0029-hyundai-fca11-cal-command-mode)
# ============================================================================================
from opendbc.sunnypilot.car.hyundai import cal_mode as cm  # noqa: E402


def _cal_plan(reps, cond=None, **over):
  """A well-formed plan.json for a temp dir. `reps` = list of (band_kph, level_lsb[, hold_s[, level2]]).

  Condition defaults are deliberately FAST (band_stable 0.5 s, straight_min 0) so a rep becomes due
  within a few hundred frames; production defaults (5 s / 2 s) are exercised by the gate tests.
  """
  conditions = {"min_lead_m": 60.0, "turn_lat_max": 0.5, "straight_min_s": 0.0,
                "band_tol_kph": 5.0, "band_stable_s": 0.5, "recovery_s": 3.5}
  if cond:
    conditions.update(cond)
  rep_list = []
  for i, r in enumerate(reps):
    rep = {"id": f"r{i}", "phase": "steps", "band_kph": r[0], "level_lsb": r[1]}
    if len(r) > 2:
      rep["hold_s"] = r[2]
    if len(r) > 3:
      rep["level2_lsb"] = r[3]
      rep["switch_at_s"] = (r[2] / 2.0) if len(r) > 2 else 1.0
    rep_list.append(rep)
  doc = {"rev": 1, "issued_at": time.time(), "ttl_s": 900.0, "conditions": conditions, "reps": rep_list}
  doc.update(over)
  return doc


# actuation tests need a rep to be due promptly: don't wait 5 s of band stability / 2 s of straight
_FAST = {"band_stable_s": 0.5, "straight_min_s": 0.0}


def _write_plan(d, doc):
  with open(os.path.join(d, "plan.json"), "w") as f:
    json.dump(doc, f)


class _CalCS:
  """CarState double carrying the cal sequencer's condition inputs."""
  def __init__(self, v_ego=16.67, gear=structs.CarState.GearShifter.drive, brake=False, gas=False,
               state=0, cam=None, now=0, lead=None, yaw=0.0):
    self.out = structs.CarState()
    self.out.vEgo = v_ego
    self.out.gearShifter = gear
    self.out.brakePressed = brake
    self.out.gasPressed = gas
    self.out.yawRate = yaw
    self.interceptor_state = state
    self.fca11_cam_frame = cam
    self.fca11_now_nanos = now
    self.fca11_lead_closing = False
    self.fca11_lead_drel_m = lead
    self.fca11_cal_active = False
    self._fca11_braking_now = False


class TestCalModeGate(unittest.TestCase):
  """plan.json gate: absent / malformed / expired / stale -> INERT; the cal path is subordinate to the
  production toggle (self.enabled), so the file alone can NEVER actuate."""

  def _brake(self, cp_sp, d):
    b = fl.Fca11LongBrake(structs.CarParams(), cp_sp)
    b.cal_seq = cm.CalSequencer(d)
    b.cal_seq.mark_session(1)
    b.cal_seq.set_phases(("steps",))
    return b

  def _cp_sp(self, fca11=True, pedal=True):
    cp_sp = structs.CarParamsSP()
    cp_sp.fca11Brake = fca11
    cp_sp.enableGasInterceptor = pedal
    return cp_sp

  def _cc(self, accel=0.0, engaged=True, long_active=True):
    cc = structs.CarControl()
    cc.enabled = engaged
    cc.longActive = long_active
    cc.actuators.accel = accel
    return cc

  def _drive(self, b, frame, cs=None, cc=None):
    """Run the cal gate directly: a plan that is due + every condition ok must command the level."""
    b.cal_seq.tick(cc or self._cc(), cs or _CalCS(cam=_cam()), frame)

  def _warm(self, b, cs, frames=600):
    for f in range(frames):
      b.cal_seq.tick(self._cc(), cs, f)

  def test_plan_absent_is_inert(self):
    with tempfile.TemporaryDirectory() as d:
      b = self._brake(self._cp_sp(), d)
      for f in range(40):
        self._drive(b, f)
        self.assertFalse(b.cal_seq.active_actuation, f)
      # and through update(): a planner ask with no plan behaves exactly as today (planner-driven)
      b2 = self._brake(self._cp_sp(), d)
      s = b2.update(self._cc(-2.0), _CS(cam=_cam()), 0)
      self.assertTrue(b2.braking)                 # the planner ask still brakes
      self.assertFalse(b2._cal_active)            # ...but it is NOT a cal hold
      # the planner path is unchanged: its first SENT frame is the production +RATE_STEP rate limit
      self.assertEqual(fl.HYUNDAI_FCA11_LONG_RATE_STEP, fl._get_bits(s[0].dat, 8, 8))

  def test_toggle_off_dominates_even_with_a_valid_plan(self):
    """Owner directive #2: FCA11 param OFF + valid/fresh plan + perfect conditions -> zero sends.
    The cal code lives inside the `enabled` branch, so update() returns before it is reached."""
    with tempfile.TemporaryDirectory() as d:
      _write_plan(d, _cal_plan([(60, 30)]))
      b = fl.Fca11LongBrake(structs.CarParams(), self._cp_sp(fca11=False))  # enabled = False -> no cal_seq
      self.assertIsNone(b.cal_seq)
      cs = _CalCS(cam=_cam())
      for f in range(0, 1200, 2):
        self.assertEqual([], b.update(self._cc(0.0), cs, f), f)
        self.assertFalse(b.braking, f)
        self.assertFalse(b._cal_active, f)

  def test_malformed_plan_is_inert(self):
    with tempfile.TemporaryDirectory() as d:
      b = self._brake(self._cp_sp(), d)
      for bad in ('{"reps": [', '[]', '{"reps": "x"}', '{"reps": [{"band_kph": "z", "level_lsb": 5}]}',
                  'null', '{"reps": [{"level_lsb": 5}]}'):
        with open(os.path.join(d, "plan.json"), "w") as f:
          f.write(bad)
        self._warm(b, _CalCS(cam=_cam()), frames=30)
        self.assertFalse(b.cal_seq.active_actuation, bad)

  def test_ttl_expiry_is_inert(self):
    with tempfile.TemporaryDirectory() as d:
      _write_plan(d, _cal_plan([(60, 20)], issued_at=time.time() - 5000.0, ttl_s=900.0))
      b = self._brake(self._cp_sp(), d)
      self._warm(b, _CalCS(cam=_cam()))
      self.assertFalse(b.cal_seq.active_actuation)
      self.assertEqual("plan_stale_ttl", b.cal_seq._block_reason)

  def test_ttl_zero_or_negative_is_inert(self):
    with tempfile.TemporaryDirectory() as d:
      for ttl in (0, -1, 0.0):
        _write_plan(d, _cal_plan([(60, 20)], ttl_s=ttl))
        b = self._brake(self._cp_sp(), d)
        self._warm(b, _CalCS(cam=_cam()))
        self.assertFalse(b.cal_seq.active_actuation, ttl)

  def test_absolute_expiry_is_inert(self):
    with tempfile.TemporaryDirectory() as d:
      _write_plan(d, _cal_plan([(60, 20)], expires_at=time.time() - 1.0))
      b = self._brake(self._cp_sp(), d)
      self._warm(b, _CalCS(cam=_cam()))
      self.assertFalse(b.cal_seq.active_actuation)
      self.assertEqual("plan_expired", b.cal_seq._block_reason)

  def test_conditions_gate(self):
    """Each condition alone must block a rep: band, lead, straight, feet (via fca11_ok), gear."""
    with tempfile.TemporaryDirectory() as d:
      _write_plan(d, _cal_plan([(60, 20)]))
      # band wrong: cruising at 40 km/h, plan wants 60 -> band_not_ready
      b = self._brake(self._cp_sp(), d)
      self._warm(b, _CalCS(v_ego=40 / 3.6, cam=_cam()))
      self.assertFalse(b.cal_seq.active_actuation)
      self.assertEqual("band_not_ready", b.cal_seq._block_reason)
      # band right but a lead within 60 m -> blocked
      b = self._brake(self._cp_sp(), d)
      self._warm(b, _CalCS(cam=_cam(), lead=30.0))
      self.assertFalse(b.cal_seq.active_actuation)
      self.assertEqual("lead_close", b.cal_seq._block_reason)
      # band right, no lead, but a sustained turn (yaw) -> not_straight
      b = self._brake(self._cp_sp(), d)
      _write_plan(d, _cal_plan([(60, 20)], cond={"straight_min_s": 2.0}))
      self._warm(b, _CalCS(cam=_cam(), yaw=0.05, v_ego=16.67))  # a_lat = 0.83 > 0.5
      self.assertFalse(b.cal_seq.active_actuation)
      self.assertEqual("not_straight", b.cal_seq._block_reason)

  def test_conditions_ok_then_a_rep_is_commandable(self):
    with tempfile.TemporaryDirectory() as d:
      _write_plan(d, _cal_plan([(60, 20)]))
      b = self._brake(self._cp_sp(), d)
      # a rep becomes due after band_stable_s (0.5 s = 50 frames); it then holds 2.2 s
      self._warm(b, _CalCS(v_ego=60 / 3.6, cam=_cam()), frames=120)
      self.assertTrue(b.cal_seq.active_actuation)
      self.assertEqual(20, b.cal_seq.level_lsb)


class TestCalModeActuation(unittest.TestCase):
  """The command surface through Fca11LongBrake.update: the scripted level REPLACES the planner ask;
  it bypasses the onset debounce; it is clamped to the PANDA gate (not the personality cap); the entry
  is a panda-rate clean step; release goes through the production fade/passive path."""

  def _mk(self, d, fca11=True):
    cp_sp = structs.CarParamsSP()
    cp_sp.fca11Brake = fca11
    cp_sp.enableGasInterceptor = True
    b = fl.Fca11LongBrake(structs.CarParams(), cp_sp)
    b.cal_seq = cm.CalSequencer(d)
    b.cal_seq.mark_session(1)
    b.cal_seq.set_phases(("steps",))
    return b

  def _prep(self, b, d, cs, warm=600):
    _write_plan(d, _cal_plan([(60, int(cs.out.vEgo * 3.6 / 3.6 and cs.out.vEgo * 3.6 or 60), 20)]))
    for f in range(warm):
      b.cal_seq.tick(structs.CarControl(enabled=True, longActive=True), cs, f)

  def test_cal_actuates_on_an_empty_road_with_no_planner_ask(self):
    """The scripted level IS the ask: with the planner commanding 0 (a coast request, i.e. NO brake
    ask at all) the cal episode opens and commands the scripted level. This is the steered-requirement
    test: it must actuate on an empty road with no lead and no limit change."""
    with tempfile.TemporaryDirectory() as d:
      b = self._mk(d)
      _write_plan(d, _cal_plan([(60, 25)]))
      cc = structs.CarControl()
      cc.enabled = cc.longActive = True
      cc.actuators.accel = 0.0                    # the planner is NOT asking for a brake
      cs = _CalCS(v_ego=60 / 3.6, cam=_cam())
      sends = []
      cal_flagged = False
      for f in range(0, 400):
        sends += b.update(cc, cs, f)
        cal_flagged = cal_flagged or b._cal_active
      decs = [fl._get_bits(s.dat, 8, 8) for s in sends]
      self.assertTrue(decs, "cal must actuate with no planner ask")
      self.assertEqual(25, max(decs))            # the exact scripted level was reached
      self.assertTrue(cal_flagged, "the frame is flagged as a cal hold (gas neutralization)")

  def test_cal_level_reaches_panda_gate_regardless_of_personality(self):
    """cal must reach 0.30 g (30 LSB) whatever the owner's feel dial: the clamp is the PANDA gate."""
    for p, tier in ((fl.PERSONALITY_RELAXED, 12), (fl.PERSONALITY_STANDARD, 20),
                    (fl.PERSONALITY_AGGRESSIVE, 30)):
      with tempfile.TemporaryDirectory() as d:
        b = self._mk(d)
        _write_plan(d, _cal_plan([(60, 30)]))
        cc = structs.CarControl()
        cc.enabled = cc.longActive = True
        cc.actuators.accel = 0.0
        cs = _CalCS(v_ego=60 / 3.6, cam=_cam())
        decs = []
        for f in range(0, 400):
          for s in b.update(cc, cs, f, p):
            decs.append(fl._get_bits(s.dat, 8, 8))
        self.assertEqual(30, max(decs), p)       # NOT the tier cap: 30 regardless
        self.assertGreaterEqual(max(decs), tier, (p, tier))

  def test_cal_clamped_to_panda_gate(self):
    """A plan asking above the panda gate is clamped to 30 (the plan can never widen authority)."""
    with tempfile.TemporaryDirectory() as d:
      b = self._mk(d)
      _write_plan(d, _cal_plan([(60, 200)]))
      cc = structs.CarControl()
      cc.enabled = cc.longActive = True
      cc.actuators.accel = 0.0
      cs = _CalCS(v_ego=60 / 3.6, cam=_cam())
      decs = []
      for f in range(0, 400):
        for s in b.update(cc, cs, f):
          decs.append(fl._get_bits(s.dat, 8, 8))
      self.assertEqual(fl.HYUNDAI_FCA11_LONG_MAX_DEC, max(decs))

  def test_cal_entry_is_a_clean_panda_rate_step(self):
    """A scripted step enters at +RATE_STEP per SENT frame (the clean step the BITE fit needs), not
    smeared by the production LPF/slew; the SENT-frame deltas never exceed +4."""
    with tempfile.TemporaryDirectory() as d:
      b = self._mk(d)
      _write_plan(d, _cal_plan([(60, 12)]))
      cc = structs.CarControl()
      cc.enabled = cc.longActive = True
      cc.actuators.accel = 0.0
      cs = _CalCS(v_ego=60 / 3.6, cam=_cam())
      decs = []
      for f in range(0, 400):
        for s in b.update(cc, cs, f):
          decs.append(fl._get_bits(s.dat, 8, 8))
      self.assertEqual(4, decs[0], "first SENT frame is rate-limited to +4")
      for a, c in zip(decs, decs[1:]):
        self.assertLessEqual(c - a, fl.HYUNDAI_FCA11_LONG_RATE_STEP, (a, c))
      self.assertEqual(12, max(decs))
      # the level is held FLAT at 12 for the rest of the hold (a clean plateau, not a slope); the
      # trailing 0 is the passive frame that closes the episode.
      nonzero = [d for d in decs if d > 0]
      self.assertEqual([4, 8, 12], nonzero[:3], "0 -> 4 -> 8 -> 12")
      self.assertTrue(all(d == 12 for d in nonzero[3:]), "then a flat plateau at 12")

  def test_cal_release_uses_the_production_fade_and_passive_frame(self):
    """When the sequencer drops the hold, the episode ends through the PRODUCTION path: a passive
    frame, not a mid-episode 0."""
    with tempfile.TemporaryDirectory() as d:
      b = self._mk(d)
      _write_plan(d, _cal_plan([(60, 20, 0.5)]))   # a 0.5 s hold
      cc = structs.CarControl()
      cc.enabled = cc.longActive = True
      cc.actuators.accel = 0.0
      cs = _CalCS(v_ego=60 / 3.6, cam=_cam())
      got = []
      for f in range(0, 400):
        for s in b.update(cc, cs, f):
          got.append((f, fl._get_bits(s.dat, 8, 8), fl._get_bits(s.dat, 3, 2)))
      self.assertTrue(any(g[1] > 0 for g in got), "actuation happened")
      self.assertTrue(any(g[1] == 0 and g[2] == 0 for g in got), "exactly one passive frame closes it")
      # only ONE passive frame (the episode does not flicker passive/active)
      self.assertEqual(1, sum(1 for g in got if g[1] == 0))

  def test_cal_driver_cut_still_cuts_mid_hold(self):
    """Every production veto still applies on every frame: a driver brake / gas / gear-out mid-hold releases the
    cal episode on the same frame. (0038: gas releases per-frame without latching, brake/gear still latch until the
    next resume.)"""
    with tempfile.TemporaryDirectory() as d:
      for bad, latches in (({"brake": True}, True), ({"gas": True}, False),
                           ({"gear": structs.CarState.GearShifter.sport}, True)):
        b = self._mk(d)
        self.assertIsNotNone(b.cal_seq)
        _write_plan(d, _cal_plan([(60, 25)]))
        cc = structs.CarControl()
        cc.enabled = cc.longActive = True
        cc.actuators.accel = 0.0
        cs = _CalCS(v_ego=60 / 3.6, cam=_cam())
        # a rep becomes due after band_stable_s (0.5 s = 50 frames) and holds 2.2 s
        for f in range(0, 200):
          b.update(cc, cs, f)
        self.assertTrue(b.braking, bad)
        cut = _CalCS(v_ego=60 / 3.6, cam=_cam(), **bad)
        b.update(cc, cut, 200)
        self.assertFalse(b.braking, bad)
        self.assertEqual(latches, b.blocked_until_resume, bad)

  def test_cal_holds_and_moves_on_when_a_rep_completes(self):
    """The sequencer persists completion and never re-runs a finished rep (idempotent)."""
    with tempfile.TemporaryDirectory() as d:
      b = self._mk(d)
      _write_plan(d, _cal_plan([(60, 20, 0.5)]))
      cc = structs.CarControl()
      cc.enabled = cc.longActive = True
      cc.actuators.accel = 0.0
      cs = _CalCS(v_ego=60 / 3.6, cam=_cam())
      for f in range(0, 2000):
        b.update(cc, cs, f)
      self.assertTrue(b.cal_seq.is_done, "plan complete -> done")
      prog = json.load(open(os.path.join(d, "progress.json")))
      self.assertTrue(prog.get("done"))
      self.assertTrue(any(r.get("outcome") == "ok" for r in prog["reps"].values()))
      # a second process on the same dir must NOT restart the finished matrix
      b2 = self._mk(d)
      b2.cal_seq.mark_session(2)
      cc.actuators.accel = 0.0
      for f in range(0, 2000):
        b2.update(cc, cs, f)
      self.assertFalse(b2._cal_active, "a completed plan must not re-run")

  def test_reboot_mid_matrix_does_not_restart_completed_reps(self):
    with tempfile.TemporaryDirectory() as d:
      _write_plan(d, _cal_plan([(60, 10, 0.5), (60, 20, 0.5)]))
      cc = structs.CarControl()
      cc.enabled = cc.longActive = True
      cc.actuators.accel = 0.0
      cs = _CalCS(v_ego=60 / 3.6, cam=_cam())
      b = self._mk(d)
      # complete only the first rep, then "reboot"
      for f in range(0, 900):
        b.update(cc, cs, f)
        pj = os.path.join(d, "progress.json")
        if os.path.exists(pj) and json.load(open(pj)).get("reps", {}).get("r0"):
          break
      b2 = self._mk(d)
      b2.cal_seq.mark_session(2)
      decs = []
      for f in range(0, 2000):
        for s in b2.update(cc, cs, f):
          decs.append(fl._get_bits(s.dat, 8, 8))
      # only the remaining rep ran: exactly ONE episode (one passive 0 frame), and it reached 20
      self.assertEqual(20, max(decs, default=0), "only the remaining 20 LSB rep runs")
      self.assertEqual(1, sum(1 for d in decs if d == 0), "exactly one episode -> the done rep did not re-run")

  def test_progress_survives_a_new_plan_revision_and_only_new_reps_run(self):
    with tempfile.TemporaryDirectory() as d:
      b = self._mk(d)
      _write_plan(d, _cal_plan([(60, 20, 0.5)]))
      cc = structs.CarControl()
      cc.enabled = cc.longActive = True
      cc.actuators.accel = 0.0
      cs = _CalCS(v_ego=60 / 3.6, cam=_cam())
      for f in range(0, 2000):
        b.update(cc, cs, f)
      self.assertTrue(b.cal_seq.is_done)
      # a NEW plan revision ADDS a gap-fill rep: completion is keyed by rep id, so the already-done
      # rep (r0) is NOT re-run and only the new rep (extra) runs.
      doc = _cal_plan([(60, 20, 0.5)])
      doc["rev"] = 2
      doc["reps"].append({"id": "extra", "phase": "steps", "band_kph": 60, "level_lsb": 30, "hold_s": 0.5})
      _write_plan(d, doc)
      b2 = self._mk(d)
      b2.cal_seq.mark_session(2)
      decs = []
      for f in range(0, 3000):
        for s in b2.update(cc, cs, f):
          decs.append(fl._get_bits(s.dat, 8, 8))
      # the already-done rep (id r0) is NOT re-run; only the new gap-fill rep (id extra) runs
      self.assertEqual(30, max(decs, default=0), "only the new 30 LSB gap-fill rep runs")
      self.assertEqual(1, sum(1 for d in decs if d == 0), "exactly one new episode (progress keyed by rep id)")

  def test_per_drive_cap_stops_after_n_reps(self):
    with tempfile.TemporaryDirectory() as d:
      _write_plan(d, _cal_plan([(60, lv, 0.3) for lv in (5, 8, 10, 12, 15)], per_drive_cap=2))
      b = self._mk(d)
      cc = structs.CarControl()
      cc.enabled = cc.longActive = True
      cc.actuators.accel = 0.0
      cs = _CalCS(v_ego=60 / 3.6, cam=_cam())
      for f in range(0, 6000):
        b.update(cc, cs, f)
      prog = json.load(open(os.path.join(d, "progress.json")))
      ok = [r for r in prog["reps"].values() if r.get("outcome") == "ok"]
      self.assertEqual(2, len(ok), "the per-drive soft cap stops after 2 reps")

  def test_gas_neutralization_only_during_a_cal_hold(self):
    """The brake exposes _cal_active ONLY on frames it commands a scripted hold; it is False for a
    planner-driven brake and for an off plan (so the gas path is byte-identical when cal is off)."""
    with tempfile.TemporaryDirectory() as d:
      # no plan: a planner-driven brake must NOT set _cal_active
      b = self._mk(d)
      cc = structs.CarControl()
      cc.enabled = cc.longActive = True
      cc.actuators.accel = -2.0
      cs = _CalCS(v_ego=60 / 3.6, cam=_cam())
      b.update(cc, cs, 0)
      self.assertTrue(b.braking)
      self.assertFalse(b._cal_active)
      # with a due plan: a cal hold sets _cal_active
      _write_plan(d, _cal_plan([(60, 25)]))
      cc.actuators.accel = 0.0
      for f in range(0, 400):
        b.update(cc, cs, f)
        if b._cal_active:
          break
      self.assertTrue(b._cal_active)

  def test_malformed_progress_is_inert(self):
    """A present-but-corrupt progress.json must be treated as INERT, never as 'no progress' (which
    would silently re-run completed reps)."""
    with tempfile.TemporaryDirectory() as d:
      _write_plan(d, _cal_plan([(60, 20, 0.5)]))
      with open(os.path.join(d, "progress.json"), "w") as f:
        f.write("{not json")
      b = self._mk(d)
      cc = structs.CarControl()
      cc.enabled = cc.longActive = True
      cc.actuators.accel = 0.0
      cs = _CalCS(v_ego=60 / 3.6, cam=_cam())
      for f in range(0, 800):
        b.update(cc, cs, f)
        self.assertFalse(b._cal_active, f)
      self.assertEqual("progress_malformed", b.cal_seq._block_reason)

  def test_plan_lapse_mid_hold_drops_cal(self):
    """A plan/file lapse mid-hold (deleted / TTL fired) is an explicit VETO: cal drops and the
    production release closes the episode (one passive frame), not a silent continue."""
    with tempfile.TemporaryDirectory() as d:
      b = self._mk(d)
      _write_plan(d, _cal_plan([(60, 25, 5.0)]))   # a long hold we will interrupt
      cc = structs.CarControl()
      cc.enabled = cc.longActive = True
      cc.actuators.accel = 0.0
      cs = _CalCS(v_ego=60 / 3.6, cam=_cam())
      got = []
      hit = False
      for f in range(0, 800):
        if not hit and b._cal_active:
          os.remove(os.path.join(d, "plan.json"))   # plan vanishes mid-hold
          hit = True
        for s in b.update(cc, cs, f):
          got.append((f, fl._get_bits(s.dat, 8, 8)))
      self.assertTrue(hit, "the hold must have started")
      self.assertFalse(b._cal_active)
      self.assertTrue(any(g[1] == 0 for g in got), "the episode is released with a passive frame")

  def test_planner_ask_is_never_silenced_by_cal(self):
    """Fable hardening: the scripted level is a FLOOR. A harder planner ask mid-hold must still win
    (cal can only ADD braking)."""
    with tempfile.TemporaryDirectory() as d:
      b = self._mk(d)
      _write_plan(d, _cal_plan([(60, 5, 5.0)]))    # scripted 5 LSB
      cc = structs.CarControl()
      cc.enabled = cc.longActive = True
      cc.actuators.accel = 0.0
      cs = _CalCS(v_ego=60 / 3.6, cam=_cam())
      for f in range(0, 200):
        b.update(cc, cs, f)                        # bring the hold up at the scripted 5
      self.assertTrue(b._cal_active)
      # now the PLANNER asks hard (a lead) mid-hold: the emitted command must exceed the scripted 5
      cc.actuators.accel = -2.0
      decs = []
      for f in range(200, 400):
        for s in b.update(cc, cs, f):
          decs.append(fl._get_bits(s.dat, 8, 8))
      self.assertTrue(decs)
      self.assertGreater(max(decs), 5, "the harder planner ask must not be silenced")

  def test_gas_neutralization_has_no_same_frame_gap(self):
    """Fable timing item: the FIRST frame a cal hold is active must also be a frame that emits an
    actuating 0x38D (the sequencer decides on the 50 Hz SEND slot), so no frame exists where gas is
    forced to 0 but no brake command goes out."""
    with tempfile.TemporaryDirectory() as d:
      b = self._mk(d)
      _write_plan(d, _cal_plan([(60, 25)]))
      cc = structs.CarControl()
      cc.enabled = cc.longActive = True
      cc.actuators.accel = 0.0
      cs = _CalCS(v_ego=60 / 3.6, cam=_cam())
      first = None
      for f in range(0, 600):
        sends = b.update(cc, cs, f)
        has = any(int(s.address) == fl.FCA11_ADDR and fl._get_bits(s.dat, 8, 8) > 0 for s in sends)
        if b._cal_active and first is None:
          first = (f, has, bool(sends))
      self.assertIsNotNone(first, "a cal hold must have been active")
      self.assertTrue(first[1], f"the first cal_active frame must also actuate: {first}")

  def test_status_preflight(self):
    """The operator's pre-flight status: no plan -> inert reason; a valid+due plan -> counts; and it
    reports the toggle-independent plan state."""
    with tempfile.TemporaryDirectory() as d:
      b = self._mk(d)
      self.assertFalse(b.cal_seq.status()["plan_present"])
      self.assertFalse(b.cal_seq.status()["plan_valid"])
      _write_plan(d, _cal_plan([(60, 20, 0.5), (60, 30, 0.5)]))
      cs = _CalCS(v_ego=60 / 3.6, cam=_cam())
      b.cal_seq.tick(structs.CarControl(enabled=True, longActive=True), cs, 0)
      st = b.cal_seq.status()
      self.assertTrue(st["plan_present"] and st["plan_valid"])
      self.assertEqual(2, st["n_reps_total"])
      self.assertEqual(0, st["n_reps_done"])
      self.assertFalse(st["done"])

  def test_mid_hold_step_down_partial_release(self):
    """The contract's partial-release test (0.20 -> 0.10 g): a rep with level2_lsb/switch_at_s steps
    the command DOWN mid-hold, emitted immediately, within one clean episode."""
    with tempfile.TemporaryDirectory() as d:
      b = self._mk(d)
      _write_plan(d, _cal_plan([(60, 20, 2.2, 10)]))   # hold 2.2 s, step to 10 @1.1 s
      cc = structs.CarControl()
      cc.enabled = cc.longActive = True
      cc.actuators.accel = 0.0
      cs = _CalCS(v_ego=60 / 3.6, cam=_cam())
      # patch the plan to carry the step fields
      import json as _json
      doc = _json.load(open(os.path.join(d, "plan.json")))
      doc["reps"][0]["level2_lsb"] = 10
      doc["reps"][0]["switch_at_s"] = 1.1
      _write_plan(d, doc)
      decs = []
      for f in range(0, 800):
        for s in b.update(cc, cs, f):
          decs.append(fl._get_bits(s.dat, 8, 8))
      self.assertIn(20, decs, "the hold reaches the 20 LSB plateau")
      self.assertIn(10, decs, "the mid-hold step-down reaches 10 LSB")
      # the step-down comes AFTER the 20 plateau and only one episode closes
      i20 = decs.index(20)
      i10 = decs.index(10)
      self.assertGreater(i10, i20)
      self.assertEqual(1, sum(1 for dd in decs if dd == 0), "one episode -> one passive frame")

  # *** 0035 (F8): budget-free arming + the tool-side hold cap + watchdog ***

  def _run(self, b, cc, cs, frames, start=0):
    """Step update(); return per-frame (frame, braking, [sent decs], cal_active)."""
    out = []
    for f in range(start, start + frames):
      decs = [fl._get_bits(x.dat, 8, 8) for x in b.update(cc, cs, f)]
      out.append((f, b.braking, decs, b._cal_active))
    return out

  def test_f8_arming_is_budget_free_right_after_a_long_planner_brake(self):
    """F8: a planner brake LONGER than the old 2.5 s budget closes, and a due cal rep arms as soon as its own
    conditions hold (band stable 0.5 s) - not after a 3 s cooldown. RED on 0034: the planner brake was clipped at
    frame 251 and the 0031 predicate held the rep off until the cooldown (+margin) had run."""
    with tempfile.TemporaryDirectory() as d:
      b = self._mk(d)
      cc = structs.CarControl()
      cc.enabled = cc.longActive = True
      cs = _CalCS(v_ego=60 / 3.6, cam=_cam())
      cc.actuators.accel = -2.0
      trace = self._run(b, cc, cs, 400)                  # 4 s planner brake, never clipped
      self.assertTrue(all(t[1] for t in trace[1:]), "planner brake must not be clipped at 2.5 s")
      cc.actuators.accel = 0.0
      self._run(b, cc, cs, 2, start=400)                 # release + passive close
      _write_plan(d, _cal_plan([(60, 20, 1.0)]))
      started = next((f for f, _, _, cal in self._run(b, cc, cs, 300, start=402) if cal), None)
      self.assertIsNotNone(started, "the rep must arm")
      self.assertLess(started - 402, 120, f"armed {started - 402} frames after the close: no 3 s cooldown")

  def test_f8_hold_at_the_tool_cap_completes_untruncated(self):
    """F8: a rep held for exactly CAL_MAX_HOLD_S (10 s, 4x the old budget) brakes continuously for the whole
    hold - no gap, no watchdog trip - then closes with one passive frame and is recorded ok."""
    with tempfile.TemporaryDirectory() as d:
      b = self._mk(d)
      _write_plan(d, _cal_plan([(60, 20, cm.CAL_MAX_HOLD_S)]))
      cc = structs.CarControl()
      cc.enabled = cc.longActive = True
      cc.actuators.accel = 0.0
      cs = _CalCS(v_ego=60 / 3.6, cam=_cam())
      trace = self._run(b, cc, cs, 2400)
      on = [t[0] for t in trace if t[1]]
      self.assertTrue(on, "the rep must actuate")
      self.assertEqual(list(range(on[0], on[-1] + 1)), on, "one contiguous episode, never a gap")
      self.assertGreaterEqual(len(on), int(cm.CAL_MAX_HOLD_S * 100), "held for the whole 10 s")
      self.assertEqual(0, b.cal_watchdog_trips)
      self.assertEqual(1, sum(1 for t in trace for x in t[2] if x == 0), "one passive close frame")
      prog = json.load(open(os.path.join(d, "progress.json")))
      self.assertEqual("ok", prog["reps"]["r0"]["outcome"])

  def test_f8_over_cap_hold_is_refused(self):
    """F8: a rep whose hold exceeds CAL_MAX_HOLD_S is never armed (no actuation at all), with the reason surfaced;
    a legal rep in the same plan still runs."""
    with tempfile.TemporaryDirectory() as d:
      b = self._mk(d)
      _write_plan(d, _cal_plan([(60, 20, cm.CAL_MAX_HOLD_S + 0.5)]))
      cc = structs.CarControl()
      cc.enabled = cc.longActive = True
      cc.actuators.accel = 0.0
      cs = _CalCS(v_ego=60 / 3.6, cam=_cam())
      trace = self._run(b, cc, cs, 800)
      self.assertFalse(any(t[1] for t in trace), "an over-cap rep must never actuate")
      self.assertEqual("hold_over_cap", b.cal_seq._block_reason)
      self.assertIn("hold_over_cap", b.cal_seq.status()["hold_reason"])
      for bad in (0.0, -1.0, "x", None):
        self.assertFalse(cm.hold_ok(bad)[0], bad)
      self.assertTrue(cm.hold_ok(cm.CAL_MAX_HOLD_S)[0])

  def test_f8_watchdog_forces_the_close_when_the_sequencer_overruns(self):
    """F8: the brake's independent watchdog bounds a scripted hold even if the SEQUENCER's own hold arithmetic
    fails (simulated: the active rep's hold is corrupted to 100 s mid-hold). At CAL_WATCHDOG_FRAMES the ask is
    dropped, the production release emits one passive close frame, the rep is recorded 'watchdog' (not ok) and is
    not re-armed this session."""
    with tempfile.TemporaryDirectory() as d:
      b = self._mk(d)
      _write_plan(d, _cal_plan([(60, 20, 5.0)]))
      cc = structs.CarControl()
      cc.enabled = cc.longActive = True
      cc.actuators.accel = 0.0
      cs = _CalCS(v_ego=60 / 3.6, cam=_cam())
      f = 0
      while not b._cal_active and f < 400:
        b.update(cc, cs, f)
        f += 1
      self.assertTrue(b._cal_active)
      b.cal_seq._active.hold_s = 100.0                    # the sequencer fault
      trace = self._run(b, cc, cs, 2000, start=f)
      on = [t[0] for t in trace if t[1]]
      self.assertEqual(1, b.cal_watchdog_trips)
      self.assertLessEqual(len(on) + 1, cm.CAL_WATCHDOG_FRAMES + 1, "the watchdog bounds the hold")
      self.assertGreater(len(on), int(cm.CAL_MAX_HOLD_S * 100), "but never before the tool cap")
      self.assertEqual(1, sum(1 for t in trace for x in t[2] if x == 0), "one passive close frame")
      self.assertEqual("watchdog", json.load(open(os.path.join(d, "progress.json")))["reps"]["r0"]["outcome"])
      self.assertFalse(trace[-1][1], "and it is not re-armed this session")

  # *** 0036: a rep that commanded NO braking is never recorded 'ok' ***

  def test_0036_rep_vetoed_end_to_end_is_no_brake_not_ok(self):
    """Route 151: r2-stair-up-24/20/30 were logged 'ok' with ZERO actuating frames (a gas press had latched the
    car-layer driver cut). A rep whose whole hold is vetoed must be recorded 'no_brake' (not complete, retried),
    and a rep that did brake is still 'ok'. RED on 0035: the vetoed rep is recorded 'ok'. (0038: gas no longer
    latches, so a BRAKE-held veto - a real driver latch - stands in for the same 'vetoed whole hold' scenario.)"""
    with tempfile.TemporaryDirectory() as d:
      b = self._mk(d)
      _write_plan(d, _cal_plan([(60, 24, 1.0)]))
      cc = structs.CarControl()
      cc.enabled = cc.longActive = True
      cc.actuators.accel = 0.0
      cs = _CalCS(v_ego=60 / 3.6, cam=_cam())
      cut = _CalCS(v_ego=60 / 3.6, cam=_cam(), brake=True)
      b.update(cc, cs, 0)                                                # engage edge consumed on a clean frame
      b.update(cc, cut, 1)                                             # brake held while engaged -> driver cut
      self.assertTrue(b.blocked_until_resume)                           # latched, no resume edge to re-arm it
      sent = []
      for f in range(2, 1200):
        sent += [fl._get_bits(x.dat, 8, 8) for x in b.update(cc, cut, f)]   # brake held the whole hold
        pj = os.path.join(d, "progress.json")
        if os.path.exists(pj) and json.load(open(pj)).get("reps", {}).get("r0"):
          break
      self.assertEqual([], [x for x in sent if x > 0], "the latch vetoed every frame of the hold")
      rec = json.load(open(os.path.join(d, "progress.json")))["reps"]["r0"]
      self.assertEqual("no_brake", rec["outcome"])
      self.assertFalse(b.cal_seq._is_complete(b.cal_seq.plan[0]), "a no_brake rep is NOT done; it is retried")
    with tempfile.TemporaryDirectory() as d:
      b = self._mk(d)
      _write_plan(d, _cal_plan([(60, 24, 1.0)]))
      cc = structs.CarControl()
      cc.enabled = cc.longActive = True
      cc.actuators.accel = 0.0
      cs = _CalCS(v_ego=60 / 3.6, cam=_cam())
      for f in range(0, 1200):
        b.update(cc, cs, f)
      self.assertEqual("ok", json.load(open(os.path.join(d, "progress.json")))["reps"]["r0"]["outcome"])

  def test_lead_condition_skips_the_high_level_rep(self):
    """A high-level rep (>= 20 LSB) is skipped while a lead is close; the sequencer waits."""
    with tempfile.TemporaryDirectory() as d:
      b = self._mk(d)
      _write_plan(d, _cal_plan([(60, 25)]))
      cs = _CalCS(v_ego=60 / 3.6, cam=_cam(), lead=40.0)
      for f in range(0, 600):
        b.cal_seq.tick(structs.CarControl(enabled=True, longActive=True), cs, f)
      self.assertFalse(b.cal_seq.active_actuation)
      self.assertEqual("lead_close", b.cal_seq._block_reason)
      # the lead clears -> the rep runs (it starts, holds 2.2 s, then releases; assert it actuated)
      cs.fca11_lead_drel_m = None
      ran = False
      for f in range(600, 1200):
        b.cal_seq.tick(structs.CarControl(enabled=True, longActive=True), cs, f)
        ran = ran or b.cal_seq.active_actuation
      self.assertTrue(ran, "with the lead gone the rep actuates")


class Test0030CameraLatchWithoutLongActive(unittest.TestCase):
  """0030 (D1-a): the camera hand-back latch must mirror the panda EXACTLY - set on a camera request even while
  openpilot is DISENGAGED (longActive False). The panda's hyundai_fca11_long_rx sets camera_owns/cut on ANY bus-2
  0x38D carrying an actuation/warning field with NO host-engagement condition (safety/modes/hyundai.h:228-236), and
  only hyundai_init clears it. FALSIFIABLE CONTRAST: on the deployed 29-patch tree every latch assert below FAILS
  (the old `CC.longActive and camera_requesting(...)` gate meant a stock FCW seen while disengaged never latched, so
  openpilot kept believing it could brake and kept commanding into a refused wall - 1983 refused actuating frames on
  drive 14d, incl. all three 20@90 cal reps)."""

  def _brake(self):
    cp_sp = structs.CarParamsSP()
    cp_sp.fca11Brake = True
    cp_sp.enableGasInterceptor = True
    return fl.Fca11LongBrake(structs.CarParams(), cp_sp)

  def _cc(self, accel=0.0, engaged=True, long_active=True):
    cc = structs.CarControl()
    cc.enabled = engaged
    cc.longActive = long_active
    cc.actuators.accel = accel
    return cc

  def test_camera_request_latches_while_long_inactive(self):
    b = self._brake()
    cam_req = (0, _real_camera_frame(warn=2))       # the camera itself is braking/warning
    # longActive OFF (and disengaged): the OLD update() returned in its `not CC.longActive` branch BEFORE the latch,
    # so the camera request was never seen at all.
    b.update(self._cc(long_active=False, engaged=False), _CS(cam=cam_req), 0)
    self.assertTrue(b._hard_cut, "camera request must latch the hand-back even when long is inactive")
    self.assertTrue(b.blocked_until_resume)
    self.assertTrue(b.fca11_unavailable)            # D1-b: surfaced
    # and it stays latched (not re-armed) once long comes back: openpilot must not silently keep braking
    b.update(self._cc(-2.0), _CS(cam=_cam()), 2)
    self.assertTrue(b.blocked_until_resume)
    self.assertEqual([], b.update(self._cc(-2.0), _CS(cam=_cam()), 4))
    self.assertFalse(b.braking)

  def test_camera_request_latches_while_long_inactive_through_controller(self):
    """The same divergence through the real controller entry point (create_gas_command), where longActive is the
    live CarControl value and openpilot can be disengaged at the moment of the stock FCW."""
    CarInterface, CP, CP_SP = _get_ci(True, "1")
    CI = CarInterface(CP, CP_SP)
    # use the real CarState (with its pedal_fault_monitor etc.) and just feed the camera frame + params; the
    # divergence we are pinning is the latch, not the pedal stream.
    cs = CI.CS
    cs.out = structs.CarState()
    cs.out.vEgo = 20.0
    cs.out.gearShifter = structs.CarState.GearShifter.drive
    cs.interceptor_state = 0
    cs.fca11_cam_frame = (0, _real_camera_frame(warn=2))
    cs.fca11_now_nanos = 0
    cs.fca11_echoes = []
    cc = self._cc(-2.0, engaged=False, long_active=False)
    sends = CI.CC.create_gas_command(cc, cs, 0, 0)
    self.assertFalse(any(int(a) == fl.FCA11_ADDR for a, _d, _s in sends))
    self.assertTrue(CI.CC.fca11_brake._hard_cut)

  def test_no_camera_request_does_not_latch_while_inactive(self):
    """Guard: the widened latch must NOT fire on an ordinary idle camera frame (else FCA11-long would self-cut)."""
    b = self._brake()
    b.update(self._cc(long_active=False, engaged=False), _CS(cam=_cam()), 0)   # idle camera shape
    self.assertFalse(b._hard_cut)
    self.assertFalse(b.fca11_unavailable)

  def test_fca11_unavailable_only_when_armed(self):
    """D1-b: the flag is exactly 'FCA11-long is unavailable now' - off (and never latched) when the toggle is off."""
    cp_sp = structs.CarParamsSP()
    cp_sp.fca11Brake = False
    cp_sp.enableGasInterceptor = True
    b = fl.Fca11LongBrake(structs.CarParams(), cp_sp)
    b.update(self._cc(-2.0), _CS(cam=(0, _real_camera_frame(warn=2))), 0)
    self.assertFalse(b.fca11_unavailable)


class Test0030EchoConfirmedClose(unittest.TestCase):
  """0030 (D2-iii): the car layer RE-OWES the close frame when the panda REFUSED it (src 192): the panda only ends
  the episode on an ACCEPTED passive 0x38D. (0035: there is no cooldown stamp any more; the echo only drives the
  re-owe.) FALSIFIABLE CONTRAST: on the deployed 29-patch tree observe_echo() does not even exist."""

  def _brake(self):
    cp_sp = structs.CarParamsSP()
    cp_sp.fca11Brake = True
    cp_sp.enableGasInterceptor = True
    return fl.Fca11LongBrake(structs.CarParams(), cp_sp)

  def _cc(self, accel=0.0, engaged=True, long_active=True):
    cc = structs.CarControl()
    cc.enabled = engaged
    cc.longActive = long_active
    cc.actuators.accel = accel
    return cc

  def _emit_close(self, b):
    """Open a hard-ask episode, then let the ask rise so the episode releases on the 50 Hz send slot; return
    (frame, passive_dat) of the single passive close frame."""
    for f in range(0, 6):
      b.update(self._cc(-2.0), _CS(cam=_cam()), f)        # hard ask (<= -1.2) opens immediately
    self.assertTrue(b.braking)
    for f in range(6, 400):
      s = b.update(self._cc(-0.2), _CS(cam=_cam()), f)    # ask above -0.30 -> release after min-on
      if not b.braking:
        if not s:
          s = b.update(self._cc(-0.2), _CS(cam=_cam()), f + 1)
        p = [x for x in s if int(x.address) == fl.FCA11_ADDR and fl._get_bits(x.dat, 8, 8) == 0]
        if p:
          return f + (1 if not s else 0), bytes(p[0].dat)
    raise AssertionError("no passive close frame emitted")

  def test_wired_refused_close_is_re_owed_and_re_emitted(self):
    b = self._brake()
    b.observe_echo([])                                     # wire the echo path (as create_gas_command does each frame)
    f_emit, dat = self._emit_close(b)
    self.assertEqual(dat, b._pending_echo, "the emitted close frame must be recorded for echo confirmation")
    self.assertFalse(b._release_pending)
    # the panda REFUSED it (engage/disengage edge): re-owe
    b.observe_echo([(192, dat)])
    self.assertTrue(b._release_pending, "a refused close frame must be re-owed")
    self.assertIsNone(b._pending_echo)
    # the re-owed close frame goes out on the NEXT send slot
    s = b.update(self._cc(-0.2), _CS(cam=_cam()), f_emit + 2)
    p = [x for x in s if int(x.address) == fl.FCA11_ADDR and fl._get_bits(x.dat, 8, 8) == 0]
    self.assertEqual(1, len(p), "the re-owed passive frame must be re-emitted")

  def test_wired_accepted_close_then_immediate_rebrake(self):
    """0035: an accepted close needs nothing more, and the next brake is commanded on the very next frame."""
    b = self._brake()
    b.observe_echo([])
    f_emit, dat = self._emit_close(b)
    b.observe_echo([(128, dat)])                          # accepted by the panda TX hook
    self.assertFalse(b._release_pending)
    self.assertIsNone(b._pending_echo)
    for f in range(f_emit + 1, f_emit + 400):
      b.update(self._cc(-2.0), _CS(cam=_cam()), f)
      self.assertTrue(b.braking, f)

  def test_refused_close_echo_arriving_after_a_rebrake_never_cuts_it(self):
    """0035: a REFUSED-close echo that lands after the next brake has already started must NOT re-owe the close
    frame (that would force the live brake off for a slot - a silent truncation)."""
    b = self._brake()
    b.observe_echo([])
    f_emit, dat = self._emit_close(b)
    b.update(self._cc(-2.0), _CS(cam=_cam()), f_emit + 2)
    self.assertTrue(b.braking)
    b.observe_echo([(192, dat)])                           # late refusal of the OLD close frame
    self.assertFalse(b._release_pending)
    for f in range(f_emit + 3, f_emit + 200):
      b.update(self._cc(-2.0), _CS(cam=_cam()), f)
      self.assertTrue(b.braking, f)

  def test_refused_actuating_echo_restarts_the_ramp(self):
    """0035: if the panda REFUSES an actuating frame mid-brake (e.g. its onset rule after a >100 ms stream gap), the
    next frame restarts at RATE_STEP instead of repeating the refused level forever (which would keep the stream
    stale and lose the rest of the brake). Passive/accepted echoes do not touch the ramp."""
    b = self._brake()
    b.observe_echo([])
    sent = []
    for f in range(0, 40):
      sent += [bytes(x.dat) for x in b.update(self._cc(-2.0), _CS(cam=_cam()), f, fl.PERSONALITY_AGGRESSIVE)]
    self.assertEqual(30, sent[-1][1])
    b.observe_echo([(128, sent[-1])])                     # accepted: no change
    s = b.update(self._cc(-2.0), _CS(cam=_cam()), 40, fl.PERSONALITY_AGGRESSIVE)
    self.assertEqual([30], [x.dat[1] for x in s])
    b.observe_echo([(192, bytes(s[0].dat))])              # refused actuating frame
    decs = []
    for f in range(41, 60):
      decs += [x.dat[1] for x in b.update(self._cc(-2.0), _CS(cam=_cam()), f, fl.PERSONALITY_AGGRESSIVE)]
    self.assertEqual(fl.HYUNDAI_FCA11_LONG_RATE_STEP, decs[0], "the ramp restarts at the onset step")
    self.assertEqual(30, decs[-1], "and climbs back to the ask")
    self.assertTrue(b.braking)

  def test_unwired_emits_the_close_and_records_no_echo(self):
    """Guard: without an observe_echo call the close frame is still emitted once and nothing waits on an echo."""
    b = self._brake()
    f_emit, dat = self._emit_close(b)
    self.assertFalse(b._release_pending)
    self.assertIsNone(b._pending_echo)

  def test_wired_retries_until_an_echo_is_accepted(self):
    b = self._brake()
    b.observe_echo([])
    f_emit, dat = self._emit_close(b)
    b.observe_echo([(192, dat)])                         # refused
    self.assertTrue(b._release_pending)
    # re-emit and refuse again: still owed
    s = b.update(self._cc(-0.2), _CS(cam=_cam()), f_emit + 2)
    p = [x for x in s if int(x.address) == fl.FCA11_ADDR and fl._get_bits(x.dat, 8, 8) == 0]
    self.assertEqual(1, len(p))
    b.observe_echo([(192, bytes(p[0].dat))])
    self.assertTrue(b._release_pending)
    # finally accepted
    s = b.update(self._cc(-0.2), _CS(cam=_cam()), f_emit + 4)
    p = [x for x in s if int(x.address) == fl.FCA11_ADDR and fl._get_bits(x.dat, 8, 8) == 0]
    self.assertEqual(1, len(p))
    b.observe_echo([(128, bytes(p[0].dat))])
    self.assertFalse(b._release_pending)
    self.assertIsNone(b._pending_echo)


class TestEchoCollectionShape(unittest.TestCase):
  """0030 field regression: the echo collector must accept the REAL transport shape.

  ``can_capnp_to_list`` (openpilot/selfdrive/pandad/pandad_api_impl.py) yields
  ``(logMonoTime, [(address, data, src), ...])`` -- plain TUPLES. The first shipped revision of 0030 read
  ``c.address``/``c.src`` on them, so card raised AttributeError on the very first CAN frame and
  crash-looped: no fingerprint -> "Unknown Vehicle Variant" on screen, openpilot dead. These tests pin
  BOTH shapes (real tuples from the device, CanData objects from the offline harnesses) so the mistake
  cannot come back.
  """

  def test_tuples_from_the_real_transport(self):
    from opendbc.car.interfaces import collect_fca11_echoes
    # exactly what can_capnp_to_list produces
    cans = [(123456, [(0x38D, b"\x01\x08\x00\x00\x00\x00\x00\x00", 128),
                      (0x38D, b"\x01\x08\x00\x00\x00\x00\x00\x00", 192),
                      (0x340, b"\x00" * 8, 2),
                      (0x38D, b"\x02" * 8, 0)])]          # src 0 (camera) must be ignored
    out = collect_fca11_echoes(cans)
    self.assertEqual([(128, b"\x01\x08\x00\x00\x00\x00\x00\x00"),
                      (192, b"\x01\x08\x00\x00\x00\x00\x00\x00")], out)

  def test_objects_from_the_offline_harness(self):
    from opendbc.car.interfaces import collect_fca11_echoes

    class C:
      def __init__(self, address, dat, src):
        self.address, self.dat, self.src = address, dat, src
    cans = [(0, [C(0x38D, b"\xaa" * 8, 128), C(0x1FF, b"\xbb" * 8, 128)])]
    self.assertEqual([(128, b"\xaa" * 8)], collect_fca11_echoes(cans))

  def test_empty_and_mixed(self):
    from opendbc.car.interfaces import collect_fca11_echoes
    self.assertEqual([], collect_fca11_echoes([]))
    self.assertEqual([], collect_fca11_echoes([(0, [])]))


class Test0041AlertGating(unittest.TestCase):
  """0041: the SHIP-BLOCKING BRAKE NOW false-positives. 'no_response' is MOVING-only with a command SUSTAINED for
  the ESC lag, and 'hold_lost' disarms the instant we stop commanding the hold - so a held standstill and an
  openpilot auto-launch are both quiet. These drive the REAL class with a mocked clock (the 500 ms / 0.62 s / 2 s
  timers all read time.monotonic)."""

  def setUp(self):
    self._mono = time.monotonic
    self.t = 1000.0
    time.monotonic = lambda: self.t

  def tearDown(self):
    time.monotonic = self._mono

  def _brake(self):
    cp_sp = structs.CarParamsSP()
    cp_sp.fca11Brake = True
    cp_sp.enableGasInterceptor = True
    return fl.Fca11LongBrake(structs.CarParams(), cp_sp)

  def _cc(self, accel=0.0, engaged=True, long_active=True):
    cc = structs.CarControl()
    cc.enabled = engaged
    cc.longActive = long_active
    cc.actuators.accel = accel
    return cc

  def _tick(self, b, cc, cs, frame, dt=0.01):
    self.t += dt
    return b.update(cc, cs, frame)

  def test_brake_now_does_not_fire_for_a_held_standstill(self):
    """RED on 0040: a car HELD at standstill (aEgo ~ 0, ask -0.73 = the pass-1 median) trips 'no_response' in 0.51 s.
    Under 0041 the motion gate (v > 5 km/h) makes it impossible - a delivered net decel of ~0 at a stop is CORRECT."""
    b = self._brake()
    cc = self._cc(-0.73)
    cs = _CS(v_ego=0.0, cam=_cam(), standstill=True, a_ego=0.0)
    for f in range(300):                       # 3 s of a standstill hold
      self._tick(b, cc, cs, f)
      self.assertFalse(b.brake_now, f"BRAKE NOW fired at a standstill hold @frame {f}")
    self.assertTrue(b.braking)
    self.assertTrue(b.stop_complete)

  def test_brake_now_does_not_fire_on_openpilot_auto_launch(self):
    """RED on 0040: after an FCA11 stop the planner LAUNCHES (ask -> positive, braking releases, v rises past 2 km/h
    with NO driver gas) and 'hold_lost' fires 0.56 s in. Under 0041 the arm disarms the moment we stop commanding the
    hold (the planner released), so the launch never raises it."""
    b = self._brake()
    # 1) a REAL stop hold (seconds long, as at a red light): armed, nothing fires
    for f in range(300):
      self._tick(b, self._cc(-2.0), _CS(v_ego=0.0, cam=_cam(), standstill=True, a_ego=-2.0), f)
    self.assertTrue(b.braking)
    self.assertTrue(b._hold_lost_armed)
    self.assertFalse(b.brake_now)
    # 2) openpilot LAUNCHES: ask > 0 for the whole phase, the driver NEVER touches the gas
    for f in range(300, 600):
      self._tick(b, self._cc(1.0), _CS(v_ego=min(8.0, (f - 300) * 0.01), cam=_cam(), a_ego=1.0), f)
      self.assertFalse(b.brake_now, f"BRAKE NOW fired on an openpilot launch @frame {f}")
    self.assertFalse(b._hold_lost_armed, "the arm must disarm when we stop commanding the hold")

  def test_brake_now_fires_true_positive_when_moving_and_unresponsive(self):
    """TRUE POSITIVE: moving > 5 km/h, commanding >= 8 LSB, but the ESC delivers ~0 -> RED 'no_response'."""
    b = self._brake()
    cc = self._cc(-2.0)
    cs = _CS(v_ego=20.0 / 3.6, cam=_cam(), a_ego=0.0)
    for f in range(200):                      # ramp + 0.62 s command-stable + 0.5 s measurement
      self._tick(b, cc, cs, f)
      if b.brake_now:
        break
    self.assertTrue(b.brake_now, "a sustained unresponsive brake while moving must raise BRAKE NOW")
    self.assertEqual("no_response", b.brake_now_reason)

  def test_brake_now_requires_a_sustained_command(self):
    """The command-stable gate: the 500 ms timer cannot even start until the command has held >= 8 LSB for the ESC
    lag. A single frame that only ramps to RATE_STEP (4 < 8) never opens it."""
    b = self._brake()
    b.update(self._cc(-2.0), _CS(v_ego=20.0 / 3.6, cam=_cam(), a_ego=0.0), 0)
    self.assertLess(b._dec_last_sent, fl.FCA11_NORESPONSE_MIN_LSB)
    self.assertIsNone(b._noresp_cmd_since, "a sub-8-LSB command must not open the command-stable gate")
    self.assertFalse(b.brake_now)

  def test_brake_now_self_heals_after_two_seconds(self):
    """0041 latching: persistence is frame-to-frame survival, not forever. A transient clears 2 s after the trigger
    goes away (and immediately on driver brake / gear != D / gas)."""
    b = self._brake()
    cc = self._cc(-2.0)
    cs = _CS(v_ego=20.0 / 3.6, cam=_cam(), a_ego=0.0)
    for f in range(200):
      self._tick(b, cc, cs, f)
      if b.brake_now:
        break
    self.assertTrue(b.brake_now)
    good = _CS(v_ego=20.0 / 3.6, cam=_cam(), a_ego=-5.0)   # the ESC responds: condition false from here
    self._tick(b, cc, good, 500)
    self.assertTrue(b.brake_now, "not cleared instantly: it is persistent")
    for f in range(501, 501 + 250):          # > 2 s of a continuously false condition
      self._tick(b, cc, good, f)
      if not b.brake_now:
        break
    self.assertFalse(b.brake_now, "self-heal must clear BRAKE NOW after 2 s")
    self.assertEqual("", b.brake_now_reason)


class Test0041AffineGain(unittest.TestCase):
  """0041: the OPT-IN affine command law (HyundaiFca11AffineGain, DEFAULT OFF). OFF -> byte-identical to 0040 (the
  0.67 through-origin law + the 8 LSB floor); ON -> the fitted K/BITE inverse with the 4 LSB release band floor WHILE
  MOVING and the >= 8 LSB hold floor at/below the low-speed stop band. self.affine_gain is resolved ONCE at __init__.
  The env FCA11_AFFINE is an explicit A/B override (wins when set); these tests clear it so the field-based cases are
  deterministic regardless of the ambient environment."""

  def setUp(self):
    self._env_prev = os.environ.pop("FCA11_AFFINE", None)

  def tearDown(self):
    os.environ.pop("FCA11_AFFINE", None)
    if self._env_prev is not None:
      os.environ["FCA11_AFFINE"] = self._env_prev

  def _brake(self, affine):
    cp_sp = structs.CarParamsSP()
    cp_sp.fca11Brake = True
    cp_sp.enableGasInterceptor = True
    cp_sp.fca11AffineGain = affine
    return fl.Fca11LongBrake(structs.CarParams(), cp_sp)

  def _cc(self, accel=0.0, engaged=True, long_active=True):
    cc = structs.CarControl()
    cc.enabled = engaged
    cc.longActive = long_active
    cc.actuators.accel = accel
    return cc

  def test_default_off_and_constants(self):
    self.assertFalse(self._brake(False).affine_gain, "the affine law must default OFF")
    self.assertTrue(self._brake(True).affine_gain)
    # the fitted constants (K CI [9.43, 10.38], BITE CI [0.33, 0.48]) and the 4 LSB release band floor
    self.assertEqual(9.9, fl.FCA11_AFFINE_K)
    self.assertEqual(0.40, fl.FCA11_AFFINE_BITE)
    self.assertEqual(4, fl.FCA11_AFFINE_RELEASE_LSB)
    self.assertLess(fl.FCA11_AFFINE_RELEASE_LSB, fl.FCA11_NORESPONSE_MIN_LSB, "affine releases below the 8 LSB floor")
    # FIX (B): the affine ON path keeps the deployed hold floor in the low-speed band (== the 8 LSB floor by construction)
    self.assertEqual(8, fl.FCA11_AFFINE_HOLD_LSB)
    self.assertEqual(fl.FCA11_NORESPONSE_MIN_LSB, fl.FCA11_AFFINE_HOLD_LSB)
    self.assertEqual(3.0, fl.FCA11_AFFINE_STOP_BAND_KPH)

  def test_env_override_has_priority_over_the_field(self):
    """0041 FIX (A): FCA11_AFFINE is an EXPLICIT A/B override that WINS over the always-present capnp field.
    The field defaults to False but is ALWAYS present, so reading it FIRST (the old bug) shadowed the sim's only way
    in and FCA11_AFFINE=1 was ignored. Order now: explicit env -> field -> guarded Params key -> default."""
    cp_on = structs.CarParamsSP()
    cp_on.fca11AffineGain = True
    cp_off = structs.CarParamsSP()
    cp_off.fca11AffineGain = False

    # explicit truthy env -> ON even when the field is False (the defect: this used to return False)
    for tok in ("1", "true", "TRUE", "yes", "on"):
      try:
        os.environ["FCA11_AFFINE"] = tok
        self.assertTrue(fl.affine_gain_enabled(cp_off), tok)
        self.assertTrue(fl.affine_gain_enabled(structs.CarParamsSP()), tok)
      finally:
        os.environ.pop("FCA11_AFFINE", None)
    # explicit falsy env -> OFF even when the field is True
    for tok in ("0", "false", "no", "off"):
      try:
        os.environ["FCA11_AFFINE"] = tok
        self.assertFalse(fl.affine_gain_enabled(cp_on), tok)
      finally:
        os.environ.pop("FCA11_AFFINE", None)
    # env UNSET -> the CP_SP field is authoritative
    os.environ.pop("FCA11_AFFINE", None)
    self.assertTrue(fl.affine_gain_enabled(cp_on))
    self.assertFalse(fl.affine_gain_enabled(cp_off))
    # env UNSET and the field ABSENT -> the guarded Params key / default (no Params service here -> default)
    class _NoField:
      pass
    self.assertFalse(fl.affine_gain_enabled(_NoField(), default=False))
    # an empty / unrecognized value is NOT an explicit override: fall through to the field
    try:
      os.environ["FCA11_AFFINE"] = ""
      self.assertTrue(fl.affine_gain_enabled(cp_on))
      os.environ["FCA11_AFFINE"] = "maybe"
      self.assertFalse(fl.affine_gain_enabled(cp_off))
    finally:
      os.environ.pop("FCA11_AFFINE", None)

  def test_env_override_flips_the_controller(self):
    """End to end: with the field False, FCA11_AFFINE=1 must flip Fca11LongBrake.affine_gain ON (read once in __init__)."""
    try:
      os.environ["FCA11_AFFINE"] = "1"
      cp_sp = structs.CarParamsSP()
      cp_sp.fca11Brake = True
      cp_sp.enableGasInterceptor = True
      cp_sp.fca11AffineGain = False
      self.assertTrue(fl.Fca11LongBrake(structs.CarParams(), cp_sp).affine_gain)
    finally:
      os.environ.pop("FCA11_AFFINE", None)
    cp_sp = structs.CarParamsSP()
    cp_sp.fca11Brake = True
    cp_sp.enableGasInterceptor = True
    self.assertFalse(fl.Fca11LongBrake(structs.CarParams(), cp_sp).affine_gain)

  def test_affine_law_inverts_the_affine_plant_and_tapering(self):
    # want_extra = 0.8 -> (0.8 - 0.40) / 9.9 g = 0.04040 g -> 4 LSB  (vs the 0.67 law's 12 LSB)
    self.assertEqual(4, fl.dec_cmd_from_accel_affine(-0.8))
    self.assertEqual(12, fl.dec_cmd_from_accel(-0.8))
    # a want at/below the plant's own BITE delivers more than asked -> command nothing; coast above onset -> 0
    self.assertEqual(0, fl.dec_cmd_from_accel_affine(-0.40))
    self.assertEqual(0, fl.dec_cmd_from_accel_affine(-0.30))
    self.assertEqual(0, fl.dec_cmd_from_accel_affine(0.0))
    self.assertEqual(0, fl.dec_cmd_from_accel_affine(fl.FCA11_BRAKE_ONSET_ACCEL + 1e-6))
    # clamp to the panda gate (30) and monotone in magnitude
    self.assertEqual(30, fl.dec_cmd_from_accel_affine(-4.0))
    vals = [fl.dec_cmd_from_accel_affine(-a) for a in (0.5, 0.8, 1.0, 1.5, 2.0)]
    self.assertEqual(vals, sorted(vals))

  def test_affine_path_inert_when_off(self):
    """OFF -> the emitted CR_VSM_DecCmd is EXACTLY the 0.67 law's (byte-identical to 0040)."""
    b = self._brake(False)
    decs = []
    for f in range(0, 120, 2):
      for s in b.update(self._cc(-0.8), _CS(cam=_cam()), f, fl.PERSONALITY_AGGRESSIVE):
        decs.append(fl._get_bits(s.dat, 8, 8))
    old = fl.dec_cmd_from_accel(-0.8)                       # 12
    self.assertEqual(min(old, fl.HYUNDAI_FCA11_LONG_RATE_STEP), decs[0], "first sent frame is rate-limited")
    self.assertEqual(old, max(decs), "OFF must command the 0.67 value, unchanged")

  def test_affine_path_changed_when_on(self):
    """ON -> the request inverts the AFFINE plant; a -0.8 ask commands 4 LSB while MOVING, not the 0.67 law's 12."""
    b = self._brake(True)
    decs = []
    for f in range(0, 120, 2):
      for s in b.update(self._cc(-0.8), _CS(v_ego=20.0 / 3.6, cam=_cam()), f, fl.PERSONALITY_AGGRESSIVE):
        decs.append(fl._get_bits(s.dat, 8, 8))
    self.assertTrue(decs)
    self.assertEqual(4, max(decs), "the affine law commands 4 LSB for a -0.8 ask, not 12")
    self.assertEqual(4, min(decs), "and it holds there (the honest level), never over-commanding")

  def test_affine_on_keeps_the_hold_floor_at_standstill(self):
    """FIX (B): at a standstill the affine ON path must NOT weaken the hold below today's 8 LSB. A DCT car creeps at a
    stop and the sim cannot see the ESC, so the released 4 LSB floor is only legal WHILE MOVING.

    FIX (low-speed refusal): the 8 LSB hold floor is a REQUEST floor applied BEFORE the +RATE_STEP rate limiter, so the
    FIRST emitted actuating frame is the rate-limited onset step (4 LSB, <= RATE_STEP, which the panda ACCEPTS) and the
    hold floor (8) is reached by the 2nd frame. The floor is never applied after the limiter (that lifted the first
    frame straight to 8, over the panda's onset gate, so every frame was refused and the brake never engaged)."""
    b = self._brake(True)
    decs = []
    for f in range(0, 120, 2):
      for s in b.update(self._cc(-0.73), _CS(v_ego=0.0, cam=_cam(), standstill=True, a_ego=0.0), f,
                        fl.PERSONALITY_AGGRESSIVE):
        decs.append(fl._get_bits(s.dat, 8, 8))
    self.assertTrue(decs)
    # onset obeys the panda's +RATE_STEP gate ...
    self.assertLessEqual(decs[0], fl.HYUNDAI_FCA11_LONG_RATE_STEP,
                         f"first standstill frame must be <= RATE_STEP so the panda accepts it, got {decs}")
    self.assertEqual(fl.HYUNDAI_FCA11_LONG_RATE_STEP, decs[0])
    # ... and the >= 8 LSB hold floor is reached by the 2nd emitted frame (4 -> 8 in one 20 ms camera period)
    self.assertGreaterEqual(decs[1], fl.FCA11_AFFINE_HOLD_LSB, f"hold floor reached by frame 2, got {decs}")
    self.assertEqual(8, max(decs))
    # the ramp NEVER jumps: every consecutive SENT frame grows <= RATE_STEP (the panda's per-accepted-frame rule)
    for a, c in zip(decs, decs[1:]):
      self.assertLessEqual(c - a, fl.HYUNDAI_FCA11_LONG_RATE_STEP, (a, c))

  def test_affine_on_standstill_first_frame_never_exceeds_the_panda_onset_gate(self):
    """RED-on-the-bug regression: the OLD affine path applied the 8 LSB floor AFTER the +RATE_STEP limiter, so the
    FIRST emitted actuating frame was 8 LSB -- over the panda's onset rule (only +RATE_STEP over the last ACCEPTED
    frame, which is 0 at an episode open), so the panda refused EVERY frame (dec=8 last=0, 494 refusals observed) and
    the brake never engaged. With the floor on the REQUEST the first frame is the rate-limited step (4), which the
    panda accepts. Asserted on the emitted CR_VSM_DecCmd bytes, not on internals."""
    for in_hold in (True, False):
      cs = _CS(v_ego=0.0, cam=_cam(), standstill=True, a_ego=0.0) if in_hold else _CS(v_ego=20.0 / 3.6, cam=_cam())
      b = self._brake(True)
      decs = []
      for f in range(0, 120, 2):
        for s in b.update(self._cc(-0.73), cs, f, fl.PERSONALITY_AGGRESSIVE):
          decs.append(fl._get_bits(s.dat, 8, 8))
      self.assertTrue(decs)
      self.assertLessEqual(decs[0], fl.HYUNDAI_FCA11_LONG_RATE_STEP,
                           f"hold_band={in_hold}: first emitted frame must not exceed RATE_STEP, got {decs}")
      # the floor the law wants is reached by the 2nd frame either way (hold 8 / release 4); a moving ask stays at 4
      want = fl.FCA11_AFFINE_HOLD_LSB if in_hold else fl.FCA11_AFFINE_RELEASE_LSB
      self.assertGreaterEqual(decs[1], min(want, fl.FCA11_AFFINE_RELEASE_LSB), (in_hold, decs))
      self.assertEqual(want, max(decs), (in_hold, decs))
      for a, c in zip(decs, decs[1:]):
        self.assertLessEqual(c - a, fl.HYUNDAI_FCA11_LONG_RATE_STEP, (in_hold, a, c))

  def test_affine_on_moving_modest_ask_commands_the_affine_result(self):
    """FIX (B) contrast: an identical modest ask while MOVING (above the stop band) commands the honest affine result
    (4 LSB), NOT the 8 LSB hold floor - the hold floor is scoped to the low-speed band only."""
    b = self._brake(True)
    decs = []
    for f in range(0, 120, 2):
      for s in b.update(self._cc(-0.8), _CS(v_ego=20.0 / 3.6, cam=_cam()), f, fl.PERSONALITY_AGGRESSIVE):
        decs.append(fl._get_bits(s.dat, 8, 8))
    self.assertEqual([4], sorted(set(decs)), f"moving affine ask is 4 LSB, got {decs}")

  def test_affine_on_creep_stays_in_the_hold_band(self):
    """FIX (B) band edge: v <= FCA11_AFFINE_STOP_BAND_KPH (3 km/h), or standstill, keeps the 8 LSB floor; just above it,
    the honest affine level applies."""
    b = self._brake(True)
    decs = []
    for f in range(0, 60, 2):
      for s in b.update(self._cc(-0.73), _CS(v_ego=2.0 / 3.6, cam=_cam()), f, fl.PERSONALITY_AGGRESSIVE):
        decs.append(fl._get_bits(s.dat, 8, 8))
    self.assertEqual(8, max(decs), "creeping (2 km/h <= 3) keeps the hold floor")
    b2 = self._brake(True)
    decs2 = []
    for f in range(0, 60, 2):
      for s in b2.update(self._cc(-0.73), _CS(v_ego=10.0 / 3.6, cam=_cam()), f, fl.PERSONALITY_AGGRESSIVE):
        decs2.append(fl._get_bits(s.dat, 8, 8))
    self.assertEqual(4, max(decs2), "above the band the affine level applies")


if __name__ == "__main__":
  unittest.main()
