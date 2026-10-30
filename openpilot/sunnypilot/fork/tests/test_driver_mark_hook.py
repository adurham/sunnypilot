"""17c FEEDBACK: the card-side driver-mark glue (fork/driver_mark_hook.py) end to end.

Drives the REAL Fca11LongBrake through a fake SubMaster + fake car interface and asserts the k="m" records that land in
a temporary FCA11 log dir - the brief's six required cases plus the pair-dedupe the UI's two-services-per-press forces.
"""
import json
import os
import shutil
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from opendbc.car import structs
from opendbc.sunnypilot.car.hyundai import driver_mark as dm
from opendbc.sunnypilot.car.hyundai import fca11_long as fl
from openpilot.sunnypilot.fork.driver_mark_hook import DriverMarkHook, law_of

REPO = Path(__file__).resolve().parents[4]
CARD = REPO / "openpilot" / "selfdrive" / "car" / "card.py"
S = 1_000_000_000


def _real_camera_frame(dec=0, alive=5):
  b = bytearray(8)
  def setb(start, length, val):
    for i in range(length):
      pos = start + i
      b[pos // 8] |= ((int(val) >> i) & 1) << (pos % 8)
  setb(0, 1, 1)
  setb(3, 2, 3)
  setb(8, 8, dec)
  setb(18, 2, 2)
  setb(23, 4, alive)
  return bytes(b)


class _CS:
  def __init__(self, cam=None):
    self.out = structs.CarState()
    self.out.vEgo = 20.0
    self.out.aEgo = 0.0
    self.out.gearShifter = structs.CarState.GearShifter.drive
    self.out.brakePressed = False
    self.out.gasPressed = False
    self.out.standstill = False
    self.interceptor_state = 0
    self.interceptor_fault_seen = False
    self.fca11_cam_frame = cam
    self.fca11_now_nanos = 0
    self.fca11_lead_closing = False
    self.fca11_wheel_min_kph = None
    self.fca11_echoes = []
    self.fca11_clutch_tq_nm = None


class _SM:
  """Just the two things the hook touches: sm.updated[...] and sm['carControl']."""

  def __init__(self):
    self.updated = dict.fromkeys(("userBookmark", "bookmarkButton", "carControl"), False)
    self._cc = SimpleNamespace(enabled=True, longActive=True)

  def __getitem__(self, name):
    if name == "carControl":
      return self._cc
    raise KeyError(name)


def _cp_sp(fca11=True):
  cp_sp = structs.CarParamsSP()
  cp_sp.fca11Brake = fca11
  cp_sp.enableGasInterceptor = True
  return cp_sp


class TestDriverMarkHook(unittest.TestCase):
  def setUp(self):
    self.dir = tempfile.mkdtemp(prefix="fca11mark-hook-")
    self._env = os.environ.get("FCA11_LOG_DIR")
    os.environ["FCA11_LOG_DIR"] = self.dir
    self.law = fl.Fca11LongBrake(structs.CarParams(), _cp_sp())
    self.CI = SimpleNamespace(CC=SimpleNamespace(fca11_brake=self.law))
    self.sm = _SM()
    self.hook = DriverMarkHook()
    self.cs = _CS(cam=(0, _real_camera_frame()))
    self.t = 0

  def tearDown(self):
    if self._env is None:
      os.environ.pop("FCA11_LOG_DIR", None)
    else:
      os.environ["FCA11_LOG_DIR"] = self._env
    shutil.rmtree(self.dir, ignore_errors=True)

  # ---- helpers ----------------------------------------------------------------------------------------------
  def _frame(self, dt_s=0.01, press=False, second_service=False, ask=0.0):
    """One card tick: optional press message(s), then the hook step. The law is ticked like controls_update would."""
    self.t += int(dt_s * S)
    frame = self.t // int(0.01 * S)
    self.law.update(_cc(ask), self.cs, frame)
    self.sm.updated["userBookmark"] = bool(press)
    self.sm.updated["bookmarkButton"] = bool(press and second_service)
    self.hook.step(self.sm, self.CI, self.cs, self.t)
    self.sm.updated["userBookmark"] = self.sm.updated["bookmarkButton"] = False
    return frame

  def _marks(self):
    p = os.path.join(self.dir, "fca11-shadow.jsonl")
    if not os.path.exists(p):
      return []
    with open(p) as f:
      return [r for r in (json.loads(line) for line in f if line.strip()) if r["k"] == "m"]

  # ---- the brief's cases ------------------------------------------------------------------------------------
  def test_one_press_is_one_good_mark(self):
    self._frame(press=True)
    self.assertEqual([], self._marks(), "the verdict is not known before the gap closes")
    for _ in range(210):
      self._frame()
    marks = self._marks()
    self.assertEqual(1, len(marks))
    self.assertEqual("good", marks[0]["verdict"])
    self.assertEqual("bookmark_single", marks[0]["source"])
    self.assertEqual(1, self.hook.presses)

  def test_two_presses_one_second_apart_are_both_bad(self):
    self._frame(press=True)
    for _ in range(99):
      self._frame()
    self._frame(press=True)          # +1.00 s
    for _ in range(210):
      self._frame()
    marks = self._marks()
    self.assertEqual(2, len(marks), "every press of the burst is recorded")
    self.assertEqual(["bad", "bad"], [m["verdict"] for m in marks])
    self.assertEqual(["bookmark_double", "bookmark_double"], [m["source"] for m in marks])
    self.assertEqual(2, self.hook.presses)

  def test_two_presses_five_seconds_apart_are_two_goods(self):
    self._frame(press=True)
    for _ in range(500):
      self._frame()
    self._frame(press=True)
    for _ in range(210):
      self._frame()
    marks = self._marks()
    self.assertEqual(2, len(marks))
    self.assertEqual(["good", "good"], [m["verdict"] for m in marks])
    self.assertEqual(["bookmark_single", "bookmark_single"], [m["source"] for m in marks])

  def test_the_ui_pair_is_one_press(self):
    self._frame(press=True, second_service=True)     # userBookmark + bookmarkButton, same tick
    for _ in range(210):
      self._frame()
    self.assertEqual(1, self.hook.presses)
    self.assertEqual(["good"], [m["verdict"] for m in self._marks()])

  def test_the_ui_pair_across_two_ticks_is_still_one_press(self):
    self._frame(press=True)                          # bookmarkButton lands on the NEXT tick
    self._frame(press=True)
    for _ in range(210):
      self._frame()
    self.assertEqual(1, self.hook.presses, "a pair straddling a card tick must not read as a double press")
    self.assertEqual(["good"], [m["verdict"] for m in self._marks()])

  def test_snapshot_is_frozen_at_the_press_not_at_the_verdict(self):
    f_press = self._frame(press=True, ask=-1.7)
    for _ in range(210):
      self._frame(ask=0.0)         # the planner has fully released 2 s later
    m = self._marks()[0]
    self.assertEqual(f_press, m["frame"], "the record must describe the PRESS frame")
    self.assertAlmostEqual(-1.7, m["ask_mps2"], places=4)

  def test_no_press_anywhere_no_mark_at_all(self):
    for _ in range(400):
      self._frame()
    self.assertEqual([], self._marks(), "negative control: feature not triggered -> no k=m record")
    self.assertEqual(0, self.hook.presses)
    self.assertEqual(dm.MARK_CODE_NONE, self.law.mark_alert_code(self.t // int(0.01 * S)))

  def test_confirmation_arms_for_2s_from_the_mark(self):
    self._frame(press=True)
    for _ in range(210):
      self._frame()
    until = self.law._mark_alert_until
    closed = until - dm.MARK_ALERT_FRAMES
    self.assertEqual(dm.MARK_CODE_GOOD, self.law.mark_alert_code(closed), "the confirmation is up on the frame the mark lands")
    self.assertEqual(dm.MARK_CODE_GOOD, self.law.mark_alert_code(until - 1))
    self.assertEqual(dm.MARK_CODE_NONE, self.law.mark_alert_code(until), "2 s, then silent")
    self.assertEqual(200, dm.MARK_ALERT_FRAMES, "2 s at the 100 Hz controller rate")

  def test_no_law_no_mark(self):
    hook = DriverMarkHook()
    self.sm.updated["userBookmark"] = True
    hook.step(self.sm, SimpleNamespace(CC=SimpleNamespace()), self.cs, 5)
    hook.step(self.sm, SimpleNamespace(), self.cs, int(3 * S))
    self.assertEqual(0, hook.presses)
    self.assertIsNone(law_of(SimpleNamespace()))


def _cc(accel=0.0):
  cc = structs.CarControl()
  cc.enabled = True
  cc.longActive = True
  cc.actuators.accel = accel
  return cc


class TestWiring(unittest.TestCase):
  def test_card_subscribes_both_services_and_steps_the_hook(self):
    src = CARD.read_text()
    self.assertIn("'userBookmark', 'bookmarkButton'", src)
    self.assertIn("DriverMarkHook()", src)
    self.assertRegex(src, r"self\.driver_mark_hook\.step\(self\.sm, self\.CI, CS, now_nanos\)")


class TestCarSpecificRaises(unittest.TestCase):
  def test_driver_mark_codes_raise_the_confirmation_alerts(self):
    from openpilot.cereal import custom
    from openpilot.selfdrive.selfdrived.events import Events
    from openpilot.sunnypilot.selfdrive.car.car_specific import CarSpecificEventsSP
    EventNameSP = custom.OnroadEventSP.EventName

    CP = structs.CarParams()
    CP.brand = 'hyundai'
    CP_SP = structs.CarParamsSP()
    CP_SP.fca11Brake = True
    CP_SP.enableGasInterceptor = True
    cse = CarSpecificEventsSP(CP, CP_SP)
    CS = structs.CarState()

    def names(code):
      CS_SP = structs.CarStateSP()
      CS_SP.driverMark = code
      return cse.update(CS, CS_SP, Events(), True)

    def has(ev, name):
      return bool(ev.has(name))

    self.assertTrue(has(names(1), EventNameSP.driverMarkGood))
    self.assertTrue(has(names(2), EventNameSP.driverMarkBad))
    self.assertFalse(has(names(0), EventNameSP.driverMarkGood))
    self.assertFalse(has(names(0), EventNameSP.driverMarkBad))

  def test_the_confirmations_do_not_reuse_a_fca11_safety_ordinal(self):
    from openpilot.cereal import custom
    EventNameSP = custom.OnroadEventSP.EventName
    used = {EventNameSP.fca11Unavailable, EventNameSP.fca11SuperviseStop, EventNameSP.fca11StopComplete,
            EventNameSP.fca11BrakeNow, EventNameSP.fca11CameraBraking, EventNameSP.fca11Divergence}
    self.assertNotIn(EventNameSP.driverMarkGood, used)
    self.assertNotIn(EventNameSP.driverMarkBad, used)
    self.assertNotEqual(EventNameSP.driverMarkGood, EventNameSP.driverMarkBad)


if __name__ == "__main__":
  unittest.main()
