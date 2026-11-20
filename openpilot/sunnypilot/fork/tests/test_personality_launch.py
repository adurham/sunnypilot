"""
Personality (adurham fork): the driver's LongitudinalPersonality is carried from selfdriveState to the car controller on
``CarControlSP.personality``, where the Hyundai pedal law (launch ceiling / pull gain / interceptor cap) reads it — that
half is covered by the opendbc patch's test_gas_interceptor_personality.py.

922513f312 upstream sync: upstream deleted the ACC/"chill" cruise path together with the personality-scaled cruise
ceiling (``get_max_accel`` / ``personality_scale`` / ``PERSONALITY_CRUISE_SCALE`` / ``A_CRUISE_MAX_VALS``/``BP``), so the
cruise candidate is now ``get_cruise_accel`` with a personality-INDEPENDENT ceiling (ACCEL_MAX, jerk-limited). The tests
for the personality-scaled ceiling were deleted rather than retargeted; what is still fork behavior — and still pinned
here — is that the personality itself survives the capnp / opendbc boundary, including the "not yet seen" and
"unknown value" fail-safes.
"""
import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.car.helpers import convert_carControlSP
from opendbc.car import structs
from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt

P = log.LongitudinalPersonality
PERSONALITIES = (P.relaxed, P.standard, P.aggressive)


class TestCapnpPlumbing(OpenpilotTestCase):
  def test_capnp_field_round_trips(self):
    m = messaging.new_message('carControlSP')
    # capnp primitive default is 0 = aggressive, so the producer ALWAYS writes the field (controlsd_ext) and the opendbc
    # mirror defaults to standard; consumers clamp unknown to standard.
    self.assertEqual(int(m.carControlSP.personality), 0)
    for p in PERSONALITIES:
      m.carControlSP.personality = int(p)
      self.assertEqual(int(m.carControlSP.personality), int(p))

  def test_convert_carcontrolsp_carries_personality(self):
    m = messaging.new_message('carControlSP')
    m.carControlSP.personality = int(P.aggressive)
    sp = convert_carControlSP(m.carControlSP)
    self.assertEqual(sp.personality, int(P.aggressive))
    # opendbc dataclass default (no-arg construction, e.g. tests / other brands) is STANDARD, never aggressive
    self.assertEqual(structs.CarControlSP().personality, int(P.standard))

  def test_state_control_ext_copies_live_personality(self):
    class _SM(dict):
      def __init__(self, d, seen):
        super().__init__(d)
        self.seen = {'selfdriveState': seen}  # SubMaster.seen is a per-service dict

    def make_sm(personality, seen=True):
      sd = messaging.new_message('selfdriveState').selfdriveState
      sd.personality = personality
      rs = messaging.new_message('radarState').radarState
      return _SM({'selfdriveState': sd, 'radarState': rs,
                  'selfdriveStateSP': messaging.new_message('selfdriveStateSP').selfdriveStateSP}, seen)

    for p in PERSONALITIES:
      inst = ControlsExt.__new__(ControlsExt)  # get_lead_data is a staticmethod; no init state needed
      cc_sp = ControlsExt.state_control_ext(inst, make_sm(p))
      self.assertEqual(int(cc_sp.personality), int(p))
    # a not-yet-seen selfdriveState must NOT silently read as 0 (aggressive)
    inst = ControlsExt.__new__(ControlsExt)
    cc_sp = ControlsExt.state_control_ext(inst, make_sm(P.aggressive, seen=False))
    self.assertEqual(int(cc_sp.personality), int(P.standard))


# removed at the 922513f312 upstream sync: the personality-scaled cruise ceiling (get_max_accel / personality_scale /
# PERSONALITY_CRUISE_SCALE / A_CRUISE_MAX_VALS+BP and get_cruise_accel's personality argument) no longer exists
# upstream, so TestCruiseCeilingScale and TestEndToEndCruiseCeiling (which drove the real planner to that ceiling) are
# deleted rather than retargeted.
