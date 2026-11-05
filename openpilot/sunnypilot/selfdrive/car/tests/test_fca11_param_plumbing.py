"""
fork: FCA11-long param plumbing, END TO END through the REAL path card.py takes.

card.py:113 does `sunnypilot_interfaces.initialize_params(self.params)` and hands the result to
opendbc's `get_car(...)` -> `setup_interfaces(...)`, which is the ONLY place `CP_SP.fca11Brake` and
panda safety bit 256 (`FCA11_LONG`) are set. The pre-existing car-layer tests inject
{'HyundaiFca11Brake': ...} straight into `params_list`, so they never exercise `initialize_params()`
and missed the bug where the key was simply not read there (feature came up OFF on route 149
even though the UI/param said ON).

These tests drive a real `Params` object, the real `initialize_params()`, and the real
`setup_interfaces()`, and assert:
  * the key reaches opendbc exactly when the stored value is 1 (and, defensively, at all);
  * CP_SP.fca11Brake AND the safetyParam FCA11_LONG bit are set for '1', and clear for '0'/absent;
  * a raw string '1' (libparams form) still arms it (the bool/str type tolerance);
  * nothing else about the pedal-only baseline changes when the toggle is off.
0044 adds TestFca11ShapingParamPlumbingE2E: the same real path for HyundaiFca11AffineGain (the brake-shaping law), four
cases {pedal on/off} x {shaping on/off} incl. a behavioural difference of the controller.
"""
import os
import tempfile
import unittest

# Isolated params store. PARAMS_ROOT is honored by libparams at Params() construction, so this must
# be set before any Params() is built in this process (openpilot's common/params fallback does not).
os.environ["PARAMS_ROOT"] = tempfile.mkdtemp(prefix="fca11-plumbing-test-")

from types import SimpleNamespace

from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.car import interfaces as sp_interfaces
from opendbc.car import structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.hyundai.values import CAR
from opendbc.sunnypilot.car.hyundai import fca11_long as fl
from opendbc.sunnypilot.car.hyundai.values import HyundaiSafetyFlagsSP
from opendbc.sunnypilot.car.interfaces import setup_interfaces

CAR_UNDER_TEST = CAR.HYUNDAI_ELANTRA_2022_NON_SCC
# standard pedal sensor 0x201 present so the gas-interceptor capability flag is set (FCA11 rides on it)
BASE_FINGERPRINT = {0x260: 8, 0x371: 8, 0x386: 8, 0x394: 8, 0x251: 8, 0x4F1: 4, 0x340: 8}
STD = {0x201: 6}
FCA11_BIT = HyundaiSafetyFlagsSP.FCA11_LONG
GAS_BIT = HyundaiSafetyFlagsSP.GAS_INTERCEPTOR


def _run_real_path(fca11, shaping=None):
  """Real Params -> initialize_params() -> setup_interfaces(). fca11 / shaping are True / False / None (absent).
  0044: `shaping` is the HyundaiFca11AffineGain key (the brake-shaping law), written through the REAL compiled
  registry - put_bool raises UnknownKeyName if the key is not in params_keys.h (the pre-0044 defect)."""
  params = Params()
  params.remove("HyundaiFca11Brake")
  params.remove("HyundaiGasInterceptor")
  params.remove("HyundaiFca11AffineGain")
  params.put_bool("HyundaiGasInterceptor", True, block=True)
  if fca11 is not None:
    params.put_bool("HyundaiFca11Brake", fca11, block=True)
  if shaping is not None:
    params.put_bool("HyundaiFca11AffineGain", shaping, block=True)

  init_params_list_sp = sp_interfaces.initialize_params(params)

  fingerprint = {i: {} for i in range(8)}
  fingerprint[0] = {**BASE_FINGERPRINT, **STD}
  CarInterface = interfaces[CAR_UNDER_TEST]
  CP = CarInterface.get_params(CAR_UNDER_TEST, fingerprint, [], alpha_long=False, is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, CAR_UNDER_TEST, fingerprint, [], alpha_long=False, is_release_sp=False, docs=False)
  setup_interfaces(CarInterface, CP, CP_SP, init_params_list_sp)
  return CP, CP_SP, init_params_list_sp


def _soft_ask_dec_series(CP, CP_SP, accel=-0.8, kph=50.0, frames=200):
  """BEHAVIOUR probe: an Fca11LongBrake built from the production CP/CP_SP, fed a fixed soft ask while moving.
  Returns (affine_gain, [sent CR_VSM_DecCmd...]). FCA11_AFFINE must be unset (0044 refuses it)."""
  CS = SimpleNamespace(out=structs.CarState(), interceptor_state=0, interceptor_fault_seen=False,
                       fca11_cam_frame=(0, bytes(8)), fca11_now_nanos=0, fca11_wheel_min_kph=None)
  CS.out.gearShifter = structs.CarState.GearShifter.drive
  CS.out.vEgo = kph / 3.6
  CC = structs.CarControl()
  CC.enabled = CC.longActive = True
  CC.actuators.accel = accel
  b = fl.Fca11LongBrake(CP, CP_SP)
  decs = []
  for f in range(frames):
    CS.fca11_cam_frame = (f * 10_000_000, bytes(8))
    CS.fca11_now_nanos = f * 10_000_000
    decs += [fl._get_bits(s.dat, 8, 8) for s in b.update(CC, CS, f)]
  return b.affine_gain, decs


def _run_with_pedal(pedal, fca11, shaping):
  """As _run_real_path but the pedal param is driven too (pedal False => the whole FCA11 stack must stay off)."""
  params = Params()
  for k in ("HyundaiGasInterceptor", "HyundaiFca11Brake", "HyundaiFca11AffineGain"):
    params.remove(k)
  params.put_bool("HyundaiGasInterceptor", pedal, block=True)
  params.put_bool("HyundaiFca11Brake", fca11, block=True)
  params.put_bool("HyundaiFca11AffineGain", shaping, block=True)
  plist = sp_interfaces.initialize_params(params)
  fingerprint = {i: {} for i in range(8)}
  fingerprint[0] = {**BASE_FINGERPRINT, **STD}
  CarInterface = interfaces[CAR_UNDER_TEST]
  CP = CarInterface.get_params(CAR_UNDER_TEST, fingerprint, [], alpha_long=False, is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, CAR_UNDER_TEST, fingerprint, [], alpha_long=False, is_release_sp=False, docs=False)
  setup_interfaces(CarInterface, CP, CP_SP, plist)
  return CP, CP_SP, plist


class TestFca11ShapingParamPlumbingE2E(OpenpilotTestCase):
  """0044 (Fable Ruling 2 (3)1): the brake-SHAPING param end to end. Before 0044 `HyundaiFca11AffineGain` was not in
  params_keys.h (put_bool raised UnknownKeyName on the compiled registry), initialize_params never read it, and nothing
  wrote CP_SP.fca11AffineGain - the law was UNREACHABLE on the car. Four cases {pedal on/off} x {shaping on/off} with
  FCA11 Braking ON, each through REAL Params.put_bool -> initialize_params -> setup_interfaces, asserting the CP_SP
  fields, the safetyParam bit, the car-layer dependency, and a BEHAVIOURAL difference of the controller."""

  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    os.environ.pop("FCA11_AFFINE", None)

  def test_four_cases_pedal_x_shaping(self):
    """The four cases {FCA11 Braking ("pedal" in Fable's wording) on/off} x {shaping on/off}, gas-interceptor pedal ON,
    plus the same four with the gas-interceptor pedal OFF (2x2x2 = 8). FCA11 braking requires the gas-interceptor
    pedal; shaping requires FCA11 braking (car-layer dependency)."""
    seen = {}
    for pedal in (True, False):
      for brake in (True, False):
        for shaping in (True, False):
          CP, CP_SP, plist = _run_with_pedal(pedal, brake, shaping)
          ctx = {"gas_pedal": pedal, "fca11_brake": brake, "shaping": shaping}
          self.assertTrue(any("HyundaiFca11AffineGain" in d for d in plist), f"initialize_params() must read the key {ctx}")
          armed = pedal and brake
          self.assertEqual(armed, CP_SP.fca11Brake, ctx)
          self.assertEqual(armed, bool(CP_SP.safetyParam & FCA11_BIT), ctx)            # bit 256 rides on fca11Brake only
          self.assertEqual(armed and shaping, CP_SP.fca11AffineGain, ctx)              # the dependency (inert otherwise)
          affine, decs = _soft_ask_dec_series(CP, CP_SP)
          self.assertEqual(armed and shaping, affine, ctx)
          if armed:
            self.assertTrue(decs, f"FCA11 braking is armed: the soft ask must actuate {ctx}")
          else:
            self.assertEqual([], decs, f"FCA11 braking not armed: the controller is inert {ctx}")
          seen[(pedal, brake, shaping)] = decs
    # BEHAVIOUR: armed, shaping ON emits a DIFFERENT dec_cmd sequence than OFF for the same fixed soft ask (-0.8 moving)
    on, off = seen[(True, True, True)], seen[(True, True, False)]
    self.assertNotEqual(on, off, "the shaping param must change the controller's output")
    self.assertEqual({4}, set(on[:fl.FCA11_SHAPE_ONSET_HOLD_SENT]), "shaped law: the onset hold at the 4 LSB floor")
    self.assertEqual(fl.dec_cmd_from_accel(-0.8), max(off), "OFF = the deployed 0.67 law level")
    self.assertGreater(max(off), max(on))
    # shaping is inert whenever braking is not armed: ON and OFF identical (and empty)
    for pedal, brake in ((True, False), (False, True), (False, False)):
      self.assertEqual(seen[(pedal, brake, True)], seen[(pedal, brake, False)], (pedal, brake))

  def test_persisted_shaping_on_with_braking_off_is_inert(self):
    """The dependency is enforced in the CAR LAYER: a persisted ON (e.g. restored from a backup) with FCA11 Braking OFF
    must not set the field nor change the controller."""
    CP, CP_SP, _ = _run_with_pedal(True, False, True)
    self.assertFalse(CP_SP.fca11Brake)
    self.assertFalse(CP_SP.fca11AffineGain)
    self.assertFalse(CP_SP.safetyParam & FCA11_BIT)
    affine, decs = _soft_ask_dec_series(CP, CP_SP)
    self.assertFalse(affine)
    self.assertEqual([], decs)

  def test_shaping_key_absent_is_off(self):
    _, CP_SP, plist = _run_real_path(True, None)
    self.assertTrue(CP_SP.fca11Brake)
    self.assertFalse(CP_SP.fca11AffineGain)
    self.assertTrue(any("HyundaiFca11AffineGain" in d for d in plist))

  def test_key_is_registered_in_the_compiled_registry(self):
    """The exact pre-0044 failure: the compiled libparams must know the key (check_key / put_bool / get_bool)."""
    params = Params()
    params.check_key("HyundaiFca11AffineGain")
    params.put_bool("HyundaiFca11AffineGain", True, block=True)
    self.assertTrue(params.get_bool("HyundaiFca11AffineGain"))
    params.remove("HyundaiFca11AffineGain")
    self.assertFalse(params.get("HyundaiFca11AffineGain", return_default=True), "default must be OFF")


class TestFca11ParamPlumbingE2E(OpenpilotTestCase):

  def test_initialize_params_reads_the_key(self):
    # the plumbing bug: initialize_params() must emit the key at all (it did not before this fix)
    for setting in (True, False, None):
      _, _, plist = _run_real_path(setting)
      self.assertTrue(any("HyundaiFca11Brake" in d for d in plist),
                      f"initialize_params() did not read HyundaiFca11Brake (setting={setting})")

  def test_toggle_on_arms_through_the_real_path(self):
    _, CP_SP, _ = _run_real_path(True)
    self.assertTrue(CP_SP.fca11Brake, "CP_SP.fca11Brake must be True when the stored param is 1")
    self.assertTrue(CP_SP.safetyParam & FCA11_BIT, "safetyParam FCA11_LONG bit (256) must be set")
    # the pedal it rides on is armed too
    self.assertTrue(CP_SP.safetyParam & GAS_BIT)

  def test_toggle_off_is_inert_through_the_real_path(self):
    _, CP_SP, _ = _run_real_path(False)
    self.assertFalse(CP_SP.fca11Brake)
    self.assertFalse(CP_SP.safetyParam & FCA11_BIT)

  def test_key_absent_is_inert_through_the_real_path(self):
    # stale libparams / fresh device: the key is unreadable -> default False -> feature off, no raise
    _, CP_SP, _ = _run_real_path(None)
    self.assertFalse(CP_SP.fca11Brake)
    self.assertFalse(CP_SP.safetyParam & FCA11_BIT)

  def test_on_matches_pedal_only_baseline_plus_bit256(self):
    _, CP_SP_off, _ = _run_real_path(False)
    _, CP_SP_on, _ = _run_real_path(True)
    # turning it on changes exactly fca11Brake + bit 256, nothing else in safetyParam
    self.assertEqual(CP_SP_off.safetyParam | FCA11_BIT, CP_SP_on.safetyParam)
    self.assertEqual(CP_SP_off.safetyParam & FCA11_BIT, 0)

  def test_raw_string_one_still_arms(self):
    # guards the str(...) == "1" clause independently of the bool path (libparams C-string form)
    fingerprint = {i: {} for i in range(8)}
    fingerprint[0] = {**BASE_FINGERPRINT, **STD}
    CarInterface = interfaces[CAR_UNDER_TEST]
    CP = CarInterface.get_params(CAR_UNDER_TEST, fingerprint, [], alpha_long=False, is_release=False, docs=False)
    CP_SP = CarInterface.get_params_sp(CP, CAR_UNDER_TEST, fingerprint, [], alpha_long=False, is_release_sp=False, docs=False)
    setup_interfaces(CarInterface, CP, CP_SP, [{"HyundaiGasInterceptor": True}, {"HyundaiFca11Brake": "1"}])
    self.assertTrue(CP_SP.fca11Brake)
    self.assertTrue(CP_SP.safetyParam & FCA11_BIT)


def _run_shadow_path(brake, shaping, shadow):
  """0045: real Params (compiled registry) -> initialize_params() -> setup_interfaces() with HyundaiFca11Shadow."""
  params = Params()
  for k in ("HyundaiGasInterceptor", "HyundaiFca11Brake", "HyundaiFca11AffineGain", "HyundaiFca11Shadow"):
    params.remove(k)
  params.put_bool("HyundaiGasInterceptor", True, block=True)
  for k, v in (("HyundaiFca11Brake", brake), ("HyundaiFca11AffineGain", shaping), ("HyundaiFca11Shadow", shadow)):
    if v is not None:
      params.put_bool(k, v, block=True)
  plist = sp_interfaces.initialize_params(params)
  fingerprint = {i: {} for i in range(8)}
  fingerprint[0] = {**BASE_FINGERPRINT, **STD}
  CarInterface = interfaces[CAR_UNDER_TEST]
  CP = CarInterface.get_params(CAR_UNDER_TEST, fingerprint, [], alpha_long=False, is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, CAR_UNDER_TEST, fingerprint, [], alpha_long=False, is_release_sp=False, docs=False)
  setup_interfaces(CarInterface, CP, CP_SP, plist)
  return CP, CP_SP, plist


class TestFca11ShadowParamPlumbingE2E(OpenpilotTestCase):
  """0045 v2 (Amendment 1): HyundaiFca11Shadow end to end through the REAL compiled registry. Truth table: Shadow ON =>
  the wire stays OLD-LAW: fca11Brake / bit 256 follow HyundaiFca11Brake exactly as the deployed build, fca11AffineGain is
  FORCED False (V3 never actuates); Shadow OFF => unchanged freeze behaviour."""

  def test_key_is_registered_in_the_compiled_registry(self):
    params = Params()
    params.check_key("HyundaiFca11Shadow")
    params.put_bool("HyundaiFca11Shadow", True, block=True)
    self.assertTrue(params.get_bool("HyundaiFca11Shadow"))
    params.remove("HyundaiFca11Shadow")
    self.assertFalse(params.get("HyundaiFca11Shadow", return_default=True), "default must be OFF")

  def test_initialize_params_reads_the_key(self):
    for setting in (True, False, None):
      _, _, plist = _run_shadow_path(True, True, setting)
      self.assertTrue(any("HyundaiFca11Shadow" in d for d in plist), setting)

  def test_truth_table(self):
    for brake in (True, False, None):
      for shaping in (True, False, None):
        for shadow in (True, False, None):
          ctx = dict(brake=brake, shaping=shaping, shadow=shadow)
          _, CP_SP, _ = _run_shadow_path(brake, shaping, shadow)
          self.assertEqual(bool(shadow), CP_SP.fca11Shadow, ctx)
          self.assertEqual(bool(brake), CP_SP.fca11Brake, ctx)                       # OLD-LAW armed exactly as today
          self.assertEqual(bool(brake), bool(CP_SP.safetyParam & FCA11_BIT), ctx)
          self.assertEqual(bool(brake and shaping and not shadow), CP_SP.fca11AffineGain, ctx)   # V3 never on the wire

  def test_shadow_safety_config_equals_deployed_oldlaw(self):
    for brake in (True, False):
      CP_s, CP_SP_s, _ = _run_shadow_path(brake, True, True)
      CP_o, CP_SP_o, _ = _run_shadow_path(brake, False, None)
      self.assertEqual(CP_SP_o.safetyParam, CP_SP_s.safetyParam)
      self.assertEqual(CP_o.safetyConfigs[-1].safetyParam, CP_s.safetyConfigs[-1].safetyParam)

  def test_shadow_module_wire_is_oldlaw_and_v3_computes(self):
    """the module built from the real-registry CP_SP: Shadow + Brake + Shaping persisted ON => the actuating instance is
    OLD-LAW (affine_gain False) and its frames equal a Shadow-OFF OLD-LAW instance's; V3 computes post-send."""
    CP, CP_SP, _ = _run_shadow_path(True, True, True)
    CPo, CP_SPo, _ = _run_shadow_path(True, False, None)
    os.environ["FCA11_LOG_DIR"] = "0"
    try:
      b = fl.Fca11LongBrake(CP, CP_SP)
      bo = fl.Fca11LongBrake(CPo, CP_SPo)
      self.assertTrue(b.shadow and b.enabled and not b.affine_gain and b._shadow_v3.affine_gain)
      CS = SimpleNamespace(out=structs.CarState(), interceptor_state=0, interceptor_fault_seen=False,
                           fca11_cam_frame=(0, bytes(8)), fca11_now_nanos=0, fca11_wheel_min_kph=None)
      CS.out.gearShifter = structs.CarState.GearShifter.drive
      CS.out.vEgo = 50 / 3.6
      CC = structs.CarControl()
      CC.enabled = CC.longActive = True
      CC.actuators.accel = -1.5
      wb, n_sent = [], 0
      for f in range(200):
        CS.fca11_cam_frame = (f * 10_000_000, bytes(8))
        CS.fca11_now_nanos = f * 10_000_000
        s1 = b.update(CC, CS, f)
        s0 = bo.update(CC, CS, f)
        self.assertEqual([bytes(x.dat) for x in s0], [bytes(x.dat) for x in s1], f)
        n_sent += len(s1)
        b.v3_post_send()
        self.assertTrue(b.shadow_out.non_actuating)
        wb.append(b.shadow_out.v3_wouldbe_lsb)
      self.assertGreater(n_sent, 50)
      self.assertGreater(max(wb), 0)
    finally:
      os.environ.pop("FCA11_LOG_DIR", None)


def _run_stop_hold_path(stop_hold):
  """Real Params -> initialize_params() -> get_params/get_params_sp -> setup_interfaces(), FCA11 braking ON and the gas
  interceptor pedal ON, with HyundaiFca11StopHoldClutch = stop_hold (True / False / None = absent)."""
  params = Params()
  for k in ("HyundaiGasInterceptor", "HyundaiFca11Brake", "HyundaiFca11StopHoldClutch"):
    params.remove(k)
  params.put_bool("HyundaiGasInterceptor", True, block=True)
  params.put_bool("HyundaiFca11Brake", True, block=True)
  if stop_hold is not None:
    params.put_bool("HyundaiFca11StopHoldClutch", stop_hold, block=True)
  plist = sp_interfaces.initialize_params(params)
  fingerprint = {i: {} for i in range(8)}
  fingerprint[0] = {**BASE_FINGERPRINT, **STD}
  CarInterface = interfaces[CAR_UNDER_TEST]
  CP = CarInterface.get_params(CAR_UNDER_TEST, fingerprint, [], alpha_long=False, is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, CAR_UNDER_TEST, fingerprint, [], alpha_long=False, is_release_sp=False, docs=False)
  setup_interfaces(CarInterface, CP, CP_SP, plist)
  return CP, CP_SP, plist


def _pair_bits(CP, CP_SP, frames=1200):
  """Drive the REAL law from that CP/CP_SP through a full stop and a 6 s hold at the ROUTE 195 logged condition, and
  return [(frame, level, hba, stopreq)] for every 0x38D it sends.

  The scenario's hold publishes exactly what carstate.py published on drive 195 (vEgo 0.015..0.066 m/s, standstill True,
  fastest wheel 0.00..0.31 km/h, TCS13 FCA_ACK = 1, TCS13 StandStill = 1) - the frames the pair was missing from."""
  os.environ["FCA11_LOG_DIR"] = "0"
  try:
    b = fl.Fca11LongBrake(CP, CP_SP)
    CS = SimpleNamespace(out=structs.CarState(), interceptor_state=0, interceptor_fault_seen=False,
                         fca11_cam_frame=(0, bytes(8)), fca11_now_nanos=0, fca11_wheel_min_kph=0.0,
                         fca11_wheel_max_kph=0.0, fca11_esc_ack=True, fca11_esc_standstill=True)
    CS.out.gearShifter = structs.CarState.GearShifter.drive
    CC = structs.CarControl()
    CC.enabled = CC.longActive = True
    out = []
    for f in range(frames):
      if f < 50:
        ask, v, ss = -0.2, 20.0 / 3.6, False
      elif f < 350:
        v = max(0.0, 20.0 / 3.6 * (1.0 - (f - 50) / 300.0))
        ask, ss = -2.0, v == 0.0
      elif f < 950:
        ask, v, ss = -2.0, 0.03, True
      else:
        ask, v, ss = 0.5, min(3.0, (f - 950) * 0.02), False
      CS.out.vEgo = v
      CS.out.standstill = ss
      CS.out.aEgo = -1.5 if ask < 0 else 0.3
      CS.fca11_wheel_max_kph = 0.1 if ss else v * 3.6
      CS.fca11_wheel_min_kph = 0.03 if ss else v * 3.6
      CS.fca11_cam_frame = (f * 10_000_000, bytes(8))
      CS.fca11_now_nanos = f * 10_000_000
      CC.actuators.accel = ask
      for s in b.update(CC, CS, f):
        if s.address == fl.FCA11_ADDR:
          d = bytes(s.dat)
          out.append((f, fl._get_bits(d, 8, 8), fl._get_bits(d, 1, 2), fl._get_bits(d, 21, 1)))
    return out
  finally:
    os.environ.pop("FCA11_LOG_DIR", None)


class TestFca11StopHoldParamPlumbingE2E(OpenpilotTestCase):
  """0034's stop-hold clutch lane end to end through the REAL compiled registry and the REAL card.py path.

  THE DEFECT THIS PINS (route 195, 2026-11-05): `HyundaiFca11StopHoldClutch` was registered in params_keys.h and the UI
  wrote it, opendbc's setup_interfaces read it, and Fca11LongBrake._stop_hold_ok checked every input - but
  initialize_params() never put the key in the list that feeds setup_interfaces, so CP_SP.fca11StopHoldClutch stayed
  False, `_stop_hold_ok` returned False on its FIRST clause (before it looked at a single input) and the pair was sent
  on 0 of 399 level-30 frames while vEgo, standstill, fastest wheel, TCS13 FCA_ACK and StandStill were all satisfied.
  The pre-0039 plumbing tests injected {'HyundaiFca11StopHoldClutch': ...} straight into setup_interfaces, so they
  exercised the writer and never the feed - the same shape as the 0044 HyundaiFca11AffineGain bug."""

  def test_key_is_registered_and_defaults_off(self):
    params = Params()
    params.check_key("HyundaiFca11StopHoldClutch")
    params.remove("HyundaiFca11StopHoldClutch")
    self.assertFalse(params.get("HyundaiFca11StopHoldClutch", return_default=True), "default must be OFF")

  def test_initialize_params_reads_the_key(self):
    for setting in (True, False, None):
      _, _, plist = _run_stop_hold_path(setting)
      vals = [d["HyundaiFca11StopHoldClutch"] for d in plist if "HyundaiFca11StopHoldClutch" in d]
      self.assertEqual(1, len(vals), f"initialize_params must feed the key exactly once (setting={setting})")
      if setting is None:
        self.assertFalse(vals[0])
      else:
        self.assertEqual(setting, bool(vals[0]))

  def test_cp_sp_field_follows_the_param(self):
    CP, CP_SP, _ = _run_stop_hold_path(True)
    self.assertTrue(CP_SP.fca11Brake, "precondition: FCA11 braking is armed")
    self.assertTrue(CP_SP.fca11StopHoldClutch, "toggle ON must reach CP_SP through initialize_params")
    for setting in (False, None):
      _, CP_SP, _ = _run_stop_hold_path(setting)
      self.assertFalse(CP_SP.fca11StopHoldClutch, f"setting={setting} must leave the lane OFF")

  def test_pair_on_the_wire_at_a_standstill_hold(self):
    """The behaviour the owner asked for: the actuating frame at a standstill hold carries CF_VSM_HBACmd = 1 and
    FCA_StopReq = 1 (at a level >= 7 LSB), and with the toggle OFF the pair never appears."""
    CP, CP_SP, _ = _run_stop_hold_path(True)
    sent = _pair_bits(CP, CP_SP)
    holds = [(f, lvl) for f, lvl, hba, sr in sent if (hba, sr) == (1, 1)]
    self.assertTrue(holds, "the pair must ride the standstill hold frames")
    self.assertTrue(all(lvl >= fl.FCA11_STOP_HOLD_MIN_LSB for _f, lvl in holds), "the pair never rides below the hold level")
    self.assertFalse([lvl for _f, lvl, hba, sr in sent if lvl < fl.FCA11_STOP_HOLD_MIN_LSB and (hba, sr) == (1, 1)])

    CPo, CP_SPo, _ = _run_stop_hold_path(False)
    self.assertFalse([x for x in _pair_bits(CPo, CP_SPo) if x[2:] == (1, 1)], "OFF: byte-for-byte today's frames")

  def test_route_195_reproduction_without_the_feed(self):
    """Reproduce the field failure from the fixed tree: hand setup_interfaces the list an UNFIXED initialize_params
    produced (identical except the key is absent). CP_SP stays False, the law sends 0 of the same frames with the pair
    even though every input the gate reads is satisfied - the exact 0-of-399 signature of drive 195."""
    _, _, plist = _run_stop_hold_path(True)
    unfixed = [d for d in plist if "HyundaiFca11StopHoldClutch" not in d]
    fingerprint = {i: {} for i in range(8)}
    fingerprint[0] = {**BASE_FINGERPRINT, **STD}
    CarInterface = interfaces[CAR_UNDER_TEST]
    CP = CarInterface.get_params(CAR_UNDER_TEST, fingerprint, [], alpha_long=False, is_release=False, docs=False)
    CP_SP = CarInterface.get_params_sp(CP, CAR_UNDER_TEST, fingerprint, [], alpha_long=False, is_release_sp=False, docs=False)
    setup_interfaces(CarInterface, CP, CP_SP, unfixed)
    self.assertTrue(CP_SP.fca11Brake)
    self.assertFalse(CP_SP.fca11StopHoldClutch, "an absent key leaves the lane OFF (the 0034-0039 field state)")
    b = fl.Fca11LongBrake(CP, CP_SP)
    self.assertFalse(b.stop_hold_clutch)
    sent = _pair_bits(CP, CP_SP)
    self.assertTrue(sent, "precondition: the law is braking and holding")
    self.assertFalse([x for x in sent if x[2:] == (1, 1)], "0 pair frames - the route-195 signature")


if __name__ == "__main__":
    unittest.main()
