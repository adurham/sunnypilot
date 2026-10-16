"""
0045 v2 (settlement Amendment 1): FCA11 SHADOW mode, opendbc side (HyundaiFca11Shadow -> CP_SP.fca11Shadow).

Terms: OLD-LAW = the currently deployed law (on the wire in Shadow, exactly as today); V3 = the shaped candidate law
(computed + logged post-send, NEVER on the wire); FCA11-DISABLED = our stack fully off (debug-only transform, never
reachable through the Shadow flag).

Pins:
  * truthy parser + the Shadow config in setup_interfaces: fca11Brake / bit 256 exactly as the deployed build for the same
    HyundaiFca11Brake, fca11AffineGain FORCED False (truth table over Brake x AffineGain);
  * WIRE = OLD-LAW: through the REAL CarInterface.apply path the Shadow stream is byte-identical to the deployed-semantics
    (Shadow OFF, shaping OFF) stream for the same Brake value, and differs from a V3-actuating stream (the check is live);
  * V3 computes (would-be == an actuating V3 instance's sent frames, frame for frame) regardless of Brake / AffineGain;
  * V3-FAULT ISOLATION: V3 raising mid-episode / overrunning leaves the OLD-LAW stream unchanged and writes k="v3fail";
    V3 runs only in v3_post_send (update() never steps it);
  * lead fields: vRel forwarded by carcontroller.py; logged with the pinned no-lead (null) handling;
  * env ban; writer format (schema 2); FCA11-DISABLED debug mode (named, never produced by setup_interfaces).
"""
import json
import os

import pytest

from opendbc.car import structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.hyundai.values import CAR
from opendbc.sunnypilot.car import interfaces as sp_ifaces
from opendbc.sunnypilot.car.hyundai import fca11_long as fl
from opendbc.sunnypilot.car.hyundai import fca11_shadow_log as shlog
from opendbc.sunnypilot.car.hyundai import gas_interceptor as gi
from opendbc.sunnypilot.car.hyundai.values import HyundaiSafetyFlagsSP
from opendbc.sunnypilot.car.interfaces import setup_interfaces

CAR_UNDER_TEST = CAR.HYUNDAI_ELANTRA_2022_NON_SCC
BASE_FINGERPRINT = {0x260: 8, 0x371: 8, 0x386: 8, 0x394: 8, 0x251: 8, 0x4F1: 4, 0x340: 8}
STD = {0x201: 6}
TRUTHY_TABLE = [
  (True, True), ("1", True), (b"1", True), (" 1 ", True), ("1\n", True),
  (False, False), ("0", False), (b"0", False), (None, False), ("", False), (b"", False),
  ("true", False), ("True", False), ("yes", False), ("on", False), ("2", False), ("01", False), ("junk", False),
]


@pytest.fixture(autouse=True)
def _clean_env(monkeypatch, tmp_path):
  for k in ("FCA11_SHADOW", "FCA11_AFFINE", "FCA11_CAL_DIR"):
    monkeypatch.delenv(k, raising=False)
  monkeypatch.setenv("FCA11_CAL_DIR", "0")
  monkeypatch.setenv("FCA11_LOG_DIR", str(tmp_path / "fca11-log"))


def _cp(brake=None, shaping=None, shadow=None, pedal=True):
  fingerprint = {i: {} for i in range(8)}
  fingerprint[0] = {**BASE_FINGERPRINT, **STD}
  CarInterface = interfaces[CAR_UNDER_TEST]
  CP = CarInterface.get_params(CAR_UNDER_TEST, fingerprint, [], alpha_long=False, is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, CAR_UNDER_TEST, fingerprint, [], alpha_long=False, is_release_sp=False, docs=False)
  pl = [{"HyundaiGasInterceptor": pedal}]
  for k, v in (("HyundaiFca11Brake", brake), ("HyundaiFca11AffineGain", shaping), ("HyundaiFca11Shadow", shadow)):
    if v is not None:
      pl.append({k: v})
  setup_interfaces(CarInterface, CP, CP_SP, pl)
  return CarInterface, CP, CP_SP


def _cam(alive=5, warn=0, dec=0):
  from opendbc.car.hyundai.hyundaican import hyundai_checksum
  b = bytearray(8)

  def setb(start, length, val):
    for i in range(length):
      pos = start + i
      b[pos // 8] |= ((int(val) >> i) & 1) << (pos % 8)
  setb(3, 2, warn)
  setb(8, 8, dec)
  setb(18, 2, 2)
  setb(22, 3, 1)
  setb(35, 4, alive)
  setb(39, 9, 255)
  setb(48, 8, 254)
  b[4] |= 0x80
  b[7] = hyundai_checksum(bytes(b[:7]))
  return bytes(b)


class _CS:
  def __init__(self, v=15.0):
    self.out = structs.CarState()
    self.out.vEgo = v
    self.out.aEgo = 0.0
    self.out.gearShifter = structs.CarState.GearShifter.drive
    self.interceptor_state = 0
    self.interceptor_fault_seen = False
    self.fca11_cam_frame = None
    self.fca11_now_nanos = 0
    self.fca11_lead_closing = False
    self.fca11_lead_drel_m = 30.0
    self.fca11_lead_vrel_mps = -2.0
    self.pedal_long_engaged = True
    self.pedal_fault_monitor = gi.PedalFaultMonitor()
    self.fca11_echoes = []

  def __getattr__(self, name):
    if name == "is_metric":
      return False
    raise AttributeError(name)


def _cc(accel, long_active=True):
  cc = structs.CarControl()
  cc.enabled = cc.latActive = cc.longActive = long_active
  cc.actuators.accel = accel
  return cc


# a scripted drive: cruise, a soft stop, a HARD ask, release, a camera FCW blip near the end (hand-back latch)
def _script(n=900):
  out = []
  for k in range(n):
    t = k * 0.01
    if t < 1.0:
      a = 0.3
    elif t < 3.0:
      a = -0.9
    elif t < 4.0:
      a = -2.6          # hard (and below the -2.0 law bound: exercises the shadow clip)
    elif t < 5.0:
      a = -1.4
    elif t < 6.0:
      a = 0.2
    elif t < 8.0:
      a = -0.7
    else:
      a = 0.4
    v = max(0.0, 15.0 - 2.0 * max(0.0, t - 1.0))
    warn = 2 if 8.5 <= t < 8.6 else 0
    out.append((k, a, v, warn))
  return out


def _drive(brake: fl.Fca11LongBrake, script, accel_clip=None, echo=True):
  """step an Fca11LongBrake (actuating, Shadow or not) over a script; its own frames are ACCEPTED (echo next frame). In
  Shadow, v3_post_send() runs after every update() exactly as card.py does after the sendcan publish."""
  cs = _CS()
  sent, prev = [], []
  for k, a, v, warn in script:
    cs.out.vEgo = v
    cs.out.standstill = v < 0.1
    cs.fca11_cam_frame = (k * 10_000_000, _cam(alive=(k // 2) % 16, warn=warn))
    cs.fca11_now_nanos = k * 10_000_000
    if echo:
      brake.observe_echo(prev)
    ask = a if accel_clip is None else max(a, accel_clip)
    s = brake.update(_cc(ask), cs, k, fl.PERSONALITY_STANDARD)
    prev = [(128, bytes(x.dat)) for x in s]
    if brake.shadow:
      brake.v3_post_send()
    sent.append((k, s, brake.shadow_out if brake.shadow else None))
  return sent


def _wire(sends):
  return [(int(m[0]), bytes(m[1]), int(m[2])) for m in sends]


def _ci_stream(brake=None, shaping=None, shadow=None, script=None, post_send=True, lead=True, inject=None):
  """the REAL CarInterface.apply path (CarController.update incl. the carcontroller.py lead forward -> create_gas_command)
  over a script; the panda ACCEPTS every 0x38D (src-128 echo next frame). Returns (CI, per-frame sent stream)."""
  CIc, CP, CP_SP = _cp(brake=brake, shaping=shaping, shadow=shadow)
  ci = CIc(CP, CP_SP)
  if inject is not None:
    inject(ci)
  out, prev = [], []
  for k, a, v, warn in (script or _script()):
    ci.update([])                    # the real CarState step (no CAN this test); then the fields the bus would carry
    ci.CS.out.gearShifter = structs.CarState.GearShifter.drive
    ci.CS.out.vEgo = v
    ci.CS.out.standstill = v < 0.1
    ci.CS.fca11_cam_frame = (k * 10_000_000, _cam(alive=(k // 2) % 16, warn=warn))
    ci.CS.fca11_echoes = prev
    cc_sp = structs.CarControlSP()
    if lead:
      cc_sp.leadOne.status = True
      cc_sp.leadOne.dRel = 40.0 - 0.02 * k
      cc_sp.leadOne.vRel = -2.0
      cc_sp.leadOne.vLead = v - 2.0
      cc_sp.leadOne.aLeadK = -0.5
      cc_sp.leadOne.modelProb = 0.9
      cc_sp.leadOne.radar = True
    _, sends = ci.apply(_cc(a).as_reader(), cc_sp, k * 10_000_000)
    if post_send:
      ci.CC.fca11_post_send()        # what card.py does right after the sendcan publish
    out.append(_wire(sends))
    prev = [(128, bytes(m[1])) for m in sends if m[0] == 0x38D]
  return ci, out


def _log(tmp_path):
  path = tmp_path / "fca11-log" / shlog.FCA11_SHADOW_FILE
  return [json.loads(ln) for ln in path.read_text().splitlines()]


# ---------------------------------------------------------------------------------------------- plumbing
class TestShadowParamPlumbing:
  def test_field_default_false(self):
    assert structs.CarParamsSP().fca11Shadow is False
    _, _, CP_SP = _cp()
    assert CP_SP.fca11Shadow is False

  @pytest.mark.parametrize("raw,want", TRUTHY_TABLE)
  def test_truthy_parser_shared(self, raw, want):
    assert sp_ifaces._param_truthy(raw) == want
    _, _, CP_SP = _cp(shadow=raw)
    assert CP_SP.fca11Shadow == want

  @pytest.mark.parametrize("brake", [None, "0", "1"])
  @pytest.mark.parametrize("shaping", [None, "0", "1"])
  def test_shadow_config_is_the_deployed_oldlaw_config(self, brake, shaping):
    """Shadow ON => the car config equals the DEPLOYED OLD-LAW config for the same Brake (shaping OFF), except the
    fca11Shadow field itself: fca11Brake + bit 256 follow HyundaiFca11Brake, fca11AffineGain is forced False."""
    _, CP, CP_SP = _cp(brake=brake, shaping=shaping, shadow="1")
    _, CP0, CP_SP0 = _cp(brake=brake, shaping="0", shadow=None)
    b_on = brake == "1"
    assert CP_SP.fca11Shadow and CP_SP.fca11Brake == b_on and not CP_SP.fca11AffineGain
    assert bool(CP_SP.safetyParam & HyundaiSafetyFlagsSP.FCA11_LONG) == b_on
    assert CP_SP.safetyParam == CP_SP0.safetyParam
    assert CP.safetyConfigs[-1].safetyParam == CP0.safetyConfigs[-1].safetyParam
    d = dict(vars(CP_SP)); d.pop("fca11Shadow")
    d0 = dict(vars(CP_SP0)); d0.pop("fca11Shadow")
    assert d == d0
    CI = interfaces[CAR_UNDER_TEST]
    assert CI.get_pid_accel_limits(CP, CP_SP, 10., 20.) == CI.get_pid_accel_limits(CP0, CP_SP0, 10., 20.)

  def test_shadow_off_leaves_brake_path_unchanged(self):
    _, _, CP_SP = _cp(brake="1", shaping="1", shadow="0")
    assert CP_SP.fca11Brake and CP_SP.fca11AffineGain and not CP_SP.fca11Shadow
    assert CP_SP.safetyParam & HyundaiSafetyFlagsSP.FCA11_LONG

  def test_shadow_without_pedal_is_inert(self):
    _, CP, CP_SP = _cp(shadow="1", pedal=False)
    b = fl.Fca11LongBrake(CP, CP_SP)
    assert not b.shadow and not b.enabled and b._shadow_v3 is None

  def test_module_forces_oldlaw_on_a_hand_built_cp_sp(self):
    """even a hand-built CP_SP with fca11AffineGain set can never put V3 on the wire in Shadow."""
    _, CP, CP_SP = _cp(brake="1", shadow="1")
    CP_SP.fca11AffineGain = True
    b = fl.Fca11LongBrake(CP, CP_SP)
    assert b.shadow and b.enabled and not b.affine_gain and b._shadow_v3.affine_gain


# ---------------------------------------------------------------------------------------------- dependency / env
class TestShadowDependencyAndEnv:
  @pytest.mark.parametrize("brake", [None, "0", "1"])
  @pytest.mark.parametrize("shaping", [None, "0", "1"])
  def test_v3_computes_regardless_of_brake_and_shaping(self, brake, shaping):
    _, CP, CP_SP = _cp(brake=brake, shaping=shaping, shadow="1")
    b = fl.Fca11LongBrake(CP, CP_SP)
    assert b.shadow and b.enabled == (brake == "1") and not b.affine_gain
    assert b._shadow_v3.affine_gain
    res = _drive(b, _script(400))
    assert max(o.v3_wouldbe_lsb for _, _, o in res) > 0
    assert max(o.oldlaw_instant_lsb for _, _, o in res) > 0
    assert all(o.v3_ok for _, _, o in res)
    assert (max(o.oldlaw_cmd_lsb for _, _, o in res) > 0) == (brake == "1")

  @pytest.mark.parametrize("val", ["1", "0", "", "on"])
  def test_env_shadow_set_raises(self, monkeypatch, val):
    monkeypatch.setenv("FCA11_SHADOW", val)
    _, CP, CP_SP = _cp(brake="1")
    with pytest.raises(RuntimeError, match="HyundaiFca11Shadow"):
      fl.Fca11LongBrake(CP, CP_SP)

  def test_env_shadow_raises_through_the_controller(self, monkeypatch):
    monkeypatch.setenv("FCA11_SHADOW", "1")
    CarInterface, CP, CP_SP = _cp(shadow="1")
    with pytest.raises(RuntimeError):
      CarInterface(CP, CP_SP)


# ---------------------------------------------------------------------------------------------- wire = OLD-LAW
class TestShadowWireIsOldLaw:
  @pytest.mark.parametrize("brake,shaping", [("1", "1"), ("1", "0"), ("0", "0"), (None, "1")])
  def test_controller_stream_equals_deployed_oldlaw_stream(self, brake, shaping):
    """REAL CarInterface.apply path (incl. the vRel forward): Shadow stream == the deployed-semantics stream (same Brake,
    shaping OFF, Shadow OFF) byte-for-byte on EVERY frame, pedal command included; V3 is computed alongside."""
    ci_s, s1 = _ci_stream(brake=brake, shaping=shaping, shadow="1")
    ci_o, s0 = _ci_stream(brake=brake, shaping="0", shadow=None)
    assert s1 == s0
    n38d = sum(1 for fr in s1 for m in fr if m[0] == 0x38D)
    assert (n38d > 100) == (brake == "1") and (n38d == 0) == (brake != "1")
    b = ci_s.CC.fca11_brake
    assert b.shadow and b.shadow_out.non_actuating and b._shadow_v3.affine_gain
    assert b.shadow_out.frame == len(s1) - 1        # V3 computed every frame (post-send)

  def test_the_wire_check_is_live_v3_actuating_differs(self):
    """the comparison above can fail: a V3-ACTUATING build (Shadow OFF, Brake + Shaping ON) sends a different stream."""
    _, s_v3 = _ci_stream(brake="1", shaping="1", shadow=None)
    _, s_ol = _ci_stream(brake="1", shaping="0", shadow=None)
    assert s_v3 != s_ol

  def test_oldlaw_cmd_lsb_is_the_frame_on_the_wire(self):
    _, CP, CP_SP = _cp(brake="1", shaping="1", shadow="1")
    b = fl.Fca11LongBrake(CP, CP_SP)
    res = _drive(b, _script())
    n = 0
    for _, s, o in res:
      want = int(s[-1].dat[1]) if s else 0
      assert o.oldlaw_cmd_lsb == want and o.oldlaw_send == bool(s)
      n += int(want > 0)
    assert n > 100

  def test_v3_differs_from_oldlaw_on_the_script(self):
    """V3 computes rather than echoing OLD-LAW: the two levels differ on many frames of the same drive."""
    _, CP, CP_SP = _cp(brake="1", shadow="1")
    res = _drive(fl.Fca11LongBrake(CP, CP_SP), _script())
    assert sum(1 for _, _, o in res if o.v3_wouldbe_lsb != o.oldlaw_cmd_lsb) > 50


# ---------------------------------------------------------------------------------------------- V3-fault isolation
class TestV3FaultIsolation:
  def _boom_at(self, ci, start):
    core = ci.CC.fca11_brake._shadow_v3
    orig = core.update
    state = dict(n=0)

    def upd(*a, **k):
      state["n"] += 1
      if state["n"] >= start:
        raise ZeroDivisionError("injected V3 fault")
      return orig(*a, **k)
    core.update = upd

  @pytest.mark.parametrize("brake", ["1", "0"])
  def test_v3_raises_mid_episode_oldlaw_unchanged_and_logged(self, brake, tmp_path):
    """V3 raises mid-episode (t=1.6 s: inside the -0.9 stop) => the OLD-LAW stream is byte-identical to the fault-free
    run on every frame, a k=v3fail record is written, V3 is disabled for the ignition (v3_ok False, nulls in the log)."""
    _, ref = _ci_stream(brake=brake, shaping="1", shadow="1")
    ci, got = _ci_stream(brake=brake, shaping="1", shadow="1", inject=lambda c: self._boom_at(c, 160))
    assert got == ref
    b = ci.CC.fca11_brake
    assert b._v3_disabled and b.v3_failures == 1 and not b.shadow_out.v3_ok and b.shadow_out.v3_wouldbe_lsb is None
    b.shadow_log.flush()
    recs = _log(tmp_path)
    fails = [r for r in recs if r["k"] == "v3fail"]
    assert len(fails) == 1 and fails[0]["kind"] == "exception" and "injected V3 fault" in fails[0]["err"]
    assert set(shlog.V3FAIL_FIELDS) <= set(fails[0]) and fails[0]["frame"] == 159
    after = [r for r in recs if r["k"] == "f" and r["frame"] > 159]
    assert after and all(not r["v3_ok"] and r["v3_wouldbe_lsb"] is None and r["excl_reason"] == "v3_disabled" for r in after)
    if brake == "1":
      assert any(r["oldlaw_cmd_lsb"] > 0 for r in after)     # OLD-LAW kept braking after V3 died

  def test_update_never_steps_v3(self):
    """isolation by construction: update() (the send path) only captures V3 inputs; V3 runs in v3_post_send()."""
    _, CP, CP_SP = _cp(brake="1", shadow="1")
    b = fl.Fca11LongBrake(CP, CP_SP)
    calls = []
    b._shadow_v3.update = lambda *a, **k: calls.append(1) or []
    cs = _CS()
    cs.fca11_cam_frame = (0, _cam())
    b.update(_cc(-1.5), cs, 0, fl.PERSONALITY_STANDARD)
    assert calls == [] and b._v3_pending is not None
    b.v3_post_send()
    assert calls == [1] and b._v3_pending is None

  def test_v3_post_send_never_raises(self):
    _, CP, CP_SP = _cp(brake="1", shadow="1")
    b = fl.Fca11LongBrake(CP, CP_SP)
    b._shadow_v3.update = lambda *a, **k: (_ for _ in ()).throw(RuntimeError("x"))
    cs = _CS()
    cs.fca11_cam_frame = (0, _cam())
    for k in range(5):
      b.update(_cc(-1.5), cs, k, fl.PERSONALITY_STANDARD)
      b.v3_post_send()
    assert b.v3_failures == 1 and b._v3_disabled      # disabled after the first: no repeated faults

  def test_overrun_logged_then_disabled_oldlaw_unchanged(self, monkeypatch, tmp_path):
    """a V3 step slower than the budget is logged as an overrun; FCA11_V3_OVERRUN_TRIP consecutive overruns disable V3;
    the OLD-LAW stream is unchanged throughout."""
    _, ref = _ci_stream(brake="1", shadow="1")
    monkeypatch.setattr(fl, "FCA11_V3_BUDGET_S", -1.0)      # every V3 step is an overrun
    ci, got = _ci_stream(brake="1", shadow="1")
    assert got == ref
    b = ci.CC.fca11_brake
    assert b._v3_disabled and b.v3_overruns == fl.FCA11_V3_OVERRUN_TRIP
    b.shadow_log.flush()
    fails = [r for r in _log(tmp_path) if r["k"] == "v3fail"]
    assert [r["kind"] for r in fails] == ["overrun"] * fl.FCA11_V3_OVERRUN_TRIP
    assert [r["v3_disabled"] for r in fails] == [False] * (fl.FCA11_V3_OVERRUN_TRIP - 1) + [True]

  def test_isolated_overruns_do_not_disable(self, monkeypatch):
    _, CP, CP_SP = _cp(brake="1", shadow="1")
    b = fl.Fca11LongBrake(CP, CP_SP)
    cs = _CS()
    for k in range(40):
      cs.fca11_cam_frame = (k * 10_000_000, _cam(alive=(k // 2) % 16))
      monkeypatch.setattr(fl, "FCA11_V3_BUDGET_S", -1.0 if k % 2 else 10.0)   # every other step overruns
      b.update(_cc(-1.5), cs, k, fl.PERSONALITY_STANDARD)
      b.v3_post_send()
    assert b.v3_overruns == 20 and not b._v3_disabled and b.shadow_out.v3_ok


# ---------------------------------------------------------------------------------------------- compute equivalence
class TestShadowComputeEquivalence:
  @pytest.mark.parametrize("brake", ["1", "0"])
  def test_v3_wouldbe_equals_actuating_v3_sent_stream(self, brake):
    """v3_wouldbe_lsb / wouldbe_sends == what an ACTUATING V3 instance sends, frame for frame, on identical inputs (the
    actuating instance sees the -2.0 FCA11 PID bound; its frames all ACCEPTED). Independent of the OLD-LAW on the wire."""
    script = _script()
    _, CP, CP_SP = _cp(brake=brake, shaping="1", shadow="1")
    sh = _drive(fl.Fca11LongBrake(CP, CP_SP), script)
    _, CPa, CP_SPa = _cp(brake="1", shaping="1")
    act = _drive(fl.Fca11LongBrake(CPa, CP_SPa), script, accel_clip=-fl.FCA11_A_MAX_MSS2)
    n_cmp = 0
    for (k, _, o), (_, s, _) in zip(sh, act, strict=True):
      want = int(s[-1].dat[1]) if s else 0
      assert o.v3_wouldbe_lsb == want, (k, o.v3_wouldbe_lsb, want)
      assert [bytes(x.dat) for x in o.wouldbe_sends] == [bytes(x.dat) for x in s], k
      n_cmp += int(bool(s))
    assert n_cmp > 50

  def test_oldlaw_instant_lsb_is_the_instant_mapping(self):
    _, CP, CP_SP = _cp(shadow="1")
    b = fl.Fca11LongBrake(CP, CP_SP)
    for k, a, v, warn in _script(600):
      cs = _CS(v)
      cs.fca11_cam_frame = (k * 10_000_000, _cam())
      cs.fca11_now_nanos = k * 10_000_000
      b.update(_cc(a), cs, k, fl.PERSONALITY_STANDARD)
      b.v3_post_send()
      al = max(a, -fl.FCA11_A_MAX_MSS2)
      want = fl.clamp_dec_cmd(fl.dec_cmd_from_accel(al), fl.PERSONALITY_STANDARD) if al <= fl.FCA11_BRAKE_ONSET_ACCEL else 0
      assert b.shadow_out.oldlaw_instant_lsb == want

  def test_v3_core_never_writes_the_cars_cs(self):
    _, CP, CP_SP = _cp(brake="0", shadow="1")
    b = fl.Fca11LongBrake(CP, CP_SP)
    cs = _CS()
    for k in range(200):
      cs.fca11_now_nanos = k * 10_000_000
      cs.fca11_cam_frame = (k * 10_000_000, _cam(alive=(k // 2) % 16))
      b.update(_cc(-1.5), cs, k, fl.PERSONALITY_STANDARD)
      b.v3_post_send()
    assert "_fca11_braking_now" not in vars(cs)

  def test_unavailable_surfacing_is_the_oldlaw_instances(self):
    """the 0042 fca11_unavailable surfacing is the ACTUATING (OLD-LAW) instance's, exactly as deployed; V3's is logged."""
    _, CP, CP_SP = _cp(brake="1", shadow="1")
    b = fl.Fca11LongBrake(CP, CP_SP)
    _drive(b, _script())                          # the script ends with a camera FCW (warn=2) blip
    assert b._hard_cut and b.fca11_unavailable and b._shadow_v3._hard_cut


# ---------------------------------------------------------------------------------------------- lead fields
class TestLeadFields:
  def test_carcontroller_forwards_vrel_and_lead(self):
    ci, _ = _ci_stream(brake="1", shadow="1", script=_script(5))
    assert ci.CS.fca11_lead_vrel_mps == -2.0
    assert ci.CS.fca11_lead_one is not None and ci.CS.fca11_lead_one.modelProb == pytest.approx(0.9)

  def test_no_lead_forwards_none(self):
    ci, _ = _ci_stream(brake="1", shadow="1", script=_script(5), lead=False)
    assert ci.CS.fca11_lead_vrel_mps is None and ci.CS.fca11_lead_one is None and ci.CS.fca11_lead_drel_m is None

  def test_lead_fields_logged(self, tmp_path):
    ci, _ = _ci_stream(brake="1", shadow="1")
    ci.CC.fca11_brake.shadow_log.flush()
    frames = [r for r in _log(tmp_path) if r["k"] == "f"]
    assert frames and all(r["lead_present"] for r in frames)
    r = frames[0]
    assert r["lead_vrel_mps"] == -2.0 and r["lead_alead_mps2"] == -0.5 and r["lead_model_prob"] == 0.9
    assert r["lead_radar"] is True and r["lead_drel_m"] == round(40.0 - 0.02 * r["frame"], 2)
    assert r["lead_vlead_mps"] is not None and isinstance(r["t_mono_ns"], int) and r["t_wall"] > 0

  def test_no_lead_logs_nulls_never_zero(self, tmp_path):
    ci, _ = _ci_stream(brake="1", shadow="1", lead=False)
    ci.CC.fca11_brake.shadow_log.flush()
    frames = [r for r in _log(tmp_path) if r["k"] == "f"]
    assert frames
    for r in frames:
      assert r["lead_present"] is False
      assert all(r[f] is None for f in ("lead_drel_m", "lead_vrel_mps", "lead_vlead_mps", "lead_alead_mps2",
                                        "lead_model_prob", "lead_radar"))

  def test_low_quality_lead_is_logged_raw_not_filtered(self, tmp_path):
    def low(ci):
      orig = ci.CC.update

      def upd(CC, CC_SP, CS, now):
        CC_SP.leadOne.modelProb = 0.1
        CC_SP.leadOne.radar = False
        return orig(CC, CC_SP, CS, now)
      ci.CC.update = upd
    ci, _ = _ci_stream(brake="1", shadow="1", inject=low)
    ci.CC.fca11_brake.shadow_log.flush()
    frames = [r for r in _log(tmp_path) if r["k"] == "f"]
    assert frames and all(r["lead_present"] and r["lead_model_prob"] == 0.1 and r["lead_radar"] is False
                          and r["lead_vrel_mps"] == -2.0 for r in frames)


# ---------------------------------------------------------------------------------------------- writer format
class TestShadowLogFormat:
  def _records(self, tmp_path, brake="1"):
    _, CP, CP_SP = _cp(brake=brake, shaping="1", shadow="1")
    b = fl.Fca11LongBrake(CP, CP_SP)
    _drive(b, _script())
    b.shadow_log.flush()
    return _log(tmp_path)

  def test_schema_of_records(self, tmp_path):
    recs = self._records(tmp_path)
    kinds = [r["k"] for r in recs]
    assert kinds[0] == "hdr" and "f" in kinds and "ep" in kinds
    assert recs[0]["oldlaw_armed"] is True
    assert all(r["schema"] == shlog.FCA11_SHADOW_SCHEMA == 2 for r in recs)
    frames = [r for r in recs if r["k"] == "f"]
    for r in frames:
      assert set(shlog.FRAME_FIELDS) <= set(r), set(shlog.FRAME_FIELDS) - set(r)
    for r in (r for r in recs if r["k"] == "ep"):
      assert set(shlog.EPILOGUE_FIELDS) <= set(r), set(shlog.EPILOGUE_FIELDS) - set(r)
    hard = [r for r in frames if r["ask_mps2"] == -2.6]
    assert hard and all(r["ask_law_mps2"] == -fl.FCA11_A_MAX_MSS2 for r in hard)
    assert any(r["v3_wouldbe_lsb"] > 0 for r in frames) and any(r["oldlaw_cmd_lsb"] > 0 for r in frames)
    assert any(r["camera_requesting"] for r in frames)
    assert all(r["camera_request_level"] is not None for r in frames)
    assert any(r["episode_id"] is not None for r in frames)

  def test_logs_computed_values_not_constants(self, tmp_path):
    recs = [r for r in self._records(tmp_path) if r["k"] == "f" and r["v3_braking"]]
    assert len({r["v3_wouldbe_lsb"] for r in recs}) >= 3
    assert len({r["oldlaw_instant_lsb"] for r in recs}) >= 3
    assert len({r["oldlaw_cmd_lsb"] for r in recs}) >= 3

  def test_braking_off_logs_v3_and_zero_oldlaw(self, tmp_path):
    recs = self._records(tmp_path, brake="0")
    frames = [r for r in recs if r["k"] == "f"]
    assert recs[0]["oldlaw_armed"] is False
    assert all(r["oldlaw_cmd_lsb"] == 0 and not r["oldlaw_send"] for r in frames)
    assert any(r["v3_wouldbe_lsb"] > 0 for r in frames)

  def test_disabled_dir_writes_nothing(self, monkeypatch, tmp_path):
    monkeypatch.setenv("FCA11_LOG_DIR", "0")
    _, CP, CP_SP = _cp(shadow="1")
    b = fl.Fca11LongBrake(CP, CP_SP)
    _drive(b, _script(300))
    assert not b.shadow_log.enabled and b.shadow_log.n_records == 0
    assert not os.path.exists(tmp_path / "fca11-log")

  def test_rotation_per_drive_and_byte_bound(self, monkeypatch, tmp_path):
    d = tmp_path / "rot"
    for i in range(shlog.FCA11_SHADOW_KEEP + 2):
      w = shlog.ShadowLogger(str(d), header=dict(drive=i))
      w.flush()
    files = sorted(os.listdir(d))
    assert len(files) == shlog.FCA11_SHADOW_KEEP + 1     # current + KEEP generations
    monkeypatch.setattr(shlog, "FCA11_SHADOW_MAX_BYTES", 2000)
    w = shlog.ShadowLogger(str(d), header={})
    for i in range(200):
      w.write(dict(k="f", i=i))
    w.flush()
    lines = (d / shlog.FCA11_SHADOW_FILE).read_text().splitlines()
    assert json.loads(lines[-1])["k"] == "trunc" and w.truncated
    assert os.path.getsize(d / shlog.FCA11_SHADOW_FILE) < 2400


# ---------------------------------------------------------------------------------------------- FCA11-DISABLED (debug)
class TestFca11DisabledDebugMode:
  """The first shadow build's semantics (our stack fully off, V3 logged), kept ONLY as an explicitly named debug transform.
  It is NEVER reachable through the Shadow flag / any param combination."""

  def test_debug_transform_disables_our_stack_and_computes_v3(self):
    _, CP, CP_SP = _cp(brake="1", shaping="1", shadow="1")
    dbg = fl.fca11_disabled_debug_params(CP_SP)
    assert dbg is not CP_SP and CP_SP.fca11Brake                        # a copy; the car's CP_SP untouched
    assert dbg.fca11Shadow and not dbg.fca11Brake and not dbg.fca11AffineGain
    assert not (dbg.safetyParam & HyundaiSafetyFlagsSP.FCA11_LONG)
    b = fl.Fca11LongBrake(CP, dbg)
    assert b.shadow and not b.enabled
    res = _drive(b, _script())
    assert all(s == [] for _, s, _ in res)                              # FCA11-DISABLED: zero host 0x38D
    assert max(o.v3_wouldbe_lsb for _, _, o in res) > 0                 # ... V3 still computed

  @pytest.mark.parametrize("brake", [None, "0", "1"])
  @pytest.mark.parametrize("shaping", [None, "0", "1"])
  @pytest.mark.parametrize("shadow", [None, "0", "1"])
  def test_never_produced_by_setup_interfaces(self, brake, shaping, shadow):
    """no param combination yields 'Shadow ON with Braking forced OFF while HyundaiFca11Brake is ON'."""
    _, _, CP_SP = _cp(brake=brake, shaping=shaping, shadow=shadow)
    if brake == "1":
      assert CP_SP.fca11Brake and CP_SP.safetyParam & HyundaiSafetyFlagsSP.FCA11_LONG

  def test_no_production_caller(self):
    import pathlib
    root = pathlib.Path(fl.__file__).resolve().parents[3]
    import ast
    hits = []
    for p in root.rglob("*.py"):
      if "tests" in p.parts:
        continue
      src = p.read_text(encoding="utf-8", errors="ignore")
      if "fca11_disabled_debug_params" not in src:
        continue
      for n in ast.walk(ast.parse(src)):
        if isinstance(n, ast.Call) and getattr(n.func, "id", getattr(n.func, "attr", None)) == "fca11_disabled_debug_params":
          hits.append(f"{p}:{n.lineno}")
    assert hits == [], hits                          # no CALL anywhere outside tests


# ---------------------------------------------------------------------------------------------- Shadow=0 identity
def test_shadow_off_module_has_no_shadow_state():
  _, CP, CP_SP = _cp(brake="1", shaping="1")
  b = fl.Fca11LongBrake(CP, CP_SP)
  assert not b.shadow and b.enabled and b.affine_gain and b._shadow_v3 is None and b.shadow_log is None
  cs = _CS()
  cs.fca11_cam_frame = (0, _cam())
  b.update(_cc(-1.5), cs, 0, fl.PERSONALITY_STANDARD)
  assert b._v3_pending is None
  b.v3_post_send()                                   # a no-op off Shadow
