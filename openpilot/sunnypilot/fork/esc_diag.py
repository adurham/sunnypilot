"""
Fork: automatic READ-ONLY ESC (HECU) UDS read on every ignition, inside openpilot's own fingerprint window
(adurham/sunnypilot; car-features/auto-esc-read-report.md).

Why this exists
---------------
The FCA11/AEB investigation needs the ESC's identification / variant-coding DIDs and its stored DTCs. The hand-run
``car-features/esc_uds_read.py`` needed openpilot stopped and (it assumed) a panda firmware TX allowance. Neither is
needed: openpilot's own boot-time firmware query already talks UDS to this ESC every ignition (``carFw`` ``abs``
0x7D1 -> 0x7D9). It does so while the panda is still in ELM327 safety (no car safety model is set until
``FirmwareQueryDone`` + ``ControlsReady``), whose tx_hook allows any 8-byte 0x7xx frame. Logged drives show the ESC
answers ONLY through the OBD port, i.e. bus 1 with OBD multiplexing on (0x7D9 src 1; nothing on bus 0).

So this module runs in card, after ``get_car`` and BEFORE ``FirmwareQueryDone`` is set, reusing openpilot's own
OBD-multiplexing callback: no firmware change, no safety-mode change, no extra process owning the panda.

Hard rules, mechanically enforced here (tests: fork/tests/test_esc_diag.py, incl. mutation kills)
-------------------------------------------------------------------------------------------------
* Read-only allowlist: the ONLY frames that can reach ``can_send`` are ISO-TP single frames carrying 0x22
  ReadDataByIdentifier, 0x3E TesterPresent, 0x10 sub 0x03 (extended session), 0x19 sub 0x02 (ReadDTCByStatusMask),
  plus the one constant ISO-TP flow-control frame needed to receive multi-frame answers. Checked twice: per service
  (``guard_service``) before a frame is built and per frame (``guard_frame``) at the single TX call site. Address is
  fixed 0x7D1, bus fixed 1 (OBD port; never the car's C-CAN).
* Only when stationary: gear Park (LVR12 CF_Lvr_Gear == 0) and the wheels at standstill (WHL_SPD11, see
  ``wheels_moving``: fewer than 2 wheels above 12 LSB = 0.375 km/h and no wheel above 96 LSB = 3 km/h), from fresh
  frames (< ``STATE_MAX_AGE_S``), ignition on (pandaStates). The SAME definition (``VehicleGate.violation``) is the
  pre-check and is re-checked before EVERY frame and while waiting for every answer; any violation aborts at once.
  (Was: all four wheels exactly 0. A parked car reads single-wheel sensor noise of 1..59 LSB, so that skipped the
  read on the car, 2026-10-05: ``precheck: moving (wheel speeds [0.0, 0.0, 0.0, 0.03125])``.)
* Never interferes with driving: card has not created controls yet in this window (openpilot cannot be engaged, the
  panda has no car safety mode, the pedal gets no commands = driver pass-through, the stock camera is relayed). The
  OBD-multiplexing request is always returned to OFF in ``finally`` (also on SIGTERM / KeyboardInterrupt / any
  exception) and verified from pandaStates (safetyParam == 1). Every error is swallowed: this can never stop card.
* At most once per ignition cycle (key = boot id + deviceState.startedMonoTime). A pre-check skip does NOT consume
  the cycle: it is recorded in ``precheck_skips{key,n}`` before the wait, and ``plan()`` retries the same ignition up
  to ``MAX_PRECHECK_SKIPS`` tries. In Park, a not-yet-stationary car is given up to ``PRECHECK_WAIT_S`` to settle
  (out of Park the skip is immediate, so no startup delay). ``last_ignition`` is written only once the pre-check has
  passed, still before the multiplexer or any TX, so neither a crash nor a skipped try can lose the cycle. Within a
  cycle the full DID read also stops once a complete read exists for this ESC firmware version (max
  ``MAX_FULL_ATTEMPTS`` tries per version), and the DTC read repeats at most once per UTC day. Hard time budget
  ``RUN_BUDGET_S``.

Output: ``/data/esc-uds/<UTC>-<kind>.json`` (raw request/response hex per DID, NRCs, vehicle-state trace) and a
compact ``esc_uds_read`` cloudlog event, so the result is also in that drive's rlog. Disable: touch
``/data/esc-uds/DISABLE``.
"""
import hashlib
import json
import os
import signal
import time
from collections.abc import Callable

from opendbc.car.can_definitions import CanData

# ---------------------------------------------------------------------------------------------------------------------
# Read-only allowlist (same semantics as the reviewed car-features/esc_uds_read.py)
# ---------------------------------------------------------------------------------------------------------------------
SVC_READ_DATA_BY_IDENTIFIER = 0x22
SVC_DIAGNOSTIC_SESSION_CONTROL = 0x10
SVC_TESTER_PRESENT = 0x3E
SVC_READ_DTC_INFORMATION = 0x19
ALLOWED_SERVICES = frozenset({SVC_READ_DATA_BY_IDENTIFIER, SVC_DIAGNOSTIC_SESSION_CONTROL, SVC_TESTER_PRESENT,
                              SVC_READ_DTC_INFORMATION})
ALLOWED_SESSION_SUBFUNC = 0x03   # extended only; never default/programming/safety
ALLOWED_DTC_SUBFUNC = 0x02       # ReadDTCByStatusMask only
ALLOWED_DTC_MASK = 0xFF
# The single flow-control frame we ever send: ContinueToSend, block size 0 (send all), STmin 10 ms.
FLOW_CONTROL_FRAME = bytes([0x30, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00])

ESC_REQ_ADDR = 0x7D1
ESC_RSP_ADDR = 0x7D9
ESC_BUS = 1                      # OBD port (needs OBD multiplexing). Logged drives: the ESC never answers on bus 0.

# Vehicle state from raw C-CAN (bus 0). Verified on route 0000012f against carState.gearShifter / vEgoRaw.
LVR12_ADDR = 0x367               # CF_Lvr_Gear: bits 32..35 (Intel), 0 = P, 5 = D, 6 = N, 7 = R
WHL_SPD11_ADDR = 0x386           # WHL_SPD_FL/FR/RL/RR: 14-bit Intel at bits 0/16/32/48, 0.03125 km/h
WHEEL_SPEED_LSB_KPH = 0.03125
CAR_BUS = 0
GEAR_PARK = 0
STATE_MAX_AGE_S = 0.25           # LVR12 50 Hz, WHL_SPD11 50 Hz; older than this = unknown = abort
# Standstill (car-features/esc-read-and-probe-fixes.md; 396 rlog segments of this car, bus 0, every WHL_SPD11 frame):
# * Parked noise (Park, >= 3 s after the last non-Park gear frame; 215,942 frames): 8.4 % of frames have a nonzero
#   wheel, single wheels read up to 59 LSB, but NEVER 2 wheels above 8 LSB at once (0 frames).
# * Genuine motion (131 starts from an all-zero frame that reach >= 1 km/h on all 4 wheels within 2 s): >= 2 wheels
#   above 12 LSB fires within <= 42 frames (0.84 s), after <= 2.5 cm of travel; none missed.
# 12 LSB = 0.375 km/h is opendbc's own Hyundai carstate STANDSTILL_THRESHOLD. A single wheel above 96 LSB (3 km/h; max
# parked single-wheel noise seen: 59) also counts as moving (defense in depth, 0 parked frames).
STANDSTILL_WHEEL_LSB = 12
STANDSTILL_MIN_WHEELS = 2
SINGLE_WHEEL_MOVING_LSB = 96
PRECHECK_WAIT_S = 3.0            # in Park only: wait this long for the wheels to settle before skipping this try
MAX_PRECHECK_SKIPS = 3           # pre-check skips per ignition cycle before the cycle is given up

RESP_TIMEOUT_S = 0.25            # first answer frame
PENDING_TIMEOUT_S = 2.0          # after NRC 0x78 responsePending
CF_TIMEOUT_S = 0.5               # between consecutive frames
RUN_BUDGET_S = 20.0              # hard cap on the whole read (openpilot start is delayed by at most this)
SILENT_ABORT_N = 4               # this many consecutive requests with no answer at all = ESC not reachable, stop
MAX_FULL_ATTEMPTS = 3            # full-read tries per ESC firmware version before giving up on it
KEEP_FILES = 60

OUT_DIR = "/data/esc-uds"
TARGET_FINGERPRINTS = ("HYUNDAI_ELANTRA_2022_NON_SCC",)


class SafetyViolation(Exception):
  pass


class Abort(Exception):
  """Vehicle state left the allowed envelope (or the budget ran out): stop sending at once."""


def guard_service(service: int, subfunc: int | None) -> None:
  if service not in ALLOWED_SERVICES:
    raise SafetyViolation(f"service 0x{service:02X} is not in the read-only allowlist")
  if service == SVC_DIAGNOSTIC_SESSION_CONTROL and subfunc != ALLOWED_SESSION_SUBFUNC:
    raise SafetyViolation(f"session control sub-function {subfunc!r} refused (only 0x03 extended)")
  if service == SVC_READ_DTC_INFORMATION and subfunc != ALLOWED_DTC_SUBFUNC:
    raise SafetyViolation(f"read-DTC sub-function {subfunc!r} refused (only 0x02)")


def guard_frame(addr: int, dat: bytes, bus: int) -> None:
  """Last check before the bus: the exact frame shapes this module may emit, nothing else."""
  dat = bytes(dat)
  if addr != ESC_REQ_ADDR or bus != ESC_BUS or len(dat) != 8:
    raise SafetyViolation(f"frame 0x{addr:X} bus {bus} len {len(dat)} refused")
  if dat == FLOW_CONTROL_FRAME:
    return
  if dat[0] >> 4 != 0 or not 1 <= dat[0] <= 7:
    raise SafetyViolation(f"only ISO-TP single frames may be sent: {dat.hex()}")
  ln, svc = dat[0], dat[1]
  if svc == SVC_READ_DATA_BY_IDENTIFIER and ln == 3:
    return
  if svc == SVC_TESTER_PRESENT and ln == 2 and dat[2] in (0x00, 0x80):
    return
  if svc == SVC_DIAGNOSTIC_SESSION_CONTROL and ln == 2 and dat[2] == ALLOWED_SESSION_SUBFUNC:
    return
  if svc == SVC_READ_DTC_INFORMATION and ln == 3 and dat[2] == ALLOWED_DTC_SUBFUNC and dat[3] == ALLOWED_DTC_MASK:
    return
  raise SafetyViolation(f"frame {dat.hex()} is not an allowlisted read-only request")


def build_single_frame(payload: bytes) -> bytes:
  if not 1 <= len(payload) <= 7:
    raise ValueError(f"payload length {len(payload)} does not fit a single frame")
  return (bytes([len(payload)]) + payload).ljust(8, b"\x00")


def did_table() -> list[int]:
  """Same DIDs as car-features/esc_uds_read.py (70): identification + candidate variant-coding ranges."""
  dids = [0xF100, *range(0xF110, 0xF120), 0xF15A, 0xF15B, 0xF187, 0xF188, 0xF189, 0xF18A, 0xF18B, 0xF18C, 0xF190,
          0xF191, 0xF192, 0xF193, 0xF194, 0xF195, 0xF19E, *range(0xF1A0, 0xF1A6), *range(0x0100, 0x0110),
          *range(0x0600, 0x0610)]
  return dids


def decode_gear(dat: bytes) -> int:
  return (int.from_bytes(bytes(dat)[:8].ljust(8, b"\x00"), "little") >> 32) & 0xF


def decode_wheel_speeds_raw(dat: bytes) -> list[int]:
  w = int.from_bytes(bytes(dat)[:8].ljust(8, b"\x00"), "little")
  return [(w >> (16 * i)) & 0x3FFF for i in range(4)]


def decode_wheel_speeds(dat: bytes) -> list[float]:
  return [r * WHEEL_SPEED_LSB_KPH for r in decode_wheel_speeds_raw(dat)]


def wheels_moving(raw: list[int]) -> bool:
  """The single standstill definition (pre-check AND every per-frame check). Raw LSB, not km/h: no float compare."""
  if sum(1 for r in raw if r > STANDSTILL_WHEEL_LSB) >= STANDSTILL_MIN_WHEELS:
    return True
  return any(r > SINGLE_WHEEL_MOVING_LSB for r in raw)


class VehicleGate:
  """Tracks gear + wheel speed from raw bus-0 frames; ``check`` raises Abort unless parked and stationary."""

  def __init__(self, now: Callable[[], float]):
    self.now = now
    self.gear: int | None = None
    self.speeds: list[int] | None = None   # raw LSB
    self.t_gear = -1e9
    self.t_speed = -1e9
    self.reason = ""

  def feed(self, msg: CanData) -> None:
    if msg.src != CAR_BUS:
      return
    if msg.address == LVR12_ADDR:
      self.gear, self.t_gear = decode_gear(msg.dat), self.now()
    elif msg.address == WHL_SPD11_ADDR:
      self.speeds, self.t_speed = decode_wheel_speeds_raw(msg.dat), self.now()

  def violation(self) -> str:
    t = self.now()
    if self.gear is None or t - self.t_gear > STATE_MAX_AGE_S:
      return "gear unknown/stale"
    if self.speeds is None or t - self.t_speed > STATE_MAX_AGE_S:
      return "wheel speed unknown/stale"
    if self.gear != GEAR_PARK:
      return f"not in Park (gear {self.gear})"
    if wheels_moving(self.speeds):
      return f"moving (wheel speeds {[r * WHEEL_SPEED_LSB_KPH for r in self.speeds]} km/h)"
    return ""

  def check(self) -> None:
    v = self.violation()
    if v:
      self.reason = v
      raise Abort(v)


class EscUdsClient:
  """Minimal read-only UDS client. ``_tx`` is the ONLY place a frame is handed to can_send."""

  def __init__(self, can_send, can_recv, gate: VehicleGate, now: Callable[[], float], deadline: float):
    self._can_send = can_send
    self._can_recv = can_recv
    self.gate = gate
    self.now = now
    self.deadline = deadline
    self.tx_log: list[str] = []
    self.session = "default"
    self.silent = 0

  def _tx(self, dat: bytes) -> None:
    self.gate.check()
    if self.now() > self.deadline:
      raise Abort("time budget exhausted")
    guard_frame(ESC_REQ_ADDR, dat, ESC_BUS)
    self._can_send([CanData(ESC_REQ_ADDR, bytes(dat), ESC_BUS)])
    self.tx_log.append(bytes(dat).hex())

  def _rx_frames(self) -> list[bytes]:
    out = []
    for packet in self._can_recv(wait_for_one=True):
      for msg in packet:
        self.gate.feed(msg)
        if msg.src == ESC_BUS and msg.address == ESC_RSP_ADDR:
          out.append(bytes(msg.dat))
    self.gate.check()
    return out

  def drain(self) -> None:
    self._rx_frames()

  def request(self, service: int, subfunc: int | None, payload: bytes = b"") -> dict:
    guard_service(service, subfunc)
    req = bytes([service]) + (bytes([subfunc]) if subfunc is not None else b"") + bytes(payload)
    frame = build_single_frame(req)
    self.drain()
    self._tx(frame)
    res: dict = {"req": frame.hex(), "frames": []}
    t_end = self.now() + RESP_TIMEOUT_S
    data = b""
    expect = None
    while self.now() < t_end:
      for f in self._rx_frames():
        res["frames"].append(f.hex())
        kind = f[0] >> 4
        if expect is None and kind == 0:
          body = f[1:1 + (f[0] & 0xF)]
          if len(body) >= 3 and body[0] == 0x7F and body[2] == 0x78:
            t_end = self.now() + PENDING_TIMEOUT_S   # responsePending: keep waiting
            continue
          self.silent = 0
          return self._finish(res, service, body)
        if expect is None and kind == 1:
          expect = ((f[0] & 0xF) << 8) | f[1]
          data = f[2:8]
          self._tx(FLOW_CONTROL_FRAME)
          t_end = self.now() + CF_TIMEOUT_S
        elif expect is not None and kind == 2:
          data += f[1:8]
          t_end = self.now() + CF_TIMEOUT_S
          if len(data) >= expect:
            self.silent = 0
            return self._finish(res, service, data[:expect])
    res["no_response" if not res["frames"] else "incomplete"] = True
    self.silent = self.silent + 1 if not res["frames"] else 0
    if self.silent >= SILENT_ABORT_N:
      raise Abort(f"ESC silent for {self.silent} requests")
    return res

  @staticmethod
  def _finish(res: dict, service: int, body: bytes) -> dict:
    res["resp"] = body.hex()
    if body[:1] == b"\x7f":
      res["nrc"] = body[2] if len(body) > 2 else None
    elif body[:1] == bytes([service + 0x40]):
      res["positive"] = True
    return res

  def read_did(self, did: int) -> dict:
    return self.request(SVC_READ_DATA_BY_IDENTIFIER, None, bytes([did >> 8, did & 0xFF]))

  def extended_session(self) -> dict:
    r = self.request(SVC_DIAGNOSTIC_SESSION_CONTROL, ALLOWED_SESSION_SUBFUNC)
    if r.get("positive"):
      self.session = "extended"
    return r

  def read_dtcs(self) -> dict:
    return self.request(SVC_READ_DTC_INFORMATION, ALLOWED_DTC_SUBFUNC, bytes([ALLOWED_DTC_MASK]))


def read_all_dids(client: EscUdsClient) -> list[dict]:
  """0x22 every DID in the default session; DIDs refused with NRC 0x31/0x7F are retried once in extended (0x10 0x03).
  The ESC drops back to default by itself (S3 timeout); 0x10 0x01 is never sent."""
  results = []
  for did in did_table():
    r = client.read_did(did)
    r["did"] = f"{did:04X}"
    results.append(r)
  retry = [r for r in results if r.get("nrc") in (0x31, 0x7F)]
  if retry:
    ext = client.extended_session()
    results.append({"did": "session_10_03", **ext})
    if client.session == "extended":
      for r in retry:
        rr = client.read_did(int(r["did"], 16))
        rr["did"] = r["did"] + "@ext"
        results.append(rr)
  return results


# ---------------------------------------------------------------------------------------------------------------------
# Scheduling state (once per ignition, per-firmware completion, daily DTC)
# ---------------------------------------------------------------------------------------------------------------------
def _load_state(out_dir: str) -> dict:
  try:
    with open(os.path.join(out_dir, "state.json")) as f:
      return json.load(f)
  except (OSError, ValueError):
    return {}


def _save_state(out_dir: str, state: dict) -> None:
  path = os.path.join(out_dir, "state.json")
  tmp = path + ".tmp"
  with open(tmp, "w") as f:
    json.dump(state, f, indent=1, sort_keys=True)
  os.replace(tmp, path)


def _write_result(out_dir: str, kind: str, doc: dict, wall: float) -> str:
  name = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime(wall)) + f"-{kind}.json"
  path = os.path.join(out_dir, name)
  with open(path, "w") as f:
    json.dump(doc, f, indent=1, sort_keys=True)
  files = sorted(p for p in os.listdir(out_dir) if p.endswith(".json") and p != "state.json")
  for old in files[:-KEEP_FILES]:
    try:
      os.remove(os.path.join(out_dir, old))
    except OSError:
      pass
  return path


def esc_fw_key(car_fw) -> str:
  for fw in car_fw or []:
    if str(getattr(fw, "ecu", "")) == "abs" and int(getattr(fw, "address", 0)) == ESC_REQ_ADDR:
      return hashlib.sha256(bytes(fw.fwVersion)).hexdigest()[:16]
  return "unknown"


def precheck_skips(state: dict, ignition_key: str) -> int:
  sk = state.get("precheck_skips") or {}
  return int(sk.get("n", 0)) if sk.get("key") == ignition_key else 0


def plan(state: dict, ignition_key: str, fw_key: str, day: str) -> tuple[bool, bool]:
  """-> (do_full, do_dtc). Pure; nothing that reached the bus runs twice in one ignition cycle, and a cycle whose
  pre-check was skipped is retried at most MAX_PRECHECK_SKIPS times in total."""
  if state.get("last_ignition") == ignition_key:
    return False, False
  if precheck_skips(state, ignition_key) >= MAX_PRECHECK_SKIPS:
    return False, False
  fw = state.get("fw", {}).get(fw_key, {})
  do_full = not fw.get("complete") and fw.get("attempts", 0) < MAX_FULL_ATTEMPTS
  do_dtc = state.get("last_dtc_day") != day
  return do_full, do_dtc


def run(can_send, can_recv, set_obd_multiplexing, *, fingerprint: str, car_fw, ignition_key: str | None,
        ignition_on: bool, out_dir: str = OUT_DIR, now: Callable[[], float] = time.monotonic,
        wall: Callable[[], float] = time.time,  # noqa: TID251 - wall clock wanted (file names, daily DTC)
        verify_mux_off: Callable[[], bool] | None = None,
        log_event: Callable[..., None] | None = None) -> dict:
  """Core (dependency-injected, so the tests drive the real code path). Returns a summary dict; never raises
  SafetyViolation silently: a guard trip is recorded as the result and nothing further is sent."""
  summary: dict = {"ran": False}
  if fingerprint not in TARGET_FINGERPRINTS:
    return {**summary, "skip": "not the target car"}
  if os.path.exists(os.path.join(out_dir, "DISABLE")):
    return {**summary, "skip": "disabled"}
  if not ignition_on:
    return {**summary, "skip": "ignition off"}
  if not ignition_key:
    return {**summary, "skip": "ignition cycle unknown"}
  os.makedirs(out_dir, exist_ok=True)
  state = _load_state(out_dir)
  fw_key = esc_fw_key(car_fw)
  day = time.strftime("%Y-%m-%d", time.gmtime(wall()))
  do_full, do_dtc = plan(state, ignition_key, fw_key, day)
  if not (do_full or do_dtc):
    return {**summary, "skip": "already done (ignition/firmware/day)"}

  gate = VehicleGate(now)
  t0 = now()
  client = EscUdsClient(can_send, can_recv, gate, now, t0 + RUN_BUDGET_S)
  doc: dict = {"ignition_key": ignition_key, "fw_key": fw_key, "car_fw_abs": None, "results": [], "dtc": None,
               "aborted": None, "error": None, "mux_restored": None}
  for fw in car_fw or []:
    if str(getattr(fw, "ecu", "")) == "abs":
      doc["car_fw_abs"] = bytes(fw.fwVersion).hex()

  # Pre-check from live frames before touching the multiplexer, with the SAME definition the per-frame checks use.
  # Wait until the state is KNOWN (fresh frames, normally < 20 ms). Out of Park: skip at once (no startup delay). In
  # Park but not (yet) at standstill, e.g. still settling onto the parking pawl: keep watching for at most
  # PRECHECK_WAIT_S, then skip this try. Count the skip BEFORE waiting, so a crash/kill mid-wait still counts.
  state["precheck_skips"] = {"key": ignition_key, "n": precheck_skips(state, ignition_key) + 1}
  _save_state(out_dir, state)
  t_pre = now()
  known_by, settle_by = t_pre + 0.5, t_pre + PRECHECK_WAIT_S

  def _pre_wait() -> bool:
    v = gate.violation()
    if "stale" in v:
      return now() < known_by
    return v.startswith("moving") and now() < settle_by

  while _pre_wait():
    for packet in can_recv(wait_for_one=True):
      for msg in packet:
        gate.feed(msg)
  pre = gate.violation()
  doc["precheck_wait_s"] = round(now() - t_pre, 3)
  if pre:
    doc["aborted"] = "precheck: " + pre
    doc["precheck_try"] = state["precheck_skips"]["n"]
    summary.update(ran=False, skip=doc["aborted"], precheck_try=doc["precheck_try"])
    _write_result(out_dir, "skipped", doc, wall())
    return summary

  # Pre-check passed: NOW mark the ignition cycle, BEFORE sending anything, so a crash mid-read can never cause a
  # retry loop in this cycle.
  state["last_ignition"] = ignition_key
  state.pop("precheck_skips", None)
  _save_state(out_dir, state)

  summary["ran"] = True
  old_sigterm = None
  try:
    try:
      old_sigterm = signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
    except ValueError:
      old_sigterm = None  # not the main thread: SIGINT/finally still restore the multiplexer
    set_obd_multiplexing(True)
    client.drain()
    if do_full:
      fwst = state.setdefault("fw", {}).setdefault(fw_key, {})
      fwst["attempts"] = fwst.get("attempts", 0) + 1
      _save_state(out_dir, state)
      doc["results"] = read_all_dids(client)
      answered = sum(1 for r in doc["results"] if r.get("frames"))
      fwst["complete"] = answered > 0
      fwst["answered"] = answered
    if do_dtc:
      doc["dtc"] = client.read_dtcs()
      if doc["dtc"].get("frames"):
        state["last_dtc_day"] = day
  except Abort as e:
    doc["aborted"] = str(e)
  except SafetyViolation as e:
    doc["error"] = f"SafetyViolation: {e}"
  except Exception as e:  # never let this take card down
    doc["error"] = repr(e)
  finally:
    try:
      set_obd_multiplexing(False)
      doc["mux_restored"] = verify_mux_off() if verify_mux_off is not None else True
    except BaseException as e:
      doc["mux_restored"] = False
      doc["error"] = (doc["error"] or "") + f" | mux restore failed: {e!r}"
    if old_sigterm is not None:
      signal.signal(signal.SIGTERM, old_sigterm)

  doc["tx"] = client.tx_log
  doc["duration_s"] = round(now() - t0, 3)
  doc["final_gate"] = gate.violation() or "parked+stationary"
  if doc["aborted"] or doc["error"]:
    fwst = state.get("fw", {}).get(fw_key)
    if fwst is not None and doc["aborted"]:
      fwst["complete"] = False
  _save_state(out_dir, state)
  kind = ("full" if do_full else "") + ("dtc" if do_dtc else "")
  path = _write_result(out_dir, kind, doc, wall())
  answered = sum(1 for r in doc["results"] if r.get("frames"))
  summary.update(path=path, full=do_full, dtc=do_dtc, answered=answered, tx=len(client.tx_log),
                 aborted=doc["aborted"], error=doc["error"], mux_restored=doc["mux_restored"],
                 duration_s=doc["duration_s"], dtc_resp=(doc["dtc"] or {}).get("resp"))
  if log_event is not None:
    log_event("esc_uds_read", **summary)
  return summary


def _raise_keyboard_interrupt(signum, frame):
  raise KeyboardInterrupt


# ---------------------------------------------------------------------------------------------------------------------
# card hook
# ---------------------------------------------------------------------------------------------------------------------
def _ignition_from_services() -> tuple[str | None, bool, Callable[[], bool]]:
  import openpilot.cereal.messaging as messaging
  ds_sock = messaging.sub_sock("deviceState", timeout=1500)
  ps_sock = messaging.sub_sock("pandaStates", timeout=1500)
  ds = messaging.recv_one(ds_sock)
  ps = messaging.recv_one(ps_sock)
  key = None
  if ds is not None and ds.deviceState.started and ds.deviceState.startedMonoTime:
    try:
      with open("/proc/sys/kernel/random/boot_id") as f:
        boot = f.read().strip()
    except OSError:
      boot = "noboot"
    key = f"{boot}:{ds.deviceState.startedMonoTime}"
  ign = ps is not None and any(p.ignitionLine or p.ignitionCan for p in ps.pandaStates)

  def verify_mux_off() -> bool:
    messaging.drain_sock_raw(ps_sock)  # only states published AFTER the restore request count
    t_end = time.monotonic() + 2.0
    while time.monotonic() < t_end:
      m = messaging.recv_one(ps_sock)
      if m is not None and len(m.pandaStates) and str(m.pandaStates[0].safetyModel) == "elm327" \
         and m.pandaStates[0].safetyParam == 1:
        return True
    return False
  return key, ign, verify_mux_off


def run_from_card(CP, can_callbacks, set_obd_multiplexing) -> None:
  """Called by card between get_car() and FirmwareQueryDone. Never raises."""
  try:
    from openpilot.common.hardware import PC
    from openpilot.common.swaglog import cloudlog
    if PC or os.environ.get("REPLAY") or CP.carFingerprint not in TARGET_FINGERPRINTS:
      return
    key, ign, verify = _ignition_from_services()
    can_recv, can_send = can_callbacks
    summary = run(can_send, can_recv, set_obd_multiplexing, fingerprint=CP.carFingerprint, car_fw=CP.carFw,
                  ignition_key=key, ignition_on=ign, verify_mux_off=verify, log_event=cloudlog.event)
    if not summary.get("ran"):
      cloudlog.event("esc_uds_read", **summary)
  except BaseException as e:
    try:
      set_obd_multiplexing(False)
    except BaseException:
      pass
    try:
      from openpilot.common.swaglog import cloudlog
      cloudlog.exception(f"esc_uds_read failed: {e!r}")
    except BaseException:
      pass
    if isinstance(e, KeyboardInterrupt):
      raise
