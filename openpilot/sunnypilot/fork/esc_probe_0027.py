"""
Fork: ESC (HECU) security-access PROBE — request the 0x27 seed and write the CURRENT 0x0103 variant-coding
value straight back (a no-op by construction), inside openpilot's own fingerprint window.
(adurham/sunnypilot; car-features/esc-probe-0027-report.md)

Why this exists
---------------
Question to answer on the car: does WRITING the ESC's variant-coding DID 0x0103 require UDS security access
(0x27) at all? The one-minute probe is deliberately the smallest possible write: read 0x0103 as it is, ask for
the 0x27 seed, then write those exact bytes back with 0x2E. Writing a value onto itself is a no-op for the ESC's
configuration; what is informative is the *reply*:

  * a refused seed + a refused write  -> the write path is gated by security access;
  * a refused seed + an ACCEPTED write -> the ESC takes 0x2E without an unlock (the gate is elsewhere/absent);
  * an all-zero seed                 -> "already unlocked", the write is attempted anyway to see if it lands.

Phase 1 never sends the key (0x27 sub 0x02) — asking is the whole probe.

Vendor-confirmed write flow (GIT VariantCodingTable, decoded 2026-10-05; CN7N ESC block SecuritySupported=0)
------------------------------------------------------------------------------------------------------------
The vendor's own variant-coding table for THIS car's ESC (GIT VariantCodingTable_HY.git.xml, the CN7N/"Elantra N"
block — HECU 58910-IB000, CAN IDs 0x7D1/0x7D9) exposes the authoritative write sequence as plain request hex:

    <Backup requestvalue="07D103220103">          -> 0x7D1: 03 22 01 03      read the current 0x0103 value
    <Input  requestvalue="07D1021003">            -> 0x7D1: 02 10 03         ENTER EXTENDED SESSION
    <Input  requestvalue="07D1072E0103$01$$02$$03$$04$"> -> 0x7D1: 07 2E 01 03 <4 code bytes>   write

and `<Security SecuritySupported="0" Securityindex="0" CANID="07D1"/>` — the vendor tool never sends 0x27 for this
ECU (the 27 01 templates exist only for the Kia CONTI/MANDO entries with security indexes 26300/270100). This probe
still ASKS for the 0x27 seed (that contrast is informative), but mirrors the vendor exactly at the write: read 0x0103
-> 10 03 extended session -> 2E 0103 no-op.

Safety, mechanically enforced here (tests: fork/tests/test_esc_probe_0027.py, mutation-proven)
----------------------------------------------------------------------------------------------
* Enabled only by a state file: ``/data/esc-probe-0027/state.json`` with ``{"probe_enabled": true}``. Missing file
  or flag -> ``run()`` returns ``{"skip": "not enabled"}`` and nothing happens. ``touch .../DISABLE`` forces off.
* At most ONE probe per ignition cycle: ``done_ignition`` is written once the standstill pre-check passes, BEFORE
  the multiplexer or any TX, so neither a crash nor a killed process can lose the cycle.
* Allowlist (checked twice: ``guard_service`` before a frame is built, ``guard_frame`` at the single TX site):
  0x22 ReadDataByIdentifier, 0x3E TesterPresent, 0x10 sub 0x03 (extended: retry a refused read, or the
  vendor-confirmed session step before the write),
  0x27 sub 0x01 ONLY (sendKey 0x02 raises), 0x2E with DID 0x0103 ONLY — and the 0x2E payload bytes MUST equal
  the bytes read back from 0x0103 in step 1 (an argument to the request, asserted at the TX site too).
* Only when stationary: the SAME definition as esc_diag — gear Park (LVR12) and the wheels at standstill
  (WHL_SPD11; ``esc_diag.wheels_moving``), from fresh bus-0 frames. ``VehicleGate`` is IMPORTED, not copied, so
  there is exactly one standstill definition in the tree. Re-checked before every frame and while waiting for
  every answer; any violation aborts at once.
* The write happens ONLY if step 1 returned exactly ``62 01 03 <4 bytes>``; otherwise nothing is written.
* Never interferes: no car safety mode, no controls; OBD multiplexing is switched off in ``finally`` (SIGTERM too)
  and verified from pandaStates. Errors never propagate into card. Hard budget ``RUN_BUDGET_S``.

Output: ``/data/esc-probe-0027/<UTC>-result.json`` (raw request/response hex, TX log, duration, aborted/error,
mux_restored) + a compact ``esc_probe_0027`` cloudlog event, so the result is also in that drive's rlog.
"""
import json
import os
import signal
import time
from collections.abc import Callable

from opendbc.car.can_definitions import CanData

# The standstill definition lives in exactly one place: esc_diag. Import it, do not copy it.
from openpilot.sunnypilot.fork.esc_diag import Abort, FLOW_CONTROL_FRAME, SafetyViolation, VehicleGate, \
  build_single_frame, wheels_moving

# ---------------------------------------------------------------------------------------------------------------------
# Probe allowlist — a strict superset of esc_diag's read set (0x27/0x2E added) and nothing else.
# ---------------------------------------------------------------------------------------------------------------------
SVC_READ_DATA_BY_IDENTIFIER = 0x22
SVC_DIAGNOSTIC_SESSION_CONTROL = 0x10
SVC_TESTER_PRESENT = 0x3E
SVC_SECURITY_ACCESS = 0x27
SVC_WRITE_DATA_BY_IDENTIFIER = 0x2E
ALLOWED_SERVICES = frozenset({SVC_READ_DATA_BY_IDENTIFIER, SVC_DIAGNOSTIC_SESSION_CONTROL, SVC_TESTER_PRESENT,
                              SVC_SECURITY_ACCESS, SVC_WRITE_DATA_BY_IDENTIFIER})
ALLOWED_SESSION_SUBFUNC = 0x03   # extended only (used to retry a refused 0x0103 read)
ALLOWED_SEC_SUBFUNC = 0x01       # requestSeed ONLY; sendKey (0x02) is never sent in this probe
DID_VARIANT_CODING = 0x0103      # the ESC's variant-coding DID; the ONLY DID this module may write
WRITE_DID = DID_VARIANT_CODING

ESC_REQ_ADDR = 0x7D1
ESC_RSP_ADDR = 0x7D9
ESC_BUS = 1                      # OBD port (needs OBD multiplexing); the ESC never answers on bus 0

RESP_TIMEOUT_S = 0.25            # first answer frame
PENDING_TIMEOUT_S = 2.0          # after NRC 0x78 responsePending
CF_TIMEOUT_S = 0.5               # between consecutive frames
RUN_BUDGET_S = 15.0              # hard cap on the whole probe
SILENT_ABORT_N = 3               # consecutive requests with no answer at all = ESC not reachable, stop
STATE_KNOWN_WAIT_S = 0.5         # wait this long for the first fresh gear/wheel frames before the pre-check
KEEP_FILES = 60

OUT_DIR = "/data/esc-probe-0027"
TARGET_FINGERPRINTS = ("HYUNDAI_ELANTRA_2022_NON_SCC",)


def guard_service(service: int, subfunc: int | None, did: int | None = None) -> None:
  """Layer 1: which (service, sub-function, DID) triples may even be turned into a frame."""
  if service not in ALLOWED_SERVICES:
    raise SafetyViolation(f"service 0x{service:02X} is not in the probe allowlist")
  if service == SVC_DIAGNOSTIC_SESSION_CONTROL and subfunc != ALLOWED_SESSION_SUBFUNC:
    raise SafetyViolation(f"session control sub-function {subfunc!r} refused (only 0x03 extended)")
  if service == SVC_SECURITY_ACCESS and subfunc != ALLOWED_SEC_SUBFUNC:
    raise SafetyViolation(f"security-access sub-function {subfunc!r} refused (only 0x01 requestSeed)")
  if service == SVC_WRITE_DATA_BY_IDENTIFIER and did != WRITE_DID:
    raise SafetyViolation(f"write to DID {did!r} refused (only 0x0103)")


def guard_frame(addr: int, dat: bytes, bus: int, readback: bytes | None = None) -> None:
  """Layer 2: the exact frame shapes this module may emit, checked immediately before can_send.

  ``readback`` is the 4 bytes read from 0x0103 in step 1. A 0x2E frame is only admitted when its data bytes are
  byte-identical to ``readback`` — so the write is a no-op by construction, not by convention."""
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
  if svc == SVC_SECURITY_ACCESS and ln == 2 and dat[2] == ALLOWED_SEC_SUBFUNC:
    return
  if svc == SVC_WRITE_DATA_BY_IDENTIFIER and ln == 7 and dat[2:4] == WRITE_DID.to_bytes(2, "big"):
    if readback is None:
      raise SafetyViolation("0x2E write without the step-1 read-back value")
    if dat[4:8] != bytes(readback):
      raise SafetyViolation(f"0x2E payload {dat[4:8].hex()} != step-1 read-back {bytes(readback).hex()}")
    return
  raise SafetyViolation(f"frame {dat.hex()} is not an allowlisted probe request")


def parse_read_did(resp_hex: str | None, did: int) -> bytes | None:
  """A usable positive 0x22 answer is exactly ``62 <did hi> <did lo>`` + 4 data bytes (the ESC's 0x0103 shape)."""
  if not resp_hex:
    return None
  try:
    body = bytes.fromhex(resp_hex)
  except ValueError:
    return None
  if len(body) == 7 and body[:3] == bytes([0x62, did >> 8, did & 0xFF]):
    return body[3:7]
  return None


class EscProbeClient:
  """Minimal UDS client for the probe. ``_tx`` is the ONLY place a frame is handed to can_send."""

  def __init__(self, can_send, can_recv, gate, now: Callable[[], float], deadline: float):
    self._can_send = can_send
    self._can_recv = can_recv
    self.gate = gate
    self.now = now
    self.deadline = deadline
    self.tx_log: list[str] = []
    self.silent = 0

  def _tx(self, dat: bytes, readback: bytes | None = None) -> None:
    self.gate.check()
    if self.now() > self.deadline:
      raise Abort("time budget exhausted")
    guard_frame(ESC_REQ_ADDR, dat, ESC_BUS, readback)
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

  def request(self, service: int, subfunc: int | None, payload: bytes = b"", did: int | None = None,
              readback: bytes | None = None) -> dict:
    guard_service(service, subfunc, did)
    req = bytes([service]) + (bytes([subfunc]) if subfunc is not None else b"") + bytes(payload)
    frame = build_single_frame(req)
    self.drain()
    self._tx(frame, readback)
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
    return self.request(SVC_READ_DATA_BY_IDENTIFIER, None, bytes([did >> 8, did & 0xFF]), did=did)

  def extended_session(self) -> dict:
    return self.request(SVC_DIAGNOSTIC_SESSION_CONTROL, ALLOWED_SESSION_SUBFUNC)

  def request_seed(self) -> dict:
    return self.request(SVC_SECURITY_ACCESS, ALLOWED_SEC_SUBFUNC, did=None)

  def write_did(self, did: int, value: bytes, readback: bytes) -> dict:
    """The ONLY write this module can make. ``value`` is mechanically forced to equal the step-1 read-back."""
    value, readback = bytes(value), bytes(readback)
    if did != WRITE_DID:
      raise SafetyViolation(f"write to DID {did!r} refused (only 0x0103)")
    if len(value) != 4:
      raise SafetyViolation(f"refusing to write {value.hex()}: 0x0103 is a 4-byte DID")
    if value != readback:
      raise SafetyViolation(f"refusing to write {value.hex()}: it must equal the step-1 read-back {readback.hex()}")
    return self.request(SVC_WRITE_DATA_BY_IDENTIFIER, None, did.to_bytes(2, "big") + value, did=did, readback=readback)


# ---------------------------------------------------------------------------------------------------------------------
# State + result files
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


def plan(state: dict, ignition_key: str) -> bool:
  """Pure: may this ignition be probed? Enabled by the operator, and not already probed in this cycle."""
  if not state.get("probe_enabled"):
    return False
  if state.get("done_ignition") == ignition_key:
    return False
  return True


def _raise_keyboard_interrupt(signum, frame):
  raise KeyboardInterrupt


# ---------------------------------------------------------------------------------------------------------------------
# Core (dependency-injected, so the tests drive the real code path)
# ---------------------------------------------------------------------------------------------------------------------
def run(can_send, can_recv, set_obd_multiplexing, *, fingerprint: str, car_fw=None, ignition_key: str | None,
        ignition_on: bool, out_dir: str = OUT_DIR, now: Callable[[], float] = time.monotonic,
        wall: Callable[[], float] = time.time,  # noqa: TID251 - wall clock wanted (file names)
        verify_mux_off: Callable[[], bool] | None = None,
        log_event: Callable[..., None] | None = None) -> dict:
  """Core probe. Returns a summary dict; never raises into card. All errors are recorded in the result JSON."""
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
  if not state.get("probe_enabled"):
    return {**summary, "skip": "not enabled"}
  if not plan(state, ignition_key):
    return {**summary, "skip": "already done (ignition)"}

  gate = VehicleGate(now)
  t0 = now()
  client = EscProbeClient(can_send, can_recv, gate, now, t0 + RUN_BUDGET_S)
  doc: dict = {"ignition_key": ignition_key, "fingerprint": fingerprint, "car_fw_abs": None,
              "read": None, "extended_session": None, "extended_retry": False, "current_value": None,
              "session_before_write": None,
              "seed_request": None, "seed": None, "already_unlocked": None,
              "write": None, "write_attempted": False, "reread": None, "reread_value": None,
              "value_changed": None, "tx": [], "duration_s": None, "aborted": None, "error": None,
              "mux_restored": None, "gate_speeds_moving": None}
  for fw in car_fw or []:
    if str(getattr(fw, "ecu", "")) == "abs":
      doc["car_fw_abs"] = bytes(fw.fwVersion).hex()

  # The pre-check and the probe body share one try/finally: any failure (including a CAN receive error) is recorded
  # in the result JSON, and the multiplexer is only ever turned off if it was turned on.
  precheck_failed = False
  mux_on = False
  old_sigterm = None
  summary["ran"] = True
  try:
    try:
      old_sigterm = signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
    except ValueError:
      old_sigterm = None  # not the main thread: finally still restores the multiplexer

    # Pre-check from live frames with the SAME definition the per-frame checks use (esc_diag.VehicleGate). Wait a
    # moment for the state to be KNOWN, then require parked + standstill; a violation does NOT consume the ignition.
    known_by = now() + STATE_KNOWN_WAIT_S
    while True:
      v = gate.violation()
      if "stale" in v and now() < known_by:
        for packet in can_recv(wait_for_one=True):
          for msg in packet:
            gate.feed(msg)
        continue
      break
    pre = gate.violation()
    doc["gate_speeds_moving"] = wheels_moving(gate.speeds) if gate.speeds is not None else None
    if pre:
      precheck_failed = True
      doc["aborted"] = "precheck: " + pre
      raise Abort(doc["aborted"])

    # Pre-check passed: mark the ignition BEFORE touching the multiplexer or sending anything, so a crash mid-probe
    # can never cause a second probe in this cycle.
    state["done_ignition"] = ignition_key
    _save_state(out_dir, state)

    mux_on = True
    set_obd_multiplexing(True)
    client.drain()

    # ---- step 1: read the CURRENT 0x0103 value (default session; extended retry only if refused) -------------
    r1 = client.read_did(DID_VARIANT_CODING)
    doc["read"] = r1
    current = parse_read_did(r1.get("resp"), DID_VARIANT_CODING)
    if current is None and r1.get("nrc") in (0x31, 0x7F):
      ext = client.extended_session()
      doc["extended_session"] = ext
      if ext.get("positive"):
        doc["extended_retry"] = True
        r1 = client.read_did(DID_VARIANT_CODING)
        doc["read"] = r1
        current = parse_read_did(r1.get("resp"), DID_VARIANT_CODING)
    doc["current_value"] = current.hex() if current is not None else None
    if current is None:
      # No usable read -> nothing is written. (The no-op write is only meaningful against the value we just read.)
      raise Abort("step-1 read of 0x0103 refused/missing; refusing to write")

    # ---- step 1.5: enter extended session (10 03) before the write --------------------------------------------
    # The vendor VariantCodingTable for this ECU writes as READ 22 0103 -> 10 03 -> 2E 0103. The read above ran in the
    # default session; if the retry path already entered extended (extended_retry), we are already there — do not send
    # 10 03 twice. A 10 03 that gets ANY answer (positive 50 03 or negative 7F 10 ..) continues; true silence (no frame
    # at all) means the ESC will not hold a write session, so abort before the write.
    if not doc["extended_retry"]:
      sess = client.extended_session()
      doc["session_before_write"] = sess
      if not sess.get("frames"):
        raise Abort("10 03 session got no response; not attempting the write")

    # ---- step 2: request the 0x27 seed ONLY (never the key) --------------------------------------------------
    r2 = client.request_seed()
    doc["seed_request"] = r2
    seed_resp = bytes.fromhex(r2.get("resp", "")) if r2.get("resp") else None
    doc["seed"] = seed_resp.hex() if seed_resp is not None else None
    if r2.get("positive") and seed_resp is not None and seed_resp[:2] == b"\x67\x01":
      doc["already_unlocked"] = seed_resp[2:] == b"\x00\x00\x00\x00"
    if seed_resp is None:
      # Silence (no frame at all): the ESC is not in a state to talk. Do NOT attempt the write.
      raise Abort("0x27 seed request got no response; not attempting the write")

    # ---- step 3: the no-op write — 0x2E 0x0103 + the exact bytes read in step 1 -------------------------------
    doc["write_attempted"] = True
    r3 = client.write_did(DID_VARIANT_CODING, current, current)
    doc["write"] = r3

    # ---- step 4: re-read and record whether the value changed (it must not) -----------------------------------
    r4 = client.read_did(DID_VARIANT_CODING)
    doc["reread"] = r4
    reread = parse_read_did(r4.get("resp"), DID_VARIANT_CODING)
    doc["reread_value"] = reread.hex() if reread is not None else None
    doc["value_changed"] = (reread is not None and reread != current)
  except Abort as e:
    doc["aborted"] = str(e)
  except SafetyViolation as e:
    doc["error"] = f"SafetyViolation: {e}"
  except Exception as e:  # never let this take card down
    doc["error"] = repr(e)
  finally:
    if mux_on:
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
  if precheck_failed:
    summary.update(ran=False, skip=doc["aborted"])
  path = _write_result(out_dir, "result", doc, wall())
  summary.update(path=path, current_value=doc["current_value"], session_before_write=doc["session_before_write"],
                 session_before_write_positive=bool((doc["session_before_write"] or {}).get("positive")),
                 seed=doc["seed"],
                 seed_positive=bool((doc["seed_request"] or {}).get("positive")),
                 write_attempted=doc["write_attempted"], write_positive=bool((doc["write"] or {}).get("positive")),
                 write_nrc=(doc["write"] or {}).get("nrc"), reread_value=doc["reread_value"],
                 value_changed=doc["value_changed"], aborted=doc["aborted"], error=doc["error"],
                 tx=len(client.tx_log), mux_restored=doc["mux_restored"], duration_s=doc["duration_s"])
  if log_event is not None:
    log_event("esc_probe_0027", **summary)
  return summary


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
  """Called by card right after run_esc_diag_from_card, still before FirmwareQueryDone. Never raises."""
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
      cloudlog.event("esc_probe_0027", **summary)
  except BaseException as e:
    try:
      set_obd_multiplexing(False)
    except BaseException:
      pass
    try:
      from openpilot.common.swaglog import cloudlog
      cloudlog.exception(f"esc_probe_0027 failed: {e!r}")
    except BaseException:
      pass
    if isinstance(e, KeyboardInterrupt):
      raise
