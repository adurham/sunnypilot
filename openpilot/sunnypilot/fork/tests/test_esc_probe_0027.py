"""
Fork: ESC 0x27 probe (fork/esc_probe_0027.py). Every hard rule of the brief has a test here; the mutants in
car-features/auto-esc-mutation.py prove each test is load-bearing (all must be killed).

The fake car below is driven through the REAL run() code path: real VehicleGate (imported from esc_diag), real
client, real guards, real scheduling state on disk. It answers like the logged ESC does (0x0103 -> 62 01 03 90 06
03 50; the ESC answers only on bus 1 with OBD multiplexing on).
"""
import ast
import inspect
import json
import os
import signal
import tempfile
from unittest import mock

from opendbc.car.can_definitions import CanData

from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.fork import esc_diag as D
from openpilot.sunnypilot.fork import esc_probe_0027 as E

CURRENT = b"\x90\x06\x03\x50"       # the known-good 0x0103 value on this car
SEED = b"\x11\x22\x33\x44"
DAY = 86400.


class FW:
  def __init__(self, ecu="abs", address=0x7D1, fw=b"CN ESC \t 100!"):
    self.ecu, self.address, self.fwVersion = ecu, address, fw


def lvr12(gear: int) -> bytes:
  return (gear << 32).to_bytes(8, "little")


def whl_raw(raw: list[int]) -> bytes:
  return sum((r & 0x3FFF) << (16 * i) for i, r in enumerate(raw)).to_bytes(8, "little")


def whl(kph: float) -> bytes:
  r = int(round(kph / 0.03125)) & 0x3FFF
  return whl_raw([r, r, r, r])


class FakeCar:
  """Clock + bus. Each can_recv() is 10 ms; every call carries fresh LVR12 + WHL_SPD11 unless frozen."""

  def __init__(self, gear=0, kph=0.):
    self.t = 1000.
    self.gear, self.kph = gear, kph
    self.raw = None
    self.mux = False
    self.mux_calls = []
    self.sent = []
    self.sent_t = []
    self.pending = []
    self.state_frozen = False
    self.freeze = set()
    self.mux_t = []
    self.on_recv = None
    self.send_exc = None
    self.recv_exc = None
    self.responsive = True
    # probe answer config
    self.current = CURRENT
    self.seed = SEED
    self.read_refused = False
    self.read_silent = False
    self.seed_refused = False
    self.seed_silent = False
    self.write_refused = False

  def now(self):
    return self.t

  def wall(self):
    return 1_790_000_000. + self.t

  def set_mux(self, on):
    self.mux_calls.append(on)
    self.mux_t.append(self.t)
    self.mux = on

  def can_send(self, msgs):
    if self.send_exc is not None:
      raise self.send_exc
    for m in msgs:
      self.sent.append((m.address, bytes(m.dat), m.src))
      self.sent_t.append(self.t)
      self._answer(m)

  def _q(self, dat, delay=0.005):
    self.pending.append((self.t + delay, dat.ljust(8, b"\xaa")))

  def _answer(self, m):
    if not (self.responsive and self.mux and m.src == 1 and m.address == 0x7D1):
      return
    d = bytes(m.dat)
    if d == D.FLOW_CONTROL_FRAME:
      return
    body = d[1:1 + d[0]]
    if not body:
      return
    svc = body[0]
    if svc == 0x22:
      did = (body[1] << 8) | body[2]
      if did == 0x0103 and not self.read_refused and not self.read_silent:
        self._q(bytes([7, 0x62, 0x01, 0x03]) + bytes(self.current))
      else:
        self._q(bytes([3, 0x7F, 0x22, 0x31]))
    elif svc == 0x10:
      self._q(bytes([6, 0x50, 0x03, 0x00, 0x32, 0x01, 0xF4]))
    elif svc == 0x27:
      if self.seed_silent:
        pass
      elif self.seed_refused:
        self._q(bytes([3, 0x7F, 0x27, 0x35]))
      else:
        self._q(bytes([6, 0x67, 0x01]) + bytes(self.seed))
    elif svc == 0x2E:
      if self.write_refused:
        self._q(bytes([3, 0x7F, 0x2E, 0x33]))
      else:
        self._q(bytes([3, 0x6E, 0x01, 0x03]))
    elif svc == 0x3E:
      self._q(bytes([3, 0x7F, 0x3E, 0x7F]))

  def can_recv(self, wait_for_one=False):
    self.t += 0.01
    if self.on_recv is not None:
      self.on_recv(self)
    if self.recv_exc is not None:
      raise self.recv_exc
    pkt = []
    if not self.state_frozen:
      w = whl_raw(self.raw) if self.raw is not None else whl(self.kph)
      pkt += [CanData(a, d, 0) for a, d in ((0x367, lvr12(self.gear)), (0x386, w)) if a not in self.freeze]
      pkt += [CanData(0x367, lvr12(0), 2), CanData(0x386, whl(0.), 128)]
    due = [p for p in self.pending if p[0] <= self.t]
    self.pending = [p for p in self.pending if p[0] > self.t]
    pkt += [CanData(0x7D9, d, 1) for _, d in due]
    return [pkt]


class Base(OpenpilotTestCase):
  def setUp(self):
    self.tmp = tempfile.TemporaryDirectory()
    self.out = self.tmp.name
    self.events = []
    self.enable()

  def tearDown(self):
    self.tmp.cleanup()

  def enable(self, on=True):
    with open(os.path.join(self.out, "state.json"), "w") as f:
      json.dump({"probe_enabled": on}, f)

  def fresh_state(self, on=True):
    self.out = tempfile.mkdtemp(dir=self.tmp.name)
    self.enable(on)

  def run_car(self, car, key="boot:1", ign=True, fp="HYUNDAI_ELANTRA_2022_NON_SCC", car_fw=None, verify=None, wall=None):
    return E.run(car.can_send, car.can_recv, car.set_mux, fingerprint=fp, car_fw=[FW()] if car_fw is None else car_fw,
                 ignition_key=key, ignition_on=ign, out_dir=self.out, now=car.now, wall=wall or car.wall,
                 verify_mux_off=verify or (lambda: not car.mux), log_event=lambda name, **kw: self.events.append((name, kw)))

  def result_doc(self):
    files = sorted(f for f in os.listdir(self.out) if f.endswith(".json") and f != "state.json")
    with open(os.path.join(self.out, files[-1])) as f:
      return json.load(f)

  def tx_frames(self, car):
    return [d for _, d, _ in car.sent]

  def write_frames(self, car):
    return [d for _, d, _ in car.sent if len(d) == 8 and d[0] == 7 and d[1] == 0x2E]


class TestHappyPath(Base):
  def test_happy_path(self):
    car = FakeCar()
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["aborted"])
    self.assertIsNone(s["error"])
    self.assertEqual(car.mux_calls, [True, False])
    self.assertTrue(s["mux_restored"])
    # sequence: read 0x0103, request seed, write no-op, re-read
    services = [d[1] for d in self.tx_frames(car)]
    self.assertEqual(services, [0x22, 0x27, 0x2E, 0x22])
    # no key is ever sent
    self.assertNotIn(0x02, [d[2] for d in self.tx_frames(car) if d[1] == 0x27])
    doc = self.result_doc()
    self.assertEqual(doc["current_value"], "90060350")
    self.assertEqual(doc["seed"], "670111223344")
    self.assertFalse(doc["already_unlocked"])
    self.assertTrue(doc["write_attempted"])
    self.assertTrue(doc["write"]["positive"])
    self.assertEqual(doc["reread_value"], "90060350")
    self.assertFalse(doc["value_changed"])
    self.assertEqual(self.events[-1][0], "esc_probe_0027")
    self.assertLessEqual(s["duration_s"], E.RUN_BUDGET_S)

  def test_write_payload_is_the_readback_value(self):
    car = FakeCar()
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    w = self.write_frames(car)
    self.assertEqual(len(w), 1)
    self.assertEqual(w[0], bytes([7, 0x2E, 0x01, 0x03]) + CURRENT)
    self.assertEqual(w[0][4:8], CURRENT)

  def test_write_payload_derives_from_the_read(self):
    car = FakeCar()
    car.current = b"\xab\xcd\xef\x01"
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertEqual(s["current_value"], "abcdef01")
    w = self.write_frames(car)
    self.assertEqual(len(w), 1)
    self.assertEqual(w[0][4:8], b"\xab\xcd\xef\x01")
    doc = self.result_doc()
    self.assertEqual(doc["reread_value"], "abcdef01")
    self.assertFalse(doc["value_changed"])

  def test_card_order_after_esc_read_before_firmware_query_done(self):
    from openpilot.selfdrive.car import card
    src = inspect.getsource(card.Car.__init__)
    i_esc = src.index("run_esc_diag_from_card(")
    i_probe = src.index("run_esc_probe_0027_from_card(")
    i_done = src.index('"FirmwareQueryDone"')
    self.assertLess(i_esc, i_probe)
    self.assertLess(i_probe, i_done)


class TestSeedOutcomes(Base):
  def test_seed_refused_still_writes(self):
    car = FakeCar()
    car.seed_refused = True
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["error"])
    self.assertFalse(s["seed_positive"])
    self.assertTrue(s["write_attempted"])          # a refused seed is informative with the write attempted
    self.assertIsNone(s["aborted"])                # neither the refused seed nor the accepted write aborted
    self.assertTrue(s["write_positive"])
    self.assertEqual(len(self.write_frames(car)), 1)
    doc = self.result_doc()
    self.assertEqual(doc["seed_request"]["nrc"], 0x35)
    self.assertEqual(doc["reread_value"], "90060350")

  def test_seed_silent_no_write(self):
    car = FakeCar()
    car.seed_silent = True
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIn("no response", s["aborted"])
    self.assertFalse(s["write_attempted"])
    self.assertEqual(self.write_frames(car), [])

  def test_all_zero_seed_already_unlocked(self):
    car = FakeCar()
    car.seed = b"\x00\x00\x00\x00"
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["aborted"])
    self.assertTrue(s["write_attempted"])
    doc = self.result_doc()
    self.assertTrue(doc["already_unlocked"])
    self.assertEqual(doc["seed"], "670100000000")
    self.assertEqual(len(self.write_frames(car)), 1)


class TestReadGate(Base):
  def test_read_refused_no_write(self):
    car = FakeCar()
    car.read_refused = True
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIn("step-1 read", s["aborted"])
    self.assertFalse(s["write_attempted"])
    self.assertEqual(self.write_frames(car), [])
    self.assertEqual(s["current_value"], None)
    # the extended retry was attempted (NRC 0x31) but the DID stayed refused
    doc = self.result_doc()
    self.assertEqual(doc["extended_session"]["positive"], True)
    self.assertTrue(doc["extended_retry"])

  def test_read_silent_no_write(self):
    car = FakeCar()
    car.read_silent = True
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertFalse(s["write_attempted"])
    self.assertEqual(self.write_frames(car), [])

  def test_short_read_answer_refuses_write(self):
    """A positive 0x22 answer of the wrong shape (not 62 0103 + 4) must not be used as a write payload."""
    self.assertEqual(E.parse_read_did("6201039006", 0x0103), None)     # 2 data bytes
    self.assertEqual(E.parse_read_did("620103900603", 0x0103), None)   # 3 data bytes
    self.assertEqual(E.parse_read_did("62F19090060350", 0x0103), None)  # wrong DID
    self.assertEqual(E.parse_read_did("7F2231", 0x0103), None)         # NRC
    self.assertEqual(E.parse_read_did(None, 0x0103), None)
    self.assertEqual(E.parse_read_did("62010390060350", 0x0103), CURRENT)


class TestGuards(Base):
  def test_service_guard_fuzz(self):
    """Every (service, sub-function, DID) triple through guard_service: only the exact allowlist passes."""
    allowed = []
    for svc in range(256):
      for sub in (None, *range(256)):
        for did in (None, 0x0103, 0x0104):
          try:
            E.guard_service(svc, sub, did)
            allowed.append((svc, sub, did))
          except D.SafetyViolation:
            pass
    self.assertEqual({s for s, _, _ in allowed}, {0x22, 0x3E, 0x10, 0x27, 0x2E})
    self.assertEqual({sub for s, sub, _ in allowed if s == 0x10}, {0x03})
    self.assertEqual({sub for s, sub, _ in allowed if s == 0x27}, {0x01})
    self.assertEqual({did for s, _, did in allowed if s == 0x2E}, {0x0103})
    with self.assertRaises(D.SafetyViolation):
      E.guard_service(0x27, 0x02, None)      # sendKey is never allowed
    with self.assertRaises(D.SafetyViolation):
      E.guard_service(0x2E, None, 0x0104)    # write to any other DID
    for bad in (0x31, 0x14, 0x11, 0x2F, 0x28, 0x85, 0x34, 0x19, 0x23):
      with self.assertRaises(D.SafetyViolation):
        E.guard_service(bad, None, None)

  def test_frame_guard_fuzz(self):
    """The per-frame guard alone admits exactly the probe shapes; sendKey / other DIDs / mismatched payloads raise."""
    admitted = []
    for b0 in range(256):
      for b1 in range(256):
        for b2 in (0x00, 0x01, 0x02, 0x03, 0x80):
          dat = bytes([b0, b1, b2, 0xFF, 0, 0, 0, 0])
          for rb in (None, CURRENT):
            try:
              E.guard_frame(0x7D1, dat, 1, rb)
              admitted.append((dat, rb))
            except D.SafetyViolation:
              pass
    for d, _ in admitted:
      self.assertTrue(d == D.FLOW_CONTROL_FRAME
                      or (d[0], d[1]) in ((3, 0x22), (2, 0x3E), (2, 0x10), (2, 0x27), (7, 0x2E)), d.hex())
      if d[1] == 0x27:
        self.assertEqual(d[2], 0x01)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([2, 0x27, 0x02, 0, 0, 0, 0, 0]), 1)   # sendKey
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([7, 0x2E, 0x01, 0x04]) + CURRENT, 1, CURRENT)  # other DID
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([7, 0x2E, 0x01, 0x03]) + CURRENT, 1, None)     # missing read-back
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([7, 0x2E, 0x01, 0x03, 1, 2, 3, 4]), 1, CURRENT)  # payload != read-back
    for bad in (0x31, 0x14, 0x11, 0x2F, 0x28, 0x85, 0x34, 0x3D, 0x23):
      for ln in range(1, 8):
        with self.assertRaises(D.SafetyViolation):
          E.guard_frame(0x7D1, bytes([ln, bad, 0, 0, 0, 0, 0, 0]), 1, CURRENT)
    for addr, bus in ((0x7D1, 0), (0x7D1, 2), (0x7D0, 1), (0x7DF, 1), (0x340, 0)):
      with self.assertRaises(D.SafetyViolation):
        E.guard_frame(addr, bytes([3, 0x22, 0x01, 0x03, 0, 0, 0, 0]), bus)

  def test_write_did_refuses_anything_but_the_readback(self):
    car = FakeCar()
    car.set_mux(True)
    gate = D.VehicleGate(car.now)
    gate.feed(CanData(0x367, lvr12(0), 0))
    gate.feed(CanData(0x386, whl(0.), 0))
    c = E.EscProbeClient(car.can_send, car.can_recv, gate, car.now, car.now() + 99)
    with self.assertRaises(D.SafetyViolation):
      c.write_did(0x0104, CURRENT, CURRENT)
    with self.assertRaises(D.SafetyViolation):
      c.write_did(0x0103, b"\x01\x02\x03\x04", CURRENT)
    with self.assertRaises(D.SafetyViolation):
      c.write_did(0x0103, CURRENT[:3], CURRENT)
    self.assertEqual(car.sent, [])

  def test_single_tx_site_and_no_mode_changes(self):
    tree = ast.parse(inspect.getsource(E))
    calls = [(n.lineno, n.func.attr if isinstance(n.func, ast.Attribute) else n.func.id) for n in ast.walk(tree)
             if isinstance(n, ast.Call) and getattr(n.func, "attr", getattr(n.func, "id", "")) in
             ("_can_send", "can_send", "can_send_many", "set_safety_mode", "set_safety_model", "controlWrite",
              "set_alternative_experience", "put_bool", "put")]
    self.assertEqual([c[1] for c in calls], ["_can_send"], calls)
    tx = inspect.getsource(E.EscProbeClient._tx)
    self.assertLess(tx.index("self.gate.check()"), tx.index("self._can_send("))
    self.assertLess(tx.index("guard_frame("), tx.index("self._can_send("))
    self.assertNotIn("set_safety", inspect.getsource(E))


class TestStationaryOnly(Base):
  def test_not_in_park_sends_nothing(self):
    for gear in (5, 6, 7, 8):
      car = FakeCar(gear=gear)
      s = self.run_car(car, key=f"k{gear}")
      self.assertFalse(s["ran"])
      self.assertEqual(car.sent, [])
      self.assertEqual(car.mux_calls, [])

  def test_rolling_in_park_sends_nothing(self):
    car = FakeCar(kph=1.0)
    s = self.run_car(car)
    self.assertFalse(s["ran"])
    self.assertEqual(car.sent, [])
    self.assertEqual(car.mux_calls, [])

  def test_two_wheel_creep_in_park_sends_nothing(self):
    for raw in ([13, 13, 0, 0], [97, 0, 0, 0]):
      with self.subTest(raw=raw):
        car = FakeCar()
        car.raw = raw
        s = self.run_car(car, key=f"creep{raw}")
        self.assertFalse(s["ran"])
        self.assertEqual(car.sent, [])
        self.assertEqual(car.mux_calls, [])

  def test_parked_noise_does_not_block_the_probe(self):
    car = FakeCar()
    car.raw = [0, 0, 0, 1]
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["aborted"])

  def test_unknown_state_sends_nothing(self):
    car = FakeCar()
    car.state_frozen = True
    s = self.run_car(car)
    self.assertFalse(s["ran"])
    self.assertEqual(car.sent, [])

  def test_uses_esc_diag_standstill_definition(self):
    self.assertIs(E.wheels_moving, D.wheels_moving)
    self.assertIs(E.VehicleGate, D.VehicleGate)

  def _mid(self, change):
    car = FakeCar()
    changed = {}

    def hook(c):
      if len(c.sent) >= 2 and "n" not in changed:
        change(c)
        changed["n"] = len(c.sent)
    car.on_recv = hook
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNotNone(s["aborted"])
    return s, car, changed["n"]

  def test_abort_when_car_starts_moving_mid_probe(self):
    s, car, n = self._mid(lambda c: setattr(c, "kph", 1.0))
    self.assertIn("moving", s["aborted"])
    self.assertFalse(s["write_attempted"])
    self.assertEqual(len(car.sent), n)
    self.assertEqual(car.mux_calls, [True, False])

  def test_abort_when_leaving_park_mid_probe(self):
    s, car, n = self._mid(lambda c: setattr(c, "gear", 7))
    self.assertIn("Park", s["aborted"])
    self.assertEqual(len(car.sent), n)

  def test_abort_when_state_goes_stale(self):
    """The ESC stops answering mid-probe and the bus state freezes: the wait for the write answer ends in a stale abort."""
    car = FakeCar()
    moved = {}

    def hook(c):
      if c.sent and c.sent[-1][1][:3] == bytes([7, 0x2E, 0x01]) and "t" not in moved:
        c.state_frozen = True
        c.responsive = False
        moved["t"] = c.t
    car.on_recv = hook
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNotNone(s["aborted"])
    self.assertIn("stale", s["aborted"])
    self.assertTrue(all(t <= moved["t"] + D.STATE_MAX_AGE_S for t in car.sent_t))
    self.assertEqual(car.mux_calls, [True, False])


class TestNeverInterferes(Base):
  def test_not_enabled_sends_nothing(self):
    self.enable(False)
    car = FakeCar()
    s = self.run_car(car)
    self.assertEqual(s["skip"], "not enabled")
    self.assertEqual(car.sent, [])
    self.assertEqual(car.mux_calls, [])
    # missing state file entirely: same
    os.remove(os.path.join(self.out, "state.json"))
    s = self.run_car(car, key="boot:2")
    self.assertEqual(s["skip"], "not enabled")
    self.assertEqual(car.sent, [])

  def test_wrong_car_ignition_off_disabled(self):
    car = FakeCar()
    self.assertEqual(self.run_car(car, fp="TOYOTA_RAV4")["skip"], "not the target car")
    self.assertEqual(self.run_car(car, ign=False)["skip"], "ignition off")
    self.assertEqual(self.run_car(car, key=None)["skip"], "ignition cycle unknown")
    open(os.path.join(self.out, "DISABLE"), "w").close()
    self.assertEqual(self.run_car(car)["skip"], "disabled")
    self.assertEqual(car.sent, [])
    self.assertEqual(car.mux_calls, [])

  def test_mux_restored_on_send_exception(self):
    car = FakeCar()
    car.send_exc = RuntimeError("sendcan broke")
    s = self.run_car(car)
    self.assertIn("sendcan broke", s["error"])
    self.assertEqual(car.mux_calls, [True, False])
    self.assertTrue(self.result_doc()["mux_restored"])

  def test_mux_restored_on_recv_exception(self):
    car = FakeCar()
    car.recv_exc = RuntimeError("recvcan broke")
    s = self.run_car(car)
    self.assertIn("recvcan broke", s["error"])
    # the pre-check itself faulted before the multiplexer was ever touched
    self.assertEqual(car.mux_calls, [])
    self.assertIsNone(self.result_doc()["mux_restored"])

  def test_mux_restored_on_sigterm(self):
    car = FakeCar()

    def hook(c):
      if len(c.sent) == 2:
        os.kill(os.getpid(), signal.SIGTERM)
    car.on_recv = hook
    prev = signal.getsignal(signal.SIGTERM)
    with self.assertRaises(KeyboardInterrupt):
      self.run_car(car)
    self.assertEqual(car.mux_calls, [True, False])
    self.assertIs(signal.getsignal(signal.SIGTERM), prev)

  def test_unverified_restore_is_reported(self):
    car = FakeCar()
    s = self.run_car(car, verify=lambda: False)
    self.assertFalse(s["mux_restored"])

  def test_result_json_written_on_every_path(self):
    # happy
    car = FakeCar()
    self.run_car(car, key="boot:1")
    self.assertTrue(self.result_doc()["current_value"])
    # abort (read refused)
    self.fresh_state()
    car = FakeCar()
    car.read_refused = True
    self.run_car(car, key="boot:1")
    self.assertIsNotNone(self.result_doc()["aborted"])
    # precheck skip
    self.fresh_state()
    car = FakeCar(gear=5)
    self.run_car(car, key="boot:1")
    self.assertIn("precheck", self.result_doc()["aborted"])

  def test_budget(self):
    car = FakeCar()
    with mock.patch.object(E, "RUN_BUDGET_S", 0.015):
      s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIn("budget", s["aborted"])
    self.assertEqual(car.sent, [])                      # nothing was sent once the budget was gone
    self.assertEqual(car.mux_calls, [True, False])       # but the multiplexer still came back off

  def test_silent_esc_stops_fast(self):
    car = FakeCar()
    car.read_silent = True
    s = self.run_car(car)
    self.assertLess(s["duration_s"], 1.5)
    self.assertEqual(car.mux_calls, [True, False])

  def test_run_from_card_pc_returns_early(self):
    class CP:
      carFingerprint = "HYUNDAI_ELANTRA_2022_NON_SCC"
      carFw = []
    calls = []
    E.run_from_card(CP, (None, None), calls.append)
    self.assertEqual(calls, [])

  def test_run_from_card_swallows_exceptions(self):
    class CP:
      carFingerprint = "HYUNDAI_ELANTRA_2022_NON_SCC"
      carFw = []
    calls = []
    with mock.patch("openpilot.common.hardware.PC", False), \
         mock.patch.object(E, "_ignition_from_services", side_effect=RuntimeError("boom")):
      E.run_from_card(CP, (None, None), calls.append)   # must not raise
    self.assertEqual(calls, [False])                    # set_obd_multiplexing(False) in the except


class TestScheduling(Base):
  def test_once_per_ignition(self):
    car = FakeCar()
    self.run_car(car, key="boot:1")
    n = len(car.sent)
    s = self.run_car(car, key="boot:1")
    self.assertFalse(s["ran"])
    self.assertEqual(s["skip"], "already done (ignition)")
    self.assertEqual(len(car.sent), n)

  def test_new_ignition_runs_again(self):
    car = FakeCar()
    self.run_car(car, key="boot:1")
    n = len(car.sent)
    s = self.run_car(car, key="boot:2")
    self.assertTrue(s["ran"])
    self.assertGreater(len(car.sent), n)

  def test_ignition_marked_even_if_crash(self):
    car = FakeCar()
    car.send_exc = RuntimeError("x")
    self.run_car(car, key="boot:1")
    car.send_exc = None
    with open(os.path.join(self.out, "state.json")) as f:
      self.assertEqual(json.load(f)["done_ignition"], "boot:1")
    n = len(car.sent)
    self.assertFalse(self.run_car(car, key="boot:1")["ran"])
    self.assertEqual(len(car.sent), n)

  def test_precheck_skip_does_not_consume_the_ignition(self):
    car = FakeCar(gear=5)
    s = self.run_car(car, key="boot:1")
    self.assertFalse(s["ran"])
    self.assertIn("precheck", s["aborted"])
    with open(os.path.join(self.out, "state.json")) as f:
      self.assertNotIn("done_ignition", json.load(f))
    car.gear = 0
    s = self.run_car(car, key="boot:1")
    self.assertTrue(s["ran"])
    self.assertGreater(s["tx"], 0)
    self.assertFalse(self.run_car(car, key="boot:1")["ran"])   # and then once per ignition as before

  def test_plan_pure(self):
    self.assertFalse(E.plan({}, "a"))
    self.assertFalse(E.plan({"probe_enabled": True, "done_ignition": "a"}, "a"))
    self.assertTrue(E.plan({"probe_enabled": True, "done_ignition": "a"}, "b"))
    self.assertFalse(E.plan({"probe_enabled": False, "done_ignition": "a"}, "b"))

  def test_storage_bounded(self):
    car = FakeCar()
    for i in range(E.KEEP_FILES + 5):
      self.run_car(car, key=f"boot:{i}", wall=lambda i=i: car.wall() + i * DAY)
    files = [f for f in os.listdir(self.out) if f.endswith(".json") and f != "state.json"]
    self.assertLessEqual(len(files), E.KEEP_FILES)
