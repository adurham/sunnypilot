"""
Fork: automatic read-only ESC UDS read (fork/esc_diag.py). Every hard rule of the brief has a test here, and
car-features/auto-esc-mutation.py proves each test is load-bearing (one mutant per rule, all must be killed).

The fake car below is driven through the REAL run() code path: real VehicleGate, real client, real guards, real
scheduling state on disk. It answers like the logged ESC does (route 0000012e seg 0: F100 is a 30-byte multi-frame
answer, 0x3E is refused with NRC 0x7F, the ESC answers only on bus 1 with OBD multiplexing on).
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
from openpilot.sunnypilot.fork import esc_diag as E

F100 = b"\xf1\x00CN ESC \t 100!\x05\x01 58910-IB000"  # as logged in carFw
DAY = 86400.


class FW:
  def __init__(self, ecu="abs", address=0x7D1, fw=F100):
    self.ecu, self.address, self.fwVersion = ecu, address, fw


def lvr12(gear: int) -> bytes:
  return (gear << 32).to_bytes(8, "little")


def whl(kph: float) -> bytes:
  raw = int(round(kph / 0.03125)) & 0x3FFF
  return sum(raw << (16 * i) for i in range(4)).to_bytes(8, "little")


class FakeCar:
  """Clock + bus. Each can_recv() is 10 ms; every call carries fresh LVR12 + WHL_SPD11 unless frozen."""

  def __init__(self, gear=0, kph=0.):
    self.t = 1000.
    self.gear, self.kph = gear, kph
    self.mux = False
    self.mux_calls: list[bool] = []
    self.sent: list[tuple[int, bytes, int]] = []
    self.sent_t: list[float] = []
    self.pending: list[tuple[float, bytes]] = []
    self.state_frozen = False
    self.freeze = set()          # addresses whose frames stop arriving
    self.mux_t: list[float] = []
    self.on_recv = None          # hook(car) called on every recv (to change state mid-read)
    self.send_exc: Exception | None = None
    self.recv_exc: BaseException | None = None
    self.mf_left: list[bytes] = []
    self.responsive = True

  def now(self):
    return self.t

  def wall(self):
    return 1_790_000_000. + self.t

  def set_mux(self, on: bool):
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

  def _q(self, dat: bytes, delay=0.005):
    self.pending.append((self.t + delay, dat.ljust(8, b"\xaa")))

  def _answer(self, m: CanData):
    if not (self.responsive and self.mux and m.src == 1 and m.address == 0x7D1):
      return
    d = bytes(m.dat)
    if d == E.FLOW_CONTROL_FRAME:
      for i, cf in enumerate(self.mf_left):
        self._q(cf, 0.01 * (i + 1))
      self.mf_left = []
      return
    body = d[1:1 + d[0]]
    svc = body[0]
    if svc == 0x22:
      did = (body[1] << 8) | body[2]
      if did == 0xF100:
        payload = b"\x62\xf1\x00" + F100[2:]
        self._q(bytes([0x10 | (len(payload) >> 8), len(payload) & 0xFF]) + payload[:6])
        rest, n, self.mf_left = payload[6:], 1, []
        while rest:
          self.mf_left.append(bytes([0x20 | (n & 0xF)]) + rest[:7])
          rest, n = rest[7:], n + 1
      elif did == 0xF18C:
        self._q(bytes([0x03, 0x7F, 0x22, 0x78]))           # responsePending, then the answer
        self._q(bytes([0x07, 0x62, 0xF1, 0x8C, 1, 2, 3, 4]), 0.5)
      elif did in (0xF187, 0xF189):
        self._q(bytes([0x07, 0x62, body[1], body[2], 0x41, 0x42, 0x43, 0x44]))
      elif 0x0100 <= did < 0x0110:
        self._q(bytes([0x03, 0x7F, 0x22, 0x31]))           # out of range in default, ok in extended
      else:
        self._q(bytes([0x03, 0x7F, 0x22, 0x31]))
    elif svc == 0x10:
      self._q(bytes([0x06, 0x50, 0x03, 0x00, 0x32, 0x01, 0xF4]))
    elif svc == 0x19:
      self._q(bytes([0x07, 0x59, 0x02, 0xFF, 0x12, 0x34, 0x56, 0x08]))
    elif svc == 0x3E:
      self._q(bytes([0x03, 0x7F, 0x3E, 0x7F]))

  def can_recv(self, wait_for_one=False):
    self.t += 0.01
    if self.on_recv is not None:
      self.on_recv(self)
    if self.recv_exc is not None:
      raise self.recv_exc
    pkt = []
    if not self.state_frozen:
      pkt += [CanData(a, d, 0) for a, d in ((0x367, lvr12(self.gear)), (0x386, whl(self.kph))) if a not in self.freeze]
      # the same frames echoed on the camera side and as TX echoes must be ignored by the gate
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

  def tearDown(self):
    self.tmp.cleanup()

  def run_car(self, car, key="boot:1", ign=True, fp="HYUNDAI_ELANTRA_2022_NON_SCC", car_fw=None, verify=None, wall=None):
    return E.run(car.can_send, car.can_recv, car.set_mux, fingerprint=fp, car_fw=[FW()] if car_fw is None else car_fw,
                 ignition_key=key, ignition_on=ign, out_dir=self.out, now=car.now, wall=wall or car.wall,
                 verify_mux_off=verify or (lambda: not car.mux), log_event=lambda name, **kw: self.events.append((name, kw)))

  def assert_only_readonly(self, sent):
    for addr, dat, bus in sent:
      self.assertEqual((addr, bus), (0x7D1, 1))
      E.guard_frame(addr, dat, bus)  # raises on anything else
      ok = dat == E.FLOW_CONTROL_FRAME or (dat[0] <= 7 and dat[1] in (0x22, 0x3E) or dat[1:3] in (b"\x10\x03", b"\x19\x02"))
      self.assertTrue(ok, dat.hex())

  def result_doc(self):
    files = sorted(f for f in os.listdir(self.out) if f.endswith(".json") and f != "state.json")
    with open(os.path.join(self.out, files[-1])) as f:
      return json.load(f)


class TestHappyPath(Base):
  def test_full_read_and_dtc(self):
    car = FakeCar()
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["aborted"])
    self.assertIsNone(s["error"])
    self.assertTrue(s["full"] and s["dtc"])
    self.assertEqual(car.mux_calls, [True, False])   # multiplexer on for the read, back off after, verified
    self.assertTrue(s["mux_restored"])
    self.assert_only_readonly(car.sent)
    services = {d[1] for _, d, _ in car.sent if d != E.FLOW_CONTROL_FRAME}
    self.assertEqual(services, {0x22, 0x10, 0x19})
    doc = self.result_doc()
    by = {r["did"]: r for r in doc["results"]}
    self.assertEqual(bytes.fromhex(by["F100"]["resp"]), b"\x62\xf1\x00" + F100[2:])   # multi-frame reassembled
    self.assertEqual(by["F18C"]["resp"], "62f18c01020304")                            # waited through 0x78
    self.assertEqual(by["0100"]["nrc"], 0x31)
    self.assertTrue("0100@ext" in by)  # retried in extended
    self.assertEqual(doc["dtc"]["resp"], "5902ff12345608")
    self.assertEqual(len(E.did_table()), 70)
    self.assertEqual(self.events[-1][0], "esc_uds_read")
    self.assertLess(s["duration_s"], E.RUN_BUDGET_S)

  def test_card_order_before_firmware_query_done(self):
    """The read must sit in the fingerprint window: after get_car (CP known), before FirmwareQueryDone (pandad then
    switches to the car safety mode)."""
    from openpilot.selfdrive.car import card
    src = inspect.getsource(card.Car.__init__)
    i_get, i_esc, i_done = src.index("get_car("), src.index("run_esc_diag_from_card("), src.index('"FirmwareQueryDone"')
    self.assertLess(i_get, i_esc)
    self.assertLess(i_esc, i_done)


class TestReadOnly(Base):
  def test_service_fuzz(self):
    """Every service byte x sub-function through the public request path: only the allowlist reaches the bus."""
    reached = set()
    for svc in range(256):
      for sub in (None, 0x00, 0x01, 0x02, 0x03, 0x04, 0x60, 0x80, 0xFF):
        car = FakeCar()
        car.set_mux(True)
        gate = E.VehicleGate(car.now)
        c = E.EscUdsClient(car.can_send, car.can_recv, gate, car.now, car.now() + 99)
        for payload in (b"", b"\xff", b"\xf1\x00"):
          try:
            c.request(svc, sub, payload)
          except (E.SafetyViolation, ValueError, E.Abort):
            pass
        for _, d, _ in car.sent:
          if d != E.FLOW_CONTROL_FRAME:
            reached.add((d[1], d[2] if sub is not None else None))
    services = {s for s, _ in reached}
    self.assertEqual(services, {0x22, 0x3E, 0x10, 0x19})
    self.assertEqual({sub for s, sub in reached if s == 0x10}, {0x03})
    self.assertEqual({sub for s, sub in reached if s == 0x19}, {0x02})

  def test_service_guard_alone(self):
    """Layer 1 on its own (the frame guard below is layer 2): every non-allowlisted service/sub-function raises."""
    allowed = set()
    for svc in range(256):
      for sub in (None, *range(256)):
        try:
          E.guard_service(svc, sub)
          allowed.add((svc, sub))
        except E.SafetyViolation:
          pass
    self.assertEqual({s for s, _ in allowed}, {0x22, 0x3E, 0x10, 0x19})
    self.assertEqual({sub for s, sub in allowed if s == 0x10}, {0x03})
    self.assertEqual({sub for s, sub in allowed if s == 0x19}, {0x02})

  def test_frame_guard_fuzz(self):
    """The per-frame guard alone (the last check before can_send) admits exactly the read-only shapes."""
    admitted = []
    for b0 in range(256):
      for b1 in range(256):
        for b2 in (0x00, 0x02, 0x03, 0x80, 0xF1):
          dat = bytes([b0, b1, b2, 0xFF, 0, 0, 0, 0])
          try:
            E.guard_frame(0x7D1, dat, 1)
            admitted.append(dat)
          except E.SafetyViolation:
            pass
    for d in admitted:
      self.assertTrue(d == E.FLOW_CONTROL_FRAME or (d[0], d[1]) in ((3, 0x22), (2, 0x3E), (2, 0x10), (3, 0x19)), d.hex())
      if d[1] == 0x10:
        self.assertEqual(d[2], 0x03)
      if d[1] == 0x19:
        self.assertEqual(d[2:4], b"\x02\xff")
    for bad in (0x2E, 0x27, 0x31, 0x11, 0x14, 0x2F, 0x28, 0x85, 0x34, 0x3D, 0x23):
      for ln in range(1, 8):
        with self.assertRaises(E.SafetyViolation):
          E.guard_frame(0x7D1, bytes([ln, bad, 0, 0, 0, 0, 0, 0]), 1)
    for addr, bus in ((0x7D1, 0), (0x7D1, 2), (0x7D0, 1), (0x7DF, 1), (0x340, 0)):
      with self.assertRaises(E.SafetyViolation):
        E.guard_frame(addr, bytes([3, 0x22, 0xF1, 0x00, 0, 0, 0, 0]), bus)
    with self.assertRaises(E.SafetyViolation):
      E.guard_frame(0x7D1, bytes([0x30, 0x01, 0, 0, 0, 0, 0, 0]), 1)  # any other flow control

  def test_single_tx_site_and_no_mode_changes(self):
    tree = ast.parse(inspect.getsource(E))
    calls = [(n.lineno, n.func.attr if isinstance(n.func, ast.Attribute) else n.func.id) for n in ast.walk(tree)
             if isinstance(n, ast.Call) and getattr(n.func, "attr", getattr(n.func, "id", "")) in
             ("_can_send", "can_send", "can_send_many", "set_safety_mode", "set_safety_model", "controlWrite",
              "set_alternative_experience", "put_bool", "put")]
    self.assertEqual([c[1] for c in calls], ["_can_send"], calls)
    tx = inspect.getsource(E.EscUdsClient._tx)
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

  def test_not_parked_costs_no_startup_time(self):
    car = FakeCar(gear=5, kph=40.)
    t0 = car.t
    self.run_car(car)
    self.assertLess(car.t - t0, 0.05)

  def test_rolling_in_park_sends_nothing(self):
    car = FakeCar(kph=0.03125)       # one LSB of wheel speed
    s = self.run_car(car)
    self.assertFalse(s["ran"])
    self.assertEqual(car.sent, [])

  def test_unknown_state_sends_nothing(self):
    car = FakeCar()
    car.state_frozen = True
    s = self.run_car(car)
    self.assertFalse(s["ran"])
    self.assertEqual(car.sent, [])

  def _mid_read(self, change, stale=False):
    car = FakeCar()
    changed_at = {}

    def hook(c):
      if len(c.sent) >= 10 and "n" not in changed_at:
        change(c)
        changed_at["n"], changed_at["t"] = len(c.sent), c.t
    car.on_recv = hook
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNotNone(s["aborted"])
    if stale:
      # a stale state can only be detected once it is older than STATE_MAX_AGE_S: nothing after that
      self.assertTrue(all(t <= changed_at["t"] + E.STATE_MAX_AGE_S for t in car.sent_t))
    else:
      self.assertEqual(len(car.sent), changed_at["n"])   # not one frame after the change
    self.assertEqual(car.mux_calls, [True, False])
    self.assertTrue(s["mux_restored"])
    return s

  def test_abort_when_car_starts_moving(self):
    s = self._mid_read(lambda c: setattr(c, "kph", 1.0))
    self.assertIn("moving", s["aborted"])

  def test_abort_when_leaving_park(self):
    s = self._mid_read(lambda c: setattr(c, "gear", 7))
    self.assertIn("Park", s["aborted"])

  def test_abort_when_state_goes_stale(self):
    s = self._mid_read(lambda c: setattr(c, "state_frozen", True), stale=True)
    self.assertIn("stale", s["aborted"])

  def test_gear_stale_alone_aborts(self):
    s = self._mid_read(lambda c: c.freeze.add(0x367), stale=True)
    self.assertIn("gear", s["aborted"])

  def test_speed_stale_alone_aborts(self):
    s = self._mid_read(lambda c: c.freeze.add(0x386), stale=True)
    self.assertIn("wheel speed", s["aborted"])

  def test_abort_while_waiting_for_answer(self):
    """F18C is answered only after 0.5 s (NRC 0x78 responsePending). If the car starts moving during that wait, the
    read must abort immediately, not after the answer."""
    car = FakeCar()
    moved = {}

    def hook(c):
      if c.sent and c.sent[-1][1][:4] == bytes([3, 0x22, 0xF1, 0x8C]) and "t" not in moved:
        c.kph = 2.0
        moved["t"] = c.t
    car.on_recv = hook
    s = self.run_car(car)
    self.assertIn("moving", s["aborted"])
    self.assertEqual(car.sent[-1][1][:4], bytes([3, 0x22, 0xF1, 0x8C]))
    self.assertLessEqual(car.mux_t[-1] - moved["t"], 0.02)   # mux back off within one receive cycle

  def test_aborted_read_is_not_complete(self):
    self._mid_read(lambda c: setattr(c, "kph", 1.0))
    with open(os.path.join(self.out, "state.json")) as f:
      st = json.load(f)
    self.assertFalse(list(st["fw"].values())[0]["complete"])


class TestNeverInterferes(Base):
  def test_mux_restored_on_send_exception(self):
    car = FakeCar()
    car.send_exc = RuntimeError("sendcan broke")
    s = self.run_car(car)
    self.assertIn("sendcan broke", s["error"])
    self.assertEqual(car.mux_calls, [True, False])

  def test_mux_restored_on_sigterm(self):
    car = FakeCar()

    def hook(c):
      if len(c.sent) == 5:
        os.kill(os.getpid(), signal.SIGTERM)
    car.on_recv = hook
    prev = signal.getsignal(signal.SIGTERM)
    with self.assertRaises(KeyboardInterrupt):
      self.run_car(car)
    self.assertEqual(car.mux_calls, [True, False])
    self.assertIs(signal.getsignal(signal.SIGTERM), prev)   # handler restored

  def test_unverified_restore_is_reported(self):
    car = FakeCar()
    s = self.run_car(car, verify=lambda: False)
    self.assertFalse(s["mux_restored"])

  def test_run_from_card_never_raises(self):
    class CP:
      carFingerprint = "HYUNDAI_ELANTRA_2022_NON_SCC"
      carFw = []
    calls = []
    E.run_from_card(CP, (None, None), calls.append)   # PC: returns before touching anything
    self.assertEqual(calls, [])

  def test_budget(self):
    car = FakeCar()
    with mock.patch.object(E, "RUN_BUDGET_S", 0.3):
      s = self.run_car(car)
    self.assertIn("budget", s["aborted"])
    self.assertLessEqual(max(car.sent_t) - min(car.sent_t), 0.3)
    self.assertEqual(car.mux_calls, [True, False])

  def test_silent_esc_stops_fast(self):
    car = FakeCar()
    car.responsive = False     # e.g. multiplexer did not switch: 4 requests, then stop (startup delayed ~1 s, not 18 s)
    s = self.run_car(car)
    self.assertIn("silent", s["aborted"])
    self.assertEqual(len(car.sent), E.SILENT_ABORT_N)
    self.assertLess(s["duration_s"], 2.)
    self.assertEqual(car.mux_calls, [True, False])

  def test_wrong_car_ignition_off_disabled(self):
    car = FakeCar()
    self.assertEqual(self.run_car(car, fp="TOYOTA_RAV4")["skip"], "not the target car")
    self.assertEqual(self.run_car(car, ign=False)["skip"], "ignition off")
    self.assertEqual(self.run_car(car, key=None)["skip"], "ignition cycle unknown")
    open(os.path.join(self.out, "DISABLE"), "w").close()
    self.assertEqual(self.run_car(car)["skip"], "disabled")
    self.assertEqual(car.sent, [])
    self.assertEqual(car.mux_calls, [])


class TestScheduling(Base):
  def test_once_per_ignition(self):
    car = FakeCar()
    self.run_car(car, key="boot:1")
    n = len(car.sent)
    s = self.run_car(car, key="boot:1")
    self.assertFalse(s["ran"])
    self.assertEqual(len(car.sent), n)

  def test_ignition_marked_even_if_crash(self):
    car = FakeCar()
    car.send_exc = RuntimeError("x")
    self.run_car(car, key="boot:1")
    car.send_exc = None
    n = len(car.sent)
    self.assertFalse(self.run_car(car, key="boot:1")["ran"])
    self.assertEqual(len(car.sent), n)

  def test_full_read_skipped_once_complete_dtc_daily(self):
    car = FakeCar()
    self.run_car(car, key="boot:1")
    # next ignition, same day: nothing at all
    car.sent.clear()
    s = self.run_car(car, key="boot:2")
    self.assertFalse(s["ran"])
    self.assertEqual(car.sent, [])
    # next day: DTC only
    s = self.run_car(car, key="boot:3", wall=lambda: car.wall() + DAY)
    self.assertTrue(s["ran"])
    self.assertFalse(s["full"])
    self.assertTrue(s["dtc"])
    self.assertEqual({d[1] for _, d, _ in car.sent}, {0x19})
    # new ESC firmware: full read again (and no second DTC read that day)
    car.sent.clear()
    s = self.run_car(car, key="boot:4", wall=lambda: car.wall() + DAY, car_fw=[FW(fw=F100 + b"X")])
    self.assertTrue(s["full"])
    self.assertFalse(s["dtc"])

  def test_full_read_gives_up_after_max_attempts(self):
    car = FakeCar()
    car.responsive = False
    for i in range(E.MAX_FULL_ATTEMPTS + 2):
      self.run_car(car, key=f"boot:{i}", wall=lambda i=i: car.wall() + i * DAY)
    with open(os.path.join(self.out, "state.json")) as f:
      st = json.load(f)
    self.assertEqual(list(st["fw"].values())[0]["attempts"], E.MAX_FULL_ATTEMPTS)

  def test_plan_pure(self):
    self.assertEqual(E.plan({}, "a", "fw", "d"), (True, True))
    self.assertEqual(E.plan({"last_ignition": "a"}, "a", "fw", "d"), (False, False))
    self.assertEqual(E.plan({"fw": {"fw": {"complete": True}}, "last_dtc_day": "d"}, "b", "fw", "d"), (False, False))
    self.assertEqual(E.plan({"fw": {"fw": {"complete": True}}, "last_dtc_day": "d"}, "b", "fw", "e"), (False, True))

  def test_storage_bounded(self):
    car = FakeCar()
    for i in range(E.KEEP_FILES + 5):
      self.run_car(car, key=f"boot:{i}", wall=lambda i=i: car.wall() + i * DAY)
    files = [f for f in os.listdir(self.out) if f.endswith(".json") and f != "state.json"]
    self.assertLessEqual(len(files), E.KEEP_FILES)
