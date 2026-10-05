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
SEED8 = b"\xaa\xbb\xcc\xdd\xaa\xbb\xcc\xdd"   # the 8-byte ESC seed the phase-3 battery pins the key to
DAY = 86400.

# phase-3 module request addr -> response addr (the six the battery walks)
ADDR_RSP = {0x7D1: 0x7D9, 0x7C6: 0x7CE, 0x7E1: 0x7E9, 0x7D4: 0x7DC, 0x7C4: 0x7CC, 0x7B7: 0x7BF}


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
    self.session_refused = False
    self.session_silent = False
    self.write_refused = False
    # phase-3 module answer config
    self.esc_seed8 = bytes(SEED)   # 4-byte default so phases 1/2 are unchanged; phase-3 tests set the 8-byte seed
    self.clu_seed = bytes([0x11, 0x22, 0x33, 0x44])
    self.eps_seed = bytes([0x55, 0x66, 0x77, 0x88])
    self.cam_seed_nrc = 0x11
    self.auth_nrc = 0x11
    self.routine_nrc = 0x31
    self.key_unlock = False        # False -> 7F 27 35; True -> 67 02 (the 27 02 with key==seed unlocks)

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

  def _q(self, dat, delay=0.005, addr=0x7D9):
    self.pending.append((self.t + delay, dat.ljust(8, b"\xaa"), addr))

  def _q_mf(self, payload, delay=0.005, addr=0x7D9, cf_delay=0.005):
    """Queue an ISO-TP multi-frame response (first frame, flow-control, then consecutive frames)."""
    n = len(payload)
    self._q(bytes([0x10 | ((n >> 8) & 0xF), n & 0xFF]) + payload[:6], delay, addr)
    self._q(bytes([0x30, 0x00, 0x00]), delay + cf_delay, addr)   # flow control
    for i in range(0, n - 6, 7):
      self._q(bytes([0x21]) + payload[6 + i:6 + i + 7], delay + cf_delay * (1 + i // 7 + 1), addr)

  def _answer(self, m):
    if not (self.responsive and self.mux and m.src == 1):
      return
    d = bytes(m.dat)
    if d == D.FLOW_CONTROL_FRAME:
      return
    if d[0] == 0x21:
      return                                        # our own consecutive frame (sendKey): nothing to answer
    addr = m.address
    if addr in (0x7C6, 0x7E1, 0x7D4, 0x7C4, 0x7B7):   # phase-3 non-ESC modules: 27 01 seed sweep
      if len(d) >= 2 and d[1] == 0x27 and d[2] == 0x01:
        if addr == 0x7C6:
          self._q(bytes([6, 0x67, 0x01]) + bytes(self.clu_seed), addr=ADDR_RSP[addr])
        elif addr == 0x7D4:
          self._q(bytes([6, 0x67, 0x01]) + bytes(self.eps_seed), addr=ADDR_RSP[addr])
        elif addr == 0x7C4:
          self._q(bytes([3, 0x7F, 0x27, self.cam_seed_nrc]), addr=ADDR_RSP[addr])
        # 0x7E1 (TCU) and 0x7B7 (CR): silent -> timeout, as specified
      return
    if addr != 0x7D1:
      return
    if len(d) >= 3 and d[0] == 0x10 and d[1] == 0x0A and d[2] == 0x27 and d[3] == 0x02:
      self._q(bytes([0x30, 0x00, 0x00]))            # sendKey multi-frame: flow control, then the 27 02 result
      if self.key_unlock:
        self._q(bytes([2, 0x67, 0x02]))
      else:
        self._q(bytes([3, 0x7F, 0x27, 0x35]))
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
      if self.session_silent:
        pass
      elif self.session_refused:
        self._q(bytes([3, 0x7F, 0x10, 0x12]))
      else:
        self._q(bytes([6, 0x50, 0x03, 0x00, 0x32, 0x01, 0xF4]))
    elif svc == 0x27:
      if self.seed_silent:
        pass
      elif self.seed_refused:
        self._q(bytes([3, 0x7F, 0x27, 0x35]))
      elif len(self.esc_seed8) > 4:
        self._q_mf(bytes([0x67, 0x01]) + bytes(self.esc_seed8))    # multi-frame (e.g. 8-byte seed)
      else:
        self._q(bytes([6, 0x67, 0x01]) + bytes(self.esc_seed8))
    elif svc == 0x2E:
      if self.write_refused:
        self._q(bytes([3, 0x7F, 0x2E, 0x33]))
      else:
        self._q(bytes([3, 0x6E, 0x01, 0x03]))
    elif svc == 0x29:
      self._q(bytes([3, 0x7F, 0x29, self.auth_nrc]))
    elif svc == 0x31:
      self._q(bytes([3, 0x7F, 0x31, self.routine_nrc]))
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
    pkt += [CanData(a, d, 1) for _, d, a in due]
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
    # vendor-confirmed sequence: read 0x0103 -> 10 03 extended session -> request seed -> write no-op -> re-read
    frames = self.tx_frames(car)
    self.assertEqual(frames, [bytes([3, 0x22, 0x01, 0x03]).ljust(8, b"\x00"),
                              bytes([2, 0x10, 0x03]).ljust(8, b"\x00"),
                              bytes([2, 0x27, 0x01]).ljust(8, b"\x00"),
                              bytes([7, 0x2E, 0x01, 0x03]) + CURRENT,
                              bytes([3, 0x22, 0x01, 0x03]).ljust(8, b"\x00")])
    services = [d[1] for d in frames]
    self.assertEqual(services, [0x22, 0x10, 0x27, 0x2E, 0x22])
    # the 10 03 session frame sits between the read and the write (the vendor's own order)
    i_read, i_sess = services.index(0x22), services.index(0x10)
    i_write = services.index(0x2E)
    self.assertLess(i_read, i_sess)
    self.assertLess(i_sess, i_write)
    self.assertNotIn(0x22, services[i_sess + 1:i_write])       # no second read between session and write
    # no key is ever sent
    self.assertNotIn(0x02, [d[2] for d in self.tx_frames(car) if d[1] == 0x27])
    doc = self.result_doc()
    self.assertEqual(doc["current_value"], "90060350")
    self.assertTrue(doc["session_before_write"]["positive"])   # 10 03 was answered (50 03)
    self.assertEqual(doc["seed"], "670111223344")
    self.assertFalse(doc["already_unlocked"])
    self.assertTrue(doc["write_attempted"])
    self.assertTrue(doc["write"]["positive"])
    self.assertEqual(doc["reread_value"], "90060350")
    self.assertFalse(doc["value_changed"])
    self.assertEqual(self.events[-1][0], "esc_probe_0027")
    self.assertTrue(s["session_before_write_positive"])
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
    car.esc_seed8 = b"\x00\x00\x00\x00"
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["aborted"])
    self.assertTrue(s["write_attempted"])
    doc = self.result_doc()
    self.assertTrue(doc["already_unlocked"])
    self.assertEqual(doc["seed"], "670100000000")
    self.assertEqual(len(self.write_frames(car)), 1)


class TestSessionBeforeWrite(Base):
  """The vendor-confirmed 10 03 step between the 0x0103 read and the no-op 0x2E write."""

  def test_session_silent_no_write(self):
    car = FakeCar()
    car.session_silent = True
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIn("session", s["aborted"])            # the abort names the missing 10 03 answer
    self.assertFalse(s["write_attempted"])
    self.assertEqual(self.write_frames(car), [])
    services = [d[1] for d in self.tx_frames(car)]
    self.assertNotIn(0x2E, services)                  # silence -> no write frame at all
    self.assertEqual(services.count(0x10), 1)         # the session frame was sent exactly once
    doc = self.result_doc()
    self.assertTrue(doc["session_before_write"]["no_response"])
    self.assertIsNone(doc["session_before_write"].get("resp"))

  def test_session_refused_still_writes(self):
    car = FakeCar()
    car.session_refused = True
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["aborted"])                   # a refused 10 03 (7F 10 12) does NOT stop the vendor write
    self.assertIsNone(s["error"])
    self.assertTrue(s["write_attempted"])
    self.assertEqual(len(self.write_frames(car)), 1)
    self.assertEqual([d[1] for d in self.tx_frames(car)].count(0x10), 1)
    doc = self.result_doc()
    self.assertEqual(doc["session_before_write"]["nrc"], 0x12)
    self.assertIsNone(doc["session_before_write"].get("positive"))
    self.assertEqual(doc["reread_value"], "90060350")


class TestPhase2(Base):
  """Phase 2 (state ``{"phase": 2}``): the vendor-exact write order with NO pre-write seed request, and ONE post-write
  ``27 01`` sample recorded as ``seed_post`` (a data point only — it never gates the write, which already happened)."""

  def setUp(self):
    super().setUp()
    self.set_phase(2)

  def set_phase(self, phase):
    with open(os.path.join(self.out, "state.json"), "w") as f:
      json.dump({"probe_enabled": True, "phase": phase}, f)

  def test_phase2_no_pre_write_seed_and_post_write_sample(self):
    car = FakeCar()
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["aborted"])
    self.assertIsNone(s["error"])
    # vendor-exact order: READ -> SESSION -> WRITE -> re-read -> (single) post-write seed sample
    frames = self.tx_frames(car)
    self.assertEqual(frames, [bytes([3, 0x22, 0x01, 0x03]).ljust(8, b"\x00"),
                              bytes([2, 0x10, 0x03]).ljust(8, b"\x00"),
                              bytes([7, 0x2E, 0x01, 0x03]) + CURRENT,
                              bytes([3, 0x22, 0x01, 0x03]).ljust(8, b"\x00"),
                              bytes([2, 0x27, 0x01]).ljust(8, b"\x00")])
    services = [d[1] for d in frames]
    self.assertEqual(services, [0x22, 0x10, 0x2E, 0x22, 0x27])
    i_write = services.index(0x2E)
    self.assertNotIn(0x27, services[:i_write])       # NO seed frame before the write (the whole point of phase 2)
    self.assertEqual(services.index(0x27), len(services) - 1)  # the seed sample is the LAST frame
    self.assertNotIn(0x02, [d[2] for d in frames if d[1] == 0x27])  # sendKey is never sent
    doc = self.result_doc()
    self.assertEqual(doc["phase"], 2)
    self.assertIsNone(doc["seed_request"])           # no pre-write seed
    self.assertIsNone(doc["seed"])
    self.assertIsNotNone(doc["seed_post"])           # post-write sample recorded
    self.assertTrue(doc["seed_post"]["positive"])
    self.assertEqual(doc["seed_post"]["resp"], "670111223344")
    self.assertTrue(doc["write_attempted"])
    self.assertTrue(doc["write"]["positive"])
    self.assertEqual(doc["reread_value"], "90060350")
    self.assertFalse(doc["value_changed"])
    self.assertEqual(s["phase"], 2)
    self.assertTrue(s["seed_post_positive"])
    self.assertEqual(self.events[-1][0], "esc_probe_0027")

  def test_phase2_write_refused_seed_post_still_sampled(self):
    car = FakeCar()
    car.write_refused = True                          # the fake answers 7F 2E 33 (like phase 1 did on the car)
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["aborted"])
    self.assertIsNone(s["error"])
    self.assertTrue(s["write_attempted"])
    self.assertEqual(s["write_nrc"], 0x33)            # refused write is recorded, does not abort
    doc = self.result_doc()
    self.assertEqual(doc["write"]["nrc"], 0x33)
    self.assertIsNone(doc["seed_request"])
    self.assertIsNotNone(doc["seed_post"])            # the post-write sample is STILL attempted + recorded
    self.assertTrue(doc["seed_post"]["positive"])
    self.assertEqual(len(self.write_frames(car)), 1)
    self.assertEqual([d[1] for d in self.tx_frames(car)], [0x22, 0x10, 0x2E, 0x22, 0x27])

  def test_phase2_seed_post_silent_no_abort(self):
    car = FakeCar()
    car.seed_silent = True                            # the post-write 27 01 gets no frame at all
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["aborted"])                   # the write already happened: seed silence must NOT abort
    self.assertIsNone(s["error"])
    self.assertTrue(s["write_attempted"])
    self.assertTrue(s["write_positive"])
    doc = self.result_doc()
    self.assertIsNone(doc["seed_request"])
    self.assertIsNotNone(doc["seed_post"])
    self.assertTrue(doc["seed_post"]["no_response"])  # silence, recorded as such
    self.assertIsNone(doc["seed_post"].get("resp"))
    self.assertFalse(s["seed_post_positive"])

  def test_phase2_session_silent_aborts_before_write(self):
    car = FakeCar()
    car.session_silent = True
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIn("session", s["aborted"])            # same rule as phase 1: no 10 03 answer -> no write
    self.assertFalse(s["write_attempted"])
    self.assertEqual(self.write_frames(car), [])
    services = [d[1] for d in self.tx_frames(car)]
    self.assertNotIn(0x2E, services)
    self.assertNotIn(0x27, services)                  # phase 2 has no pre-write seed either
    self.assertIsNone(self.result_doc()["seed_post"])

  def test_missing_phase_key_is_phase1(self):
    self.set_phase(None)                              # {"phase": None} is treated as the default
    car = FakeCar()
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertEqual(s["phase"], 1)
    self.assertEqual([d[1] for d in self.tx_frames(car)], [0x22, 0x10, 0x27, 0x2E, 0x22])
    doc = self.result_doc()
    self.assertEqual(doc["phase"], 1)
    self.assertIsNotNone(doc["seed_request"])         # phase 1 keeps the pre-write seed
    self.assertIsNone(doc["seed_post"])

  def test_unknown_phase_sends_nothing(self):
    # NB: `int(state.get("phase", 1) or 1)` maps a FALSY value (missing, None, 0) to the default 1; only a value that
    # is neither 1, 2 nor 3 (e.g. -1, 4) or non-numeric ("banana") is unknown.
    for phase in (-1, 4, "banana"):
      with self.subTest(phase=phase):
        self.fresh_state()
        self.set_phase(phase)
        car = FakeCar()
        s = self.run_car(car, key=f"k{phase}")
        self.assertFalse(s["ran"])
        self.assertEqual(s["skip"], "unknown phase")
        self.assertEqual(car.sent, [])
        self.assertEqual(car.mux_calls, [])


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
    # the retry already entered extended, so no SECOND 10 03 is sent before the (never-reached) write
    self.assertEqual([d[1] for d in self.tx_frames(car)].count(0x10), 1)
    self.assertIsNone(doc["session_before_write"])           # the write's session step was skipped: read aborted first

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
    # _can_send appears only in EscProbeClient._tx and ._tx_at — the two documented TX sites (phase 3's non-ESC
    # modules go through _tx_at rather than _tx, but both apply the same gate + guard_frame immediately before send).
    self.assertEqual([c[1] for c in calls], ["_can_send", "_can_send"], calls)
    for method in (E.EscProbeClient._tx, E.EscProbeClient._tx_at):
      src = inspect.getsource(method)
      self.assertLess(src.index("self.gate.check()"), src.index("self._can_send("))
      self.assertLess(src.index("guard_frame("), src.index("self._can_send("))
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


class TestPhase3(Base):
  """Phase 3 (state ``{"phase": 3}``): the single-ignition discriminating battery — 6-address 27 01 seed sweep,
  default-session no-op 2E, 29 01 + 31 01 probes, extended-session no-op 2E, ONE identity-key 27 02 attempt
  (mechanically pinned: single attempt + key == the step-2 ESC seed), and the conditional post-unlock no-op write."""

  SEED_FRAME = bytes([2, 0x27, 0x01]).ljust(8, b"\x00")
  AUTH_FRAME = bytes([2, 0x29, 0x01]).ljust(8, b"\x00")
  ROUTINE_FRAME = bytes([4, 0x31, 0x01, 0x00, 0x00]).ljust(8, b"\x00")
  KEY_FIRST = bytes([0x10, 0x0A, 0x27, 0x02]) + SEED8[:4]        # 10 0A 27 02 <k0..k3>, padded to 8
  KEY_CF = (bytes([0x21]) + SEED8[4:8]).ljust(8, b"\x00")
  EXPECTED_TX = [bytes([3, 0x22, 0x01, 0x03]).ljust(8, b"\x00"),
                 SEED_FRAME, D.FLOW_CONTROL_FRAME, SEED_FRAME, SEED_FRAME, SEED_FRAME, SEED_FRAME, SEED_FRAME,
                 bytes([7, 0x2E, 0x01, 0x03]) + CURRENT,
                 AUTH_FRAME, ROUTINE_FRAME,
                 bytes([3, 0x22, 0x01, 0x03]).ljust(8, b"\x00"),
                 bytes([2, 0x10, 0x03]).ljust(8, b"\x00"),
                 bytes([7, 0x2E, 0x01, 0x03]) + CURRENT,
                 KEY_FIRST, KEY_CF]

  def setUp(self):
    super().setUp()
    self.set_phase(3)

  def set_phase(self, phase):
    with open(os.path.join(self.out, "state.json"), "w") as f:
      json.dump({"probe_enabled": True, "phase": phase}, f)

  def p3_car(self, **kw):
    car = FakeCar(**kw)
    car.esc_seed8 = SEED8
    car.write_refused = True    # the real car refuses every 2E no-op with 7F 2E 33 (both sessions)
    return car

  def test_battery_exact_tx_order_and_outcome(self):
    car = self.p3_car()
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["aborted"])
    self.assertIsNone(s["error"])
    self.assertEqual(self.tx_frames(car), self.EXPECTED_TX)
    # six 27 01 seed frames (one per module) sit between the read and the default-session write; no key before step 9
    services = [d[1] for d in self.tx_frames(car)]
    self.assertEqual(services.count(0x27), 6)
    self.assertEqual(services[:2], [0x22, 0x27])
    self.assertEqual([d for d in self.tx_frames(car) if d[:3] == bytes([0x10, 0x0A, 0x27])], [self.KEY_FIRST])
    self.assertTrue(self.result_doc()["steps"])
    self.assertEqual(car.mux_calls, [True, False])
    self.assertTrue(s["mux_restored"])

  def test_battery_records_each_step_with_the_verbatim_nrc(self):
    car = self.p3_car()
    self.run_car(car)
    doc = self.result_doc()
    names = [st["name"] for st in doc["steps"]]
    self.assertEqual(names, ["read_esc", "seed_7D1", "seed_7C6", "seed_7E1", "seed_7D4", "seed_7C4", "seed_7B7",
                             "write_default", "auth_probe", "routine_probe", "reread_esc", "session",
                             "write_extended", "key_attempt"])
    self.assertEqual(doc["current_value"], "90060350")
    self.assertEqual(doc["write_default"]["nrc"], 0x33)     # 7F 2E 33 in the DEFAULT session
    self.assertEqual(doc["write_extended"]["nrc"], 0x33)    # and in the EXTENDED session (blanket filter?)
    self.assertEqual(doc["auth_probe"]["nrc"], 0x11)        # 29 -> service not supported
    self.assertEqual(doc["routine_probe"]["nrc"], 0x31)     # 31 -> requestOutOfRange (session logic answered)
    self.assertEqual(doc["reread_value"], "90060350")
    self.assertFalse(doc["value_changed"])
    self.assertTrue(doc["key_attempted"])
    self.assertEqual(doc["key"], SEED8.hex())
    self.assertFalse(doc["unlocked"])
    self.assertEqual(doc["key_nrc"], 0x35)
    # every step carries a per-frame receive timestamp (ms relative to its request TX)
    for st in doc["steps"]:
      if st["frames"]:
        self.assertEqual(len(st["frames_t_ms"]), len(st["frames"]), st["name"])
        self.assertTrue(all(ms >= 0 for ms in st["frames_t_ms"]), st["name"])

  def test_seed_sweep_all_six_addrs_including_timeouts(self):
    car = self.p3_car()
    self.run_car(car)
    sweep = self.result_doc()["seed_sweep"]
    self.assertEqual([r["addr"] for r in sweep], [0x7D1, 0x7C6, 0x7E1, 0x7D4, 0x7C4, 0x7B7])
    by = {r["addr"]: r for r in sweep}
    self.assertEqual(by[0x7D1]["resp"], "6701" + SEED8.hex())     # ESC: 8-byte seed
    self.assertEqual(by[0x7C6]["resp"], "670111223344")           # CLU seed
    self.assertEqual(by[0x7D4]["resp"], "670155667788")           # EPS seed
    self.assertEqual(by[0x7C4]["nrc"], 0x11)                      # CAM: NRC 7F 27 11
    self.assertTrue(by[0x7E1]["no_response"])                    # TCU: silence -> timeout
    self.assertTrue(by[0x7B7]["no_response"])                    # CR:  silence -> timeout
    self.assertIsNone(by[0x7E1].get("resp"))
    # the sweep frames really went to the six distinct request addresses
    addrs = [a for a, d, _ in car.sent if d[1] == 0x27]
    self.assertEqual(addrs, [0x7D1, 0x7C6, 0x7E1, 0x7D4, 0x7C4, 0x7B7])

  def test_key_unlock_then_automatic_noop_write(self):
    car = self.p3_car()
    car.key_unlock = True
    car.write_refused = False        # hypothetical: once unlocked the no-op write lands (6E 0103) and the re-read changes
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["aborted"])
    tail = [bytes([2, 0x10, 0x03]).ljust(8, b"\x00"),
            bytes([7, 0x2E, 0x01, 0x03]) + CURRENT,
            bytes([3, 0x22, 0x01, 0x03]).ljust(8, b"\x00")]
    self.assertEqual(self.tx_frames(car), self.EXPECTED_TX + tail)
    doc = self.result_doc()
    self.assertTrue(doc["unlocked"])
    self.assertTrue(doc["key_result"]["positive"])
    self.assertEqual(doc["key_result"]["resp"], "6702")
    self.assertTrue(doc["write_unlocked"]["positive"])   # the auto step-10 write lands (6E 0103)
    self.assertEqual(doc["reread_unlocked"]["resp"], "62010390060350")

  def test_failed_key_stops_there_no_post_unlock_write(self):
    car = self.p3_car()                                   # key_unlock False -> 7F 27 35
    self.run_car(car)
    # after the single 27 02 attempt there is nothing after it (the attempt is the last frame)
    self.assertEqual(self.tx_frames(car)[-1], self.KEY_CF)
    doc = self.result_doc()
    self.assertFalse(doc["unlocked"])
    self.assertIn("no further keys", doc["key_note"])

  def test_sendkey_is_at_most_once(self):
    car = self.p3_car()
    car.set_mux(True)
    gate = D.VehicleGate(car.now)
    gate.feed(CanData(0x367, lvr12(0), 0))
    gate.feed(CanData(0x386, whl(0.), 0))
    c = E.EscProbeClient(car.can_send, car.can_recv, gate, car.now, car.now() + 99, phase=3)
    c.send_key(SEED8)
    self.assertEqual(c.key_attempts, 1)
    with self.assertRaises(D.SafetyViolation):
      c.send_key(SEED8)                                   # a second key can never reach the bus
    self.assertEqual([d for _, d, _ in car.sent if d[0] == 0x10].__len__(), 1)

  def test_guard_frame_pins_key_to_step2_seed_and_single_attempt(self):
    good_key = SEED8
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.KEY_FIRST, 1, phase=3, seed=None, key_attempts=0)     # no seed
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.KEY_FIRST, 1, phase=3, seed=good_key, key_attempts=1)  # second attempt
    E.guard_frame(0x7D1, self.KEY_FIRST, 1, phase=3, seed=good_key, key_attempts=0)   # exactly once, right key
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.KEY_CF, 1, phase=3, seed=good_key, key_attempts=0)      # cf before the first frame
    E.guard_frame(0x7D1, self.KEY_CF, 1, phase=3, seed=good_key, key_attempts=1)        # cf after the first frame
    wrong = bytes([0x10, 0x0A, 0x27, 0x02]) + b"\x01\x02\x03\x04"
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, wrong, 1, phase=3, seed=good_key, key_attempts=0)            # wrong key
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([0x21]) + b"\x09\x09\x09\x09" + b"\x00" * 4, 1, phase=3, seed=good_key, key_attempts=1)
    # 27 02 stays refused for phases 1/2 (phase defaults to 1: byte-identical guards)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.KEY_FIRST, 1, seed=good_key, key_attempts=0)

  def test_send_key_requires_eight_byte_seed(self):
    car = self.p3_car()
    car.set_mux(True)
    gate = D.VehicleGate(car.now)
    gate.feed(CanData(0x367, lvr12(0), 0))
    gate.feed(CanData(0x386, whl(0.), 0))
    c = E.EscProbeClient(car.can_send, car.can_recv, gate, car.now, car.now() + 99, phase=3)
    with self.assertRaises(D.SafetyViolation):
      c.send_key(b"\x01\x02\x03")                         # not the 8-byte step-2 seed

  def test_no_sendkey_when_esc_has_no_positive_seed(self):
    car = self.p3_car()
    car.seed_refused = True                               # ESC 27 01 -> 7F 27 35 (no seed to pin the key to)
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    doc = self.result_doc()
    self.assertFalse(doc["key_attempted"])
    self.assertIn("no positive ESC seed", doc["key_note"])
    self.assertEqual([d for d in self.tx_frames(car) if d[0] == 0x10 and len(d) > 2 and d[2] == 0x27], [])

  def test_no_current_value_aborts_the_battery(self):
    car = self.p3_car()
    car.read_refused = True
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIn("no current value", s["aborted"])
    self.assertEqual([d for d in self.tx_frames(car) if d[1] == 0x2E], [])   # nothing written
    self.assertEqual(car.mux_calls, [True, False])
    self.assertTrue(self.result_doc()["mux_restored"])

  def test_phase3_budget_is_thirty_seconds(self):
    self.assertEqual(E.RUN_BUDGET_S_PHASE3, 30.0)
    self.assertEqual(E.RUN_BUDGET_S, 15.0)
