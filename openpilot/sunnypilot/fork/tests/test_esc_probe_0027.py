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
from openpilot.sunnypilot.fork import esc_probe_seedkey as SK

CURRENT = b"\x90\x06\x03\x50"       # the known-good 0x0103 value on this car
SEED = b"\x11\x22\x33\x44"
SEED8 = b"\xaa\xbb\xcc\xdd\xaa\xbb\xcc\xdd"   # the 8-byte ESC seed the phase-3 battery pins the key to
PHASE4_SEED = bytes.fromhex("5AB05AB05AB05AB0")   # the real car's 8-byte seed (a 2-byte value repeated 4x)
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
    self.key_nrc = 0x35            # the NRC returned when key_unlock is False (35 invalidKey / 36 / 37)
    self.key_silent = False        # phase 4: no answer at all to the 27 02
    # phase-5 battery answer config
    self.fp_canary_nrc = 0x31      # 22 F100 -> 7F 22 31 by default; None -> positive 62 F100 <4 bytes>
    self.fp_canary_value = b"\x01\x02\x03\x04"
    self.sub_probe_nrc = 0x12      # 27 03/05/... -> 7F 27 12 (subFunctionNotSupported)
    self.svc_probe_nrc = {0x23: 0x11, 0x34: 0x11, 0x35: 0x11, 0x36: 0x11, 0x37: 0x11}   # bare 1-byte probes
    self.extra_answer = {}         # {0x770: True, 0x7A0: False}: does the extra request addr answer?
    self.seed_sequence = None      # list of seed byte-strings returned by successive 27 01 asks (None -> esc_seed8)
    self.seed_i = 0
    # phase-6 policy-matrix answer config
    self.prog_session_positive = True     # 10 02 -> 50 02 (True) or 7F 10 12 (False)
    self.key_nrc_sequence = None          # NRCs returned by SUCCESSIVE key attempts (None -> key_unlock/key_nrc)
    self.key_seen = 0
    self.sweep_dids = {}                  # {0xF186: "62f1...": }: default -> 7F 22 31 for every sweep DID
    self.dtc_a5_resp = "037f1931"          # 19 02 A5 -> ISO-TP 03 7f 19 31 (requestOutOfRange) by default
    # phase-7 ASK-family answer config
    self.esc_seed8_11 = bytes(PHASE4_SEED)  # the 8-byte seed the ESC answers 27 11 with
    self.seed11_sequence = None          # list of 8-byte seeds for successive 27 11 asks (None -> esc_seed8_11)
    self.seed11_i = 0
    self.seed11_refused = False          # 27 11 -> 7F 27 35
    self.seed11_silent = False           # 27 11 -> no answer
    self.key12_unlock = False            # False -> 7F 27 <key12_nrc>; True -> 67 12 (the 27 12 unlocks)
    self.key12_nrc = 0x35                # the NRC returned when key12_unlock is False
    self.key12_nrc_sequence = None       # NRCs for SUCCESSIVE 27 12 attempts (None -> key12_nrc)
    self.key12_seen = 0
    self.key12_silent = False            # phase 7: no answer at all to the 27 12
    self.extra_f100 = {}                 # phase 7: {0x770: True, 0x7A0: False} -- does the extra addr answer 22 F100?
    self.extra_f100_nrc = None           # phase 7: NRC for an unanswered extra-addr F100 (None -> silence)
    self.ask_family = False              # phase 7: answer the ASK family (27 11/27 12); False keeps everything else untouched
    self.key12_unlock_index = None       # phase 7: unlock ONLY on the Nth 27 12 attempt (None -> use key12_unlock)
    # phase-8 door-B counter-economics answer config
    self.session_positive = {0x01: True, 0x03: True}   # 10 01 / 10 03 -> 50 xx (True) or 7F 10 12 (False)
    self.key12_actions = None            # full script: key12_actions[i] applied to the i-th 27 12 ({unlock,nrc,silent})
    self.key12_log = []                  # [(key_hex, time, action)] one per 27 12, for the counter-economics asserts
    self.auth05_nrc = 0x11               # 29 05 -> 7F 29 <auth05_nrc>

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

  def _key_nrc(self):
    """The NRC for the NEXT key attempt: the key_nrc_sequence[i] if set (i = attempts so far), else the fixed key_nrc."""
    if self.key_nrc_sequence is not None:
      return self.key_nrc_sequence[min(self.key_seen, len(self.key_nrc_sequence) - 1)]
    return self.key_nrc

  def _key12_nrc(self):
    """Phase 7: the NRC for the NEXT 27 12 attempt (key12_nrc_sequence[i] if set, else the fixed key12_nrc)."""
    if self.key12_nrc_sequence is not None:
      return self.key12_nrc_sequence[min(self.key12_seen, len(self.key12_nrc_sequence) - 1)]
    return self.key12_nrc

  def _answer(self, m):
    if not (self.responsive and self.mux and m.src == 1):
      return
    d = bytes(m.dat)
    if d == D.FLOW_CONTROL_FRAME:
      return
    if d[0] == 0x21:
      return                                        # our own consecutive frame (sendKey): nothing to answer
    addr = m.address
    if addr in (0x770, 0x7A0):                        # phase-5 extra-address peeks (10 03) / phase-7 (22 F100)
      if d[:3] == bytes([2, 0x10, 0x03]) and self.extra_answer.get(addr):
        self._q(bytes([6, 0x50, 0x03, 0x00, 0x32, 0x01, 0xF4]), addr=addr + 8)
      elif d[:4] == bytes([3, 0x22, 0xF1, 0x00]):     # phase-7 extra-address 22 F100 read
        if self.extra_f100.get(addr):
          self._q(bytes([7, 0x62, 0xF1, 0x00]) + bytes(self.fp_canary_value), addr=addr + 8)
        elif self.extra_f100_nrc is not None:
          self._q(bytes([3, 0x7F, 0x22, self.extra_f100_nrc]), addr=addr + 8)
        # else: silence on 0x778/0x7A8
      return                                          # else: silence on 0x778/0x7A8
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
      if self.key_silent:
        return
      nrc = self._key_nrc()
      self.key_seen += 1
      if self.key_unlock:
        self._q(bytes([2, 0x67, 0x02]))
      else:
        self._q(bytes([3, 0x7F, 0x27, nrc]))
      return
    if len(d) >= 3 and d[0] in (4, 6) and d[1] == 0x27 and d[2] == 0x02:
      # phase 4: a 2- or 4-byte candidate key as a single frame (04 27 02 K1K2 / 06 27 02 K1..K4)
      if self.key_silent:
        return
      nrc = self._key_nrc()
      self.key_seen += 1
      if self.key_unlock:
        self._q(bytes([2, 0x67, 0x02]))
      else:
        self._q(bytes([3, 0x7F, 0x27, nrc]))
      return
    if len(d) >= 4 and d[0] == 0x10 and d[1] == 0x0A and d[2] == 0x27 and d[3] == 0x12:
      # phase 7/8: 27 12 sendKey multi-frame -> flow control, then the 27 12 result
      attempt = self.key12_seen
      nrc = self._key12_nrc()
      self.key12_seen += 1
      if self.key12_actions is not None:
        # phase 8: a FULL per-attempt script (unlock / nrc / silence) -- the answer depends on where we are in the
        # cycle/time-reset battery, not on a fixed NRC. The key + answer time are logged for the wait asserts.
        act = self.key12_actions[min(attempt, len(self.key12_actions) - 1)]
        self.key12_log.append((bytes(d[4:8]).hex(), self.t, act))
        self._q(bytes([0x30, 0x00, 0x00]))
        if act.get("silent"):
          return
        if act.get("unlock"):
          self._q(bytes([2, 0x67, 0x12]))
        else:
          self._q(bytes([3, 0x7F, 0x27, act.get("nrc", self.key12_nrc)]))
        return
      self._q(bytes([0x30, 0x00, 0x00]))
      if self.key12_silent:
        return
      unlock = self.key12_unlock or (self.key12_unlock_index is not None and attempt == self.key12_unlock_index)
      if unlock:
        self._q(bytes([2, 0x67, 0x12]))
      else:
        self._q(bytes([3, 0x7F, 0x27, nrc]))
      return
    body = d[1:1 + d[0]]
    if not body:
      return
    svc = body[0]
    if svc == 0x22:
      did = (body[1] << 8) | body[2]
      if did == 0xF100:
        if self.fp_canary_nrc is None:
          self._q(bytes([7, 0x62, 0xF1, 0x00]) + bytes(self.fp_canary_value))
        else:
          self._q(bytes([3, 0x7F, 0x22, self.fp_canary_nrc]))
      elif did in self.sweep_dids:                   # phase-6 identification sweep: verbatim configured response
        self._q(bytes.fromhex(self.sweep_dids[did]))
      elif did == 0x0103 and not self.read_refused and not self.read_silent:
        self._q(bytes([7, 0x62, 0x01, 0x03]) + bytes(self.current))
      else:
        self._q(bytes([3, 0x7F, 0x22, 0x31]))
    elif svc == 0x10:
      sub = body[1] if len(body) > 1 else None
      if sub == 0x01:
        if self.session_positive.get(0x01, True):
          self._q(bytes([2, 0x50, 0x01]))
        else:
          self._q(bytes([3, 0x7F, 0x10, 0x12]))
      elif sub == 0x02:                              # phase-6 programming-session probe
        if self.prog_session_positive:
          self._q(bytes([3, 0x50, 0x02, 0x00]))
        else:
          self._q(bytes([3, 0x7F, 0x10, 0x12]))
      elif self.session_silent:
        pass
      elif self.session_refused or not self.session_positive.get(0x03, True):
        self._q(bytes([3, 0x7F, 0x10, 0x12]))
      else:
        self._q(bytes([6, 0x50, 0x03, 0x00, 0x32, 0x01, 0xF4]))
    elif svc == 0x27:
      sub = body[1] if len(body) > 1 else None
      if sub == 0x11 and self.ask_family:             # phase-7 ASK-family requestSeed
        if self.seed11_silent:
          pass
        elif self.seed11_refused:
          self._q(bytes([3, 0x7F, 0x27, 0x35]))
        else:
          seed = bytes(self.esc_seed8_11)
          if self.seed11_sequence:
            seed = bytes(self.seed11_sequence[min(self.seed11_i, len(self.seed11_sequence) - 1)])
            self.seed11_i += 1
          self._q_mf(bytes([0x67, 0x11]) + seed)     # 8-byte seed -> multi-frame
      elif sub == 0x12 and self.ask_family:           # phase-7 sendKey single-frame form (the guard refuses it)
        self._q(bytes([3, 0x7F, 0x27, 0x12]))
      elif sub != 0x01:                              # phase-5 sub-probes (27 03/05/... incl. 27 11): a plain refusal
        self._q(bytes([3, 0x7F, 0x27, self.sub_probe_nrc]))
      elif self.seed_silent:
        pass
      elif self.seed_refused:
        self._q(bytes([3, 0x7F, 0x27, 0x35]))
      else:
        seed = bytes(self.esc_seed8)
        if self.seed_sequence:
          seed = bytes(self.seed_sequence[min(self.seed_i, len(self.seed_sequence) - 1)])
          self.seed_i += 1
        if len(seed) > 4:
          self._q_mf(bytes([0x67, 0x01]) + seed)     # multi-frame (e.g. 8-byte seed)
        else:
          self._q(bytes([6, 0x67, 0x01]) + seed)
    elif svc == 0x2E:
      if self.write_refused:
        self._q(bytes([3, 0x7F, 0x2E, 0x33]))
      else:
        self._q(bytes([3, 0x6E, 0x01, 0x03]))
    elif svc == 0x19:                                # phase-6 19 02 A5 (ReadDTCByStatusMask)
      self._q(bytes.fromhex(self.dtc_a5_resp))
    elif svc == 0x29:
      sub = body[1] if len(body) > 1 else None
      self._q(bytes([3, 0x7F, 0x29, self.auth05_nrc if sub == 0x05 else self.auth_nrc]))
    elif svc == 0x31:
      self._q(bytes([3, 0x7F, 0x31, self.routine_nrc]))
    elif svc in self.svc_probe_nrc:                  # phase-5 bare 1-byte probes 23/34/35/36/37
      self._q(bytes([3, 0x7F, svc, self.svc_probe_nrc[svc]]))
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
  ``27 01`` sample recorded as ``seed_post`` (a data point only -- it never gates the write, which already happened)."""

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
    # is neither 1..9 (e.g. -1, 10) or non-numeric ("banana") is unknown.
    for phase in (-1, 10, "banana"):
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
    # _can_send appears only in EscProbeClient._tx and ._tx_at -- the two documented TX sites (phase 3's non-ESC
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
  """Phase 3 (state ``{"phase": 3}``): the single-ignition discriminating battery -- 6-address 27 01 seed sweep,
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


class TestPhase4(Base):
  """Phase 4 (state ``{"probe_enabled": true, "phase": 4, "key_mode": ...}``): the sendKey attempt.
  read 22 0103 -> 10 03 (REQUIRED positive, seeds are session-gated) -> 27 01 seed (REQUIRED positive, >=2 bytes)
  -> ~500 ms -> ONE 27 02 with the RESOLVED candidate key (identity2/identity4/identity8/algo/hex, plus the 8-byte
  construction modes algo8/algo8p/repeat8/hex8) -> only on 67 02 the no-op 2E + re-read. Mechanically single-attempt
  and pinned to the resolved bytes."""

  READ = bytes([3, 0x22, 0x01, 0x03]).ljust(8, b"\x00")
  SESSION = bytes([2, 0x10, 0x03]).ljust(8, b"\x00")
  SEED01 = bytes([2, 0x27, 0x01]).ljust(8, b"\x00")
  FC = D.FLOW_CONTROL_FRAME                                          # sent for the multi-frame 67 01 seed answer
  WRITE = bytes([7, 0x2E, 0x01, 0x03]) + CURRENT
  KEY2 = (bytes([4, 0x27, 0x02]) + PHASE4_SEED[:2]).ljust(8, b"\x00")                  # 04 27 02 5A B0
  KEY4 = (bytes([6, 0x27, 0x02]) + PHASE4_SEED[:4]).ljust(8, b"\x00")                  # 06 27 02 5A B0 5A B0
  KEY8_FF = bytes([0x10, 0x0A, 0x27, 0x02]) + PHASE4_SEED[:4]                          # 10 0A 27 02 5A B0 5A B0
  KEY8_CF = (bytes([0x21]) + PHASE4_SEED[4:8]).ljust(8, b"\x00")                       # 21 5A B0 5A B0

  def setUp(self):
    super().setUp()

  def set_state(self, phase=4, **extra):
    st = {"probe_enabled": True, "phase": phase, **extra}
    with open(os.path.join(self.out, "state.json"), "w") as f:
      json.dump(st, f)

  def p4_car(self, **kw):
    car = FakeCar(**kw)
    car.esc_seed8 = PHASE4_SEED   # the real car's 8-byte seed (2-byte value x4)
    return car

  def client4(self, car):
    car.set_mux(True)
    gate = D.VehicleGate(car.now)
    gate.feed(CanData(0x367, lvr12(0), 0))
    gate.feed(CanData(0x386, whl(0.), 0))
    return E.EscProbeClient(car.can_send, car.can_recv, gate, car.now, car.now() + 99, phase=4)

  # ---- (a) happy identity2: extended seed -> key 5AB0 -> 67 02 -> automatic 2E no-op -> reread -------------------
  def test_phase4_identity2_unlock_then_auto_noop_write(self):
    self.set_state(key_mode="identity2")
    car = self.p4_car()
    car.key_unlock = True
    car.write_refused = False          # once unlocked the no-op write lands (6E 0103)
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["aborted"])
    self.assertIsNone(s["error"])
    # exact TX order INCLUDING the automatic step-7 write + re-read
    self.assertEqual(self.tx_frames(car),
                     [self.READ, self.SESSION, self.SEED01, self.FC, self.KEY2, self.WRITE, self.READ])
    self.assertEqual([d[1] for d in self.tx_frames(car) if d != self.FC], [0x22, 0x10, 0x27, 0x27, 0x2E, 0x22])
    self.assertEqual([d for _, d, _ in car.sent if d[1] == 0x27 and d[2] == 0x02], [self.KEY2])   # exactly one 27 02
    self.assertEqual(car.mux_calls, [True, False])
    doc = self.result_doc()
    self.assertEqual(doc["phase"], 4)
    self.assertEqual(doc["key_mode"], "identity2")
    self.assertEqual(doc["seed"], PHASE4_SEED.hex())
    self.assertEqual(doc["key"], "5ab0")
    self.assertTrue(doc["key_attempted"])
    self.assertTrue(doc["unlocked"])
    self.assertTrue(doc["key_result"]["positive"])
    self.assertEqual(doc["key_result"]["resp"], "6702")
    self.assertTrue(doc["write_unlocked"]["positive"])            # the auto step-7 write landed
    self.assertEqual(doc["reread_unlocked"]["resp"], "62010390060350")
    self.assertEqual(doc["reread_unlocked_value"], "90060350")
    self.assertTrue(s["key_attempted"])
    self.assertTrue(s["unlocked"])
    self.assertTrue(s["write_unlocked_positive"])
    self.assertEqual(self.events[-1][0], "esc_probe_0027")

  # ---- (b) identity4 frame shape 06 27 02 5AB05AB0 ---------------------------------------------------------------
  def test_phase4_identity4_single_frame_shape(self):
    self.set_state(key_mode="identity4")
    car = self.p4_car()
    car.key_unlock = True
    car.write_refused = False
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["aborted"])
    self.assertEqual(self.tx_frames(car),
                     [self.READ, self.SESSION, self.SEED01, self.FC, self.KEY4, self.WRITE, self.READ])
    keys = [d for _, d, _ in car.sent if d[1] == 0x27 and d[2] == 0x02]
    self.assertEqual(keys, [self.KEY4])
    self.assertEqual(keys[0][:7], bytes([6, 0x27, 0x02, 0x5A, 0xB0, 0x5A, 0xB0]))
    doc = self.result_doc()
    self.assertEqual(doc["key"], "5ab05ab0")
    self.assertTrue(doc["unlocked"])

  # ---- (c) identity8 multiframe shapes 10 0A 27 02 . + 21 . ------------------------------------------------------
  def test_phase4_identity8_multiframe_shapes(self):
    self.set_state(key_mode="identity8")
    car = self.p4_car()
    car.key_unlock = True
    car.write_refused = False
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["aborted"])
    self.assertIsNone(s["error"])
    self.assertEqual(self.tx_frames(car),
                     [self.READ, self.SESSION, self.SEED01, self.FC, self.KEY8_FF, self.KEY8_CF, self.WRITE, self.READ])
    self.assertEqual([d for _, d, _ in car.sent if d[:4] == bytes([0x10, 0x0A, 0x27, 0x02])], [self.KEY8_FF])
    self.assertEqual([d for _, d, _ in car.sent if d[0] == 0x21], [self.KEY8_CF])
    self.assertEqual(self.KEY8_FF, bytes([0x10, 0x0A, 0x27, 0x02, 0x5A, 0xB0, 0x5A, 0xB0]))
    self.assertEqual(self.KEY8_CF, bytes([0x21, 0x5A, 0xB0, 0x5A, 0xB0]).ljust(8, b"\x00"))
    doc = self.result_doc()
    self.assertEqual(doc["key"], PHASE4_SEED.hex())
    self.assertTrue(doc["unlocked"])

  # ---- (d) invalidKey 7F 27 35 -> recorded, NO 2E frame in tx ---------------------------------------------------
  def test_phase4_invalid_key_records_and_no_write(self):
    self.set_state(key_mode="identity2")
    car = self.p4_car()                    # key_unlock False -> 7F 27 35
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["aborted"])
    self.assertIsNone(s["error"])
    self.assertFalse(s["unlocked"])
    self.assertEqual(self.write_frames(car), [])          # NO 2E frame at all after a failed key
    self.assertEqual(self.tx_frames(car), [self.READ, self.SESSION, self.SEED01, self.FC, self.KEY2])
    doc = self.result_doc()
    self.assertEqual(doc["key_nrc"], 0x35)
    self.assertFalse(doc["key_positive"])
    self.assertIn("no further keys", doc["key_note"])
    self.assertIsNone(doc["write_unlocked"])

  def test_phase4_invalid_key_nrc_classes_recorded(self):
    for nrc in (0x35, 0x36, 0x37):        # invalidKey / exceededNumberOfAttempts / requiredTimeDelayNotExpired
      with self.subTest(nrc=nrc):
        self.fresh_state()
        self.set_state(key_mode="identity2")
        car = self.p4_car()
        car.key_nrc = nrc
        s = self.run_car(car, key=f"nrc{nrc}")
        self.assertTrue(s["ran"])
        self.assertEqual(self.result_doc()["key_nrc"], nrc)
        self.assertEqual(self.write_frames(car), [])

  # ---- (e) key silence -> recorded, no 2E -----------------------------------------------------------------------
  def test_phase4_key_silent_no_write(self):
    self.set_state(key_mode="identity2")
    car = self.p4_car()
    car.key_silent = True
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["error"])
    self.assertFalse(s["unlocked"])
    self.assertEqual(self.write_frames(car), [])
    doc = self.result_doc()
    self.assertTrue(doc["key_result"]["no_response"])
    self.assertFalse(doc["key_attempted"] is None)      # it was attempted; only the answer was silent
    self.assertIn("no further keys", doc["key_note"])

  # ---- (f) seed refused (7F 27 7F) -> abort; no key, no write ----------------------------------------------------
  def test_phase4_seed_refused_aborts_before_key(self):
    self.set_state(key_mode="identity2")
    car = self.p4_car()
    car.seed_refused = True
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIn("seed not positive", s["aborted"])
    self.assertFalse(s["key_attempted"])
    self.assertFalse(s["write_attempted"])
    self.assertEqual([d for _, d, _ in car.sent if d[1] == 0x27 and d[2] == 0x02], [])
    self.assertEqual(self.write_frames(car), [])
    self.assertEqual(self.tx_frames(car), [self.READ, self.SESSION, self.SEED01])
    self.assertEqual(car.mux_calls, [True, False])

  def test_phase4_seed_silent_aborts_before_key(self):
    self.set_state(key_mode="identity2")
    car = self.p4_car()
    car.seed_silent = True
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIn("seed not positive", s["aborted"])
    self.assertEqual([d for _, d, _ in car.sent if d[1] == 0x27 and d[2] == 0x02], [])

  # ---- (g) 10 03 refused -> abort -------------------------------------------------------------------------------
  def test_phase4_session_refused_aborts(self):
    self.set_state(key_mode="identity2")
    car = self.p4_car()
    car.session_refused = True            # 7F 10 12 - phase 4 REQUIRES a positive 50 03
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIn("extended session", s["aborted"])
    self.assertFalse(s["key_attempted"])
    self.assertFalse(s["write_attempted"])
    self.assertEqual([d for _, d, _ in car.sent if d[1] == 0x27 and d[2] == 0x02], [])
    self.assertEqual(self.tx_frames(car), [self.READ, self.SESSION])
    self.assertEqual(car.mux_calls, [True, False])

  def test_phase4_session_silent_aborts(self):
    self.set_state(key_mode="identity2")
    car = self.p4_car()
    car.session_silent = True
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIn("extended session", s["aborted"])
    self.assertFalse(s["key_attempted"])

  # ---- (h) second 27 02 raises ----------------------------------------------------------------------------------
  def test_phase4_second_send_key_raises(self):
    car = self.p4_car()
    c = self.client4(car)
    c.send_key_candidate(PHASE4_SEED[:2])
    self.assertEqual(c.key_attempts, 1)
    with self.assertRaises(D.SafetyViolation):
      c.send_key_candidate(PHASE4_SEED[:2])          # a second 27 02 can never reach the bus
    self.assertEqual(len([d for _, d, _ in car.sent if d[1] == 0x27 and d[2] == 0x02]), 1)

  def test_phase4_send_key_rejects_other_lengths(self):
    car = self.p4_car()
    c = self.client4(car)
    for bad in (b"\x01", b"\x01\x02\x03", b"\x01\x02\x03\x04\x05", b"\x01" * 6, b""):
      with self.assertRaises(D.SafetyViolation):
        c.send_key_candidate(bad)

  # ---- (i) wrong key raises (guard pinned to the resolved candidate) --------------------------------------------
  def test_phase4_guard_frame_pins_key_to_resolved_candidate(self):
    good = (bytes([4, 0x27, 0x02]) + PHASE4_SEED[:2]).ljust(8, b"\x00")
    E.guard_frame(0x7D1, good, 1, phase=4, key=PHASE4_SEED[:2], key_attempts=0)      # exactly right key
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([4, 0x27, 0x02, 0x11, 0x22]).ljust(8, b"\x00"), 1, phase=4, key=PHASE4_SEED[:2])
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, good, 1, phase=4, key=PHASE4_SEED[:2], key_attempts=1)   # second attempt
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, good, 1, phase=4, key=None)                             # no resolved key
    # frame length vs key length must agree (04 needs 2 key bytes, 06 needs 4)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([6, 0x27, 0x02, 0x5A, 0xB0, 0x00]).ljust(8, b"\x00"), 1, phase=4,
                    key=PHASE4_SEED[:2], key_attempts=0)
    # a 2-byte key never goes out for phase 3 (whose pin is the 8-byte seed), and 27 02 never for phases 1/2
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, good, 1, phase=3, seed=PHASE4_SEED, key_attempts=0)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, good, 1, phase=1, key=PHASE4_SEED[:2], key_attempts=0)

  def test_phase4_guard_service_admits_sendkey_only_in_phases_3_and_4(self):
    for phase in (3, 4):
      E.guard_service(0x27, 0x02, None, phase=phase)     # admitted
    for phase in (1, 2):
      with self.assertRaises(D.SafetyViolation):
        E.guard_service(0x27, 0x02, None, phase=phase)

  # ---- (j) bad key_mode -> skip/abort; bad hex -> recorded abort ------------------------------------------------
  def test_phase4_unknown_key_mode_skips(self):
    for mode in ("bogus", "", 5, "identity3", "IDENTITY2"):
      with self.subTest(mode=mode):
        self.fresh_state()
        self.set_state(key_mode=mode)
        car = self.p4_car()
        s = self.run_car(car, key=f"km{mode}")
        self.assertFalse(s["ran"])
        self.assertIn("unknown key_mode", s["skip"])
        self.assertEqual(car.sent, [])
        self.assertEqual(car.mux_calls, [])

  def test_phase4_default_key_mode_is_identity2(self):
    with open(os.path.join(self.out, "state.json"), "w") as f:
      json.dump({"probe_enabled": True, "phase": 4}, f)    # no key_mode at all
    car = self.p4_car()
    car.key_unlock = True
    car.write_refused = False
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertEqual(self.result_doc()["key"], "5ab0")
    self.assertEqual(self.result_doc()["key_mode"], "identity2")

  def test_phase4_hex_mode_uses_key_hex(self):
    self.set_state(key_mode="hex", key_hex="a1b2c3d4")      # 4-byte key
    car = self.p4_car()
    car.key_unlock = True
    car.write_refused = False
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["aborted"])
    key4 = (bytes([6, 0x27, 0x02, 0xA1, 0xB2, 0xC3, 0xD4])).ljust(8, b"\x00")
    self.assertEqual([d for _, d, _ in car.sent if d[1] == 0x27 and d[2] == 0x02], [key4])
    self.assertEqual(self.result_doc()["key"], "a1b2c3d4")

  def test_phase4_hex_mode_bad_hex_aborts_recorded(self):
    self.set_state(key_mode="hex", key_hex="zz")
    car = self.p4_car()
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIn("bad key_hex", s["aborted"])
    self.assertFalse(s["key_attempted"])
    self.assertEqual([d for _, d, _ in car.sent if d[1] == 0x27 and d[2] == 0x02], [])

  def test_phase4_hex_mode_wrong_length_aborts(self):
    self.set_state(key_mode="hex", key_hex="a1b2c3")        # 3 bytes: never admissible
    car = self.p4_car()
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIn("resolved key length", s["aborted"])
    self.assertEqual([d for _, d, _ in car.sent if d[1] == 0x27 and d[2] == 0x02], [])

  def test_phase4_algo_mode_missing_module_aborts(self):
    # A missing module (None in sys.modules) must abort (recorded), never send an unpinned key.
    import sys
    self.set_state(key_mode="algo")
    car = self.p4_car()
    with mock.patch.dict(sys.modules, {"openpilot.sunnypilot.fork.esc_probe_seedkey": None}):
      s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIn("algo module missing/failed", s["aborted"])
    self.assertEqual([d for _, d, _ in car.sent if d[1] == 0x27 and d[2] == 0x02], [])
    self.assertEqual(self.write_frames(car), [])

  def test_phase4_algo_mode_uses_resolved_key(self):
    from types import SimpleNamespace
    import openpilot.sunnypilot.fork as F
    import sys
    fake = SimpleNamespace(key_for=lambda seed, algo=None: bytes(seed)[:2])
    self.set_state(key_mode="algo")
    car = self.p4_car()
    car.key_unlock = True
    car.write_refused = False
    # `import a.b.c as x` binds x from the PARENT package attribute, so patch that too (sys.modules alone is not enough)
    with mock.patch.dict(sys.modules, {"openpilot.sunnypilot.fork.esc_probe_seedkey": fake}), \
         mock.patch.object(F, "esc_probe_seedkey", fake):
      s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["aborted"])
    self.assertEqual(self.result_doc()["key"], "5ab0")
    self.assertEqual([d for _, d, _ in car.sent if d[1] == 0x27 and d[2] == 0x02], [self.KEY2])

  # ---- (k) algo mode: the recovered G-scan CalKeyAlgorithm_* family, selectable by Securityindex -----------------
  ALGO_SEED = bytes.fromhex("1122334455667788")    # a generic non-repeating 8-byte seed (no zero byte)
  # Frozen literal vectors computed ONCE from esc_probe_seedkey.key_for(<seed>, <algo>) -- the ported algorithms are
  # byte-identical to car-features/esc-software/06-git/12-seedkey.py, so a future algorithm regression breaks these.
  ALGO_27100_KEY = bytes.fromhex("f30f0000")       # key_for(1122334455667788, "27100")
  ALGO_26700_KEY = bytes.fromhex("2375f360")       # key_for(1122334455667788, "26700")

  def test_phase4_algo_27100_frame_matches_module_and_literal(self):
    self.assertEqual(SK.key_for(self.ALGO_SEED, "27100"), self.ALGO_27100_KEY)   # module output == frozen literal
    self.set_state(key_mode="algo", algo="27100")
    car = self.p4_car()
    car.esc_seed8 = self.ALGO_SEED      # the fake ESC answers 27 01 with THIS 8-byte seed
    car.key_unlock = True
    car.write_refused = False
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["aborted"])
    key4 = (bytes([6, 0x27, 0x02]) + self.ALGO_27100_KEY).ljust(8, b"\x00")
    self.assertEqual(key4, bytes([6, 0x27, 0x02, 0xF3, 0x0F, 0x00, 0x00]).ljust(8, b"\x00"))
    keys = [d for _, d, _ in car.sent if d[1] == 0x27 and d[2] == 0x02]
    self.assertEqual(keys, [key4])                       # exactly one 27 02, carrying the module's key bytes
    doc = self.result_doc()
    self.assertEqual(doc["algo"], "27100")
    self.assertEqual(doc["key"], "f30f0000")
    self.assertEqual(doc["key_bytes"], "f30f0000")       # recorded before TX
    self.assertTrue(doc["unlocked"])
    self.assertEqual(s["algo"], "27100")                 # (e) both fields surface in the summary/cloudlog too
    self.assertEqual(s["key_bytes"], "f30f0000")

  def test_phase4_algo_26700_single_frame_4byte(self):
    self.assertEqual(SK.key_for(self.ALGO_SEED, "26700"), self.ALGO_26700_KEY)
    self.set_state(key_mode="algo", algo="26700")
    car = self.p4_car()
    car.esc_seed8 = self.ALGO_SEED
    car.key_unlock = True
    car.write_refused = False
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["aborted"])
    key4 = (bytes([6, 0x27, 0x02]) + self.ALGO_26700_KEY).ljust(8, b"\x00")   # 06 27 02 23 75 F3 60
    self.assertEqual([d for _, d, _ in car.sent if d[1] == 0x27 and d[2] == 0x02], [key4])
    doc = self.result_doc()
    self.assertEqual(doc["algo"], "26700")
    self.assertEqual(doc["key_bytes"], "2375f360")

  def test_phase4_algo_zero_byte_seed_bails_no_frame(self):
    # seed[0:4] = 11 00 33 44 contains a zero byte -> cal_27100's vendor bail -> key_for None -> recorded abort.
    self.assertIsNone(SK.key_for(bytes.fromhex("1100334455667788"), "27100"))
    self.set_state(key_mode="algo", algo="27100")
    car = self.p4_car()
    car.esc_seed8 = bytes.fromhex("1100334455667788")
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIn("algo returned no key for this seed (zero-byte bail)", s["aborted"])
    self.assertFalse(s["key_attempted"])
    self.assertEqual([d for _, d, _ in car.sent if d[1] == 0x27 and d[2] == 0x02], [])   # NO 27 02 frame in tx
    self.assertEqual(self.write_frames(car), [])

  def test_phase4_algo_unknown_name_aborts_no_key(self):
    self.set_state(key_mode="algo", algo="99999")     # not a recovered Securityindex
    car = self.p4_car()
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIn("algo module missing/failed", s["aborted"])
    self.assertFalse(s["key_attempted"])
    self.assertEqual([d for _, d, _ in car.sent if d[1] == 0x27 and d[2] == 0x02], [])
    self.assertEqual(self.write_frames(car), [])

  # ---- (l) 8-byte CONSTRUCTION modes (algo8/algo8p/repeat8/hex8) -----------------------------------------------
  # On-car evidence: a 4-byte key drew 7F 27 13 (incorrect length) and the 8-byte seed-as-key drew 7F 27 35
  # (invalidKey), so 8 bytes is the accepted LENGTH and VALUE iteration begins. Every new mode resolves to exactly 8
  # wire bytes -> the ISO-TP multi-frame shape (10 0A 27 02 K1..K4 + 21 K5..K8).
  ALGO_26400_KEY = bytes.fromhex("37e2")          # key_for(1122334455667788, "26400") (2-byte algo output)
  # 27100's vendor key is the 4-byte f30f0000 (double sprintf+concat -- see esc_probe_seedkey.py).
  #   algo8  (full-output repeat k*2) = f30f0000f30f0000
  #   algo8w (first TWO bytes [lo,hi]=f30f repeated x4) = f30ff30ff30ff30f  <- the on-car seed's own wire shape
  ALGO_27100_ALGO8 = bytes.fromhex("f30f0000f30f0000")    # key_for(1122334455667788,"27100") * 2
  ALGO_27100_WIRE8 = bytes.fromhex("f30ff30ff30ff30f")    # key_for(1122334455667788,"27100")[:2] * 4 = [lo,hi]x4

  def test_phase4_algo8_2byte_algo_repeats_to_8(self):
    # (a) algo8 with a 2-byte algorithm (26400): k = cal_26400(seed[:2]) -> repeated to fill: k*4 = 8 bytes
    self.assertEqual(SK.key_for(self.ALGO_SEED, "26400"), self.ALGO_26400_KEY)
    self.set_state(key_mode="algo8", algo="26400")
    car = self.p4_car()
    car.esc_seed8 = self.ALGO_SEED
    car.key_unlock = True
    car.write_refused = False
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["aborted"])
    self.assertIsNone(s["error"])
    key8 = self.ALGO_26400_KEY * 4
    self.assertEqual(key8.hex(), "37e237e237e237e2")
    ff = bytes([0x10, 0x0A, 0x27, 0x02]) + key8[:4]              # FF: 10 0A 27 02 37 E2 37 E2
    cf = (bytes([0x21]) + key8[4:8]).ljust(8, b"\x00")           # CF: 21 37 E2 37 E2
    self.assertEqual(ff, bytes([0x10, 0x0A, 0x27, 0x02, 0x37, 0xE2, 0x37, 0xE2]))
    self.assertEqual(cf, bytes([0x21, 0x37, 0xE2, 0x37, 0xE2]).ljust(8, b"\x00"))
    self.assertEqual(self.tx_frames(car),
                     [self.READ, self.SESSION, self.SEED01, self.FC, ff, cf, self.WRITE, self.READ])
    self.assertEqual([d for _, d, _ in car.sent if d[:4] == bytes([0x10, 0x0A, 0x27, 0x02])], [ff])
    self.assertEqual([d for _, d, _ in car.sent if d[0] == 0x21], [cf])
    doc = self.result_doc()
    self.assertEqual(doc["key_mode"], "algo8")
    self.assertEqual(doc["algo"], "26400")
    self.assertEqual(doc["key"], "37e237e237e237e2")
    self.assertEqual(doc["key_bytes"], "37e237e237e237e2")
    self.assertTrue(doc["unlocked"])

  def test_phase4_algo8_4byte_algo_doubles_to_8(self):
    # (b) algo8 with a 4-byte algorithm (26700): key = cal_26700(seed[:4]) * 2 -> 8 bytes
    self.set_state(key_mode="algo8", algo="26700")
    car = self.p4_car()
    car.esc_seed8 = self.ALGO_SEED
    car.key_unlock = True
    car.write_refused = False
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["aborted"])
    key8 = self.ALGO_26700_KEY * 2
    self.assertEqual(key8.hex(), "2375f3602375f360")
    ff = bytes([0x10, 0x0A, 0x27, 0x02]) + key8[:4]
    cf = (bytes([0x21]) + key8[4:8]).ljust(8, b"\x00")
    self.assertEqual(self.tx_frames(car),
                     [self.READ, self.SESSION, self.SEED01, self.FC, ff, cf, self.WRITE, self.READ])
    doc = self.result_doc()
    self.assertEqual(doc["algo"], "26700")
    self.assertEqual(doc["key"], "2375f3602375f360")
    self.assertEqual(doc["key_bytes"], "2375f3602375f360")

  def test_phase4_algo8w_27100_first_two_bytes_repeated(self):
    # algo8w: the resolved key's FIRST TWO BYTES repeated to fill 8. 27100's vendor output is the 4-byte
    # f30f0000 (double sprintf+concat -- see esc_probe_seedkey.py), so algo8w = [lo,hi]x4 = f30f x4 =
    # f30ff30ff30ff30f, sent as ISO-TP FF 10 0A 27 02 F30FF30F + CF 21 F30FF30F. algo8 (full-output
    # repeat) is the DISTINCT f30f0000f30f0000 -- both compared against the 4-byte vendor key f30f0000.
    self.assertEqual(SK.key_for(self.ALGO_SEED, "27100"), bytes.fromhex("f30f0000"))       # 4-byte vendor key
    self.assertEqual(E.resolve_key(self.ALGO_SEED, "algo8", None, "27100"), self.ALGO_27100_ALGO8)
    self.set_state(key_mode="algo8w", algo="27100")
    car = self.p4_car()
    car.esc_seed8 = self.ALGO_SEED
    car.key_unlock = True
    car.write_refused = False
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["aborted"])
    self.assertIsNone(s["error"])
    key8 = self.ALGO_27100_WIRE8
    self.assertEqual(key8.hex(), "f30ff30ff30ff30f")
    ff = bytes([0x10, 0x0A, 0x27, 0x02]) + key8[:4]              # FF: 10 0A 27 02 F3 0F F3 0F
    cf = (bytes([0x21]) + key8[4:8]).ljust(8, b"\x00")           # CF: 21 F3 0F F3 0F
    self.assertEqual(ff, bytes([0x10, 0x0A, 0x27, 0x02, 0xF3, 0x0F, 0xF3, 0x0F]))
    self.assertEqual(cf, bytes([0x21, 0xF3, 0x0F, 0xF3, 0x0F]).ljust(8, b"\x00"))
    self.assertEqual([d for _, d, _ in car.sent if d[:4] == bytes([0x10, 0x0A, 0x27, 0x02])], [ff])
    self.assertEqual([d for _, d, _ in car.sent if d[0] == 0x21], [cf])
    doc = self.result_doc()
    self.assertEqual(doc["key_mode"], "algo8w")
    self.assertEqual(doc["algo"], "27100")
    self.assertEqual(doc["key"], "f30ff30ff30ff30f")
    self.assertEqual(doc["key_bytes"], "f30ff30ff30ff30f")
    self.assertTrue(doc["unlocked"])

  def test_phase4_algo8p_4byte_algo_pads_to_8(self):
    # (c) algo8p with a 4-byte algorithm (26700): k4 + 0000 (zero padding to 8)
    self.set_state(key_mode="algo8p", algo="26700")
    car = self.p4_car()
    car.esc_seed8 = self.ALGO_SEED
    car.key_unlock = True
    car.write_refused = False
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["aborted"])
    key8 = bytes.fromhex("2375f36000000000")
    ff = bytes([0x10, 0x0A, 0x27, 0x02]) + key8[:4]
    cf = (bytes([0x21]) + key8[4:8]).ljust(8, b"\x00")
    self.assertEqual(ff, bytes([0x10, 0x0A, 0x27, 0x02, 0x23, 0x75, 0xF3, 0x60]))
    self.assertEqual(cf, bytes([0x21, 0x00, 0x00, 0x00, 0x00]).ljust(8, b"\x00"))
    self.assertEqual(self.tx_frames(car),
                     [self.READ, self.SESSION, self.SEED01, self.FC, ff, cf, self.WRITE, self.READ])
    doc = self.result_doc()
    self.assertEqual(doc["key_mode"], "algo8p")
    self.assertEqual(doc["key"], "2375f36000000000")
    self.assertEqual(doc["key_bytes"], "2375f36000000000")

  def test_phase4_repeat8_is_seed_first_two_repeated(self):
    # (d) repeat8: key == seed[:2] * 4 (the seed's own 2-byte value repeated)
    self.set_state(key_mode="repeat8")
    car = self.p4_car()                      # seed 5AB05AB05AB05AB0 -> 5AB0 x4
    car.key_unlock = True
    car.write_refused = False
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["aborted"])
    key8 = PHASE4_SEED[:2] * 4
    self.assertEqual(key8.hex(), "5ab05ab05ab05ab0")
    ff = bytes([0x10, 0x0A, 0x27, 0x02]) + key8[:4]
    cf = (bytes([0x21]) + key8[4:8]).ljust(8, b"\x00")
    self.assertEqual([d for _, d, _ in car.sent if d[:4] == bytes([0x10, 0x0A, 0x27, 0x02])], [ff])
    self.assertEqual([d for _, d, _ in car.sent if d[0] == 0x21], [cf])
    doc = self.result_doc()
    self.assertEqual(doc["key_mode"], "repeat8")
    self.assertEqual(doc["key"], key8.hex())
    self.assertEqual(doc["key_bytes"], key8.hex())

  def test_phase4_hex8_requires_exactly_eight_bytes(self):
    # (e) hex8 accepts EXACTLY 8 bytes: any other length -> abort, NO 27 02 frame (stricter than hex)
    for bad in ("a1b2c3d4", "a1b2c3d4e5", "a1b2", ""):
      with self.subTest(bad=bad):
        self.fresh_state()
        self.set_state(key_mode="hex8", key_hex=bad)
        car = self.p4_car()
        s = self.run_car(car, key=f"hex8-{bad}")
        self.assertTrue(s["ran"])
        self.assertIn("exactly 8 bytes", s["aborted"])
        self.assertFalse(s["key_attempted"])
        self.assertEqual([d for _, d, _ in car.sent if d[1] == 0x27 and d[2] == 0x02], [])
        self.assertEqual(self.write_frames(car), [])

  def test_phase4_hex8_uses_key_hex(self):
    self.set_state(key_mode="hex8", key_hex="0123456789abcdef")
    car = self.p4_car()
    car.key_unlock = True
    car.write_refused = False
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["aborted"])
    key8 = bytes.fromhex("0123456789abcdef")
    ff = bytes([0x10, 0x0A, 0x27, 0x02]) + key8[:4]
    cf = (bytes([0x21]) + key8[4:8]).ljust(8, b"\x00")
    self.assertEqual([d for _, d, _ in car.sent if d[:4] == bytes([0x10, 0x0A, 0x27, 0x02])], [ff])
    self.assertEqual([d for _, d, _ in car.sent if d[0] == 0x21], [cf])
    self.assertEqual(self.result_doc()["key"], "0123456789abcdef")

  def test_phase4_algo8p_none_key_aborts_no_frame(self):
    # (f) algo8p with a zero-byte seed -> key_for None -> recorded abort, NO 27 02
    self.assertIsNone(SK.key_for(bytes.fromhex("1100334455667788"), "27100"))
    self.set_state(key_mode="algo8p", algo="27100")
    car = self.p4_car()
    car.esc_seed8 = bytes.fromhex("1100334455667788")
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIn("algo returned no key for this seed (zero-byte bail)", s["aborted"])
    self.assertFalse(s["key_attempted"])
    self.assertEqual([d for _, d, _ in car.sent if d[1] == 0x27 and d[2] == 0x02], [])
    self.assertEqual(self.write_frames(car), [])

  def test_phase4_guard_frame_pins_8byte_construction_modes(self):
    # (g) an 8-byte key from a construction mode goes through the SAME single-attempt counter + candidate pinning:
    # guard_frame(seed=key) admits the FF only with the resolved bytes and only as the first attempt; the CF only once.
    key8 = PHASE4_SEED[:2] * 4
    ff = bytes([0x10, 0x0A, 0x27, 0x02]) + key8[:4]
    cf = (bytes([0x21]) + key8[4:8]).ljust(8, b"\x00")
    E.guard_frame(0x7D1, ff, 1, phase=4, seed=key8, key_attempts=0)     # pinned to the resolved 8 bytes
    E.guard_frame(0x7D1, cf, 1, phase=4, seed=key8, key_attempts=1)     # the one consecutive frame
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, ff, 1, phase=4, seed=key8, key_attempts=1)   # a second key attempt is refused
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([0x10, 0x0A, 0x27, 0x02, 0, 0, 0, 0]), 1, phase=4, seed=key8, key_attempts=0)

  def test_phase4_budget_is_thirty_seconds(self):
    # phase 4 shares the phase-3 budget (the added 500 ms delay + key step)
    self.assertEqual(E.RUN_BUDGET_S_PHASE3, 30.0)

  def test_phase4_no_current_value_aborts(self):
    self.set_state(key_mode="identity2")
    car = self.p4_car()
    car.read_refused = True
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIn("step-1 read", s["aborted"])
    self.assertFalse(s["key_attempted"])
    self.assertEqual([d for _, d, _ in car.sent if d[1] == 0x27 and d[2] == 0x02], [])


class TestPhase5(Base):
  """Phase 5 (state ``{"probe_enabled": true, "phase": 5}``): the READ-ONLY capability battery -- a FIXED frame list,
  no keys, no writes, no multi-frame TX. Every guard is mechanically enforced; only refusals are recorded, never fatal."""

  CANARY = bytes([3, 0x22, 0xF1, 0x00]).ljust(8, b"\x00")
  READ = bytes([3, 0x22, 0x01, 0x03]).ljust(8, b"\x00")
  SESSION = bytes([2, 0x10, 0x03]).ljust(8, b"\x00")
  SEED = bytes([2, 0x27, 0x01]).ljust(8, b"\x00")
  # Hard-coded (NOT derived from E.PHASE5_*): a mutant that drops or reorders a probe must change the TX list and fail.
  SUB_SUBS = (0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x0F, 0x11, 0x41, 0x61)
  SVC_SVCS = (0x23, 0x29, 0x31, 0x34, 0x35, 0x36, 0x37)
  SUB = [bytes([2, 0x27, s]).ljust(8, b"\x00") for s in SUB_SUBS]
  SVC = [bytes([1, s]).ljust(8, b"\x00") for s in SVC_SVCS]
  FC = D.FLOW_CONTROL_FRAME

  def setUp(self):
    super().setUp()
    self.set_phase(5)

  def set_phase(self, phase):
    with open(os.path.join(self.out, "state.json"), "w") as f:
      json.dump({"probe_enabled": True, "phase": phase}, f)

  def p5_car(self, **kw):
    car = FakeCar(**kw)
    car.esc_seed8 = SEED8          # the real ESC answers 27 01 with an 8-byte seed (one FC per seed ask)
    return car

  def expected_tx(self):
    # 8-byte seed answers -> the client emits ONE flow-control frame after each of seed1/seed2/seed3
    return [self.CANARY, self.READ, self.SESSION,
            self.SEED, self.FC, self.SEED, self.FC,
            *self.SUB, *self.SVC,
            self.SEED, self.FC, self.READ,
            self.SESSION, self.SESSION]

  def step_names(self, doc):
    return [st["name"] for st in doc["steps"]]

  def test_battery_exact_tx_order(self):
    car = self.p5_car()
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["aborted"])
    self.assertIsNone(s["error"])
    self.assertEqual(self.tx_frames(car), self.expected_tx())
    # the ESC request addr carries everything except the two extra-address peeks
    addrs = [a for a, _, _ in car.sent]
    self.assertEqual(addrs.count(0x770), 1)
    self.assertEqual(addrs.count(0x7A0), 1)
    # the two extra-address peeks are the 10 03 frame and nothing else
    self.assertEqual([d for a, d, _ in car.sent if a == 0x770], [self.SESSION])
    self.assertEqual([d for a, d, _ in car.sent if a == 0x7A0], [self.SESSION])
    self.assertEqual(car.mux_calls, [True, False])
    self.assertTrue(s["mux_restored"])

  def test_battery_step_names_and_order(self):
    car = self.p5_car()
    self.run_car(car)
    names = self.step_names(self.result_doc())
    self.assertEqual(names, ["fp_canary", "read_esc", "session", "seed1", "seed2",
                             *[f"sub_{s:02X}" for s in E.PHASE5_SEC_SUB_PROBES],
                             *[f"svc_{s:02X}" for s in E.PHASE5_SVC_PROBES],
                             "seed3", "read_esc_end", "extra_770", "extra_7A0"])

  def test_no_keys_no_writes_and_no_multiframe(self):
    car = self.p5_car()
    self.run_car(car)
    frames = self.tx_frames(car)
    # never a 27 02 sendKey, never a 2E write, never an ISO-TP multi-frame TX
    self.assertEqual([d for d in frames if d[1] == 0x27 and d[2] == 0x02], [])
    self.assertEqual([d for d in frames if d[1] == 0x2E], [])
    self.assertEqual([d for d in frames if d != self.FC and d[0] >> 4 == 1], [])
    self.assertEqual([d for d in frames if d != self.FC and d[0] == 0x21], [])
    doc = self.result_doc()
    self.assertFalse(doc["key_attempted"])
    self.assertFalse(doc["write_attempted"])
    self.assertIsNone(doc["key"])

  def test_result_records_every_probe(self):
    car = self.p5_car()
    self.run_car(car)
    doc = self.result_doc()
    self.assertEqual(doc["fp_canary"]["nrc"], 0x31)          # 22 F100 -> 7F 22 31 on the fake
    self.assertEqual(doc["fp_canary_hex"], "7f2231")
    self.assertEqual(doc["value_start"], "90060350")
    self.assertEqual(doc["value_end"], "90060350")
    self.assertFalse(doc["value_changed"])
    self.assertEqual(doc["seed1_hex"], SEED8.hex())
    self.assertEqual(doc["seed2_hex"], SEED8.hex())
    self.assertEqual(doc["seed3_hex"], SEED8.hex())
    self.assertTrue(doc["seed_stable_12"])
    self.assertTrue(doc["seed_stable_all"])
    self.assertEqual([r["sub"] for r in doc["sub_probes"]], list(E.PHASE5_SEC_SUB_PROBES))
    self.assertTrue(all(r["nrc"] == 0x12 for r in doc["sub_probes"]))
    self.assertEqual([r["svc"] for r in doc["svc_probes"]], list(E.PHASE5_SVC_PROBES))
    self.assertEqual({r["svc"]: r["nrc"] for r in doc["svc_probes"]},
                     {0x23: 0x11, 0x29: 0x11, 0x31: 0x31, 0x34: 0x11, 0x35: 0x11, 0x36: 0x11, 0x37: 0x11})
    self.assertEqual([r["req_addr"] for r in doc["extra_probes"]], [0x770, 0x7A0])
    self.assertTrue(all(r["no_response"] for r in doc["extra_probes"]))   # fake: silent on 0x778/0x7A8
    # a per-frame receive timestamp for every answered step
    for st in doc["steps"]:
      if st["frames"]:
        self.assertEqual(len(st["frames_t_ms"]), len(st["frames"]), st["name"])
        self.assertTrue(all(ms >= 0 for ms in st["frames_t_ms"]), st["name"])

  def test_summary_fields(self):
    car = self.p5_car()
    s = self.run_car(car)
    self.assertEqual(s["phase"], 5)
    self.assertEqual(s["fp_canary_hex"], "7f2231")
    self.assertEqual(s["value_start"], "90060350")
    self.assertEqual(s["value_end"], "90060350")
    self.assertEqual(s["seed1"], SEED8.hex())
    self.assertTrue(s["seed_stable_12"])
    self.assertTrue(s["seed_stable_all"])
    self.assertFalse(s["key_attempted"])
    self.assertFalse(s["write_attempted"])
    self.assertEqual(s["sub_probes"][0]["sub"], 0x03)
    self.assertEqual(len(s["sub_probes"]), 10)
    self.assertEqual(len(s["svc_probes"]), 7)
    self.assertEqual(len(s["extra_probes"]), 2)
    self.assertEqual(self.events[-1][0], "esc_probe_0027")
    self.assertLessEqual(s["duration_s"], E.RUN_BUDGET_S_PHASE5)

  def test_fp_canary_positive(self):
    car = self.p5_car()
    car.fp_canary_nrc = None            # 22 F100 -> 62 F100 01020304
    self.run_car(car)
    doc = self.result_doc()
    self.assertEqual(doc["fp_canary_hex"], "62f10001020304")
    self.assertTrue(doc["fp_canary"]["positive"])

  def test_unstable_seed_flags_false(self):
    car = self.p5_car()
    car.seed_sequence = [SEED8, SEED8[:7] + b"\x99", SEED8]   # seed2 differs from seed1
    self.run_car(car)
    doc = self.result_doc()
    self.assertFalse(doc["seed_stable_12"])
    self.assertFalse(doc["seed_stable_all"])

  def test_seed_stable_all_false_when_only_third_differs(self):
    car = self.p5_car()
    car.seed_sequence = [SEED8, SEED8, bytes([0x77]) * 8]
    self.run_car(car)
    doc = self.result_doc()
    self.assertTrue(doc["seed_stable_12"])
    self.assertFalse(doc["seed_stable_all"])

  def test_refused_read_is_recorded_not_fatal(self):
    car = self.p5_car()
    car.read_refused = True             # 22 0103 -> 7F 22 31; read_end -> 62? no, still refused
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["aborted"])     # phase 5 never aborts on a refusal
    doc = self.result_doc()
    self.assertIsNone(doc["value_start"])
    self.assertIsNone(doc["value_end"])
    self.assertEqual(self.step_names(doc)[1], "read_esc")

  def test_extra_addr_answers_are_recorded(self):
    car = self.p5_car()
    car.extra_answer = {0x770: True}    # 0x770 answers 50 03; 0x7A0 stays silent
    self.run_car(car)
    by = {r["req_addr"]: r for r in self.result_doc()["extra_probes"]}
    self.assertTrue(by[0x770]["positive"])
    self.assertEqual(by[0x770]["resp"], "5003003201f4")
    self.assertTrue(by[0x7A0]["no_response"])

  def test_once_per_ignition(self):
    car = self.p5_car()
    self.run_car(car, key="boot:1")
    n = len(car.sent)
    s = self.run_car(car, key="boot:1")
    self.assertFalse(s["ran"])
    self.assertEqual(s["skip"], "already done (ignition)")
    self.assertEqual(len(car.sent), n)

  def test_not_in_park_sends_nothing(self):
    car = self.p5_car(gear=5)
    s = self.run_car(car)
    self.assertFalse(s["ran"])
    self.assertEqual(car.sent, [])
    self.assertEqual(car.mux_calls, [])

  def test_phase5_budget_is_forty_seconds(self):
    self.assertEqual(E.RUN_BUDGET_S_PHASE5, 40.0)
    self.assertEqual(E.RUN_BUDGET_S, 15.0)

  # ---- guards: the closed phase-5 allowlist ---------------------------------------------------------------
  def test_guard_service_phase5_closed_set(self):
    E.guard_service(0x22, None, 0xF100, phase=5)
    E.guard_service(0x22, None, 0x0103, phase=5)
    E.guard_service(0x10, 0x03, None, phase=5)
    E.guard_service(0x27, 0x01, None, phase=5)
    for s in E.PHASE5_SEC_SUB_PROBES:
      E.guard_service(0x27, s, None, phase=5)
    for s in E.PHASE5_SVC_PROBES:
      E.guard_service(s, None, None, phase=5)
    # 27 02 sendKey is NEVER admissible in phase 5
    with self.assertRaises(D.SafetyViolation):
      E.guard_service(0x27, 0x02, None, phase=5)
    # no 2E write
    with self.assertRaises(D.SafetyViolation):
      E.guard_service(0x2E, None, 0x0103, phase=5)
    # 10 only sub 03 (10 83 refused)
    for sub in (0x01, 0x02, 0x81, 0x83):
      with self.assertRaises(D.SafetyViolation):
        E.guard_service(0x10, sub, None, phase=5)
    # only DIDs F100/0103
    for did in (0x0104, 0xF101, 0x0000):
      with self.assertRaises(D.SafetyViolation):
        E.guard_service(0x22, None, did, phase=5)
    # bare service probes only (a sub-function on 23/29/31/34/35/36/37 is refused)
    for svc in E.PHASE5_SVC_PROBES:
      with self.assertRaises(D.SafetyViolation):
        E.guard_service(svc, 0x01, None, phase=5)
    # an unlisted 27 sub is refused
    for sub in (0x02, 0x04, 0x00, 0x62):
      with self.assertRaises(D.SafetyViolation):
        E.guard_service(0x27, sub, None, phase=5)

  def test_guard_frame_phase5_closed_shapes(self):
    for f in (self.CANARY, self.READ, self.SESSION, self.SEED, *self.SUB, *self.SVC):
      E.guard_frame(0x7D1, f, 1, phase=5)
    E.guard_frame(0x7D1, self.FC, 1, phase=5)                 # the one flow-control exemption
    # the extra addrs are 10 03 only
    E.guard_frame(0x770, self.SESSION, 1, phase=5)
    E.guard_frame(0x7A0, self.SESSION, 1, phase=5)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x770, self.READ, 1, phase=5)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x771, self.SESSION, 1, phase=5)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7A1, self.SESSION, 1, phase=5)
    # 27 02 never, 2E never
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([4, 0x27, 0x02, 0x11, 0x22]).ljust(8, b"\x00"), 1, phase=5)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([0x10, 0x0A, 0x27, 0x02]) + SEED8[:4], 1, phase=5, seed=SEED8, key_attempts=0)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([7, 0x2E, 0x01, 0x03]) + CURRENT, 1, phase=5, readback=CURRENT)
    # no multi-frame sends at all
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([0x10, 0x0A, 0x27, 0x01]) + b"\x00" * 4, 1, phase=5)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([0x21, 0x00, 0x00]) + b"\x00" * 5, 1, phase=5)
    # 10 only sub 03: a 10 83 programming-session frame is refused (on the ESC and on the extra addrs)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([2, 0x10, 0x83]).ljust(8, b"\x00"), 1, phase=5)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([2, 0x10, 0x01]).ljust(8, b"\x00"), 1, phase=5)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x770, bytes([2, 0x10, 0x83]).ljust(8, b"\x00"), 1, phase=5)
    # wrong DID on the ESC
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([3, 0x22, 0x01, 0x04]).ljust(8, b"\x00"), 1, phase=5)
    # a bare service probe with a sub-function, and an unlisted 27 sub
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([2, 0x29, 0x01]).ljust(8, b"\x00"), 1, phase=5)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([2, 0x27, 0x04]).ljust(8, b"\x00"), 1, phase=5)

  def test_phase5_shapes_are_refused_outside_phase5(self):
    # the phase-5-only shapes must NOT be admissible in phases 1-4 (guards stay byte-identical there)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([1, 0x23]).ljust(8, b"\x00"), 1, phase=1)      # bare 1-byte service probe
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([2, 0x27, 0x03]).ljust(8, b"\x00"), 1, phase=4)  # 27 sub-probe
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x770, self.SESSION, 1, phase=1)                            # extra request addr
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7A0, self.SESSION, 1, phase=3)
    with self.assertRaises(D.SafetyViolation):
      E.guard_service(0x23, None, None, phase=1)                                # 0x23 not in the base allowlist
    with self.assertRaises(D.SafetyViolation):
      E.guard_service(0x27, 0x03, None, phase=3)                                # 27 03 not a phase-3 sub

  def test_single_tx_site_unchanged(self):
    # phase 5 adds no new can_send path: still exactly the two documented TX sites
    tree = ast.parse(inspect.getsource(E))
    calls = [getattr(n.func, "attr", getattr(n.func, "id", "")) for n in ast.walk(tree)
             if isinstance(n, ast.Call) and getattr(n.func, "attr", getattr(n.func, "id", "")) == "_can_send"]
    self.assertEqual(calls, ["_can_send", "_can_send"])



class TestPhase6(Base):
  """Phase 6 (state ``{"probe_enabled": true, "phase": 6}``): the security-policy matrix + DID sweep. Zero-key ONLY,
  27 02 admissible ONLY immediately after a 27 01 and at most 4 times (the ONE 4th being the cycle-reset step), no
  write anywhere. Every guard is mechanically enforced."""

  READ = bytes([3, 0x22, 0x01, 0x03]).ljust(8, b"\x00")
  SESSION = bytes([2, 0x10, 0x03]).ljust(8, b"\x00")
  SESSION01 = bytes([2, 0x10, 0x01]).ljust(8, b"\x00")
  SESSION02 = bytes([2, 0x10, 0x02]).ljust(8, b"\x00")
  SEED = bytes([2, 0x27, 0x01]).ljust(8, b"\x00")
  FC = D.FLOW_CONTROL_FRAME
  KEYFF = bytes([0x10, 0x0A, 0x27, 0x02]) + bytes(4)             # 10 0A 27 02 00 00 00 00
  KEYCF = (bytes([0x21]) + bytes(4)).ljust(8, b"\x00")           # 21 00 00 00 00 00 00 00
  SWEEP = (0xF186, 0xF187, 0xF190, 0xF199, 0xF18A, 0xF18C, 0xF191, 0xF195)
  DTC = bytes([3, 0x19, 0x02, 0xA5]).ljust(8, b"\x00")

  def setUp(self):
    super().setUp()
    self.set_phase(6)

  def set_phase(self, phase):
    with open(os.path.join(self.out, "state.json"), "w") as f:
      json.dump({"probe_enabled": True, "phase": phase}, f)

  def p6_car(self, **kw):
    car = FakeCar(**kw)
    car.esc_seed8 = PHASE4_SEED   # a real-looking 8-byte seed (2-byte value x4) -> MF answers, one FC each
    return car

  def did_frame(self, did):
    return bytes([3, 0x22, did >> 8, did & 0xFF]).ljust(8, b"\x00")

  def expected_tx(self):
    sweep = [self.did_frame(d) for d in self.SWEEP]
    return [self.READ, self.SESSION,
            self.SEED, self.FC, self.SEED, self.FC,
            self.KEYFF, self.KEYCF,
            self.SEED, self.FC, self.KEYFF, self.KEYCF,
            self.SEED, self.FC, self.KEYFF, self.KEYCF,
            self.SESSION01, self.SESSION, self.SEED, self.FC, self.KEYFF, self.KEYCF,
            *sweep,
            self.SESSION02, self.SEED, self.FC,      # 10 02 positive -> 27 01 -> S6 (no key)
            self.DTC,
            self.SESSION01, self.READ]

  def step_names(self, doc):
    return [st["name"] for st in doc["steps"]]

  def client6(self, car):
    car.set_mux(True)
    gate = D.VehicleGate(car.now)
    gate.feed(CanData(0x367, lvr12(0), 0))
    gate.feed(CanData(0x386, whl(0.), 0))
    return E.EscProbeClient(car.can_send, car.can_recv, gate, car.now, car.now() + 99, phase=6)

  # ---- happy path: exact TX order + summary ----------------------------------------------------------------------
  def test_battery_exact_tx_order_and_zero_key(self):
    car = self.p6_car()
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["aborted"])
    self.assertIsNone(s["error"])
    self.assertEqual(self.tx_frames(car), self.expected_tx())
    keys = [d for _, d, _ in car.sent if d[:4] == bytes([0x10, 0x0A, 0x27, 0x02])]
    self.assertEqual(len(keys), 4)                         # exactly FOUR key attempts, all zero key
    for k in keys:
      self.assertEqual(k[4:8], bytes(4))
    self.assertEqual([d for _, d, _ in car.sent if d[1] == 0x2E], [])   # never writes
    self.assertEqual(car.mux_calls, [True, False])
    self.assertTrue(s["mux_restored"])
    doc = self.result_doc()
    self.assertEqual(doc["phase"], 6)
    self.assertTrue(doc["key_attempted"])
    self.assertFalse(doc["write_attempted"])
    self.assertEqual(doc["value_start"], "90060350")
    self.assertEqual(doc["value_end"], "90060350")
    self.assertFalse(doc["value_changed"])
    self.assertTrue(doc["seed_stable_pre"])
    self.assertEqual([doc[k]["nrc"] for k in ("R1", "R2", "R3", "R4")], [0x35, 0x35, 0x35, 0x35])
    self.assertFalse(doc["lockout_seen"])
    self.assertTrue(doc["cycle_reset"])                     # R4 was not 0x36/0x37
    self.assertEqual(self.events[-1][0], "esc_probe_0027")
    self.assertLessEqual(s["duration_s"], E.RUN_BUDGET_S_PHASE6)

  def test_battery_step_names_and_order(self):
    car = self.p6_car()
    self.run_car(car)
    names = self.step_names(self.result_doc())
    self.assertEqual(names, ["read_esc", "session", "S1", "S2", "R1", "S3", "R2", "S4", "R3",
                             "reset_session_default", "reset_session_extended", "S5", "R4",
                             *[f"did_{d:04X}" for d in self.SWEEP],
                             "prog_session_1002", "S6", "dtc_19_02_a5", "leave_session", "read_esc_end"])

  def test_seed_samples_and_stability_fields(self):
    car = self.p6_car()
    self.run_car(car)
    doc = self.result_doc()
    for tag in ("S1", "S2", "S3", "S4", "S5", "S6"):
      self.assertEqual(doc[tag]["resp"], "6701" + PHASE4_SEED.hex(), tag)
    self.assertTrue(doc["seed_stable_pre"])
    self.assertEqual(doc["seed_after_fail"], PHASE4_SEED.hex())   # S3 == S1/S2 (stable)

  def test_seed_after_fail_reports_new_seed(self):
    car = self.p6_car()
    car.seed_sequence = [PHASE4_SEED, PHASE4_SEED, bytes([0x77]) * 8] + [PHASE4_SEED] * 5
    self.run_car(car)
    doc = self.result_doc()
    self.assertTrue(doc["seed_stable_pre"])
    self.assertEqual(doc["seed_after_fail"], "new:" + (bytes([0x77]) * 8).hex())

  def test_did_sweep_records_every_did(self):
    car = self.p6_car()
    for did in (0xF186, 0xF190, 0xF191):
      car.sweep_dids[did] = "07" + f"62{did:04X}".lower() + "41424344"   # ISO-TP 07 62 <did> 41 42 43 44
    self.run_car(car)
    doc = self.result_doc()
    self.assertEqual(set(doc["dids"]), {f"{d:04X}" for d in self.SWEEP})
    self.assertEqual(doc["dids"]["F186"]["resp"], "62f18641424344")
    self.assertEqual(doc["dids"]["F190"]["resp"], "62f19041424344")
    self.assertEqual(doc["dids"]["F187"]["resp"], "7f2231")             # unconfigured -> 7F 22 31
    self.assertEqual(doc["dids"]["F187"]["nrc"], 0x31)
    # the sweep frames really went to the eight DIDs, in order
    dids = [d for _, d, _ in car.sent if d[1] == 0x22 and d[2] == 0xF1 and d[3] != 0x00]
    self.assertEqual(dids, [self.did_frame(d) for d in self.SWEEP])

  # ---- the lockout path: R1 0x36 -> skip 6/7 -> cycle reset R4 ---------------------------------------------------
  def test_lockout_skips_pre_cycle_and_cycle_reset_clears(self):
    car = self.p6_car()
    car.key_nrc_sequence = [0x36, 0x35]      # R1 -> 0x36 (lockout); R4 -> 0x35 (a DIFFERENT class -> reset cleared)
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    keys = [d for _, d, _ in car.sent if d[:4] == bytes([0x10, 0x0A, 0x27, 0x02])]
    self.assertEqual(len(keys), 2)           # R1 and R4 ONLY (6/7 skipped after the lockout)
    doc = self.result_doc()
    self.assertEqual(doc["R1"]["nrc"], 0x36)
    self.assertIsNone(doc["R2"])
    self.assertIsNone(doc["R3"])
    self.assertIsNone(doc["S3"])
    self.assertIsNone(doc["S4"])
    self.assertEqual(doc["R4"]["nrc"], 0x35)
    self.assertTrue(doc["lockout_seen"])
    self.assertTrue(doc["cycle_reset"])
    # the DID sweep still ran even after the lockout
    self.assertEqual(set(doc["dids"]), {f"{d:04X}" for d in self.SWEEP})
    self.assertEqual([d[1] for d in self.tx_frames(car) if d[1] == 0x2E], [])

  def test_lockout_persists_through_cycle_reset(self):
    car = self.p6_car()
    car.key_nrc_sequence = [0x37]            # EVERY key attempt (R1..R4) -> 0x37
    self.run_car(car)
    doc = self.result_doc()
    self.assertEqual(doc["R1"]["nrc"], 0x37)
    self.assertEqual(doc["R4"]["nrc"], 0x37)
    self.assertTrue(doc["lockout_seen"])
    self.assertFalse(doc["cycle_reset"])     # R4 is STILL 0x37 -> the reset did not clear the lockout

  def test_positive_key_is_recorded_unlocked(self):
    car = self.p6_car()
    car.key_unlock = True                    # hypothetical: the zero key lands -> 67 02
    self.run_car(car)
    doc = self.result_doc()
    self.assertEqual(doc["R4"]["resp"], "6702")
    self.assertTrue(doc["unlocked"])
    self.assertEqual([d for d in self.tx_frames(car) if d[1] == 0x2E], [])   # still NO write

  # ---- session / programming-session / DTC behaviours ------------------------------------------------------------
  def test_session_refused_skips_key_part_but_still_sweeps(self):
    car = self.p6_car()
    car.session_refused = True               # 10 03 -> 7F 10 12: skip steps 5-7, still do the DID sweep + step 8
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["aborted"])
    doc = self.result_doc()
    self.assertIsNone(doc["R1"])             # no key in the (refused) extended session
    self.assertIsNone(doc["R2"])
    self.assertIsNone(doc["R3"])
    self.assertEqual(set(doc["dids"]), {f"{d:04X}" for d in self.SWEEP})
    # step 8 still ran (the reset test is ALWAYS): 10 01, 10 03, 27 01, 27 02
    self.assertIsNotNone(doc["R4"])
    self.assertIsNotNone(doc["S5"])
    self.assertEqual(doc["reset_session_default"]["resp"], "5001")

  def test_prog_session_positive_adds_s6_no_key(self):
    car = self.p6_car()
    self.run_car(car)
    doc = self.result_doc()
    self.assertEqual(doc["prog_session_1002"]["resp"], "500200")
    self.assertTrue(doc["prog_session_1002"]["positive"])
    self.assertIsNotNone(doc["S6"])
    # S6 is a 27 01 after the 10 02 frame, and it is NOT followed by a key in the programming session
    idx = self.step_names(doc).index("S6")
    self.assertEqual(self.step_names(doc)[idx + 1], "dtc_19_02_a5")
    keys = [d for _, d, _ in car.sent if d[:4] == bytes([0x10, 0x0A, 0x27, 0x02])]
    self.assertEqual(len(keys), 4)          # still only the four matrix keys

  def test_prog_session_negative_no_s6(self):
    car = self.p6_car()
    car.prog_session_positive = False        # 10 02 -> 7F 10 12
    self.run_car(car)
    doc = self.result_doc()
    self.assertEqual(doc["prog_session_1002"]["nrc"], 0x12)
    self.assertIsNone(doc["S6"])
    self.assertIn("prog_session_1002", self.step_names(doc))

  def test_dtc_19_02_a5_recorded(self):
    car = self.p6_car()
    self.run_car(car)
    doc = self.result_doc()
    self.assertEqual(doc["dtc_19_02_a5"]["nrc"], 0x31)        # default 7f1931
    self.assertEqual([d for _, d, _ in car.sent if d[1] == 0x19], [self.DTC])
    car2 = self.p6_car()
    car2.dtc_a5_resp = "055902ff00000000"                     # a positive DTC answer (59 02 ff 00 00)
    self.fresh_state()
    self.set_phase(6)
    self.run_car(car2, key="boot:dtc")
    self.assertTrue(self.result_doc()["dtc_19_02_a5"]["positive"])

  def test_read_gate_aborts_and_sends_nothing_else(self):
    car = self.p6_car()
    car.read_refused = True
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIn("0x0103", s["aborted"])
    self.assertEqual([d[1] for d in self.tx_frames(car)], [0x22])   # only the failed read
    self.assertEqual(car.mux_calls, [True, False])

  # ---- guards (the closed phase-6 allowlist) --------------------------------------------------------------------
  def test_guard_service_phase6_closed_set(self):
    E.guard_service(0x22, None, 0x0103, phase=6)
    for did in self.SWEEP:
      E.guard_service(0x22, None, did, phase=6)
    for sub in (0x01, 0x02, 0x03):
      E.guard_service(0x10, sub, None, phase=6)
    E.guard_service(0x27, 0x01, None, phase=6)
    E.guard_service(0x27, 0x02, None, phase=6)
    E.guard_service(0x19, 0x02, None, phase=6)
    # 2E (write) and 3E (tester present) are NOT in the phase-6 allowlist
    for svc in (0x2E, 0x3E, 0x29, 0x31, 0x23, 0x34, 0x35, 0x36, 0x37):
      with self.assertRaises(D.SafetyViolation):
        E.guard_service(svc, None, 0x0103 if svc == 0x2E else None, phase=6)
    with self.assertRaises(D.SafetyViolation):
      E.guard_service(0x10, 0x04, None, phase=6)             # 10 04 not admissible
    with self.assertRaises(D.SafetyViolation):
      E.guard_service(0x10, 0x83, None, phase=6)             # 10 83 not admissible
    with self.assertRaises(D.SafetyViolation):
      E.guard_service(0x22, None, 0xF188, phase=6)           # a DID outside the sweep set
    with self.assertRaises(D.SafetyViolation):
      E.guard_service(0x19, 0x01, None, phase=6)             # 19 01 not admissible (only 02)
    for sub in (0x00, 0x03, 0x04):
      with self.assertRaises(D.SafetyViolation):
        E.guard_service(0x27, sub, None, phase=6)

  def test_guard_frame_phase6_closed_shapes(self):
    for f in (self.READ, self.SESSION, self.SESSION01, self.SESSION02, self.SEED, self.DTC, *[self.did_frame(d) for d in self.SWEEP]):
      E.guard_frame(0x7D1, f, 1, phase=6)
    E.guard_frame(0x7D1, self.FC, 1, phase=6)
    E.guard_frame(0x7D1, self.KEYFF, 1, phase=6, p6_seed=True, key_attempts=0)
    E.guard_frame(0x7D1, self.KEYCF, 1, phase=6)
    # wrong bus / wrong addr / wrong length
    for addr, bus in ((0x7D1, 0), (0x7D2, 1), (0x770, 1)):
      with self.assertRaises(D.SafetyViolation):
        E.guard_frame(addr, self.READ, bus, phase=6)
    # 2E never, 19 02 with the wrong mask never, 22 with a non-sweep DID never
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([7, 0x2E, 0x01, 0x03]) + CURRENT, 1, phase=6, readback=CURRENT)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([3, 0x19, 0x02, 0x01]).ljust(8, b"\x00"), 1, phase=6)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.did_frame(0xF188), 1, phase=6)
    # a 27 02 with a NON-zero key is refused
    bad = bytes([0x10, 0x0A, 0x27, 0x02, 0, 0, 0, 1])
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bad, 1, phase=6, p6_seed=True, key_attempts=0)
    # a 27 02 with NO preceding 27 01 (sentinel absent) is refused
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.KEYFF, 1, phase=6, p6_seed=False, key_attempts=0)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.KEYFF, 1, phase=6, seed=E.PHASE6_KEY, key_attempts=0)
    # a 5th key attempt (counter already at the cap) is refused
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.KEYFF, 1, phase=6, p6_seed=True, key_attempts=E.PHASE6_MAX_KEY_ATTEMPTS)
    # the consecutive frame must carry the zero-key tail
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, (bytes([0x21]) + b"\x09\x09\x09\x09").ljust(8, b"\x00"), 1, phase=6)
    # single-frame 27 02 with a non-zero key refused too
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([6, 0x27, 0x02, 0, 0, 0, 1]).ljust(8, b"\x00"), 1, phase=6, p6_seed=True, key_attempts=0)

  def test_send_key_phase6_requires_zero_key_and_preceding_seed(self):
    car = self.p6_car()
    c = self.client6(car)
    with self.assertRaises(D.SafetyViolation):
      c.send_key_phase6(b"\x01" * 8)                         # non-zero key
    with self.assertRaises(D.SafetyViolation):
      c.send_key_phase6(E.PHASE6_KEY)                        # no preceding 27 01 (sentinel False)
    self.assertEqual(car.sent, [])
    # after a 27 01 it is admissible
    c.request_seed()
    c.send_key_phase6(E.PHASE6_KEY)
    self.assertEqual(c.key_attempts, 1)

  def test_fifth_key_attempt_is_refused(self):
    car = self.p6_car()
    c = self.client6(car)
    for _ in range(E.PHASE6_MAX_KEY_ATTEMPTS):
      c.request_seed()
      c.send_key_phase6(E.PHASE6_KEY)
    self.assertEqual(c.key_attempts, E.PHASE6_MAX_KEY_ATTEMPTS)
    n = len([d for _, d, _ in car.sent if d[:4] == bytes([0x10, 0x0A, 0x27, 0x02])])
    c.request_seed()
    with self.assertRaises(D.SafetyViolation):
      c.send_key_phase6(E.PHASE6_KEY)                        # the 5th key can never reach the bus
    self.assertEqual(len([d for _, d, _ in car.sent if d[:4] == bytes([0x10, 0x0A, 0x27, 0x02])]), n)

  def test_phase6_shapes_refused_outside_phase6(self):
    # the phase-6-only shapes must NOT be admissible in phases 1-5
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.SESSION02, 1, phase=5)       # 10 02 programming session
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.DTC, 1, phase=4)             # 19 02 A5
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.DTC, 1, phase=5)             # 19 02 A5 not a phase-5 service
    with self.assertRaises(D.SafetyViolation):
      E.guard_service(0x19, 0x02, None, phase=5)             # 0x19 not a phase-5 service
    with self.assertRaises(D.SafetyViolation):
      E.guard_service(0x10, 0x02, None, phase=1)             # 10 02 not a phase-1 sub
    # phase 5 still refuses the zero-key 27 02
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.KEYFF, 1, phase=5, p6_seed=True, key_attempts=0)

  def test_phase6_never_writes_and_budget(self):
    self.assertEqual(E.RUN_BUDGET_S_PHASE6, 60.0)
    self.assertEqual(E.RUN_BUDGET_S, 15.0)
    car = self.p6_car()
    self.run_car(car)
    self.assertEqual([d for d in self.tx_frames(car) if d[1] == 0x2E], [])
    # no bare 29/31 probes either
    self.assertEqual([d for d in self.tx_frames(car) if d[1] in (0x29, 0x31)], [])
    self.assertEqual([d for d in self.tx_frames(car) if d[1] == 0x3E], [])

  def test_once_per_ignition(self):
    car = self.p6_car()
    self.run_car(car, key="boot:1")
    n = len(car.sent)
    s = self.run_car(car, key="boot:1")
    self.assertFalse(s["ran"])
    self.assertEqual(s["skip"], "already done (ignition)")
    self.assertEqual(len(car.sent), n)

  def test_not_in_park_sends_nothing(self):
    car = self.p6_car(gear=5)
    s = self.run_car(car)
    self.assertFalse(s["ran"])
    self.assertEqual(car.sent, [])
    self.assertEqual(car.mux_calls, [])


class TestPhase7(Base):
  """Phase 7 (state ``{"probe_enabled": true, "phase": 7, "p7_candidates": [...]}``): the ASK-FAMILY probe -- ``27 11``
  requestSeed -> at most 4 ``27 12`` sendKey (one per FRESH seed, 8-byte key ONLY, only right after a POSITIVE 27 11) ->
  on a positive ``67 12`` the ONE no-op ``2E 0103`` + re-read. No ``27 01``/``27 02``. Every guard is mechanically
  enforced."""

  READ = bytes([3, 0x22, 0x01, 0x03]).ljust(8, b"\x00")
  SESSION = bytes([2, 0x10, 0x03]).ljust(8, b"\x00")
  SESSION01 = bytes([2, 0x10, 0x01]).ljust(8, b"\x00")
  SEED11 = bytes([2, 0x27, 0x11]).ljust(8, b"\x00")
  A29 = bytes([2, 0x29, 0x01]).ljust(8, b"\x00")
  F100 = bytes([3, 0x22, 0xF1, 0x00]).ljust(8, b"\x00")
  WRITE = bytes([7, 0x2E, 0x01, 0x03]) + CURRENT
  FC = D.FLOW_CONTROL_FRAME
  CAND4 = ["zero8", "identity8", "algo8w_27100", "algo8_27100"]

  def setUp(self):
    super().setUp()
    self.set_phase(7)

  def set_phase(self, phase, candidates=None):
    state = {"probe_enabled": True, "phase": phase}
    if candidates is not None:
      state["p7_candidates"] = candidates
    with open(os.path.join(self.out, "state.json"), "w") as f:
      json.dump(state, f)

  def p7_car(self, **kw):
    car = FakeCar(**kw)
    car.ask_family = True          # answer the ASK family (27 11 -> 67 11 + 8-byte seed; 27 12 -> 7F 27 35)
    car.esc_seed8_11 = PHASE4_SEED
    return car

  @staticmethod
  def ff(key):
    return bytes([0x10, 0x0A, 0x27, 0x12]) + bytes(key)[:4]

  @staticmethod
  def cf(key):
    return (bytes([0x21]) + bytes(key)[4:8]).ljust(8, b"\x00")

  def step_names(self, doc):
    return [st["name"] for st in doc["steps"]]

  # ---- keys the on-car seed resolves to (hard-coded AND cross-checked against resolve_phase7) -----------------------
  K_ZERO8 = bytes(8)
  K_IDENTITY8 = PHASE4_SEED                     # 5AB05AB05AB05AB0
  K_ALGO8W = bytes.fromhex("f1c6f1c6f1c6f1c6")  # cal_27100(seed[:4])[:2] x4 = [f1,c6]x4
  K_ALGO8 = bytes.fromhex("f1c60000f1c60000")   # cal_27100(seed[:4]) x2

  def test_resolve_phase7_modes(self):
    seed = PHASE4_SEED
    self.assertEqual(E.resolve_phase7("zero8", seed), self.K_ZERO8)
    self.assertEqual(E.resolve_phase7("identity8", seed), self.K_IDENTITY8)
    self.assertEqual(E.resolve_phase7("algo8w_27100", seed), self.K_ALGO8W)
    self.assertEqual(E.resolve_phase7("algo8_27100", seed), self.K_ALGO8)
    self.assertEqual(E.resolve_phase7("algo8_26300", seed), SK.cal_26300(seed[:4]) * 2)
    self.assertEqual(E.resolve_phase7("algo8_26700", seed), SK.cal_26700(seed[:4]) * 2)
    # a zero seed byte makes 27100/26300 bail -> None (no frame)
    z = bytes.fromhex("00b05ab05ab05ab0")
    self.assertIsNone(E.resolve_phase7("algo8w_27100", z))
    self.assertIsNone(E.resolve_phase7("algo8_26300", z))
    # unknown candidate aborts
    with self.assertRaises(D.Abort):
      E.resolve_phase7("banana", seed)

  # ---- happy path A: the DEFAULT single candidate (zero8), refused -> no write -------------------------------------
  def test_default_single_candidate_exact_tx_order(self):
    car = self.p7_car()
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["aborted"])
    self.assertIsNone(s["error"])
    self.assertEqual(self.tx_frames(car), [self.READ, self.SESSION,
                                           self.SEED11, self.FC, self.ff(self.K_ZERO8), self.cf(self.K_ZERO8),
                                           self.A29, self.F100, self.F100,
                                           self.SESSION01, self.READ])
    # no 27 01/27 02, no extra 2E anywhere
    self.assertEqual([d for d in self.tx_frames(car) if d[1] == 0x27 and d[2] == 0x02], [])
    self.assertEqual([d for d in self.tx_frames(car) if d[1] == 0x27 and d[2] == 0x01], [])
    self.assertEqual([d for d in self.tx_frames(car) if d[1] == 0x2E], [])
    addrs = [a for a, _, _ in car.sent]
    self.assertEqual(addrs.count(0x770), 1)
    self.assertEqual(addrs.count(0x7A0), 1)
    self.assertEqual([d for a, d, _ in car.sent if a == 0x770], [self.F100])
    self.assertEqual([d for a, d, _ in car.sent if a == 0x7A0], [self.F100])
    self.assertEqual(car.mux_calls, [True, False])
    self.assertTrue(s["mux_restored"])
    doc = self.result_doc()
    self.assertEqual(doc["phase"], 7)
    self.assertFalse(doc["unlocked_ask"])
    self.assertFalse(doc["lockout_ask"])
    self.assertFalse(doc["write_attempted"])
    self.assertIsNone(doc["write_ask"])
    self.assertEqual(doc["value_start"], "90060350")
    self.assertEqual(doc["value_end"], "90060350")
    self.assertFalse(doc["value_changed"])
    self.assertEqual([c["candidate"] for c in doc["cands_ask"]], ["zero8"])
    self.assertEqual(doc["cands_ask"][0]["key"], self.K_ZERO8.hex())
    self.assertEqual(doc["cands_ask"][0]["nrc"], 0x35)
    self.assertEqual(doc["seeds_ask"][0]["seed"], PHASE4_SEED.hex())
    self.assertEqual(self.events[-1][0], "esc_probe_0027")
    self.assertLessEqual(s["duration_s"], E.RUN_BUDGET_S_PHASE7)

  def test_default_step_names_and_order(self):
    car = self.p7_car()
    self.run_car(car)
    self.assertEqual(self.step_names(self.result_doc()),
                     ["read_esc", "session", "seed_zero8", "key_zero8", "a29_01", "f100_770", "f100_7A0",
                      "leave_session", "read_esc_end"])

  def test_summary_fields(self):
    car = self.p7_car()
    s = self.run_car(car)
    self.assertEqual(s["phase"], 7)
    self.assertFalse(s["unlocked_ask"])
    self.assertFalse(s["lockout_ask"])
    self.assertEqual(s["a29_01_nrc"], 0x11)          # fake answers 29 01 -> 7F 29 11
    self.assertEqual(s["cands_ask"][0]["key"], self.K_ZERO8.hex())
    self.assertEqual(s["seeds_ask"][0]["candidate"], "zero8")
    self.assertIsNone(s["f100_770"])                  # extra addrs stay silent by default
    self.assertIsNone(s["f100_7a0"])

  # ---- happy path B: FOUR candidates, the LAST one unlocks -> no-op write + re-read ---------------------------------
  def test_four_candidates_last_unlocks_then_noop_write(self):
    self.fresh_state()
    self.set_phase(7, candidates=self.CAND4)
    car = self.p7_car()
    car.key12_unlock_index = 3        # zero8/identity8/algo8w all 7F 27 35; algo8_27100 -> 67 12
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["error"])
    exp_keys = [self.K_ZERO8, self.K_IDENTITY8, self.K_ALGO8W, self.K_ALGO8]
    exp = [self.READ, self.SESSION]
    for k in exp_keys:
      exp += [self.SEED11, self.FC, self.ff(k), self.cf(k)]
    exp += [self.WRITE, self.READ, self.A29, self.F100, self.F100, self.SESSION01, self.READ]
    self.assertEqual(self.tx_frames(car), exp)
    doc = self.result_doc()
    self.assertTrue(doc["unlocked_ask"])
    self.assertTrue(doc["write_attempted"])
    self.assertTrue(doc["write_ask"]["positive"])
    self.assertEqual(doc["write_ask"]["req"][:14], "072e0103900603")   # 2E 0103 + value_start (no-op)
    self.assertEqual(doc["value_after_ask"], "90060350")
    self.assertEqual([c["candidate"] for c in doc["cands_ask"]], self.CAND4)
    self.assertEqual([c["key"] for c in doc["cands_ask"]],
                     [self.K_ZERO8.hex(), self.K_IDENTITY8.hex(), self.K_ALGO8W.hex(), self.K_ALGO8.hex()])
    self.assertEqual([c["positive"] for c in doc["cands_ask"]], [False, False, False, True])
    keys = [d for _, d, _ in car.sent if d[:4] == bytes([0x10, 0x0A, 0x27, 0x12])]
    self.assertEqual(len(keys), 4)
    # exactly ONE no-op write, and its payload equals value_start
    writes = [d for d in self.tx_frames(car) if d[1] == 0x2E]
    self.assertEqual(writes, [self.WRITE])
    names = self.step_names(doc)
    self.assertEqual(names[-3:], ["f100_7A0", "leave_session", "read_esc_end"])
    self.assertIn("write_ask", names)
    self.assertEqual(names.index("write_ask") + 1, names.index("read_after_ask"))

  def test_four_candidates_first_unlocks_stops_early(self):
    self.fresh_state()
    self.set_phase(7, candidates=self.CAND4)
    car = self.p7_car()
    car.key12_unlock = True           # the FIRST (zero8) unlocks -> loop stops, only ONE key
    self.run_car(car)
    doc = self.result_doc()
    self.assertTrue(doc["unlocked_ask"])
    self.assertEqual([c["candidate"] for c in doc["cands_ask"]], ["zero8"])
    keys = [d for _, d, _ in car.sent if d[:4] == bytes([0x10, 0x0A, 0x27, 0x12])]
    self.assertEqual(len(keys), 1)
    self.assertEqual([d for d in self.tx_frames(car) if d[1] == 0x2E], [self.WRITE])

  def test_candidates_capped_at_four(self):
    self.fresh_state()
    self.set_phase(7, candidates=["zero8", "identity8", "algo8w_27100", "algo8_27100", "algo8_26700", "algo8_26300"])
    car = self.p7_car()
    self.run_car(car)
    keys = [d for _, d, _ in car.sent if d[:4] == bytes([0x10, 0x0A, 0x27, 0x12])]
    self.assertEqual(len(keys), 4)                     # the 5th/6th entries are dropped before any frame
    self.assertEqual(len(self.result_doc()["cands_ask"]), 4)

  def test_unknown_candidate_names_are_dropped(self):
    self.fresh_state()
    self.set_phase(7, candidates=["banana", "zero8"])
    car = self.p7_car()
    self.run_car(car)
    doc = self.result_doc()
    self.assertEqual([c["candidate"] for c in doc["cands_ask"]], ["zero8"])

  def test_all_invalid_candidates_skips(self):
    self.fresh_state()
    self.set_phase(7, candidates=["banana", "nope"])
    car = self.p7_car()
    s = self.run_car(car)
    self.assertFalse(s["ran"])
    self.assertIn("p7_candidates", s["skip"])
    self.assertEqual(car.sent, [])

  # ---- the lockout path: a 27 12 -> 0x36 with more candidates left stops the loop (no write) -----------------------
  def test_lockout_on_36_stops_loop_no_write(self):
    self.fresh_state()
    self.set_phase(7, candidates=self.CAND4)
    car = self.p7_car()
    car.key12_nrc = 0x36              # the FIRST 27 12 -> 7F 27 36
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    doc = self.result_doc()
    self.assertTrue(doc["lockout_ask"])
    self.assertFalse(doc["unlocked_ask"])
    self.assertFalse(doc["write_attempted"])
    self.assertEqual([c["candidate"] for c in doc["cands_ask"]], ["zero8"])   # stopped after the first
    self.assertEqual([d for d in self.tx_frames(car) if d[1] == 0x2E], [])
    # the trailing steps still run
    self.assertIn("a29_01", self.step_names(doc))

  def test_lockout_37_stops_loop_no_write(self):
    self.fresh_state()
    self.set_phase(7, candidates=self.CAND4)
    car = self.p7_car()
    car.key12_nrc = 0x37
    self.run_car(car)
    doc = self.result_doc()
    self.assertTrue(doc["lockout_ask"])
    self.assertEqual([c["candidate"] for c in doc["cands_ask"]], ["zero8"])

  # ---- session / seed gate behaviours ------------------------------------------------------------------------------
  def test_session_not_positive_skips_candidate_loop(self):
    car = self.p7_car()
    car.session_refused = True         # 10 03 -> 7F 10 12: no 27 11 at all, but 29 01 + F100 + leave still run
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["aborted"])
    doc = self.result_doc()
    self.assertEqual(doc["cands_ask"], [])
    self.assertEqual(doc["seeds_ask"], [])
    self.assertFalse(doc["unlocked_ask"])
    frames = self.tx_frames(car)
    self.assertEqual([d for d in frames if d[1] == 0x27], [])
    self.assertIn(self.A29, frames)
    self.assertEqual([a for a, _, _ in car.sent].count(0x770), 1)

  def test_negative_seed11_stops_candidate_loop(self):
    self.fresh_state()
    self.set_phase(7, candidates=self.CAND4)
    car = self.p7_car()
    car.seed11_refused = True          # 27 11 -> 7F 27 35: stop before any 27 12
    self.run_car(car)
    doc = self.result_doc()
    self.assertEqual([c["candidate"] for c in doc["cands_ask"]], ["zero8"])
    self.assertFalse(doc["cands_ask"][0]["positive"])
    self.assertIsNone(doc["cands_ask"][0]["key"])
    self.assertEqual([d for _, d, _ in car.sent if d[:4] == bytes([0x10, 0x0A, 0x27, 0x12])], [])

  def test_silent_seed11_stops_candidate_loop(self):
    self.fresh_state()
    self.set_phase(7, candidates=self.CAND4)
    car = self.p7_car()
    car.seed11_silent = True
    self.run_car(car)
    doc = self.result_doc()
    self.assertEqual([c["candidate"] for c in doc["cands_ask"]], ["zero8"])
    self.assertEqual([d for _, d, _ in car.sent if d[:4] == bytes([0x10, 0x0A, 0x27, 0x12])], [])

  def test_read_gate_aborts(self):
    car = self.p7_car()
    car.read_refused = True
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIn("0x0103", s["aborted"])
    self.assertEqual([d[1] for d in self.tx_frames(car)], [0x22])   # only the failed read
    self.assertEqual(car.mux_calls, [True, False])

  def test_27_11_returns_a_fresh_mf_seed_per_candidate(self):
    self.fresh_state()
    self.set_phase(7, candidates=self.CAND4)
    car = self.p7_car()
    car.seed11_sequence = [bytes.fromhex("5ab05ab05ab05ab0"), bytes.fromhex("1122334455667788"),
                           bytes.fromhex("aabbccddaabbccdd"), bytes.fromhex("0102030405060708")]
    self.run_car(car)
    doc = self.result_doc()
    self.assertEqual([s["seed"] for s in doc["seeds_ask"]],
                     [s.hex() for s in car.seed11_sequence])
    self.assertEqual([c["key"] for c in doc["cands_ask"]],
                     [bytes(8).hex(), car.seed11_sequence[1].hex(),
                      (SK.cal_27100(car.seed11_sequence[2][:4])[:2] * 4).hex(),
                      (SK.cal_27100(car.seed11_sequence[3][:4]) * 2).hex()])

  # ---- summary / extra-addr / once-per-ignition --------------------------------------------------------------------
  def test_extra_addr_f100_answers_recorded(self):
    car = self.p7_car()
    car.extra_f100 = {0x770: True}     # 0x770 answers 62 F100 01020304; 0x7A0 stays silent
    self.run_car(car)
    doc = self.result_doc()
    self.assertEqual(doc["f100_770"]["resp"], "62f10001020304")
    self.assertTrue(doc["f100_770"]["positive"])
    self.assertTrue(doc["f100_7A0"]["no_response"])

  def test_once_per_ignition(self):
    car = self.p7_car()
    self.run_car(car, key="boot:1")
    n = len(car.sent)
    s = self.run_car(car, key="boot:1")
    self.assertFalse(s["ran"])
    self.assertEqual(s["skip"], "already done (ignition)")
    self.assertEqual(len(car.sent), n)

  def test_not_in_park_sends_nothing(self):
    car = self.p7_car(gear=5)
    s = self.run_car(car)
    self.assertFalse(s["ran"])
    self.assertEqual(car.sent, [])
    self.assertEqual(car.mux_calls, [])

  def test_phase7_budget_is_sixty_seconds(self):
    self.assertEqual(E.RUN_BUDGET_S_PHASE7, 60.0)
    self.assertEqual(E.RUN_BUDGET_S_PHASE6, 60.0)
    self.assertEqual(E.RUN_BUDGET_S, 15.0)

  # ---- guards: the closed phase-7 allowlist ------------------------------------------------------------------------
  def test_guard_service_phase7_closed_set(self):
    E.guard_service(0x22, None, 0x0103, phase=7)
    E.guard_service(0x22, None, 0xF100, phase=7)
    E.guard_service(0x10, 0x01, None, phase=7)
    E.guard_service(0x10, 0x03, None, phase=7)
    E.guard_service(0x27, 0x11, None, phase=7)
    E.guard_service(0x27, 0x12, None, phase=7)
    E.guard_service(0x29, 0x01, None, phase=7)
    E.guard_service(0x2E, None, 0x0103, phase=7)
    # 27 01/27 02 are NOT phase-7 sub-functions
    for sub in (0x01, 0x02, 0x10, 0x13, 0x00):
      with self.assertRaises(D.SafetyViolation):
        E.guard_service(0x27, sub, None, phase=7)
    # 10 only 01/03
    for sub in (0x02, 0x04, 0x83, 0x81):
      with self.assertRaises(D.SafetyViolation):
        E.guard_service(0x10, sub, None, phase=7)
    # 22 only 0103/F100
    for did in (0x0104, 0xF101, 0x0000):
      with self.assertRaises(D.SafetyViolation):
        E.guard_service(0x22, None, did, phase=7)
    # 2E only DID 0103; 29 only sub 01
    with self.assertRaises(D.SafetyViolation):
      E.guard_service(0x2E, None, 0x0104, phase=7)
    with self.assertRaises(D.SafetyViolation):
      E.guard_service(0x29, 0x02, None, phase=7)
    # everything else is out
    for svc in (0x19, 0x31, 0x3E, 0x23, 0x34, 0x35, 0x36, 0x37):
      with self.assertRaises(D.SafetyViolation):
        E.guard_service(svc, None, None, phase=7)

  def test_guard_frame_phase7_closed_shapes(self):
    for f in (self.READ, self.SESSION, self.SESSION01, self.SEED11, self.A29, self.F100):
      E.guard_frame(0x7D1, f, 1, phase=7)
    E.guard_frame(0x7D1, self.FC, 1, phase=7)                              # RX flow control
    # the 27 12 multi-frame is admissible only with the resolved 8-byte key + the sentinel
    E.guard_frame(0x7D1, self.ff(self.K_ZERO8), 1, phase=7, key=self.K_ZERO8, p7_seed=True, key_attempts=0)
    E.guard_frame(0x7D1, self.cf(self.K_ZERO8), 1, phase=7, key=self.K_ZERO8)
    # the ONE no-op 2E with the exact readback
    E.guard_frame(0x7D1, self.WRITE, 1, phase=7, readback=CURRENT)
    # extra addrs take ONLY 22 F100
    E.guard_frame(0x770, self.F100, 1, phase=7)
    E.guard_frame(0x7A0, self.F100, 1, phase=7)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x770, self.READ, 1, phase=7)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x770, self.SESSION, 1, phase=7)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x771, self.F100, 1, phase=7)
    # 27 12 without the sentinel, and with a wrong-length / wrong / missing key
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.ff(self.K_ZERO8), 1, phase=7, key=self.K_ZERO8, p7_seed=False, key_attempts=0)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.ff(self.K_ZERO8), 1, phase=7, key=None, p7_seed=True, key_attempts=0)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.ff(self.K_ZERO8), 1, phase=7, key=self.K_IDENTITY8, p7_seed=True, key_attempts=0)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.ff(self.K_ZERO8), 1, phase=7, key=self.K_ZERO8[:4], p7_seed=True, key_attempts=0)
    # a 5th attempt (counter at the cap) is refused
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.ff(self.K_ZERO8), 1, phase=7, key=self.K_ZERO8, p7_seed=True,
                    key_attempts=E.PHASE7_MAX_CANDIDATES)
    # the consecutive frame must carry the key tail
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.cf(self.K_IDENTITY8), 1, phase=7, key=self.K_ZERO8)
    # a 27 12 single frame is NEVER admissible in phase 7 (8-byte ISO-TP only)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([6, 0x27, 0x12, 1, 2, 3, 4]).ljust(8, b"\x00"), 1, phase=7, p7_seed=True)
    # 27 01/27 02, wrong DIDs, wrong 2E payload / missing readback
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([2, 0x27, 0x01]).ljust(8, b"\x00"), 1, phase=7)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([2, 0x27, 0x02]).ljust(8, b"\x00"), 1, phase=7)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.ff(b"\x00" * 4)[:4] + bytes([0x02]) , 1, phase=7)  # 10 0A 27 02 (not 27 12)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([3, 0x22, 0x01, 0x04]).ljust(8, b"\x00"), 1, phase=7)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([7, 0x2E, 0x01, 0x03]) + bytes(4), 1, phase=7, readback=CURRENT)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.WRITE, 1, phase=7)                           # no readback -> refused
    # wrong bus / length
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.READ, 0, phase=7)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.SEED11[:3], 1, phase=7)

  def test_phase7_shapes_refused_outside_phase7(self):
    # the phase-7-only shapes must NOT be admissible in phases 1-6
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x770, self.F100, 1, phase=5)          # phase-5 extra addrs take 10 03 only
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.SEED11, 1, phase=4)        # 27 11 not a phase-4 sub
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.SEED11, 1, phase=6)        # 27 11 not a phase-6 sub
    with self.assertRaises(D.SafetyViolation):
      E.guard_service(0x27, 0x11, None, phase=4)
    with self.assertRaises(D.SafetyViolation):
      E.guard_service(0x27, 0x12, None, phase=6)
    with self.assertRaises(D.SafetyViolation):
      E.guard_service(0x27, 0x11, None, phase=6)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.ff(self.K_ZERO8), 1, phase=6, key=self.K_ZERO8, p7_seed=True)
    # phase 5 still admits 27 11 (it is one of its read-only sub-probes) -- NOT a phase-7 leak
    E.guard_service(0x27, 0x11, None, phase=5)
    E.guard_frame(0x7D1, self.SEED11, 1, phase=5)

  def test_single_tx_site_unchanged(self):
    tree = ast.parse(inspect.getsource(E))
    calls = [getattr(n.func, "attr", getattr(n.func, "id", "")) for n in ast.walk(tree)
             if isinstance(n, ast.Call) and getattr(n.func, "attr", getattr(n.func, "id", "")) == "_can_send"]
    self.assertEqual(calls, ["_can_send", "_can_send"])

  def test_write_did_is_the_only_write_and_is_a_noop(self):
    # the phase-7 write path reuses write_did: value must equal the step-1 readback
    car = self.p7_car()
    c = E.EscProbeClient(car.can_send, car.can_recv, D.VehicleGate(car.now), car.now, car.now() + 99, phase=7)
    with self.assertRaises(D.SafetyViolation):
      c.write_did(E.DID_VARIANT_CODING, b"\x00\x00\x00\x00", CURRENT)      # value != readback


class TestPhase8(Base):
  """Phase 8 (state ``{"probe_enabled": true, "phase": 8, "p8_candidates": [...]}``): the DOOR-B COUNTER ECONOMICS
  battery -- the free key (A1), a session-cycle test (A2), a time-reset test (25 s, or 30 s after a lockout at the very
  first key), and -- only if the cycle clears the counter -- a <=3-candidate walk; the first candidate is the new
  vendor-shape literal ``lit270100`` = ``00 32 37 30 30`` -> no, ``0032373031303000``. Only a positive ``67 12``
  unlocks the ONE no-op ``2E 0103``. No ``27 01``/``27 02``, no ``31``, no ``34``-``37``. Every guard is mechanically
  enforced."""

  READ = bytes([3, 0x22, 0x01, 0x03]).ljust(8, b"\x00")
  SESSION = bytes([2, 0x10, 0x03]).ljust(8, b"\x00")
  SESSION01 = bytes([2, 0x10, 0x01]).ljust(8, b"\x00")
  SEED11 = bytes([2, 0x27, 0x11]).ljust(8, b"\x00")
  A2905 = bytes([2, 0x29, 0x05]).ljust(8, b"\x00")
  WRITE = bytes([7, 0x2E, 0x01, 0x03]) + CURRENT
  FC = D.FLOW_CONTROL_FRAME
  CAND3 = ["lit270100", "algo8w_27100", "algo8_27100"]

  K_LIT = bytes.fromhex("0032373031303000")     # lit270100 = 00 '2' '7' '0' '1' '0' '0' 00
  K_ALGO8W = bytes.fromhex("f1c6f1c6f1c6f1c6")   # cal_27100(seed[:4])[:2] x4
  K_ALGO8 = bytes.fromhex("f1c60000f1c60000")    # cal_27100(seed[:4]) x2

  def setUp(self):
    super().setUp()
    self.set_phase(8)

  def set_phase(self, phase, candidates=None):
    state = {"probe_enabled": True, "phase": phase}
    if candidates is not None:
      state["p8_candidates"] = candidates
    with open(os.path.join(self.out, "state.json"), "w") as f:
      json.dump(state, f)

  def p8_car(self, **kw):
    car = FakeCar(**kw)
    car.ask_family = True             # answer the ASK family (27 11 -> 67 11 + 8-byte seed; 27 12 per the action script)
    car.esc_seed8_11 = PHASE4_SEED
    return car

  @staticmethod
  def ff(key):
    return bytes([0x10, 0x0A, 0x27, 0x12]) + bytes(key)[:4]

  @staticmethod
  def cf(key):
    return (bytes([0x21]) + bytes(key)[4:8]).ljust(8, b"\x00")

  def step_names(self, doc):
    return [st["name"] for st in doc["steps"]]

  def key_times(self, car):
    """The send-time of every 27 12 first frame -- for the wait-path timing asserts."""
    return [car.sent_t[i] for i, (_, d, _) in enumerate(car.sent) if d[:4] == bytes([0x10, 0x0A, 0x27, 0x12])]

  def seed_frames(self, car):
    return [d for _, d, _ in car.sent if d[:3] == bytes([2, 0x27, 0x11])]

  def key_frames(self, car):
    return [d for _, d, _ in car.sent if d[:4] == bytes([0x10, 0x0A, 0x27, 0x12])]

  # ---- resolver: the vendor literal + every shared phase-7 token ----------------------------------------------------
  def test_resolve_phase8_lit_and_tokens(self):
    seed = PHASE4_SEED
    self.assertEqual(E.resolve_phase8("lit270100", seed), self.K_LIT)
    self.assertEqual(self.K_LIT.hex(), "0032373031303000")
    self.assertEqual(E.resolve_phase8("lit270100", b"\x00" * 8), self.K_LIT)   # seed-independent literal
    self.assertEqual(E.resolve_phase8("zero8", seed), bytes(8))
    self.assertEqual(E.resolve_phase8("identity8", seed), PHASE4_SEED)
    self.assertEqual(E.resolve_phase8("algo8w_27100", seed), self.K_ALGO8W)
    self.assertEqual(E.resolve_phase8("algo8_27100", seed), self.K_ALGO8)
    self.assertEqual(E.resolve_phase8("algo8_26700", seed), SK.cal_26700(seed[:4]) * 2)
    self.assertEqual(E.resolve_phase8("algo8_26300", seed), SK.cal_26300(seed[:4]) * 2)
    # a zero seed byte makes 27100/26300 bail -> None (no frame)
    z = bytes.fromhex("00b05ab05ab05ab0")
    self.assertIsNone(E.resolve_phase8("algo8w_27100", z))
    with self.assertRaises(D.Abort):
      E.resolve_phase8("banana", seed)
    # the vocabulary is exactly the phase-7 tokens + lit270100
    self.assertEqual(set(E.PHASE8_CANDIDATES), set(E.P7_CANDIDATES) | {"lit270100"})
    self.assertEqual(E.PHASE8_DEFAULT_CANDIDATES, ("lit270100", "algo8w_27100", "algo8_27100"))

  # ---- happy path: A1/A2 refused with 0x35 -> cycle clears -> walk (last candidate unlocks) -> no-op write ----------
  def test_cycle_clears_walk_then_unlock(self):
    self.fresh_state()
    self.set_phase(8, candidates=self.CAND3)
    car = self.p8_car()
    car.key12_actions = [{"nrc": 0x35}, {"nrc": 0x35}, {"nrc": 0x35}, {"unlock": True}]
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["error"])
    self.assertIsNone(s["aborted"])
    exp = [self.READ, self.SESSION,
           self.SEED11, self.FC, self.ff(self.K_LIT), self.cf(self.K_LIT),          # A1 (cand0)
           self.SESSION01, self.SESSION,
           self.SEED11, self.FC, self.ff(self.K_LIT), self.cf(self.K_LIT),          # A2 (cycle, cand0)
           self.SESSION01, self.SESSION,
           self.SEED11, self.FC, self.ff(self.K_ALGO8W), self.cf(self.K_ALGO8W),    # walk1
           self.SESSION01, self.SESSION,
           self.SEED11, self.FC, self.ff(self.K_ALGO8), self.cf(self.K_ALGO8),      # walk2 -> 67 12
           self.WRITE, self.READ,                                                    # WIN PATH: no-op 2E + re-read
           self.A2905, self.SESSION01, self.READ]
    self.assertEqual(self.tx_frames(car), exp)
    doc = self.result_doc()
    self.assertTrue(doc["cycle_clears"])
    self.assertTrue(doc["unlocked8"])
    self.assertIsNone(doc["lockout_at_start"])
    self.assertIsNone(doc["time_reset"])
    self.assertIsNone(doc["time_reset_after_lockout"])
    self.assertEqual([c["step_label"] for c in doc["cands8"]], ["a1", "a2", "walk1", "walk2"])
    self.assertEqual([c["candidate"] for c in doc["cands8"]],
                     ["lit270100", "lit270100", "algo8w_27100", "algo8_27100"])
    self.assertEqual([c["positive"] for c in doc["cands8"]], [False, False, False, True])
    self.assertEqual([c["key"] for c in doc["cands8"]],
                     [self.K_LIT.hex(), self.K_LIT.hex(), self.K_ALGO8W.hex(), self.K_ALGO8.hex()])
    # exactly ONE no-op write, payload == value_start
    self.assertEqual([d for d in self.tx_frames(car) if d[1] == 0x2E], [self.WRITE])
    self.assertTrue(doc["write8"]["positive"])
    self.assertEqual(doc["value_after_write8"], "90060350")
    self.assertEqual(doc["value_start"], "90060350")
    # no 27 01/27 02, no 31/34-37 as a SERVICE anywhere (single frames only -- a `21 ..` consecutive frame's byte 1
    # is key data, not a service)
    single = [d for d in self.tx_frames(car) if d[0] >> 4 == 0]
    for svc in (0x31, 0x34, 0x35, 0x36, 0x37):
      self.assertEqual([d for d in single if d[1] == svc], [])
    self.assertEqual([d for d in single if d[1] == 0x27 and d[2] in (0x01, 0x02)], [])
    # counters: 4 keys, 4 seeds, all <= the caps
    self.assertEqual(len(self.key_frames(car)), 4)
    self.assertEqual(len(self.seed_frames(car)), 4)
    self.assertLessEqual(len(self.key_frames(car)), E.PHASE8_MAX_KEY_ATTEMPTS)
    self.assertLessEqual(len(self.seed_frames(car)), E.PHASE8_MAX_SEEDS)
    self.assertEqual(self.step_names(doc)[-3:], ["a29_05", "leave_session", "read_esc_end"])
    self.assertEqual(self.events[-1][0], "esc_probe_0027")
    self.assertLessEqual(s["duration_s"], E.RUN_BUDGET_S_PHASE8)

  def test_cycle_clears_step_names_and_order(self):
    self.fresh_state()
    self.set_phase(8, candidates=self.CAND3)
    car = self.p8_car()
    car.key12_actions = [{"nrc": 0x35}, {"nrc": 0x35}, {"nrc": 0x35}, {"nrc": 0x35}]
    self.run_car(car)
    self.assertEqual(self.step_names(self.result_doc()),
                     ["read_esc", "session", "seed_S0", "key_a1", "seed_S1", "key_a2",
                      "seed_Sx1", "key_walk1", "seed_Sx2", "key_walk2",
                      "a29_05", "leave_session", "read_esc_end"])

  # ---- persists: A2 -> 0x36 (cycle does NOT clear) -> 25 s wait -> A2b -> time_reset -------------------------------
  def test_cycle_persists_then_time_reset(self):
    self.fresh_state()
    self.set_phase(8, candidates=self.CAND3)
    car = self.p8_car()
    car.key12_actions = [{"nrc": 0x35}, {"nrc": 0x36}, {"nrc": 0x35}]
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["error"])
    exp = [self.READ, self.SESSION,
           self.SEED11, self.FC, self.ff(self.K_LIT), self.cf(self.K_LIT),          # A1 -> 0x35
           self.SESSION01, self.SESSION,
           self.SEED11, self.FC, self.ff(self.K_LIT), self.cf(self.K_LIT),          # A2 -> 0x36 (cycle did NOT clear)
           self.SESSION01, self.SESSION,                                             # the 25 s wait then re-cycle
           self.SEED11, self.FC, self.ff(self.K_LIT), self.cf(self.K_LIT),          # A2b -> 0x35
           self.A2905, self.SESSION01, self.READ]
    self.assertEqual(self.tx_frames(car), exp)
    doc = self.result_doc()
    self.assertFalse(doc["cycle_clears"])
    self.assertTrue(doc["time_reset"])
    self.assertIsNone(doc["lockout_at_start"])
    self.assertIsNone(doc["time_reset_after_lockout"])
    self.assertFalse(doc["unlocked8"])
    self.assertFalse(doc["write_attempted"])
    self.assertEqual([d for d in self.tx_frames(car) if d[1] == 0x2E], [])
    self.assertEqual([c["step_label"] for c in doc["cands8"]], ["a1", "a2", "a2b"])
    self.assertEqual([c["nrc"] for c in doc["cands8"]], [0x35, 0x36, 0x35])
    self.assertEqual(len(self.key_frames(car)), 3)
    # the wait happened between A2 and A2b (>= 25 s of the run's own clock)
    kt = self.key_times(car)
    self.assertGreaterEqual(kt[2] - kt[1], E.PHASE8_WAIT_S)
    self.assertLessEqual(s["duration_s"], E.RUN_BUDGET_S_PHASE8)

  def test_time_reset_false_when_still_locked(self):
    self.fresh_state()
    self.set_phase(8, candidates=self.CAND3)
    car = self.p8_car()
    car.key12_actions = [{"nrc": 0x35}, {"nrc": 0x37}, {"nrc": 0x36}]
    self.run_car(car)
    doc = self.result_doc()
    self.assertFalse(doc["cycle_clears"])
    self.assertFalse(doc["time_reset"])
    self.assertFalse(doc["write_attempted"])

  # ---- lockout at the very first key: 30 s wait -> A1b -> time_reset_after_lockout ----------------------------------
  def test_lockout_at_start_time_reset_35(self):
    self.fresh_state()
    self.set_phase(8, candidates=self.CAND3)
    car = self.p8_car()
    car.key12_actions = [{"nrc": 0x36}, {"nrc": 0x35}]
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    exp = [self.READ, self.SESSION,
           self.SEED11, self.FC, self.ff(self.K_LIT), self.cf(self.K_LIT),          # A1 -> 0x36
           self.SESSION01, self.SESSION,                                             # the 30 s wait then re-cycle
           self.SEED11, self.FC, self.ff(self.K_LIT), self.cf(self.K_LIT),          # A1b -> 0x35
           self.A2905, self.SESSION01, self.READ]
    self.assertEqual(self.tx_frames(car), exp)
    doc = self.result_doc()
    self.assertTrue(doc["lockout_at_start"])
    self.assertTrue(doc["time_reset_after_lockout"])
    self.assertIsNone(doc["cycle_clears"])
    self.assertFalse(doc["unlocked8"])
    self.assertEqual([c["step_label"] for c in doc["cands8"]], ["a1", "a1b"])
    # NO walk: only the first candidate is ever tried
    self.assertEqual([c["candidate"] for c in doc["cands8"]], ["lit270100", "lit270100"])
    kt = self.key_times(car)
    self.assertGreaterEqual(kt[1] - kt[0], E.PHASE8_LOCKOUT_WAIT_S)
    self.assertLessEqual(s["duration_s"], E.RUN_BUDGET_S_PHASE8)

  def test_lockout_at_start_a1b_unlocks_then_write(self):
    self.fresh_state()
    self.set_phase(8, candidates=self.CAND3)
    car = self.p8_car()
    car.key12_actions = [{"nrc": 0x36}, {"unlock": True}]
    self.run_car(car)
    doc = self.result_doc()
    self.assertTrue(doc["lockout_at_start"])
    self.assertTrue(doc["time_reset_after_lockout"])
    self.assertTrue(doc["unlocked8"])
    self.assertTrue(doc["write8"]["positive"])
    self.assertEqual([d for d in self.tx_frames(car) if d[1] == 0x2E], [self.WRITE])

  def test_lockout_at_start_a1b_locked_again(self):
    self.fresh_state()
    self.set_phase(8, candidates=self.CAND3)
    car = self.p8_car()
    car.key12_actions = [{"nrc": 0x37}, {"nrc": 0x36}]
    self.run_car(car)
    doc = self.result_doc()
    self.assertTrue(doc["lockout_at_start"])
    self.assertFalse(doc["time_reset_after_lockout"])
    self.assertFalse(doc["write_attempted"])

  # ---- the very first key unlocks -> straight to the win, no cycle/time test ----------------------------------------
  def test_first_attempt_unlocks_no_cycle(self):
    self.fresh_state()
    self.set_phase(8, candidates=self.CAND3)
    car = self.p8_car()
    car.key12_actions = [{"unlock": True}]
    self.run_car(car)
    doc = self.result_doc()
    self.assertTrue(doc["unlocked8"])
    self.assertIsNone(doc["lockout_at_start"])
    self.assertIsNone(doc["cycle_clears"])
    self.assertIsNone(doc["time_reset"])
    self.assertEqual([c["step_label"] for c in doc["cands8"]], ["a1"])
    self.assertEqual(len(self.key_frames(car)), 1)
    self.assertEqual([d for d in self.tx_frames(car) if d[1] == 0x2E], [self.WRITE])

  # ---- session / read gates -----------------------------------------------------------------------------------------
  def test_session_not_positive_skips_key_battery(self):
    self.fresh_state()
    self.set_phase(8)
    car = self.p8_car()
    car.session_positive = {0x01: True, 0x03: False}
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["aborted"])
    doc = self.result_doc()
    self.assertEqual(doc["seeds8"], [])
    self.assertEqual(doc["cands8"], [])
    self.assertFalse(doc["unlocked8"])
    self.assertEqual(self.key_frames(car), [])
    self.assertEqual(self.seed_frames(car), [])
    # the trailing steps still run
    self.assertIn(self.A2905, self.tx_frames(car))
    self.assertIn(self.SESSION01, self.tx_frames(car))

  def test_read_gate_aborts(self):
    car = self.p8_car()
    car.read_refused = True
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIn("0x0103", s["aborted"])
    self.assertEqual([d[1] for d in self.tx_frames(car)], [0x22])   # only the failed read
    self.assertEqual(car.mux_calls, [True, False])

  # ---- candidates list handling -------------------------------------------------------------------------------------
  def test_default_candidates_are_lit_algo8w_algo8(self):
    self.fresh_state()
    self.set_phase(8)
    car = self.p8_car()
    car.key12_actions = [{"nrc": 0x35}] * 8
    self.run_car(car)
    doc = self.result_doc()
    # A1 + A2 (cand0, twice) then the walk over the remaining defaults until the list is exhausted
    self.assertEqual([c["candidate"] for c in doc["cands8"]],
                     ["lit270100", "lit270100", "algo8w_27100", "algo8_27100"])
    self.assertEqual([c["step_label"] for c in doc["cands8"]], ["a1", "a2", "walk1", "walk2"])
    self.assertFalse(doc["unlocked8"])
    self.assertTrue(doc["cycle_clears"])

  def test_candidates_capped_at_four_and_unknown_dropped(self):
    self.fresh_state()
    self.set_phase(8, candidates=["lit270100", "banana", "algo8w_27100", "algo8_27100", "algo8_26700", "zero8"])
    car = self.p8_car()
    car.key12_actions = [{"nrc": 0x35}] * 6
    self.run_car(car)
    doc = self.result_doc()
    used = [c["candidate"] for c in doc["cands8"]]
    self.assertEqual(used, ["lit270100", "lit270100", "algo8w_27100", "algo8_27100", "algo8_26700"])
    self.assertNotIn("zero8", used)
    self.assertNotIn("banana", used)

  def test_all_invalid_p8_candidates_skips(self):
    self.fresh_state()
    self.set_phase(8, candidates=["banana", "nope"])
    car = self.p8_car()
    s = self.run_car(car)
    self.assertFalse(s["ran"])
    self.assertIn("p8_candidates", s["skip"])
    self.assertEqual(car.sent, [])

  # ---- scheduling / stationarity ------------------------------------------------------------------------------------
  def test_once_per_ignition(self):
    car = self.p8_car()
    self.run_car(car, key="boot:1")
    n = len(car.sent)
    s = self.run_car(car, key="boot:1")
    self.assertFalse(s["ran"])
    self.assertEqual(s["skip"], "already done (ignition)")
    self.assertEqual(len(car.sent), n)

  def test_not_in_park_sends_nothing(self):
    car = self.p8_car(gear=5)
    s = self.run_car(car)
    self.assertFalse(s["ran"])
    self.assertEqual(car.sent, [])
    self.assertEqual(car.mux_calls, [])

  def test_phase8_budget_is_140(self):
    self.assertEqual(E.RUN_BUDGET_S_PHASE8, 140.0)
    self.assertEqual(E.RUN_BUDGET_S_PHASE7, 60.0)
    self.assertEqual(E.PHASE8_MAX_KEY_ATTEMPTS, 6)
    self.assertEqual(E.PHASE8_MAX_SEEDS, 8)

  # ---- guards: the closed phase-8 allowlist -------------------------------------------------------------------------
  def test_guard_service_phase8_closed_set(self):
    E.guard_service(0x22, None, 0x0103, phase=8)
    E.guard_service(0x10, 0x01, None, phase=8)
    E.guard_service(0x10, 0x03, None, phase=8)
    E.guard_service(0x27, 0x11, None, phase=8)
    E.guard_service(0x27, 0x12, None, phase=8)
    E.guard_service(0x29, 0x05, None, phase=8)
    E.guard_service(0x2E, None, 0x0103, phase=8)
    # 27 01/27 02 are NOT phase-8 sub-functions (the door-B battery never sends them)
    for sub in (0x01, 0x02, 0x10, 0x13, 0x00):
      with self.assertRaises(D.SafetyViolation):
        E.guard_service(0x27, sub, None, phase=8)
    # 10 only 01/03 (10 02 is NOT admissible here)
    for sub in (0x02, 0x04, 0x81, 0x83):
      with self.assertRaises(D.SafetyViolation):
        E.guard_service(0x10, sub, None, phase=8)
    # 22 only 0103 (F100 is NOT admissible in phase 8)
    for did in (0xF100, 0x0104, 0x0000):
      with self.assertRaises(D.SafetyViolation):
        E.guard_service(0x22, None, did, phase=8)
    # 2E only DID 0103; 29 only sub 05
    with self.assertRaises(D.SafetyViolation):
      E.guard_service(0x2E, None, 0x0104, phase=8)
    with self.assertRaises(D.SafetyViolation):
      E.guard_service(0x29, 0x01, None, phase=8)
    with self.assertRaises(D.SafetyViolation):
      E.guard_service(0x29, 0x02, None, phase=8)
    # everything else is out
    for svc in (0x19, 0x31, 0x3E, 0x23, 0x34, 0x35, 0x36, 0x37):
      with self.assertRaises(D.SafetyViolation):
        E.guard_service(svc, None, None, phase=8)

  def test_guard_frame_phase8_closed_shapes(self):
    for f in (self.READ, self.SESSION, self.SESSION01, self.SEED11, self.A2905):
      E.guard_frame(0x7D1, f, 1, phase=8)
    E.guard_frame(0x7D1, self.FC, 1, phase=8)                              # RX flow control
    E.guard_frame(0x7D1, self.ff(self.K_LIT), 1, phase=8, key=self.K_LIT, p8_seed=True, key_attempts=0)
    E.guard_frame(0x7D1, self.cf(self.K_LIT), 1, phase=8, key=self.K_LIT)
    E.guard_frame(0x7D1, self.WRITE, 1, phase=8, readback=CURRENT)
    # extra request addresses are NOT admissible in phase 8 (no F100 anywhere)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x770, self.READ, 1, phase=8)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x770, bytes([3, 0x22, 0xF1, 0x00]).ljust(8, b"\x00"), 1, phase=8)
    # 27 12 without the sentinel, and with a wrong-length / wrong / missing key
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.ff(self.K_LIT), 1, phase=8, key=self.K_LIT, p8_seed=False, key_attempts=0)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.ff(self.K_LIT), 1, phase=8, key=None, p8_seed=True, key_attempts=0)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.ff(self.K_LIT), 1, phase=8, key=self.K_ALGO8W, p8_seed=True, key_attempts=0)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.ff(self.K_LIT), 1, phase=8, key=self.K_LIT[:4], p8_seed=True, key_attempts=0)
    # a 7th attempt (counter at the cap) is refused
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.ff(self.K_LIT), 1, phase=8, key=self.K_LIT, p8_seed=True,
                    key_attempts=E.PHASE8_MAX_KEY_ATTEMPTS)
    # the consecutive frame must carry the key tail
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.cf(self.K_ALGO8W), 1, phase=8, key=self.K_LIT)
    # a 27 12 single frame is NEVER admissible in phase 8 (8-byte ISO-TP only)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([6, 0x27, 0x12, 1, 2, 3, 4]).ljust(8, b"\x00"), 1, phase=8, p8_seed=True)
    # 27 01/27 02, wrong 10 sub, wrong 29 sub, F100, wrong 2E payload / missing readback
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([2, 0x27, 0x01]).ljust(8, b"\x00"), 1, phase=8)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([2, 0x27, 0x02]).ljust(8, b"\x00"), 1, phase=8)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([2, 0x10, 0x02]).ljust(8, b"\x00"), 1, phase=8)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([2, 0x29, 0x01]).ljust(8, b"\x00"), 1, phase=8)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([3, 0x22, 0xF1, 0x00]).ljust(8, b"\x00"), 1, phase=8)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([7, 0x2E, 0x01, 0x03]) + bytes(4), 1, phase=8, readback=CURRENT)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.WRITE, 1, phase=8)                           # no readback -> refused
    # 27 11 seed cap
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.SEED11, 1, phase=8, seed_attempts=E.PHASE8_MAX_SEEDS)
    # wrong bus / length
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.READ, 0, phase=8)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.SEED11[:3], 1, phase=8)

  def test_phase8_shapes_refused_outside_phase8(self):
    # the phase-8-only shapes must NOT be admissible in phases 1-7
    with self.assertRaises(D.SafetyViolation):
      E.guard_service(0x29, 0x05, None, phase=7)           # 29 05 not a phase-7 sub
    with self.assertRaises(D.SafetyViolation):
      E.guard_service(0x29, 0x05, None, phase=5)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.A2905, 1, phase=7)
    # NB: phase 7 DOES admit an 8-byte `27 12` with its own key/sentinel (that is phase 7's own door) -- it just does
    # not know the phase-8 `29 05` probe. Both facts are asserted above/below.
    E.guard_service(0x29, 0x01, None, phase=7)
    with self.assertRaises(D.SafetyViolation):
      E.guard_service(0x29, 0x05, None, phase=7)

  def test_single_tx_site_unchanged(self):
    tree = ast.parse(inspect.getsource(E))
    calls = [getattr(n.func, "attr", getattr(n.func, "id", "")) for n in ast.walk(tree)
             if isinstance(n, ast.Call) and getattr(n.func, "attr", getattr(n.func, "id", "")) == "_can_send"]
    self.assertEqual(sorted(calls), ["_can_send", "_can_send"])

  def test_write_did_is_the_only_write_and_is_a_noop(self):
    # the phase-8 write path reuses write_did: value must equal the step-1 readback
    car = self.p8_car()
    c = E.EscProbeClient(car.can_send, car.can_recv, D.VehicleGate(car.now), car.now, car.now() + 99, phase=8)
    with self.assertRaises(D.SafetyViolation):
      c.write_did(E.DID_VARIANT_CODING, b"\x01\x02\x03\x04", CURRENT)     # value != readback


class TestPhase9(Base):
  """Phase 9 (state ``{"probe_enabled": true, "phase": 9, "p9_candidates": [...], "p9_wait_s": ...}``): the DOOR-B
  CANDIDATE WALK -- up to 10 candidates, ONE ``27 12`` per slot, separated by the workable time-reset recipe (a REAL
  ``p9_wait_s`` sleep, then ``10 01`` -> ``10 03``). Only a positive ``67 12`` unlocks the ONE no-op ``2E 0103``; a
  ``0x36``/``0x37`` gets ONE same-candidate retry after the recipe, and a still-locked retry sets ``hard_lock``. No
  ``27 01``/``27 02``, no ``29``/``31``/``34``-``37``. Every guard is mechanically enforced."""

  READ = bytes([3, 0x22, 0x01, 0x03]).ljust(8, b"\x00")
  SESSION = bytes([2, 0x10, 0x03]).ljust(8, b"\x00")
  SESSION01 = bytes([2, 0x10, 0x01]).ljust(8, b"\x00")
  SEED11 = bytes([2, 0x27, 0x11]).ljust(8, b"\x00")
  WRITE = bytes([7, 0x2E, 0x01, 0x03]) + CURRENT
  FC = D.FLOW_CONTROL_FRAME

  K_LIT = bytes.fromhex("0032373031303000")     # lit270100 = 00 '2' '7' '0' '1' '0' '0' 00
  K_ALGO8W = bytes.fromhex("f1c6f1c6f1c6f1c6")   # cal_27100(seed[:4])[:2] x4
  K_ALGO8 = bytes.fromhex("f1c60000f1c60000")    # cal_27100(seed[:4]) x2
  K_26400W = bytes.fromhex("7c707c707c707c70")   # cal_26400(seed[:2])[:2] x4 (7c70)
  K_26800W = bytes.fromhex("4d2d4d2d4d2d4d2d")   # cal_26800(seed[:2])[:2] x4 (4d2d)
  K_26600W = bytes.fromhex("d408d408d408d408")   # cal_26600(seed[:2])[:2] x4 (d408)
  # the four offline-leftover tokens (phase-9 extended walk). All on the on-car seed 5AB05AB05AB05AB0.
  K_26300W = bytes.fromhex("0086008600860086")   # cal_26300(seed[:4])[:2] x4 ([00,HI]x4; raw 0086002b)
  K_26300_8 = bytes.fromhex("0086002b0086002b")  # cal_26300(seed[:4]) x2 (the 4-byte key doubled)
  K_27400W = bytes.fromhex("c0b0c0b0c0b0c0b0")   # cal_27400(seed[:4])[:2] x4 ([c0,b0]x4)
  K_27400_8 = bytes.fromhex("c0b0c0b0c0b0c0b0")  # cal_27400(seed[:4]) x2 (on-car seed repeats -> same bytes)
  K_40000W = bytes.fromhex("0000000000000000")   # cal_40000(seed[:2]) [:2] x4 = 0000 x4 (5AB0 not in the lookup table)
  # a generic NON-repeating seed, where the x4-repeat and x2-double shapes DIFFER
  P9_SEED = bytes.fromhex("1122334455667788")

  CAND3 = ["algo8w_27100", "algo8_27100", "algo8w_26400"]
  CAND5 = ["algo8w_27100", "algo8_27100", "algo8w_26400", "algo8w_26800", "algo8w_26600"]

  def setUp(self):
    super().setUp()
    self.set_phase(9, candidates=self.CAND3)

  def set_phase(self, phase, candidates=None, wait_s=None):
    state = {"probe_enabled": True, "phase": phase}
    if candidates is not None:
      state["p9_candidates"] = candidates
    if wait_s is not None:
      state["p9_wait_s"] = wait_s
    with open(os.path.join(self.out, "state.json"), "w") as f:
      json.dump(state, f)

  def p9_car(self, **kw):
    car = FakeCar(**kw)
    car.ask_family = True             # 27 11 -> 67 11 + 8-byte seed; 27 12 per the action script
    car.esc_seed8_11 = PHASE4_SEED
    return car

  @staticmethod
  def ff(key):
    return bytes([0x10, 0x0A, 0x27, 0x12]) + bytes(key)[:4]

  @staticmethod
  def cf(key):
    return (bytes([0x21]) + bytes(key)[4:8]).ljust(8, b"\x00")

  def slot_frames(self, key):
    return [self.SEED11, self.FC, self.ff(key), self.cf(key)]

  def step_names(self, doc):
    return [st["name"] for st in doc["steps"]]

  def key_times(self, car):
    """The send-time of every 27 12 first frame -- for the wait-path timing asserts."""
    return [car.sent_t[i] for i, (_, d, _) in enumerate(car.sent) if d[:4] == bytes([0x10, 0x0A, 0x27, 0x12])]

  def seed_frames(self, car):
    return [d for _, d, _ in car.sent if d[:3] == bytes([2, 0x27, 0x11])]

  def key_frames(self, car):
    return [d for _, d, _ in car.sent if d[:4] == bytes([0x10, 0x0A, 0x27, 0x12])]

  # ---- resolver: the three new 2-byte-algo tokens + every shared phase-8 token ---------------------------------------
  def test_resolve_phase9_new_tokens(self):
    seed = PHASE4_SEED
    self.assertEqual(E.resolve_phase9("algo8w_26400", seed), self.K_26400W)
    self.assertEqual(E.resolve_phase9("algo8w_26800", seed), self.K_26800W)
    self.assertEqual(E.resolve_phase9("algo8w_26600", seed), self.K_26600W)
    self.assertEqual(E.resolve_phase9("algo8w_26400", seed), SK.cal_26400(seed[:2])[:2] * 4)
    self.assertEqual(E.resolve_phase9("algo8w_26800", seed), SK.cal_26800(seed[:2])[:2] * 4)
    self.assertEqual(E.resolve_phase9("algo8w_26600", seed), SK.cal_26600(seed[:2])[:2] * 4)
    # every token resolves to EXACTLY 8 wire bytes
    for c in E.PHASE9_CANDIDATES:
      self.assertEqual(len(E.resolve_phase9(c, seed)), 8, c)
    # the shared phase-8/7 tokens keep their values
    self.assertEqual(E.resolve_phase9("lit270100", seed), self.K_LIT)
    self.assertEqual(E.resolve_phase9("algo8w_27100", seed), self.K_ALGO8W)
    self.assertEqual(E.resolve_phase9("algo8_27100", seed), self.K_ALGO8)
    self.assertEqual(E.resolve_phase9("zero8", seed), bytes(8))
    self.assertEqual(E.resolve_phase9("identity8", seed), PHASE4_SEED)
    self.assertEqual(E.resolve_phase9("algo8_26700", seed), SK.cal_26700(seed[:4]) * 2)
    self.assertEqual(E.resolve_phase9("algo8_26300", seed), SK.cal_26300(seed[:4]) * 2)
    # ---- the four offline-leftover tokens (the extended walk) ----
    self.assertEqual(E.resolve_phase9("algo8w_26300", seed), self.K_26300W)
    self.assertEqual(E.resolve_phase9("algo8w_27400", seed), self.K_27400W)
    self.assertEqual(E.resolve_phase9("algo8_27400", seed), self.K_27400_8)
    self.assertEqual(E.resolve_phase9("algo8w_40000", seed), self.K_40000W)
    self.assertEqual(E.resolve_phase9("algo8w_26300", seed), SK.cal_26300(seed[:4])[:2] * 4)
    self.assertEqual(E.resolve_phase9("algo8w_27400", seed), SK.cal_27400(seed[:4])[:2] * 4)
    self.assertEqual(E.resolve_phase9("algo8_27400", seed), SK.cal_27400(seed[:4]) * 2)
    self.assertEqual(E.resolve_phase9("algo8w_40000", seed), SK.cal_40000(seed[:2])[:2] * 4)
    # on a NON-repeating seed the [HI,LO]x4 and (4B)x2 shapes genuinely DIFFER (a mutant cannot alias them)
    self.assertEqual(E.resolve_phase9("algo8w_27400", self.P9_SEED), bytes.fromhex("4a444a444a444a44"))
    self.assertEqual(E.resolve_phase9("algo8_27400", self.P9_SEED), bytes.fromhex("4a44ce224a44ce22"))
    self.assertEqual(E.resolve_phase9("algo8w_26300", self.P9_SEED), bytes.fromhex("0078007800780078"))
    self.assertNotEqual(E.resolve_phase9("algo8w_27400", self.P9_SEED),
                        E.resolve_phase9("algo8_27400", self.P9_SEED))
    # 40000 is the fixed lookup: 0x0028 -> 0x0144; off-table -> 0000 (no zero-bail)
    self.assertEqual(E.resolve_phase9("algo8w_40000", bytes.fromhex("0028") + bytes(6)),
                     bytes.fromhex("0144014401440144"))
    self.assertEqual(E.resolve_phase9("algo8w_40000", bytes(8)), bytes(8))
    # 26400 bails on a zero seed byte; 26600 bails only on a zero WORD
    z = bytes.fromhex("00b05ab05ab05ab0")
    self.assertIsNone(E.resolve_phase9("algo8w_26400", z))
    self.assertIsNotNone(E.resolve_phase9("algo8w_26600", z))
    self.assertIsNone(E.resolve_phase9("algo8w_26600", bytes(8)))
    # 26300 bails on a zero seed byte (like 27100); 27400/40000 have NO zero-bail (always 8 bytes)
    self.assertIsNone(E.resolve_phase9("algo8w_26300", z))
    self.assertEqual(len(E.resolve_phase9("algo8w_27400", bytes(8))), 8)
    self.assertEqual(len(E.resolve_phase9("algo8_27400", bytes(8))), 8)
    with self.assertRaises(D.Abort):
      E.resolve_phase9("banana", seed)
    # the vocabulary is exactly the phase-8 tokens + the three 2-byte-algo wrappers + the four offline leftovers (== 14)
    self.assertEqual(set(E.PHASE9_CANDIDATES),
                     set(E.PHASE8_CANDIDATES) | {"algo8w_26400", "algo8w_26800", "algo8w_26600",
                                                 "algo8w_26300", "algo8w_27400", "algo8_27400", "algo8w_40000"})
    self.assertEqual(len(E.PHASE9_CANDIDATES), 14)
    self.assertEqual(E.PHASE9_DEFAULT_CANDIDATES,
                     ("algo8_26300", "lit270100", "algo8w_26300", "algo8w_27400", "algo8_27400", "algo8w_40000"))

  def test_cal_26600_known_vectors(self):
    self.assertEqual(SK.cal_26600(bytes.fromhex("5ab0")), bytes.fromhex("d408"))
    self.assertEqual(SK.cal_26600(bytes.fromhex("1122")), bytes.fromhex("9a65"))
    self.assertIsNone(SK.cal_26600(bytes(2)))                       # zero word -> vendor bail
    self.assertIn("26600", SK.ALGO_NAMES)
    self.assertIn("26600", SK.candidates(PHASE4_SEED))              # 2-byte seed -> 2-byte key, no crash
    self.assertEqual(SK.key_for(PHASE4_SEED, "26600"), SK.cal_26600(PHASE4_SEED[:2]))

  def test_cal_40000_known_vectors(self):
    # Securityindex 40000: a FIXED lookup; 0000 for any seed off the table (NO zero-bail)
    self.assertEqual(SK.cal_40000(bytes.fromhex("0003")), bytes.fromhex("001a"))
    self.assertEqual(SK.cal_40000(bytes.fromhex("0028")), bytes.fromhex("0144"))
    self.assertEqual(SK.cal_40000(bytes.fromhex("007a")), bytes.fromhex("03d6"))
    self.assertEqual(SK.cal_40000(bytes.fromhex("5ab0")), bytes(2))              # off-table -> 0000
    self.assertEqual(SK.cal_40000(bytes(2)), bytes(2))
    self.assertIn("40000", SK.ALGO_NAMES)
    self.assertIn("40000", SK.candidates(PHASE4_SEED))
    self.assertEqual(SK.key_for(PHASE4_SEED, "40000"), SK.cal_40000(PHASE4_SEED[:2])[:2])

  def test_phase9_constants(self):
    self.assertEqual(E.RUN_BUDGET_S_PHASE9, 420.0)
    self.assertEqual(E.PHASE9_MAX_CANDIDATES, 10)
    self.assertEqual(E.PHASE9_MAX_KEY_ATTEMPTS, 16)
    self.assertEqual(E.PHASE9_MAX_SEEDS, 20)
    self.assertEqual(E.PHASE9_WAIT_S, 25.0)
    self.assertEqual((E.PHASE9_WAIT_MIN_S, E.PHASE9_WAIT_MAX_S), (10.0, 60.0))

  # ---- run 1: win at slot 5 (slots 1-4 -> 0x35 each with waits; slot 5 -> 67 12 -> no-op write + reread) -------------
  def test_walk_win_at_slot_5(self):
    self.fresh_state()
    self.set_phase(9, candidates=self.CAND5)
    car = self.p9_car()
    car.key12_actions = [{"nrc": 0x35}, {"nrc": 0x35}, {"nrc": 0x35}, {"nrc": 0x35}, {"unlock": True}]
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["error"])
    self.assertIsNone(s["aborted"])
    keys = [self.K_ALGO8W, self.K_ALGO8, self.K_26400W, self.K_26800W, self.K_26600W]
    exp = [self.READ, self.SESSION]
    for i, k in enumerate(keys):
      if i:
        exp += [self.SESSION01, self.SESSION]
      exp += self.slot_frames(k)
    exp += [self.WRITE, self.READ, self.SESSION01, self.READ]        # win path + clean leave + value_end
    self.assertEqual(self.tx_frames(car), exp)
    doc = self.result_doc()
    self.assertEqual([a["slot"] for a in doc["attempts9"]], [0, 1, 2, 3, 4])
    self.assertEqual([a["candidate"] for a in doc["attempts9"]], self.CAND5)
    self.assertEqual([a["positive"] for a in doc["attempts9"]], [False, False, False, False, True])
    self.assertEqual([a["key_hex"] for a in doc["attempts9"]],
                     [self.K_ALGO8W.hex(), self.K_ALGO8.hex(), self.K_26400W.hex(), self.K_26800W.hex(),
                      self.K_26600W.hex()])
    self.assertTrue(doc["unlocked9"])
    self.assertIsNone(doc["hard_lock"])
    self.assertTrue(doc["write9"]["positive"])
    self.assertEqual(doc["value_after_write9"], "90060350")
    self.assertEqual(doc["value_start"], "90060350")
    # exactly ONE no-op write, payload == value_start
    self.assertEqual([d for d in self.tx_frames(car) if d[1] == 0x2E], [self.WRITE])
    # no 27 01/27 02, no 29/31/34-37 anywhere (single frames only)
    single = [d for d in self.tx_frames(car) if d[0] >> 4 == 0]
    for svc in (0x29, 0x31, 0x34, 0x35, 0x36, 0x37):
      self.assertEqual([d for d in single if d[1] == svc], [])
    self.assertEqual([d for d in single if d[1] == 0x27 and d[2] in (0x01, 0x02)], [])
    self.assertEqual(len(self.key_frames(car)), 5)
    self.assertEqual(len(self.seed_frames(car)), 5)
    self.assertEqual(self.step_names(doc)[-4:], ["write9", "read_after_write9", "leave_session", "read_esc_end"])
    self.assertEqual(self.events[-1][0], "esc_probe_0027")
    # the wait happened between EVERY adjacent pair of slots (>= 25 s of the run's own clock)
    kt = self.key_times(car)
    for i in range(1, 5):
      self.assertGreaterEqual(kt[i] - kt[i - 1], E.PHASE9_WAIT_S)
    self.assertLessEqual(s["duration_s"], E.RUN_BUDGET_S_PHASE9)
    self.assertEqual(s["p9_wait_s"], E.PHASE9_WAIT_S)

  # ---- run 2: all 6 default slots fail cleanly -----------------------------------------------------------------------
  def test_walk_all_6_default_slots_fail_cleanly(self):
    self.fresh_state()
    self.set_phase(9)                              # the 6-entry default list (the offline leftovers)
    car = self.p9_car()
    car.key12_actions = [{"nrc": 0x35}] * 6
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["error"])
    doc = self.result_doc()
    self.assertEqual([a["slot"] for a in doc["attempts9"]], list(range(6)))
    self.assertEqual([a["candidate"] for a in doc["attempts9"]], list(E.PHASE9_DEFAULT_CANDIDATES))
    self.assertEqual([a["nrc"] for a in doc["attempts9"]], [0x35] * 6)
    self.assertEqual([a["positive"] for a in doc["attempts9"]], [False] * 6)
    # every default token resolved to its EXACT 8 wire bytes (pinned in the frame, recorded in attempts9)
    self.assertEqual([a["key_hex"] for a in doc["attempts9"]],
                     [self.K_26300_8.hex(), self.K_LIT.hex(), self.K_26300W.hex(),
                      self.K_27400W.hex(), self.K_27400_8.hex(), self.K_40000W.hex()])
    self.assertFalse(doc["unlocked9"])
    self.assertIsNone(doc["hard_lock"])
    self.assertFalse(doc["write_attempted"])
    self.assertEqual([d for d in self.tx_frames(car) if d[1] == 0x2E], [])
    self.assertEqual(len(self.key_frames(car)), 6)
    self.assertEqual(len(self.seed_frames(car)), 6)
    self.assertEqual(self.step_names(doc), ["read_esc", "session"] +
                     [n for i in range(6) for n in (f"seed_{i}", f"key_{i}")] + ["leave_session", "read_esc_end"])
    kt = self.key_times(car)
    for i in range(1, 6):
      self.assertGreaterEqual(kt[i] - kt[i - 1], E.PHASE9_WAIT_S)
    self.assertLessEqual(s["duration_s"], E.RUN_BUDGET_S_PHASE9)

  # ---- the offline-leftover tokens' EXACT wire bytes (synthetic seeds) ------------------------------------------------
  def test_leftover_tokens_exact_wire_bytes(self):
    # on the on-car 8-byte seed 5AB05AB05AB05AB0
    self.assertEqual(E.resolve_phase9("algo8w_26300", PHASE4_SEED), self.K_26300W)
    self.assertEqual(E.resolve_phase9("algo8_26300", PHASE4_SEED), self.K_26300_8)
    self.assertEqual(E.resolve_phase9("algo8w_27400", PHASE4_SEED), self.K_27400W)
    self.assertEqual(E.resolve_phase9("algo8_27400", PHASE4_SEED), self.K_27400_8)
    self.assertEqual(E.resolve_phase9("algo8w_40000", PHASE4_SEED), self.K_40000W)
    self.assertEqual(self.K_40000W, bytes(8))
    # on a synthetic NON-repeating seed every token differs from every other
    a = {c: E.resolve_phase9(c, self.P9_SEED) for c in ("algo8w_26300", "algo8_26300", "algo8w_27400",
                                                        "algo8_27400", "algo8w_40000")}
    for c, k in a.items():
      self.assertEqual(len(k), 8, c)
    self.assertEqual(a["algo8w_26300"], bytes.fromhex("0078007800780078"))   # [00,0x78] x4
    self.assertEqual(a["algo8_26300"], bytes.fromhex("0078004b0078004b"))   # cal_26300 raw 0078004b, doubled
    self.assertEqual(a["algo8w_27400"], bytes.fromhex("4a444a444a444a44"))   # [4a,44] x4
    self.assertEqual(a["algo8_27400"], bytes.fromhex("4a44ce224a44ce22"))   # 4a44ce22 x2
    self.assertEqual(a["algo8w_40000"], bytes(8))                            # 1122 off-table -> 0000 x4
    # the five distinct shapes are pairwise distinct (no aliasing between the x4-repeat and x2-double constructions)
    self.assertEqual(len(set(a.values())), 5)

  def test_default_walk_slot_frame_sequence(self):
    # the 6-slot default walk emits exactly one slot (27 11 -> FC -> 27 12 FF -> 27 12 CF) per candidate, each preceded
    # (k>0) by the wait+cycle recipe (10 01 -> 10 03); every 27 12 first frame carries the token's EXACT 4-byte head.
    self.fresh_state()
    self.set_phase(9)                              # the default list
    car = self.p9_car()
    car.key12_actions = [{"nrc": 0x35}] * 6
    self.run_car(car)
    keys = [self.K_26300_8, self.K_LIT, self.K_26300W, self.K_27400W, self.K_27400_8, self.K_40000W]
    exp = [self.READ, self.SESSION]
    for i, k in enumerate(keys):
      if i:
        exp += [self.SESSION01, self.SESSION]
      exp += self.slot_frames(k)
    exp += [self.SESSION01, self.READ]             # clean leave + value_end
    self.assertEqual(self.tx_frames(car), exp)

  # ---- run 3: hard lock at slot 3 (0x36 twice -> stop) ----------------------------------------------------------------
  def test_hard_lock_at_slot_3(self):
    self.fresh_state()
    self.set_phase(9, candidates=self.CAND5)
    car = self.p9_car()
    car.key12_actions = [{"nrc": 0x35}, {"nrc": 0x35}, {"nrc": 0x35}, {"nrc": 0x36}, {"nrc": 0x36}]
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["error"])
    doc = self.result_doc()
    self.assertTrue(doc["hard_lock"])
    self.assertEqual(doc["hard_lock_slot"], 3)
    self.assertEqual([a["slot"] for a in doc["attempts9"]], [0, 1, 2, 3, 3])    # slot 3 recorded twice (the retry)
    self.assertEqual([a["nrc"] for a in doc["attempts9"]], [0x35, 0x35, 0x35, 0x36, 0x36])
    self.assertFalse(doc["unlocked9"])
    self.assertFalse(doc["write_attempted"])
    self.assertEqual([d for d in self.tx_frames(car) if d[1] == 0x2E], [])
    # 3 clean slots + slot-3 first + slot-3 retry = 5 keys, 5 seeds
    self.assertEqual(len(self.key_frames(car)), 5)
    self.assertEqual(len(self.seed_frames(car)), 5)
    # slot 4 was never tried
    self.assertNotIn("seed_4", self.step_names(doc))
    # the retry happened after the wait (~25 s) between the two slot-3 keys
    kt = self.key_times(car)
    self.assertGreaterEqual(kt[4] - kt[3], E.PHASE9_WAIT_S)
    self.assertLessEqual(s["duration_s"], E.RUN_BUDGET_S_PHASE9)

  # ---- run 4: first-slot 67 12 win -----------------------------------------------------------------------------------
  def test_first_slot_win(self):
    self.fresh_state()
    self.set_phase(9, candidates=self.CAND3)
    car = self.p9_car()
    car.key12_actions = [{"unlock": True}]
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    exp = [self.READ, self.SESSION] + self.slot_frames(self.K_ALGO8W) + [self.WRITE, self.READ,
                                                                         self.SESSION01, self.READ]
    self.assertEqual(self.tx_frames(car), exp)
    doc = self.result_doc()
    self.assertTrue(doc["unlocked9"])
    self.assertIsNone(doc["hard_lock"])
    self.assertEqual([a["slot"] for a in doc["attempts9"]], [0])
    self.assertEqual(len(self.key_frames(car)), 1)
    self.assertEqual(len(self.seed_frames(car)), 1)
    self.assertEqual([d for d in self.tx_frames(car) if d[1] == 0x2E], [self.WRITE])
    self.assertEqual(doc["value_after_write9"], "90060350")
    # no wait at all -> the two 27 12 slots never happened, and the run is short
    self.assertLess(s["duration_s"], E.PHASE9_WAIT_S)

  # ---- lockout that RECOVERS via the recipe (0x36 then 0x35 -> continue) -----------------------------------------------
  def test_lockout_retry_recovers_then_walks_on(self):
    self.fresh_state()
    self.set_phase(9, candidates=self.CAND3)
    car = self.p9_car()
    car.key12_actions = [{"nrc": 0x35}, {"nrc": 0x36}, {"nrc": 0x35}, {"unlock": True}]
    self.run_car(car)
    doc = self.result_doc()
    self.assertIsNone(doc["hard_lock"])
    self.assertTrue(doc["unlocked9"])
    self.assertEqual([a["slot"] for a in doc["attempts9"]], [0, 1, 1, 2])
    self.assertEqual([a["nrc"] for a in doc["attempts9"]], [0x35, 0x36, 0x35, None])
    self.assertEqual([d for d in self.tx_frames(car) if d[1] == 0x2E], [self.WRITE])

  # ---- session / read gates -------------------------------------------------------------------------------------------
  def test_session_not_positive_skips_walk(self):
    self.fresh_state()
    self.set_phase(9, candidates=self.CAND3)
    car = self.p9_car()
    car.session_positive = {0x01: True, 0x03: False}
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIsNone(s["aborted"])
    doc = self.result_doc()
    self.assertEqual(doc["seeds9"], [])
    self.assertEqual(doc["attempts9"], [])
    self.assertFalse(doc["unlocked9"])
    self.assertEqual(self.key_frames(car), [])
    self.assertEqual(self.seed_frames(car), [])
    # the clean leave + value_end still run
    self.assertIn(self.SESSION01, self.tx_frames(car))
    self.assertIn(self.READ, self.tx_frames(car))

  def test_read_gate_aborts(self):
    car = self.p9_car()
    car.read_refused = True
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    self.assertIn("0x0103", s["aborted"])
    self.assertEqual([d[1] for d in self.tx_frames(car)], [0x22])   # only the failed read
    self.assertEqual(car.mux_calls, [True, False])

  def test_bad_seed_stops_walk(self):
    self.fresh_state()
    self.set_phase(9, candidates=self.CAND3)
    car = self.p9_car()
    car.seed11_refused = True
    s = self.run_car(car)
    self.assertTrue(s["ran"])
    doc = self.result_doc()
    self.assertEqual(self.key_frames(car), [])
    self.assertEqual(len(doc["attempts9"]), 1)
    self.assertIn("no 27 12", doc["attempts9"][0]["note"])
    self.assertIsNotNone(doc["walk_stopped"])
    self.assertFalse(doc["write_attempted"])

  # ---- candidates list handling ---------------------------------------------------------------------------------------
  def test_candidates_capped_at_ten_and_unknown_dropped(self):
    self.fresh_state()
    self.set_phase(9, candidates=["algo8w_27100", "banana", "algo8_27100", "algo8w_26400", "algo8w_26800",
                                  "algo8w_26600", "algo8_26700", "algo8w_26300", "algo8w_27400", "algo8_27400",
                                  "algo8w_40000", "lit270100", "zero8", "identity8"])
    car = self.p9_car()
    car.key12_actions = [{"nrc": 0x35}] * 10
    self.run_car(car)
    doc = self.result_doc()
    used = [a["candidate"] for a in doc["attempts9"]]
    self.assertEqual(len(used), 10)
    self.assertNotIn("banana", used)
    self.assertEqual(used, ["algo8w_27100", "algo8_27100", "algo8w_26400", "algo8w_26800", "algo8w_26600",
                            "algo8_26700", "algo8w_26300", "algo8w_27400", "algo8_27400", "algo8w_40000"])
    # the dropped tail (lit270100/zero8/identity8) never got a 27 12
    self.assertEqual(len(self.key_frames(car)), 10)

  def test_all_invalid_p9_candidates_skips(self):
    self.fresh_state()
    self.set_phase(9, candidates=["banana", "nope"])
    car = self.p9_car()
    s = self.run_car(car)
    self.assertFalse(s["ran"])
    self.assertIn("p9_candidates", s["skip"])
    self.assertEqual(car.sent, [])

  def test_p9_wait_clamped(self):
    # below the floor -> 10 s
    self.fresh_state()
    self.set_phase(9, candidates=["algo8w_27100", "algo8_27100"], wait_s=5.0)
    car = self.p9_car()
    car.key12_actions = [{"nrc": 0x35}, {"nrc": 0x35}]
    s = self.run_car(car)
    self.assertEqual(s["p9_wait_s"], E.PHASE9_WAIT_MIN_S)
    kt = self.key_times(car)
    self.assertGreaterEqual(kt[1] - kt[0], E.PHASE9_WAIT_MIN_S)
    # above the ceiling -> 60 s
    self.fresh_state()
    self.set_phase(9, candidates=["algo8w_27100", "algo8_27100"], wait_s=999.0)
    car = self.p9_car()
    car.key12_actions = [{"nrc": 0x35}, {"nrc": 0x35}]
    s = self.run_car(car)
    self.assertEqual(s["p9_wait_s"], E.PHASE9_WAIT_MAX_S)
    kt = self.key_times(car)
    self.assertGreaterEqual(kt[1] - kt[0], E.PHASE9_WAIT_MAX_S)

  # ---- scheduling / stationarity --------------------------------------------------------------------------------------
  def test_once_per_ignition(self):
    car = self.p9_car()
    car.key12_actions = [{"unlock": True}]
    self.run_car(car, key="boot:1")
    n = len(car.sent)
    s = self.run_car(car, key="boot:1")
    self.assertFalse(s["ran"])
    self.assertEqual(s["skip"], "already done (ignition)")
    self.assertEqual(len(car.sent), n)

  def test_not_in_park_sends_nothing(self):
    car = self.p9_car(gear=5)
    s = self.run_car(car)
    self.assertFalse(s["ran"])
    self.assertEqual(car.sent, [])
    self.assertEqual(car.mux_calls, [])

  # ---- guards: the closed phase-9 allowlist ---------------------------------------------------------------------------
  def test_guard_service_phase9_closed_set(self):
    E.guard_service(0x22, None, 0x0103, phase=9)
    E.guard_service(0x10, 0x01, None, phase=9)
    E.guard_service(0x10, 0x03, None, phase=9)
    E.guard_service(0x27, 0x11, None, phase=9)
    E.guard_service(0x27, 0x12, None, phase=9)
    E.guard_service(0x2E, None, 0x0103, phase=9)
    # 27 01/27 02 are NOT phase-9 sub-functions (the walk never sends them)
    for sub in (0x01, 0x02, 0x10, 0x13, 0x00):
      with self.assertRaises(D.SafetyViolation):
        E.guard_service(0x27, sub, None, phase=9)
    # 10 only 01/03; 22 only 0103
    for sub in (0x02, 0x04, 0x81, 0x83):
      with self.assertRaises(D.SafetyViolation):
        E.guard_service(0x10, sub, None, phase=9)
    for did in (0xF100, 0x0104, 0x0000):
      with self.assertRaises(D.SafetyViolation):
        E.guard_service(0x22, None, did, phase=9)
    with self.assertRaises(D.SafetyViolation):
      E.guard_service(0x2E, None, 0x0104, phase=9)
    # 0x29 has NO admissible sub in phase 9 (the walk has no auth probe)
    for sub in (0x01, 0x05, None):
      with self.assertRaises(D.SafetyViolation):
        E.guard_service(0x29, sub, None, phase=9)
    # everything else is out
    for svc in (0x19, 0x31, 0x3E, 0x23, 0x34, 0x35, 0x36, 0x37):
      with self.assertRaises(D.SafetyViolation):
        E.guard_service(svc, None, None, phase=9)

  def test_guard_frame_phase9_closed_shapes(self):
    for f in (self.READ, self.SESSION, self.SESSION01, self.SEED11):
      E.guard_frame(0x7D1, f, 1, phase=9)
    E.guard_frame(0x7D1, self.FC, 1, phase=9)                              # RX flow control
    E.guard_frame(0x7D1, self.ff(self.K_ALGO8W), 1, phase=9, key=self.K_ALGO8W, p9_seed=True, key_attempts=0)
    E.guard_frame(0x7D1, self.cf(self.K_ALGO8W), 1, phase=9, key=self.K_ALGO8W)
    E.guard_frame(0x7D1, self.WRITE, 1, phase=9, readback=CURRENT)
    # extra request addresses are NOT admissible
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x770, self.READ, 1, phase=9)
    # 27 12 without the sentinel, and with a wrong-length / wrong / missing key
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.ff(self.K_ALGO8W), 1, phase=9, key=self.K_ALGO8W, p9_seed=False, key_attempts=0)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.ff(self.K_ALGO8W), 1, phase=9, key=None, p9_seed=True, key_attempts=0)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.ff(self.K_ALGO8W), 1, phase=9, key=self.K_ALGO8, p9_seed=True, key_attempts=0)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.ff(self.K_ALGO8W), 1, phase=9, key=self.K_ALGO8W[:4], p9_seed=True, key_attempts=0)
    # a 13th attempt (counter at the cap) is refused
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.ff(self.K_ALGO8W), 1, phase=9, key=self.K_ALGO8W, p9_seed=True,
                    key_attempts=E.PHASE9_MAX_KEY_ATTEMPTS)
    # the consecutive frame must carry the key tail
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.cf(self.K_ALGO8), 1, phase=9, key=self.K_ALGO8W)
    # a 27 12 single frame is NEVER admissible in phase 9
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([6, 0x27, 0x12, 1, 2, 3, 4]).ljust(8, b"\x00"), 1, phase=9, p9_seed=True)
    # 27 01/27 02, wrong 10 sub, F100, wrong 2E payload / missing readback
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([2, 0x27, 0x01]).ljust(8, b"\x00"), 1, phase=9)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([2, 0x27, 0x02]).ljust(8, b"\x00"), 1, phase=9)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([2, 0x10, 0x02]).ljust(8, b"\x00"), 1, phase=9)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([3, 0x22, 0xF1, 0x00]).ljust(8, b"\x00"), 1, phase=9)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([7, 0x2E, 0x01, 0x03]) + bytes(4), 1, phase=9, readback=CURRENT)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.WRITE, 1, phase=9)                          # no readback -> refused
    # 27 11 seed cap
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.SEED11, 1, phase=9, seed_attempts=E.PHASE9_MAX_SEEDS)
    # wrong bus / length
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.READ, 0, phase=9)
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.SEED11[:3], 1, phase=9)

  def test_phase9_shapes_refused_outside_phase9(self):
    # a `29 05` frame (admissible in phase 8) is NOT admissible in phase 9
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, bytes([2, 0x29, 0x05]).ljust(8, b"\x00"), 1, phase=9)
    # the phase-9-only walk shapes must NOT leak into phase 8: a 27 12 with p9_seed (and no p8_seed) is refused there
    with self.assertRaises(D.SafetyViolation):
      E.guard_frame(0x7D1, self.ff(self.K_ALGO8W), 1, phase=8, key=self.K_ALGO8W, p9_seed=True, p8_seed=False,
                    key_attempts=0)
    # and the phase-9 caps differ from phase 8's
    self.assertNotEqual(E.PHASE9_MAX_KEY_ATTEMPTS, E.PHASE8_MAX_KEY_ATTEMPTS)
    self.assertNotEqual(E.PHASE9_MAX_SEEDS, E.PHASE8_MAX_SEEDS)

  def test_single_tx_site_unchanged(self):
    tree = ast.parse(inspect.getsource(E))
    calls = [getattr(n.func, "attr", getattr(n.func, "id", "")) for n in ast.walk(tree)
             if isinstance(n, ast.Call) and getattr(n.func, "attr", getattr(n.func, "id", "")) == "_can_send"]
    self.assertEqual(sorted(calls), ["_can_send", "_can_send"])

  def test_write_did_is_the_only_write_and_is_a_noop(self):
    # the phase-9 write path reuses write_did: value must equal the step-1 readback
    car = self.p9_car()
    c = E.EscProbeClient(car.can_send, car.can_recv, D.VehicleGate(car.now), car.now, car.now() + 99, phase=9)
    with self.assertRaises(D.SafetyViolation):
      c.write_did(E.DID_VARIANT_CODING, b"\x01\x02\x03\x04", CURRENT)     # value != readback
