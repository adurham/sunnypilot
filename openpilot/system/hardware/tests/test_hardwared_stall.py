"""hardwared must keep publishing deviceState while /data is stalled (routes 189/18b/193: 3.8-8.0 s silences)."""
import queue
import threading
import time

import openpilot.cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.system.hardware import hardwared
from openpilot.system.hardware.params_worker import ParamsWorker


class FakeSubMaster:
  """Stands in for SubMaster(poll="pandaStates"): ~20 Hz updates; hardwared's gate (frame % 10) then publishes at ~2 Hz."""
  PERIOD = 0.05

  def __init__(self, services, poll=None):
    self.frame = -1
    self.services = services
    self.updated = dict.fromkeys(services, False)
    self.recv_time = dict.fromkeys(services, time.monotonic())
    self.alive = dict.fromkeys(services, False)
    self.valid = dict.fromkeys(services, True)
    self.data = {s: getattr(messaging.new_message(s), s) for s in services if s != "pandaStates"}
    self.data["pandaStates"] = []

  def update(self, timeout=100):
    time.sleep(self.PERIOD)
    self.frame += 1
    self.recv_time["pandaStates"] = time.monotonic()

  def __getitem__(self, s):
    return self.data[s]


class FakePubMaster:
  sends: list[float] = []

  def __init__(self, services):
    pass

  def send(self, s, msg):
    FakePubMaster.sends.append(time.monotonic())


class DataStall:
  """Makes every Params file op block (like a /data ext4 journal stall) while `stalled` is set."""

  def __init__(self, mocker):
    self.stalled = threading.Event()
    self.release = threading.Event()
    self.blocked_calls = 0
    for name in ("get", "get_bool", "put", "put_bool", "remove"):
      orig = getattr(Params, name)
      mocker.patch.object(Params, name, self._wrap(orig))

  def _wrap(self, orig):
    stall = self

    def wrapper(*args, **kwargs):
      if stall.stalled.is_set():
        stall.blocked_calls += 1
        stall.release.wait(10)
      return orig(*args, **kwargs)
    return wrapper


class NoChestnutStatus:
  """ChestnutStatus isn't under test; stubbed because a locally built libparams_c can predate its alert keys."""
  def update(self, *args, **kwargs):
    pass


class TestHardwaredStall(OpenpilotTestCase):
  def setup_method(self):
    mocker = self._fixture("mocker")
    mocker.patch.object(hardwared, "ChestnutStatus", NoChestnutStatus)
    mocker.patch.object(hardwared.messaging, "SubMaster", FakeSubMaster)
    mocker.patch.object(hardwared.messaging, "PubMaster", FakePubMaster)
    FakePubMaster.sends = []
    self.mocker = mocker
    self.end_event = threading.Event()
    self.thread = None

  def teardown_method(self):
    self.end_event.set()
    if self.thread is not None:
      self.thread.join(15)

  def _start(self):
    self.thread = threading.Thread(target=hardwared.hardware_thread, args=(self.end_event, queue.Queue(maxsize=1)), daemon=True)
    self.thread.start()
    t0 = time.monotonic()
    while len(FakePubMaster.sends) < 3:
      assert time.monotonic() - t0 < 20, "hardware_thread never published"
      time.sleep(0.05)

  def test_blocked_data_does_not_delay_device_state(self):
    stall = DataStall(self.mocker)
    self._start()

    # /data stops answering for 3 s (193 was 8 s)
    n0 = len(FakePubMaster.sends)
    stall.stalled.set()
    t_stall = time.monotonic()
    time.sleep(3.0)
    during = [t for t in FakePubMaster.sends[n0:] if t >= t_stall]
    stall.stalled.clear()
    stall.release.set()

    assert stall.blocked_calls > 0, "the stall never hit a params op - test is not exercising anything"
    # 2 Hz -> ~6 sends in 3 s; on the base code this is 0-1 (the loop sits in the blocked params op)
    assert len(during) >= 4, f"only {len(during)} deviceState sends during a 3 s /data stall"
    gaps = [b - a for a, b in zip([t_stall] + during, during, strict=False)]
    assert max(gaps) < 1.0, f"deviceState gap {max(gaps):.2f} s while /data was stalled"

  def test_slow_iteration_names_slowest_step(self):
    events = []
    real_event = hardwared.cloudlog.event
    self.mocker.patch.object(hardwared.cloudlog, "event", lambda name, **kw: (events.append((name, kw)), real_event(name, **kw)))
    slow = threading.Event()
    real_draw = hardwared.HARDWARE.get_current_power_draw

    def draw():
      if slow.is_set():
        slow.clear()
        time.sleep(0.5)
      return real_draw()
    self.mocker.patch.object(hardwared.HARDWARE, "get_current_power_draw", draw)
    self._start()
    slow.set()
    t0 = time.monotonic()
    while not any(e[0] == "hardwared slow iteration" for e in events):
      assert time.monotonic() - t0 < 5, "no slow-iteration cloudlog"
      time.sleep(0.05)
    kw = next(e[1] for e in events if e[0] == "hardwared slow iteration")
    assert kw["slowest_step"] == "power_draw"
    assert kw["duration"] >= 0.5 and kw["slowest_duration"] >= 0.5

  def test_values_still_persisted(self):
    self._start()
    params = Params()
    t0 = time.monotonic()
    while params.get("UptimeOffroad") is None:   # written at count 0, persisted by the worker
      assert time.monotonic() - t0 < 5, "UptimeOffroad never persisted"
      time.sleep(0.05)
    assert params.get_bool("IsEngaged") is False
    params.put_bool("OnroadCycleRequested", True, block=True)
    t0 = time.monotonic()
    while params.get_bool("OnroadCycleRequested"):  # picked up by the cached read, cleared through the worker
      assert time.monotonic() - t0 < 5, "OnroadCycleRequested was not consumed"
      time.sleep(0.05)


class TestParamsWorker(OpenpilotTestCase):
  def setup_method(self):
    self.params = Params()

  def test_reads_are_cached_and_refreshed(self):
    self.params.put_bool("DisableUpdates", True, block=True)
    w = ParamsWorker(["DisableUpdates"], refresh_period=0.05)
    try:
      assert w.get_bool("DisableUpdates")
      self.params.put_bool("DisableUpdates", False, block=True)
      t0 = time.monotonic()
      while w.get_bool("DisableUpdates"):
        assert time.monotonic() - t0 < 2
        time.sleep(0.01)
    finally:
      w.stop()

  def test_unregistered_read_raises(self):
    w = ParamsWorker([], start=False)
    try:
      w.get("DisableUpdates")
      raise AssertionError("expected KeyError")
    except KeyError:
      pass

  def test_latest_value_wins_and_read_your_writes(self):
    w = ParamsWorker(["OnroadCycleRequested"], start=False)  # worker not running: writes queue up
    w.put_bool("OnroadCycleRequested", True)
    for v in (1.0, 2.0, 3.0):
      w.put("UptimeOnroad", v)
    assert w.get_bool("OnroadCycleRequested") is True
    assert list(w._pending) == ["OnroadCycleRequested", "UptimeOnroad"] and w._pending["UptimeOnroad"][1] == 3.0
    w._thread.start()
    assert w.flush(5)
    assert self.params.get("UptimeOnroad") == 3.0
    assert self.params.get_bool("OnroadCycleRequested")
    w.stop()

  def test_refresh_does_not_resurrect_overwritten_value(self):
    self.params.put_bool("OnroadCycleRequested", True, block=True)
    w = ParamsWorker(["OnroadCycleRequested"], start=False)
    gate, entered = threading.Event(), threading.Event()
    real_get = w.params.get

    def slow_get(key, *a, **kw):
      value = real_get(key, *a, **kw)   # reads True
      entered.set()
      gate.wait(5)
      return value
    w.params.get = slow_get
    t = threading.Thread(target=w._refresh)
    t.start()
    entered.wait(5)
    w.put_bool("OnroadCycleRequested", False)  # write lands while the refresh holds a stale True
    gate.set()
    t.join(5)
    assert w.get_bool("OnroadCycleRequested") is False

  def test_set_alert_is_idempotent_and_off_thread(self):
    w = ParamsWorker([], start=False)
    w.set_alert("Offroad_TiciSupport", False)
    assert "Offroad_TiciSupport" in w._pending   # first call always clears a stale alert
    w._pending.clear()
    for _ in range(100):
      w.set_alert("Offroad_TiciSupport", False)
    assert not w._pending                          # the old code did 100 unlink+fsync(dir) here
    w.set_alert("Offroad_TemperatureTooHigh", True, "80.0C")
    w._thread.start()
    assert w.flush(5)
    assert self.params.get("Offroad_TemperatureTooHigh")["extra"] == "80.0C"
    w.set_alert("Offroad_TemperatureTooHigh", False)
    assert w.flush(5)
    assert self.params.get("Offroad_TemperatureTooHigh") is None
    w.stop()

  def test_dedupe(self):
    w = ParamsWorker([], start=False)
    w.put_bool("NetworkMetered", False, dedupe=True)
    w._pending.clear()
    w.put_bool("NetworkMetered", False, dedupe=True)
    assert not w._pending
    w.put_bool("NetworkMetered", True, dedupe=True)
    assert "NetworkMetered" in w._pending
