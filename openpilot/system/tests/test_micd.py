import numpy as np

from openpilot.common.test import OpenpilotTestCase
from openpilot.system import micd


class FakeStream:
  def __init__(self, start_raises: bool = False):
    self.active = False
    self.samplerate = micd.SAMPLE_RATE
    self.channels = 1
    self.dtype = "float32"
    self.device = 31
    self.blocksize = micd.SAMPLE_BUFFER
    self.start_calls = 0
    self.close_calls = 0
    self.start_raises = start_raises

  def start(self):
    self.start_calls += 1
    if self.start_raises:
      raise RuntimeError("PortAudio: Device unavailable")
    self.active = True

  def close(self):
    self.close_calls += 1
    self.active = False


class FakeSD:
  """Minimal stand-in for the sounddevice module. Never imports PortAudio."""

  def __init__(self, fail_attempts: int = 0, devices: list | None = None):
    self.fail_attempts = fail_attempts
    self.devices = devices if devices is not None else [{"name": "fake", "max_input_channels": 1, "max_output_channels": 2}]
    self.attempts = 0
    self.terminate_calls = 0
    self.initialize_calls = 0
    self.query_calls = 0
    self.streams: list[FakeStream] = []

  def _terminate(self):
    self.terminate_calls += 1

  def _initialize(self):
    self.initialize_calls += 1

  def query_devices(self):
    self.query_calls += 1
    return self.devices

  def factory(self):
    def make(_sd):
      self.attempts += 1
      if self.attempts <= self.fail_attempts:
        raise RuntimeError("PortAudio: Device unavailable")
      s = FakeStream()
      self.streams.append(s)
      return s
    return make


class TestOpenAudioStream(OpenpilotTestCase):
  def test_returns_stream_after_failures(self):
    sd = FakeSD(fail_attempts=3)

    stream = micd.open_audio_stream(sd, sd.factory(), log_prefix="test", retry_delay=0.0, attempts=10)

    assert stream is not None
    assert sd.attempts == 4
    # portaudio is torn down and re-enumerated once per attempt
    assert sd.terminate_calls == 4
    assert sd.initialize_calls == 4

  def test_returned_stream_is_started(self):
    sd = FakeSD()

    stream = micd.open_audio_stream(sd, sd.factory(), log_prefix="test", retry_delay=0.0, attempts=3)

    assert stream is not None
    assert stream.active
    assert stream.start_calls == 1

  def test_stream_closed_when_start_fails(self):
    sd = FakeSD()

    # first attempt: stream opens but start() raises; second: normal success
    calls = {"n": 0}

    def make(_sd):
      calls["n"] += 1
      if calls["n"] == 1:
        s = FakeStream(start_raises=True)
        sd.streams.append(s)
        return s
      s = FakeStream()
      sd.streams.append(s)
      return s

    stream = micd.open_audio_stream(sd, make, log_prefix="test", retry_delay=0.0, attempts=2)

    assert stream is not None
    assert stream.active
    # the stream that opened but failed to start must have been closed
    assert sd.streams[0].close_calls == 1
    assert sd.streams[0].active is False

  def test_bounded_attempts_exhausted_returns_none(self, mocker):
    sd = FakeSD(fail_attempts=99)
    warning = mocker.patch.object(micd.cloudlog, "warning")

    stream = micd.open_audio_stream(sd, sd.factory(), log_prefix="test", retry_delay=0.0, attempts=3)

    assert stream is None
    assert sd.attempts == 3
    assert sd.terminate_calls == 3
    assert sd.initialize_calls == 3
    # unchanged device set is logged once, not once per attempt
    assert warning.call_count == 1

  def test_never_raises_when_device_query_fails(self, mocker):
    sd = FakeSD(fail_attempts=99)

    def broken_query_devices():
      raise RuntimeError("PortAudio: not initialized")

    sd.query_devices = broken_query_devices
    mocker.patch.object(micd.cloudlog, "warning")

    stream = micd.open_audio_stream(sd, sd.factory(), log_prefix="test", retry_delay=0.0, attempts=2)

    assert stream is None
    assert sd.attempts == 2

  def test_logs_real_exception(self, mocker):
    sd = FakeSD(fail_attempts=1)
    warning = mocker.patch.object(micd.cloudlog, "warning")

    stream = micd.open_audio_stream(sd, sd.factory(), log_prefix="micd", retry_delay=0.0, attempts=2)

    assert stream is not None
    assert warning.call_count == 1
    assert "Device unavailable" in warning.call_args[0][0]
    assert "micd" in warning.call_args[0][0]


class TestMicStreamLifecycle(OpenpilotTestCase):
  def test_update_publishes_without_stream(self):
    m = micd.Mic()
    m.update()  # stream-independent; must not require an active stream

  def test_callback_fills_measurements(self):
    m = micd.Mic()
    indata = np.zeros((micd.SAMPLE_BUFFER, 1), dtype=np.float32)
    m.callback(indata, micd.SAMPLE_BUFFER, None, None)
    with m.lock:
      assert m.measurements.size == micd.SAMPLE_BUFFER
