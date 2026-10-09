import threading
import time

import numpy as np

from openpilot.common.test import OpenpilotTestCase
from openpilot.cereal import log, messaging
from openpilot.cereal.messaging import SubMaster, PubMaster
from openpilot.selfdrive.ui.soundd import SELFDRIVE_STATE_TIMEOUT, AudioStreamSlot, Soundd, check_selfdrive_timeout_alert
from openpilot.system.tests.test_micd import FakeStream

AudibleAlert = log.SelfdriveState.AudibleAlert


class TestSoundd(OpenpilotTestCase):
  def test_check_selfdrive_timeout_alert(self, mocker):
    sm = SubMaster(['selfdriveState', 'selfdriveStateSP'])
    pm = PubMaster(['selfdriveState', 'selfdriveStateSP'])

    cs = messaging.new_message('selfdriveState')
    cs.selfdriveState.enabled = True
    threading.Timer(0.01, pm.send, args=("selfdriveState", cs)).start()
    sm.update(100)
    assert sm.updated['selfdriveState']

    sm.recv_time['selfdriveState'] = 0
    clock = mocker.patch("openpilot.selfdrive.ui.soundd.time.monotonic", return_value=SELFDRIVE_STATE_TIMEOUT)
    assert not check_selfdrive_timeout_alert(sm)

    clock.return_value = SELFDRIVE_STATE_TIMEOUT + 0.1
    assert check_selfdrive_timeout_alert(sm)

    clock.return_value = SELFDRIVE_STATE_TIMEOUT + 10
    assert not check_selfdrive_timeout_alert(sm)

  def test_check_selfdrive_timeout_alert_mads_lateral_only(self):
    sm = SubMaster(['selfdriveState', 'selfdriveStateSP'])
    pm = PubMaster(['selfdriveState', 'selfdriveStateSP'])

    for _ in range(100):
      cs = messaging.new_message('selfdriveState')
      cs.selfdriveState.enabled = False

      ss_sp = messaging.new_message('selfdriveStateSP')
      ss_sp.selfdriveStateSP.mads.enabled = True

      pm.send("selfdriveState", cs)
      pm.send("selfdriveStateSP", ss_sp)

      time.sleep(0.01)

      sm.update(0)

      assert not check_selfdrive_timeout_alert(sm)

    for _ in range(SELFDRIVE_STATE_TIMEOUT * 110):
      sm.update(0)
      time.sleep(0.01)

    assert check_selfdrive_timeout_alert(sm)

  # TODO: add test with micd for checking that soundd actually outputs sounds


class TestSounddCallback(OpenpilotTestCase):
  """The PortAudio callback must always produce a full buffer (silence when no
  alert is active) and never touch the stream, so audio unavailability can't
  raise inside the audio thread."""

  def _make_soundd(self):
    # bypass __init__: sound assets are git-lfs pointers off-device
    s = Soundd.__new__(Soundd)
    s.enabled = False
    s.current_alert = AudibleAlert.none
    s.current_volume = 1.0
    s.current_sound_frame = 0
    s.pending_stop = False
    s.loaded_sounds = {AudibleAlert.engage: np.ones(100, dtype=np.float32)}
    return s

  def test_callback_silent_without_alert(self):
    s = self._make_soundd()
    out = np.ones((64, 1), dtype=np.float32)

    s.callback(out, 64, None, None)

    assert np.all(out == 0)

  def test_callback_plays_current_alert(self):
    s = self._make_soundd()
    s.current_alert = AudibleAlert.engage
    out = np.zeros((64, 1), dtype=np.float32)

    s.callback(out, 64, None, None)

    assert np.all(out == 1)


class TestAudioStreamSlot(OpenpilotTestCase):
  """Regression coverage for the background acquire / re-acquire state machine.

  This is where the audio-unavailability handling lives: these tests pin the
  threading behavior that replaced the fatal startup deadline."""

  def test_ensure_acquires_in_background_and_get_active_returns_stream(self):
    stream = FakeStream()
    stream.active = True
    slot = AudioStreamSlot(lambda: stream, log_prefix="test")

    slot.ensure()
    for _ in range(100):
      if slot.get_active() is not None:
        break
      time.sleep(0.01)

    assert slot.get_active() is stream

  def test_get_active_drops_inactive_stream_and_closes_it(self):
    stream = FakeStream()
    stream.active = False
    slot = AudioStreamSlot(lambda: stream, log_prefix="test")
    slot.ensure()
    time.sleep(0.05)

    assert slot.get_active() is None
    assert stream.close_calls == 1

  def test_ensure_is_idempotent_while_acquiring(self):
    calls = {"n": 0}
    release = threading.Event()

    def open_fn():
      calls["n"] += 1
      release.wait(2.0)
      return FakeStream()

    slot = AudioStreamSlot(open_fn, log_prefix="test")
    for _ in range(5):
      slot.ensure()
    time.sleep(0.05)
    assert calls["n"] == 1  # one acquire in flight, not five
    release.set()

  def test_reacquires_after_stream_dropped(self):
    streams = []

    def open_fn():
      s = FakeStream()
      s.active = True
      streams.append(s)
      return s

    slot = AudioStreamSlot(open_fn, log_prefix="test")
    slot.ensure()
    for _ in range(100):
      if slot.get_active() is not None:
        break
      time.sleep(0.01)
    first = slot.get_active()
    assert first is not None

    first.active = False  # simulate mid-run device loss
    assert slot.get_active() is None

    slot.ensure()
    for _ in range(100):
      if slot.get_active() is not None:
        break
      time.sleep(0.01)

    assert slot.get_active() is not first
    assert len(streams) == 2

  def test_acquiring_flag_cleared_when_open_fn_raises(self, mocker):
    mocker.patch("openpilot.selfdrive.ui.soundd.cloudlog")
    attempt = {"n": 0}

    def open_fn():
      attempt["n"] += 1
      if attempt["n"] == 1:
        raise RuntimeError("boom")
      s = FakeStream()
      s.active = True
      return s

    slot = AudioStreamSlot(open_fn, log_prefix="test")
    slot.ensure()
    time.sleep(0.1)

    # the failed acquire must not wedge the slot: a later ensure() retries
    slot.ensure()
    for _ in range(100):
      if slot.get_active() is not None:
        break
      time.sleep(0.01)
    assert slot.get_active() is not None

