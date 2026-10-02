#!/usr/bin/env python3
import numpy as np
from functools import cache
import threading
import time

from openpilot.cereal import messaging
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog

RATE = 10
FFT_SAMPLES = 1600 # 100ms
REFERENCE_SPL = 2e-5  # newtons/m^2
SAMPLE_RATE = 16000
SAMPLE_BUFFER = 800  # 50ms


def patch_sounddevice(sd):
  # TODO: remove once sounddevice uses np.reshape internally.
  def sounddevice_array(buffer, channels, dtype):
    return np.frombuffer(buffer, dtype=dtype).reshape(-1, channels)

  sd._array = sounddevice_array


def open_audio_stream(sd, factory, *, log_prefix: str, retry_delay: float = 0.5, attempts: int | None = None):
  """Best-effort open of a STARTED PortAudio stream. Returns a live stream, or None if
  bounded attempts run out. Never raises: 'audio device not ready' is an operational
  state. Re-enumerates PortAudio each attempt; logs the REAL exception (rate-limited)."""
  last_devs = None
  attempt = 0
  while attempts is None or attempt < attempts:
    attempt += 1
    # reload sounddevice to reinitialize portaudio
    sd._terminate()
    sd._initialize()
    stream = None
    try:
      stream = factory(sd)
      stream.start()  # streams open stopped; a successful start is part of being ready
      return stream
    except Exception as e:
      if stream is not None:
        try:
          stream.close()
        except Exception:
          pass
      try:
        devs = [(d['name'], d['max_input_channels'], d['max_output_channels']) for d in sd.query_devices()]
      except Exception as e2:
        devs = f"<query_devices failed: {e2!r}>"
      if devs != last_devs:  # rate-limit to device-set changes
        cloudlog.warning(f"{log_prefix}: audio device not ready: {e!r}; devices={devs}")
        last_devs = devs
      time.sleep(retry_delay)
  return None


@cache
def get_a_weighting_filter():
  # Calculate the A-weighting filter
  # https://en.wikipedia.org/wiki/A-weighting
  freqs = np.fft.fftfreq(FFT_SAMPLES, d=1 / SAMPLE_RATE)
  A = 12194 ** 2 * freqs ** 4 / ((freqs ** 2 + 20.6 ** 2) * (freqs ** 2 + 12194 ** 2) * np.sqrt((freqs ** 2 + 107.7 ** 2) * (freqs ** 2 + 737.9 ** 2)))
  return A / np.max(A)


def calculate_spl(measurements):
  # https://www.engineeringtoolbox.com/sound-pressure-d_711.html
  sound_pressure = np.sqrt(np.mean(measurements ** 2))  # RMS of amplitudes
  if sound_pressure > 0:
    sound_pressure_level = 20 * np.log10(sound_pressure / REFERENCE_SPL)  # dB
  else:
    sound_pressure_level = 0
  return sound_pressure, sound_pressure_level


def apply_a_weighting(measurements: np.ndarray) -> np.ndarray:
  # Generate a Hanning window of the same length as the audio measurements
  measurements_windowed = measurements * np.hanning(len(measurements))

  # Apply the A-weighting filter to the signal
  return np.abs(np.fft.ifft(np.fft.fft(measurements_windowed) * get_a_weighting_filter()))


class Mic:
  def __init__(self):
    self.rk = Ratekeeper(RATE)
    self.pm = messaging.PubMaster(['soundPressure', 'rawAudioData'])

    self.measurements = np.empty(0)

    self.sound_pressure = 0
    self.sound_pressure_weighted = 0
    self.sound_pressure_level_weighted = 0

    self.lock = threading.Lock()

  def update(self):
    with self.lock:
      sound_pressure = self.sound_pressure
      sound_pressure_weighted = self.sound_pressure_weighted
      sound_pressure_level_weighted = self.sound_pressure_level_weighted

    msg = messaging.new_message('soundPressure', valid=True)
    msg.soundPressure.soundPressure = float(sound_pressure)
    msg.soundPressure.soundPressureWeighted = float(sound_pressure_weighted)
    msg.soundPressure.soundPressureWeightedDb = float(sound_pressure_level_weighted)

    self.pm.send('soundPressure', msg)
    self.rk.keep_time()

  def callback(self, indata, frames, time, status):
    """
    Using amplitude measurements, calculate an uncalibrated sound pressure and sound pressure level.
    Then apply A-weighting to the raw amplitudes and run the same calculations again.

    Logged A-weighted equivalents are rough approximations of the human-perceived loudness.
    """
    msg = messaging.new_message('rawAudioData', valid=True)
    audio_data_int_16 = (indata[:, 0] * 32767).astype(np.int16)
    msg.rawAudioData.data = audio_data_int_16.tobytes()
    msg.rawAudioData.sampleRate = SAMPLE_RATE
    self.pm.send('rawAudioData', msg)

    with self.lock:
      self.measurements = np.concatenate((self.measurements, indata[:, 0]))

      while self.measurements.size >= FFT_SAMPLES:
        measurements = self.measurements[:FFT_SAMPLES]

        self.sound_pressure, _ = calculate_spl(measurements)
        measurements_weighted = apply_a_weighting(measurements)
        self.sound_pressure_weighted, self.sound_pressure_level_weighted = calculate_spl(measurements_weighted)

        self.measurements = self.measurements[FFT_SAMPLES:]

  def micd_thread(self):
    # sounddevice must be imported after forking processes
    import sounddevice as sd
    patch_sounddevice(sd)

    stream = None
    while True:
      if stream is None:
        stream = open_audio_stream(
          sd, lambda s: s.InputStream(channels=1, samplerate=SAMPLE_RATE, callback=self.callback, blocksize=SAMPLE_BUFFER),
          log_prefix="micd")
        if stream is None:
          continue
        cloudlog.info(f"micd stream started: {stream.samplerate=} {stream.channels=} {stream.dtype=} {stream.device=}, {stream.blocksize=}")

      # publish while the stream is alive; if it dies mid-run, drop it and re-acquire
      try:
        while stream.active:
          self.update()
        cloudlog.error("micd stream inactive, re-acquiring")
      except Exception as e:
        cloudlog.error(f"micd stream lost, re-acquiring: {e!r}")
      try:
        stream.close()  # release the device promptly before re-opening
      except Exception:
        pass
      stream = None
      time.sleep(0.5)  # bound the re-acquire rate if the stream opens but never stays active


def main():
  mic = Mic()
  mic.micd_thread()


if __name__ == "__main__":
  main()
