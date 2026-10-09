"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

0040: PER-EPISODE FCA11 braking logger.

WHY THIS EXISTS
The owner removed the self-imposed speed floors (panda 9 km/h, car layer 12 km/h) precisely so the ESC can be asked
to brake down to and through zero. We do NOT know whether the ESC accepts sub-12 km/h commands or holds at a full
stop, because OUR floor stopped us from ever asking. This module records, per BRAKING EPISODE (one compact JSON line,
never per-frame spam), exactly what we commanded and what the car/ESC did, so after a drive we can answer from the logs
alone: "did the ESC accept sub-12 km/h commands, did it hold, and for how long".

WHAT ONE RECORD HOLDS (see ``episode_record``)
  * commanded DecCmd: min / max / last (raw CR_VSM_DecCmd LSB, 0.01 g/LSB), and the frame count.
  * speed: vEgo-derived km/h at start / lowest / end, and the raw slowest-wheel km/h at start / lowest / end when the
    car layer published it (``CS.fca11_wheel_min_kph``; the panda's own raw WHL_SPD11 observable, best-effort).
  * stop/hold: whether a full stop was reached while FCA11 was still commanding, and how many frames / seconds it was
    held at/near zero (v <= FCA11_STOP_KPH).
  * delivered decel: min aEgo (m/s^2) over the final second of the episode - the physics that answers "did the ESC
    actually brake".
  * refusals: a histogram of OUR car-layer gate refusals by reason code, PLUS the count of frames the PANDA itself
    refused (the pandad echo src==192 of an actuating 0x38D) - the firmware's own verdict, which is the ground truth.

WHERE IT LANDS ON THE DEVICE
``/data/fca11-log/episodes.jsonl`` (one JSON object per line, appended). Overridable with the ``FCA11_LOG_DIR``
environment variable; ``FCA11_LOG_DIR=0`` disables the logger entirely (used by tests and by anyone who wants the
feature byte-identical to pre-0040). The directory is created once at construction. The file is rotated to
``episodes.jsonl.1`` past FCA11_LOG_MAX_BYTES so a long-running card cannot fill /data.

REALTIME NOTE
The write is a single, best-effort ``open(...,"a")`` + one ``write`` on the braking true -> false edge (the same
cadence and the same try/except-OSError discipline as cal_mode.py's ``reps.jsonl``). It is NOT on the per-frame path.
A log write must never take card down, so every failure is swallowed.
"""
import json
import os
import time

FCA11_LOG_DIR = "/data/fca11-log"
FCA11_LOG_EPISODES = "episodes.jsonl"
FCA11_LOG_MAX_BYTES = 2 * 1024 * 1024  # rotate past 2 MB so /data cannot fill


class EpisodeLogger:
  """Accumulates one FCA11 braking episode and writes ONE JSON line when the episode ends.

  Inert (no file I/O, no accumulation) when ``dir_path`` is falsy or ``FCA11_LOG_DIR=0``. Ticked only from the car
  layer's FCA11 controller (opendbc/sunnypilot/car/hyundai/fca11_long.py), never from the panda.
  """

  def __init__(self, dir_path: str | None = None):
    d = FCA11_LOG_DIR if dir_path is None else dir_path
    d = os.environ.get("FCA11_LOG_DIR", d)
    self.enabled = bool(d) and d != "0"
    self.dir = d if self.enabled else None
    self.path = os.path.join(self.dir, FCA11_LOG_EPISODES) if self.enabled else None
    self._makedirs()
    self.reset()

  def _makedirs(self) -> None:
    if not self.enabled:
      return
    try:
      os.makedirs(self.dir, exist_ok=True)
    except OSError:
      self.enabled = False
      self.dir = None
      self.path = None

  # ------------------------------------------------------------------ episode state
  def reset(self) -> None:
    self._open = False
    self._t0 = 0.0
    self._frames = 0
    self._dec_min = 0
    self._dec_max = 0
    self._dec_last = 0
    self._v_start = None
    self._v_min = None
    self._v_end = None
    self._w_start = None
    self._w_min = None
    self._w_end = None
    self._reached_stop = False
    self._hold_frames = 0
    self._aego_final = None
    self._aego_final_frame = -10**9
    self._refused: dict[str, int] = {}
    self._panda_refused = 0

  # ------------------------------------------------------------------ accumulation
  def begin(self, frame: int, v_kph: float, wheel_kph: float | None) -> None:
    self.reset()
    self._open = True
    self._t0 = time.time()
    self._v_start = v_kph
    self._v_min = v_kph
    self._w_start = wheel_kph
    self._w_min = wheel_kph
    self._aego_final_frame = frame

  def note_frame(self, frame: int, dec_cmd: int, v_kph: float, wheel_kph: float | None,
                 aego: float | None, standstill: bool, stop_kph: float) -> None:
    """One 100 Hz frame that the car layer COMMANDED an actuating FCA11 frame (dec_cmd >= 1)."""
    if not self._open:
      self.begin(frame, v_kph, wheel_kph)
    self._frames += 1
    self._dec_min = dec_cmd if self._dec_min == 0 else min(self._dec_min, dec_cmd)
    self._dec_max = max(self._dec_max, dec_cmd)
    self._dec_last = dec_cmd
    self._v_min = v_kph if self._v_min is None else min(self._v_min, v_kph)
    self._v_end = v_kph
    if wheel_kph is not None:
      self._w_min = wheel_kph if self._w_min is None else min(self._w_min, wheel_kph)
      self._w_end = wheel_kph
    if v_kph <= stop_kph or standstill:
      self._reached_stop = True
      self._hold_frames += 1
    if aego is not None:
      self._aego_final = float(aego) if self._aego_final is None else min(self._aego_final, float(aego))
      self._aego_final_frame = frame

  def note_refusal(self, why: str) -> None:
    """A car-layer gate refused this frame (fca11_ok's reason code). Counted only while an episode is open."""
    if self._open and why:
      self._refused[why] = self._refused.get(why, 0) + 1

  def note_panda_refusal(self, n: int = 1) -> None:
    """The panda's own TX hook refused an actuating 0x38D we sent (pandad echo src==192). Ground truth."""
    if self._open:
      self._panda_refused += n

  # ------------------------------------------------------------------ record + write
  def episode_record(self, frame: int, reason: str, stop_kph: float) -> dict | None:
    if not self._open or self._frames == 0:
      return None
    return {
      "event": "episode",
      "t": round(self._t0, 3),
      "dur_s": round(max(0.0, time.time() - self._t0), 3),
      "end_reason": reason,
      "frames": self._frames,
      "dec_cmd_min": self._dec_min,
      "dec_cmd_max": self._dec_max,
      "dec_cmd_last": self._dec_last,
      "v_kph_start": _r(self._v_start),
      "v_kph_min": _r(self._v_min),
      "v_kph_end": _r(self._v_end),
      "wheel_kph_start": _r(self._w_start),
      "wheel_kph_min": _r(self._w_min),
      "wheel_kph_end": _r(self._w_end),
      "reached_stop": bool(self._reached_stop),
      "hold_frames": self._hold_frames,
      "hold_s": round(self._hold_frames / 100.0, 3),
      "aego_min_mss2": _r(self._aego_final),
      "refused": dict(sorted(self._refused.items())),
      "refused_panda": self._panda_refused,
      "stop_kph": stop_kph,
      "frame": frame,
    }

  def flush(self, frame: int, reason: str, stop_kph: float) -> None:
    rec = self.episode_record(frame, reason, stop_kph)
    self.reset()
    if rec is not None:
      self._append(rec)

  def _append(self, rec: dict) -> None:
    if not self.enabled or self.path is None:
      return
    try:
      self._rotate_if_needed()
      with open(self.path, "a") as f:
        f.write(json.dumps(rec, sort_keys=True) + "\n")
    except OSError:
      pass

  def _rotate_if_needed(self) -> None:
    try:
      if os.path.getsize(self.path) > FCA11_LOG_MAX_BYTES:
        os.replace(self.path, self.path + ".1")
    except OSError:
      pass  # no file yet, or no permission: the append below is the best effort that matters


def _r(x):
  return None if x is None else round(float(x), 3)
