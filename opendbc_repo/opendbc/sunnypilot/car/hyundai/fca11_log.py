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

# PLANT-CURVE PLATEAUS (instrumentation only). The brake-plant curve (DecCmd LSB -> delivered m/s^2) is measured at
# 4/8/13/20 LSB from a handful of runs and has NEVER been observed above 20 LSB while moving; the stock camera only
# brakes in a real AEB event, so no existing log can fill the top of the curve. Every episode therefore records its
# sustained SENT-level plateaus: maximal windows where the level put on the wire is constant for >= PLATEAU_MIN_S
# while v_ego > PLATEAU_V_MIN_MPS and neither pedal is pressed. Mean v / aEgo are taken over the STEADY part (the
# first PLATEAU_SETTLE_S of the window is skipped: ESC lag + the transient of the step into the level). Repeats of the
# same level in one episode are MERGED (sample-weighted), and at most PLATEAU_MAX distinct levels are kept, so a record
# stays one bounded line. Nothing here is read by any send decision.
PLATEAU_MIN_S = 0.6
PLATEAU_SETTLE_S = 0.3
PLATEAU_V_MIN_MPS = 4.0
PLATEAU_MAX = 12
_DT = 0.01  # controller frame period (100 Hz); plateau times are frame-derived so they are deterministic


class PlateauTracker:
  """Accumulates the sustained constant-SENT-level windows of ONE episode. Fed one sample per SENT frame."""

  def __init__(self):
    self.reset()

  def reset(self) -> None:
    self._lsb = None          # level of the open window (None = no window)
    self._f0 = 0              # first frame of the open window
    self._f_last = 0          # last frame sampled into the open window
    self._gap = 0             # max frame step tolerated inside a window (the send cadence)
    self._n = 0               # steady samples
    self._sv = 0.0            # sum v over the steady part
    self._sa = 0.0            # sum aEgo over the steady part
    self.plateaus: list[dict] = []   # merged, insertion-ordered by first occurrence
    self.dropped = 0          # distinct levels beyond PLATEAU_MAX (counted, not stored)

  def sample(self, frame: int, lsb: int, v_ego: float, a_ego, gas: bool, brake: bool, max_gap: int = 2) -> None:
    frame = int(frame)
    qualifies = (a_ego is not None and int(lsb) >= 1 and float(v_ego) > PLATEAU_V_MIN_MPS
                 and not gas and not brake)
    if self._lsb is not None and (not qualifies or int(lsb) != self._lsb or frame - self._f_last > max_gap
                                  or frame <= self._f_last):
      self.close()
    if not qualifies:
      return
    if self._lsb is None:
      self._lsb, self._f0, self._n, self._sv, self._sa = int(lsb), frame, 0, 0.0, 0.0
    self._f_last = frame
    if (frame - self._f0) * _DT >= PLATEAU_SETTLE_S - 1e-9 and a_ego is not None:
      self._n += 1
      self._sv += float(v_ego)
      self._sa += float(a_ego)

  def close(self) -> None:
    """End the open window; keep it if it lasted >= PLATEAU_MIN_S."""
    lsb, dur, n = self._lsb, (self._f_last - self._f0) * _DT, self._n
    sv, sa = self._sv, self._sa
    self._lsb = None
    if lsb is None or dur < PLATEAU_MIN_S - 1e-9 or n == 0:
      return
    for p in self.plateaus:
      if p["lsb"] == lsb:
        p["_sv"] += sv
        p["_sa"] += sa
        p["n"] += n
        p["dur_s"] += dur
        p["runs"] += 1
        return
    if len(self.plateaus) >= PLATEAU_MAX:
      self.dropped += 1
      return
    self.plateaus.append({"lsb": lsb, "dur_s": dur, "n": n, "runs": 1, "_sv": sv, "_sa": sa})

  def record(self) -> list[dict]:
    """The bounded, rounded list for the episode line (closes any open window first)."""
    self.close()
    return [{"lsb": p["lsb"], "dur_s": round(p["dur_s"], 2), "v_mps": round(p["_sv"] / p["n"], 2),
             "a_mss2": round(p["_sa"] / p["n"], 3), "n": p["n"], "runs": p["runs"]} for p in self.plateaus]


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
    self.plateau = PlateauTracker()

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

  def note_sent(self, frame: int, lsb: int, v_ego: float, a_ego, gas: bool, brake: bool, max_gap: int = 2) -> None:
    """One SENT actuating frame (the exact level built into the 0x38D) -> the plateau tracker. Logging only: a fault
    here must never reach the controller, so every error is swallowed."""
    if not self._open:
      return
    try:
      self.plateau.sample(frame, lsb, v_ego, a_ego, gas, brake, max_gap)
    except Exception:  # noqa: BLE001 - instrumentation must never take the law down
      pass

  def note_refusal(self, why: str) -> None:
    """A car-layer gate refused this frame (fca11_ok's reason code). Counted only while an episode is open."""
    if self._open and why:
      self._refused[why] = self._refused.get(why, 0) + 1

  def note_panda_refusal(self, n: int = 1) -> None:
    """The panda's own TX hook refused an actuating 0x38D we sent (pandad echo src==192). Ground truth."""
    if self._open:
      self._panda_refused += n

  # ------------------------------------------------------------------ record + write
  def episode_record(self, frame: int, reason: str, stop_kph: float, cont_low_lsb_frames: int = 0,
                     stop_hold_frames: int = 0, stop_hold_blockers: dict | None = None) -> dict | None:
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
      # 0037/M3: SENT frames at 1-3 LSB in THIS episode - the low end the law's plant inverse EXTENDS below the
      # measured 4 LSB cliff. Its delivered decel is unmeasured; the same episode's `plateaus` carry the measured
      # aEgo for every sustained level (including 1-3 LSB), which is what makes the first drive able to falsify it.
      "cont_low_lsb_frames": int(cont_low_lsb_frames),
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
      # 0039: the stop-hold clutch lane's own census. `stop_hold_frames` = SENT frames that carried the HBA/StopReq pair;
      # `stop_hold_blockers` = {first failing clause: count} for the SENT frames that did not, so the next drive answers
      # "why was the pair never sent" from the log instead of a source read (route 195: 0 pairs, `lane_off` on 399).
      "stop_hold_frames": int(stop_hold_frames),
      "stop_hold_blockers": dict(sorted((stop_hold_blockers or {}).items())),
      "plateaus": self._plateau_record(),
      "plateaus_dropped": self.plateau.dropped,
      "stop_kph": stop_kph,
      "frame": frame,
    }

  def _plateau_record(self) -> list:
    try:
      return self.plateau.record()
    except Exception:  # noqa: BLE001 - a malformed sample must not cost the whole episode line
      return []

  def flush(self, frame: int, reason: str, stop_kph: float, cont_low_lsb_frames: int = 0,
            stop_hold_frames: int = 0, stop_hold_blockers: dict | None = None) -> None:
    rec = self.episode_record(frame, reason, stop_kph, cont_low_lsb_frames, stop_hold_frames, stop_hold_blockers)
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
