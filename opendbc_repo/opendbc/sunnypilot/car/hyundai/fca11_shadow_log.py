"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

0045 (v2, settlement Amendment 1): FCA11 SHADOW writer -- ``fca11-shadow.jsonl``.

TERMS: OLD-LAW = the currently deployed FCA11 law (what the car does today; in Shadow it is what is ON THE WIRE, exactly as
deployed, when HyundaiFca11Brake is ON; with Braking OFF nothing is sent, as today). V3 = the shaped candidate law (0043 v3):
computed every frame AFTER the send and logged here, NEVER sent. FCA11-DISABLED = our stack fully off (a debug-only
configuration; never produced by the Shadow flag).

WHY THIS EXISTS
The FCA11 settlement BLOCKED actuating V3 (it trades braking authority for smoothness on a safety tail the sim plant
cannot confirm or clear) and AUTHORIZED a SHADOW deploy: the car keeps today's actuation (OLD-LAW) and the build logs what
V3 WOULD have commanded alongside what OLD-LAW DID command. Records are PRIMITIVES only (asks, speeds, both laws' levels,
camera request, latch/state flags, lead) so any authority-floor design can be derived OFFLINE; no floor formula is here.

FORMAT (one JSON object per line, ``sort_keys``; every record has ``"k"`` = record kind and ``"schema"`` = 2):
  k="hdr"    once per part: schema, t_wall, oldlaw_armed, law constants, note. ``cont`` (= part number, >= 2) marks a
             CONTINUATION part — a long drive that rotated at the byte bound; the first part has no ``cont`` and the
             remaining fields are identical to it (a fresh t_wall).
  k="f"      one FRAME record per 50 Hz send slot while the shadow window is OPEN (see ``ShadowLogger.frame``):
    timestamps   t_wall (wall clock, s), t_mono_ns (the car layer's now_nanos = card's monotonic clock), frame (100 Hz
                 controller frame), t_since_engage (s; null when disengaged)
    ego / ask    v_ego_mps, a_ego_mps2, ask_mps2 (raw planner accel = CC.actuators.accel), ask_law_mps2 (the ask V3
                 consumed: clipped to -FCA11_A_MAX_MSS2, the actuating FCA11 PID bound), standstill, gas_pressed,
                 brake_pressed, engaged, long_active
    camera       camera_requesting (bool), camera_request_level (the camera's own CR_VSM_DecCmd raw LSB, byte 1 of the
                 freshest bus-2 0x38D; null with no frame), camera_warn (CF_VSM_Warn bits), cam_hex (raw camera frame)
    OLD-LAW      oldlaw_armed (HyundaiFca11Brake ON this ignition), oldlaw_cmd_lsb (the CR_VSM_DecCmd OLD-LAW actually put
                 ON THE WIRE this frame; 0 = no frame / passive), oldlaw_send (a host 0x38D went out this frame),
                 oldlaw_braking, oldlaw_instant_lsb (the unshaped instant OLD-LAW mapping of the same ask:
                 clamp_dec_cmd(dec_cmd_from_accel(ask)), 0 above the onset ask - a pure function, for reference)
    V3           v3_ok (False once V3 was disabled by a fault -> the v3_* values are null), v3_wouldbe_lsb (V3's would-be
                 SENT CR_VSM_DecCmd this frame; 0 = passive/none), v3_send_wouldbe, v3_target_lsb (the V3 level target
                 before slew/hold/+4 clamps), v3_braking, held_band, latched (blocked_until_resume), hc_unavailable (V3
                 core's 0042 fca11_unavailable), hard_cut (0053: the V3 core's cam_brake - the camera is braking and
                 holds FCA11; name kept for schema 2 readers), pedal_cut, excl_reason (V3 core's fca11_ok why-not; null = ok;
                 "v3_disabled" after a fault), v3_alert_bits_wouldbe (1 supervise-stop | 2 stop-complete | 4 brake-now)
    lead         lead_present, lead_drel_m, lead_vrel_mps, lead_vlead_mps, lead_alead_mps2 (radarState aLeadK),
                 lead_model_prob, lead_radar - see the fca11_long.py SHADOW block for the pinned conventions:
                 vRel = vLead - vEgo (< 0 = CLOSING); no lead => lead_present False and every other lead_* null (never 0);
                 low-quality leads are NOT filtered: lead_model_prob / lead_radar are logged raw for an offline gate.
    tier_cap, personality, episode_id (V3 would-be episode; null outside one)
  k="ep"     one EPILOGUE per V3 would-be episode (V3 braking true -> false): episode_id, frame/t bounds, duration_s,
             n_frames, n_sent, v3 min/max/last, oldlaw_instant max, oldlaw_cmd max, cap-hit and hold-band frame counts,
             v start/min/end, ask min, camera_request_seen, end_reason, tier_cap.
  k="v3fail" one per V3 FAILURE (V3-fault isolation): frame, kind ("exception" | "overrun"), err, elapsed_ms, n_failures,
             n_overruns, v3_disabled. The OLD-LAW send of that frame was already on the wire (V3 runs post-send).
  k="rot"    once per part that ended at the byte bound: t_wall, part (the part being closed), bytes (its size). The
             generation chain is shifted as at construction and the drive continues in the next part (``cont``).
  k="trunc"  once if the byte bound is hit on the LAST allowed part (FCA11_SHADOW_MAX_PARTS reached: nothing further is
             written this drive).

COUNTERFACTUAL NOTE (read before analysing): the car is driven by OLD-LAW (or not braked by us at all with Braking OFF).
The V3 stream is computed OPEN-LOOP against the vehicle as it actually moved, assuming every V3 frame were accepted by the
panda (no echo verdicts exist for V3). It is NOT the closed-loop trajectory V3 would have produced.

WHERE / SIZE
Directory = the FCA11 log dir convention (fca11_log.FCA11_LOG_DIR = /data/fca11-log, env FCA11_LOG_DIR; "0"
disables I/O). File ``fca11-shadow.jsonl``. Per-drive bound: at construction the previous drive's file rotates to
``.1`` (.1 -> .2 ... up to FCA11_SHADOW_KEEP generations; the oldest is dropped) and a part writes at most
FCA11_SHADOW_MAX_BYTES. Reaching that bound ROTATES instead of truncating: the part is closed with one k="rot"
record, the same generation shift runs, and a fresh ``fca11-shadow.jsonl`` opens with the drive header plus
cont=<part>, so a long drive is a SEQUENCE of parts (bounded at FCA11_SHADOW_MAX_PARTS (= FCA11_SHADOW_KEEP, 5); once those are used up
the last part truncates exactly as before, with one k="trunc" record). Retention is FCA11_SHADOW_KEEP generations
total, so the parts of a rotating drive evict the oldest ones as it goes (MAX_PARTS == KEEP, so a drive that uses
all 8 parts keeps its newest 5 on disk). Writes are buffered and flushed ~1/s and at every episode end / V3 failure;
every OSError is swallowed (a log must never take card down).
"""
import json
import os
import time

from opendbc.sunnypilot.car.hyundai import fca11_log

FCA11_SHADOW_FILE = "fca11-shadow.jsonl"
FCA11_SHADOW_SCHEMA = 2
FCA11_SHADOW_MAX_BYTES = 25 * 1024 * 1024   # per part (a drive, or one part of a rotated long drive)
FCA11_SHADOW_KEEP = 5                        # previous drives kept (.1 newest .. .5 oldest)
FCA11_SHADOW_MAX_PARTS = FCA11_SHADOW_KEEP   # parts per drive; == KEEP so a rotating drive keeps ALL of its own parts
FCA11_SHADOW_FLUSH_RECORDS = 50              # ~1 s of 50 Hz frame records
# the shadow WINDOW (when frame records are written): open while V3 or OLD-LAW is braking, the ask is at/below the
# release threshold, or the camera is requesting; plus PRE/POST context of this many 50 Hz records (1 s each).
FCA11_SHADOW_CONTEXT_RECORDS = 50

FRAME_FIELDS = (
  "t_wall", "t_mono_ns", "frame", "t_since_engage", "v_ego_mps", "a_ego_mps2", "ask_mps2", "ask_law_mps2",
  "camera_requesting", "camera_request_level", "camera_warn", "cam_hex",
  "oldlaw_armed", "oldlaw_cmd_lsb", "oldlaw_send", "oldlaw_braking", "oldlaw_instant_lsb",
  "v3_ok", "v3_wouldbe_lsb", "v3_send_wouldbe", "v3_target_lsb", "v3_braking", "held_band", "latched", "hc_unavailable",
  "hard_cut", "pedal_cut", "excl_reason", "v3_alert_bits_wouldbe",
  "tier_cap", "personality", "engaged", "long_active", "episode_id", "standstill", "gas_pressed", "brake_pressed",
  "lead_present", "lead_drel_m", "lead_vrel_mps", "lead_vlead_mps", "lead_alead_mps2", "lead_model_prob", "lead_radar",
)
EPILOGUE_FIELDS = (
  "episode_id", "frame_start", "frame_end", "t_wall_start", "t_wall_end", "duration_s", "n_frames", "n_sent",
  "v3_min", "v3_max", "v3_last", "oldlaw_instant_max", "oldlaw_cmd_max", "cap_hit_frames", "held_band_frames",
  "v_start_mps", "v_min_mps", "v_end_mps", "ask_min_mps2", "camera_request_seen", "end_reason", "tier_cap",
)
V3FAIL_FIELDS = ("t_wall", "frame", "kind", "err", "elapsed_ms", "n_failures", "n_overruns", "v3_disabled")


def resolve_dir(dir_path=None):
  """explicit dir (tests / offline tools) > env FCA11_LOG_DIR > fca11_log.FCA11_LOG_DIR. "0" / "" = disabled."""
  if dir_path is not None:
    d = dir_path
  else:
    d = os.environ.get("FCA11_LOG_DIR", fca11_log.FCA11_LOG_DIR)
  return d if (d and d != "0") else None


class ShadowLogger:
  """Buffered JSONL writer for the shadow streams. Inert (no I/O) when the dir resolves to disabled."""

  def __init__(self, dir_path=None, header=None):
    self.dir = resolve_dir(dir_path)
    self.enabled = self.dir is not None
    self.path = os.path.join(self.dir, FCA11_SHADOW_FILE) if self.enabled else None
    self.bytes = 0
    self.part = 1
    self.truncated = False
    self.n_records = 0
    self._header = dict(header or {})   # the drive's header constants, repeated (plus cont=) by every continuation part
    self._buf: list[str] = []
    self._pre: list[dict] = []        # pre-context ring (frame records not yet written)
    self._post_left = 0
    if self.enabled:
      try:
        os.makedirs(self.dir, exist_ok=True)
        self._rotate()
      except OSError:
        self.enabled = False
    self.write(dict(k="hdr", t_wall=time.time(), **(header or {})))

  # ------------------------------------------------------------------ rotation / io
  def _rotate(self) -> None:
    for i in range(FCA11_SHADOW_KEEP - 1, 0, -1):
      src = f"{self.path}.{i}"
      if os.path.exists(src):
        os.replace(src, f"{self.path}.{i + 1}")
    if os.path.exists(self.path):
      os.replace(self.path, f"{self.path}.1")

  def _rotate_at_cap(self) -> bool:
    """The byte cap was reached: close the current part with one k="rot" record, shift the generation chain exactly as
    at construction, and keep writing in a fresh part whose first record is the same header plus cont=<part>.
    False — the caller truncates instead — once FCA11_SHADOW_MAX_PARTS parts are used up or the chain cannot be moved."""
    if self.part >= FCA11_SHADOW_MAX_PARTS:
      return False
    self._buf.append(json.dumps(dict(k="rot", schema=FCA11_SHADOW_SCHEMA, t_wall=time.time(), part=self.part,
                                     bytes=self.bytes), sort_keys=True, separators=(",", ":")) + "\n")
    self.flush()
    try:
      self._rotate()
    except OSError:
      return False
    self.part += 1
    self.bytes = 0
    # the continuation header is written directly (never dropped, never capped): a part must open with a k="hdr"
    self._emit(json.dumps(dict(k="hdr", schema=FCA11_SHADOW_SCHEMA, t_wall=time.time(), cont=self.part, **self._header),
                          sort_keys=True, separators=(",", ":")) + "\n")
    return True

  def _emit(self, line: str) -> None:
    self.bytes += len(line)
    self.n_records += 1
    self._buf.append(line)
    if len(self._buf) >= FCA11_SHADOW_FLUSH_RECORDS:
      self.flush()

  def write(self, rec: dict) -> None:
    if not self.enabled or self.truncated:
      return
    rec.setdefault("schema", FCA11_SHADOW_SCHEMA)
    line = json.dumps(rec, sort_keys=True, separators=(",", ":")) + "\n"
    if self.bytes + len(line) > FCA11_SHADOW_MAX_BYTES and not self._rotate_at_cap():
      self.truncated = True
      self._buf.append(json.dumps(dict(k="trunc", schema=FCA11_SHADOW_SCHEMA, t_wall=time.time(), bytes=self.bytes),
                                  sort_keys=True) + "\n")
      self.flush()
      return
    self._emit(line)

  def flush(self) -> None:
    if not self._buf:
      return
    buf, self._buf = self._buf, []
    if not self.enabled:
      return
    try:
      with open(self.path, "a") as f:
        f.write("".join(buf))
    except OSError:
      pass

  # ------------------------------------------------------------------ windowed frame records
  def frame(self, rec: dict, window_open: bool) -> None:
    """Write a k="f" record if the shadow window is open (with 1 s pre/post context); otherwise keep it in the ring."""
    rec = dict(rec, k="f")
    if window_open:
      for r in self._pre:
        self.write(r)
      self._pre = []
      self.write(rec)
      self._post_left = FCA11_SHADOW_CONTEXT_RECORDS
    elif self._post_left > 0:
      self.write(rec)
      self._post_left -= 1
    else:
      self._pre.append(rec)
      if len(self._pre) > FCA11_SHADOW_CONTEXT_RECORDS:
        self._pre.pop(0)

  def epilogue(self, rec: dict) -> None:
    self.write(dict(rec, k="ep"))
    self.flush()
