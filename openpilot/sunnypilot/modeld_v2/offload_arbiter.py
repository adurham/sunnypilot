"""OFFLOAD §7 — device-side output arbitration (INTERFACES.md §7; contract.SHADOW_OUTPUT_SERVICES).

The device modeld_v2 always computes AND publishes its own model outputs. When the remote (Mac)
model's outputs are *continuously* fresh, this arbiter lets modeld_v2 publish the REMOTE outputs
under the REAL service names instead — a dead-man-switch on the big model: the big model must prove
liveness every frame to be selected, and the local model is the default that never stops.

Default off / byte-identical: nothing here runs unless explicitly enabled — OFFLOAD_ARBITRATION
truthy (bench) or (device: COMMA_HARDWARE and Params OffloadMode == 'drive'). When disabled,
``make_arbiter()`` returns None, no socket is opened, and modeld_v2's publish path is untouched.

Transport (see device/offloadd.py, the sender; device/RESULTS.md "cereal service-name validation"):
the four SHADOW msgq topics are read with RAW msgq sockets because those names have no cereal
``log.Event`` union member, so a SubMaster/new_message cannot construct on them. The wire payload's
union member is the REAL one (modelV2 / cameraOdometry / drivingModelData / modelDataV2SP), so
``log.Event.from_bytes`` decodes it normally.

Cohesion: a frame's four messages are published from ONE source. A remote set is usable only when
all four pieces are cached. ``modelDataV2SP`` carries no frameId (custom.ModelDataV2SP), so it is
bound to a frame by arrival proximity (<= OFFLOAD_ARB_SP_PROXIMITY_MS, default 20 ms) to that
frame's modelV2 arrival — a documented choice (ARBITRATION.md).

Restamp semantics: only the Event header ``logMonoTime`` of each forwarded message is rewritten to
device-now (``as_builder()`` on the reader, one scalar write). Payload bytes are preserved exactly.
"""
from __future__ import annotations

from collections import deque

from openpilot.common.hardware import COMMA_HARDWARE
from openpilot.offload.contract import (
  ELIGIBLE_MS,
  ENTER_N,
  EXIT_N,
  SETTLE_N,
  SHADOW_OUTPUT_SERVICES,
)

# Shadow topic -> real service name (the name the device must publish under when selected).
SHADOW_TO_REAL = {
  "offloadModelV2": "modelV2",
  "offloadCameraOdometry": "cameraOdometry",
  "offloadDrivingModelData": "drivingModelData",
  "offloadModelDataV2SP": "modelDataV2SP",
}
SHADOW_NAMES = tuple(SHADOW_TO_REAL)
REAL_SERVICES = ("modelV2", "drivingModelData", "cameraOdometry", "modelDataV2SP")
# Real names whose payload carries a frameId (modelDataV2SP does not).
ID_SERVICES = ("modelV2", "cameraOdometry", "drivingModelData")
assert set(SHADOW_NAMES) == set(SHADOW_OUTPUT_SERVICES), "shadow names drifted from contract.SHADOW_OUTPUT_SERVICES"

DEFAULT_SP_PROXIMITY_MS = 20   # OFFLOAD_ARB_SP_PROXIMITY_MS — modelDataV2SP bind window
DEFAULT_PURGE_S = 2.0          # OFFLOAD_ARB_PURGE_S — drop cached frames older than this
DEFAULT_MAX_FRAMES = 8         # keep only the newest N cached frames
LOG_INTERVAL_NS = 1_000_000_000
_TRAV = 2 ** 64 - 1


def _env_flag(name: str, default: bool = False) -> bool:
  """Truthy env var: unset/empty -> ``default``; 0/false/no/off -> False; else True."""
  import os
  raw = os.getenv(name)
  if raw is None or raw == "":
    return default
  return raw.strip().lower() not in ("0", "false", "no", "off")


def _env_int(name: str, default: int) -> int:
  import os
  raw = os.getenv(name)
  if raw is None or raw == "":
    return default
  try:
    return int(float(raw))
  except ValueError:
    return default


def _env_float(name: str, default: float) -> float:
  import os
  raw = os.getenv(name)
  if raw is None or raw == "":
    return default
  try:
    return float(raw)
  except ValueError:
    return default


def is_arbiter_enabled(params=None) -> bool:
  """OFFLOAD §7 gate. True iff OFFLOAD_ARBITRATION is truthy, or device drive mode.

  Device path: COMMA_HARDWARE and the ``OffloadMode`` param == 'drive'. Params access is
  defensive (UnknownKeyName / any backend error -> disabled).
  """
  if _env_flag('OFFLOAD_ARBITRATION', False):
    return True
  if not COMMA_HARDWARE:
    return False
  try:
    if params is None:
      from openpilot.common.params import Params
      params = Params()
    mode = params.get("OffloadMode")
    if mode is None:
      return False
    if isinstance(mode, bytes):
      mode = mode.decode(errors="ignore")
    return str(mode).strip().lower() == "drive"
  except Exception:
    return False


def make_arbiter(params=None):
  """Construct the arbiter when enabled; return None (no sockets, no behavior) otherwise."""
  if not is_arbiter_enabled(params):
    return None
  return OffloadArbiter()


def _cloudlog():
  from openpilot.common.swaglog import cloudlog
  return cloudlog


def _parse(raw: bytes):
  """Return (which, frame_id) for a raw Event, or (None, None) on any decode failure."""
  from openpilot.cereal import log
  try:
    with log.Event.from_bytes(raw, traversal_limit_in_words=_TRAV) as reader:
      which = reader.which()
      msg = getattr(reader, which, None)
      fid = int(msg.frameId) if (msg is not None and hasattr(msg, "frameId")) else None
      return which, fid
  except Exception:
    return None, None


def restamp(raw: bytes, now_ns: int) -> bytes:
  """Return ``raw`` with the Event header ``logMonoTime`` set to ``now_ns``; payload untouched."""
  from openpilot.cereal import log
  with log.Event.from_bytes(raw, traversal_limit_in_words=_TRAV) as reader:
    b = reader.as_builder()
    b.logMonoTime = int(now_ns)
    return b.to_bytes()


class _FrameSet:
  """The four remote messages of one device frame, plus their arrival times (device clock ns)."""

  __slots__ = ("frame_id", "pieces", "arrivals", "sp_raw", "sp_arrival")

  def __init__(self, frame_id: int):
    self.frame_id = frame_id
    self.pieces: dict[str, bytes] = {}    # real name -> raw shadow payload bytes
    self.arrivals: dict[str, int] = {}
    self.sp_raw: bytes | None = None      # modelDataV2SP, bound by proximity
    self.sp_arrival: int | None = None

  def has_ids(self) -> bool:
    return all(s in self.pieces for s in ID_SERVICES)

  @property
  def arrival_ns(self) -> int:
    vals = list(self.arrivals.values())
    if self.sp_arrival is not None:
      vals.append(self.sp_arrival)
    return max(vals) if vals else 0


class OffloadArbiter:
  """§7 state machine + raw-msgq shadow reader. One instance per modeld_v2 process, OFFLOAD-gated.

  Lifecycle: construct -> ``poll(now_ns)`` once per model loop (drains the shadows, 20 Hz) ->
  ``select(local_frame_id, now_ns)`` at the publish point (returns {real_name: restamped bytes}
  to publish, or None to publish the local outputs). Tests may drive it with ``ingest()`` directly.
  """

  def __init__(self, *, eligible_ms: int | None = None, enter_n: int | None = None,
               exit_n: int | None = None, settle_n: int | None = None,
               sp_proximity_ms: int | None = None, purge_s: float | None = None,
               max_frames: int = DEFAULT_MAX_FRAMES):
    self.eligible_ns = int(ELIGIBLE_MS if eligible_ms is None else eligible_ms) * 1_000_000
    self.enter_n = int(ENTER_N if enter_n is None else enter_n)
    self.exit_n = int(EXIT_N if exit_n is None else exit_n)
    self.settle_n = int(SETTLE_N if settle_n is None else settle_n)
    self.sp_proximity_ns = int(DEFAULT_SP_PROXIMITY_MS if sp_proximity_ms is None else sp_proximity_ms) * 1_000_000
    self.purge_s = float(DEFAULT_PURGE_S if purge_s is None else purge_s)
    self.max_frames = int(max_frames)

    self.frames: dict[int, _FrameSet] = {}
    self.sp_buffer: deque[tuple[bytes, int]] = deque()
    self._subs: dict | None = None

    # §7 state machine
    self.engaged = False
    self.eligible_streak = 0
    self.ineligible_streak = 0
    self.settle_remaining = 0
    self.engage_count = 0
    self.disengage_count = 0
    self.settle_skipped = 0
    self.eligible_frames = 0
    self.ineligible_frames = 0
    self.published_remote = 0
    self.published_local = 0
    self.last_remote_frame_id: int | None = None
    self._last_log_ns: int | None = None

  # -- sockets ----------------------------------------------------------------
  def open(self) -> None:
    """Lazily open the four raw msgq shadow subscriptions (no-op once open)."""
    if self._subs is not None:
      return
    import msgq
    from openpilot.cereal.services import SERVICE_LIST
    subs = {}
    for name in SHADOW_NAMES:
      svc = SERVICE_LIST.get(name)
      seg = svc.queue_size if svc is not None else 0
      subs[name] = msgq.sub_sock(name, addr="127.0.0.1", timeout=0, segment_size=seg)
    self._subs = subs

  def close(self) -> None:
    self._subs = None

  # -- ingest -----------------------------------------------------------------
  def poll(self, now_ns: int) -> None:
    """Drain every shadow socket (non-blocking) and purge stale cache. Call once per loop."""
    self.open()
    subs = self._subs or {}
    for name, sock in subs.items():
      while True:
        d = sock.receive(non_blocking=True)
        if d is None:
          break
        self.ingest(name, bytes(d), now_ns)
    self._purge(now_ns)

  def ingest(self, shadow_name: str, raw: bytes, arrival_ns: int) -> None:
    """Cache one raw shadow message under its real name. ``arrival_ns`` is device-monotonic."""
    real = SHADOW_TO_REAL.get(shadow_name)
    if real is None:
      return
    which, fid = _parse(raw)
    if real == "modelDataV2SP":
      self.sp_buffer.append((raw, arrival_ns))
      return
    if which != real or fid is None:
      return
    fs = self.frames.get(fid)
    if fs is None:
      fs = _FrameSet(fid)
      self.frames[fid] = fs
    fs.pieces[real] = raw
    fs.arrivals[real] = arrival_ns

  def _resolve_sp(self, fs: _FrameSet) -> bytes | None:
    """Bind a buffered modelDataV2SP to ``fs`` by proximity to its modelV2 arrival (<= window)."""
    if fs.sp_raw is not None:
      return fs.sp_raw
    anchor = fs.arrivals.get("modelV2")
    if anchor is None:
      return None
    best = None
    for raw, arr in self.sp_buffer:
      d = abs(arr - anchor)
      if d <= self.sp_proximity_ns and (best is None or d < best[0]):
        best = (d, raw, arr)
    if best is not None:
      fs.sp_raw = best[1]
      fs.sp_arrival = best[2]
      return fs.sp_raw
    return None

  def _complete(self, fs: _FrameSet) -> bool:
    return fs.has_ids() and self._resolve_sp(fs) is not None

  def _purge(self, now_ns: int) -> None:
    cutoff = now_ns - int(self.purge_s * 1e9)
    for fid in [fid for fid, fs in self.frames.items() if fs.arrival_ns < cutoff]:
      del self.frames[fid]
    if len(self.frames) > self.max_frames:
      for fid in sorted(self.frames)[:len(self.frames) - self.max_frames]:
        del self.frames[fid]
    while self.sp_buffer and self.sp_buffer[0][1] < cutoff:
      self.sp_buffer.popleft()

  def _candidate(self, local_frame_id: int) -> _FrameSet | None:
    """Newest complete remote set at or behind the frame being published (no duplicate frames)."""
    for fid in sorted(self.frames, reverse=True):
      if fid > local_frame_id:
        continue
      if self.last_remote_frame_id is not None and fid <= self.last_remote_frame_id:
        continue
      fs = self.frames[fid]
      if self._complete(fs):
        return fs
    return None

  # -- decision ---------------------------------------------------------------
  def select(self, local_frame_id: int, now_ns: int) -> dict[str, bytes] | None:
    """§7 decision for the frame being published. Returns real-name -> restamped bytes, or None.

    None means "publish the local outputs" (the default, always correct path).
    """
    cand = self._candidate(local_frame_id)
    eligible = False
    if cand is not None:
      age = now_ns - cand.arrival_ns
      eligible = 0 <= age <= self.eligible_ns

    # settling gate: only right after an engage, do not republish a set > SETTLE_N frames behind.
    settled = True
    if eligible and self.engaged and self.settle_remaining > 0 and cand.frame_id < local_frame_id - self.settle_n:
      settled = False

    if eligible and settled:
      self.eligible_frames += 1
      self.eligible_streak += 1
      self.ineligible_streak = 0
    else:
      self.ineligible_frames += 1
      self.ineligible_streak += 1
      self.eligible_streak = 0
      if eligible and not settled:
        self.settle_skipped += 1

    if not self.engaged and self.eligible_streak >= self.enter_n:
      self.engaged = True
      self.engage_count += 1
      self.settle_remaining = self.settle_n
      # a just-engaged frame still has to satisfy settling
      if eligible and cand is not None and cand.frame_id < local_frame_id - self.settle_n:
        settled = False
        self.settle_skipped += 1
      self._log_event(now_ns, "engaged")
    elif self.engaged and self.ineligible_streak >= self.exit_n:
      self.engaged = False
      self.disengage_count += 1
      self._log_event(now_ns, "disengaged")

    use_remote = self.engaged and eligible and settled and cand is not None
    if self.engaged and self.settle_remaining > 0:
      self.settle_remaining -= 1

    self._log_counters(now_ns)
    if use_remote:
      out = self._restamped(cand, now_ns)
      if out is not None:
        self.published_remote += 1
        self.last_remote_frame_id = cand.frame_id
        return out
    self.published_local += 1
    return None

  # ``decide`` is the §7 spec's name for the publish-point decision.
  decide = select

  def _restamped(self, fs: _FrameSet, now_ns: int) -> dict[str, bytes] | None:
    out: dict[str, bytes] = {}
    for real in REAL_SERVICES:
      raw = fs.sp_raw if real == "modelDataV2SP" else fs.pieces.get(real)
      if raw is None:
        return None
      out[real] = restamp(raw, now_ns)
    return out

  # -- observability ----------------------------------------------------------
  @property
  def switches(self) -> int:
    return self.engage_count + self.disengage_count

  def stats(self) -> dict:
    return {
      "eligible": self.eligible_frames,
      "ineligible": self.ineligible_frames,
      "engaged": self.engaged,
      "switches": self.switches,
      "settle_skipped": self.settle_skipped,
      "published_remote": self.published_remote,
      "published_local": self.published_local,
    }

  def _log_event(self, now_ns: int, kind: str) -> None:
    _cloudlog().info("OFFLOAD §7 arbitration %s: frame=%s switches=%d eligible=%d ineligible=%d",
                     kind, self.last_remote_frame_id, self.switches, self.eligible_frames, self.ineligible_frames)

  def _log_counters(self, now_ns: int) -> None:
    if self._last_log_ns is not None and now_ns - self._last_log_ns < LOG_INTERVAL_NS:
      return
    self._last_log_ns = now_ns
    _cloudlog().info("OFFLOAD §7 arbitration: eligible=%d ineligible=%d engaged=%s switches=%d settle_skipped=%d",
                     self.eligible_frames, self.ineligible_frames, self.engaged, self.switches, self.settle_skipped)
