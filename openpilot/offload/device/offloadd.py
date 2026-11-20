#!/usr/bin/env python3
"""openpilot.offload.device.offloadd — DEVICE-side return-path republisher.

Status: P2 code-complete, P4-prep. **NOT activated in P1-P4 without an explicit user gate.**
It is inert unless the device param ``OffloadMode`` is ``shadow``/``drive`` (or the daemon is
run with ``--offload-mode shadow|drive`` for bench use).

What it does (normative spec = ../INTERFACES.md §1, ../contract.py):

  drive/shadow mode: the Mac computes the driving model and publishes its outputs
  (``RETURN_SERVICES``) back over ZMQ. This daemon subscribes to those, freshness-checks them
  against the DEVICE monotonic clock, re-stamps the cereal ``Event`` header ``logMonoTime`` to
  device now, and republishes into the **local** msgq so that the unchanged ``controlsd``/
  ``plannerd`` see a normal stream. No clock sync is ever performed anywhere.

Steps:
  (a) ZMQ SUB (connect) to the Mac for every RETURN_SERVICES port (Mac side binds PUB,
      device connects; ports from ``openpilot.offload.ports``).
  (b) Freshness gate per §1.3 using the device clock: track ``timestampSof`` per camera from the
      last local ``narrowRoadCameraState``. For each incoming return msg, ``sof_of_its_frame``
      is the device-clock SOF of the camera frame the model output refers to. If
      ``now_device_ns - sof > OFFLOAD_STALE_MS`` (default 300 ms) -> DROP + count (never forward).
      A msg whose payload carries any non-finite float -> DROP + count (never forward). Else ->
      keep the payload byte-identical, set header ``logMonoTime = time.monotonic_ns()``, and
      forward into local msgq under the SHADOW service name (``shadow_map``), never the real one.
      ``modelDataV2SP`` has neither frameId nor timestamps, so it cannot be aged on its own; per
      the §7 SP pairing rule it is forwarded ONLY when paired within ``OFFLOAD_SP_PAIR_MS``
      (default 25 ms) of an aged-able message that itself passed the gate (buffered in a single
      slot while it waits); an unpaired SP is dropped as ``dropped_unpaired`` — never forwarded
      blind. Other un-ageable services keep the ``dropped_no_sof`` path.
  (c) Output / staleness: the real modelV2/cameraOdometry/drivingModelData/modelDataV2SP are the
      device's OWN model's — this daemon neither publishes nor removes them. When no eligible
      remote output exists for a frame, the shadow name simply goes quiet; that absence is the
      remote's staleness signal for the modeld_v2 arbitration (INTERFACES §7), not for controlsd.
      A gap watchdog logs open/close edges for the return link only.
  (d) Stats: one JSON object per line to **stdout only**. Bench mode (``--bench``) may also append
      a local JSONL for convenience. Never writes to /data.
  (e) Mode gate: only runs when ``params.get("OffloadMode")`` in {"shadow", "drive"}; tolerates
      ``UnknownKeyName`` (key absent) by staying disabled.
  (f) SIGTERM/SIGINT -> publish nothing further, close sockets, exit 0.
  (g) HOLD/BEHIND/LOST timing policy (Jetlink borrow; OFFLOAD_HOLDPOLICY=1, default off):
      per-frame deadline HOLD (OFFLOAD_HOLD_MS, 46 ms) measured from a camera frame's first
      arrival at offloadd; 5 consecutive HOLDs or >20 HOLDs in a rolling 10 s -> BEHIND
      (hand back = stop republishing, pluggable callback); no Mac message at all for
      OFFLOAD_LOST_MS (200 ms) -> LOST. Default off preserves the pre-port behavior exactly.
      See the ported-state-machine section below.

Failure-mode table (design reference / operational rules)
---------------------------------------------------------
  | Mode                        | Detected by                          | Daemon action                    | System effect                                  |
  |-----------------------------|--------------------------------------|----------------------------------|------------------------------------------------|
  | link death (tether drops)   | ZMQ recv timeout budget exhausted    | forward nothing; open gap        | shadow stays quiet; LOCAL model still drives   |
  | bridge death (device)       | local narrowRoadCameraState stalls   | SOF map ages out -> all drops    | shadow quiet; LOCAL model still drives         |
  | Mac pipeline stall/lag      | age of return frame SOF > stale_ms   | drop + count stale (never forward)| shadow missing; arbitration sees remote stale  |
  | segment boundary / route end| cameraOdometry frameId resets/gap    | drop unmatched; keep forwarding  | brief shadow gap; loggerd unaffected           |
  | unpaired modelDataV2SP     | no aged-able msg forwarded within   | hold pending; drop + count        | SP shadow goes quiet; arbitration sees the set |
  |                            | OFFLOAD_SP_PAIR_MS (default 25 ms)  | dropped_unpaired (never blind)   | incomplete -> remote ineligible (§7 pairing)    |
  | encoder GOP gap            | no return msg for a frame id         | nothing to forward for that frame| handled upstream (WS-B); not this daemon's job |
  | ZMQ HWM drops              | SUB side: messages silently dropped  | (see note) count via recv gaps   | gap watchdog covers the symptom                |
  | non-finite remote output   | any NaN/Inf float in the payload     | drop + count nonfinite           | shadow missing; never a bad plan into msgq     |
  | echo loop (Mac sub'd to dev)| Mac SubMaster on modelV2 sees its own| MUST NOT HAPPEN                  | OPERATIONAL RULE: the Mac runner must NOT      |
  |                             | republished output                    |                                  | subscribe to modelV2 on the device ZMQ while it |
  |                             |                                       |                                  | is republishing; keep the return PUB separate   |

Ported HOLD/BEHIND/LOST state machine (Jetlink borrow; OFFLOAD_HOLDPOLICY=1, off by default)
------------------------------------------------------------------------------------------
JOB (INTERFACES §7, reframed 2026-10-07): the device's LOCAL model keeps computing AND publishing
every frame. This daemon never publishes, and never removes, the real modelV2/cameraOdometry/
drivingModelData/modelDataV2SP. Its job is to receive the remote (Mac) outputs over ZMQ,
freshness-validate them, and forward each eligible one into device msgq under the SHADOW service
names ``offloadModelV2`` / ``offloadCameraOdometry`` / ``offloadDrivingModelData`` /
``offloadModelDataV2SP`` (``Config.shadow_map``), re-stamping ONLY the header ``logMonoTime`` to
device monotonic at receipt (payload untouched). The local-vs-remote choice (arbitration,
hysteresis, settling) lives in the device modeld_v2 and is NOT this daemon's job.

Mapping from jetlink/openpilot/model_state.py (their COMMA-side gate) to ours (DEVICE side, where
the Mac publishes remotely). Their per-frame gate is ``_note_hold`` (model_state.py:321); the wire
rule is docs/link-protocol.md "Late replies":

  | Jetlink (comma)                          | OURS (device)                                            |
  |------------------------------------------|----------------------------------------------------------|
  | HOLD_FRAME=0.046 from frame warp start   | OFFLOAD_HOLD_MS=46 from frame first arrival at offloadd  |
  | held -> republish previous frame output  | (reframe) a HOLD means "no eligible remote output for this frame" |
  | HOLDS_IN_A_ROW=5                         | OFFLOAD_HOLDS_IN_A_ROW=5 (consecutive HOLDs)             |
  | HOLDS_ALLOWED=20 within HOLD_WINDOW=10 s | OFFLOAD_HOLDS_ALLOWED=20 within OFFLOAD_HOLD_WINDOW=10 s |
  | `behind` -> hand the drive to the small model | `behind` event: remote ineligible this frame; arbitration (modeld_v2) decides |
  | lost: no answer for 0.2 s (link.INFERENCE_TIMEOUT) | OFFLOAD_LOST_MS=200: no Mac message at all -> LOST     |
  | PROVING_FRAMES=20 / SETTLING_FRAMES=3    | OFFLOAD_PROVING_FRAMES=20 / OFFLOAD_SETTLING_FRAMES=3    |

Correctness rule carried over (their "never replace hidden state with a non-finite/failed
output", model_state.py:41): we forward whole messages, so the equivalent is **never forward a
stale or non-finite message**. A frame's expected remote output is matched by ``frameId`` from the
local ``narrowRoadCameraState``; eligibility uses the same 46 ms deadline. A message older than
OFFLOAD_STALE_MS at receipt is dropped (never forwarded); a message with any non-finite float in
its payload is dropped (never forwarded).

Reframe divergences from Jetlink (single-publisher, always-on-local — see COMPARISON-COMMS.md W2):
  * The state machine's output is an ELIGIBILITY signal + logged events, not an action that stops
    a publication. Absence of a remote message NEVER removes model output: the local model always
    publishes the real services.
  * On BEHIND we emit a ``behind`` event and mark the remote ineligible; the device modeld_v2's
    arbitration picks the fallback. On LOST we emit a ``lost`` event, mark the remote ineligible,
    and reset HOLD accounting; the next received Mac message re-enters normal operation.

Notes:
  * SOF (not EOF) is used for the freshness age, matching §1.4 ("now_device - timestamp_sof").
  * ``drivingModelData`` has no ``timestampSof`` field; its SOF is taken from the same-frame
    camera state (by ``frameId``), exactly like the others. If no camera SOF is known for a
    frame, the message is dropped as un-ageable (never republished blind).
  * ``modelDataV2SP`` (custom.ModelDataV2SP: laneTurnDirection + 2 bools) has no frameId and no
    timestamps, so it can never be aged. SP PAIRING RULE (INTERFACES §7): it is forwarded ONLY when
    paired — by device-clock receipt time, within ``OFFLOAD_SP_PAIR_MS`` (default 25 ms) — with an
    aged-able message (modelV2/cameraOdometry/drivingModelData) that itself passed the freshness +
    finite gate and was forwarded; either side may arrive first. An SP waits in a single-slot
    pending buffer; with no fresh partner inside the window it is dropped and counted as
    ``dropped_unpaired`` (a newer SP supersedes an older pending one, also counted). A stale- or
    non-finite-dropped aged message never anchors a pair. The SP's header is restamped at forward
    time like every other message. Proximity is measured on TRUE receipt stamps (a fresh monotonic
    read when each message is drained — NOT the step-top clock, which predates the blocking camera
    ``update`` and would let a long block in the later step silently over-pair an SP with a partner
    that is really tens of ms away); the injected ``now_ns`` is the receipt stamp in tests. The main
    loop is paced by the camera ``SubMaster.update``, so while an SP pairing is in flight (an SP is
    pending, or a partner was just forwarded and its SP has not been seen) the camera poll is
    shortened to ``SP_PAIR_POLL_MS`` (5 ms), so a straggler that straddles two drains is picked up
    within ~5 ms and still pairs. Pairing is one-to-one: a partner is consumed by the SP it pairs with.
    Failure is always the safe direction (counted ``dropped_unpaired``; the arbiter sees an incomplete
    remote set and the local model publishes that frame).
  * ZMQ SUB socket ``RCVTIMEO`` is set (default 100 ms) so the gap watchdog and SIGTERM handling
    are promptly serviced even when the Mac goes quiet. ``CONFLATE`` is deliberately NOT set:
    every frame must be seen so the stale accounting is exact.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import time
from collections import deque
from dataclasses import dataclass, field

import zmq

from openpilot.offload import ports
from openpilot.offload.contract import RETURN_SERVICES, SHADOW_OUTPUT_SERVICES

# --- tunables (env-overridable; see INTERFACES/README) -----------------------------
DEFAULT_STALE_MS = 300          # OFFLOAD_STALE_MS — max device-clock age of a return frame
DEFAULT_GAP_MS = 400            # OFFLOAD_GAP_MS   — no-valid-republish watchdog
DEFAULT_SOF_TTL_MS = 1000       # drop camera SOF entries older than this
DEFAULT_RECV_TIMEOUT_MS = 100   # ZMQ SUB RCVTIMEO (prompt watchdog + SIGTERM)
RING_SIZE = 4096                # recent republish timestamps for the watchdog
CAMERA_SERVICES = ("narrowRoadCameraState", "wideRoadCameraState")
ENABLED_MODES = ("shadow", "drive", "holds")  # 'holds' = drive + the HOLD/BEHIND/LOST policy
HOLD_POLICY_MODE = "holds"                    # OffloadMode sub-value that turns the policy on
SP_SERVICE = "modelDataV2SP"                  # the one return service with no frameId/timestamps
DEFAULT_SP_PAIR_MS = 25                       # OFFLOAD_SP_PAIR_MS — SP<->aged-msg pairing window
SP_PAIR_POLL_MS = 5                           # camera-poll timeout while an SP pairing is in flight

# Remote (Mac) return service -> the DEVICE msgq SHADOW service name it is forwarded under.
# Never the real modelV2/cameraOdometry/... : those are the device's own model's, and the local
# model publishes them every frame. Names come from contract.SHADOW_OUTPUT_SERVICES (INTERFACES §7).
DEFAULT_SHADOW_MAP = {
  "modelV2": "offloadModelV2",
  "cameraOdometry": "offloadCameraOdometry",
  "drivingModelData": "offloadDrivingModelData",
  "modelDataV2SP": "offloadModelDataV2SP",
}
SHADOW_SERVICES = tuple(SHADOW_OUTPUT_SERVICES)
# The map's values must be exactly the frozen §7 shadow set (order-insensitive).
assert set(DEFAULT_SHADOW_MAP.values()) == set(SHADOW_OUTPUT_SERVICES), "shadow names drifted from contract.SHADOW_OUTPUT_SERVICES"

# --- HOLD/BEHIND/LOST timing policy (Jetlink borrow; default OFF) -----------------
# All of these are inert unless hold_policy is on (OFFLOAD_HOLDPOLICY=1 / OffloadMode
# 'holds' / --hold-policy). With the policy off, offloadd behaves exactly as before.
DEFAULT_HOLD_MS = 46            # OFFLOAD_HOLD_MS   — per-frame reply deadline (Jetlink HOLD_FRAME)
DEFAULT_HOLDS_IN_A_ROW = 5      # OFFLOAD_HOLDS_IN_A_ROW — consecutive HOLDs -> behind
DEFAULT_HOLDS_ALLOWED = 20      # OFFLOAD_HOLDS_ALLOWED  — HOLDs within the window -> behind
DEFAULT_HOLD_WINDOW_S = 10.0    # OFFLOAD_HOLD_WINDOW   — rolling HOLD window (s)
DEFAULT_PROVING_FRAMES = 20     # OFFLOAD_PROVING_FRAMES — settle window; any HOLD here is behind
DEFAULT_SETTLING_FRAMES = 3     # OFFLOAD_SETTLING_FRAMES — first frames never hand back
DEFAULT_LOST_MS = 200           # OFFLOAD_LOST_MS   — no Mac message at all -> lost


def _env_flag(name: str, default: bool = False) -> bool:
  """Truthy env var: unset/empty -> ``default``; 0/false/no/off -> False; else True."""
  raw = os.getenv(name)
  if raw is None or raw == "":
    return default
  return raw.strip().lower() not in ("0", "false", "no", "off")


def _env_float(name: str, default: float) -> float:
  raw = os.getenv(name)
  if raw is None or raw == "":
    return default
  try:
    return float(raw)
  except ValueError:
    return default


def _env_ms(name: str, default: int) -> int:
  """Read an integer-ms env var, tolerating garbage."""
  raw = os.getenv(name)
  if raw is None or raw == "":
    return default
  try:
    return int(float(raw))
  except ValueError:
    return default


def enabled_mode(param_value) -> bool:
  """True when ``param_value`` (bytes | str | None) names an enabling OffloadMode."""
  if param_value is None:
    return False
  if isinstance(param_value, bytes):
    try:
      param_value = param_value.decode()
    except UnicodeDecodeError:
      return False
  return str(param_value).strip().lower() in ENABLED_MODES


def read_offload_mode(params) -> str | None:
  """Best-effort read of the OffloadMode param; tolerant of an absent key and of bad values."""
  if params is None:
    return None
  try:
    v = params.get("OffloadMode")
  except Exception:
    # covers UnknownKeyName and any Params backend hiccup: stay disabled
    return None
  if v is None:
    return None
  if isinstance(v, bytes):
    try:
      v = v.decode()
    except UnicodeDecodeError:
      return None
  return str(v).strip() or None


def sof_for_service(msg, camera_sof: dict) -> int | None:
  """Device-clock SOF (ns) of the camera frame a return message refers to, or None if unknown.

  Prefers an inline ``timestampSof`` when the message carries one (cameraOdometry does not —
  it has only ``timestampEof``/``frameId``; modelV2/drivingModelData carry ``frameId``).
  ``modelDataV2SP`` carries NEITHER a frameId nor any timestamp, so it has no age source here at
  all: it never reaches this function and is handled by the SP pairing rule instead (see
  ``Offloadd._handle_sp``). Falls back to the ``frameId`` -> SOF map fed by the local camera
  states; that map is the authoritative device-clock reference.
  """
  fid = None
  if hasattr(msg, "frameId"):
    try:
      fid = int(msg.frameId)
    except Exception:
      fid = None
  if fid is not None:
    sof = camera_sof.get(fid)
    if sof is not None:
      return sof
  # inline SOF (present on some services, e.g. FrameData family); last resort only
  if hasattr(msg, "timestampSof"):
    try:
      return int(msg.timestampSof)
    except Exception:
      return None
  return None


@dataclass
class Config:
  connect_host: str = "127.0.0.1"          # where the Mac PUBs the RETURN services
  stale_ms: int = field(default_factory=lambda: _env_ms("OFFLOAD_STALE_MS", DEFAULT_STALE_MS))
  gap_ms: int = field(default_factory=lambda: _env_ms("OFFLOAD_GAP_MS", DEFAULT_GAP_MS))
  sof_ttl_ms: int = field(default_factory=lambda: _env_ms("OFFLOAD_SOF_TTL_MS", DEFAULT_SOF_TTL_MS))
  recv_timeout_ms: int = DEFAULT_RECV_TIMEOUT_MS
  bench: bool = False
  bench_path: str = "/tmp/offloadd_bench.jsonl"
  offload_mode: str = "auto"               # auto | shadow | drive | holds (explicit = bench bypass)
  camera_addr: str = "127.0.0.1"           # local msgq address for camera states
  services: tuple = field(default_factory=lambda: tuple(RETURN_SERVICES))
  endpoint_for: object = None             # callable(service)->endpoint; tests inject a free port
  shadow_map: dict = field(default_factory=lambda: dict(DEFAULT_SHADOW_MAP))
  # shadow service names to open a device msgq PubMaster on (defaults to shadow_map's values).
  shadow_services: tuple = field(default_factory=lambda: SHADOW_SERVICES)

  # modelDataV2SP pairing (INTERFACES §7 'SP pairing rule'): SP carries no frameId and no
  # timestamps, so it cannot be aged on its own. It is forwarded only when paired within this
  # window of an aged-able message (modelV2/cameraOdometry/drivingModelData) that itself passed
  # the freshness gate; otherwise it is dropped as ``dropped_unpaired`` (never forwarded blind).
  sp_pair_ms: int = field(default_factory=lambda: _env_ms("OFFLOAD_SP_PAIR_MS", DEFAULT_SP_PAIR_MS))

  # --- HOLD/BEHIND/LOST eligibility policy (Jetlink borrow; default OFF) --------
  # When hold_policy is False every field below is inert and the daemon behaves exactly
  # as it did before this port. Turn it on with OFFLOAD_HOLDPOLICY=1, OffloadMode='holds',
  # or --hold-policy.
  hold_policy: bool = field(default_factory=lambda: _env_flag("OFFLOAD_HOLDPOLICY", False))
  hold_ms: int = field(default_factory=lambda: _env_ms("OFFLOAD_HOLD_MS", DEFAULT_HOLD_MS))
  holds_in_a_row: int = field(default_factory=lambda: _env_ms("OFFLOAD_HOLDS_IN_A_ROW", DEFAULT_HOLDS_IN_A_ROW))
  holds_allowed: int = field(default_factory=lambda: _env_ms("OFFLOAD_HOLDS_ALLOWED", DEFAULT_HOLDS_ALLOWED))
  hold_window_s: float = field(default_factory=lambda: _env_float("OFFLOAD_HOLD_WINDOW", DEFAULT_HOLD_WINDOW_S))
  proving_frames: int = field(default_factory=lambda: _env_ms("OFFLOAD_PROVING_FRAMES", DEFAULT_PROVING_FRAMES))
  settling_frames: int = field(default_factory=lambda: _env_ms("OFFLOAD_SETTLING_FRAMES", DEFAULT_SETTLING_FRAMES))
  lost_ms: int = field(default_factory=lambda: _env_ms("OFFLOAD_LOST_MS", DEFAULT_LOST_MS))
  on_event: object = None                 # optional callable(kind:str, info:dict)->None observer


class _CerealIO:
  """Thin wrapper so tests can drive the daemon without a real device msgq.

  Lazily imports the cereal bindings on first use. Any import failure is surfaced by
  ``ready()`` so the daemon can stay inert (and the manager must not start it) on a host
  that lacks the built native extension.
  """

  def __init__(self, cfg: Config):
    self.cfg = cfg
    self._SubMaster = None
    self._PubMaster = None
    self._log = None
    self.sub = None
    self.pub = None

  def ready(self) -> bool:
    if self._SubMaster is not None:
      return True
    try:
      from openpilot.cereal import log
      from openpilot.cereal.messaging import PubMaster, SubMaster
    except Exception:
      return False
    self._log = log
    self._SubMaster = SubMaster
    self._PubMaster = PubMaster
    return True

  def open(self) -> None:
    self.sub = self._SubMaster(list(CAMERA_SERVICES), addr=self.cfg.camera_addr)
    # forward remote outputs under the SHADOW names (INTERFACES §7), never the real ones.
    self.pub = self._PubMaster(list(self.cfg.shadow_services))

  def update(self, timeout_ms: int) -> None:
    self.sub.update(timeout_ms)

  def camera_sof(self, service: str) -> tuple[int, int] | None:
    """(frameId, timestampSof) of the latest camera state, or None if never received."""
    if not self.sub.seen.get(service):
      return None
    fr = self.sub[service]
    try:
      return int(fr.frameId), int(fr.timestampSof)
    except Exception:
      return None

  def publish(self, service: str, payload: bytes) -> None:
    self.pub.send(service, payload)


def _finite(o) -> bool:
  """Recursively True when every float in ``o`` (dict/list/scalar from ``to_dict``) is finite."""
  if isinstance(o, float):
    return math.isfinite(o)
  if isinstance(o, dict):
    return all(_finite(v) for v in o.values())
  if isinstance(o, (list, tuple)):
    return all(_finite(v) for v in o)
  return True


def _all_finite(reader) -> bool:
  """True when every float in the message payload is finite (never forward a non-finite output).

  Jetlink only replaces hidden state after an all-finite frame (link-protocol.md "Hidden state
  on the server"); carried over, we never forward a message carrying a NaN/Inf. The scan walks
  the nested payload dict; on a synthetic modelV2 with 6208 B of position/velocity/acceleration/
  laneLines floats it measured 0.024 ms on the bench Mac (RESULTS.md), so it is affordable at
  20 Hz. Any decode error is treated as not-finite (fail safe: never forward).
  """
  try:
    return _finite(reader.to_dict())
  except Exception:
    return False


def _restamp(raw: bytes, now_ns: int) -> bytes:
  """Return ``raw`` with the Event header ``logMonoTime`` set to ``now_ns`` and payload untouched.

  Uses pycapnp's zero-copy ``as_builder()`` on the reader; only the root header scalar is
  written, so every payload field survives byte-identically (proven by
  tests/test_offloadd.py::test_restamp_payload_byte_identical).
  """
  from openpilot.cereal import log
  limit = 2 ** 64 - 1
  with log.Event.from_bytes(raw, traversal_limit_in_words=limit) as reader:
    b = reader.as_builder()
    b.logMonoTime = int(now_ns)
    return b.to_bytes()


class Offloadd:
  """Return-path republisher. Construct, then ``run()`` (blocking) — or drive with ``step()`` in tests."""

  def __init__(self, cfg: Config | None = None, cereal: _CerealIO | None = None):
    self.cfg = cfg or Config()
    self.cereal = cereal or _CerealIO(self.cfg)
    self.sof_by_frame: dict[int, int] = {}
    self.recent_republish: deque[float] = deque(maxlen=RING_SIZE)
    self.zmq_msgs = 0
    self.forwarded = 0                       # eligible remote msgs written to a shadow topic
    self.published = 0                      # DEPRECATED alias of ``forwarded`` (compat)
    self.dropped_stale = 0
    self.dropped_nonfinite = 0
    self.dropped_no_sof = 0
    self.dropped_unpaired = 0
    self.dropped_error = 0
    # -- modelDataV2SP pairing (INTERFACES §7 'SP pairing rule') ------------------
    # SP has no frameId/timestamps -> un-ageable. Single-slot pending buffer holds the last SP
    # (raw bytes, receipt device-monotonic ns) until an aged-able message within cfg.sp_pair_ms
    # is forwarded; otherwise it is dropped as ``dropped_unpaired`` (never forwarded blind).
    self.pending_sp: tuple[bytes, int] | None = None
    self.last_forwarded_msg_ns: int | None = None   # receipt ns of the last forwarded aged msg (pairing anchor)
    self.sp_paired = 0                       # SPs forwarded via the pairing rule
    self._clock = time.monotonic_ns          # tests may swap in a fake monotonic source
    self._injected_ns: int | None = None     # step(now_ns=...) test clock; None in production
    self.gap_open = False
    self.last_forward_mono: float | None = None
    self.last_republish_mono: float | None = None   # DEPRECATED alias of last_forward_mono
    self._stop = False
    self._sub_socks: dict[str, zmq.Socket] = {}
    self._zmq_ctx = None
    self._start_mono = time.monotonic()

    # -- HOLD/BEHIND/LOST eligibility state (inert unless cfg.hold_policy) ------
    self.frame_expected: dict[int, float] = {}   # frameId -> device monotonic s first seen
    self.handled_frames: set[int] = set()          # frameIds whose expected output was seen
    self.frame_seen: set[int] = set()                # every camera frameId seen (for index-based ids)
    self.last_received_mono: float | None = None   # last message from the Mac of ANY kind
    self.hold_times: deque[float] = deque()        # monotonic s of recent HOLDs (rolling window)
    self.holds_in_a_row = 0
    self.behind = False                             # remote ineligible (behind); eligibility signal
    self._proving_behind: str | None = None          # first-HOLD-in-proving-window reason, if any
    self.remote_eligible = True                     # aggregate eligibility for modeld_v2 arbitration
    self.hold_count = 0
    self.behind_count = 0
    self.lost_count = 0
    self.lost = False
    # last observed values (eligibility signal inputs). Empty dicts -> the corresponding
    # sub-condition is not evaluated yet (matches Jetlink's "not seen -> pass" default).
    self.max_tx_age_ns: dict[str, int] = {}
    self.max_rx_age_ns: dict[str, int] = {}

  # -- lifecycle ---------------------------------------------------------------
  def request_stop(self, *_: object) -> None:
    self._stop = True

  def install_signal_handlers(self) -> None:
    signal.signal(signal.SIGTERM, self.request_stop)
    signal.signal(signal.SIGINT, self.request_stop)

  def open(self) -> None:
    """Open ZMQ SUB sockets (connect to the Mac) and the local msgq endpoints."""
    if not self.cereal.ready():
      raise RuntimeError("cereal bindings unavailable: openpilot.cereal.messaging / log not importable; manager must not start offloadd on this host")
    self._zmq_ctx = zmq.Context()
    for svc in self.cfg.services:
      s = self._zmq_ctx.socket(zmq.SUB)
      s.setsockopt(zmq.SUBSCRIBE, b"")
      s.setsockopt(zmq.RCVTIMEO, self.cfg.recv_timeout_ms)
      s.setsockopt(zmq.LINGER, 0)
      endpoint = (self.cfg.endpoint_for(svc) if callable(self.cfg.endpoint_for)
                  else ports.endpoint(svc, self.cfg.connect_host))
      s.connect(endpoint)
      self._sub_socks[svc] = s
    self.cereal.open()

  def close(self) -> None:
    for s in self._sub_socks.values():
      try:
        s.close(0)
      except Exception:
        pass
    self._sub_socks.clear()
    if self._zmq_ctx is not None:
      try:
        self._zmq_ctx.term()
      except Exception:
        pass
      self._zmq_ctx = None

  # -- core step ---------------------------------------------------------------
  def step(self, now_ns: int | None = None) -> None:
    """One iteration: refresh camera SOF, drain ZMQ, validate+forward, run watchdogs.

    ``now_ns`` injects the device clock for deterministic tests; production callers omit it.
    """
    self._injected_ns = now_ns
    if now_ns is None:
      now_ns = self._clock()
    now_mono = now_ns / 1e9
    self._refresh_camera_sof(now_mono)
    if self.cfg.hold_policy:
      self._note_frame_arrivals(now_mono)
    self._drain_zmq(now_ns, now_mono)
    self._resolve_pending_sp(now_ns, now_mono)
    self._watchdog(now_mono)
    if self.cfg.hold_policy:
      self._hold_policy_tick(now_mono)

  def _recv_ns(self) -> int:
    """Receipt stamp used ONLY for SP pairing proximity (never for freshness or the header restamp).

    Production (no injected clock): a fresh monotonic read, i.e. when the message was actually drained.
    The step's top-of-step stamp is NOT used for this: it predates the blocking camera ``update`` (up to
    ``recv_timeout_ms``), so two messages drained in different steps would be "paired" or "unpaired"
    according to the length of the camera block instead of the real gap between them — and a long block in
    the LATER step would silently OVER-pair an SP with a partner that is really tens of ms away (the unsafe
    direction; nothing downstream can see it). Tests inject ``step(now_ns=...)``; that clock is then the
    receipt stamp, exactly as it is for every other rule. Residual limit: ZMQ gives no arrival timestamp, so
    "receipt" is the moment the message is drained from the SUB socket.
    """
    return self._injected_ns if self._injected_ns is not None else self._clock()

  def _anchor_fresh(self) -> bool:
    """True while an aged partner was forwarded within the pairing window and no SP has claimed it yet."""
    last = self.last_forwarded_msg_ns
    return last is not None and (self._recv_ns() - last) <= self.cfg.sp_pair_ms * 1_000_000

  def _camera_poll_ms(self) -> int:
    """Camera-poll timeout for this step.

    The camera ``SubMaster.update`` paces the loop and normally blocks up to ``recv_timeout_ms``. While an SP
    pairing is in flight (an SP is pending, or a partner was just forwarded and its SP has not been seen) poll
    briefly instead, so the straggler is drained within ~``SP_PAIR_POLL_MS`` of its arrival: the receipt-stamp
    proximity then measures the network spread of the frame's messages, not the camera block. Costs a few
    extra ~50 us iterations per frame, only inside the <= OFFLOAD_SP_PAIR_MS window.
    """
    if SP_SERVICE in self._sub_socks and (self.pending_sp is not None or self._anchor_fresh()):
      return min(self.cfg.recv_timeout_ms, SP_PAIR_POLL_MS)
    return self.cfg.recv_timeout_ms

  def _refresh_camera_sof(self, now_mono: float) -> None:
    self.cereal.update(self._camera_poll_ms())
    for svc in CAMERA_SERVICES:
      got = self.cereal.camera_sof(svc)
      if got is not None:
        fid, sof = got
        if sof > 0:
          self.sof_by_frame[fid] = sof
          self.frame_seen.add(fid)
    # evict stale entries (route/segment rollover safety)
    if self.sof_by_frame:
      cutoff = (now_mono - self.cfg.sof_ttl_ms / 1000.0) * 1e9
      stale_ids = [fid for fid, sof in self.sof_by_frame.items() if sof < cutoff]
      for fid in stale_ids:
        del self.sof_by_frame[fid]

  def _drain_zmq(self, now_ns: int, now_mono: float) -> None:
    for svc, sock in self._sub_socks.items():
      # drain everything currently queued for this service
      while True:
        try:
          raw = sock.recv(zmq.NOBLOCK)
        except zmq.Again:
          break
        except Exception:
          break
        self.zmq_msgs += 1
        self._handle_return(svc, raw, now_ns, now_mono)

  def _handle_return(self, svc: str, raw: bytes, now_ns: int, now_mono: float) -> None:
    """Freshness-validate one remote message and, if eligible, forward it under its shadow name.

    Never forwards a stale message; never forwards a message with a non-finite payload float;
    never publishes a real service name (the local model owns those). ``modelDataV2SP`` has no
    frameId/timestamps, so it is NOT aged here: it is held pending (INTERFACES §7 'SP pairing
    rule') and forwarded only when paired within ``OFFLOAD_SP_PAIR_MS`` of a message that itself
    passed the freshness gate (see ``_handle_sp``). Any message at all is proof the link is alive,
    so it clears LOST (the 'comma re-hellos' equivalent).
    """
    recv_ns = self._recv_ns()          # TRUE receipt stamp: used ONLY for SP pairing proximity
    if self.cfg.hold_policy:
      self.last_received_mono = now_mono
      if self.lost:
        self.lost = False
        self._reset_hold_accounting()
        self._event("recovered", now_mono)
    if svc == SP_SERVICE:
      self._handle_sp(raw, recv_ns)
      return
    try:
      from openpilot.cereal import log
      limit = 2 ** 64 - 1
      with log.Event.from_bytes(raw, traversal_limit_in_words=limit) as reader:
        if reader.which() != svc:
          # wrong service on this port: never forward
          self.dropped_error += 1
          return
        msg = getattr(reader, svc)
        # -- "never forward stale" (drop, do not forward) --
        sof = sof_for_service(msg, self.sof_by_frame)
        if sof is None:
          # un-ageable: never forward blind (kept from the pre-reframe design)
          self.dropped_no_sof += 1
          return
        age_ns = now_ns - sof
        if age_ns > self.cfg.stale_ms * 1_000_000:
          self.dropped_stale += 1
          return
        if self.cfg.hold_policy:
          self.max_tx_age_ns[svc] = int(age_ns)
        # -- "never forward non-finite output" (drop, do not forward) --
        if not _all_finite(reader):
          self.dropped_nonfinite += 1
          return
        # fresh + finite: single-pass re-stamp; only the root header scalar is written.
        b = reader.as_builder()
        b.logMonoTime = int(now_ns)
        out = b.to_bytes()
        fid = int(msg.frameId) if hasattr(msg, "frameId") else None
    except Exception:
      self.dropped_error += 1
      return

    if self.cfg.hold_policy and fid is not None:
      # this frame's remote output has now been seen; mark it handled so no HOLD fires for it.
      self.handled_frames.add(fid)
      self.holds_in_a_row = 0

    # fresh + finite: a valid "aged" partner. Forward it; its receipt stamp becomes the anchor an SP may pair with.
    self._forward(svc, out, now_ns, now_mono, anchor_ns=recv_ns)

  def _handle_sp(self, raw: bytes, recv_ns: int) -> None:
    """Buffer one ``modelDataV2SP`` for pairing (INTERFACES §7 'SP pairing rule'). Never forwards here.

    SP carries no frameId and no timestamps, so it cannot be aged on its own. It is validated once
    (shadow name mapped / really a modelDataV2SP / all floats finite / decodable — each counted like
    any other message if not, and a bad SP never evicts a good pending one) and held in a SINGLE-SLOT
    pending buffer as (raw bytes, receipt ns). A newer SP supersedes an older pending one (the older
    never found a partner -> ``dropped_unpaired``), so a burst of queued SPs can never be forwarded
    en masse under one fresh partner. The pair / forward / drop decision is made by
    ``_resolve_pending_sp`` once per step AFTER the whole drain, so it does not depend on the order
    in which the per-service sockets happen to be drained.
    """
    if self.cfg.shadow_map.get(SP_SERVICE) is None:
      self.dropped_error += 1          # no shadow name mapped: never forward under the real one
      return
    try:
      from openpilot.cereal import log
      limit = 2 ** 64 - 1
      with log.Event.from_bytes(raw, traversal_limit_in_words=limit) as reader:
        if reader.which() != SP_SERVICE:
          self.dropped_error += 1      # wrong service on the SP port: never forward
          return
        if not _all_finite(reader):
          self.dropped_nonfinite += 1
          return
    except Exception:
      self.dropped_error += 1
      return
    if self.pending_sp is not None:
      self.dropped_unpaired += 1
    self.pending_sp = (raw, recv_ns)

  def _resolve_pending_sp(self, now_ns: int, now_mono: float) -> None:
    """Pair, forward, or drop the pending SP (called once per step, after the ZMQ drain).

    Forward ONLY if an aged-able message that passed the freshness+finite gate was received within
    ``OFFLOAD_SP_PAIR_MS`` of the SP's receipt (either side: SP just before its partner, or just
    after it), comparing TRUE receipt stamps (``_recv_ns``), not the step-top clock. The SP's header
    is restamped at FORWARD time with the step clock like every other message. A stale- or
    non-finite-dropped aged message never became an anchor (``last_forwarded_msg_ns`` only moves on a
    successful aged forward), so it cannot pair. Pairing is one-to-one: the anchor is consumed, so one
    fresh message cannot launder a stream of SPs. With no partner and the window elapsed, the SP is
    dropped as ``dropped_unpaired`` — never forwarded blind.
    """
    if self.pending_sp is None:
      return
    raw, recv_ns = self.pending_sp
    window_ns = self.cfg.sp_pair_ms * 1_000_000
    last = self.last_forwarded_msg_ns
    if last is not None and abs(recv_ns - last) <= window_ns:
      self.pending_sp = None
      self.last_forwarded_msg_ns = None    # consume the anchor: it pairs with at most one SP
      try:
        out = _restamp(raw, now_ns)
      except Exception:
        self.dropped_error += 1
        return
      if self._forward(SP_SERVICE, out, now_ns, now_mono):
        self.sp_paired += 1
    elif self._recv_ns() - recv_ns > window_ns:
      self.pending_sp = None
      self.dropped_unpaired += 1       # window elapsed, no fresh partner

  def _forward(self, svc: str, out: bytes, now_ns: int, now_mono: float, *, anchor_ns: int | None = None) -> bool:
    """Write one restamped payload to its shadow topic; on success update counters and the anchor.

    ``anchor_ns`` is passed only for a message that carried an age source (modelV2/cameraOdometry/
    drivingModelData) and passed the freshness+finite gate: after a SUCCESSFUL publish its receipt
    stamp becomes ``last_forwarded_msg_ns``, the anchor a nearby modelDataV2SP may pair with
    (``_resolve_pending_sp``). Returns True when the message reached the shadow topic.
    """
    shadow = self.cfg.shadow_map.get(svc)
    if shadow is None:
      self.dropped_error += 1
      return False
    try:
      self.cereal.publish(shadow, out)
    except Exception:
      self.dropped_error += 1
      return False
    self.forwarded += 1
    self.published = self.forwarded    # compat alias
    self.recent_republish.append(now_mono)
    self.last_forward_mono = now_mono
    self.last_republish_mono = now_mono
    if anchor_ns is not None:
      self.last_forwarded_msg_ns = anchor_ns
    return True

  # -- HOLD/BEHIND/LOST eligibility (Jetlink borrow; only when cfg.hold_policy) -----
  def _note_frame_arrivals(self, now_mono: float) -> None:
    """Record the device-monotonic time each camera frame was first seen at offloadd.

    This anchors the HOLD deadline: Jetlink measures HOLD_FRAME from the frame's warp start; we
    measure OFFLOAD_HOLD_MS from the frame's first arrival here, because the device has no warp
    of its own for a remotely computed frame. A frameId already known keeps its first-seen time.
    """
    for fid in list(self.sof_by_frame):
      self.frame_expected.setdefault(fid, now_mono)
    if not self.frame_expected:
      return
    # prune: keep an entry only as long as its output could still arrive fresh, plus the window.
    keep_s = self.cfg.stale_ms / 1000.0 + self.cfg.hold_window_s
    for fid in [fid for fid, t in self.frame_expected.items() if now_mono - t > keep_s]:
      del self.frame_expected[fid]
    self.handled_frames.intersection_update(self.frame_expected)
    self.frame_seen.intersection_update(self.frame_expected)

  def _hold_policy_tick(self, now_mono: float) -> None:
    """Advance LOST, evaluate HOLD deadlines, and recompute the remote eligibility signal."""
    # LOST: nothing from the Mac (any message) for lost_ms.
    if self.last_received_mono is not None and (now_mono - self.last_received_mono) * 1000.0 >= self.cfg.lost_ms:
      if not self.lost:
        self.lost = True
        self.lost_count += 1
        self._event("lost", now_mono, extra={"silent_ms": round((now_mono - self.last_received_mono) * 1000.0, 1)})
      self._reset_hold_accounting()
    else:
      if self.lost:
        self.lost = False
        self._reset_hold_accounting()
      # HOLD: the oldest expected frame whose remote output has not been seen and is overdue.
      deadline_s = self.cfg.hold_ms / 1000.0
      for fid in sorted(self.frame_expected, key=lambda k: self.frame_expected[k]):
        if fid in self.handled_frames:
          continue
        if now_mono - self.frame_expected[fid] < deadline_s:
          continue
        self._do_hold(fid, now_mono)
        break

    # behind is a per-frame signal (Jetlink recomputes it each frame) derived from the persistent
    # HOLD state; the "behind" EVENT fires once per onset, so a sustained overshoot logs one line.
    reason = None if self.lost else self._behind_now_reason()
    if reason is not None and not self.behind:
      self.behind_count += 1
      self._event("behind", now_mono, extra={"reason": reason})
    self.behind = reason is not None

    # Eligibility signal for the modeld_v2 arbitration (INTERFACES §7). Local TX age is always
    # satisfied here (the local model publishes steadily); RX age is the remote's freshness.
    self.remote_eligible = self._compute_eligibility()

  def _do_hold(self, fid: int, now_mono: float) -> None:
    """A frame's remote output is overdue: count a HOLD and note a proving-window violation."""
    self.handled_frames.add(fid)
    self.holds_in_a_row += 1
    self.hold_count += 1
    self.hold_times.append(now_mono)
    while self.hold_times and now_mono - self.hold_times[0] > self.cfg.hold_window_s:
      self.hold_times.popleft()
    self._event("hold", now_mono, extra={"frame_id": fid, "in_a_row": self.holds_in_a_row})
    if self.cfg.settling_frames < fid <= self.cfg.proving_frames and self._proving_behind is None:
      self._proving_behind = f"held frame {fid} of the first {self.cfg.proving_frames}"

  def _behind_now_reason(self) -> str | None:
    """Why the remote is behind right now, or None. Jetlink's _note_hold, recomputed per frame:
    the first SETTLING_FRAMES never go behind; any HOLD among the first PROVING_FRAMES does;
    then holds_in_a_row, then holds_allowed within the window."""
    if self._proving_behind is not None:
      return self._proving_behind
    if self.holds_in_a_row >= self.cfg.holds_in_a_row:
      return f"held {self.holds_in_a_row} frames in a row"
    if len(self.hold_times) > self.cfg.holds_allowed:
      return f"held {len(self.hold_times)} frames in {self.cfg.hold_window_s:.0f} s"
    return None

  def _compute_eligibility(self) -> bool:
    """True when the remote may be selected. Any unsatisfied sub-condition makes it ineligible."""
    if self.lost or self.behind:
      return False
    if self.hold_times:                       # a frame is currently held -> not eligible this frame
      return False
    limit_ns = self.cfg.stale_ms * 1_000_000
    if self.max_tx_age_ns and max(self.max_tx_age_ns.values()) > limit_ns:
      return False
    if self.max_rx_age_ns and max(self.max_rx_age_ns.values()) > limit_ns:
      return False
    return True

  def _reset_hold_accounting(self) -> None:
    """Clear HOLD bookkeeping (on LOST, and before resuming fresh after a LOST)."""
    self.holds_in_a_row = 0
    self.hold_times.clear()
    self._proving_behind = None

  def _watchdog(self, now_mono: float) -> None:
    """Log the return-link gap edges: no eligible remote forward within gap_ms.

    This is about the RETURN link only. The real modelV2/... are the local model's and are never
    gap-gated here; the shadow topics going quiet is the remote's staleness signal for the
    modeld_v2 arbitration (INTERFACES §7), not for controlsd.
    """
    if self.last_forward_mono is None:
      if not self.gap_open and (now_mono - self._start_mono) * 1000.0 >= self.cfg.gap_ms:
        self._open_gap(now_mono)
      return
    if (now_mono - self.last_forward_mono) * 1000.0 >= self.cfg.gap_ms:
      self._open_gap(now_mono)
    elif self.gap_open:
      self.gap_open = False
      self._event("gap_closed", now_mono)

  def _open_gap(self, now_mono: float) -> None:
    if not self.gap_open:
      self.gap_open = True
      self._event("gap_open", now_mono)

  # -- output ------------------------------------------------------------------
  def _event(self, kind: str, now_mono: float, extra: dict | None = None) -> None:
    line = {
      "ts_mono": round(now_mono, 6),
      "event": kind,
      "gap_ms": self.cfg.gap_ms,
    }
    if extra:
      line.update(extra)
    print(json.dumps(line), flush=True)

  def stats(self, now_mono: float | None = None) -> dict:
    now_mono = time.monotonic() if now_mono is None else now_mono
    st = {
      "ts_mono": round(now_mono, 6),
      "uptime_s": round(now_mono - self._start_mono, 3),
      "zmq_msgs": self.zmq_msgs,
      "forwarded": self.forwarded,
      "published": self.published,          # compat alias of forwarded
      "dropped_stale": self.dropped_stale,
      "dropped_nonfinite": self.dropped_nonfinite,
      "dropped_no_sof": self.dropped_no_sof,
      "dropped_unpaired": self.dropped_unpaired,
      "sp_paired": self.sp_paired,
      "dropped_error": self.dropped_error,
      "sof_frames_tracked": len(self.sof_by_frame),
      "gap_open": self.gap_open,
    }
    if self.cfg.hold_policy:
      st.update({
        "hold_policy": True,
        "holds": self.hold_count,
        "held_in_a_row": self.holds_in_a_row,
        "behind": self.behind,
        "behind_events": self.behind_count,
        "lost": self.lost,
        "lost_events": self.lost_count,
        "remote_eligible": self.remote_eligible,
      })
    return st

  def _emit_stats(self, now_mono: float) -> dict:
    st = self.stats(now_mono)
    line = json.dumps(st, separators=(",", ":"))
    print(line, flush=True)
    if self.cfg.bench:
      try:
        with open(self.cfg.bench_path, "a") as f:
          f.write(line + "\n")
      except OSError:
        pass
    return st

  # -- run loop ----------------------------------------------------------------
  def run(self, stats_period_s: float = 5.0) -> int:
    self.install_signal_handlers()
    self.open()
    self._event("offloadd_start", time.monotonic())
    last_stats = time.monotonic()
    try:
      while not self._stop:
        self.step()
        now_mono = time.monotonic()
        if now_mono - last_stats >= stats_period_s:
          self._emit_stats(now_mono)
          last_stats = now_mono
    finally:
      self._emit_stats(time.monotonic())
      self.close()
      self._event("offloadd_stop", time.monotonic())
    return 0


def main() -> int:
  ap = argparse.ArgumentParser(description="device-side offload return-path republisher")
  ap.add_argument("--offload-mode", default=os.getenv("OFFLOAD_MODE", "auto"),
                  choices=("auto", "shadow", "drive", "holds"),
                  help="auto = read OffloadMode param (production); shadow/drive/holds = explicit enable (bench)")
  ap.add_argument("--connect-host", default=os.getenv("OFFLOAD_RETURN_HOST", "127.0.0.1"),
                  help="host where the Mac publishes RETURN_SERVICES")
  ap.add_argument("--hold-policy", action="store_true",
                  help="enable the HOLD/BEHIND/LOST timing policy (also OFFLOAD_HOLDPOLICY=1, or --offload-mode holds)")
  ap.add_argument("--bench", action="store_true", help="also append stats to a local /tmp jsonl")
  ap.add_argument("--bench-path", default="/tmp/offloadd_bench.jsonl")
  ap.add_argument("--stats-period", type=float, default=5.0)
  args = ap.parse_args()

  cfg = Config(bench=args.bench, bench_path=args.bench_path,
               connect_host=args.connect_host, offload_mode=args.offload_mode)

  # mode gate (step e)
  if args.offload_mode == "auto":
    try:
      from openpilot.common.params import Params
      params = Params()
    except Exception:
      params = None
    mode = read_offload_mode(params)
    if not enabled_mode(mode):
      print(json.dumps({"event": "disabled", "reason": "OffloadMode not in {shadow,drive,holds}",
                        "offload_mode": mode}), flush=True)
      return 0
    cfg.offload_mode = mode or args.offload_mode
  else:
    cfg.offload_mode = args.offload_mode

  # HOLD/BEHIND/LOST policy: on for --hold-policy, OFFLOAD_HOLDPOLICY=1, or OffloadMode 'holds'.
  if args.hold_policy or cfg.offload_mode == HOLD_POLICY_MODE:
    cfg.hold_policy = True

  if not _CerealIO(cfg).ready():
    print(json.dumps({"event": "disabled", "reason": "cereal bindings unavailable",
                      "offload_mode": cfg.offload_mode}), flush=True)
    return 0

  return Offloadd(cfg).run(stats_period_s=args.stats_period)


if __name__ == "__main__":
  sys.exit(main())
