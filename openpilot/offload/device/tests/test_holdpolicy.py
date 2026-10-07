"""Unit tests for the ported HOLD/BEHIND/LOST eligibility machine in offloadd (INTERFACES §7 reframe).

Runnable on the Mac (no device): a real localhost ZMQ PUB + the in-process cereal msgq, with the
device clock injected via ``step(now_ns=...)`` so the 46 ms HOLD deadline, the 0.2 s LOST timeout,
and the behind rules are deterministic.

Reframe (2026-10-07): the device's LOCAL model always publishes; this daemon only forwards remote
outputs into SHADOW topics (offloadModelV2/...). The state machine produces an ELIGIBILITY signal +
events, never a stop-publishing action, so a "behind" frame still forwards later eligible frames.
Absence of a remote message never removes model output.

Run:
  PYTHONPATH=$PWD:$PWD/opendbc_repo:$PWD/msgq_repo:$PWD/tinygrad_repo \\
    .venv/bin/python -m pytest openpilot/offload/device/tests/test_holdpolicy.py -v
"""
from __future__ import annotations

import socket
import struct
import time

import pytest
import zmq

from openpilot.offload.contract import RETURN_SERVICES
from openpilot.offload.device import offloadd as od
from openpilot.offload.device.tests.test_offloadd import (
  _HAVE_CEREAL,
  _load,
  _modelv2,
  needs_cereal,
)

if not _HAVE_CEREAL:  # pragma: no cover - only where cereal/msgq are missing
  pytest.skip("cereal/msgq native bindings not importable", allow_module_level=True)

import msgq
from openpilot.cereal.services import SERVICE_LIST

T0 = 10_000_000_000           # device clock base (ns) — arbitrary, only deltas matter
MS = 1_000_000


def _free_port() -> int:
  s = socket.socket()
  s.bind(("127.0.0.1", 0))
  p = s.getsockname()[1]
  s.close()
  return p


class _HoldCereal(od._CerealIO):
  """_CerealIO with the device camera SUB replaced by direct injection.

  The HOLD/BEHIND/LOST logic is about the state machine, not the camera transport, and the
  shared-machine msgq camera topic is cross-process (another session's integration run feeds it
  stray frameIds — RESULTS.md). So these tests inject frames into ``sof_by_frame`` and keep only
  the REAL shadow PubMaster, which nothing else publishes to. The end-to-end camera path is
  covered by test_offloadd.py.
  """

  def open(self) -> None:
    self.sub = None
    self.pub = self._PubMaster(list(self.cfg.shadow_services))

  def update(self, timeout_ms: int) -> None:
    pass

  def camera_sof(self, service: str):
    return None


class HoldFixture:
  """A daemon with hold_policy ON, the device clock injected, and frames injected directly.

  A "frame" is ``inject_frame(fid, t_ns)`` (sets the device-clock SOF); a "remote reply" is a
  ``modelV2`` on the localhost ZMQ PUB. Forwarding is observed on the RAW msgq shadow topic.
  """

  def __init__(self, cfg_extra=None):
    extra = dict(cfg_extra or {})
    extra.setdefault("hold_policy", True)
    self.ports_map = {svc: _free_port() for svc in RETURN_SERVICES}
    self.cfg = od.Config(
      stale_ms=extra.pop("stale_ms", 300), gap_ms=extra.pop("gap_ms", 100_000),
      recv_timeout_ms=20,
      endpoint_for=lambda svc: f"tcp://127.0.0.1:{self.ports_map[svc]}",
      services=tuple(RETURN_SERVICES), **extra,
    )
    self.ctx = zmq.Context()
    self.pub = self.ctx.socket(zmq.PUB)
    self.pub.setsockopt(zmq.LINGER, 0)
    self.pub.bind(self.cfg.endpoint_for("modelV2"))
    self.pub.monitor(f"inproc://holdpolicy-mon-{id(self)}", zmq.EVENT_ACCEPTED)
    self.mon = self.pub.get_monitor_socket()

    self.daemon = od.Offloadd(self.cfg, cereal=_HoldCereal(self.cfg))
    self.daemon.open()

    shadow = self.cfg.shadow_map["modelV2"]
    self.shadow_sub = msgq.sub_sock(shadow, addr="127.0.0.1", timeout=200,
                                    segment_size=SERVICE_LIST[shadow].queue_size)

    self._wait_accepted()
    time.sleep(0.25)                 # let SUBSCRIBE propagate past the ACCEPTED edge

  def _wait_accepted(self, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
      if self.mon.poll(100):
        try:
          frames = self.mon.recv_multipart()
        except zmq.ZMQError:
          continue
        if struct.unpack("<H", frames[0][:2])[0] & zmq.EVENT_ACCEPTED:
          return
    raise AssertionError("ZMQ subscription never accepted")

  # -- device-clock driving ----------------------------------------------------
  def inject_frame(self, fid, t_ns, age_ms=5, timeout=2.0):
    """Announce frame ``fid`` (SOF ``t_ns - age_ms``) by direct injection; drive until recorded."""
    sof = t_ns - age_ms * MS
    self.daemon.sof_by_frame[fid] = sof
    self.daemon.frame_seen.add(fid)
    end = time.monotonic() + timeout
    while time.monotonic() < end:
      self.daemon.step(now_ns=t_ns)
      if self.daemon.frame_expected.get(fid) is not None:
        return sof
      time.sleep(0.01)
    return sof

  def send_modelv2(self, fid, t_ns, eof_offset_ms=2):
    self.pub.send(_modelv2(fid, t_ns + eof_offset_ms * MS))

  def step_until(self, pred, t_ns, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
      self.daemon.step(now_ns=t_ns)
      if pred():
        return True
      time.sleep(0.03)
    return False

  def forward_reply(self, fid, t_ns):
    """Send a fresh reply for ``fid`` and drive until it is forwarded; returns success."""
    self.send_modelv2(fid, t_ns)
    return self.step_until(lambda: self.daemon.forwarded >= 1, t_ns)

  def recv_shadow(self, timeout_s=3.0):
    """Raw msgq receive on the shadow topic: (logMonoTime, frameId, frameAge), or None."""
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
      d = self.shadow_sub.receive()
      if d:
        with _load(bytes(d)) as r:
          return r.logMonoTime, r.modelV2.frameId, r.modelV2.frameAge
    return None

  def close(self):
    self.daemon.close()
    try:
      self.mon.close(0)
    except Exception:
      pass
    self.pub.close(0)
    self.ctx.term()


# ---------------------------------------------------------------------------
# config plumbing
# ---------------------------------------------------------------------------

def test_hold_policy_defaults_off():
  cfg = od.Config()
  assert cfg.hold_policy is False
  assert cfg.hold_ms == 46
  assert cfg.holds_in_a_row == 5
  assert cfg.holds_allowed == 20
  assert cfg.hold_window_s == 10.0
  assert cfg.lost_ms == 200
  assert cfg.shadow_map["modelV2"] == "offloadModelV2"
  assert "offloadCameraOdometry" in cfg.shadow_services


def test_hold_policy_env_flag(monkeypatch):
  monkeypatch.setenv("OFFLOAD_HOLDPOLICY", "1")
  assert od.Config().hold_policy is True
  monkeypatch.setenv("OFFLOAD_HOLDPOLICY", "0")
  assert od.Config().hold_policy is False
  monkeypatch.delenv("OFFLOAD_HOLDPOLICY", raising=False)
  assert od.Config().hold_policy is False


def test_holds_mode_is_enabled():
  assert od.enabled_mode("holds")
  assert od.enabled_mode(b" Holds ")
  assert od.HOLD_POLICY_MODE in od.ENABLED_MODES


# ---------------------------------------------------------------------------
# HOLD: fires at the deadline; eligibility drops; nothing is published
# ---------------------------------------------------------------------------

@needs_cereal
def test_hold_does_not_fire_before_deadline():
  fx = HoldFixture({"settling_frames": 0, "proving_frames": 0})
  try:
    fx.inject_frame(50, T0)
    assert fx.forward_reply(50, T0)
    fx.inject_frame(51, T0)                    # frame 51 announced, its output never arrives
    fx.daemon.step(now_ns=T0 + 45 * MS)         # before the 46 ms deadline
    assert fx.daemon.hold_count == 0
  finally:
    fx.close()


@needs_cereal
def test_hold_fires_at_46ms_and_makes_remote_ineligible():
  fx = HoldFixture({"settling_frames": 0, "proving_frames": 0})
  try:
    fx.inject_frame(50, T0)
    assert fx.forward_reply(50, T0)
    assert fx.daemon.forwarded == 1

    fx.inject_frame(51, T0)
    held_at = T0 + 47 * MS
    assert fx.step_until(lambda: fx.daemon.hold_count == 1, held_at)
    assert fx.daemon.hold_count == 1
    assert fx.daemon.holds_in_a_row == 1
    assert fx.daemon.remote_eligible is False   # eligibility signal for modeld_v2
    assert fx.daemon.forwarded == 1             # HOLD does NOT forward anything
  finally:
    fx.close()


@needs_cereal
def test_remote_output_forwarded_under_shadow_name_header_restamped():
  fx = HoldFixture({"settling_frames": 0, "proving_frames": 0})
  try:
    fx.inject_frame(50, T0)
    got_at = T0 + 20 * MS
    fx.send_modelv2(50, T0)
    assert fx.step_until(lambda: fx.daemon.forwarded >= 1, got_at)
    got = fx.recv_shadow()
    assert got is not None
    mt, fid, age = got
    assert fid == 50 and age == 3               # payload intact (frame 50, frameAge 3)
    assert mt == got_at                          # header = device monotonic now
  finally:
    fx.close()


# ---------------------------------------------------------------------------
# the correctness rules: never forward stale, never forward non-finite
# ---------------------------------------------------------------------------

@needs_cereal
def test_stale_never_forwarded():
  fx = HoldFixture({"settling_frames": 0, "proving_frames": 0, "stale_ms": 300,
                    "lost_ms": 10**9})   # isolate the stale rule from LOST
  try:
    fx.inject_frame(50, T0)
    assert fx.forward_reply(50, T0)
    assert fx.daemon.forwarded == 1

    # frame 51 announced; its output only "arrives" 400 ms late -> stale now -> never forwarded
    fx.inject_frame(51, T0)
    fx.send_modelv2(51, T0)
    assert fx.step_until(lambda: fx.daemon.dropped_stale >= 1, T0 + 400 * MS)
    assert fx.daemon.forwarded == 1
  finally:
    fx.close()


@needs_cereal
def test_nonfinite_never_forwarded():
  fx = HoldFixture({"settling_frames": 0, "proving_frames": 0, "lost_ms": 10**9})
  try:
    fx.inject_frame(50, T0)
    raw = _modelv2(50, T0 + 2 * MS)
    with _load(raw) as r:
      b = r.as_builder()
      b.modelV2.modelExecutionTime = float("inf")
      raw = b.to_bytes()
    fx.pub.send(raw)
    assert fx.step_until(lambda: fx.daemon.dropped_nonfinite == 1, T0 + 5 * MS)
    assert fx.daemon.forwarded == 0
  finally:
    fx.close()


# ---------------------------------------------------------------------------
# BEHIND: 5 consecutive, and >20 in the window; the remote goes ineligible but
# the daemon keeps FORWARDING later eligible messages (no stop-publishing action)
# ---------------------------------------------------------------------------

def _announce_gapless(fx, fids, t0, spacing_ms):
  for i, fid in enumerate(fids):
    fx.inject_frame(fid, t0 + i * spacing_ms * MS)


@needs_cereal
def test_five_consecutive_holds_trigger_behind_not_stop():
  fx = HoldFixture({"settling_frames": 0, "proving_frames": 0,
                    "holds_in_a_row": 5, "holds_allowed": 10**6, "lost_ms": 10**9})
  try:
    fx.inject_frame(99, T0)
    fx.forward_reply(99, T0)                    # baseline output
    _announce_gapless(fx, [100, 101, 102, 103, 104], T0, spacing_ms=10)
    end = T0 + 200 * MS                         # every deadline is past
    for k in range(6):
      fx.daemon.step(now_ns=end + k * MS)       # one HOLD per step, one per frame
    assert fx.daemon.holds_in_a_row == 5
    assert fx.daemon.behind is True
    assert fx.daemon.behind_count == 1
    assert fx.daemon.remote_eligible is False

    # REFRAME: behind does NOT stop forwarding. A later fresh reply is still forwarded.
    before = fx.daemon.forwarded
    fx.inject_frame(120, end + 400 * MS)
    fx.send_modelv2(120, end + 400 * MS)
    assert fx.step_until(lambda: fx.daemon.forwarded == before + 1, end + 400 * MS)
  finally:
    fx.close()


@needs_cereal
def test_twenty_in_window_trigger_behind():
  fx = HoldFixture({"settling_frames": 0, "proving_frames": 0,
                    "holds_in_a_row": 10**6, "holds_allowed": 20, "hold_window_s": 2.0})
  try:
    _announce_gapless(fx, list(range(200, 221)), T0, spacing_ms=5)  # 21 frames
    end = T0 + 400 * MS
    for k in range(22):
      fx.daemon.step(now_ns=end + k * 1000)     # 1 ms apart, all inside the 2 s window
    assert len(fx.daemon.hold_times) > 20
    assert fx.daemon.behind is True
    assert fx.daemon.behind_count == 1
  finally:
    fx.close()


# ---------------------------------------------------------------------------
# LOST and recovery
# ---------------------------------------------------------------------------

@needs_cereal
def test_lost_at_200ms_then_recovery():
  fx = HoldFixture({"settling_frames": 0, "proving_frames": 0, "lost_ms": 200})
  try:
    fx.inject_frame(50, T0)
    fx.forward_reply(50, T0)
    assert fx.daemon.forwarded == 1
    assert fx.daemon.lost is False

    fx.daemon.step(now_ns=T0 + 201 * MS)        # no Mac message at all -> lost
    assert fx.daemon.lost is True
    assert fx.daemon.lost_count == 1
    assert fx.daemon.remote_eligible is False
    assert fx.daemon.forwarded == 1             # kept NOT forwarding

    # a subsequent Mac message re-enters normal operation ("comma re-hellos")
    fx.inject_frame(52, T0 + 300 * MS)
    fx.send_modelv2(52, T0 + 300 * MS)
    assert fx.step_until(lambda: fx.daemon.lost is False, T0 + 300 * MS)
    assert fx.daemon.forwarded == 2
  finally:
    fx.close()


# ---------------------------------------------------------------------------
# stats + default-off parity
# ---------------------------------------------------------------------------

def test_stats_includes_hold_fields_only_when_on():
  on_cfg = od.Config(hold_policy=True)
  on = od.Offloadd(on_cfg, cereal=od._CerealIO(on_cfg))
  st = on.stats(now_mono=1.0)
  assert {"hold_policy", "holds", "behind", "lost", "behind_events", "lost_events",
          "remote_eligible", "forwarded", "dropped_nonfinite"}.issubset(st)

  off_cfg = od.Config(hold_policy=False)
  off = od.Offloadd(off_cfg, cereal=od._CerealIO(off_cfg))
  assert "hold_policy" not in off.stats(now_mono=1.0)


@needs_cereal
def test_default_off_path_unchanged():
  """hold_policy off: a reply-less frame holds nothing and never goes behind/lost."""
  fx = HoldFixture({"hold_policy": False, "settling_frames": 0, "proving_frames": 0})
  try:
    fx.inject_frame(50, T0)
    assert fx.forward_reply(50, T0)
    fx.inject_frame(51, T0)
    fx.daemon.step(now_ns=T0 + 500 * MS)        # would be hold + lost if the policy were on
    assert fx.daemon.hold_count == 0
    assert fx.daemon.lost is False
    assert fx.daemon.behind is False
    assert "hold_policy" not in fx.daemon.stats()
  finally:
    fx.close()


if __name__ == "__main__":
  raise SystemExit(pytest.main([__file__, "-v"]))
