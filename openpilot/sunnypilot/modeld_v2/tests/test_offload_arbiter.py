"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Unit + integration tests for the §7 output-arbitration hook (offload_arbiter.OffloadArbiter).

Runnable offline on the Mac (no device) and under BOTH `pytest` and
`tools/test_runner.py` (unittest): no pytest-only fixtures are used. The state-machine tests are
hermetic — the codec (``_parse``/``restamp``) is patched out so no native cereal/msgq bindings are
needed. The socket-path test uses the real localhost msgq transport (raw sub/pub on the four
SHADOW names and the four REAL names), exactly like device/tests/test_offloadd.py, and asserts the
emitted real-name bytes differ from the inbound shadow bytes ONLY in the header logMonoTime
(INTERFACES §1.3 / §7).

Run:
  pytest openpilot/sunnypilot/modeld_v2/tests/test_offload_arbiter.py -q
  PYTHONPATH=$PWD:$PWD/opendbc_repo:$PWD/msgq_repo:$PWD/tinygrad_repo \\
    .venv/bin/python tools/test_runner.py openpilot/sunnypilot/modeld_v2/tests/test_offload_arbiter.py
"""
from __future__ import annotations

import os
import time
import unittest
from unittest import mock

from openpilot.offload.contract import ELIGIBLE_MS, ENTER_N, EXIT_N, SETTLE_N
from openpilot.sunnypilot.modeld_v2 import offload_arbiter as oa

MS = 1_000_000
T0 = 10_000_000_000

try:
  import msgq
  from openpilot.cereal import log as cereal_log
  from openpilot.cereal.messaging import PubMaster, new_message
  from openpilot.cereal.services import SERVICE_LIST
  _HAVE_CEREAL = True
except Exception:  # pragma: no cover - host without native bindings
  _HAVE_CEREAL = False
  SERVICE_LIST = {}

needs_cereal = unittest.skipUnless(_HAVE_CEREAL, "cereal/msgq native bindings not importable")
_TRAV = 2 ** 64 - 1


# ---------------------------------------------------------------------------
# hermetic codec: raw payload bytes carry "<real>|<fid>"; restamp tags device-now
# ---------------------------------------------------------------------------

def _raw(real: str, fid: int | None = None) -> bytes:
  return f"{real}|{fid if fid is not None else ''}".encode()


def _fake_parse(raw: bytes):
  real, _, fid = raw.decode().partition("|")
  return real, (int(fid) if fid else None)


def _fake_restamp(raw: bytes, now: int) -> bytes:
  return raw + b"@" + str(now).encode()


class _CodecPatched(unittest.TestCase):
  """Hermetic base: replace the native codec with the tag-based fake above."""

  def setUp(self):
    self.enterContext(mock.patch.object(oa, "_parse", _fake_parse))
    self.enterContext(mock.patch.object(oa, "restamp", _fake_restamp))


_SHADOW_OF = {v: k for k, v in oa.SHADOW_TO_REAL.items()}


def _ingest_set(arb, fid, t_ns, sp=True, sp_t=None):
  for real in oa.ID_SERVICES:
    arb.ingest(_SHADOW_OF[real], _raw(real, fid), t_ns)
  if sp:
    arb.ingest("offloadModelDataV2SP", _raw("modelDataV2SP"), t_ns if sp_t is None else sp_t)


# ---------------------------------------------------------------------------
# 1. gating: default off, no sockets, decide() never remote
# ---------------------------------------------------------------------------

class TestGating(unittest.TestCase):
  def setUp(self):
    self._env = dict(os.environ)
    os.environ.pop("OFFLOAD_ARBITRATION", None)

  def tearDown(self):
    os.environ.clear()
    os.environ.update(self._env)

  def test_default_off_makes_no_arbiter_and_no_socket(self):
    os.environ.pop("OFFLOAD_ARBITRATION", None)
    with mock.patch.object(oa, "COMMA_HARDWARE", False):
      with mock.patch.object(oa, "OffloadArbiter", side_effect=AssertionError("must not construct")) as ctor:
        self.assertFalse(oa.is_arbiter_enabled())
        self.assertIsNone(oa.make_arbiter())
        ctor.assert_not_called()

  def test_enabled_by_env_flag_is_lazy(self):
    os.environ["OFFLOAD_ARBITRATION"] = "1"
    arb = oa.make_arbiter()
    self.assertIsNotNone(arb)
    self.assertIsNone(arb._subs)              # lazy: no socket until the first poll()

  def test_disabled_by_env_flag_value(self):
    os.environ["OFFLOAD_ARBITRATION"] = "0"
    with mock.patch.object(oa, "COMMA_HARDWARE", False):
      self.assertFalse(oa.is_arbiter_enabled())

  def test_drive_param_gate(self):
    class P:
      def __init__(self, v):
        self.v = v

      def get(self, _k):
        if self.v is None:
          raise RuntimeError("UnknownKeyName")
        return self.v

    os.environ.pop("OFFLOAD_ARBITRATION", None)
    with mock.patch.object(oa, "COMMA_HARDWARE", True):
      self.assertTrue(oa.is_arbiter_enabled(P(b"drive")))
      self.assertTrue(oa.is_arbiter_enabled(P(" Drive ")))
      self.assertFalse(oa.is_arbiter_enabled(P(b"shadow")))
      self.assertFalse(oa.is_arbiter_enabled(P(None)))       # UnknownKeyName -> off
    with mock.patch.object(oa, "COMMA_HARDWARE", False):
      self.assertFalse(oa.is_arbiter_enabled(P(b"drive")))    # PC never gates on


# ---------------------------------------------------------------------------
# 2. eligibility boundaries + cohesion (missing piece)
# ---------------------------------------------------------------------------

class TestEligibility(_CodecPatched):
  def test_defaults_match_contract(self):
    arb = oa.OffloadArbiter()
    self.assertEqual(arb.eligible_ns, ELIGIBLE_MS * MS)
    self.assertEqual(arb.enter_n, ENTER_N)
    self.assertEqual(arb.exit_n, EXIT_N)
    self.assertEqual(arb.settle_n, SETTLE_N)

  def test_boundary_45ms_ok_47ms_not(self):
    arb = oa.OffloadArbiter(eligible_ms=46, enter_n=1, exit_n=3, settle_n=0)
    _ingest_set(arb, 100, T0)
    self.assertIsNotNone(arb.select(100, T0 + 45 * MS))

    arb2 = oa.OffloadArbiter(eligible_ms=46, enter_n=1, exit_n=3, settle_n=0)
    _ingest_set(arb2, 100, T0)
    self.assertIsNone(arb2.select(100, T0 + 47 * MS))
    self.assertFalse(arb2.engaged)

  def test_late_set_not_eligible(self):
    arb = oa.OffloadArbiter(eligible_ms=46, enter_n=1, settle_n=0)
    _ingest_set(arb, 100, T0 - 47 * MS)         # arrived 47 ms before the publish moment
    self.assertIsNone(arb.select(100, T0))

  def test_missing_one_of_four_not_eligible(self):
    for drop in oa.ID_SERVICES:                # modelV2 / cameraOdometry / drivingModelData
      arb = oa.OffloadArbiter(enter_n=1, settle_n=0)
      for real in oa.ID_SERVICES:
        if real != drop:
          arb.ingest(_SHADOW_OF[real], _raw(real, 100), T0)
      arb.ingest("offloadModelDataV2SP", _raw("modelDataV2SP"), T0)
      self.assertIsNone(arb.select(100, T0 + MS), f"missing {drop} must not be eligible")
      self.assertFalse(arb.engaged)

  def test_missing_sp_not_eligible(self):
    arb = oa.OffloadArbiter(enter_n=1, settle_n=0)
    _ingest_set(arb, 100, T0, sp=False)
    self.assertIsNone(arb.select(100, T0 + MS))
    self.assertFalse(arb.engaged)

  def test_set_newer_than_published_frame_is_not_used(self):
    arb = oa.OffloadArbiter(enter_n=1, settle_n=0)
    _ingest_set(arb, 200, T0)                   # a set for a FUTURE frame
    self.assertIsNone(arb.select(100, T0 + MS))


# ---------------------------------------------------------------------------
# 3. hysteresis
# ---------------------------------------------------------------------------

class TestHysteresis(_CodecPatched):
  def test_nine_then_ten(self):
    arb = oa.OffloadArbiter(eligible_ms=1000, enter_n=10, exit_n=3, settle_n=0)
    for i in range(9):
      _ingest_set(arb, 100 + i, T0 + i * MS)
      self.assertIsNone(arb.select(100 + i, T0 + i * MS + MS), f"frame {i}: must not engage yet")
    self.assertFalse(arb.engaged)
    _ingest_set(arb, 109, T0 + 9 * MS)
    self.assertIsNotNone(arb.select(109, T0 + 9 * MS + MS))
    self.assertTrue(arb.engaged)
    self.assertEqual(arb.engage_count, 1)

  def test_miss_resets_consecutive_streak(self):
    arb = oa.OffloadArbiter(eligible_ms=46, enter_n=10, settle_n=0)
    for i in range(9):
      _ingest_set(arb, 100 + i, T0 + i * MS)
      arb.select(100 + i, T0 + i * MS + MS)
    self.assertEqual(arb.eligible_streak, 9)
    # one ineligible frame (its set is 100 ms old, past the 46 ms window) resets the streak
    arb.select(109, T0 + 9 * MS + 100 * MS)
    self.assertFalse(arb.engaged)
    self.assertEqual(arb.eligible_streak, 0)

  def test_disengage_after_three_consecutive_misses(self):
    arb = oa.OffloadArbiter(eligible_ms=1000, enter_n=1, exit_n=3, settle_n=0)
    _ingest_set(arb, 100, T0)
    self.assertIsNotNone(arb.select(100, T0 + MS))
    self.assertTrue(arb.engaged)
    self.assertEqual(arb.switches, 1)
    for i, fid in enumerate((200, 201, 202)):  # no sets for these frames -> all misses
      self.assertIsNone(arb.select(fid, T0 + MS))
      if i < 2:
        self.assertTrue(arb.engaged, "still engaged before the 3rd miss")
    self.assertFalse(arb.engaged)
    self.assertEqual(arb.disengage_count, 1)
    self.assertEqual(arb.switches, 2)

  def test_counters_shape(self):
    arb = oa.OffloadArbiter(enter_n=1, settle_n=0)
    _ingest_set(arb, 100, T0)
    arb.select(100, T0 + MS)
    st = arb.stats()
    for key in ("eligible", "ineligible", "engaged", "switches", "settle_skipped"):
      self.assertIn(key, st)
    self.assertTrue(st["engaged"])
    self.assertEqual(st["switches"], 1)


# ---------------------------------------------------------------------------
# 4. settling on engage
# ---------------------------------------------------------------------------

class TestSettling(_CodecPatched):
  def test_settle_rejects_frame_more_than_settle_n_behind(self):
    arb = oa.OffloadArbiter(eligible_ms=1000, enter_n=1, exit_n=3, settle_n=2)
    _ingest_set(arb, 100, T0)
    # engages on local frame 110; the only set (frame 100) is > SETTLE_N behind -> settle-skipped
    self.assertIsNone(arb.select(110, T0 + 5 * MS))
    self.assertTrue(arb.engaged)
    self.assertEqual(arb.settle_skipped, 1)

  def test_settle_accepts_set_within_settle_n(self):
    arb = oa.OffloadArbiter(eligible_ms=1000, enter_n=1, exit_n=3, settle_n=2)
    _ingest_set(arb, 100, T0)
    arb.select(110, T0 + 5 * MS)               # engage + settle-skip
    _ingest_set(arb, 109, T0 + 5 * MS)          # exactly SETTLE_N behind the next local frame 111
    self.assertIsNotNone(arb.select(111, T0 + 6 * MS))
    self.assertTrue(arb.engaged)


# ---------------------------------------------------------------------------
# 5. cohesion: never a mixed-source set within one frame
# ---------------------------------------------------------------------------

class TestCohesion(_CodecPatched):
  def test_modelv2_present_cameraodometry_absent_uses_local(self):
    arb = oa.OffloadArbiter(eligible_ms=1000, enter_n=1, exit_n=3, settle_n=0)
    for i in range(3):
      _ingest_set(arb, 100 + i, T0 + i * MS)
      self.assertIsNotNone(arb.select(100 + i, T0 + i * MS + MS))
    self.assertTrue(arb.engaged)
    # frame 110: modelV2 + drivingModelData + SP present, cameraOdometry missing
    arb.ingest(_SHADOW_OF["modelV2"], _raw("modelV2", 110), T0 + 10 * MS)
    arb.ingest(_SHADOW_OF["drivingModelData"], _raw("drivingModelData", 110), T0 + 10 * MS)
    arb.ingest("offloadModelDataV2SP", _raw("modelDataV2SP"), T0 + 10 * MS)
    self.assertIsNone(arb.select(110, T0 + 10 * MS + MS), "cohesion: never mix sources in one set")

  def test_sp_proximity_binding(self):
    arb = oa.OffloadArbiter(enter_n=1, settle_n=0, sp_proximity_ms=20)
    for real in oa.ID_SERVICES:
      arb.ingest(_SHADOW_OF[real], _raw(real, 100), T0)
    arb.ingest("offloadModelDataV2SP", _raw("modelDataV2SP"), T0 + 50 * MS)   # 50 ms away
    self.assertIsNone(arb.select(100, T0))
    self.assertFalse(arb.engaged)

  def test_cameraodometry_travels_with_frame(self):
    arb = oa.OffloadArbiter(enter_n=1, settle_n=0)
    _ingest_set(arb, 100, T0)
    out = arb.select(100, T0 + MS)
    self.assertIsNotNone(out)
    self.assertEqual(set(out.keys()), set(oa.REAL_SERVICES))
    self.assertEqual(out["modelV2"], _raw("modelV2", 100) + b"@" + str(T0 + MS).encode())
    self.assertEqual(out["cameraOdometry"], _raw("cameraOdometry", 100) + b"@" + str(T0 + MS).encode())


# ---------------------------------------------------------------------------
# 7. stale-cache purge
# ---------------------------------------------------------------------------

class TestPurge(_CodecPatched):
  def test_frame_older_than_horizon_dropped(self):
    arb = oa.OffloadArbiter(enter_n=1, settle_n=0, purge_s=2.0)
    _ingest_set(arb, 100, T0)
    self.assertIn(100, arb.frames)
    arb._purge(T0 + int(3.0 * 1e9))
    self.assertNotIn(100, arb.frames)
    self.assertIsNone(arb.select(100, T0 + int(3.0 * 1e9)))

  def test_max_frames_keeps_newest(self):
    arb = oa.OffloadArbiter(enter_n=1, settle_n=0, max_frames=8)
    for i in range(20):
      _ingest_set(arb, 100 + i, T0 + i * MS)
    arb._purge(T0 + 20 * MS)
    self.assertLessEqual(len(arb.frames), 8)
    self.assertIn(119, arb.frames)


# ---------------------------------------------------------------------------
# 6. socket-path restamp round-trip (real msgq transport, end-to-end)
# ---------------------------------------------------------------------------

def _load(data: bytes):
  return cereal_log.Event.from_bytes(data, traversal_limit_in_words=_TRAV)


def _logmono(raw: bytes) -> int:
  with _load(raw) as r:
    return int(r.logMonoTime)


def _norm_header(raw: bytes, hdr: int) -> bytes:
  with _load(raw) as r:
    b = r.as_builder()
    b.logMonoTime = hdr
    return b.to_bytes()


def _mk(svc: str, fid: int = 0) -> bytes:
  m = new_message(svc)
  m.valid = True
  if svc == "modelV2":
    m.modelV2.frameId = int(fid)
    m.modelV2.frameIdExtra = int(fid) + 1000
    m.modelV2.frameAge = 3
    m.modelV2.timestampEof = T0
  elif svc == "cameraOdometry":
    m.cameraOdometry.frameId = int(fid)
    m.cameraOdometry.timestampEof = T0
    m.cameraOdometry.trans = [1.0, 2.0, 3.0]
    m.cameraOdometry.rot = [0.1, 0.2, 0.3]
  elif svc == "drivingModelData":
    m.drivingModelData.frameId = int(fid)
    m.drivingModelData.frameIdExtra = int(fid) + 1000
  elif svc == "modelDataV2SP":
    m.modelDataV2SP.laneTurnDirection = "turnLeft"
    m.modelDataV2SP.leftLaneChangeEdgeBlock = True
  return m.to_bytes()


@needs_cereal
class TestSocketPath(unittest.TestCase):
  """PubMaster on the SHADOW names -> arbiter.poll() raw sockets -> publish under the REAL names."""

  def setUp(self):
    msgq.set_fake_prefix(f"arbtest-{os.getpid()}")
    self.arb = oa.OffloadArbiter(eligible_ms=1000, enter_n=1, exit_n=3, settle_n=0)
    self.arb.open()
    # publishers first (so the msgq segments exist), then subscribers, then let SUBSCRIBE propagate
    self.pm_shadow = PubMaster(list(oa.SHADOW_NAMES))
    self.pm_real = PubMaster(list(oa.REAL_SERVICES))
    self.real_subs = {
      name: msgq.sub_sock(name, addr="127.0.0.1", timeout=200, segment_size=SERVICE_LIST[name].queue_size)
      for name in oa.REAL_SERVICES
    }
    time.sleep(0.4)

  def tearDown(self):
    self.arb.close()
    for s in self.real_subs.values():
      try:
        s.close()
      except Exception:
        pass

  def _publish_set(self, fid, t_ns, timeout_s=6.0):
    """Resend the shadow set until the arbiter has cached the complete frame set.

    Returns the inbound payloads keyed by REAL service name (for the restamp comparison).
    """
    payloads = {
      "offloadModelV2": _mk("modelV2", fid),
      "offloadCameraOdometry": _mk("cameraOdometry", fid),
      "offloadDrivingModelData": _mk("drivingModelData", fid),
      "offloadModelDataV2SP": _mk("modelDataV2SP"),
    }
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
      for name, raw in payloads.items():
        self.pm_shadow.send(name, raw)
      self.arb.poll(t_ns)
      fs = self.arb.frames.get(fid)
      if fs is not None and self.arb._complete(fs):
        return {oa.SHADOW_TO_REAL[k]: v for k, v in payloads.items()}
      time.sleep(0.05)
    raise AssertionError("arbiter never cached the complete shadow set")

  def _recv_real(self, timeout_s=3.0):
    got = {}
    end = time.monotonic() + timeout_s
    while time.monotonic() < end and len(got) < len(oa.REAL_SERVICES):
      for name, sock in self.real_subs.items():
        if name in got:
          continue
        d = sock.receive()
        if d:
          got[name] = bytes(d)
      time.sleep(0.01)
    return got

  def test_socket_path_restamp_only_header(self):
    fid, now = 500, 987_654_321_000
    inbound = self._publish_set(fid, now)
    self.assertFalse(self.arb.engaged)                 # not engaged until select()
    out = self.arb.select(fid, now + MS)
    self.assertIsNotNone(out)
    self.assertTrue(self.arb.engaged)

    pm_real = self.pm_real
    for name in oa.REAL_SERVICES:
      pm_real.send(name, out[name])

    got = self._recv_real()
    self.assertEqual(set(got.keys()), set(oa.REAL_SERVICES), "all four real-name outputs must arrive")
    for name in oa.REAL_SERVICES:
      self.assertEqual(_logmono(got[name]), now + MS, f"{name}: header not restamped to device-now")
      self.assertEqual(_norm_header(got[name], 111), _norm_header(inbound[name], 111),
                       f"{name}: payload bytes changed (must differ ONLY in logMonoTime)")
      self.assertNotEqual(got[name], inbound[name], f"{name}: header must have changed")


if __name__ == "__main__":
  raise SystemExit(unittest.main(verbosity=2))
