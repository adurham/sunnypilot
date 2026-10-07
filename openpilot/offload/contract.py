"""FROZEN — shared contract for the offload pipeline (WS-A/B/C/D consume; PM owns).

Semantics (normative; see INTERFACES.md for rationale):

1. WIRE (device -> Mac, ZMQ): raw cereal Event bytes exactly as the device bridge emits them.
   Never re-stamped in flight. logMonoTime inside wire messages is DEVICE clock.

2. MAC-LOCAL ingest: any device-origin message republished into the Mac's local msgq
   MUST have the Event header logMonoTime re-stamped to the MAC's monotonic clock
   (time.monotonic_ns()); payload fields (frameId, timestampSof/Eof, encodeId, flags,
   and all sensor fields) are preserved untouched. This is what makes SubMaster
   .updated/.valid work on the Mac. All pacing/alignment logic on the Mac uses payload
   SOF deltas (clock-independent), never header times.

3. RETURN (Mac -> device, future drive mode): offloadd computes freshness from the device
   clock (now_device - sof_device); if too old -> drop (never republish stale). If fresh,
   re-stamp header logMonoTime to device monotonic now and inject into device msgq.
   Frame-id/drop/validity accounting for cameraOdometry stays device-side.
"""
from dataclasses import dataclass

# --- Service sets (frozen) ---------------------------------------------------

# Device -> Mac over ZMQ (Mac SUB connect-list; PUB only flows to subscribers).
FORWARD_VIDEO_SERVICES = ["narrowRoadEncodeData", "wideRoadEncodeData"]
FORWARD_SMALL_SERVICES = ["narrowRoadCameraState", "wideRoadCameraState", "carState", "deviceState",
                          "carControl", "extrinsicsCalibration", "driverMonitoringState", "lateralDelay"]
FORWARD_SHADOW_SERVICES = ["modelV2", "drivingModelData", "cameraOdometry", "modelDataV2SP"]
FORWARD_SERVICES = FORWARD_SMALL_SERVICES + FORWARD_SHADOW_SERVICES  # + video (separate port each)

# Mac -> device over ZMQ (future drive mode; code-complete in P2, unused in P1-P4).
RETURN_SERVICES = ["modelV2", "cameraOdometry", "drivingModelData", "modelDataV2SP"]

# Shadow names the device offloadd forwards remote outputs to (INTERFACES §7 join). Device modeld_v2
# consumes these; python-cereal only (no capnp change); the C++ bridge/loggerd never see them.
SHADOW_OUTPUT_SERVICES = ["offloadModelV2", "offloadCameraOdometry", "offloadDrivingModelData", "offloadModelDataV2SP"]

# Arbitration defaults (INTERFACES §7; env-tunable as OFFLOAD_*).
ELIGIBLE_MS = 46
ENTER_N = 10
EXIT_N = 3
SETTLE_N = 2

# Services the Mac republishes into its LOCAL msgq for modeld_v2's SubMaster.
# (modeld_v2 subscribes: deviceState, carState, narrowRoadCameraState, extrinsicsCalibration,
#  driverMonitoringState, carControl, lateralDelay. wideRoad* kept for the frame bridge.)
MAC_LOCAL_SERVICES = ["narrowRoadCameraState", "wideRoadCameraState", "carState", "deviceState",
                      "carControl", "extrinsicsCalibration", "driverMonitoringState", "lateralDelay"]

# VisionIPC server name the Mac modeld instance connects to.
DEFAULT_VIPC_SERVER = "camerad"


# --- Frame metadata ----------------------------------------------------------

@dataclass(slots=True)
class FrameEvent:
  """One camera frame as seen by the Mac frame path. All *_ns device-clock except recv_mac_ns."""
  frame_id: int
  encode_id: int
  timestamp_sof: int          # device ns
  timestamp_eof: int          # device ns
  is_keyframe: bool
  payload_bytes: int
  recv_mac_ns: int            # Mac monotonic ns at ZMQ receipt
  decode_ms: float = -1.0     # fill after VT decode
  publish_mac_ns: int = -1    # Mac monotonic ns at VisionIPC publish


@dataclass(slots=True)
class LatencyRecord:
  """One row of the P1 latency capture (per frame, joined by frame_id)."""
  frame_id: int
  cam: str                    # 'narrow' | 'wide'
  sof_dev_ns: int
  eof_dev_ns: int
  recv_mac_ns: int
  decode_done_mac_ns: int = -1
  vipc_publish_mac_ns: int = -1
  modelv2_mac_ns: int = -1
