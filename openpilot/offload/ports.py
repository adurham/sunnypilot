"""FROZEN — ZMQ endpoint scheme for the cereal bridge (parity with openpilot/cereal/messaging/bridge_zmq.cc).

The device's ./bridge publishes every msgq service on tcp://<bind-ip>:8023 + fnv1a(name) % (65535 - 8023).
A ZMQ SUB (Mac) connects to the same port on the device IP; ZMQ PUB only sends to connected SUBs,
so the Mac's connect-list effectively selects the service set.

Do not edit without updating INTERFACES.md and every consumer (WS-B, WS-C, WS-D).
"""
import sys

FNV_OFFSET = 0xcbf29ce484222325
FNV_PRIME = 0x100000001b3
START_PORT = 8023
MAX_PORT = 65535


def fnv1a(s: str) -> int:
  h = FNV_OFFSET
  for ch in s.encode():
    h ^= ch
    h = (h * FNV_PRIME) & 0xFFFFFFFFFFFFFFFF
  return h


def get_port(service: str) -> int:
  return START_PORT + fnv1a(service) % (MAX_PORT - START_PORT)


def endpoint(service: str, host: str) -> str:
  return f"tcp://{host}:{get_port(service)}"


# Live-verified on comma-b203ed6e 2026-10-06 (do not change; test asserts these).
ANCHORS = {
  "livestreamNarrowRoadEncodeData": 8475,
  "modelV2": 58537,
  "cameraOdometry": 50972,
  "drivingModelData": 49244,
  "narrowRoadEncodeData": 52737,
  "wideRoadEncodeData": 42305,
  "narrowRoadCameraState": 20911,
  "wideRoadCameraState": 53095,
}


def _self_test() -> int:
  bad = {k: (get_port(k), v) for k, v in ANCHORS.items() if get_port(k) != v}
  if bad:
    print("PORT SCHEME DRIFT:", bad)
    return 1
  print("ports.py self-test PASS:", len(ANCHORS), "anchors match")
  return 0


if __name__ == "__main__":
  sys.exit(_self_test())
