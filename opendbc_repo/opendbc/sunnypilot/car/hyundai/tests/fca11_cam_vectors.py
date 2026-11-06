"""0053: the SHARED camera-classification vector table for the FCA11 camera hand-back.

ONE table, consumed by BOTH layers' tests: the car layer (fca11_long.camera_braking, test_fca11_long.py) and the panda
(libsafety hyundai_fca11_long_cam_step, safety/tests/test_hyundai.py). Neither side hand-writes bit tests: every vector's
EXPECTED class is derived from DBC SIGNAL NAMES (hyundai_can.dbc BO_ 909 FCA11), never from the masks under test, so a
wrong mask in values.py / hyundai.h fails here instead of agreeing with itself.

  BRAKING class (hand back):  CF_VSM_HBACmd, CF_VSM_BeltCmd, CR_VSM_DecCmd, FCA_CmdAct, FCA_StopReq, CF_VSM_DecCmdAct
  WARNING class (keep braking): CF_VSM_Prefill, CF_VSM_Warn
  NEUTRAL (never a class): FCA_Status, PAINT1_Status, FCA_DrvSetStatus, FCA_Failinfo, CR_FCA_Alive,
                           FCA_RelativeVelocity, FCA_TimetoCollision, CR_FCA_ChkSum, and the DBC-undefined bits

Each vector is (name, 8 data bytes, braking: bool, warning: bool). The frames are built in the camera's real idle shape
(FCA_Status 2, DrvSetStatus 1, RelVel/TTC idle, the always-set undefined byte4 bit7, valid CRC) with exactly the named
field(s) changed, plus REAL camera frames recorded on routes 14d / 15b (the two incidents).
"""
import os
import re

from opendbc.car.hyundai.hyundaican import hyundai_checksum

BRAKING_FIELDS = ("CF_VSM_HBACmd", "CF_VSM_BeltCmd", "CR_VSM_DecCmd", "FCA_CmdAct", "FCA_StopReq", "CF_VSM_DecCmdAct")
WARNING_FIELDS = ("CF_VSM_Prefill", "CF_VSM_Warn")
NEUTRAL_FIELDS = ("FCA_Status", "PAINT1_Status", "FCA_DrvSetStatus", "FCA_Failinfo", "CR_FCA_Alive",
                  "FCA_RelativeVelocity", "FCA_TimetoCollision")

_DBC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..", "dbc", "generator", "hyundai",
                    "hyundai_can.dbc")


def dbc_fca11_layout() -> dict[str, tuple[int, int]]:
  """{signal: (start_bit, length)} for BO_ 909 FCA11, parsed straight from the DBC (Intel / @1 signals only)."""
  with open(_DBC) as f:
    src = f.read()
  m = re.search(r"^BO_ 909 FCA11:.*?\n((?: SG_ .*\n)+)", src, re.MULTILINE)
  assert m is not None, "BO_ 909 FCA11 not found in hyundai_can.dbc"
  out = {}
  for sm in re.finditer(r"SG_ (\w+) : (\d+)\|(\d+)@1", m.group(1)):
    out[sm.group(1)] = (int(sm.group(2)), int(sm.group(3)))
  return out


def field_mask(fields) -> int:
  """The bit mask (over the frame's first 4 bytes, little-endian) of the named DBC signals."""
  lay = dbc_fca11_layout()
  mask = 0
  for name in fields:
    start, length = lay[name]
    assert start + length <= 32, f"{name} is outside bytes 0..3"
    mask |= ((1 << length) - 1) << start
  return mask


def frame(**fields) -> bytes:
  """A camera 0x38D in the real idle shape with the given DBC fields set (raw values), CRC fixed."""
  lay = dbc_fca11_layout()
  vals = {"FCA_Status": 2, "FCA_DrvSetStatus": 1, "FCA_RelativeVelocity": 255, "FCA_TimetoCollision": 254,
          "CR_FCA_Alive": 5}
  vals.update(fields)
  b = bytearray(8)
  for name, v in vals.items():
    start, length = lay[name]
    for i in range(length):
      pos = start + i
      b[pos // 8] |= ((int(v) >> i) & 1) << (pos % 8)
  b[4] |= 0x80            # byte4 bit7: undefined, always 1 on this car
  b[7] = hyundai_checksum(bytes(b[:7]))
  return bytes(b)


# REAL bus-2 camera frames from the two incidents (rlog hex, chosen by recorded_camera_frames() below and frozen here so
# the tests need no logs). Expected class is from decoding them by DBC field name, not from the masks.
RECORDED: tuple[tuple[str, str], ...] = (
  # every distinct warning-class shape on the two incident routes + one idle frame each (harvest_cam_frames.py).
  # NOTE the warning frames also carry byte3 bit 29 (DBC-undefined) - it must NOT be braking class.
  ("14d idle", "00000800887ffee0"),
  ("14d warning byte0=0x08 (Warn 1)", "08000820f07ffe3c"),
  ("15b idle", "00000800907ffef4"),
  ("15b warning byte0=0x09 (Warn 1 + Prefill)", "09000820f07ffe61"),
  ("15b warning byte0=0x08 (Warn 1)", "08000820d87ffe00"),
)


def _classify_by_fields(dat: bytes) -> tuple[bool, bool]:
  lay = dbc_fca11_layout()
  val = int.from_bytes(dat, "little")

  def get(name):
    s, n = lay[name]
    return (val >> s) & ((1 << n) - 1)
  return any(get(f) for f in BRAKING_FIELDS), any(get(f) for f in WARNING_FIELDS)


def vectors() -> list[tuple[str, bytes, bool, bool]]:
  lay = dbc_fca11_layout()
  out: list[tuple[str, bytes, bool, bool]] = [("idle", frame(), False, False)]
  # every BRAKING field: each single bit on its own (so each bit is proven to hand back), plus its max value
  for name in BRAKING_FIELDS:
    _s, n = lay[name]
    for i in range(n):
      out.append((f"{name}=bit{i}", frame(**{name: 1 << i}), True, False))
    out.append((f"{name}=max", frame(**{name: (1 << n) - 1}), True, False))
  # every WARNING field alone: never a hand-back
  for name in WARNING_FIELDS:
    _s, n = lay[name]
    for v in range(1, 1 << n):
      out.append((f"{name}={v}", frame(**{name: v}), False, True))
  out.append(("warn3+prefill (the camera's FCW lead-in)", frame(CF_VSM_Warn=3, CF_VSM_Prefill=1), False, True))
  out.append(("warn2+prefill", frame(CF_VSM_Warn=2, CF_VSM_Prefill=1), False, True))
  # NEUTRAL fields at their max: never a class
  for name in NEUTRAL_FIELDS:
    _s, n = lay[name]
    out.append((f"{name}=max", frame(**{name: (1 << n) - 1}), False, False))
  # mixed: a warning PLUS a braking field is braking (the hand-back wins)
  out.append(("warn+dec", frame(CF_VSM_Warn=2, CR_VSM_DecCmd=10), True, True))
  out.append(("prefill+cmdact", frame(CF_VSM_Prefill=1, FCA_CmdAct=1), True, True))
  out.append(("variant-B (our own actuating shape)", frame(CF_VSM_Warn=3, CF_VSM_Prefill=1, FCA_CmdAct=1, CR_VSM_DecCmd=20),
              True, True))
  for name, hx in RECORDED:
    dat = bytes.fromhex(hx)
    brk, wrn = _classify_by_fields(dat)
    out.append((f"recorded:{name}", dat, brk, wrn))
  return out
