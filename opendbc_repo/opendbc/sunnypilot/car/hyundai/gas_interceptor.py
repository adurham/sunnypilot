"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

comma pedal (gas interceptor) longitudinal for Hyundai CAN non-SCC ICE cars.

Driver-supervisory contract (there is NO brake actuator on these cars):
  - accel request >= 0  -> pedal command (throttle), capped at MAX_INTERCEPTOR_GAS
  - accel request <  0  -> pedal command drops toward 0 = lift-off/coast, engine braking only, NEVER service brakes
  - no stop-and-go: openpilot longitudinal is disengaged below minEnableSpeed; the driver must brake
"""
from collections import deque
from dataclasses import dataclass

import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.can_definitions import CanData
from opendbc.sunnypilot.car import create_gas_interceptor_command
from opendbc.sunnypilot.car.hyundai.values import HyundaiSafetyFlagsSP

GAS_INTERCEPTOR_DBC = "hyundai_gas_interceptor_generated"


@dataclass(frozen=True)
class GasInterceptorIDs:
  """One pedal CAN ID dialect: addresses + the pedal-DBC message names (identical signals) that carry them."""
  command_addr: int
  sensor_addr: int
  command_msg: str
  sensor_msg: str


# stock comma pedal firmware
STANDARD_IDS = GasInterceptorIDs(0x200, 0x201, "GAS_COMMAND", "GAS_SENSOR")
# custom pedal firmware with the IDs moved +0x500 (avoids 0x200 = EMS20 in the Hyundai DBC). Same payload/crc/counter.
REMAPPED_IDS = GasInterceptorIDs(0x700, 0x701, "GAS_COMMAND_R", "GAS_SENSOR_R")

# legacy aliases (standard dialect)
GAS_COMMAND_ADDR = STANDARD_IDS.command_addr
GAS_SENSOR_ADDR = STANDARD_IDS.sensor_addr


def get_interceptor_ids(CP_SP: structs.CarParamsSP) -> GasInterceptorIDs:
  """Active dialect. The safety param bit is the single source of truth, so panda and openpilot can't disagree."""
  return REMAPPED_IDS if CP_SP.safetyParam & HyundaiSafetyFlagsSP.GAS_INTERCEPTOR_REMAPPED else STANDARD_IDS

# The pedal parser/packer use a separate DBC (see hyundai_gas_interceptor.dbc for why), registered under this key in
# CarState.get_can_parsers so that CarInterfaceBase also folds the pedal into canValid (dead pedal -> CAN invalid).
GAS_INTERCEPTOR_BUS_KEY = Bus.alt

# MUST equal HYUNDAI_GAS_INTERCEPTOR_THRESHOLD in opendbc/safety/modes/hyundai.h (test_gas_interceptor.py enforces this).
# Raw GAS_SENSOR units: average of the two 12-bit ADC tracks of the DRIVER's pedal.
# Bench-measured (2026-10-03): foot-off rest A=465, B=241 -> average 353, noise +/-12. 420 = rest + 67 (~5x the noise
# band), i.e. ~4% pedal travel along the measured A/B curve, below the ~7% travel (A=620) where the car's own ECU
# already flags gas pressed. A too-low value fails safe (gas reads as pressed -> panda never allows longitudinal).
HYUNDAI_GAS_INTERCEPTOR_THRESHOLD = 420

# Cap on openpilot's pedal command, fraction 0..1 of pedal travel (DBC scaling is bench-measured: 0 = rest, 1 = full
# press; 0.15 -> track A 788 / track B 400 raw). 0.15 = 15% travel = mild accel; for reference the ECU already reports
# gas pressed at ~7% travel. TBD-BENCH-ROAD: conservative starting cap (half of upstream Toyota's 0.3); may be raised
# only after road validation.
# NOTE: this is intentionally NOT forced below the RX threshold in raw units. The comma pedal reports the driver's raw
# ADC on GAS_SENSOR and applies max(driver, openpilot) only at its DAC output, so openpilot's command can't latch
# gas_pressed (verified against panda c076a9f2^:board/pedal/main.c). Bench gate: ramp the command to this cap with the
# foot off and confirm GAS_SENSOR does not move. test_gas_interceptor.py pins the DBC mapping in raw counts.
MAX_INTERCEPTOR_GAS = 0.15

# TBD-BENCH: conservative. Pedal-long can't be engaged below this (NoEntry) and the pedal command is cut once speed
# falls GAS_CUT_HYSTERESIS below it, so there is no low-speed/stop-and-go throttle. Driver brakes to stop.
HYUNDAI_GAS_INTERCEPTOR_MIN_ENABLE_SPEED = 25. * CV.MPH_TO_MS
GAS_CUT_HYSTERESIS = 2.0  # m/s

# TBD-BENCH: accel (m/s^2) -> pedal fraction gain, and a speed-dependent "hold speed" feedforward (m/s^2 equivalent)
# that offsets drag/rolling resistance so a 0 m/s^2 request holds speed instead of coasting. Both are first guesses.
PEDAL_SCALE = 0.3
HOLD_SPEED_OFFSET_BP = [0., 10., 20., 30.]   # vEgo, m/s
HOLD_SPEED_OFFSET_V = [0., 0.15, 0.3, 0.45]  # m/s^2 equivalent


def get_pedal_command(accel: float, v_ego: float) -> float:
  """accel request -> pedal fraction. Accelerator only: <= 0 means coast, never brake."""
  offset = float(np.interp(v_ego, HOLD_SPEED_OFFSET_BP, HOLD_SPEED_OFFSET_V))
  return float(np.clip(PEDAL_SCALE * (accel + offset), 0., MAX_INTERCEPTOR_GAS))


def get_interceptor_gas(cp, sensor_msg: str = STANDARD_IDS.sensor_msg) -> float:
  return (cp.vl[sensor_msg]["INTERCEPTOR_GAS"] + cp.vl[sensor_msg]["INTERCEPTOR_GAS2"]) // 2


# *** pedal fault latch recovery (route 11c forensics, FORK.md #14) ***
# The pedal firmware latches ANY fault (GAS_SENSOR STATE != 0, e.g. 3 = FAULT_SCE from a CAN error frame) and then
# ignores every ENABLE=1 command (passthrough = no openpilot throttle). Only a valid ENABLE=0 / all-zero frame clears it.
# While commanding we therefore spend ONE send slot on a zero frame when the pedal reports a fault. It costs no torque:
# a faulted pedal is already in passthrough. Frame units are the 100 Hz carcontroller frame (DT_CTRL).
# Holdoff: zero frame -> 0x701 with STATE 0 measured 16-26 ms on the road, + <=10 ms until CarState sees it; 60 ms
# bounds re-clearing on stale state to at most one extra clear.
PEDAL_CLEAR_HOLDOFF_FRAMES = 6         # >= 60 ms between clear frames
# Escalation: the clear isn't working -> tell the driver throttle authority is gone (accFaulted -> "Cruise Fault").
PEDAL_FAULT_TIMEOUT_FRAMES = 50        # fault continuously visible while commanding for > 0.5 s
PEDAL_MAX_CLEARS = 8                   # more than this many clears ...
PEDAL_CLEAR_WINDOW_FRAMES = 200        # ... within 2 s


class PedalFaultMonitor:
  """Clear-frame policy + escalation. Owned by CarStateExt (which also reads `escalated` into accFaulted) and stepped
  by GasInterceptorCarController once per controller frame; inert unless the gas interceptor is enabled."""
  def __init__(self):
    self.last_clear_frame: int | None = None
    self.clear_frames: deque[int] = deque()
    self.fault_start_frame: int | None = None
    self.escalated = False

  def reset(self) -> None:
    self.fault_start_frame = None
    self.clear_frames.clear()
    self.escalated = False

  def update(self, frame: int, active: bool, state: int, send_slot: bool, gas: float) -> bool:
    """-> True if this send slot must carry the zero (clear) frame instead of `gas`."""
    if not active:
      # not commanding: the stream is all zero frames, which clear any fault by themselves. Re-arm.
      self.reset()
      return False

    if state == 0:
      self.fault_start_frame = None
    elif self.fault_start_frame is None:
      self.fault_start_frame = frame

    while self.clear_frames and frame - self.clear_frames[0] >= PEDAL_CLEAR_WINDOW_FRAMES:
      self.clear_frames.popleft()

    send_clear = (send_slot and state != 0 and gas > 0. and
                  (self.last_clear_frame is None or frame - self.last_clear_frame >= PEDAL_CLEAR_HOLDOFF_FRAMES))
    if send_clear:
      self.last_clear_frame = frame
      self.clear_frames.append(frame)

    if (self.fault_start_frame is not None and frame - self.fault_start_frame > PEDAL_FAULT_TIMEOUT_FRAMES) or \
       len(self.clear_frames) > PEDAL_MAX_CLEARS:
      self.escalated = True  # held until longitudinal disengages (active -> False)
    return send_clear


# *** bursted CLU11 CANCEL (gas interceptor mode only) ***
# Our CANCEL CLU11 (0x4F1) shares the ID with the cluster's own 50 Hz CLU11. Sent every 10 ms frame, the two collide
# (panda bit1Errors/bus-offs, 100 % inside cancel streams on route 11c) and every error frame can latch the pedal's
# FAULT_SCE. Send short bursts with pauses instead: CANCEL_BURST_FRAMES frames, then a pause that rotates through
# CANCEL_PAUSE_FRAMES (150/200/250 ms: a fixed period could phase-lock with the cluster's 20 ms schedule).
CANCEL_BURST_FRAMES = 5
CANCEL_PAUSE_FRAMES = (15, 20, 25)
_CANCEL_CYCLE = tuple((CANCEL_BURST_FRAMES, p) for p in CANCEL_PAUSE_FRAMES)
CANCEL_CYCLE_FRAMES = sum(b + p for b, p in _CANCEL_CYCLE)


def cancel_burst_active(n: int) -> bool:
  """n = frames since the cancel button send became due (0 = first frame). True -> send CANCEL on this frame."""
  k = n % CANCEL_CYCLE_FRAMES
  for burst, pause in _CANCEL_CYCLE:
    if k < burst:
      return True
    k -= burst + pause
    if k < 0:
      return False
  return False  # unreachable


class GasInterceptorCarController:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP, dbc_names: dict):
    self.CP = CP
    self.CP_SP = CP_SP

    self.gas = 0.
    self.interceptor_ids = get_interceptor_ids(CP_SP)
    self.interceptor_packer = CANPacker(dbc_names[GAS_INTERCEPTOR_BUS_KEY]) if CP_SP.enableGasInterceptor else None

  def create_gas_command(self, CC: structs.CarControl, CS, frame: int) -> list[CanData]:
    can_sends: list[CanData] = []
    if not self.CP_SP.enableGasInterceptor:
      return can_sends

    # send exactly zero when not actively controlling: the pedal outputs max(driver, command), and zero + ENABLE=0
    # also clears any latched pedal fault (pedal firmware)
    active = CC.longActive and CS.out.vEgo >= self.CP.minEnableSpeed - GAS_CUT_HYSTERESIS
    self.gas = get_pedal_command(CC.actuators.accel, CS.out.vEgo) if active else 0.

    # 50 Hz; the pedal only accepts frames whose counter is exactly previous + 1
    send_slot = frame % 2 == 0
    # pedal reports a latched fault while we command -> this slot carries the clear (zero) frame instead
    # (counter stays continuous: the clear frame simply occupies the slot)
    clear = CS.pedal_fault_monitor.update(frame, active, CS.interceptor_state, send_slot, self.gas)
    if send_slot:
      can_sends.append(create_gas_interceptor_command(self.interceptor_packer, 0. if clear else self.gas, frame // 2,
                                                     self.interceptor_ids.command_msg))

    return can_sends
