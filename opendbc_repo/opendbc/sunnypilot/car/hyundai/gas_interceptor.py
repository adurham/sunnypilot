"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

comma pedal (gas interceptor) longitudinal for Hyundai CAN non-SCC ICE cars.

Driver-supervisory contract (there is NO brake actuator on these cars):
  - accel request >= 0  -> pedal command (throttle), capped at MAX_INTERCEPTOR_GAS
  - accel request <  0  -> pedal command drops toward 0 = lift-off/coast, engine braking only, NEVER service brakes
  - No engage floor (minEnableSpeed = -1, like upstream Toyota interceptor cars): the pause/resume button engages at
    ANY speed, including a standstill. The up/down arrows only change the set speed and never engage (buttons-v3).
    Once engaged the throttle is never cut at low speed; it is only limited by LOW_SPEED_MAX_GAS (a throttle-strength
    launch limit, not an engagement floor). openpilot can still never brake: the driver stops the car.
"""
from collections import deque
from dataclasses import dataclass

import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.can_definitions import CanData
from opendbc.sunnypilot.car import create_gas_interceptor_command
from opendbc.sunnypilot.car.hyundai.fca11_long import Fca11LongBrake
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
# press; 0.35 -> track A 1218 / track B 612 raw, on the measured B = 0.494*A + 10.5 track line). Road-validated
# (route 00000123, drive-123 report): the old 0.15 cap
# was the binding limit 51% of engaged time and could not hold speed on flat ground above ~22 m/s (measured hold
# command 0.124 at 20 m/s) or on a ~4% climb (speed fell 19.9 -> 17.1 m/s pinned at 0.15 while the planner asked for
# +1.0 m/s^2, unclipped command 0.37). 0.30 = the driver's own p90 pedal while driving this car (p99 0.33), the same
# as upstream Toyota's interceptor cap. Per the road fit it holds speed on up to ~4.9% grade at 20 m/s, ~2.8% at 25 m/s
# (vs ~0.9% / none at 0.15). Panda enforces the same ceiling in raw counts
# (HYUNDAI_GAS_INTERCEPTOR_MAX_GAS_A/_B, safety/modes/hyundai.h; test_gas_interceptor.py pins them to this value).
# NOTE: this is intentionally NOT forced below the RX threshold in raw units. The comma pedal reports the driver's raw
# ADC on GAS_SENSOR and applies max(driver, openpilot) only at its DAC output, so openpilot's command can't latch
# gas_pressed (verified against panda c076a9f2^:board/pedal/main.c). Bench gate: ramp the command to this cap with the
# foot off and confirm GAS_SENSOR does not move. test_gas_interceptor.py pins the DBC mapping in raw counts.
# Raised 0.30 -> 0.35 (owner-approved, drive-128 buttons report §5): on routes 127/128 the cap bound on 21 % of the
# no-lead on-ramp samples and the driver's own merge pedal was 0.32-0.345 (p99 while driving 0.35). Panda ceiling moved
# with it (A 1110/B 559 -> A 1218/B 612).
MAX_INTERCEPTOR_GAS = 0.35

# *** personality (adurham fork): the driver's LongitudinalPersonality is the master feel dial for the pedal law ***
# The openpilot personality enum (cereal log.LongitudinalPersonality) is 0 = aggressive, 1 = standard, 2 = relaxed.
# This module is opendbc and must NOT import cereal, so the mapping is pinned here as plain ints (the sunnypilot
# CarControlSP.personality field carries the enum's raw value straight through).
PERSONALITY_AGGRESSIVE = 0
PERSONALITY_STANDARD = 1
PERSONALITY_RELAXED = 2

# The whole pedal law is personality-scaled by one factor per tier; STANDARD reproduces the shipped law bit-for-bit
# (every factor 1.0), so a drive that never touches the personality button is byte-copied from before this change.
# relaxed is 85 % of standard, standard 100 %, aggressive 120 %. The launch ceiling, the cruise-pull gain, the hold
# feedforward and the interceptor cap are all driver-felt; scaling them together keeps the pedal-authority story
# "relaxed is gentler, aggressive bites harder", and the 0.35 interceptor cap (the pedal firmware ceiling) is the hard
# stop only relaxed/standard ever reach, aggressive pins on it.
PERSONALITY_SCALE_BP = [PERSONALITY_AGGRESSIVE, PERSONALITY_STANDARD, PERSONALITY_RELAXED]  # indexes 0,1,2
PERSONALITY_SCALE_V = [1.20, 1.00, 0.85]
# Cap on the personality scale so a future table edit can never push a raw command past the pedal firmware ceiling
# (HYUNDAI_GAS_INTERCEPTOR_MAX_GAS_A/B, panda raw A 1218 / B 612 = the 0.35 command; see test_gas_interceptor). The law
# additionally clips to MAX_INTERCEPTOR_GAS below, so raising this constant alone is never enough to exceed the cap.
MAX_PERSONALITY_SCALE = MAX_INTERCEPTOR_GAS / 0.30  # 1.1667: 0.30 * 1.1667 = 0.35


def personality_scale(personality) -> float:
  """Feel multiplier for a LongitudinalPersonality raw value. Unknown / missing -> STANDARD (never the un-scaled 1.2)."""
  try:
    pid = int(getattr(personality, 'raw', personality))
  except (TypeError, ValueError):
    pid = PERSONALITY_STANDARD
  scale = PERSONALITY_SCALE_V[pid] if 0 <= pid < len(PERSONALITY_SCALE_V) else PERSONALITY_SCALE_V[PERSONALITY_STANDARD]
  return float(min(scale, MAX_PERSONALITY_SCALE))


# Speed at which the launch limit (LOW_SPEED_MAX_GAS below) reaches the full cap. Only a throttle-strength limit: it is NOT
# an engage floor (there is none in pedal mode, CP.minEnableSpeed = -1) and nothing disengages below it.
LOW_SPEED_FULL_GAS_SPEED = 25. * CV.MPH_TO_MS

# accel (m/s^2) -> pedal fraction law: cmd = pedal_scale(v) * accel + hold_cmd(v), clipped to [0, launch ceiling(v)].
#
# History: route 00000123 fit a single highway gain, aEgo(t+0.5 s) = 2.76*cmd + ..., PEDAL_SCALE = 1/2.76 = 0.36, with a
# speed-dependent hold feedforward in m/s^2 (refit on routes 127+128, speed axis vEgo*1.0125 = wheelSpeedFactor).
# Drives 12e/12f (car-features/drive-12e-12f-report.md §A) showed the scalar gain is correct only above ~22-25 m/s: the
# measured gain (aEgo per unit pedal, steady pedal, driver + openpilot fits) is 13.0 @1-3, 9.7 @3-6, 8.25/6.7 @6-9,
# 6.1/5.8 @9-12, 5.05/5.6 @12-15, 3.8/4.0 @15-20, 3.2 @20-25, 2.9 @25-30 m/s. With 0.36 everywhere the launch command was
# simply the launch ceiling (an open-loop throttle ramp: more speed -> more ceiling -> more speed) and, once it opened at
# 8-12 m/s, aEgo hit 2.1-2.5 m/s^2 for a 1.15-1.3 request: the pull-away "surge". The longitudinal PID is P/I = 0, so
# nothing closes the loop on aEgo; the feedforward gain has to be right.
# Gains are ~10-20 % below the fitted 1/G (margin: a too-low gain under-delivers, it never surges). From 22 m/s up the
# gain is the old 0.36, and from 20 m/s up the hold command is exactly 0.36 * the old m/s^2 hold table, so the highway
# law (following, set-speed hold, on-ramp merge at >= 22 m/s) is bit-for-bit the previous one at >= 22 m/s.
PEDAL_SCALE_HIGHWAY = 0.36
PEDAL_SCALE_BP = [0., 3., 6., 9., 12., 15., 18., 22.]                  # vEgo, m/s
PEDAL_SCALE_V = [0.10, 0.10, 0.12, 0.14, 0.165, 0.20, 0.25, PEDAL_SCALE_HIGHWAY]  # pedal fraction per m/s^2

# Hold command (pedal fraction at accel = 0). The standstill anchor is 0: this car has gas-only authority, so a
# zero/negative request at 0 m/s must command EXACTLY 0 pedal (any hold here is uncommanded motion), and 0-5 m/s ramps
# linearly from 0 to the 12e/12f fitted 0.08. From 5 m/s: 12e/12f steady-pedal fit (0.08 @5-10, 0.09 @12-15, 0.11-0.12
# @15-20 m/s). From 20 m/s: PEDAL_SCALE_HIGHWAY * the route-127/128 hold table in m/s^2 (0.34 @20, 0.43 @25, 0.49 @30,
# 0.56 @35), i.e. 0.1224/0.1548/0.1764/0.2016, unchanged.
HOLD_SPEED_OFFSET_HIGHWAY_V = [0.34, 0.43, 0.49, 0.56]                 # m/s^2 equivalent at 20/25/30/35 m/s
HOLD_CMD_BP = [0., 5., 10., 15., 20., 25., 30., 35.]                  # vEgo, m/s
HOLD_CMD_V = [0., 0.08, 0.085, 0.095, *[PEDAL_SCALE_HIGHWAY * o for o in HOLD_SPEED_OFFSET_HIGHWAY_V]]  # pedal fraction

# Low-speed ceiling on the command (openpilot side; panda keeps its flat raw ceiling at the cap, it has no vEgo). The
# pedal is far stronger at low speed (driver data 127+128, aEgo(+0.3 s) per unit pedal: 0-1 m/s 10.9, 1-3 13.2, 3-6
# 10.0, 6-10 7.9, 10-14 6.3 vs 2.76 at highway; driver's own launch pedal p50 0.18-0.21). 0.12 at a standstill = ~0.6
# m/s^2, 0.20 at 5 m/s = ~1.3 m/s^2: a gentle green-light launch, never a full-cap (~3 m/s^2) jump. Full cap from 25 mph
# (11.18 m/s). With the speed-scheduled gain this is a bound, no longer the active limiter above ~6 m/s (12e/12f: the
# command sat on it 72-94 % of every launch with the scalar gain).
# Personality tiers (adurham fork, drive 149 §2): the launch ceiling was the dominant "slow off the line" lever and was
# personality-independent. The tiers must satisfy relaxed < standard < aggressive (owner directive); STANDARD keeps the
# shipped 0.12/0.20 exactly (so the default-personality drive is byte-identical and the change is a pure personality
# spread), relaxed sits below it (the owner's older ~10 % backlog intent), aggressive above (drive 149 "next lever").
# All tiers are pinned at the 0.35 interceptor cap from 25 mph. Ceiling(v) <= MAX_INTERCEPTOR_GAS always (aggressive's
# 0.26 @5 m/s is below the cap, so only the cap binds at 25 mph).
LOW_SPEED_MAX_GAS_BP = [0., 5., LOW_SPEED_FULL_GAS_SPEED]  # vEgo, m/s (full cap at 25 mph)
LOW_SPEED_MAX_GAS_V = [0.12, 0.20, MAX_INTERCEPTOR_GAS]     # pedal fraction (standard tier; shipped values)
LOW_SPEED_MAX_GAS_V_OFFSET = {
  PERSONALITY_RELAXED: [-0.02, -0.03],   # 0.10 @0, 0.17 @5 m/s
  PERSONALITY_STANDARD: [0.00, 0.00],    # 0.12 @0, 0.20 @5 m/s (shipped)
  PERSONALITY_AGGRESSIVE: [0.04, 0.06],  # 0.16 @0, 0.26 @5 m/s
}

# Upward slew limit on the pedal command, as a JERK limit in accel units: rate_up(v) = PEDAL_JERK_UP * pedal_scale(v)
# pedal fraction per second (0.20/s at a standstill, 0.33/s at 12 m/s, 0.50/s at 18 m/s, 0.72/s from 22 m/s, i.e.
# about the old fixed 0.75/s on the highway). The old fixed 0.75/s was 0 -> cap in 0.47 s at any speed = +1.0 m/s^2 in
# ~0.5 s on the 11-19 m/s rolling engagements of 12e/12f (the "jump to max"); 2.0 m/s^3 is the planner's own comfort
# jerk scale. Decreases stay immediate: there are no brakes, lifting must never be delayed. Units: 100 Hz controller
# frames (DT_CTRL).
PEDAL_JERK_UP = 2.0  # m/s^3

# *** 0039 (owner requirement 5): launch-specific fast ramp ***
# PEDAL_JERK_UP alone gives rate_up(0) = 2.0 * 0.10 = 0.20 pedal-fraction/s, so from rest the command walks
# 0.02 -> 0.10 -> 0.20 over ~1.5 s (0.12 rest ceiling in ~0.6 s, 0.20 at 5 m/s in ~3.9 s): the car feels asleep
# off a green light - the owner's "at 2 %, then slowly ramps to 10, then 15, then 20". The ceiling is NOT the
# problem and stays untouched. Below LAUNCH_RAMP_V_MAX a genuine pull-away uses a flat LAUNCH_RAMP_RATE_UP,
# reaching the 0.12 rest ceiling in ~0.2 s; above it the gentler PEDAL_JERK_UP rate is byte-identical to before.
# Why this cannot bring back the 12e/12f surge: the old fixed 0.75/s was removed because at 11-19 m/s (rolling
# re-engagements) the ceiling is already high (0.35 by 11.2 m/s) so it jumped the command to the cap in 0.47 s
# (+1.0 m/s^2). Here the fast branch is hard-gated to v <= LAUNCH_RAMP_V_MAX = 3 m/s (3.7x below that band), and
# at 3 m/s the low-speed ceiling is only 0.168, so the fast rate can never reach a high command. The limiter
# only bounds the RISE; a zero/negative request still lifts the very next frame (decreases stay immediate), so
# a fast RISE is never a fast STOP. The rate is not personality-scaled (the rise limit never was): every tier
# gets the same launch latency, and 0039 therefore changes STANDARD as well as relaxed/aggressive. This is a
# deliberate, tier-independent latency change, not a personality change: all magnitude levers (per-tier
# ceiling table, MAX_INTERCEPTOR_GAS, hold, down behaviour) are untouched.
LAUNCH_RAMP_V_MAX = 3.0      # m/s; at or below this a launch from (near) rest is in progress
LAUNCH_RAMP_RATE_UP = 0.6   # pedal fraction per second; 0 -> the 0.12 rest ceiling in 0.2 s


def get_personality_ceiling(v_ego: float, personality=PERSONALITY_STANDARD) -> float:
  """Launch ceiling (pedal fraction) for this personality at this speed."""
  offset = LOW_SPEED_MAX_GAS_V_OFFSET.get(int(personality), LOW_SPEED_MAX_GAS_V_OFFSET[PERSONALITY_STANDARD])
  vals = [LOW_SPEED_MAX_GAS_V[0] + offset[0], LOW_SPEED_MAX_GAS_V[1] + offset[1], LOW_SPEED_MAX_GAS_V[2]]
  return min(MAX_INTERCEPTOR_GAS, float(np.interp(v_ego, LOW_SPEED_MAX_GAS_BP, vals)))


def get_pedal_scale(v_ego: float, personality=PERSONALITY_STANDARD) -> float:
  """Pedal fraction per m/s^2 at this speed, scaled by personality."""
  return personality_scale(personality) * float(np.interp(v_ego, PEDAL_SCALE_BP, PEDAL_SCALE_V))


def get_hold_command(v_ego: float, personality=PERSONALITY_STANDARD) -> float:
  """Pedal fraction that holds speed on flat ground (accel request 0). This is a PHYSICS feedforward
  (road-load / aero+rolling resistance), not a feel dial: it replaces a zero integral term (LongControl
  ki=0), so any personality scaling of it is an uncorrected bias - aggressive would track above its own
  plan, relaxed below. Personality-invariant since G3; `personality` is kept in the signature for call
  compatibility but no longer scales the hold. (personality_scale still applies to the pull gain and the
  per-tier launch ceilings, which ARE driver-felt.)"""
  return float(np.interp(v_ego, HOLD_CMD_BP, HOLD_CMD_V))


def get_pedal_rate_up(v_ego: float, launching: bool = False) -> float:
  """Upward slew limit, pedal fraction per second. Not personality-scaled: the rise limit is the same for every tier
  (personality changes where the ramp lands, not how fast it climbs).

  launching=True marks a pull-away from (near) rest: below LAUNCH_RAMP_V_MAX it returns the flat launch rate so the
  command reaches its (unchanged) low-speed ceiling promptly. At or above LAUNCH_RAMP_V_MAX - and whenever launching is
  False - it is the exact pre-0039 jerk rate, so a rolling re-engagement is never fast-ramped."""
  if launching and v_ego <= LAUNCH_RAMP_V_MAX:
    return LAUNCH_RAMP_RATE_UP
  return PEDAL_JERK_UP * float(np.interp(v_ego, PEDAL_SCALE_BP, PEDAL_SCALE_V))


def get_pedal_command(accel: float, v_ego: float, personality=PERSONALITY_STANDARD) -> float:
  """accel request -> pedal fraction. Accelerator only: <= 0 means coast, never brake."""
  return float(np.clip(get_pedal_scale(v_ego, personality) * accel + get_hold_command(v_ego, personality),
                       0., get_personality_ceiling(v_ego, personality)))


# *** over-speed gas release (route 16a, measured) ***
# When the car is ABOVE the cruise set speed and the planner still asks accel <= 0, the hold feedforward must NOT
# apply. Route 16a: the owner lowered the set speed 88->80->72 km/h while at 80.6 km/h; for ~20 s the plan asked
# aTarget ~0 (mean +0.022, min -0.027 m/s^2) but hold_cmd(v) is added unconditionally at accel 0, so the
# interceptor commanded ~13.5 % pedal -> ~8 % engine throttle -> the car HELD 78 km/h instead of coasting. The
# owner wants a slow deceleration / engine braking there. The release must be an EXACT zero, not a small offset:
# pass-through (interceptor idle, foot off, 40-70 km/h) coasts at a median -1.33 m/s^2, and ANY live command
# (even 2 raw counts) collapses that to about -0.05 m/s^2. A gas amount of exactly 0.0 is below the
# create_gas_interceptor_command 0.001 enable threshold, so the wire carries the all-zero ENABLE=0 frame and the
# pedal passes the driver's own signal through.
OVERSPEED_RELEASE_MARGIN = 0.5   # m/s above the set speed where the release boundary sits (fired strictly above; no hysteresis)
V_CRUISE_UNSET_KPH = 255.        # carState.vCruise "no set speed" sentinel (superproject V_CRUISE_UNSET); 0 counts as invalid here too


def over_speed_release(active: bool, accel: float, v_ego: float, v_cruise: float) -> bool:
  """True -> force the pedal command to EXACTLY 0.0 for this frame (all-zero ENABLE=0 frame -> pass-through -> the
  car coasts on engine braking). Measured reason (route 16a): set speed lowered 88->80->72 km/h while at 80.6 km/h;
  plan aTarget ~0 for 20 s, but hold_cmd added anyway -> ~13.5 % pedal -> ~8 % TPS -> no coast. And any live
  command collapses the car's own ~-1.33 m/s^2 coast to ~-0.05, so only an exactly-zero command releases. Narrow by
  design: fires only when longitudinal is active, the set speed is real (0 < vCruise < 255 km/h), vEgo is strictly
  more than OVERSPEED_RELEASE_MARGIN above it, and the plan's accel ask is <= 0. Inert at/below the set speed, on
  positive asks, disengaged, unset vCruise (255/0) and standstill launches."""
  return bool(active and accel <= 0. and
              0. < v_cruise < V_CRUISE_UNSET_KPH and
              v_ego > v_cruise * CV.KPH_TO_MS + OVERSPEED_RELEASE_MARGIN)


# *** 17c blip smoothing (route 17c, measured): over-speed release HYSTERESIS + soft recovery back up to the hold ***
# Route 17c: 87 upward throttle pulses in 825 s at/near the set speed (6.3/min; peaks p50 +5.4 %, p90 +16 %, max +22 %
# pedal; TPS swings to ~30 points). Two law-side mechanisms, both fixed here, neither touching the plan:
#  (1) the over-speed release fired strictly above set + 0.5 m/s with NO hysteresis, so a car hovering at set + 0.45..0.51
#      with a ~0 ask toggled coast <-> hold every 2-5 frames (19:57:53-54: ENABLE 0/1/0/1 at 100 ms, raw 0/480/0/558).
#      Now the release LATCHES once fired (entry predicate unchanged) and holds the exact-zero coast until the car is
#      back down at the set speed (re-arm margin 0.0 m/s: a 0.5 m/s band, wider than the +-0.1..0.3 m/s speed dither
#      at set speed on 17c) or the plan makes a DELIBERATE positive ask (>= 0.5 m/s^2; above the set speed on 17c the
#      ask never exceeded +0.13, and the micro-ask dither is +-0.1..0.4). Entry is still `accel <= 0`, so the
#      release stays inert on positive asks, at/below the set speed, disengaged, unset vCruise, standstill launches.
#  (2) the fall is instant but the rise from a dip back up to the hold was the full jerk rate (0.72 pedal/s >= 22 m/s:
#      0 -> 0.18 hold in 0.25 s), so every dip below the hold (release, law clip to 0 at ask < -hold/scale, or any
#      sag of the ask, and the gas-override hand-back: 19:40:17 was the driver lifting off the pedal, then 0 -> 0.223
#      in 0.3 s) returned as a throttle snap. Now a rise BELOW the hold, for a non-deliberate ask (< OVERSPEED_EXIT_ACCEL),
#      is limited to hold(v) / PEDAL_REENTRY_TIME_S per second: 0 -> hold takes exactly 1.0 s (0.18/s at 30 m/s =
#      0.49 m/s^3 equivalent jerk, ~4x gentler than 0.72/s), and the speed deficit of one full re-entry is bounded by
#      0.5 * hold(v)/scale(v) * 1.0 s ~= 0.25 m/s at 30 m/s. Above the hold, and for a deliberate ask (>= 0.5 m/s^2:
#      lead pulling away, merge, rolling engage with a real ask), the unchanged jerk rate applies.
# NOT touched: the launch ramp (soft recovery is gated off below LOW_SPEED_FULL_GAS_SPEED = 25 mph and while launching),
# the ceilings, the personality tables, MAX_INTERCEPTOR_GAS, the 0.001 enable threshold and the fault-clear frames.
OVERSPEED_REARM_MARGIN = 0.0     # m/s above the set speed at/below which a latched release re-arms (hysteresis band 0.5 m/s)
OVERSPEED_EXIT_ACCEL = 0.5       # m/s^2; a plan ask at/above this breaks a latched release (deliberate, not dither)
PEDAL_REENTRY_TIME_S = 1.0       # s; a rise from 0 back up to the hold takes exactly this long (rate = hold(v)/this)
PEDAL_REENTRY_V_MIN = LOW_SPEED_FULL_GAS_SPEED  # m/s; below this the soft recovery is off (launch/pull-away regime)


def over_speed_release_held(active: bool, accel: float, v_ego: float, v_cruise: float) -> bool:
  """Continuation predicate for an ALREADY-FIRED over-speed release (the latch). True -> keep the exact-zero coast.
  Holds while longitudinal is active, the set speed is real, vEgo stays above set + OVERSPEED_REARM_MARGIN and the
  plan's ask is below OVERSPEED_EXIT_ACCEL. Entry is only ever over_speed_release() (unchanged)."""
  return bool(active and accel < OVERSPEED_EXIT_ACCEL and
              0. < v_cruise < V_CRUISE_UNSET_KPH and
              v_ego > v_cruise * CV.KPH_TO_MS + OVERSPEED_REARM_MARGIN)


def get_pedal_reentry_rate(v_ego: float) -> float:
  """Soft recovery rate (pedal fraction per second) for a rise below the hold: 0 -> hold(v) in PEDAL_REENTRY_TIME_S,
  never faster than the normal jerk rate."""
  return min(get_hold_command(v_ego) / PEDAL_REENTRY_TIME_S, get_pedal_rate_up(v_ego))


def rate_limit_pedal_command(gas: float, gas_last: float, v_ego: float, launching: bool = False,
                             reentry: bool = False) -> float:
  """Rise at most get_pedal_rate_up(v_ego, launching) * DT_CTRL per controller frame; fall immediately.
  reentry=True (17c): while the last command is below the hold, the rise is limited to get_pedal_reentry_rate(v_ego);
  the cap is that RATE, never the hold value itself (review #1: a `hold` term here pinned the climb whenever vEgo rose,
  because hold(v_now) > gas_last by ~1e-5 re-entered the branch every frame). At/above the hold the normal rate takes
  over. Inert while launching or below PEDAL_REENTRY_V_MIN. reentry=False is the old limiter."""
  limit = gas_last + get_pedal_rate_up(v_ego, launching) * DT_CTRL
  if reentry and not launching and v_ego >= PEDAL_REENTRY_V_MIN:
    hold = get_hold_command(v_ego)
    if gas_last < hold:
      limit = min(limit, gas_last + get_pedal_reentry_rate(v_ego) * DT_CTRL)
  return min(gas, limit)


def get_interceptor_gas(cp, sensor_msg: str = STANDARD_IDS.sensor_msg) -> float:
  return (cp.vl[sensor_msg]["INTERCEPTOR_GAS"] + cp.vl[sensor_msg]["INTERCEPTOR_GAS2"]) // 2


# *** 17c DELIVERY: the measured inverse of the plant for negative asks (opt-in, HyundaiPedalDeliveryLaw) ***
# WHAT WAS MEASURED (route 17c, 100 Hz extract: pedal command on 0x700 vs aEgo 0.15-0.65 s later, engaged, driver's
# foot off, brake off, v >= 65 km/h; wsSTOP/fixes-17c/delivery/FINDINGS-delivery.md):
#
#   pedal     aEgo      pedal     aEgo
#   0 (EN=0)  -0.79     10-11.5%  -0.07
#   0.4-2.5%  -0.29     11.5-13%  -0.05
#   2.5-3.7%  -0.28..-0.14   13-14.5%  -0.01
#   4-5.5%    -0.11     14.5-16%  +0.02
#   5.5-9.7%  -0.085 (a DEAD BAND: 4 points of pedal, 0.007 m/s^2)   16-18%  +0.03
#                                       18-20%  +0.04
#                                       20-26%  +0.18..+0.39
#
# So the car's pedal has a ~2.5-3x weaker gain than the linear law assumes (PEDAL_SCALE 0.36 = 2.8 m/s^2 per unit
# pedal) exactly over the 4-13 % plateau, and 0.5-2 % already delivers what the law expects from 8-10 %. The linear
# law therefore UNDER-lifts for shallow negative asks: on 17c, asks of -0.6..-0.05 m/s^2 are 41 % of the engaged
# foot-off time, the mean ask -0.222 was delivered as -0.159 (+0.064 m/s^2 of runaway), and the worst band
# (-0.35..-0.25) asked -0.30 and delivered -0.16 (+0.131).
#
# THE LAW BELOW: for accel < 0 replace `scale(v)*accel + hold(v)` with the MEASURED inverse
#   cmd = clip(hold(v) - DROP(v, |accel|), 0, ceiling(v))
# where DROP is the measured pedal drop below the hold point (monotone, in pedal fraction), scaled below the highway
# band by the law's own existing speed schedule (PEDAL_SCALE(v)/PEDAL_SCALE_HIGHWAY, i.e. 1.0 at >= 22 m/s and 0.46 at
# 12 m/s where the measured gain is 2.2x stronger). It is anchored so that accel = 0 still gives exactly hold(v)
# (DROP(0) = 0) and the ceiling / the interceptor cap are untouched: the positive branch calls the unchanged
# get_pedal_command. DROP is a PHYSICS table (dead band, DFCO cliff), not a feel dial, so it is personality-invariant
# like the hold.
#
# Three parts of the measured curve are deliberate:
#  - |accel| <= 0.27: the measured inverse, 0 -> 0.115 pedal of drop.
#  - 0.27 < |accel| < 0.52 at highway speed: the plateau's FLOOR, 1.0 % pedal (DELIVERY_PEDAL_FLOOR). A live
#    0.4-2.5 % command has been measured to deliver -0.29 m/s^2 (the plateau), so the shallow-negative fix keeps a
#    small live command instead of collapsing to a coast. The floor is also what keeps the command from creeping to
#    1 DBC count (0.4 %) where the pedal's ENABLE bit would dither - the plateau is the right side of that cliff.
#    The floor exists ONLY at v >= DELIVERY_FLOOR_V, the band it was measured in: at a standstill this car must
#    command exactly 0 for any <= 0 ask (uncommanded motion), and 1 % pedal there would be exactly that.
#  - |accel| >= 0.52: EXACTLY 0.0 - byte-identical to what the deployed law already does below about -0.34, so
#    every deep ask still lifts the pedal fully and the gas/brake hand-over is unchanged (while the FCA11 brake law is
#    braking, which it opens at ask <= -0.5, the brake path hard-zeroes the pedal anyway, so the wire is identical
#    there too; the 0.50..0.52 sliver differs from today only when FCA11 is NOT braking, i.e. in pedal-only mode).
#    0.52 is the midpoint of the plant's cliff (a live plateau command delivers -0.29, the exact-zero pass-through
#    -0.79, so no ask in -0.52..0.52 can be served better than by one of them).
#
# AND THE ASK FILTER (the pedal path only): the plan's shallow asks dither at 2-4 Hz on 17c (9,509 runs of mean
# 0.09 s, only 3 longer than 0.6 s). With a correct gain that dither would reach the pedal, so accel < 0 is passed
# through a one-sided filter: a DEEPER ask is applied on the same frame (a lift is never delayed - the same rule as
# the rate limiter below), a SHALLOWER ask is approached with a 0.8 s time constant, so one 0.09 s dither run can
# re-apply at most ~11 % of the pedal it used to. Positive asks are NOT filtered (they keep the linear law).
DELIVERY_ACCEL_BP = [0., 0.05, 0.08, 0.11, 0.14, 0.18, 0.22, 0.27, 0.30]      # |accel ask|, m/s^2
DELIVERY_PEDAL_DROP = [0.000, 0.021, 0.045, 0.086, 0.104, 0.108, 0.111, 0.115, 0.1344]  # pedal below hold(v)
DELIVERY_PEDAL_HOLD_MEAS = 0.1414   # measured a=0 pedal on 17c (v >= 65 km/h) - the table's own hold anchor
DELIVERY_PEDAL_FLOOR_MEAS = 0.0070  # the table's pedal at its last point (the bottom of the plateau)
DELIVERY_PEDAL_FLOOR = 0.010     # pedal fraction the law HOLDS over 0.27..0.52 m/s^2 (the measured plateau floor)
DELIVERY_FLOOR_V = 18.0          # m/s; the floor exists only in the measured band (>= 65 km/h)
DELIVERY_COAST_ACCEL = 0.52      # m/s^2; at/above this |ask| the pedal command is EXACTLY 0.0 (coast as today)
DELIVERY_ASK_TAU_S = 0.8         # s; time constant of the SHALLOWER-ask side of the pedal-path ask filter


def get_delivery_pedal_drop(accel: float, v_ego: float) -> float:
  """Pedal fraction below hold(v) for a negative ask, from the measured inverse (monotone in |accel|).

  The measured table spans the pedal range [DELIVERY_PEDAL_FLOOR_MEAS, DELIVERY_PEDAL_HOLD_MEAS] at the route's mean
  highway speed. It is mapped onto THIS speed's own [floor, hold(v)] span, so that (a) a = 0 still lands exactly on
  hold(v) (a drop of 0), (b) the table's last point still lands on the plateau floor at any speed, and (c) below the
  measured band the drop shrinks with the law's own speed schedule (the pedal is far stronger there)."""
  floor = DELIVERY_PEDAL_FLOOR if v_ego >= DELIVERY_FLOOR_V else 0.
  span = max(get_hold_command(v_ego) - floor, 0.)
  k = span / (DELIVERY_PEDAL_HOLD_MEAS - DELIVERY_PEDAL_FLOOR_MEAS)
  return min(k * float(np.interp(abs(accel), DELIVERY_ACCEL_BP, DELIVERY_PEDAL_DROP)), span)


def filter_delivery_ask(accel: float, accel_prev: float, dt: float = DT_CTRL) -> float:
  """One-sided filter for the pedal path. Deeper (more negative) asks apply immediately; a shallower ask is approached
  with DELIVERY_ASK_TAU_S. Anything >= 0 (and anything not yet in the negative branch) is passed through untouched,
  so the positive path and every deliberate ask keep the unfiltered law input."""
  if accel >= 0. or accel <= accel_prev:
    return float(accel)
  return float(accel_prev + (accel - accel_prev) * (1. - np.exp(-dt / DELIVERY_ASK_TAU_S)))


def get_delivery_pedal_command(accel: float, v_ego: float, personality=PERSONALITY_STANDARD) -> float:
  """accel request -> pedal fraction, measured-inverse law for negative asks (the positive branch is exactly
  get_pedal_command, i.e. the deployed linear law). Accelerator only: <= 0 means coast or a small live plateau
  command, never brake."""
  if accel >= 0.:
    return get_pedal_command(accel, v_ego, personality)
  ceiling = get_personality_ceiling(v_ego, personality)
  if -accel >= DELIVERY_COAST_ACCEL:
    return 0.
  cmd = get_hold_command(v_ego, personality) - get_delivery_pedal_drop(accel, v_ego)
  if v_ego >= DELIVERY_FLOOR_V:
    cmd = max(cmd, min(DELIVERY_PEDAL_FLOOR, ceiling))
  else:
    # below the measured band the table is an extrapolation: take it only where it LIFTS more than the deployed law
    # (never command more pedal than today - a low-speed over-gas would be uncommanded motion, and the launch regime
    # is not what this fix measured).
    cmd = min(cmd, get_pedal_command(accel, v_ego, personality))
  return float(np.clip(cmd, 0., ceiling))


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


class GasInterceptorCarController:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP, dbc_names: dict):
    self.CP = CP
    self.CP_SP = CP_SP

    self.gas = 0.
    # 0039: launch latch. Set while an engaged positive request is a (near)-rest pull-away (see the ramp branch
    # below), cleared as soon as the car is moving above LAUNCH_RAMP_V_MAX or the request is non-positive (a
    # lift/coast), so it only ever selects the fast launch rate from rest and never a rolling re-engagement.
    self.gas_launching = False
    # 17c: over-speed release latch (hysteresis). Set by over_speed_release(), held by over_speed_release_held(),
    # cleared whenever longitudinal is not active.
    self.overspeed_released = False
    # 17c delivery (opt-in, HyundaiPedalDeliveryLaw): the pedal-path ask filter state. Unused (and never advanced)
    # unless CP_SP.pedalDeliveryLaw is set, so the OFF path stays byte-for-byte the deployed one.
    self.delivery_law = bool(getattr(CP_SP, 'pedalDeliveryLaw', False))
    self.delivery_ask = 0.
    self.interceptor_ids = get_interceptor_ids(CP_SP)
    self.interceptor_packer = CANPacker(dbc_names[GAS_INTERCEPTOR_BUS_KEY]) if CP_SP.enableGasInterceptor else None
    self.fca11_brake = Fca11LongBrake(CP, CP_SP, dbc_names)

  def create_gas_command(self, CC: structs.CarControl, CS, frame: int, now_nanos: int = 0,
                         personality: int = PERSONALITY_STANDARD) -> list[CanData]:
    can_sends: list[CanData] = []
    if not self.CP_SP.enableGasInterceptor:
      return can_sends

    # send exactly zero when not actively controlling: the pedal outputs max(driver, command), and zero + ENABLE=0
    # also clears any latched pedal fault (pedal firmware)
    # (no low-speed cut: a pause/resume engagement may start from a standstill; LOW_SPEED_MAX_GAS limits the launch)
    active = CC.longActive

    # fork: FCA11 long braking (HyundaiFca11Brake, default OFF). Decides FIRST: while it is braking, the pedal
    # command is hard-zeroed this frame (gas and brake are never commanded together) and the pedal stream keeps
    # flowing (zero frames), which also keeps the pedal fault-clear path alive.
    # Freshness reference for the mirrored camera frame: the same now_nanos CarInterface.apply received (replay-safe).
    CS.fca11_now_nanos = now_nanos
    # 0030 (D2-iii): feed the pandad echo of our own 0x38D frames (collected in CarInterface.update) so the class can
    # confirm the passive close frame was ACCEPTED (the clean hand-back); a refused close frame is re-owed.
    # 0053: the camera hand-back ingests this tick's camera 0x38D frames FIRST, so the echo judgement below (refusal
    # streak, probation, divergence guard) sees the camera state the panda had when it judged our frames.
    self.fca11_brake.ingest_camera(CS, frame)
    self.fca11_brake.observe_echo(getattr(CS, "fca11_echoes", None))
    # 0034: STEERING-WHEEL consent. The CarState phase recognised a double-press on this frame's cruise
    # buttons and (if it was a consent gesture for the rep parked on a PREVIOUS frame) dropped its taps
    # from CS.buttonEvents. Register this brake so CarState can read the pending rep, consume the verdict,
    # and let the sequencer write the SAME consent.json the mici UI writes -- consumed by the tick below
    # (<= 1 send slot, ~20 ms), under the identical freshness/binding/re-verify rules as a touch tap.
    if hasattr(CS, "register_wheel_brake") and self.fca11_brake.cal_seq is not None:
      CS.register_wheel_brake(self.fca11_brake)
      wheel = CS.consume_wheel_consent()
      if wheel is not None:
        self.fca11_brake.cal_seq.wheel_gesture(wheel, frame)
    brake_sends = self.fca11_brake.update(CC, CS, frame, personality)
    # 0030 (D1-b): publish the FCA11-long loss so the CarState side can surface it (CarStateSP.fca11Unavailable);
    # 0053: that PERMANENT alert is the pedal-fault latch / refusal wall only. The camera hand-back is the TRANSIENT
    # fca11CameraBraking, and a Python<->panda disagreement is fca11Divergence.
    CS.fca11_unavailable = bool(self.fca11_brake.fca11_unavailable)
    CS.fca11_camera_braking = bool(self.fca11_brake.camera_braking_alert)
    CS.fca11_divergence = bool(self.fca11_brake.fca11_divergence)
    # 0040: publish the three FCA11 braking alerts (read by CarStateExt -> CarStateSP, then car_specific.py -> events.py).
    CS.fca11_supervise_stop = bool(self.fca11_brake.supervise_stop)
    CS.fca11_stop_complete = bool(self.fca11_brake.stop_complete)
    CS.fca11_brake_now = bool(self.fca11_brake.brake_now)
    # 0029 CAL: publish the cal-hold state for this frame (read by CarStateExt.gasPressed on the NEXT
    # CarState update -- a <= 10 ms / 1-frame lag, documented in DESIGN.md). The owner's foot must be
    # off the pedal for a cal hold to be active (the production pedal-clear window), so this can only
    # ever LOWER gasPressed toward the truth = 0.
    CS.fca11_cal_active = bool(self.fca11_brake._cal_active)

    # jerk-limited upward from the last computed command (0 while inactive, so every engagement ramps from 0). A clear
    # frame below does not reset it: the pedal resumes the command right after a ~20 ms fault blip.
    # While the FCA11 brake is active the pedal command is EXACTLY zero for this frame (hard zero, not the law's
    # zero: the law would still add the hold feedforward at speed).
    # 0029 CAL: while a scripted CAL hold is active the pedal is ALSO forced to 0, for the same reason (gas and
    # brake are never commanded together). This is REQUIRED, not cosmetic: the cal brake is exogenous to the
    # planner, so the planner sees the car slowing and asks for gas; a commanded gas press would set gas_pressed
    # and the panda's driver-cut latch would cut the very episode we are measuring. Forcing the pedal to 0
    # replicates the production condition (when the planner asks brake it never gases) and can only REDUCE
    # actuation. Inert when cal mode is off (`_cal_active` is False).
    # 0053: while the CAMERA is braking (cam_brake), openpilot never throttles against it: the pedal command is forced to
    # exactly 0 (the panda refuses any non-zero GAS_COMMAND then too). The zero stream keeps flowing (fault-clear path).
    if self.fca11_brake.cam_brake:
      self.gas = 0.
      self.gas_launching = False
    elif active and not self.fca11_brake.braking and not self.fca11_brake._cal_active:
      accel = CC.actuators.accel
      # over-speed release (route 16a): above a REAL set speed with a <= 0 plan ask the pedal goes to EXACTLY zero
      # (all-zero ENABLE=0 frame, pass-through) so the car coasts on engine braking instead of holding speed on
      # hold_cmd. Every other case falls through to the unchanged law below.
      # 17c: the release LATCHES (hysteresis): entry is the unchanged over_speed_release(), and once fired it holds the
      # exact zero until vEgo is back down at set + OVERSPEED_REARM_MARGIN or the plan asks >= OVERSPEED_EXIT_ACCEL.
      self.overspeed_released = (over_speed_release(active, accel, CS.out.vEgo, CS.out.vCruise) or
                                 (self.overspeed_released and
                                  over_speed_release_held(active, accel, CS.out.vEgo, CS.out.vCruise)))
      if self.overspeed_released:
        self.gas = 0.
        self.gas_launching = False
      else:
        # 0039: a launch is an engaged positive request while the car is still at (near) rest. Latch it over the first
        # few metres so the fast launch rate is held until the car is clearly moving, then hand back to the jerk rate
        # (above LAUNCH_RAMP_V_MAX the two branches are the same function anyway). Cleared on any non-positive request
        # or once vEgo exceeds LAUNCH_RAMP_V_MAX, so a rolling re-engagement at 11-19 m/s can never set it.
        if accel > 0. and CS.out.vEgo <= LAUNCH_RAMP_V_MAX:
          self.gas_launching = True
        elif accel <= 0. or CS.out.vEgo > LAUNCH_RAMP_V_MAX:
          self.gas_launching = False
        # 17c: soft recovery up to the hold for a non-deliberate ask (< OVERSPEED_EXIT_ACCEL); a deliberate ask keeps
        # the unchanged jerk-rate response (lead pulling away, merge). rate_limit_pedal_command gates it off below
        # PEDAL_REENTRY_V_MIN and while launching.
        # 17c delivery (opt-in): the negative branch of the law is the measured inverse, and its ask is the FILTERED
        # one (deeper asks still land on the same frame; the filter only slows the return toward 0). Positive asks
        # and the deliberate-ask predicate above keep the RAW ask.
        if self.delivery_law:
          self.delivery_ask = filter_delivery_ask(accel, self.delivery_ask)
          pedal_target = get_delivery_pedal_command(self.delivery_ask, CS.out.vEgo, personality)
        else:
          pedal_target = get_pedal_command(accel, CS.out.vEgo, personality)
        self.gas = rate_limit_pedal_command(pedal_target, self.gas, CS.out.vEgo,
                                            self.gas_launching, reentry=accel < OVERSPEED_EXIT_ACCEL)
    elif self.fca11_brake.braking or self.fca11_brake._cal_active:
      self.gas = 0.
      self.gas_launching = False
    else:
      self.gas = 0.
      self.gas_launching = False
    if not active:
      self.overspeed_released = False
    if not active or self.gas == 0.:
      # 17c delivery: re-arm the ask filter whenever the pedal is not actually holding a live command (disengaged,
      # released, FCA11 braking, cal, coast): a stale deep ask must never be re-applied after the gap.
      self.delivery_ask = 0.

    # 50 Hz; the pedal only accepts frames whose counter is exactly previous + 1
    send_slot = frame % 2 == 0
    # pedal reports a latched fault while we command -> this slot carries the clear (zero) frame instead
    # (counter stays continuous: the clear frame simply occupies the slot)
    clear = CS.pedal_fault_monitor.update(frame, active, CS.interceptor_state, send_slot, self.gas)
    if send_slot:
      can_sends.append(create_gas_interceptor_command(self.interceptor_packer, 0. if clear else self.gas, frame // 2,
                                                     self.interceptor_ids.command_msg))

    can_sends.extend(brake_sends)
    return can_sends

  def fca11_post_send(self) -> None:
    """0045 FCA11 Shadow: compute + log V3 for the frame create_gas_command just returned. Called by the superproject
    card.py AFTER sendcan is published (so V3 can never delay or alter the OLD-LAW frame on the wire). No-op unless
    Shadow; never raises (Fca11LongBrake.v3_post_send catches + logs every V3 failure)."""
    self.fca11_brake.v3_post_send()
