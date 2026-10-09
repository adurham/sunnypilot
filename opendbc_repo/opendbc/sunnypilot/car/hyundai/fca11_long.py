"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Fork: FCA11 longitudinal BRAKING for Hyundai non-SCC ICE cars on the gas interceptor
(param ``HyundaiFca11Brake``, default OFF, ``CP_SP.fca11Brake``). NOT A TEST MODE: this is the
production feature built on the on-car-proven FCA11 path (roll-20261005T232357Z).

Actuator facts this is built on (measured on the car, ~31 km/h, 0.6 s pulses):
  - variant B frames (CF_VSM_Warn=3, FCA_CmdAct=1, CF_VSM_DecCmdAct=0, CR_VSM_DecCmd=g*100,
    CF_VSM_Prefill=1) make the ESC brake: 0.74 / 1.28 / 2.03 m/s^2 extra decel for 0.10 / 0.20 /
    0.30 g, ~2/3 of the command, linear.
  - no response below ~8-10 km/h and at standstill; FCA_ACK rises during braking; the dash shows a
    collision-braking notification with no chime.

Design (see car-features/fca11-long-integration.md for the full picture):
  - the PLANNER asks for accel as usual; when ``accel <= FCA11_BRAKE_ONSET_ACCEL`` and the window
    is open the pedal command is hard-zeroed for that frame (gas and brake are never commanded
    together) and one FCA11 mirror frame goes out at the camera's own rate;
  - the frame MIRRORS the freshest bus-2 camera 0x38D (zero-gap takeover) and overrides ONLY the
    brake fields: CR_VSM_DecCmd, CF_VSM_Prefill, CF_VSM_Warn (3), FCA_CmdAct, CF_VSM_DecCmdAct
    (0), the alive counter (+1, continuing the camera's sequence) and the CRC8-J1850 checksum;
  - panda (bit 256, patch 0014) independently polices every frame: controls_allowed AND
    heartbeat_engaged, gear D, no pedal, fresh inputs, every wheel above 9 km/h, DecCmd <= 0.30 g,
    rate-limited growth (every (re)opened episode restarts at +RATE_STEP), and an immediate camera
    hand-back. There is NO actuation-duration budget and NO cooldown (0035): a brake runs until the
    planner releases it or a gate (floor, driver, camera, stale) ends it. Nothing here can make the
    car brake if the panda rules say no;
  - the COMMANDED decel cap is personality-indexed (relaxed 0.12 g / standard 0.20 g / aggressive 0.30 g, the
    max) so the driver's feel dial owns the braking strength too. Every tier is <= the panda's 0.30 g gate
    (which is personality-agnostic), so this is a python-only change;
  - no auto-resume after a CUT (driver brake/gas, gear != D, pedal fault, camera request): braking
    stays off until the next deliberate pause/resume engagement, per the owner's model.
  - G7 (N1): on every braking true -> false transition ONE passive mirror frame goes out - all brake
    fields 0, Warn 0, the camera's own idle shape: the clean hand-back that ends the episode at the
    panda (an accepted passive frame drops the panda's rate-limit reference to 0, so the next episode
    restarts its onset ramp). 0035 deleted the 2.5 s budget / 3 s cooldown mirror that used to live
    here, together with the budget-edge stop-commanding it drove.
  - G8 (R7 frame contract): the mirror is made a strict subset of what the panda accepts, frame by frame.
    (R7-A) the rate-limit reference (``_dec_last_sent``) advances only on frames actually SENT: the panda
    allows +RATE_STEP over the last ACCEPTED frame, and we only send on the 50 Hz slot, so advancing the
    reference on every 100 Hz frame stepped the sent ramp +8 (twice the allowed rate) and the panda refused
    every actuating frame after the first. (R7-D) the longActive-off exit owes the passive frame too, so a
    mid-episode longActive drop still hands back cleanly.

Byte layout (hyundai_can.dbc BO_ 909 FCA11, Intel/LSB-first start bits):
  CF_VSM_Prefill (0,1), CF_VSM_HBACmd (1,2), CF_VSM_Warn (3,2), CF_VSM_BeltCmd (5,3),
  CR_VSM_DecCmd (8,8), PAINT1_Status (16,2), FCA_Status (18,2), FCA_CmdAct (20,1),
  FCA_StopReq (21,1), FCA_DrvSetStatus (22,3), CF_VSM_DecCmdAct (31,1), FCA_Failinfo (32,3),
  CR_FCA_Alive (35,4), FCA_RelativeVelocity (39,9), FCA_TimetoCollision (48,8),
  CR_FCA_ChkSum (56,8).
"""
import os
import time
from dataclasses import dataclass

from opendbc.car import structs
from opendbc.car.hyundai.hyundaican import hyundai_checksum
from opendbc.car.can_definitions import CanData
from opendbc.sunnypilot.car.hyundai import cal_mode
from opendbc.sunnypilot.car.hyundai.values import (HYUNDAI_FCA11_LONG_MAX_DEC, HYUNDAI_FCA11_LONG_RATE_STEP)
from opendbc.sunnypilot.car.hyundai import fca11_log

FCA11_ADDR = 0x38D
FCA11_CAM_BUS = 2

# accel request (m/s^2) at/below which the FCA11 brake engages. Coast (engine braking, ~0.3-0.5
# m/s^2) covers everything above this; the first meaningful "slow down" ask is about -0.5 (which
# delivers ~0.34 m/s^2 through the measured 2/3 gain). Between this and 0 the car coasts exactly
# as it does today.
FCA11_BRAKE_ONSET_ACCEL = -0.5

# *** G5: onset debounce + release hysteresis + minimum on-time (anti-flap) ***
# A planner ask that crosses -0.5 for a single frame used to open an FCA11 episode (Warn=3 = a dash
# "collision braking" notification) and immediately release the moment the ask rose back above -0.5.
# On the owner's route this produced ~1.5 notifications/min with sub-0.3 s blips and 2 s re-asks (M7).
#
# Release hysteresis (m/s^2): while braking, STAY engaged until the ask rises above this (release only
# when accel > -0.30). The band [-0.30, -0.50] is the hysteresis window; inside it the commanded decel
# is floored at the onset value so the ESC never sees a 0 mid-episode (that would be a release).
FCA11_BRAKE_RELEASE_ACCEL = -0.30
# Onset debounce (controller frames, 100 Hz): a NEW episode needs this many consecutive frames of
# (window-open AND accel <= ONSET) before the first frame goes out. 20 frames = 0.2 s. This kills the
# single-frame asks without adding latency to a sustained one.
FCA11_ONSET_HOLD_FRAMES = 20
# Minimum on-time (controller frames, 100 Hz): once engaged, stay engaged at least this long even if
# the ask goes away, so an isolated sub-0.3 s blip becomes one continuous episode. 60 frames = 0.6 s.
FCA11_MIN_ON_FRAMES = 60
# Hard-ask bypass: an ask at/below this skips the onset debounce entirely, so a genuine hard demand is
# never delayed by 0.2 s (safety + coverage). Suggested in the G5 plan; keeps acceptance (c) honest.
FCA11_ONSET_BYPASS_ACCEL = -1.2
# Above the bypass threshold this is the minimum on-time, in seconds, exposed for the report/tests.
FCA11_MIN_ON_S = FCA11_MIN_ON_FRAMES / 100.0
FCA11_ONSET_HOLD_S = FCA11_ONSET_HOLD_FRAMES / 100.0

# *** G7 (N1): passive release frame ***
# On every braking true -> false edge ONE passive 0x38D goes out (the camera's idle shape). It is the clean
# hand-back, and an ACCEPTED passive frame drops the panda's rate-limit reference to 0 (safety/modes/hyundai.h,
# the 0035 onset rule), so the next episode restarts at +RATE_STEP - exactly what this layer does too
# (`_dec_last_sent` resets on release). 0035: there is NO actuation-duration budget / cooldown any more, in the
# panda or here, so nothing in this module may end a brake or hold one off on a clock.

# *** 0038: driver GAS = per-frame suppression + a short hold-off, NOT a latch ***
# Owner requirement: hitting the gas to escape an over-brake must override the CURRENT frame (human wins) but must
# NEVER lock out FUTURE braking events. Before 0038 a gas press latched the driver cut, cleared only by a deliberate
# disengage+re-engage, so the next stop after a gas escape was unbraked. Gas is now removed from the driver latch
# (which keeps brake / gear != D only) and enforced per-frame, exactly like the panda (safety/modes/hyundai.h):
#   * while gasPressed -> fca11_ok is False, so the episode releases and the passive (all-zero) frame goes out;
#   * for GAS_HOLDOFF_FRAMES after the pedal releases, fca11_ok stays False (a plain elapsed-frame hold-off, no edge
#     needed to clear it), so a 1-2 Hz brake/gas fight cannot start. A NEW, distinct brake request after the hold-off
#     is served normally, first frame at the onset floor then ramping.
# Frames are the car layer's clock (controller frames are 100 Hz): 50 frames = 0.5 s, matching the panda's
# HYUNDAI_FCA11_LONG_GAS_HOLDOFF_US. Must stay equal (a unit test pins both).
GAS_HOLDOFF_FRAMES = 50
GAS_HOLDOFF_S = GAS_HOLDOFF_FRAMES / 100.0

# 0040: the planner-side speed floor is DELETED. It was 12 km/h, chosen to stay 3 km/h above the panda's (now also
# deleted) 9 km/h backstop. Both floors were OUR assumption that the ESC is inert below ~8-10 km/h - a belief never
# tested, because OUR floor stopped us from ever asking below 12.7 km/h. The owner's directive: treat FCA11 as a real
# brake system, command through zero, and LOG what the ESC does. There is therefore NO speed gate here any more, and no
# hold cap: below STOP_KPH the planner's standstill request keeps being commanded.
#
# The old "FCA11: Take Over Below 12 km/h" alert is now FALSE and is DELETED (see the superproject events.py /
# car_specific.py for its removal). It is replaced by the three owner-approved alerts below.
#
# steady YELLOW "supervise stop": a stop is requested (we are braking) and the car is slow. Warning only.
FCA11_SUPERVISE_KPH = 15.
# steady YELLOW "stop complete - hold brake pedal": at/near a full stop while we keep holding. An explicit HAND-OFF,
# NOT a cap - the command keeps flowing. "standstill" is CarState.standstill (all wheels <= 0.375 km/h).
# RED persistent "BRAKE NOW": the ESC is not doing what we asked, or the hold was lost. Two triggers, both measured:
#   * ESC non-response: delivered aEgo < FCA11_NORESPONSE_FACTOR x expected for FCA11_NORESPONSE_MS while commanding
#     >= FCA11_NORESPONSE_MIN_LSB. aEgo is CS.out.aEgo (m/s^2, measured). expected = dec_cmd * 0.01 g * FCA11_BRAKE_GAIN.
#   * hold lost: v > FCA11_HOLD_LOST_KPH with no gas while we were commanding a stop.
# Clears ONLY on driver brake. There is deliberately NO hold-duration alert.
FCA11_NORESPONSE_FACTOR = 0.4
FCA11_NORESPONSE_MS = 500.
FCA11_NORESPONSE_MIN_LSB = 8
FCA11_HOLD_LOST_KPH = 2.
FCA11_STOP_KPH = 0.5  # the "at/near a full stop" band (log + stop-complete alert), matching the planner's stop semantics
# 0041 (SHIP-BLOCKER gating fix): no_response is only meaningful on a REAL, SUSTAINED, MOVING brake. A car
# HELD AT STANDSTILL has aEgo ~ 0, so the raw aEgo test tripped at every red light; a standstill hold is
# hold_lost's job, not no_response's. So no_response runs ONLY while the car is moving above this, with a
# real command (>= MIN_LSB) that has been held for the ESC lag (dead 0.14 + 2*tau 0.24 = 0.62 s) before the
# 500 ms measurement timer may start. The command-stable gate resets whenever the command drops below 8 LSB.
FCA11_NORESPONSE_MIN_KPH = 5.0
FCA11_NORESPONSE_CMD_STABLE_S = 0.62
# 0041 (latching): brake_now is still PERSISTENT (it survives frame-to-frame), but it SELF-HEALS once both
# trigger conditions have been continuously false for this long, so a transient can never latch the red
# alert on for a whole drive. (It also clears immediately on driver brake / disengage / gear != D / gas.)
FCA11_BRAKE_NOW_CLEAR_S = 2.0

# 0040: per-episode logger. One compact JSON line per braking episode to /data/fca11-log/episodes.jsonl (see fca11_log
# for the schema, the fields and the reasoning). Inert (no accumulation, no I/O) unless enabled.
FCA11_LOG_ENABLED = os.environ.get("FCA11_LOG_DIR", "") != "0"

# PID accel bound the interface hands the planner above the floor: 0.30 g cap x the measured
# delivered/commanded ratio (FCA11_BRAKE_GAIN) + margin. A -2.0 request commands 0.30 g raw
# (clamped). See the FCA11_BRAKE_GAIN block below for what that ratio is and is NOT.
FCA11_A_MAX_MSS2 = 2.0

# *** FCA11_BRAKE_GAIN — ONE provenance block (the single source of truth for this constant) ***
#
# WHAT IT MEANS (exactly): the delivered-decel / commanded-decel RATIO, fitted THROUGH THE ORIGIN
# (zero intercept):  delivered_extra[m/s^2] = FCA11_BRAKE_GAIN * (cmd_LSB * 0.01 * 9.81).
# The request path INVERTS it (dec_cmd_from_accel):  dec_g = (-accel / FCA11_BRAKE_GAIN) / 9.81,
# so an accel ask of -2.0 m/s^2 commands ~0.30 g raw. It is a scalar here, not a slope+offset.
#
# THE VALUE THE DEPLOYED CODE USES: FCA11_BRAKE_GAIN = 0.67.
#
# THERE ARE THREE HISTORICALLY CITED FIGURES, FROM THREE DIFFERENT ESTIMATORS. They were measured
# on DIFFERENT FRAME SHAPES AND DURATIONS and are NOT interchangeable. Do not "reconcile" them
# by picking one; quote each with its origin.
#   1. "2/3 gain", fit slope 6.46 m/s^2 per g  -- the ROLLING-TEST measurement: ~31 km/h,
#      0.6 s pulses, 0.10 / 0.20 / 0.30 g commands delivering 0.74 / 1.28 / 2.03 m/s^2 extra.
#      This is a SLOPE fit over short pulses and it is stated in delivered-decel-per-g-of-command.
#   2. Ratio 0.67 (== this constant) -- the same rolling run read as a through-origin
#      delivered/commanded RATIO (2.03 / (0.30 * 9.81) ~= 0.69; ~0.67 across the points).
#   3. Affine fit of the route-151 PRODUCTION frames: extra = BITE + K * cmd_g, with
#      K ~ 9.7-10.1 (m/s^2 per g) and BITE ~ 0.63-0.75 (m/s^2). This is a TWO-PARAMETER (slope +
#      offset) fit on long, real production brake frames at 50-80 km/h -- a different model
#      shape from (1)/(2).
#
# NOTE THE DISAGREEMENT, PLAINLY: 0.67 x 9.81 = 6.57, so figures (1) and (2) -- both "6.4x" --
# do not even agree with each other (6.46 vs 6.57). Figure (3)'s K ~ 9.7-10.1 is ~1.5x either.
# These are NOT three measurements of one number; they are three different estimators on
# different data. This comment exists so the next reader SEES the disagreement and the
# provenance, not so it is papered over with a single "correct" slope.
#
# The live refit is Stage-1 of car-features/normal-braking/PLAN.md (K ~ 9.9, BITE ~ 0.68), which
# supersedes 0.67. Until that ships, 0.67 is what the car runs -- CHANGE NO VALUE WITHOUT THE REFIT.
FCA11_BRAKE_GAIN = 0.67

# *** 0041: FCA11_AFFINE_GAIN — the OPT-IN affine command law (DEFAULT OFF), the Stage-1 refit ***
#
# The deployed 0.67 law inverts a THROUGH-THE-ORIGIN gain (delivered = 0.67 * commanded). The measured plant is
# AFFINE: extra[m/s^2] = BITE + K * cmd_g, fitted on 17260 frames / 126 brake events from the owner's routes at
# K = 9.9 (CI [9.43, 10.38]), BITE = 0.40 (CI [0.33, 0.48]). Two consequences the through-origin law gets wrong
# at the same time: it UNDER-commands (~0.67 vs the true slope, ~1.5x low once BITE is accounted for), and the
# 8 LSB episode floor OVER-delivers at the bottom (an 8 LSB command delivers ~1.19 m/s^2 where the planner wanted
# ~0.06). See normal-braking/PLAN.md Stage 1 and stage3-floors/REPORT.md.
#
# When HyundaiFca11AffineGain is ON, the request inverts the AFFINE plant instead:
#     dec_cmd_from_accel_affine(want, K, BITE): want_extra = -want (m/s^2, positive) -> cmd_LSB = (want_extra - BITE)/K in g.
# It is clamped to [0, 30] here (the caller re-clamps to the personality tier cap as before) and the release-band
# floor drops from 8 to 4 LSB ONLY WHILE MOVING: at/below the low-speed stop band the hold keeps the >= 8 LSB floor
# (FCA11_AFFINE_HOLD_LSB), because a through-zero command that tapers honestly at speed must not weaken a physical
# standstill hold. This hold/release floor is applied to the REQUEST, BEFORE the +RATE_STEP rate limiter, so the ramp
# still governs the episode onset and the command can never exceed the panda's +RATE_STEP onset gate (see the fix note
# on FCA11_AFFINE_STOP_BAND_KPH). The tier cap (20 std / 30 agg) and FCA11_A_MAX_MSS2 are UNCHANGED.
#
# THIS IS DEFERRED / OPT-IN. With the param absent AND no explicit env override, NOTHING here runs and the module is
# byte-identical to 0040: the 0.67 law inverts exactly as before and the 8 LSB floor is untouched. The env FCA11_AFFINE
# is an explicit test / A-B override that takes PRIORITY when set (see affine_gain_enabled). No value of
# FCA11_BRAKE_GAIN or its doc block above is changed.
FCA11_AFFINE_K = 9.9
FCA11_AFFINE_BITE = 0.40
FCA11_AFFINE_RELEASE_LSB = 4  # the release-band floor under the affine law (vs FCA11_NORESPONSE_MIN_LSB = 8 today)
# 0041 hold floor: while the car is stopped or creeping (in the low-speed hold band) the affine ON path keeps the SAME
# >= 8 LSB hold floor as the deployed 0.67 law. Reason: this is a DCT car that CREEPS at a stop and the sim cannot see
# the ESC's behaviour at standstill, so a hold weaker than today's 8 LSB is an untested risk, not an improvement. (8 ==
# FCA11_NORESPONSE_MIN_LSB by construction; pinned in the tests.)
FCA11_AFFINE_HOLD_LSB = 8
# The hold-band edge: v at/below which the affine path keeps FCA11_AFFINE_HOLD_LSB. Deliberately GENEROUS (3 km/h, above
# the 0.5 km/h stop-complete band) and also true whenever CarState.standstill is set.
FCA11_AFFINE_STOP_BAND_KPH = 3.0
# THE FLOOR ABOVE IS A *REQUEST* FLOOR, APPLIED BEFORE THE +RATE_STEP RATE LIMITER (not after it). This is
# load-bearing: the panda's onset rule allows only +RATE_STEP (4 LSB) over the last ACCEPTED frame, and that frame is
# 0 at an episode open. A floor applied AFTER the limiter lifts the first SENT frame straight to 8 LSB, over the onset
# gate, so the panda REFUSES every actuating frame (dec=8, last=0) and the brake never engages at low speed. Applied
# to the REQUEST the limiter still governs the onset: request 8 -> min(8, 0+4)=4 (first frame, ACCEPTED) ->
# min(8, 4+4)=8 (ACCEPTED) -> reaches the hold floor in ~40 ms. So these values can never exceed the panda gate.


def dec_cmd_from_accel_affine(accel: float, K: float = FCA11_AFFINE_K, BITE: float = FCA11_AFFINE_BITE) -> int:
  """accel request (m/s^2, negative = brake) -> raw CR_VSM_DecCmd (0.01 g/LSB) through the AFFINE plant, clamp [0, 30].

  The inverse of ``extra = BITE + K * cmd_g``: a wanted extra decel of ``want_extra = -accel`` needs
  ``cmd_g = (want_extra - BITE) / K``. Zero for anything above the onset threshold (coast territory) and for a
  want at/below BITE (the plant's own bite already delivers more than asked). Unlike the 0.67 path this returns
  0 (not 1) when nothing is asked, so it can taper honestly through zero; the caller applies the personality cap.
  """
  if accel > FCA11_BRAKE_ONSET_ACCEL:
    return 0
  want_extra = -accel
  if want_extra <= BITE:
    return 0
  cmd_g = (want_extra - BITE) / K
  return max(0, min(30, int(round(cmd_g * 100))))


def _read_affine_gain_param(default: bool = False) -> bool:
  """Read the opt-in ``HyundaiFca11AffineGain`` bool param. DEFAULT OFF; never raises.

  Follows the fork's param-read pattern (0018): the key may be absent on a stale libparams (check_key raises
  UnknownKeyName), or the Params library may be unavailable to the opendbc process -- anything but a clean truthy
  read is False. Only used as a LAST-RESORT fallback; the primary path is CP_SP.fca11AffineGain (set by the
  superproject's setup_interfaces). The env FCA11_AFFINE is handled one level up (an explicit override), not here.
  """
  try:
    from openpilot.common.params import Params  # noqa: PLC0415  (lazy: opendbc must not hard-depend on openpilot)
    key = "HyundaiFca11AffineGain"
    p = Params()
    try:
      p.check_key(key)
    except Exception:
      return default  # unknown key on this libparams -> default (OFF)
    return bool(p.get_bool(key, False))
  except Exception:
    return default


def _affine_env_override() -> bool | None:
  """The explicit FCA11_AFFINE env value as an A/B override, or None when it is unset / ambiguous.

  Truthy ("1"/"true"/"yes"/"on") -> True, falsy ("0"/"false"/"no"/"off") -> False, anything else (unset, empty,
  unrecognized) -> None. This is the sim's ONLY way to flip the affine law, so it must be an EXPLICIT override that
  wins over the CP_SP field (which is always present, defaulting False) whenever it is set.
  """
  val = os.environ.get("FCA11_AFFINE", "").strip().lower()
  if val in ("1", "true", "yes", "on"):
    return True
  if val in ("0", "false", "no", "off"):
    return False
  return None


def affine_gain_enabled(CP_SP, default: bool = False) -> bool:
  """The resolved opt-in state for the affine law: an explicit env override, else CP_SP.fca11AffineGain, else Params.

  Reads ONCE at Fca11LongBrake.__init__ (never per frame). The env FCA11_AFFINE is a test / A-B override that takes
  PRIORITY when explicitly set (1/true/yes/on -> ON, 0/false/no/off -> OFF) -- it is the sim's only way in, so it must
  beat the always-present capnp field. When the env is unset the CP_SP field is authoritative (always present on a
  real CarParamsSP), and only a build without the field falls through to the guarded Params key, then the default.
  """
  env = _affine_env_override()
  if env is not None:
    return env
  field = getattr(CP_SP, "fca11AffineGain", None)
  if field is not None:
    return bool(field)
  return _read_affine_gain_param(default)

# camera-frame freshness for the mirror source (s): the camera sends ~50 Hz; anything older than
# two periods means we do not know the ESC's last-seen state -> no send (never a hardcoded frame).
FCA11_CAM_FRESH_S = 0.100

# send rate divisor: controller frames are 100 Hz; every other frame = the camera's own 50 Hz.
FCA11_SEND_EVERY = 2

# camera-actuation detect on the freshest bus-2 frame (a real AEB/warning request -> hand back):
# any of Prefill / Warn / CmdAct / StopReq / DecCmdAct or the HBA bits set.
CAM_ACT_PREFILL_BIT = 0
CAM_ACT_HBA_BITS = (1, 2)
CAM_ACT_WARN_BITS = (3, 2)
CAM_ACT_CMDACT_BIT = 20
CAM_ACT_STOPREQ_BIT = 21
CAM_ACT_DECCMDACT_BIT = 31

# *** personality-indexed decel cap (adurham fork) ***
# The driver's LongitudinalPersonality is the feel dial for FCA11-long braking STRENGTH (the commanded decel cap), the
# same lever the pedal law / launch ceiling / A_CRUISE_MAX read. Personality is cereal log.LongitudinalPersonality
# (0 = aggressive, 1 = standard, 2 = relaxed) and reaches here raw via CarControlSP.personality. Raw CR_VSM_DecCmd cap
# per tier (0.01 g/LSB, delivered decel ~= 0.67 x command):
#   relaxed    12 = 0.12 g  (~0.79 m/s^2 delivered)
#   standard   20 = 0.20 g  (~1.31 m/s^2 delivered)
#   aggressive 30 = 0.30 g  (== HYUNDAI_FCA11_LONG_MAX_DEC: today's cap, the max)
# Every tier is <= the PANDA firmware gate (HYUNDAI_FCA11_LONG_MAX_DEC, safety/modes/hyundai.h), which is
# personality-AGNOSTIC (it only rejects CR_VSM_DecCmd > 30). Indexing the cap python-side is therefore a strict subset of
# the firmware allow-window: NO safety/firmware change is needed and deployed firmware 00e086b9 stays valid. An unknown
# / out-of-range personality falls back to STANDARD (the middle, never the strongest).
PERSONALITY_AGGRESSIVE = 0
PERSONALITY_STANDARD = 1
PERSONALITY_RELAXED = 2
PERSONALITY_MAX_DEC = {
  PERSONALITY_RELAXED: 12,
  PERSONALITY_STANDARD: 20,
  PERSONALITY_AGGRESSIVE: HYUNDAI_FCA11_LONG_MAX_DEC,
}


def personality_max_dec(personality) -> int:
  """Raw CR_VSM_DecCmd cap for a LongitudinalPersonality raw value. Unknown / missing -> STANDARD (never the strongest).

  Mirrors the pedal law's normalization (``int(getattr(p, 'raw', p))``) so a capnp _DynamicEnum, a bare int, or junk
  all resolve the same way. The result is always <= HYUNDAI_FCA11_LONG_MAX_DEC (the panda gate)."""
  try:
    pid = int(getattr(personality, 'raw', personality))
  except (TypeError, ValueError):
    pid = PERSONALITY_STANDARD
  return PERSONALITY_MAX_DEC.get(pid, PERSONALITY_MAX_DEC[PERSONALITY_STANDARD])


def dec_cmd_from_accel(accel: float) -> int:
  """accel request (m/s^2, negative = brake) -> raw CR_VSM_DecCmd (0.01 g/LSB), clamp [1, cap].

  Zero for anything above the onset threshold (coast territory: not a brake request)."""
  if accel > FCA11_BRAKE_ONSET_ACCEL:
    return 0
  dec_g = (-accel / FCA11_BRAKE_GAIN) / 9.81
  return int(round(dec_g * 100))


def clamp_dec_cmd(dec_cmd: int, personality=PERSONALITY_STANDARD) -> int:
  """Clamp a raw CR_VSM_DecCmd to the personality's tier cap (>= 1). Standard reproduces the pre-personality cap."""
  return max(1, min(personality_max_dec(personality), dec_cmd))


@dataclass
class Fca11MirrorFrame:
  """The override fields for one mirrored FCA11 frame, before CRC/alive are applied."""
  dec_cmd: int
  prefill: bool
  warn: int
  cmd_act: bool
  dec_cmd_act: bool


def _get_bits(dat: bytes, start: int, length: int) -> int:
  v = 0
  for i in range(length):
    pos = start + i
    v |= ((dat[pos // 8] >> (pos % 8)) & 1) << i
  return v


def camera_requesting(dat: bytes) -> bool:
  """True if a bus-2 camera FCA11 frame carries a real actuation/warning request (hand back)."""
  if dat is None or len(dat) != 8:
    return False
  return (dat[1] != 0 or  # CR_VSM_DecCmd (byte 1): a commanded decel
          _get_bits(dat, CAM_ACT_PREFILL_BIT, 1) != 0 or
          _get_bits(dat, CAM_ACT_HBA_BITS[0], CAM_ACT_HBA_BITS[1]) != 0 or
          _get_bits(dat, CAM_ACT_WARN_BITS[0], CAM_ACT_WARN_BITS[1]) != 0 or
          _get_bits(dat, CAM_ACT_CMDACT_BIT, 1) != 0 or
          _get_bits(dat, CAM_ACT_STOPREQ_BIT, 1) != 0 or
          _get_bits(dat, CAM_ACT_DECCMDACT_BIT, 1) != 0)


def build_fca11_frame(cam_dat: bytes, cam_alive: int, m: Fca11MirrorFrame) -> bytes:
  """The camera's frame with ONLY the brake fields overridden + alive+1 + a fresh CRC.

  ``cam_dat`` is the camera's raw 8-byte FCA11 frame; every other byte (FCA_Status, Failinfo,
  DrvSetStatus, BeltCmd, RelativeVelocity, TimetoCollision, PAINT1, StopReq, HBA, and the
  undefined bits this car always carries, e.g. byte4 bit7 = 1 in 10549/10549 logged frames) is
  copied byte-for-byte, so the ESC only ever sees a shape it has already accepted - with a
  different brake command in it.
  """
  if cam_dat is None or len(cam_dat) != 8:
    raise ValueError(f"FCA11 mirror needs a fresh 8-byte camera frame, got {cam_dat!r}")
  b = bytearray(cam_dat)

  def clear(start: int, length: int) -> None:
    for i in range(length):
      pos = start + i
      b[pos // 8] &= (~(1 << (pos % 8))) & 0xFF

  def setb(start: int, length: int, val: int) -> None:
    for i in range(length):
      pos = start + i
      b[pos // 8] |= ((int(val) >> i) & 1) << (pos % 8)

  # override ONLY: Prefill, DecCmd, Warn, CmdAct, DecCmdAct, Alive. Everything else is mirrored.
  clear(0, 1)      # CF_VSM_Prefill
  clear(8, 8)      # CR_VSM_DecCmd
  clear(3, 2)      # CF_VSM_Warn
  clear(20, 1)     # FCA_CmdAct
  clear(31, 1)     # CF_VSM_DecCmdAct
  clear(35, 4)     # CR_FCA_Alive

  setb(0, 1, 1 if m.prefill else 0)
  setb(8, 8, int(m.dec_cmd) & 0xFF)
  setb(3, 2, int(m.warn) & 0x3)
  setb(20, 1, 1 if m.cmd_act else 0)
  setb(31, 1, 1 if m.dec_cmd_act else 0)
  setb(35, 4, int(cam_alive) & 0xF)

  out = bytes(b[:7])
  return out + bytes([hyundai_checksum(out)])


class Fca11LongBrake:
  """Owned by GasInterceptorCarController; steps once per 100 Hz controller frame.

  Inert (``update`` returns [] and ``braking`` is False) unless ``CP_SP.fca11Brake`` is set. All
  panda-facing state comes through ``CS`` attributes only (CarState writes them), so the class
  works identically against the real card loop and a test double.

  ``braking`` is True only while an ACTUATING frame is being sent; on the true -> false edge the
  class emits exactly one PASSIVE mirror frame (the camera's idle shape: all brake fields 0, Warn 0)
  to close the panda's FCA11-long episode. There is no duration budget: an episode runs as long as the
  planner (or a cal hold) asks and every gate holds.
  """

  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP, dbc_names: dict | None = None):
    self.CP = CP
    self.CP_SP = CP_SP
    self.enabled = bool(CP_SP.fca11Brake) and bool(CP_SP.enableGasInterceptor)
    # 0041: the params-gated AFFINE command law (HyundaiFca11AffineGain, DEFAULT OFF). Read ONCE here, never per
    # frame. OFF -> the 0.67 through-origin law and the 8 LSB floor, byte-identical to 0040.
    self.affine_gain = affine_gain_enabled(CP_SP)
    self.braking = False               # an FCA11 actuating frame was sent THIS frame
    self.blocked_until_resume = False  # a CUT latched: no braking until the next resume press
    # G9b: split the cut the way the panda does. _driver_cut (brake/gas/gear != D) re-arms on the resume
    # edge but clears on the first clean engaged frame; _hard_cut (camera hand-back / pedal fault) is not
    # re-armed. blocked_until_resume is their OR, so every existing consumer is unchanged.
    self._driver_cut = False
    self._hard_cut = False
    self._rearm_pending = False        # a resume edge armed a driver-latch re-arm; consumed by a clean frame
    # 0040: the single low-speed hand-over alert (below the old 12 km/h floor) is DELETED - it is now false. Replaced
    # by three owner-approved alerts (steady yellow / steady yellow / persistent red), all state read by the
    # superproject from CarStateSP. low_speed_alert is kept as an alias of supervise_stop for any stale consumer.
    self.supervise_stop = False        # YELLOW steady, slow + a stop requested
    self.stop_complete = False         # YELLOW steady at standstill while holding (an explicit hand-off, NOT a cap)
    self.brake_now = False             # RED persistent: ESC non-response or hold lost; clears ONLY on driver brake
    self.brake_now_reason = ""         # "no_response" | "hold_lost"
    self.low_speed_alert = False       # alias of supervise_stop (kept so pre-0040 readers do not crash)
    self._noresp_since: float | None = None
    self._hold_lost_armed = False
    # 0041 gating state: `_noresp_cmd_since` is the monotonic time the command has been continuously >= MIN_LSB
    # (the ESC-lag "command stable" gate); `_brake_now_clear_since` is the time BOTH trigger conditions last went
    # false (the 2 s self-heal timer, armed only while brake_now is up).
    self._noresp_cmd_since: float | None = None
    self._brake_now_clear_since: float | None = None
    # 0040: per-episode logger (/data/fca11-log/episodes.jsonl). See fca11_log.py for the schema and reasoning.
    self.log = fca11_log.EpisodeLogger()
    self._last_why = ""                # this frame's fca11_ok reason (the episode-end reason when !ok)
    # 0038: gas hold-off state. `_gas_release_frame` is the controller frame the pedal last read released (None =
    # none yet this controller); the hold-off refuses actuation for GAS_HOLDOFF_FRAMES after it. No edge is needed to
    # clear it (it just ages out), and gas is NOT part of _driver_cut any more.
    self._gas_release_frame: int | None = None
    self._gas_prev = False
    # 0030 (D1-b): the panda's camera-owns / pedal-fault hand-back (hyundai_fca11_long_cut) is fail-closed for the
    # REST OF THE IGNITION and cannot be cleared, so while FCA11-long is armed and that latch is set the panda will
    # refuse EVERY actuating frame for the whole drive. Today that failure is silent (the driver sees openpilot plan
    # stops it cannot execute). This mirrors the panda state the car layer CAN observe (_hard_cut); CarStateExt
    # publishes it as CarStateSP.fca11Unavailable for the superproject to raise a persistent driver alert.
    self.fca11_unavailable = False
    # 0030 (D2-iii): echo-confirmation of the passive close frame. The panda only CLOSES its FCA11-long episode on an
    # ACCEPTED passive 0x38D (hyundai.h:1027-1030); if the close frame was REFUSED (engage/disengage edge), the
    # episode never closes. The car layer sees the pandad echo of every 0x38D it sends (src 128 = sent, 192 = REJECTED)
    # and can re-owe a refused close frame instead of treating a frame that never landed as a hand-back. Inert until
    # the car layer feeds it via observe_echo (a no-op otherwise, so shipping it unwired is safe).
    self._pending_echo: bytes | None = None   # the last passive frame we emitted, awaiting its echo
    self._pending_echo_frame = -1              # controller frame at which it was emitted
    self._echo_rejected = False               # that frame came back 192 (never closed the episode)
    self._echo_wired = False                  # observe_echo has been called
    self._engaged_prev = False         # CC.enabled edge tracker (resume = the re-engage rising edge)
    self._dec_last_sent = 0
    self._sent_this_episode = False
    # ---- G5 anti-flap state ----
    self._ask_frames = 0               # consecutive frames of (window-open AND accel <= ONSET) since last reset
    self._on_frames = 0                # consecutive frames we have wanted to brake (the current episode age)
    # ---- G7 passive release edge ----
    self._release_pending = False      # a passive frame is owed (the braking true -> false edge)
    # ---- 0035: tool-side CAL watchdog (the panda no longer bounds a hold's duration) ----
    self._cal_frames = 0               # consecutive frames the cal sequencer has asked for a hold
    self.cal_watchdog_trips = 0        # diagnostics: how often the watchdog had to force a release
    # ---- 0029 CAL command mode (patch: 0029-hyundai-fca11-cal-command-mode) ----
    # A file-gated, autonomous sequencer that can replace the SOURCE of the decel command (the planner
    # ask) with an exact scripted level, WHILE every production gate stays on. Subordinate to the
    # production toggle: it is only ever ticked from the `enabled` branch of update(). Inert (and
    # byte-identical to the shipped behaviour) unless /data/fca11-cal/plan.json exists, parses, and is
    # fresh. See cal_mode.py for the full design.
    self.cal_seq = None
    if self.enabled and os.environ.get("FCA11_CAL_DIR") != "0":
      self.cal_seq = cal_mode.CalSequencer(os.environ.get("FCA11_CAL_DIR", cal_mode.CAL_DIR))
      self.cal_seq.mark_session(os.getpid())
    self._cal_active = False           # this frame is a CAL hold (drives the gas neutralization)

  # ---- window ----

  def _gas_holdoff_ok(self, frame: int) -> bool:
    """0038: True once the driver's gas has been released for at least GAS_HOLDOFF_FRAMES controller frames (or no
    release has been seen this controller). A plain elapsed-frame check - no edge is needed to clear it, it just
    ages out, so it can never lock out a later braking event."""
    if self._gas_release_frame is None:
      return True
    return (frame - self._gas_release_frame) >= GAS_HOLDOFF_FRAMES

  def fca11_ok(self, CS, frame: int = 0) -> tuple[bool, str]:
    """(ok, why-not) from the CarState-provided inputs. The panda re-polices everything anyway."""
    if not self.enabled:
      return False, "disabled"
    if self.blocked_until_resume:
      return False, "blocked_until_resume"
    # 0038: the driver's gas overrides braking per-frame (human always wins), and the short elapsed-frame hold-off
    # after it releases stops a 1-2 Hz brake/gas fight. This is NOT the old latch - it clears on time alone, so a NEW
    # brake request after the hold-off is served normally.
    if bool(CS.out.gasPressed):
      return False, "gas_pressed"
    if not self._gas_holdoff_ok(frame):
      return False, "gas_holdoff"
    # 0040: the speed floor ("below_floor") is DELETED. There is NO minimum speed to request FCA11 braking; the car
    # layer commands through zero and holds at a stop. See the FCA11_SUPERVISE_KPH block above.
    # pedal STATE 0 = NO_FAULT only. 4 STARTUP / 5 TIMEOUT are boot states the panda also accepts,
    # but from openpilot's side a pedal that has never taken a command must not be mixed with a
    # brake. 1 BAD_CHECKSUM / 2 SEND / 3 SCE / 6 INVALID are faults -> never brake.
    state = int(getattr(CS, "interceptor_state", 0))
    if state not in (0, 4, 5):
      return False, "pedal_fault"
    if CS.out.gearShifter != structs.CarState.GearShifter.drive:
      return False, "gear_not_d"
    cam = getattr(CS, "fca11_cam_frame", None)
    now = getattr(CS, "fca11_now_nanos", None)
    if cam is None or now is None:
      return False, "no_camera_frame"
    t_cam, dat = cam
    if len(dat) != 8 or (now - t_cam) > FCA11_CAM_FRESH_S * 1e9:
      return False, "stale_camera_frame"
    if camera_requesting(dat):
      return False, "camera_owns"
    return True, ""

  def _alerts(self, CS, frame: int, v_kph: float, dec_cmd: int) -> None:
    """0040 three-alert state machine (drives CarStateSP.superviseStop / stopComplete / brakeNow).

    YELLOW "supervise stop"  - steady while we are commanding a brake below FCA11_SUPERVISE_KPH.
    YELLOW "stop complete"   - steady while at/near a full stop and still holding (an explicit hand-off, NOT a cap:
                               the command keeps flowing; this only tells the driver to put a foot on the brake).
    RED    "BRAKE NOW"       - persistent; ESC non-response (measured aEgo < 0.4x expected for 500 ms while MOVING
                               above FCA11_NORESPONSE_MIN_KPH and commanding >= 8 LSB, itself sustained for the ESC
                               lag first) or hold lost (v > 2 km/h with no gas while we STILL command the stop hold).
                               Persistence is frame-to-frame survival, not "never clears": it clears immediately on
                               driver brake / gear != D / driver gas, and SELF-HEALS after both conditions have been
                               continuously false for FCA11_BRAKE_NOW_CLEAR_S. There is deliberately NO hold-duration
                               alert.

    0041 FIX (ship-blocker): the 0040 tests for both triggers were too weak. 'no_response' fired at EVERY standstill
    hold (aEgo ~ 0 at a red light, where delivering ~0 net decel is correct) and 'hold_lost' fired on openpilot's own
    launches (the arm ignored whether we were still holding). Both are gated here: no_response requires MOVING above
    5 km/h with a real, SUSTAINED command, and hold_lost disarms the instant we stop commanding the hold - so a
    planner launch (planner releases braking -> self.braking False) can never arm it.
    """
    if not self.enabled:
      self.supervise_stop = self.stop_complete = self.brake_now = False
      self.low_speed_alert = False
      self._noresp_since = None
      self._noresp_cmd_since = None
      self._hold_lost_armed = False
      self._brake_now_clear_since = None
      return
    now_us = time.monotonic() * 1e6
    gas = bool(CS.out.gasPressed)
    standstill = bool(getattr(CS.out, "standstill", False))
    moving = bool(v_kph > FCA11_NORESPONSE_MIN_KPH and not standstill)
    commanding = bool(dec_cmd >= FCA11_NORESPONSE_MIN_LSB)
    holding_cmd = bool(self.braking and dec_cmd > 0)   # STILL commanding the hold this frame

    # Hard clears: driver brake / gear != D / driver gas end the alert at once (never a latch). On a planner launch
    # v > 0 is expected, so these two conditions are what take a standing brake_now down; the self-heal (c) covers
    # the plain "condition went away" case.
    if bool(CS.out.brakePressed) or CS.out.gearShifter != structs.CarState.GearShifter.drive or gas:
      self.brake_now = False
      self.brake_now_reason = ""
      self._noresp_since = None
      self._hold_lost_armed = False
      self._brake_now_clear_since = None

    # (a) ESC non-response: ONLY while MOVING above 5 km/h with a real command (>= 8 LSB) that has been SUSTAINED for
    # at least the ESC lag (0.62 s) before the 500 ms measurement timer may run. At/below a stop this is not a fault
    # (a held stop correctly delivers ~0 net decel) - that case is hold_lost's job. The command-stable gate resets
    # whenever the command drops below 8 LSB, so the timer never spans a release/re-ask.
    if commanding:
      if self._noresp_cmd_since is None:
        self._noresp_cmd_since = now_us
    else:
      self._noresp_cmd_since = None
    expected = dec_cmd * 0.01 * 9.81 * FCA11_BRAKE_GAIN
    aego = float(getattr(CS.out, "aEgo", 0.0) or 0.0)
    nonresp = bool(moving and commanding and
                   aego > -(FCA11_NORESPONSE_FACTOR * expected) and
                   self._noresp_cmd_since is not None and
                   (now_us - self._noresp_cmd_since) >= FCA11_NORESPONSE_CMD_STABLE_S * 1e6)

    # (b) hold lost: ARM only while we are ACTIVELY COMMANDING a stop hold at/near a full stop; once armed it LATCHES
    # (a hold established at a stop that then creeps is exactly the lost condition). DISARM the instant the hold is
    # broken - we stop braking (a planner release / launch), the command drops below 8 LSB, gear != D, or the driver
    # brakes/gases. Fires only while the hold is STILL commanded and the car moves anyway with no driver gas.
    arm_now = bool(self.braking and dec_cmd > 0 and (v_kph <= FCA11_STOP_KPH or standstill))
    if arm_now:
      self._hold_lost_armed = True
    elif (not self.braking or dec_cmd < FCA11_NORESPONSE_MIN_LSB or
          CS.out.gearShifter != structs.CarState.GearShifter.drive or bool(CS.out.brakePressed) or gas):
      self._hold_lost_armed = False
    hold_lost = bool(self._hold_lost_armed and holding_cmd and v_kph > FCA11_HOLD_LOST_KPH and not gas)

    if self.brake_now:
      # (c) persistent: only the hard clears above or the 2 s self-heal may take it down. While either condition is
      # true the latch stands; once BOTH have been continuously false for FCA11_BRAKE_NOW_CLEAR_S it self-heals.
      if not nonresp and not hold_lost:
        if self._brake_now_clear_since is None:
          self._brake_now_clear_since = now_us
        elif (now_us - self._brake_now_clear_since) >= FCA11_BRAKE_NOW_CLEAR_S * 1e6:
          self.brake_now = False
          self.brake_now_reason = ""
          self._noresp_since = None
          self._brake_now_clear_since = None
      else:
        self._brake_now_clear_since = None
    else:
      self._brake_now_clear_since = None
      if nonresp:
        if self._noresp_since is None:
          self._noresp_since = now_us
        elif (now_us - self._noresp_since) >= FCA11_NORESPONSE_MS * 1000.0:
          self.brake_now = True
          self.brake_now_reason = "no_response"
      else:
        self._noresp_since = None
      if not self.brake_now and hold_lost:
        self.brake_now = True
        self.brake_now_reason = "hold_lost"

    self.supervise_stop = bool(self.braking and v_kph <= FCA11_SUPERVISE_KPH)
    self.stop_complete = bool(self.braking and (v_kph <= FCA11_STOP_KPH or standstill))
    self.low_speed_alert = self.supervise_stop   # alias

  def _log_tick(self, CS, frame: int, dec_cmd: int) -> None:
    """0040: feed the per-episode logger. ``dec_cmd`` >= 1 means we COMMANDED an actuating frame this frame.

    An episode opens on the first commanded frame and closes on the braking true -> false edge, at which point ONE
    compact JSON line is appended. Best-effort throughout: it must never affect control.
    """
    if not self.log.enabled:
      return
    v_kph = float(CS.out.vEgo) * 3.6
    wheel_kph = getattr(CS, "fca11_wheel_min_kph", None)
    if dec_cmd >= 1:
      if not self.braking:
        self.log.begin(frame, v_kph, wheel_kph)
      self.log.note_frame(frame, dec_cmd, v_kph, wheel_kph, getattr(CS.out, "aEgo", None),
                          bool(CS.out.standstill), FCA11_STOP_KPH)
    elif self.log._open:
      reason = self._last_why if self._last_why not in ("", "disabled") else "release"
      self.log.flush(frame, reason, FCA11_STOP_KPH)

  def observe_echo(self, echoes) -> None:
    """0030 (D2-iii): the car layer's half of the echo-confirmed close.

    ``echoes`` is the pandad echo of every host 0x38D we sent this CAN tick: an iterable of (src, dat) with
    src 128 = accepted / forwarded by the panda TX hook, src 192 = REJECTED by it. The panda CLOSES its
    FCA11-long episode (drops its rate reference to 0) ONLY on an ACCEPTED passive 0x38D. If it comes back
    REFUSED we RE-OWE the close frame and retry, so the hand-back is never assumed on a frame that never
    landed. (0035: there is no cooldown to stamp any more; the echo only drives the re-owe.)

    Called once per controller frame from create_gas_command. Inert (and byte-identical to the shipped
    behaviour) until the first call, which is what sets ``_echo_wired``.
    """
    self._echo_wired = True
    if not echoes:
      return
    # 0035: a REFUSED ACTUATING frame while we are still braking restarts the ramp. The panda's onset rule resets its
    # rate reference to 0 once our stream has been stale > TX_STALE (the camera has re-owned the ESC), and a refused
    # frame does not refresh the stream - so if a host/USB hiccup ever leaves a gap, continuing at the old level would
    # be refused on EVERY frame until the planner released (a silent loss of the rest of the brake). Dropping our own
    # reference to 0 makes the next frame RATE_STEP, which the panda accepts, and the ramp climbs back in ~150 ms.
    # It can only ever LOWER the command.
    if self.braking and any(int(src) == 192 and len(dat) == 8 and dat[1] != 0 for src, dat in echoes):
      self._dec_last_sent = 0
    # 0040: count the PANDA's own refusals of our ACTUATING frames (src 192 = the TX hook rejected it) for the
    # episode log - the firmware's verdict is the ground truth on whether the ESC was ever asked.
    if self.log.enabled and self.braking:
      self.log.note_panda_refusal(sum(1 for src, dat in echoes if int(src) == 192 and len(dat) == 8 and dat[1] != 0))
    if self._pending_echo is None:
      return
    pending = self._pending_echo
    for src, dat in echoes:
      if bytes(dat) != pending:
        continue
      if int(src) == 192 and not self.braking:
        # the close frame was REFUSED: the panda episode is STILL OPEN. Re-owe it (a later send slot retries).
        # 0035: never while braking again - re-owing then would force the live brake off for a slot (a silent
        # mid-brake truncation); the new episode's own release will owe its own close frame.
        self._echo_rejected = True
        self._release_pending = True
      self._pending_echo = None
      return

  def _passive_frame(self, CS, frame: int) -> list[CanData]:
    """One PASSIVE mirror frame (the camera's own idle shape: DecCmd 0, Prefill 0, Warn 0, CmdAct 0,
    DecCmdAct 0, fresh alive + CRC, every other byte the camera's).

    This is the clean release / hand-back frame the panda accepts while ``controls_allowed`` AND
    ``heartbeat_engaged`` (safety/modes/hyundai.h); it is what ENDS an actuation episode (the panda's
    rate reference drops to 0). Only sent on the FCA11_SEND_EVERY slot, same 50 Hz stream
    as the actuating frames, and only when a fresh camera frame is in hand (never a hardcoded frame -
    the mirror is always sourced from the latest bus-2 0x38D).
    """
    cam = getattr(CS, "fca11_cam_frame", None)
    if cam is None or len(cam[1]) != 8 or frame % FCA11_SEND_EVERY != 0:
      return []
    t_cam, dat = cam
    cam_alive = (dat[4] >> 3) & 0xF  # CR_FCA_Alive is 35|4 = byte4 bits 3-6
    m = Fca11MirrorFrame(dec_cmd=0, prefill=False, warn=0, cmd_act=False, dec_cmd_act=False)
    return [CanData(FCA11_ADDR, build_fca11_frame(dat, (cam_alive + 1) & 0xF, m), 0)]

  def update(self, CC: structs.CarControl, CS, frame: int, personality=PERSONALITY_STANDARD) -> list[CanData]:
    sends: list[CanData] = []
    if not self.enabled:
      self.braking = False
      self.low_speed_alert = False
      # 0040: the three alerts and the logger are all inert when the feature is off.
      self.supervise_stop = self.stop_complete = self.brake_now = False
      self.brake_now_reason = ""
      self._noresp_since = None
      self._hold_lost_armed = False
      # 0041: and the gating timers, so a disable is a clean start.
      self._noresp_cmd_since = None
      self._brake_now_clear_since = None
      self.log.flush(frame, "disabled", FCA11_STOP_KPH)
      # G5: an inert controller must not carry debounce/min-on state into a later re-enable.
      self._ask_frames = 0
      self._on_frames = 0
      # G7: likewise no owed close frame survives a disable (a re-arm is a clean panda episode too).
      self._release_pending = False
      self._cal_frames = 0
      # G9b: and no cut/re-arm state either (a disable is a fresh start, matching the panda's init).
      self._driver_cut = False
      self._hard_cut = False
      self._rearm_pending = False
      self.blocked_until_resume = False
      self._cal_active = False
      # 0038: no gas hold-off state either.
      self._gas_release_frame = None
      self._gas_prev = False
      return sends

    # 0029 CAL: expose last frame's braking to the sequencer's release/recovery detection (same
    # pattern as fca11_now_nanos: the controller writes a CS attribute the class later reads).
    CS._fca11_braking_now = self.braking

    # resume model: any disengage -> re-engage transition clears a cut latch (the pause/resume
    # press is the only engagement path in pedal mode, so this IS the driver's deliberate press).
    # G9b (drive-14): the resume edge only ARMS the re-arm; the driver latch clears on the first CLEAN
    # frame after the grant, so an input merely HELD at the grant (engaging with the foot still on the
    # gas: gasReleased mid-engagement) stops barring FCA11 the moment it is actually released instead
    # of killing braking for the whole engagement (the panda refuses every frame otherwise).
    engaged = bool(CC.enabled)
    if engaged and not self._engaged_prev:
      self._rearm_pending = True
    self._engaged_prev = engaged

    # latch the cut on what the car layer CAN observe (camera request, driver brake, gear).
    # 0038: GAS is deliberately NOT part of _driver_cut any more (owner requires a gas press never lock out future
    # braking). It is enforced per-frame in fca11_ok below plus the elapsed-frame hold-off. The DRIVER latch
    # (_driver_cut) mirrors the panda's hyundai_fca11_long_driver_cut (brake / gear != D): it re-arms on the resume
    # edge but clears only on the first clean engaged frame. The CAMERA hand-back and a pedal fault go to _hard_cut,
    # which matches the panda's hyundai_fca11_long_cut and is NOT re-armed.
    driver_cut = (bool(CS.out.brakePressed) or
                  CS.out.gearShifter != structs.CarState.GearShifter.drive)
    if CC.longActive and driver_cut:
      self._driver_cut = True
    elif self._rearm_pending and engaged:
      self._driver_cut = False
      self._rearm_pending = False
    cam = getattr(CS, "fca11_cam_frame", None)
    # 0030 (D1-a): latch the camera hand-back on the camera request REGARDLESS of longActive, matching the panda
    # EXACTLY. The panda's hyundai_fca11_long_rx sets camera_owns / cut on ANY bus-2 0x38D with an actuation/warning
    # field, with no host-engagement condition at all (hyundai.h:228-236), and only hyundai_init clears it. The old
    # `CC.longActive and ...` gate was the divergence: the triggering stock FCW on drive 14d arrived while openpilot
    # was DISENGAGED, so Python never latched, kept believing it could brake, and kept commanding into a refused
    # wall for the remaining ~16 min of the ignition (1983 refused actuating frames, all three 20@90 cal reps).
    if cam is not None and camera_requesting(cam[1]):
      self._hard_cut = True
    if not engaged:
      self._rearm_pending = False
    self.blocked_until_resume = self._driver_cut or self._hard_cut
    # 0038: track the driver's gas RELEASE edge (for the elapsed-frame hold-off). CS.out.gasPressed is the driver's
    # physical pedal (the panda's gas source is the interceptor sensor, not our throttle command). On the frame the
    # pedal releases, stamp the frame; _gas_holdoff_ok refuses for GAS_HOLDOFF_FRAMES after it. No latch.
    gas_now = bool(CS.out.gasPressed)
    if self._gas_prev and not gas_now:
      self._gas_release_frame = frame
    self._gas_prev = gas_now
    # 0030 (D1-b): surface the hand-back. _hard_cut == the panda's hyundai_fca11_long_cut (camera owns / pedal fault)
    # and is NOT re-armed, so while it is set FCA11-long braking is unavailable for the rest of this ignition.
    self.fca11_unavailable = bool(self.enabled and self._hard_cut)

    # 0040: this frame's speed (km/h); the alert state machine + logger both read it.
    v_kph_now = float(CS.out.vEgo) * 3.6

    if not CC.longActive:
      # R7-D: a mid-episode longActive drop must still close the FCA11 episode with a passive frame. We cannot
      # send it here (longActive is off), so we OWE it: the first longActive send slot back emits it before any
      # new actuation. An episode that never opened owes nothing.
      if self.braking:
        self._release_pending = True
      self.braking = False
      self._cal_frames = 0
      self._dec_last_sent = 0
      # G5: disengaging long clears the debounce/min-on state; the next engagement starts clean.
      self._ask_frames = 0
      self._on_frames = 0
      self._alerts(CS, frame, v_kph_now, 0)   # 0040: braking is over -> supervise/stop-complete off (brake_now persists)
      self._log_tick(CS, frame, 0)           # 0040: close the episode, if one was open
      return sends

    accel = float(CC.actuators.accel)
    ok, _why = self.fca11_ok(CS, frame)
    self._last_why = _why  # 0040: this frame's gate reason -> the episode's end_reason if !ok

    # ---- 0029 CAL: step the autonomous sequencer and set the scripted ask source -------------------
    # The sequencer decides (off the plan file + progress) whether a scripted level is DUE now. It is
    # ticked ONLY here, in the enabled+longActive path, and ONLY on the 50 Hz SEND slot
    # (frame % FCA11_SEND_EVERY == 0) -- the same slot the FCA11 frame goes out on. Deciding on the
    # send slot means cal_on can never be true on a non-send frame, so a hold's first ACTUATING frame
    # is its first TICK frame: there is no same-frame gap between "cal active" (gas forced to 0) and
    # the first dec_cmd frame (Fable's gas-neutralization timing item). cal_on is LATCHED between
    # ticks; it can only ever be true while the last tick commanded a hold AND a gate still holds.
    if self.cal_seq is not None and frame % FCA11_SEND_EVERY == 0:
      # 0035: no budget mirror is published any more (the panda has no duration budget); the sequencer
      # arms on its own conditions and bounds the hold itself (CAL_MAX_HOLD_S + the watchdog below).
      self.cal_seq.tick(CC, CS, frame)
    # cal_on is the scripted ASK, gated by `ok` exactly like a planner ask: a production veto (camera
    # owns, gear out of D, below floor, pedal fault, driver-cut latch) suppresses it on the SAME frame.
    cal_ask = bool(self.cal_seq is not None and self.cal_seq.active_actuation)
    # ---- 0035 tool-side CAL WATCHDOG -------------------------------------------------------------
    # The panda's 2.5 s budget used to be the hard backstop on a scripted hold. It is gone, so the TOOL
    # bounds itself, independently of the sequencer's own hold arithmetic: a cal ask that stays up longer
    # than CAL_MAX_HOLD_S + the watchdog margin is a sequencer fault -> drop the ask this frame, tell the
    # sequencer to abort the rep (outcome "watchdog"), and let the production release path emit the
    # passive close frame. Bounds ONLY the scripted ask - a planner ask is never silenced by this.
    self._cal_frames = self._cal_frames + 1 if cal_ask else 0
    if cal_ask and self._cal_frames > cal_mode.CAL_WATCHDOG_FRAMES:
      self.cal_watchdog_trips += 1
      self.cal_seq.watchdog_abort(frame)
      cal_ask = False
      self._cal_frames = 0
    cal_on = bool(cal_ask and ok)
    self._cal_active = False   # re-latched below iff THIS frame commands a scripted hold

    # ---- G5 anti-flap decision ----------------------------------------------------------------
    # ask: the planner wants a brake THIS frame and every pre-condition (fca11_ok) holds. `ask` drives
    # the onset debounce only; the release/min-on logic below consumes the *raw* accel.
    ask = bool(ok and accel <= FCA11_BRAKE_ONSET_ACCEL)
    self._ask_frames = self._ask_frames + 1 if ask else 0

    # onset: a NEW episode opens only after ONSET_HOLD_FRAMES consecutive ask frames (0.2 s), EXCEPT a
    # hard ask (<= -1.2) which opens immediately so a real demand is never delayed (safety/coverage).
    # holding: an EXISTING episode stays open until the ask rises above the release threshold (-0.30)
    # AND the minimum on-time (0.6 s) has elapsed. `self._on_frames` is the previous frame's episode age.
    # Every `ok == False` (blocked latch, below floor, pedal fault, gear, stale/no camera, camera owns)
    # makes both terms False -> immediate release: the safety paths are untouched.
    # 0029 CAL: `cal_on` is the scripted ask. While a cal hold is active the release/min-on logic below
    # is bypassed (held True) so the scripted level is held for the sequencer's whole hold window --
    # but STILL AND ONLY while `ok` holds. When the sequencer drops cal_on, the episode releases through
    # the PRODUCTION path (a veto stops braking on this frame; a soft cal end lets the same release
    # logic run).
    onset = bool(ok and (self._ask_frames >= FCA11_ONSET_HOLD_FRAMES or accel <= FCA11_ONSET_BYPASS_ACCEL))
    holding = bool(self.braking and ok and
                   (accel <= FCA11_BRAKE_RELEASE_ACCEL or self._on_frames < FCA11_MIN_ON_FRAMES))
    want_brake = bool(onset or holding or cal_on)
    # 0035: the G7 budget-edge stop and the 3 s cooldown hold-off that used to sit here are DELETED with the
    # panda's budget/cooldown. The only things that end a brake are the planner/cal release above and the `ok`
    # gates (driver, gear, floor, camera, pedal, staleness). The one remaining hold-off is the close-frame
    # ORDERING: a passive frame still owed from the previous episode goes out first, so a re-brake waits at most
    # ONE 50 Hz send slot (<= 20 ms). It cannot wedge: the owed frame is emitted on the next send slot whenever a
    # fresh camera frame exists (and without one `ok` is False anyway), and observe_echo never re-owes a close
    # frame while braking.
    if want_brake and self._release_pending:
      want_brake = False

    dec_cmd_now = 0   # 0040: the level COMMANDED this frame (0 = no actuating frame); feeds the alerts + logger

    if want_brake:
      # 0029 CAL: latch whether THIS frame is part of a scripted hold AND the frame is being commanded
      # (ok True). This drives the gas neutralization in GasInterceptorCarController.create_gas_command.
      self._cal_active = bool(cal_on and ok)
      # Release hysteresis: inside the band [-0.50, -0.30] the command is floored at the ONSET value so
      # the ESC never sees a 0 mid-episode (a 0 would be an immediate release, defeating the hysteresis).
      # 0041: the OPT-IN affine law (self.affine_gain, DEFAULT OFF) replaces the through-origin inverse with the
      # fitted affine plant inverse (dec_cmd_from_accel_affine); OFF -> byte-identical to 0040.
      accel_ask = min(accel, FCA11_BRAKE_ONSET_ACCEL)
      if self.affine_gain:
        dec_cmd = dec_cmd_from_accel_affine(accel_ask)
      else:
        dec_cmd = dec_cmd_from_accel(accel_ask)
      # production tiering of the planner ask (unchanged when cal is off -> byte-identical). 0041: under the affine
      # law the command may legitimately taper to 0 (no honest request), so the floor is the honest level, not the
      # fixed 1 the through-origin law uses; OFF keeps the 0040 clamp exactly (raw CR_VSM_DecCmd >= 1).
      if self.affine_gain:
        dec_cmd = min(dec_cmd, personality_max_dec(personality))
      else:
        dec_cmd = clamp_dec_cmd(dec_cmd, personality)
      # 0041 (low-speed refusal fix): the affine hold/release floor is applied to the REQUEST here, BEFORE the
      # +RATE_STEP rate limiter below -- NOT after it. Applied after, the floor lifted the FIRST frame of an episode
      # straight to 8 LSB, over the panda's onset gate (the firmware allows only +RATE_STEP over the last ACCEPTED
      # frame, which is 0 at an episode open), so panda refused EVERY actuating frame (dec=8 last=0) and the brake
      # never engaged at low speed. Applied to the request, the limiter still governs the ONSET: request 8 ->
      # min(8, 0+4) = 4 (first frame, ACCEPTED) -> min(8, 4+4) = 8 (ACCEPTED) -> 4 -> 8 in ~40 ms. The floor applies
      # ONLY when the law commands > 0 (an honest taper to 0 stays 0) and never when cal sets the level itself.
      if self.affine_gain and not cal_on and dec_cmd > 0:
        in_hold_band = bool(v_kph_now <= FCA11_AFFINE_STOP_BAND_KPH or getattr(CS.out, "standstill", False))
        dec_cmd = max(dec_cmd, FCA11_AFFINE_HOLD_LSB if in_hold_band else FCA11_AFFINE_RELEASE_LSB)
        dec_cmd = min(dec_cmd, personality_max_dec(personality))
      if cal_on:
        # 0029 CAL: the scripted level is a FLOOR that can only ADD braking -- never silence the
        # planner. max() with the scripted level means: planner coasting -> the cal level (the
        # controlled step the fit needs); a HARDER planner ask (e.g. a lead cuts in mid-hold) -> the
        # planner wins, so cal can never make the car brake LESS than production would (Fable: closes
        # the planner-silenced window). The level is clamped to the PANDA gate 30 LSB = 0.30 g, NOT the
        # personality tier cap -- cal must reach 0.30 g regardless of the feel dial, and 30 is the
        # personality-agnostic firmware gate, so there is NO new authority. (Deliberate, disclosed
        # exception: cal may exceed the owner's configured tier cap, never the panda gate.)
        dec_cmd = max(dec_cmd, max(cal_mode.CAL_MIN_LSB, min(cal_mode.CAL_MAX_LSB, int(self.cal_seq.level_lsb))))
      # rate-limit the REQUEST too (the panda enforces its own): at most +RATE_STEP (0.04 g) per
      # camera period of growth; release is immediate. Reference = our last SENT frame.
      # 0029 CAL: this +RATE_STEP-per-SENT-frame clamp IS the clean entry step the BITE fit needs --
      # a scripted step enters through the panda-rate ramp even for a low level (where the production
      # path's LPF + slew would otherwise smear the step). Do NOT bypass it for cal.
      dec_cmd = min(dec_cmd, self._dec_last_sent + HYUNDAI_FCA11_LONG_RATE_STEP)
      if cal_on:
        dec_cmd = max(cal_mode.CAL_MIN_LSB, min(cal_mode.CAL_MAX_LSB, dec_cmd))  # <= panda gate (30)
      elif not self.affine_gain:
        # 0041: the affine ON path applies its hold/release floor to the REQUEST *before* this limiter (above), so the
        # panda's onset gate still governs the ramp and there is nothing left to clamp for it here. OFF keeps the 0040
        # clamp exactly.
        dec_cmd = clamp_dec_cmd(dec_cmd, personality)  # first frame of an episode: <= RATE_STEP, then ramps

      t_cam, dat = cam
      if frame % FCA11_SEND_EVERY == 0:
        cam_alive = (dat[4] >> 3) & 0xF  # CR_FCA_Alive is 35|4 = byte4 bits 3-6
        m = Fca11MirrorFrame(dec_cmd=dec_cmd, prefill=True, warn=3, cmd_act=True, dec_cmd_act=False)
        out = build_fca11_frame(dat, (cam_alive + 1) & 0xF, m)
        sends.append(CanData(FCA11_ADDR, out, 0))
        # R7-A: the panda allows +RATE_STEP over the last ACCEPTED frame, and we only SEND on the 50 Hz
        # slot. Advancing the reference on every 100 Hz frame (the pre-G8 code) made the SENT ramp step
        # +2*RATE_STEP, so the panda refused every actuating frame after the first. The reference must
        # advance ONLY on frames actually sent (the panda's last-accepted frame).
        self._dec_last_sent = dec_cmd
      self.braking = True
      self._sent_this_episode = True
      dec_cmd_now = dec_cmd   # 0040: the level COMMANDED this frame (feeds the alerts + logger)
    else:
      # not braking: the camera's own FCA11 keeps flowing, and G7 sends ONE passive frame to close the
      # panda's episode (see _passive_frame). The camera path alone does NOT pass the panda TX hook, so
      # without this the panda's act_active stays latched across brakes (round-5 N1).
      was_braking = self.braking
      self.braking = False
      self._dec_last_sent = 0
      self._sent_this_episode = False
      dec_cmd_now = 0   # 0040: no actuating frame this frame
      if was_braking:
        # braking true -> false edge: end the panda episode, owe a passive frame (the clean hand-back).
        self._release_pending = True
      if self._release_pending:
        sends += self._passive_frame(CS, frame)
        if sends:  # emitted (frame % SEND_EVERY == 0); if [] we retry next frame
          self._release_pending = False
          if self._echo_wired:
            # 0030 (D2-iii): record the frame we just put on the wire and wait for the pandad echo (observe_echo):
            # refused (src 192) RE-OWES the close frame so the hand-back is retried.
            self._pending_echo = bytes(sends[-1].dat)
            self._pending_echo_frame = frame
            self._echo_rejected = False

    # episode age AFTER this frame's decision (consumed by `holding` on the next frame).
    self._on_frames = self._on_frames + 1 if want_brake else 0

    # 0040: alert state (over this frame's braking) and the per-episode log (one line on the true -> false edge).
    self._alerts(CS, frame, v_kph_now, dec_cmd_now)
    self._log_tick(CS, frame, dec_cmd_now)

    return sends
