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
from opendbc.sunnypilot.car.hyundai.values import (HYUNDAI_FCA11_LONG_MAX_DEC, HYUNDAI_FCA11_LONG_RATE_STEP,
                                                   HYUNDAI_FCA11_PEDAL_STATE_OK, HYUNDAI_FCA11_CAM_BRAKE_MASK,
                                                   HYUNDAI_FCA11_CAM_WARN_MASK, HYUNDAI_FCA11_CAM_QUIET_FRAMES)
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

# *** 0042 (B): echo-based refusal surfacing ***
# The panda's fail-closed cut latches (hyundai_fca11_long_cut / driver_cut) can refuse a SUSTAINED wall of our
# actuating frames for the rest of an ignition, and that failure was otherwise SILENT (route 156: 3143/3143 actuating
# frames refused, no driver alert). The car layer sees the pandad echo of every 0x38D it sends (src 128 = accepted by
# the TX hook, src 192 = REJECTED). Count CONSECUTIVE refused ACTUATING echoes while braking; a streak this long
# (~3 x the 20 ms 50 Hz send slot = ~60 ms) is a wall, not a transient, and sets the surfaced _refused_cut. An
# ACCEPTED actuating echo (src 128) means the wall is gone: reset + self-heal. No send suppression rides on this -
# refused frames never reach the car, so keeping our sends live keeps the ground truth flowing with zero deadlock.
FCA11_REFUSED_STREAK_CUT = 3

# *** 0053: NON-LATCHING camera hand-back (owner policy 2026-10-21) ***
# "A camera WARNING must not stop openpilot braking. A warning just means we got close to something. If the camera
# starts sending actual BRAKE messages, hand back - but do not latch until the next ignition: when the camera stops
# braking, openpilot resumes." Both incidents that shaped the old latch were WARNING-only (route 14d: 93,170 camera
# frames, 0 braking-class; route 15b: 23,834 frames, 0 braking-class, a 1 s warn/prefill burst that cost the last 99 s
# of the drive). The classes are the shared masks in values.py (one definition, also the panda's):
#   BRAKING (hand back): HBACmd, BeltCmd, DecCmd, CmdAct, StopReq, DecCmdAct.   WARNING (keep braking): Prefill, Warn.
# State machine (cam_brake + a saturating quiet counter, counted in CAMERA FRAMES - never wall time - and NEVER gated on
# engagement): a braking-class frame sets cam_brake and zeroes the counter, a non-braking frame advances it, and the
# hand-back is released at HYUNDAI_FCA11_CAM_QUIET_FRAMES + FCA11_CAM_QUIET_MARGIN. The panda releases at
# HYUNDAI_FCA11_CAM_QUIET_FRAMES, so this layer is STRICTER on both edges: it stops on the same frame it sees a braking
# message (before any send decision) and resumes 2 camera frames after the panda would allow it - the only possible
# disagreement is "we withheld a frame the panda would have accepted". A silent camera does not advance the counter.
FCA11_CAM_QUIET_MARGIN = 2
FCA11_CAM_RESUME_FRAMES = HYUNDAI_FCA11_CAM_QUIET_FRAMES + FCA11_CAM_QUIET_MARGIN
# *** 16a: the RELEASE TAIL - the camera keeps ownership of refusals for a moment after WE let go ***
# This layer counts its quiet tail in CAMERA frames; the panda counts its own in 50 Hz TICKS. The +MARGIN above only makes
# us stricter when the camera's 0x38D rate is at or below the tick rate - above it the two clocks diverge the other way
# and we release FIRST, into a hand-back that is still live on the panda's side. Route 16a (camera braking frames through
# 130.64-136.11 s): we let go, the panda refused the frames we resumed with at 136.07, 136.09 and 136.11, the streak
# latched at 136.13 and "FCA11 Braking Unavailable" stayed up 170 s - while the car had braking again ~1.5 s after the
# camera went quiet. Those refusals were the camera's hand-back, not a wall. So a release arms this tail (controller
# frames = the panda's own quiet window) during which refusals stay camera-owned and cannot count: see
# _camera_owns_refusal. A camera that is still quiet after the tail owns nothing, so a real wall still latches.
FCA11_CAM_RELEASE_TAIL_FRAMES = HYUNDAI_FCA11_CAM_QUIET_FRAMES

# *** 0053: refusal-cut PROBATION after a resume ***
# The echo refusal cut (FCA11_REFUSED_STREAK_CUT, consecutive) is what actually protects against the 14d failure
# (commanding into a refused wall) and stays independent of the camera state. For the first FCA11_PROBATION_ECHOES
# actuating echoes after a hand-back clears and after every (re)engage, a CUMULATIVE count is ALSO kept (an accepted
# echo does not reset it): FCA11_PROBATION_REFUSED_CUT refused actuating echoes inside the window trip the cut too, so an
# intermittent wall (accept 1 / refuse many) right after we resume cannot hide behind the consecutive rule. A probation
# trip holds until the window closes. No probe decel is ever sent to test acceptance: only real braking feeds it.
FCA11_PROBATION_ECHOES = 50
FCA11_PROBATION_REFUSED_CUT = 5

# *** 0053: Python <-> panda DIVERGENCE guard ***
# After 0053 the ONLY panda rule that refuses a well-formed PASSIVE 0x38D is its cam_brake (a passive frame is accepted
# disengaged, outside the window, with a pedal fault ...). So FCA11_DIVERGENCE_PASSIVE_STREAK consecutive REFUSED passive
# echoes while this layer believes the camera is NOT braking = the two layers disagree (e.g. a camera frame the host
# never saw). Reaction: adopt the panda's view (cam_brake = True, counter 0 - the stricter direction) and raise the
# divergence alert; the FCA11_DIVERGENCE_LATCH_EVENTS-th event in one ignition latches: no 0x38D for the rest of it.
# Armed only after the first ACCEPTED 0x38D echo (a panda that is not armed refuses everything).
FCA11_DIVERGENCE_PASSIVE_STREAK = 2
FCA11_DIVERGENCE_LATCH_EVENTS = 3
# Attribution of a refusal WALL (FCA11_REFUSED_STREAK_CUT refused actuating echoes while we see no camera brake): the
# episode is ended for at most this many controller frames (3 send slots) so the owed PASSIVE close frame goes out and
# its echo says whether the panda is handing back to the camera (refused) or the wall is something else (accepted).
FCA11_ATTRIB_HOLD_FRAMES = 6

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
# THIS IS DEFERRED / OPT-IN. With the param OFF, NOTHING here runs and the module is byte-identical to 0040: the 0.67
# law inverts exactly as before and the 8 LSB floor is untouched. (0044: the old FCA11_AFFINE env override is removed
# and refused - CP_SP.fca11AffineGain, written by setup_interfaces from the param, is the only way in.) No value of
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


# *** 0043: FCA11_SHAPE_* — the OPT-IN BRAKE-SHAPING command law (behind HyundaiFca11AffineGain, DEFAULT OFF) ***
#
# WHY (measured 2026-10-10, routes 155/157; wsB-feel). An FCA11 brake ONSET is a near-STEP. The deployed law's
# request path (0.67 through-origin inverse) plus the +RATE_STEP rate limiter sends 4 LSB and then 8 LSB within
# 40-60 ms; the ESC's own plant has a CLIFF between 4 and 8 LSB (delivered EXTRA decel ~0.15 m/s^2 at 4 LSB vs
# ~1.42 at 8), so the 4->8 step is a ~1.3 m/s^2 delivered step -> onset jerk median 7.6 m/s^3 (max 38; comfort
# <= 2-3). 86% of stops are grabby. The steady plant, though, is AFFINE (delivered = BITE + K*cmd_g, K = 8.23
# [8.07, 8.39] m/s^2/g, BITE = 0.85 [0.82, 0.87], r^2 = 0.69, n = 2701), and the 0.67 law OVER-delivers by ~0.9
# m/s^2 (steady moving delivered -1.71 vs ask -0.77).
#
# WHAT THIS LAW DOES (ON path, gated by HyundaiFca11AffineGain; OFF = BYTE-IDENTICAL to 0042):
#   1. demand:      want = max(0.0, -accel_ask); no coast subtraction.
#   2. level (moving, above FCA11_SHAPE_HOLD_BAND_KPH): a piecewise-MONOTONE inverse of the measured steady
#      curve, clamped [FCA11_SHAPE_FLOOR_LSB, tier cap]. Anchors (extra -> LSB): (0.15 -> 4) and (1.50 -> 8). For
#      extra >= 1.50 the affine inverse cmd = round(((extra - BITE)/K)*100), clamped [8, tier cap]; for extra in
#      (0.15, 1.50) a LINEAR interpolation across the cliff; extra < 0.15 -> the 4 LSB floor (never below 4 while
#      moving).
#   3. onset shaping: an episode (re)open starts at the floor and HOLDS FCA11_SHAPE_ONSET_HOLD_S (0.2 s since
#      0043 v3 / C1, was 0.4 s; in SENT-frame time) before growth; then a JERK-LIMITED slew (target delivered jerk FCA11_SHAPE_JERK_TARGET
#      = 0.75 m/s^3, band 0.5-1.0) computed through the local d(extra)/d(cmd) slope of the curve (cliff band
#      ~1 LSB per ~0.5 s; above 8 ~9 LSB/s). Growth never exceeds the panda's +RATE_STEP / sent-frame ceiling.
#      A HARD ask (<= FCA11_ONSET_BYPASS_ACCEL) is an EMERGENCY: it BYPASSES the onset hold AND the comfort jerk
#      slew - its growth runs at up to the panda ceiling (+RATE_STEP = 4 LSB per SENT frame, the same speed class as
#      the pre-shaping path's fast ramp) so a hard demand reaches the tier cap in a few sent frames, not seconds.
#      The shaped LEVEL target is KEPT (an honest inverse level once reached). The comfort slew is for comfort asks,
#      not emergencies. (0043 v2: v1 bypassed only the hold but still jerk-slewed the growth, which added ~1.7 s to
#      8 LSB / ~3 s to the cap - up to +44 m of open-loop stopping distance. Fixed.)
#   4. steady:      cmd tracks the level inverse; the only rate limit is the jerk slew. Decreases are IMMEDIATE
#      (the safe direction); the release ramp is deliberately NOT in this patch (see the REPORT).
#   5. stop/hold:   v <= FCA11_SHAPE_HOLD_BAND_KPH (or CarState.standstill) ENTERS the hold band; it is left only at
#      >= FCA11_SHAPE_HOLD_EXIT_KPH (EXIT hysteresis, 0043 v2). Inside, the command floors at the PERSONALITY TIER CAP
#      (12/20/30), reached at the panda's +RATE_STEP so a proven hold (which ran at the cap, 8-20 s) is never
#      weakened. This is the last-meter snub. The hysteresis stops a DCT creeping across the 3 km/h edge from
#      stepping the command cap<->level (e.g. 20 -> 6 -> 20) on the boundary.
#   6. release:     unchanged (episode close + passive frame). NO release ramp in this patch (deferred).
#
# 0043 v3 (C0, the owner-feel study wsG-feel, 285 accepted episodes over 9 FCA11 routes): K/BITE refit on 8-20 LSB to
# 9.6/0.74 (the 155+157-only 8.23/0.85 sat outside the pooled CI and over-delivered +0.10 m/s^2 mean at 12-20 LSB; the
# refit's inverse error is +0.02 mean). CLIFF_HI_EXTRA 1.42 -> 1.50 = the measured 8 LSB median 1.499, so the inverse
# stays CONTINUOUS at the 8 LSB boundary (affine(1.50) = round(7.92) = 8) and monotone (pinned by a test). Per-route K
# still spans 8.1-10.6 (day-to-day), so a low-K day under-delivers soft asks slightly; the gate sim covers K = 8.1.
# These constants are read ONLY by the shaping law (shape_dec_cmd_from_extra / shape_local_slope_extra_per_lsb); the
# OFF (deployed 0.67) path never reads them.
# 0043 v3 (C1): the onset hold 0.4 -> 0.2 s. The study measured the 0.4 s floor hold as ~0.4 s of dead time (the 4 LSB
# floor delivers ~0.15 m/s^2) that made the law 0.35 s later than the owner's own onset. The hold has NO frame-acceptance
# role: the panda's onset rule (hyundai.h, 0035) caps the FIRST actuating frame at +RATE_STEP over a 0 reference and each
# later frame at +RATE_STEP over the last ACCEPTED one, with no time term, and Prefill rides every actuating frame from
# the first; hard asks already skip the hold entirely and are accepted. The hold is comfort-only (wsB-feel (b)2: 300-500
# ms was a feel proposal, the 0.2 s choice is the study's C1). The G5 onset DEBOUNCE (FCA11_ONSET_HOLD_FRAMES, 0.2 s
# of ask before an episode opens) is a SEPARATE constant and is unchanged.
# The anchors + cliff slope ARE PROVISIONAL: the 5-7 LSB band was NEVER emitted by the old law (it jumped 4->8 in
# one +4 step), so it is UNMEASURED. Refit FCA11_SHAPE_CLIFF_LO/HI_EXTRA (+ FCA11_SHAPE_K/BITE) after the next
# drive logs the 5/6/7 LSB levels. The whole mapping is behind these few constants so the refit is a constant
# change, not a code change.
FCA11_SHAPE_FLOOR_LSB = 4          # the moving floor; the law never commands below this while moving
FCA11_SHAPE_CLIFF_LO_EXTRA = 0.15  # PROVISIONAL: delivered extra (m/s^2) measured at 4 LSB (0.12-0.17)
FCA11_SHAPE_CLIFF_HI_EXTRA = 1.50  # 0043 v3 (C0): MEASURED 8 LSB median 1.499 [1.42, 1.57], 106 episodes (== BITE + K*0.08 = 1.508)
FCA11_SHAPE_CLIFF_LO_LSB = 4
FCA11_SHAPE_CLIFF_HI_LSB = 8
FCA11_SHAPE_K = 9.6                # 0043 v3 (C0): 8-20 LSB refit over 285 episodes, K 9.58 [8.98, 10.34] (was 8.23, 155+157 only)
FCA11_SHAPE_BITE = 0.74            # 0043 v3 (C0): 8-20 LSB refit over 285 episodes, BITE 0.74 [0.64, 0.81] (was 0.85)
FCA11_SHAPE_ONSET_HOLD_S = 0.2     # 0043 v3 (C1): the onset hold at the floor, seconds of SENT-frame time (was 0.4)
FCA11_SHAPE_ONSET_HOLD_SENT = 10   # == FCA11_SHAPE_ONSET_HOLD_S / 0.02 (the 50 Hz sent-frame period); pinned by a test
# 0043 v3: the study's C2 (jerk 0.75 -> 0.9) is NOT folded - the pre-registered rule R1 fired (REPORT-0043 v3).
FCA11_SHAPE_JERK_TARGET = 0.75     # target delivered jerk (m/s^3); the comfort band is 0.5-1.0
FCA11_SHAPE_JERK_BAND = (0.5, 1.0)
FCA11_SHAPE_HOLD_BAND_KPH = 3.0    # ENTER the stop/hold band at/below this (or CarState.standstill): floor at the tier cap
FCA11_SHAPE_HOLD_EXIT_KPH = 5.0    # 0043 v2: EXIT the band only at/above this (hysteresis) - no cap<->level step chatter at 3 km/h
FCA11_SHAPE_SENT_DT = 0.02         # seconds per SENT (50 Hz) frame: the jerk slew is per SENT frame


def shape_dec_cmd_from_extra(extra: float, cap: int = HYUNDAI_FCA11_LONG_MAX_DEC) -> int:
  """0043: the piecewise-MONOTONE inverse of the measured steady curve (wanted extra m/s^2 -> raw CR_VSM_DecCmd).

  Clamped to [FCA11_SHAPE_FLOOR_LSB, cap]. extra <= 0.15 -> the 4 LSB floor; extra in (0.15, 1.50) -> a LINEAR
  interpolation across the CLIFF (PROVISIONAL - the 5/6/7 LSB band is unmeasured); extra >= 1.50 -> the affine
  inverse round(((extra - BITE)/K)*100), clamped [8, cap] (which keeps the curve continuous at the 8 anchor)."""
  if extra <= FCA11_SHAPE_CLIFF_LO_EXTRA:
    return FCA11_SHAPE_FLOOR_LSB
  if extra < FCA11_SHAPE_CLIFF_HI_EXTRA:
    frac = (extra - FCA11_SHAPE_CLIFF_LO_EXTRA) / (FCA11_SHAPE_CLIFF_HI_EXTRA - FCA11_SHAPE_CLIFF_LO_EXTRA)
    cmd = FCA11_SHAPE_CLIFF_LO_LSB + frac * (FCA11_SHAPE_CLIFF_HI_LSB - FCA11_SHAPE_CLIFF_LO_LSB)
    return int(round(max(FCA11_SHAPE_FLOOR_LSB, min(cap, cmd))))
  cmd = int(round(((extra - FCA11_SHAPE_BITE) / FCA11_SHAPE_K) * 100))
  return max(FCA11_SHAPE_CLIFF_HI_LSB, min(cap, cmd))


def shape_local_slope_extra_per_lsb(lsb: float) -> float:
  """d(extra)/d(LSB) of the shaped curve at a command level (m/s^2 per LSB) - the slope the jerk slew rides.

  Cliff band (4-8 LSB): the linear interpolation's slope. At/above 8: the affine slope K/100 (the inverse's
  d(extra)/d(cmd_g) = K with cmd_g = LSB/100), so the map back to LSB growth is K/100 per LSB."""
  if lsb < FCA11_SHAPE_CLIFF_HI_LSB:
    return (FCA11_SHAPE_CLIFF_HI_EXTRA - FCA11_SHAPE_CLIFF_LO_EXTRA) / (FCA11_SHAPE_CLIFF_HI_LSB - FCA11_SHAPE_CLIFF_LO_LSB)
  return FCA11_SHAPE_K / 100.0


def shape_slew_lsb_per_sent(lsb: float) -> float:
  """The jerk-limited growth allowed in ONE 50 Hz sent frame at command level ``lsb`` (LSB per sent frame).

  dLSB/dt = jerk_target / (d(extra)/dLSB), times the 0.02 s sent-frame period. 0043 v3 (C0 slopes, jerk 0.75):
  cliff band -> ~0.044 LSB/frame (~1 LSB per ~0.45 s); above 8 -> ~0.16 LSB/frame (~7.8 LSB/s)."""
  slope = shape_local_slope_extra_per_lsb(lsb)
  return FCA11_SHAPE_JERK_TARGET / slope * FCA11_SHAPE_SENT_DT


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


# 0044: the FCA11_AFFINE env override and the Params() last-resort fallback are DELETED. The env path was the sim's
# only way to turn the shaping law on, and it HID the defect that the law was unreachable on the car (the key was not
# in params_keys.h and nothing wrote CP_SP.fca11AffineGain). The ONLY path in is now the production one: the
# HyundaiFca11AffineGain param -> the superproject's initialize_params() -> opendbc setup_interfaces() (which applies
# the fca11Brake dependency) -> CP_SP.fca11AffineGain. A SET env var is refused loudly at construction so a stale
# harness can never silently run the law the car would not run.
FCA11_AFFINE_ENV_REMOVED = "FCA11_AFFINE"


def affine_gain_enabled(CP_SP) -> bool:
  """The resolved opt-in state for the shaping law: CP_SP.fca11AffineGain AND CP_SP.fca11Brake. Read ONCE at
  Fca11LongBrake.__init__ (never per frame). setup_interfaces already applies the dependency; it is re-applied here so a
  hand-built CP_SP can never run the law without FCA11 braking. A missing field (older struct) is False."""
  return bool(getattr(CP_SP, "fca11AffineGain", False)) and bool(getattr(CP_SP, "fca11Brake", False))


def _refuse_affine_env() -> None:
  """0044: raise if the removed FCA11_AFFINE env override is SET (any value, even empty) - see the note above."""
  if FCA11_AFFINE_ENV_REMOVED in os.environ:
    val = os.environ[FCA11_AFFINE_ENV_REMOVED]
    msg = f"{FCA11_AFFINE_ENV_REMOVED} is set ({val!r}) but the env override was removed in patch 0044: "
    msg += "the FCA11 brake-shaping law is enabled ONLY through the HyundaiFca11AffineGain param "
    msg += "(Params -> initialize_params -> setup_interfaces -> CP_SP.fca11AffineGain, requires HyundaiFca11Brake). Unset the variable."
    raise RuntimeError(msg)


# *** 0054: FCA11_MEAS_* — the MEASURED ask -> LSB inverse, and the RELEASE BELOW THE FLOOR rule ***
#
# WHAT IT REPLACES (quoted, so the next reader sees the exact thing rather than a paraphrase):
#   fca11_long.py:843-850   def dec_cmd_from_accel(accel):
#                             if accel > FCA11_BRAKE_ONSET_ACCEL: return 0
#                             dec_g = (-accel / FCA11_BRAKE_GAIN) / 9.81
#                             return int(round(dec_g * 100))
#   fca11_long.py:255       FCA11_BRAKE_GAIN = 0.67        <- through-the-origin gain (delivered = 0.67 * commanded)
#   fca11_long.py:75        FCA11_BRAKE_ONSET_ACCEL = -0.5 <- with `min(accel, ONSET)` at fca11_long.py:2198 and :1669
#                             this IS the 8 LSB floor: dec_cmd_from_accel(-0.5) = round(7.60) = 8
#   fca11_long.py:853-855   clamp_dec_cmd()               <- the per-frame cap: max(1, min(tier cap, dec_cmd))
#   fca11_long.py:824-828   PERSONALITY_MAX_DEC           <- the ceiling: 12 / 20 / 30 (30 == HYUNDAI_FCA11_LONG_MAX_DEC)
# So the deployed request was lsb = clamp(round(100 * (-accel / 0.67 / 9.81)), 8, 30): a through-origin inverse whose floor
# was fixed by the onset clamp. Two errors in one, and they conspire: (i) the plant is AFFINE, so the slope is ~1.5x off
# and every mid-band ask is commanded nearly twice as hard as asked; (ii) the 8 LSB floor turns ANY ask softer than
# ~1.25 m/s^2 into ~1.25 m/s^2 of delivered decel -- including the planner's own roll-out asks (0.0 ... 0.6), which the
# PLAN believes are coasting. Route 16a: 3 of 4 stops came to rest 2.07 / 2.69 / 2.06 m beyond the planner's own 6.0 m
# standstill target, and the last 2 s before stop t=152 asked 0.0 / 0.0 / 0.62 m/s^2 while the law held 8-9 LSB
# (1.19-1.29 m/s^2 delivered). The car was not failing to execute the plan; it was braking when the plan said coast.
#
# THE MEASUREMENT (91 routes; sustained FCA11 frames >= 0.5 s into an episode, driver not braking, v >= 12 km/h, no lead
# within 60 m; ~/comma-sim/wsSTOP/drive-169-16c/sub/esc-floor/frames.csv): delivered decel BEYOND COAST by level
#   4 LSB -> 0.78 m/s^2 (p10 0.65)   5 -> 0.84 (0.71)   6 -> 0.91   8 -> 1.25   10 -> 1.44   20 -> 2.28   30 -> 3.64
# i.e. affine: extra = BITE + K * cmd_g. Refitted on tonight's per-level medians over 4-20 LSB (the band the law uses):
# K = 8.94 (CI95 [8.52, 9.36]) m/s^2/g, BITE = 0.49 (CI95 [0.42, 0.55]) m/s^2, max residual 0.11 m/s^2. This block keeps
# the 0041/0043 POOLED fit (17260 production frames, 126 events) -- BITE 0.40 [0.33, 0.48], K 9.9 [9.43, 10.38] -- which
# sits inside/next to that CI and is the pair the 0043 shaping law already inverts, so the two laws can never disagree
# about the ESC. The two fits differ by <= 0.11 m/s^2 anywhere in the band (<= 0.6 LSB of inverse error), i.e. below the
# command resolution. Do NOT re-tune one without the other.
#
# THE LAW (moving, above the stop/hold band):
#   want_extra = -accel                                          (m/s^2 beyond coast the planner wants)
#   lsb        = round(100 * (want_extra - BITE) / K)            clamped [FCA11_MEAS_FLOOR_LSB, FCA11_MEAS_CEIL_LSB]
# An ask softer than FCA11_MEAS_FLOOR_EXTRA is commanded at the FLOOR (4 LSB), never released: see the REJECTED
# release variant below. FCA11_MEAS_FLOOR_EXTRA is what 4 LSB actually delivers, so the law's output is either 0
# (an ask above the onset, i.e. coast territory) or an honest level in [0.80, 3.64] m/s^2.
#
# THE REJECTED RELEASE VARIANT (measured, do NOT re-add). An earlier revision RELEASED any moving ask below
# FCA11_MEAS_FLOOR_EXTRA (closed the episode passively) on the theory that flooring it up is "more braking than asked".
# Replay of the four route-16a stops says otherwise: the last 2 s of ALL FOUR asked 0.00-0.66 m/s^2 (below the floor) while
# the car was demonstrably delivering 1.19-1.71 m/s^2 -- so a release there sends NOTHING exactly where the car needs
# brake, and every one of those stops becomes a coast that never reaches standstill, i.e. a BRAKE LOSS. It also broke the
# cycle-1 crawl-dwell escapes, which open on asks <= -0.5 (below the floor) for the creep-into-a-closing-lead case. The
# FLOOR change alone recovers the error: replay give-back against observed is +1.91/+1.81/-0.48/+5.55 m vs observed
# +2.07/+2.69/-0.01/+2.06 m on the four stops (sub/rollout-replay/replay_16a_flooronly.py), i.e. 1.9 of the 2.07 m on the
# first stop, two stops inside +-0.5 m of the 6.0 m target and three inside +-0.9 m (the 4th stop's residual -3.49 m is the
# plant model's own domain error: the log shows 1.71 m/s^2 delivered at 3 m/s, i.e. ~2x the fit's 4 LSB value, and the fit
# was made at >= 12 km/h with no lead). The release arm added nothing but the brake loss.
#
# UNTOUCHED (each a deliberate boundary, not an oversight):
#   * the ONSET rule (:75, :89, :2139-2160) and the +RATE_STEP rate step (:2265) -- this block chooses a LEVEL, never a
#     timing, and the panda's +RATE_STEP onset gate still governs every ramp;
#   * the 30 LSB cap and the personality tiers (:824-828, :853-855);
#   * the driver cut / gas hold-off / camera hand-back / refusal wall (:2009-2062, :2274);
#   * the STOP/HOLD band (:1667-1674): at/below FCA11_FEEL_HOLD_BAND_KPH the DEPLOYED request still runs, because a
#     standstill hold on a DCT car that creeps must not be weakened and the measured curve is LOWER there (a -2.0 ask is
#     16 LSB, not the 20 the deployed request gives). The 2 m this patch recovers is spent between 4 and 12 km/h, ABOVE
#     the band edge, so excluding the band loses none of the fix. 0055: the band's FLOOR is now fca11_floor_lsb(v), which
#     is 8 at every band speed; the request above the floor is untouched. See the 0055 block below;
#   * the 0043 shaping law (shape_dec_cmd_from_extra) and cal -- both take their own paths (:2212, :2250).
# Kill switch: FCA11_MEAS_INVERSE = False restores the pre-0054 request map byte-for-byte (read once per
# instance; it also needs the feel layer, which is where the deployed request lives -- see self.meas_inverse).
FCA11_MEAS_INVERSE = True
FCA11_MEAS_BITE = 0.40       # m/s^2 delivered beyond coast at 0 LSB (the plant's own bite); CI95 [0.33, 0.48]
FCA11_MEAS_K = 9.9           # m/s^2 per g of command (per 100 LSB);                     CI95 [9.43, 10.38]
FCA11_MEAS_FLOOR_LSB = 4     # the weakest actuating level the ESC accepts (was 8, set by the onset clamp at :2198)
FCA11_MEAS_CEIL_LSB = HYUNDAI_FCA11_LONG_MAX_DEC   # 30 == the panda gate; the personality tier cap is applied on top
# What the floor actually delivers beyond coast: 0.40 + 9.9 * 0.04 = 0.796 m/s^2 (measured 0.78, p10 0.65). An ask softer
# than this cannot be served by ANY actuating frame; it is commanded at the FLOOR anyway (see the rejected release
# variant above: releasing such an ask is a brake loss at exactly the asks where the car needs brake).
FCA11_MEAS_FLOOR_EXTRA = FCA11_MEAS_BITE + FCA11_MEAS_K * FCA11_MEAS_FLOOR_LSB / 100.0


# *** 0055: fca11_floor_lsb — the SPEED-INDEXED band floor (4 LSB moving -> 8 LSB in the stop band) ***
#
# WHY THERE IS A RAMP AT ALL (Fable's ruling on the 0054 stop-band step; intent quoted): 0054 dropped the MOVING floor
# from 8 to 4 LSB but left the STOP band at 8 - the deployed request floors there through the -0.5 onset clamp. That
# reproduced exactly the jerk 0054 exists to remove, just later and in one frame:
#   (a) holding 4 LSB *through* the stop band is NOT supportable: there is no stop-hold evidence below ~12 km/h, and
#       0.31 m/s^2 (what a 4 LSB-class command can leave you with at creeping speed) from 2 m/s is up to ~6 m of
#       roll-out into a lead. The planner's own standstill plan for that approach is 1.9-2.5 m;
#   (b) letting the level step 4 -> 8 in ONE frame at band entry is a SECOND jerk event of the same magnitude as the
#       one we are fixing (+0.4 m/s^2 in 20 ms, at the end of every stop).
# So the floor is a CONTINUOUS function of v instead of a piecewise step: 4 LSB at v >= 2.0 m/s, rising LINEARLY to 8 LSB
# at v <= 1.0 m/s, and constant 8 below 1.0 m/s (i.e. through the whole stop band and at standstill).
#
# SPEED-INDEXED, NOT TIME-INDEXED, on purpose: the level is a pure function of the CURRENT v, so it is deterministic,
# reproducible frame-for-frame in a pin test, has no state to arm/reset/migrate, and cannot drift with a different
# frame cadence or a pause in the frame stream. There is NO time term anywhere in this function.
#
# THE MEASURED TABLE IT INTERPOLATES (85-91 routes; sustained FCA11 frames >= 0.5 s into an episode, driver not braking,
# v >= 12 km/h, no lead within 60 m; delivered decel BEYOND COAST in m/s^2 - full table in the FCA11_MEAS_* block above):
#   4 LSB -> 0.78 (median, p10 0.65)   6 -> 0.91   8 -> 1.25   10 -> 1.44   20 -> 2.28   30 -> 3.64
# The ramp's endpoints are two of those medians - the weakest actuating frame the ESC accepts (4) and the deployed
# stop-hold floor (8) - so every interpolated value between them is a level the plant has been measured at, and the
# ramp never asks for a level below the floor (which is the rejected release shape) or above the stop hold.
#
# JERK AND ROLL-OUT (the arithmetic behind the ruling): at the 1 m/s^2 decel a stop is planned with, 2.0 -> 1.0 m/s takes
# 1.0 s and the floor climbs 4 LSB, i.e. ~0.40 m/s^3 of jerk (0.39 m/s^2 of extra decel spread over 1 s) - well under the
# owner's 1.4 m/s^3 limit, and it is the same 0.4 m/s^2 delivered over 50 sent frames instead of 1. DISTANCE: integrating
# v/a(v) over v = 2.0 -> 0 with a(v) = 0.40 + 9.9 * floor(v)/100 (0.80 m/s^2 at 2.0 m/s rising to 1.19 by 1.0 m/s) gives
# 1.99 m -- inside the planner's own 1.9-2.5 m standstill plan. Holding 4 LSB all the way instead would need 2.51 m from
# the same 2 m/s (4 / 2*0.80), and the ~6 m the ruling quotes is that same arithmetic at the ~0.31 m/s^2 the plant can
# leave you with at creep (4 / 2*0.31 = 6.4 m) -- a stop that undershoots its plan by metres into a lead. A constant 8
# from 2 m/s would stop in 1.68 m, i.e. it overshoots the plan's SHORT end, which is why 8 belongs to the last metre
# (the hold) and not to the whole approach.
#
# APPLIED EVERYWHERE THE MEASURED FLOOR IS APPLIED: dec_cmd_from_accel_measured, the feel target (_feel_target) and the
# stop-band path that used to hard-code 8 (it now takes this floor explicitly instead of inheriting 8 from the -0.5
# onset clamp; at <= 1.0 m/s, i.e. at every band speed, it IS 8, so the band stays byte-identical to 0054 there).
# The stop HOLD itself is untouched: a deployed request ABOVE the floor still commands what it commanded. ``None`` means
# "at speed" (>= 2.0 m/s) -> 4, so a caller with no speed gets the moving floor, which is what every pre-0055 pin means.
FCA11_FLOOR_RAMP_TOP_MPS = 2.0   # at/above: FCA11_MEAS_FLOOR_LSB (4), the moving floor
FCA11_FLOOR_RAMP_BOT_MPS = 1.0   # at/below: FCA11_NORESPONSE_MIN_LSB (8), the stop-band hold floor


def fca11_floor_lsb(v_ego_mps=None) -> int:
  """0055: the SPEED-INDEXED floor (LSB) of the measured law at ego speed ``v_ego_mps`` (m/s).

  v >= 2.0 -> 4 LSB; v <= 1.0 -> 8 LSB; in between, linear 4 -> 8 and rounded to the nearest LSB; None -> 4 (at speed).
  A pure function of v: no state, no time term, same v always gives the same LSB. See the 0055 block above for the
  ruling it implements, the measured table it interpolates and the jerk/roll-out arithmetic.
  """
  if v_ego_mps is None:
    return FCA11_MEAS_FLOOR_LSB
  if v_ego_mps >= FCA11_FLOOR_RAMP_TOP_MPS:
    return FCA11_MEAS_FLOOR_LSB
  if v_ego_mps <= FCA11_FLOOR_RAMP_BOT_MPS:
    return FCA11_NORESPONSE_MIN_LSB
  moving, stop = float(FCA11_MEAS_FLOOR_LSB), float(FCA11_NORESPONSE_MIN_LSB)
  frac = (v_ego_mps - FCA11_FLOOR_RAMP_BOT_MPS) / (FCA11_FLOOR_RAMP_TOP_MPS - FCA11_FLOOR_RAMP_BOT_MPS)
  return int(round(stop + (moving - stop) * frac))


def dec_cmd_from_accel_measured(accel: float, v_ego_mps=None) -> int:
  """0054: the MEASURED ask -> raw CR_VSM_DecCmd inverse (0.01 g/LSB), for a MOVING episode above the stop band.

  want_extra = -accel (m/s^2 beyond coast); lsb = round(100 * (want_extra - BITE) / K), clamped to
  [fca11_floor_lsb(v_ego_mps), FCA11_MEAS_CEIL_LSB]. Above the onset threshold the ask is coast territory, not a brake
  request -> 0. An ask softer than what the floor delivers returns the FLOOR: that is the weakest level that can be
  sent, and this law SENDS it rather than releasing (the release variant was measured and rejected -- brake loss).
  0055: the floor is speed-indexed (4 LSB at/above 2.0 m/s, ramping to 8 LSB by 1.0 m/s); ``v_ego_mps`` None = at speed.
  Monotone non-decreasing in the ask, monotone non-decreasing in 1/v (slower -> never a weaker floor), and never above
  the deployed 0.67 inverse for the same ask.
  """
  if accel > FCA11_BRAKE_ONSET_ACCEL:
    return 0
  want_extra = -accel
  lsb = int(round((want_extra - FCA11_MEAS_BITE) / FCA11_MEAS_K * 100.0))
  return max(fca11_floor_lsb(v_ego_mps), min(FCA11_MEAS_CEIL_LSB, lsb))


# *** 0049: FCA11_FEEL_* — brake FEEL on the DEPLOYED (OLD-LAW) path: coast-first open, lag-aware onset ramp through the
# intermediate LSBs, and an anticipatory release (DEFAULT ON; the shaping/V3 path and cal are untouched) ***
#
# WHY (drive-feedback-159, routes 158/159/15a, 46.5k shadow frames; plant ID in plant-notes.md): the deployed law only
# ever emitted {0, 4, 8..20} LSB. That is NOT a wire limit - CR_VSM_DecCmd is an 8-bit integer at 0.01 g and the panda
# accepts ANY level 1..30 within +RATE_STEP over the last accepted frame. It is the law: (a) the -0.5 onset maps through
# the 0.67 inverse to 7.6 -> round = 8, so 8 is the mapping's floor; (b) inside the release band (-0.50, -0.30] and the
# min-on window the ask is clamped to the onset value (`min(accel, ONSET)`), so the command is floored at 8; (c) the
# +RATE_STEP limiter takes 0 -> 4 -> 8 in two sent frames (40 ms). The plant over-delivers exactly there (L=8: ~1.20 m/s^2
# incremental for a ~0.5 ask, 1.5-2x nominal), lags ~0.28 s and has a ~0.53 s release tail, so a small ask became a
# grabby 1.2 m/s^2 hit and the owner cancelled 85/89 gas-ended episodes with the throttle.
#
# WHAT 0049 DOES (OLD-LAW instance only: not the V3/shaping law, not cal, not the low-speed hold band):
#   1. COAST-FIRST OPEN: a NEW episode (and the 0.2 s onset debounce) needs the RESIDUAL demand beyond zero-throttle coast,
#      -ask - FCA11_FEEL_COAST_DECEL, to reach FCA11_FEEL_OPEN_MARGIN (defaults 0.15 + 0.45 -> the ask must reach -0.60, was
#      -0.50). Hysteresis: once open, the deployed release rule (-0.30 + 0.6 s min-on) is unchanged. A HARD ask
#      (<= FCA11_ONSET_BYPASS_ACCEL) still opens on its first frame, and an ask SUSTAINED at/below the deployed -0.5 for
#      FCA11_FEEL_DWELL_FRAMES (1.0 s) opens anyway (dwell escape: suppress transients, never a sustained demand).
#   2. ONSET RAMP (lag-aware): the first sent frame is FCA11_FEEL_FIRST_LSB (4, the panda's onset step); growth is then
#      jerk-limited per SENT frame: FCA11_FEEL_JERK_TARGET / (delivered m/s^2 per LSB at the current level) x 0.02 s, i.e.
#      ~0.19 LSB/frame across the 4-8 band (PROVISIONAL steep low-band slope) and ~0.51 LSB/frame above 8, for asks at/above
#      FCA11_FEEL_RATE_SOFT_ACCEL, blending linearly to the panda's +RATE_STEP at FCA11_ONSET_BYPASS_ACCEL (a HARD ask is
#      never slowed; nor is an ask DEEPENING faster than FCA11_FEEL_DEEPEN_SLOPE). DISCLOSED TRADE: a soft ramp that then
#      turns hard climbs at +RATE_STEP from a lower level, so it can reach the cap up to ~3 sent frames (60 ms) after the
#      deployed law; a jump to the old trajectory would violate the panda's +RATE_STEP rule, so this is inherent. Over the 0.28 s plant lag the planner sees no response and deepens its ask; the old law followed it
#      instantly and the delayed plant then over-shot. The ramp walks through 5/6/7 (never emitted before).
#   3. ANTICIPATORY RELEASE: while the ask is EASING, the target is computed from the ask predicted FCA11_FEEL_ANTICIPATE_S
#      (the plant lag) ahead (slope = an EMA over sent frames, lead capped at FCA11_FEEL_ANTICIPATE_MAX); and inside the
#      release band / min-on window the command FOLLOWS the ask down through the intermediate LSBs (the same 0.67 inverse,
#      floored at FCA11_FEEL_FLOOR_LSB = 4) instead of being clamped at 8. Episode exit timing is UNCHANGED (the passive
#      close frame goes out on the same frame as before); only the level ahead of it is lower.
#   4. HOLD BAND: at/below FCA11_FEEL_HOLD_BAND_KPH (or CarState.standstill), until >= FCA11_FEEL_HOLD_EXIT_KPH, the
#      deployed law runs EXACTLY (open at -0.5, +RATE_STEP ramp, 8 LSB floor): a standstill hold is never weakened.
# AUTHORITY INVARIANT (pinned by tests + replay): the requested level is never above the deployed law's level for the same
# ask (the target map is the same 0.67 inverse at/below -0.5, the anticipated ask is only ever LESS negative, the ramp only
# slows growth), a HARD ask from idle produces the deployed law's frames byte-for-byte, and the steady level for any ask
# <= -0.5 is unchanged. Every panda contract (Warn 3, +RATE_STEP over the last SENT frame, passive close on every exit,
# R7 latch, camera hand-back, no auto-resume, arming) is untouched - this block only chooses a level <= the old one.
# 0054 SUPERSEDES the second half of that invariant ON THE MEASURED LAW: the level map is now the measured affine inverse
# (dec_cmd_from_accel_measured, floor 4 LSB), so the steady level for a given ask is LOWER than the deployed one and a
# sustained -0.55 still actuates (at the 4 LSB floor, as the dwell escape requires). The "never above the deployed level"
# half still holds (the measured map is <= the 0.67 map for every ask). The pre-0054 behaviour - and every test that pins
# it - is restored byte-for-byte by FCA11_MEAS_INVERSE = False.
# Set FCA11_FEEL_ENABLED False to restore the pre-0049 OLD-LAW byte-for-byte (read once per instance).
FCA11_FEEL_ENABLED = True
FCA11_FEEL_COAST_DECEL = 0.15        # m/s^2 zero-throttle coast near the onset asks (measured 0.13-0.19 by speed band)
FCA11_FEEL_OPEN_MARGIN = 0.45        # m/s^2 residual (beyond coast) a NEW episode needs -> open at ask <= -0.60
FCA11_FEEL_OPEN_ACCEL = -(FCA11_FEEL_COAST_DECEL + FCA11_FEEL_OPEN_MARGIN)
FCA11_FEEL_FIRST_LSB = 4             # first sent frame of a soft episode (== the panda's onset RATE_STEP)
FCA11_FEEL_FLOOR_LSB = 4             # moving floor while an episode is open (never 0 mid-episode; 0 is the release)
FCA11_FEEL_JERK_TARGET = 2.5         # m/s^3 delivered onset jerk the soft ramp aims at (was ~8 raw p50 measured)
# Delivered decel per LSB the ramp divides by (m/s^2 per LSB). Above 8 LSB = the MEASURED marginal slope (0.098, plant-notes
# 3: L 8->19 rises 1.20 -> 2.25). Below 8 = PROVISIONAL: 1-3 / 5-7 were never commanded and 4 only as a 20 ms pulse, so the
# 0043 cliff interpolation (0.15 m/s^2 at 4 -> 1.20 at 8 = 0.26 / LSB) is assumed - the CONSERVATIVE choice (if the low band
# is in fact linear, the ramp there is just gentler than needed). Refit once a drive logs the 5/6/7 levels.
FCA11_FEEL_SLOPE_LOW = 0.26          # m/s^2 per LSB below FCA11_FEEL_SLOPE_KNEE_LSB (PROVISIONAL)
FCA11_FEEL_SLOPE_HIGH = 0.098        # m/s^2 per LSB at/above the knee (measured)
FCA11_FEEL_SLOPE_KNEE_LSB = 8
FCA11_FEEL_RATE_SOFT_ACCEL = -1.0    # asks at/above this ramp at the jerk target; linear to +RATE_STEP at the HARD bypass
FCA11_FEEL_ANTICIPATE_S = 0.28       # measured total command -> aEgo lag: look this far ahead while the ask eases
FCA11_FEEL_ANTICIPATE_MAX = 0.25     # m/s^2 cap on the anticipated easing (noise / outlier bound)
FCA11_FEEL_SLOPE_ALPHA = 0.2         # EMA weight per SENT frame for the ask slope (~0.1 s time constant)
FCA11_FEEL_HOLD_BAND_KPH = 3.0       # at/below (or standstill): the deployed law runs unchanged (stop/hold)
# Coast-first DWELL ESCAPE (closed-loop guard): an ask held at/below the deployed onset (-0.5) for this long opens anyway
# (and then ramps softly). Coast-first must suppress transient small asks, never leave a SUSTAINED one unactuated while
# a gap closes (the planner would only re-ask later and deeper). 100 frames = 1.0 s.
FCA11_FEEL_DWELL_FRAMES = 100
# cycle 1 (route 167 seg 9, 15:50:36-39 CDT): a SLOW-CRAWL dwell escape. At a crawl toward a closing lead the planner's
# ask hovers at -0.50..-0.62; the 1.0 s dwell then never completes (74 frames at <= -0.5, one shallow lift to -0.46, 68
# more) and the car crept 1.13 -> 1.49 m/s for 2.65 s with no brake until the driver stepped in. Below
# FCA11_FEEL_DWELL_SLOW_MPS a SECOND consecutive counter - same qualifying frames (ok AND ask <= -0.5) AND speed below the
# limit, hard-reset otherwise - opens after FCA11_FEEL_DWELL_SLOW_FRAMES (0.7 s). It is strict (every counted frame was
# below the limit), so a car slowing through the limit can never open off frames it accumulated at speed; at/above it the
# 1.0 s dwell runs unchanged. Open-loop replay of routes 167/168 (wsSTOP/feel-167-168/underbrake-fix): seg 9 opens at
# 15:50:37.99 (1.35 s before the driver's brake); 2 other openings move, both at 1.39 m/s in one shared 167-s25 window;
# no changed opening above 1.4 m/s; +1.52 s brake-on over both drives. A dwell of 70 at ALL speeds moves 38 openings, 36
# of them above 2 m/s (up to 26 m/s) - the early-braking-at-speed complaint - and dropout-tolerant counters at 100 cannot
# open before ~15:50:38.2 on this ask (the lift is a 53-frame shallow dip, not a dropout). Release/gain/min-on untouched.
FCA11_FEEL_DWELL_SLOW_MPS = 2.0
FCA11_FEEL_DWELL_SLOW_FRAMES = 70
# DEEPENING bypass: while the ask is getting more negative faster than this (m/s^3, EMA over sent frames), the onset ramp
# grows at the panda's full +RATE_STEP - a demand that is building quickly is not throttled by the comfort ramp.
FCA11_FEEL_DEEPEN_SLOPE = -2.0
FCA11_FEEL_HOLD_EXIT_KPH = 5.0       # leave the hold band only at/above this (hysteresis, as 0043 v2)

# *** 0051: FEEL fix round (drive-feedback 15b-15f, 130 min on 0049; evidence: wsSTOP/feel-fix/REPORT.md) ***
# (a) WOBBLE. 0049 lifted the wire-level dip-and-recover count 24 -> 109 (15b/c/e/f). Root cause, isolated by replay: the
#     ANTICIPATION term. Its slope EMA (alpha 0.2/sent frame, ~0.1 s) tracks the planner's +-0.05 m/s^2 ask ripple, so the
#     anticipated ask flickers by up to the 0.25 cap and the follow-down target flips 1-3 LSB frame to frame (anticipation
#     off -> exactly 24 again, but every release is ~1.9 LSB later). Fix = the smallest pair that restores 24 without giving
#     the lead back: (1) ANTICIPATION reads a slower slope EMA, alpha FCA11_FEEL_SLOPE_ALPHA_0051 (0.10, ~0.2 s: ripple
#     averages out, a real easing still registers within the 0.28 s plant lag); the 0049 alpha-0.2 EMA is kept, unchanged,
#     for the DEEPENING bypass (a fast-building demand is detected exactly as fast as before); (2) a RE-RISE DEADBAND: once
#     the level has come DOWN from the episode peak, a GENTLE ask (above FCA11_FEEL_RATE_SOFT_ACCEL) must call for
#     >= FCA11_FEEL_RERISE_LSB above the current level before it climbs again. Onsets (level == peak) and harder asks
#     are untouched; the deadband only ever HOLDS a level, bounded by RERISE_LSB below the 0049 target (2 LSB ~ 0.13
#     m/s^2 nominal), and releases are never delayed by it.
FCA11_FEEL_SLOPE_ALPHA_0051 = 0.10
FCA11_FEEL_RERISE_LSB = 2.0
# (b) GENTLE GAIN. Delivered/ask on the 15x drives (ask lagged 0.28 s, >15 km/h, no pedals): 2.15x (-0.7,-0.5], 2.03x
#     (-1.0,-0.7], 1.86x (-1.5,-1.0], 1.39x (-2.1,-1.5]. Steady per-level delivered (constant level >= 0.8 s, no higher level
#     in the prior 1.5 s) fits brake-only = 0.57 + 8.06 * L/100 m/s^2 on L 6..20 (Oct-9 cal affine: 0.40 + 9.9 * L/100), i.e.
#     a ~1.0 m/s^2 BITE at L 4-7: below ~1.0 m/s^2 no nonzero level can deliver the ask, so a FULL inverse is impossible
#     there and the low-band per-level data is thin (L4 n=1, L5 n=4 clean samples). So the correction is PARTIAL and a single
#     gain, not a per-level table: the feel target is scaled by FCA11_FEEL_GENTLE_GAIN = 0.70 = sqrt(1/2.03), i.e. HALF the
#     measured over-delivery of the (-1.0,-0.7] band in log terms, for asks at/above FCA11_FEEL_GAIN_FADE_ACCEL, fading
#     linearly to 1.0 at FCA11_ONSET_BYPASS_ACCEL (a HARD ask keeps the deployed level byte-for-byte). With the measured
#     p25 per-level delivered, every corrected gentle level still delivers MORE than its ask (never under-brakes by the data).
FCA11_FEEL_GENTLE_GAIN = 0.70
FCA11_FEEL_GAIN_FADE_ACCEL = -0.8
# (b2) LOW-SPEED RESTORE (independent-review must-fix 1). The stop/hold band (<= FCA11_FEEL_HOLD_BAND_KPH, or standstill)
#     runs the DEPLOYED law, NOT the feel law (the band branch of _feel_level). With the gentle gain in force the level sits
#     ~2-3 LSB lower just before the band, so band entry re-stepped it up to the deployed request: open-loop replay of the
#     15x drives gives a +4 LSB step on 18 of the 25 scored stops, in the last ~0.6 s before standstill (0049: 0/25; the
#     step is caused by (b) alone - with the gain at 1.0 it is 0/25 - and it is exactly the "last metre too firm / jerky"
#     complaint this round exists to fix). So the gain is RESTORED to 1.0 as the car slows: at band entry the feel level IS
#     the deployed request for the same ask and the band cannot step it. The strength benefit stays where it was measured
#     (at speed: the delivered/ask table is computed on frames > 15 km/h).
#     THRESHOLDS FROM THE DRIVES (provenance): at v <= 3 km/h the band's deployed request is what the car runs to standstill
#     within ~0.2-0.4 s of band entry (the scored-stop decel is 0.8-1.1 m/s^2 at 3 km/h -> 0.83 m/s to standstill). The
#     feel ramp grows at <= 0.19 LSB/sent = 9.6 LSB/s below the L8 knee, so restoring a 3 LSB gap needs ~0.3 s. Starting
#     at 8 km/h leaves 0.5-0.9 s (2-3x the ramp time) at those decelerations and completing at 5 km/h puts the level at the
#     deployed request ~0.2-0.4 s before the band takes over - no step, and no need to rush the ramp.
#     NO LATCH, deliberately: the two thresholds are placed in the stop band's own two-threshold idiom (FULL 5.0 just
#     under the band's EXIT 5.0, FADE 8.0 above its ENTER 3.0), but the fade itself is CONTINUOUS in v, so there is no
#     state to chatter about - a latch over this curve is provably inert (the curve is already 1.0 at FADE_KPH, so a
#     latched and an unlatched call return the same value above it), and latching the DEEP end instead would re-introduce
#     a 3 LSB target step when the latch clears on a re-acceleration. Measured in the zone: <= 1 LSB of target ripple.
FCA11_FEEL_GAIN_FADE_KPH = 8.0        # at/below: the gentle gain starts being restored (stop-band approach)
FCA11_FEEL_GAIN_FULL_KPH = 5.0        # at/below: the gain IS 1.0 -> the band entry level is the deployed request (0049)
# (c) LEAD-LOSS HOLD. 15c seg 45 (47 km/h): radarState.leadOne dropped out for 0.53 s mid-brake, the planner's ask eased
#     -0.93 -> -0.49 on the same frame, anticipation + follow-down took the level 13 -> 4, the lead came back and the level
#     rebuilt to 19 (release-and-reapply). A track flicker is not evidence the need ended (34 in-brake drop-outs on 15b-f,
#     median 0.62 s), and the planner's ask stays eased for ~1 s after the track returns. On a lead present -> absent EDGE
#     inside an open episode (feel path, moving), the last SENT level becomes a FLOOR: held while the lead stays absent (at
#     most FCA11_FEEL_LEAD_LOSS_MAX_ABSENT_S) and for FCA11_FEEL_LEAD_LOSS_HOLD_S after that, then tapered linearly to 0 over
#     FCA11_FEEL_LEAD_LOSS_TAPER_S (no step at the end). A harder ask still climbs normally. BOUNDS: the floor is <= the level
#     already on the wire AND <= the PRE-0049 deployed law's request for the current ask (so the hold never commands more
#     than the law the car ran before 0049 would for that ask); every `ok` veto (driver brake/gas, gear, camera, pedal,
#     stale) and the episode exit rule (-0.30 + min-on) are UNCHANGED; anticipation is suspended while it is active.
FCA11_FEEL_LEAD_LOSS_MAX_ABSENT_S = 1.0   # the hold follows a drop-out up to this long (median in-brake drop-out 0.62 s)
FCA11_FEEL_LEAD_LOSS_HOLD_S = 0.6    # then holds this long (the planner re-acquiring the lead)
FCA11_FEEL_LEAD_LOSS_TAPER_S = 1.0   # then the floor ramps to 0 (~1 LSB per 0.1 s at L 10: a taper, not a release)
# Kill switch for (a)+(b)+(c): False restores the 0049/0050 feel layer byte-for-byte (read once per instance).
FCA11_FEEL_FIX_0051 = True

# *** 17c: STANDSTILL HOLD above the personality tier cap (route 17c, owner note "need higher hold to prevent creep") ***
# WHAT THE CAR DID (route 17c, standard tier, wsSTOP/drive-17c/REPORT.md Note 1): every FCA11-held stop ran the FEEL hold
# band below, where LongControl's stopping ramp drives the ask to -2.0 (FCA11_A_MAX_MSS2), the deployed request maps that to
# dec_cmd_from_accel(-2.0) = 30 and the TIER CLAMP cuts it to the personality cap - 20 LSB for standard - held CONSTANT for
# 18-26 s. The DCT car crept anyway: stop 2 (mono 16662.8-16665.0) went 0 -> 0.165 m/s at 20 LSB (net +0.14 m/s^2, one +0.46
# sample), CarState.standstill dropping False for ~0.8 s, while CYL_PRES read 0.0 (the injection is downstream of the
# master-cylinder sensor). FCA11_HOLD_LOST_KPH (2 km/h) is far above that creep, so brakeNow never fired.
# THE CHANGE: once the car has been at TRUE standstill (CarState.standstill: every wheel <= 0.375 km/h) inside the hold band
# of an open episode, the band's CEILING becomes max(tier cap, FCA11_STANDSTILL_HOLD_LSB) instead of the tier cap. Nothing
# else in the band changes: the request map, the 8 LSB band floor and the +RATE_STEP-over-the-last-SENT-frame limiter are the
# same lines, so the extra LSB arrive by the planner's own stopping ramp (ask -0.85 -> -2.0 over ~1.1 s after standstill on
# all 17c stops -> 13 -> 30 at ~15 LSB/s, the last 10 LSB of it after the car is already stationary) and never faster than
# +RATE_STEP per sent frame. It is a ceiling, not a floor: a weaker ask at standstill commands exactly what it commanded.
# WHY A LATCH (set on standstill, cleared only by the band's unchanged EXIT >= FCA11_FEEL_HOLD_EXIT_KPH, an episode (re)open,
# longActive off or a disable): a per-frame standstill gate would drop 30 -> 20 exactly while the car creeps (the 17c
# specimen flipped standstill False for 0.8 s mid-creep), i.e. it would weaken the hold at the one moment it is needed. A
# whole-band gate (<= 3 km/h) would add up to ~1 m/s^2 while still ROLLING into the stop - the final-metre grab the owner's
# "smoothness first" directive forbids. The episode-open clear means a re-brake while rolling in the band after a launch
# starts at the tier cap again (the latch never carries from one stop into a moving episode).
# THE NUMBER (30) and its arithmetic: the fitted moving plant (FCA11_SHAPE_K 9.6 m/s^2 per 100 LSB, BITE 0.74) gives
# +10 LSB (20 -> 30) ~= +0.96 m/s^2 of extra delivered decel, against a measured creep of 0.14 m/s^2 net (0.46 peak): ~7x the
# net and ~2x the peak. 30 == HYUNDAI_FCA11_LONG_MAX_DEC (the panda gate, CR_VSM_DecCmd > 30 refused) == the aggressive tier
# cap == the highest level ever exercised on this car (0.30 g rolling scaling pulses, 2.03 m/s^2 delivered). It is NOT yet
# proven at standstill: no production episode on disk exceeded 20 (891 episodes, 7 pulls), and the LSB -> bar / hold-force
# calibration the drive report asks for has NOT been done; the moving-plant K is an extrapolation at v = 0. Revisit this one
# constant when that calibration exists. VALID RANGE [PERSONALITY_MAX_DEC[STANDARD] (20), HYUNDAI_FCA11_LONG_MAX_DEC (30)]:
# below 20 it is a no-op for standard (the tier the owner drives), above 30 the panda refuses the frame (pinned by tests,
# and clamped again at use by fca11_standstill_hold_lsb()).
# PERSONALITY POLICY: ONE level for every tier (relaxed 12 -> 30, standard 20 -> 30, aggressive 30 unchanged). A standstill
# hold is stop-KEEPING, not a feel dial: the car is stationary, so the level is not felt as decel, and a tier that holds less
# creeps more (relaxed's 12 is the weakest hold of all). Same ruling as the pedal hold feedforward (G3, FORK.md #47).
# Exit hysteresis, FCA11_HOLD_LOST_KPH, the alerts and the moving law are untouched. False restores the deployed band
# byte-for-byte (read once per instance).
FCA11_STANDSTILL_HOLD_ENABLED = True
FCA11_STANDSTILL_HOLD_LSB = 30


def fca11_standstill_hold_lsb() -> int:
  """17c: the standstill hold ceiling (raw CR_VSM_DecCmd), clamped to the panda gate so no edit can make it unsendable."""
  return int(min(HYUNDAI_FCA11_LONG_MAX_DEC, FCA11_STANDSTILL_HOLD_LSB))


def feel_gentle_gain(accel: float) -> float:
  """0051: the gentle-ask strength scale on the feel target: GENTLE_GAIN at/above GAIN_FADE_ACCEL, linear to 1.0 at the HARD
  bypass, 1.0 at/below it. Always in (0, 1]: the feel target can only get LOWER (authority invariant kept)."""
  if accel <= FCA11_ONSET_BYPASS_ACCEL:
    return 1.0
  if accel >= FCA11_FEEL_GAIN_FADE_ACCEL:
    return FCA11_FEEL_GENTLE_GAIN
  frac = (FCA11_FEEL_GAIN_FADE_ACCEL - accel) / (FCA11_FEEL_GAIN_FADE_ACCEL - FCA11_ONSET_BYPASS_ACCEL)
  return FCA11_FEEL_GENTLE_GAIN + frac * (1.0 - FCA11_FEEL_GENTLE_GAIN)


def feel_gentle_gain_speed_frac(v_kph: float) -> float:
  """0051 (b2): the fraction of the gentle-gain correction still in force at speed ``v_kph`` inside the restore zone: 1.0
  at/above FCA11_FEEL_GAIN_FADE_KPH, 0.0 at/below FCA11_FEEL_GAIN_FULL_KPH, linear in between (never outside [0, 1])."""
  if v_kph >= FCA11_FEEL_GAIN_FADE_KPH:
    return 1.0
  if v_kph <= FCA11_FEEL_GAIN_FULL_KPH:
    return 0.0
  return (v_kph - FCA11_FEEL_GAIN_FULL_KPH) / (FCA11_FEEL_GAIN_FADE_KPH - FCA11_FEEL_GAIN_FULL_KPH)


def feel_gentle_target_lsb(accel: float, personality=None, gain_accel: float | None = None, gain_speed: float = 1.0) -> int:
  """0051 (b): the feel target with the gentle-ask gain: feel_target_lsb(accel * g_eff), where g_eff = 1 - (1 -
  feel_gentle_gain(gain_accel or accel)) * gain_speed. ``gain_speed`` is the (b2) low-speed restore: 1.0 = the gain in full
  force (at speed), 0.0 = the deployed 0049 level (inside the stop-band approach). <= feel_target_lsb(accel) always (both
  factors are in [0, 1]); identical at/below the HARD bypass and at gain_speed 0."""
  g = feel_gentle_gain(accel if gain_accel is None else gain_accel)
  g = 1.0 - (1.0 - g) * min(1.0, max(0.0, gain_speed))
  return feel_target_lsb(accel * g, personality)


def feel_target_lsb(accel: float, personality=None) -> int:
  """0049: the level the feel layer REQUESTS for an ask (before the onset ramp and the panda clamp).

  At/below the onset (-0.5) this is EXACTLY the deployed request, clamp_dec_cmd(dec_cmd_from_accel(accel)); above it (the
  release band / min-on window, where the deployed law clamps the ask to -0.5 -> 8 LSB) it continues the SAME 0.67 inverse
  down through 7/6/5, floored at FCA11_FEEL_FLOOR_LSB. Monotone in accel and never above the deployed request."""
  if personality is None:
    personality = PERSONALITY_STANDARD
  if accel <= FCA11_BRAKE_ONSET_ACCEL:
    return clamp_dec_cmd(dec_cmd_from_accel(accel), personality)
  lsb = int(round((-accel / FCA11_BRAKE_GAIN) / 9.81 * 100))
  return max(FCA11_FEEL_FLOOR_LSB, min(personality_max_dec(personality), lsb))


def feel_soft_rate_lsb_per_sent(lsb: float) -> float:
  """0049: the jerk-limited growth per SENT frame at command level ``lsb``: JERK_TARGET / slope(lsb) x 0.02 s, where
  slope is the delivered m/s^2 per LSB (PROVISIONAL low band below the knee, measured above)."""
  slope = FCA11_FEEL_SLOPE_LOW if lsb < FCA11_FEEL_SLOPE_KNEE_LSB else FCA11_FEEL_SLOPE_HIGH
  return FCA11_FEEL_JERK_TARGET / slope * (FCA11_SEND_EVERY * 0.01)


def feel_rate_lsb_per_sent(accel: float, lsb: float = float(FCA11_FEEL_SLOPE_KNEE_LSB)) -> float:
  """0049: the onset growth allowed per SENT frame: the jerk-limited soft rate for asks at/above RATE_SOFT_ACCEL, the
  panda's +RATE_STEP for a HARD ask (<= FCA11_ONSET_BYPASS_ACCEL, never slowed), linear in between. Never above
  +RATE_STEP (the panda ceiling is applied again by the caller)."""
  step = float(HYUNDAI_FCA11_LONG_RATE_STEP)
  if accel <= FCA11_ONSET_BYPASS_ACCEL:
    return step
  soft = min(step, feel_soft_rate_lsb_per_sent(lsb))
  if accel >= FCA11_FEEL_RATE_SOFT_ACCEL:
    return soft
  frac = (FCA11_FEEL_RATE_SOFT_ACCEL - accel) / (FCA11_FEEL_RATE_SOFT_ACCEL - FCA11_ONSET_BYPASS_ACCEL)
  return soft + frac * (step - soft)


def feel_anticipated_accel(accel: float, slope: float) -> float:
  """0049: the ask predicted FCA11_FEEL_ANTICIPATE_S ahead, used ONLY while it is EASING (slope > 0) and never for a HARD
  ask. Only ever returns an ask that is equal or LESS negative (a lower request) - never more braking."""
  if accel <= FCA11_ONSET_BYPASS_ACCEL or slope <= 0.0:
    return accel
  return accel + min(slope * FCA11_FEEL_ANTICIPATE_S, FCA11_FEEL_ANTICIPATE_MAX)


# *** 0045 (v2, settlement Amendment 1): SHADOW mode — param HyundaiFca11Shadow -> CP_SP.fca11Shadow (DEFAULT OFF) ***
#
# TERMS (fixed by the settlement; used everywhere in this module, the log and the UI):
#   OLD-LAW        the currently DEPLOYED FCA11 law: the 0.67 through-origin inverse + the deployed state machine (what
#                  the car does TODAY with HyundaiFca11Brake ON). In this module = Fca11LongBrake with affine_gain False.
#   V3             the shaped candidate law (0043 v3, affine_gain True). In Shadow it is COMPUTED + LOGGED, NEVER sent.
#   FCA11-DISABLED our FCA11 stack fully off (no host 0x38D at all). Exists ONLY as the explicitly named debug transform
#                  fca11_disabled_debug_params() below - NEVER reachable through the Shadow flag or any param.
#
# WHAT SHADOW DOES: the wire stays EXACTLY today's. setup_interfaces (opendbc sunnypilot/car/interfaces.py) writes
# CP_SP.fca11Shadow and FORCES CP_SP.fca11AffineGain False; HyundaiFca11Brake keeps its deployed meaning (B ON -> OLD-LAW
# actuates, safetyParam bit 256 set, exactly as today; B OFF -> no FCA11 frames, exactly as today with B OFF). So in Shadow:
#   * the actuating Fca11LongBrake IS the OLD-LAW instance (affine_gain additionally forced False here, so a hand-built
#     CP_SP can never put V3 on the wire); its frames are returned by update() untouched -> the sent CAN is byte-identical
#     to the deployed OLD-LAW build for the same HyundaiFca11Brake value;
#   * ONE private V3 compute core (Fca11LongBrake(core=True) on a synthetic armed CP_SP with affine_gain True) runs the
#     full production state machine on the car's CC/CS, open-loop (every V3 would-be frame assumed ACCEPTED: the core gets
#     synthetic src-128 echoes of its own previous frame). Its frames live on `shadow_out.wouldbe_sends` (non_actuating)
#     and in the shadow log; they are NEVER returned.
# THE SEAM (update()): `sends = self._update_law(...)` is the deployed law path, unchanged; in Shadow update() only CAPTURES
#   the V3 inputs (`self._v3_pending = ...`). One-line revert of the seam: delete the `if self.shadow: self._v3_pending = ...`
#   statement in update().
# V3-FAULT ISOLATION: V3 is computed in v3_post_send(), which the superproject card.py calls AFTER sendcan is published
#   (GasInterceptorCarController.fca11_post_send -> here). So V3 work can never delay or alter the frame already on the
#   wire. v3_post_send NEVER raises: any V3 exception is caught, written as a k="v3fail" record, and V3 is DISABLED for the
#   rest of the ignition (its state is untrusted after a fault); a V3 step slower than FCA11_V3_BUDGET_S counts an overrun
#   (k="v3fail" kind=overrun) and FCA11_V3_OVERRUN_TRIP CONSECUTIVE overruns disable V3 the same way. The OLD-LAW instance never reads
#   V3 state. (An in-process Python step cannot be pre-empted; the breaker bounds the damage of a pathological V3 step to
#   FCA11_V3_OVERRUN_TRIP post-send slots, never the current send.)
# LEAD FIELDS (floor-validation primitives; carcontroller.py forwards them from CarControlSP.leadOne = radarState.leadOne):
#   CS.fca11_lead_vrel_mps = leadOne.vRel  and  CS.fca11_lead_one = leadOne (dRel, status, modelProb, radar, aLeadK, vLead).
#   *** 0051: CS.fca11_lead_one is a CONTROL INPUT, not a log-only read: the (c) lead-loss hold arms on a
#   lead present -> absent edge and needs the PRESENCE (None = no lead). 0045 introduced the field as a log-only read
#   ("Log-only reads" in carcontroller.py) - that comment is stale for this field and is flagged for the next fork commit;
#   this patch does not touch carcontroller.py. Absent/None reads as NO lead, which is inert for a CS without the field. ***
#   SIGN CONVENTION (pinned): vRel = vLead - vEgo in m/s (radard: radar track vRel / vision lead v[0] - model vEgo), so
#   vRel < 0 means the gap is CLOSING (the existing fca11_lead_closing gate is `status and vRel < -0.5`). TTC, if wanted, is
#   derived OFFLINE as dRel / -vRel for vRel < 0 - nothing derived is logged.
#   NO LEAD (leadOne.status False, or the attributes absent): lead_present False and every lead_* value is logged as null
#   (never 0 - a 0 m/s vRel or 0 m dRel would read as a real lead).
#   LOW-QUALITY LEAD: NOT filtered here. lead_model_prob (radarState modelProb, 0..1) and lead_radar (radar-fused bool) are
#   logged raw with every lead record so the offline floor analysis applies its own gate (FLOOR-DESIGN §2 suggests
#   modelProb >= 0.5 or radar); the log never drops or rewrites a lead for quality.
# Read ONCE at construction (CarParams build time => next ignition). A SET env FCA11_SHADOW is refused (env-override ban).
FCA11_SHADOW_ENV_REMOVED = "FCA11_SHADOW"
FCA11_V3_BUDGET_S = 0.003          # a V3 post-send step slower than this is an overrun (the 100 Hz frame is 10 ms)
FCA11_V3_OVERRUN_TRIP = 5          # CONSECUTIVE overruns before V3 is disabled for the ignition (each one is logged)
# 0054: the shadow log's IDLE / availability trace. Route 16a: the per-frame trace only exists inside the shadow WINDOW
# (V3/OLD-LAW braking, camera request, or a longActive release ask), so the whole 170 s "FCA11 Braking Unavailable"
# window - raised 14:36:13, cleared 14:39:04 - fell between two file records and every flag a reader could see came
# from the frames on either side of it. While the feature is ARMED but longitudinal is NOT active (and ALWAYS while
# fca11_unavailable holds), the writer therefore emits one frame record every FCA11_SHADOW_IDLE_FRAMES controller frames
# (50 = 0.5 s = 2 Hz) plus ONE immediately on every change of the diagnostic state tuple (see _shadow_diag_state), so an
# availability loss is diagnosable from fca11-shadow.jsonl alone. Same record builder, same field names; the idle record
# adds only `idle` + `diag` (see fca11_shadow_log.frame_idle). Pure logging: nothing here is read by a send decision or
# an alert.
FCA11_SHADOW_IDLE_FRAMES = 50     # 0.5 s at the 100 Hz controller rate (2 Hz idle record)


def shadow_enabled(CP_SP) -> bool:
  """0045: the resolved Shadow state: CP_SP.fca11Shadow AND the pedal armed (the V3 core reads the pedal state).
  Independent of fca11Brake (which keeps deciding whether OLD-LAW actuates). Missing field (older struct) = False."""
  return bool(getattr(CP_SP, "fca11Shadow", False)) and bool(getattr(CP_SP, "enableGasInterceptor", False))


def _refuse_shadow_env() -> None:
  """0045: raise if FCA11_SHADOW is SET (any value): Shadow is reachable ONLY through the HyundaiFca11Shadow param."""
  if FCA11_SHADOW_ENV_REMOVED in os.environ:
    val = os.environ[FCA11_SHADOW_ENV_REMOVED]
    msg = f"{FCA11_SHADOW_ENV_REMOVED} is set ({val!r}) but there is no env override for FCA11 Shadow (patch 0045): "
    msg += "Shadow is enabled ONLY through the HyundaiFca11Shadow param "
    msg += "(Params -> initialize_params -> setup_interfaces -> CP_SP.fca11Shadow). Unset the variable."
    raise RuntimeError(msg)


def fca11_disabled_debug_params(CP_SP) -> structs.CarParamsSP:
  """DEBUG ONLY - the FCA11-DISABLED mode (the first shadow build's semantics, kept for offline A/B isolation): a COPY of
  CP_SP with our FCA11 stack fully off (fca11Brake / fca11AffineGain False, safetyParam bit 256 clear) and the V3 core still
  computing + logging (fca11Shadow True). Called by NO production code and written by NO param: setup_interfaces never
  produces this configuration (pinned by test_fca11_shadow.py::TestFca11DisabledDebugMode). Never use it on a car."""
  from opendbc.sunnypilot.car.hyundai.values import HyundaiSafetyFlagsSP
  d = {f: getattr(CP_SP, f) for f in structs.CarParamsSP.__dataclass_fields__}
  d.update(fca11Brake=False, fca11AffineGain=False, fca11Shadow=True)
  d["safetyParam"] = int(d["safetyParam"]) & ~int(HyundaiSafetyFlagsSP.FCA11_LONG)
  return structs.CarParamsSP(**d)


def _shadow_core_params() -> structs.CarParamsSP:
  """The synthetic, ARMED CarParamsSP the private V3 core is built on. It never leaves this module: it is not the car's
  CP_SP, never reaches card / the panda, and carries no safetyParam bit."""
  return structs.CarParamsSP(fca11Brake=True, enableGasInterceptor=True, fca11AffineGain=True, fca11Shadow=False)


class _ShadowCC:
  """The CarControl fields the V3 core reads (enabled, longActive, actuators.accel), with the ask clipped to the PID bound
  the actuating FCA11 arm has (-FCA11_A_MAX_MSS2; identity with Braking ON, a clip with Braking OFF = pedal-only -3.5)."""
  class _Act:
    def __init__(self, accel: float):
      self.accel = accel

  def __init__(self, enabled: bool, long_active: bool, accel_law: float):
    self.enabled = bool(enabled)
    self.longActive = bool(long_active)
    self.actuators = _ShadowCC._Act(float(accel_law))


class _ShadowCSView:
  """Read-through view of the real CS for the V3 core: reads fall through to CS, WRITES stay on the view, so the core can
  never publish anything (CS._fca11_braking_now, alert flags, ...) onto the car's CarState object."""
  def __init__(self, cs):
    object.__setattr__(self, "_cs", cs)

  def __getattr__(self, name):
    return getattr(object.__getattribute__(self, "_cs"), name)


@dataclass
class ShadowOutput:
  """0045: one frame of Shadow output. ``non_actuating`` is ALWAYS True: the V3 values here are never sent."""
  non_actuating: bool = True
  frame: int = -1
  v3_ok: bool = True                   # False once V3 is disabled by a fault (the v3_* values are then None)
  v3_wouldbe_lsb: int | None = 0       # V3 would-be SENT CR_VSM_DecCmd on this frame (0 = none/passive)
  v3_send_wouldbe: bool = False        # V3 would have put a 0x38D on the wire this frame
  wouldbe_sends: tuple = ()            # the V3 would-be frames (CanData) - NEVER returned / sent
  oldlaw_cmd_lsb: int = 0              # the OLD-LAW CR_VSM_DecCmd actually COMMANDED on the wire this frame (0 = none)
  oldlaw_send: bool = False            # an OLD-LAW 0x38D actually went on the wire this frame
  oldlaw_instant_lsb: int = 0          # the unshaped instant OLD-LAW mapping of the same ask (pure function)


# camera-frame freshness for the mirror source (s): the camera sends ~50 Hz; anything older than
# two periods means we do not know the ESC's last-seen state -> no send (never a hardcoded frame).
FCA11_CAM_FRESH_S = 0.100

# send rate divisor: controller frames are 100 Hz; every other frame = the camera's own 50 Hz.
FCA11_SEND_EVERY = 2

# bit positions of the camera's request fields (the shadow log's camera_warn reads CAM_ACT_WARN_BITS). 0053: the
# hand-back itself is decided ONLY by the shared masks (camera_braking below), never by hand-written bit tests.
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


def _cam_word(dat: bytes) -> int:
  """The camera frame's first 4 bytes, little-endian (= DBC start bits 0..31) - exactly the panda's GET_BYTES_LE(msg, 0, 4)."""
  return int.from_bytes(bytes(dat[:4]), "little")


def camera_braking(dat: bytes) -> bool:
  """0053: True if a bus-2 camera FCA11 frame is BRAKING class (HYUNDAI_FCA11_CAM_BRAKE_MASK) -> hand back."""
  if dat is None or len(dat) != 8:
    return False
  return (_cam_word(dat) & HYUNDAI_FCA11_CAM_BRAKE_MASK) != 0


def camera_warning(dat: bytes) -> bool:
  """0053: True if the frame carries a WARNING-class field (Prefill / Warn). Informational only - never a gate."""
  if dat is None or len(dat) != 8:
    return False
  return (_cam_word(dat) & HYUNDAI_FCA11_CAM_WARN_MASK) != 0


def camera_requesting(dat: bytes) -> bool:
  """Braking OR warning class (the pre-0053 predicate). 0053: INFORMATIONAL ONLY (shadow log field
  ``camera_requesting``); no gate may use it - the hand-back is ``camera_braking``."""
  return camera_braking(dat) or camera_warning(dat)


def fca11_passive(dat: bytes) -> bool:
  """A host 0x38D in the camera's idle shape: no brake field and no warning field set (DecCmd 0, Prefill 0, Warn 0,
  CmdAct 0, DecCmdAct 0, HBA 0, StopReq 0, Belt 0). The panda accepts such a frame unless its cam_brake is set."""
  return dat is not None and len(dat) == 8 and \
    (_cam_word(dat) & (HYUNDAI_FCA11_CAM_BRAKE_MASK | HYUNDAI_FCA11_CAM_WARN_MASK)) == 0


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

  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP, dbc_names: dict | None = None,
               core: bool = False):
    self.CP = CP
    self.CP_SP = CP_SP
    # 0045: `core` = a PRIVATE shadow compute core (built only by a Shadow instance below): no episode log, no cal
    # sequencer, its CS writes go to a view. 0045 SHADOW: the actuating instance is NEVER enabled in Shadow (setup_interfaces
    # already leaves fca11Brake False; re-applied here so a hand-built CP_SP can never actuate in Shadow).
    _refuse_shadow_env()
    self._core = bool(core)
    self.shadow = (not self._core) and shadow_enabled(CP_SP)
    self.enabled = bool(CP_SP.fca11Brake) and bool(CP_SP.enableGasInterceptor)
    # 0041: the params-gated AFFINE command law (HyundaiFca11AffineGain, DEFAULT OFF). Read ONCE here, never per
    # frame. OFF -> the 0.67 through-origin law and the 8 LSB floor, byte-identical to 0040. 0044: CP_SP is the ONLY
    # source (the env override is refused, the Params fallback is gone). 0045: in Shadow the WIRE is OLD-LAW, always:
    # setup_interfaces already forces fca11AffineGain False; re-applied here so V3 can never actuate in Shadow.
    _refuse_affine_env()
    self.affine_gain = affine_gain_enabled(CP_SP) and not self.shadow
    self.braking = False               # an FCA11 actuating frame was sent THIS frame
    self.blocked_until_resume = False  # a CUT latched: no braking until the next resume press
    # G9b/0042: split the cut the way the panda does. _driver_cut (brake / gear != D) re-arms on the resume
    # edge but clears on the first clean engaged frame; _pedal_cut (pedal fault, 0042) is fail-closed and NOT
    # re-armed. 0042 v2: blocked_until_resume is the DRIVER cut only - the pedal mirror is ALERT-ONLY and never
    # gates a send (a pre-arm over-latch must never fabricate a brake loss); see update() for the full reasoning.
    # 0053: the camera hand-back is no longer a latch (the old _hard_cut is deleted): see cam_brake below.
    self._driver_cut = False
    self._pedal_cut = False            # 0042: the panda's hyundai_interceptor_pedal_fault, latched (see update())
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
    if self._core:
      self.log.enabled = False         # 0045: the V3 core never writes the ACTUATION episode log (episodes.jsonl)
    self._last_why = ""                # this frame's fca11_ok reason (the episode-end reason when !ok)
    # 0038: gas hold-off state. `_gas_release_frame` is the controller frame the pedal last read released (None =
    # none yet this controller); the hold-off refuses actuation for GAS_HOLDOFF_FRAMES after it. No edge is needed to
    # clear it (it just ages out), and gas is NOT part of _driver_cut any more.
    self._gas_release_frame: int | None = None
    self._gas_prev = False
    # 0030 (D1-b) / 0042: surface the fail-closed loss. The panda's hyundai_fca11_long_cut is fail-closed for the
    # REST OF THE IGNITION and cannot be cleared, so while FCA11-long is armed it refuses EVERY actuating frame for
    # the whole drive - and that failure was otherwise SILENT (the driver just sees openpilot plan stops it cannot
    # execute; route 156 lost ALL braking this way). The car layer mirrors the panda latch SOURCE as _pedal_cut (a
    # PEDAL FAULT, 0042; fail-closed, NOT re-armed). 0053: the camera hand-back is no longer a source - it is the
    # self-releasing cam_brake with its own transient alert. _refused_cut is the ECHO-CONFIRMED wall (B): the panda's own verdict on our
    # in-flight frames, so even a latch the Python mirror cannot see still surfaces. CarStateExt publishes the OR as
    # CarStateSP.fca11Unavailable; the superproject raises a persistent driver alert from it.
    # 0042 v2: this flag is KNOW + ALERT only - it drives NO send gate (see blocked_until_resume below). While it is
    # set, the three 0040/0041 braking alerts are SUPPRESSED in _alerts() (they all assume we are actually braking):
    # the dedicated unavailability alert supersedes them.
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
    # 0042 (B): the echo-confirmed refusal wall. `_refused_streak` counts CONSECUTIVE refused ACTUATING echoes while
    # braking; at FCA11_REFUSED_STREAK_CUT it sets `_refused_cut` (surfaced via fca11_unavailable). An ACCEPTED
    # actuating echo resets the streak and self-heals the flag (the wall is gone). No send gating rides on this.
    self._refused_streak = 0
    self._refused_cut = False
    # 0053: the NON-LATCHING camera hand-back (see the FCA11_CAM_* block). `cam_brake` = the camera is braking (a
    # braking-class bus-2 0x38D within the last FCA11_CAM_RESUME_FRAMES camera frames): no 0x38D of ours goes out at
    # all and the pedal command is forced to 0. `_cam_quiet` = non-braking camera frames since the last braking one
    # (saturating). Starts released, like the panda's hyundai_init.
    self.cam_brake = False
    self._cam_quiet = FCA11_CAM_RESUME_FRAMES
    # 16a: controller frames left in which the camera STILL OWNS our refusals after we released (see
    # FCA11_CAM_RELEASE_TAIL_FRAMES). 0 = the panda's hand-back cannot outlive ours any more.
    self._cam_tail = 0
    self.camera_braking_alert = False  # transient "camera braking - openpilot braking paused" (CarStateSP)
    # 0053: refusal-cut probation (FCA11_PROBATION_*): actuating echoes left in the window, refused ones counted in it,
    # and whether the window tripped the cut (held until the window closes).
    self._probation_left = 0
    self._probation_refused = 0
    self._probation_cut = False
    # 0053: divergence guard (FCA11_DIVERGENCE_*). `_div_armed` = an accepted 0x38D echo was seen (the panda is armed);
    # `_div_refused_passive` = refused PASSIVE echoes waiting to be judged after this tick's camera frames are ingested;
    # `_div_streak` consecutive ones; `_div_events` per ignition; `_div_latched` = withhold every 0x38D until the next
    # ignition (the only hard camera-side cut left). `fca11_divergence` drives the driver alert.
    self._div_armed = False
    self._div_refused_passive = 0
    self._div_streak = 0
    self._div_events = 0
    self._div_latched = False
    self._div_active = False
    self.fca11_divergence = False
    self._attrib_hold = 0              # controller frames left of a refusal-attribution hold (passive frames only)
    self._cam_ingested_frame: int | None = None
    self._engaged_prev = False         # CC.enabled edge tracker (resume = the re-engage rising edge)
    self._dec_last_sent = 0
    self._sent_this_episode = False
    # 0043: the shaped-law state (ON path only). `_shape_acc` is the float LSB accumulator the jerk slew ramps
    # (starts at the floor); `_shape_sent_n` counts SENT actuating frames since this episode opened (the onset
    # hold is measured in SENT-frame time); `_shape_in_hold_band` is the hold-band latch (ENTRY <= 3 km/h, EXIT
    # >= 5 km/h - 0043 v2 hysteresis). Never read on the OFF path -> the OFF behaviour is untouched.
    self._shape_acc = float(FCA11_SHAPE_FLOOR_LSB)
    self._shape_sent_n = 0
    self._shape_in_hold_band = False
    # 0049: the FEEL layer (OLD-LAW instance only; see the FCA11_FEEL_* block). Read ONCE here. Never on the shaping/V3
    # path (affine_gain) - so the Shadow V3 core and the shadow log are untouched.
    self.feel = bool(FCA11_FEEL_ENABLED) and self.enabled and not self.affine_gain
    self._feel_acc = 0.0               # float LSB accumulator of the onset ramp (episode-local)
    self._feel_slope = 0.0             # EMA of d(ask)/dt over SENT frames (m/s^3), episode-local
    self._feel_prev_ask: float | None = None
    self._feel_in_hold_band = False    # hold-band latch (ENTER <= 3 km/h or standstill, EXIT >= 5 km/h)
    # 17c: the standstill-hold latch (FCA11_STANDSTILL_HOLD_*). Feel path only, read once like every other law switch.
    self.standstill_hold = self.feel and bool(FCA11_STANDSTILL_HOLD_ENABLED)
    self._feel_standstill_hold = False  # set at standstill in the band; cleared by band EXIT / episode (re)open / reset
    self._feel_cal_prev = False        # previous frame's cal_on (a cal->planner hand-off re-seeds the ramp)
    self._feel_dwell = 0               # consecutive frames of (ok AND ask <= the deployed -0.5 onset): the dwell escape
    self._feel_dwell_slow = 0          # the same, AND vEgo < FCA11_FEEL_DWELL_SLOW_MPS on every counted frame (crawl escape)
    # 0051: the feel FIX round (wobble damping, gentle gain, lead-loss hold). Read ONCE; only ever on the feel path.
    self.feel_fix = self.feel and bool(FCA11_FEEL_FIX_0051)
    # 0054: the MEASURED ask -> LSB inverse + its release-below-the-floor arm. Read ONCE here, like every other law
    # switch. Scoped to the FEEL layer on purpose: that is the deployed request path, and `self.feel` already carries
    # "enabled AND not the 0043 shaping law AND not Shadow", so this flag can never reach the shaped law, cal, the
    # shadow core or a disabled instance. OFF -> the pre-0054 request map and release rule, byte-for-byte.
    self.meas_inverse = bool(FCA11_FEEL_ENABLED) and bool(FCA11_MEAS_INVERSE) and self.feel
    self._feel_slope_ant = 0.0         # slow EMA of d(ask)/dt (anticipation only; FCA11_FEEL_SLOPE_ALPHA_0051)
    self._feel_peak = 0.0              # episode peak of the feel accumulator (re-rise deadband reference)
    self._feel_lead_prev = False       # previous frame's lead presence (edge detector)
    self._feel_ll_frame: int | None = None   # frame of the lead-loss edge while an episode was open (None = no hold)
    self._feel_ll_lsb = 0              # level frozen at that edge (the last SENT level)
    self._feel_ll_hold_from = 0        # frame the HOLD phase is counted from (advances while the lead stays absent)
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
    if self.enabled and not self._core and os.environ.get("FCA11_CAL_DIR") != "0":
      self.cal_seq = cal_mode.CalSequencer(os.environ.get("FCA11_CAL_DIR", cal_mode.CAL_DIR))
      self.cal_seq.mark_session(os.getpid())
    self._cal_active = False           # this frame is a CAL hold (drives the gas neutralization)
    self._cal_prev = False             # 0043 v2: previous frame's cal_on, for the cal-active EDGE reset of the shaped ramp
    # ---- 0045 SHADOW: the private V3 compute core + the shadow log. None unless Shadow. ----
    self.shadow_out = ShadowOutput()
    self._shadow_v3 = None
    self.shadow_log = None
    self._v3_pending = None            # this frame's V3 inputs, captured in update(), consumed by v3_post_send()
    self._v3_disabled = False          # a V3 fault (exception / FCA11_V3_OVERRUN_TRIP overruns) disabled V3 this ignition
    self.v3_failures = 0               # exceptions + overruns seen (each one logged as k="v3fail")
    self.v3_overruns = 0
    self._v3_overrun_streak = 0        # CONSECUTIVE overruns (a lone GC pause is logged, never disables V3)
    if self.shadow:
      from opendbc.sunnypilot.car.hyundai import fca11_shadow_log
      self._shadow_v3 = Fca11LongBrake(CP, _shadow_core_params(), dbc_names, core=True)
      assert self._shadow_v3.affine_gain and not self.affine_gain
      self.shadow_log = fca11_shadow_log.ShadowLogger(header=dict(
        note="FCA11 SHADOW (0045 v2): wire = OLD-LAW (deployed law, as today); V3 computed + logged post-send, NEVER sent; "
             "V3 values are an open-loop counterfactual",
        oldlaw_armed=bool(self.enabled),
        consts=dict(K=FCA11_SHAPE_K, BITE=FCA11_SHAPE_BITE, CLIFF_LO_EXTRA=FCA11_SHAPE_CLIFF_LO_EXTRA,
                    CLIFF_HI_EXTRA=FCA11_SHAPE_CLIFF_HI_EXTRA, ONSET_HOLD_SENT=FCA11_SHAPE_ONSET_HOLD_SENT,
                    JERK_TARGET=FCA11_SHAPE_JERK_TARGET, BRAKE_GAIN=FCA11_BRAKE_GAIN, A_MAX_MSS2=FCA11_A_MAX_MSS2,
                    ONSET_ACCEL=FCA11_BRAKE_ONSET_ACCEL, RELEASE_ACCEL=FCA11_BRAKE_RELEASE_ACCEL,
                    V3_BUDGET_S=FCA11_V3_BUDGET_S, V3_OVERRUN_TRIP=FCA11_V3_OVERRUN_TRIP)))
    self._sh_engaged_prev = False
    self._sh_engage_frame: int | None = None
    self._sh_episode_id = 0
    self._sh_ep: dict | None = None
    self._sh_prev_echo: list = []      # 0045 (V3 core only): synthetic ACCEPTED echoes of the previous frame's frames
    # 0054: the IDLE availability trace (armed but longActive off, or fca11_unavailable): last frame an idle record went
    # out, and the diagnostic state tuple it carried. Written by _shadow_record only; read by nothing else.
    self._sh_idle_last_frame: int | None = None
    self._sh_diag_prev: dict | None = None

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
    # brake. 1 BAD_CHECKSUM / 2 SEND / 3 SCE / 6 INVALID are faults -> never brake. 0042: the accepted
    # set is the shared HYUNDAI_FCA11_PEDAL_STATE_OK mirror (one source of truth with the panda + the latch).
    state = int(getattr(CS, "interceptor_state", 0))
    if state not in HYUNDAI_FCA11_PEDAL_STATE_OK:
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
    # 0053: ONLY the braking class hands back (a warning keeps openpilot braking). update() already withholds every
    # frame while cam_brake / the divergence latch hold; this is the belt-and-braces gate on the freshest frame.
    if self._div_latched:
      return False, "divergence_latched"
    if self.cam_brake or camera_braking(dat):
      return False, "camera_braking"
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
    # 0042 v2: while FCA11-long is UNAVAILABLE (the panda refuses every actuating frame for the rest of the
    # ignition: pedal-fault latch / echo-confirmed refusal wall; 0053: no longer the camera), the three 0040/0041 alerts
    # above are MISLEADING - they all assume we are actually braking and delivering decel. The dedicated
    # "FCA11 Braking Unavailable" permanent alert supersedes them, so force them off; they re-arm normally on
    # their own conditions once availability returns (fca11_unavailable clears on an accepted actuating echo).
    if self.fca11_unavailable:
      self.supervise_stop = False
      self.stop_complete = False
      self.brake_now = False
      self.brake_now_reason = ""
      self.low_speed_alert = False

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

  def _camera_owns_refusal(self) -> bool:
    """0053/route 16a: True when a REFUSED actuating echo this tick is the camera hand-back's doing, not a wall.

    Route 16a: the stock camera's bus-2 0x38D carried BRAKING-class frames through 130.64-136.11 s, so the panda's
    cam_brake held and it refused our actuating frames at 136.07, 136.09 and 136.11 - the panda doing exactly what the
    camera hand-back requires, and the mirror of our own withhold. The car layer counted those three as evidence of a
    wall, _refused_cut latched at 136.13 and the driver got "FCA11 Braking Unavailable", which then outlived the
    condition by 170 s: the wall could only heal on an ACCEPTED actuating echo and the next brake request was at
    306.63 s, while the car had braking available again ~1.5 s after the camera went quiet. A refusal the hand-back
    explains is therefore not evidence and must never advance (or leave standing) the streak.

    Ownership holds for every tick in which this layer is withheld from 0x38D on the camera's behalf: while
    ``cam_brake`` is set, through the quiet tail it keeps after the last braking camera frame (``_cam_quiet``
    below FCA11_CAM_RESUME_FRAMES), and through FCA11_CAM_RELEASE_TAIL_FRAMES controller frames after its release
    (``_cam_tail``) - the panda counts its quiet window in ticks, we count ours in camera frames, so our release can
    come first and its hand-back still refuse our resumption frames. Both tails decay: a camera that never braked,
    or one that released long ago and stayed quiet, owns nothing and a real wall still latches.
    """
    return bool(self.cam_brake or self._cam_quiet < FCA11_CAM_RESUME_FRAMES or self._cam_tail > 0)

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
    if self._cam_tail > 0:
      self._cam_tail -= 1              # 16a: one controller frame of the camera's post-release ownership
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
    # 0042 (B): echo-based refusal surfacing. The panda's verdict on our in-flight frames is the ground truth. A
    # REFUSED ACTUATING echo (src 192) while braking extends the consecutive streak; an ACCEPTED ACTUATING echo
    # (src 128) means the panda is taking our frames again, so reset the streak and self-heal the wall flag. At
    # FCA11_REFUSED_STREAK_CUT consecutive refusals the wall is surfaced via fca11_unavailable. NO send suppression
    # rides on this: refused frames never reach the car, so keeping our sends live keeps the ground truth flowing with
    # zero deadlock. (A refused PASSIVE close frame is the separate re-owe path below, not counted here.)
    refused = sum(1 for src, dat in echoes if int(src) == 192 and len(dat) == 8 and dat[1] != 0)
    accepted = sum(1 for src, dat in echoes if int(src) == 128 and len(dat) == 8 and dat[1] != 0)
    # 0053: refusals that land while the camera hand-back is active are the hand-back doing its job, not a wall - see
    # _camera_owns_refusal. They are EXPECTED and say nothing about a wall or a divergence, so they feed neither the
    # refusal streak, the probation, nor the guard. (`_refused_streak` is zeroed too: a wall cannot be "in progress"
    # across a hand-back, so a mixed sequence - a couple of unexplained refusals, then hand-back refusals, then more -
    # must not add up to FCA11_REFUSED_STREAK_CUT. ingest_camera runs BEFORE observe_echo on the real path, so the
    # hand-back state already reflects this tick's camera.)
    owned = self._camera_owns_refusal()
    if owned:
      refused = 0
      if self.braking:
        self._refused_streak = 0
    # 0053 divergence guard: an ACCEPTED 0x38D of any shape arms it (the panda is in FCA11-long and taking frames) and
    # breaks a refused-passive streak; a REFUSED PASSIVE echo while we believe the camera is NOT braking extends it.
    if any(int(src) == 128 and len(dat) == 8 for src, dat in echoes):
      self._div_armed = True
      self._div_streak = 0
      if any(int(src) == 128 and fca11_passive(bytes(dat)) for src, dat in echoes):
        self._attrib_hold = 0            # attribution answered: the panda takes passive frames -> not the camera
    elif self._div_armed and not self.cam_brake and \
        any(int(src) == 192 and fca11_passive(bytes(dat)) for src, dat in echoes):
      self._div_streak += 1
      if self._div_streak >= FCA11_DIVERGENCE_PASSIVE_STREAK:
        self._divergence_event()
    # 0053 probation: the first FCA11_PROBATION_ECHOES actuating echoes after a resume also run a CUMULATIVE refusal
    # count (an accepted echo does not reset it). Fed only by real braking: no probe decel is ever sent for it.
    if self._probation_left > 0 and (accepted or (refused and self.braking)):
      n = accepted + (refused if self.braking else 0)
      self._probation_refused += refused if self.braking else 0
      self._probation_left = max(0, self._probation_left - n)
      if self._probation_refused >= FCA11_PROBATION_REFUSED_CUT:
        self._probation_cut = True
        self._refused_cut = True
      if self._probation_left == 0:
        self._probation_cut = False   # the window closed: the ordinary consecutive rule (and its heal) owns the cut
    if accepted and self._probation_cut:
      # a probation trip is held until its window closes: an occasional accepted frame in a mostly refused wall is
      # exactly what the cumulative count exists to catch. (The pedal-fault heal below still applies: an accepted
      # actuating frame proves the panda's pedal cut is clear.)
      self._refused_streak = 0
      self._pedal_cut = False
    elif accepted:
      # An ACCEPTED ACTUATING echo (src 128, len 8, dat[1]!=0) is PROOF the panda is NOT latched: it accepted a
      # frame that only goes out while FCA11-long is armed AND its cut is clear. So it resets the refusal streak,
      # clears _refused_cut (the wall is gone) AND clears _pedal_cut (the pedal-fault latch cannot be live if the
      # panda is accepting our actuating frames). 0042 v2: this is the mirror's PROOF-BASED SELF-HEAL - the only
      # thing that clears _pedal_cut, and why a pre-arm over-latch (route 155) self-corrects at the first accepted
      # actuating echo instead of holding a phantom alert all drive. A PASSIVE accepted echo (dat[1]==0) is NOT
      # proof (150709 passive frames flowed on route 156 WITH the cut latched) and must not heal anything.
      self._refused_streak = 0
      self._refused_cut = False
      self._pedal_cut = False
    elif refused and self.braking:
      # 16a: `refused` is zero here whenever the camera owns the tick's refusals (see _camera_owns_refusal, which also
      # zeroes the streak), so this increment can only ever count refusals the hand-back does NOT explain.
      was = self._refused_streak
      self._refused_streak += refused
      if self._refused_streak >= FCA11_REFUSED_STREAK_CUT:
        self._refused_cut = True
        # 0053 ATTRIBUTION: a refused ACTUATING frame is ambiguous (window, rate, gas hold-off ... or the panda's
        # cam_brake), a refused PASSIVE one is not. On the crossing into a wall, end the episode so the owed passive
        # close goes out (zero decel - the ESC was getting nothing from us anyway): accepted -> a real wall, the cut
        # stands; refused twice -> the panda is handing back to a camera brake we never saw -> divergence (resync).
        if was < FCA11_REFUSED_STREAK_CUT and self._div_armed and not self.cam_brake:
          self._attrib_hold = FCA11_ATTRIB_HOLD_FRAMES
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

  # ---- 0053: camera hand-back + divergence guard ----

  def _cam_step(self, dat: bytes) -> None:
    """One bus-2 camera 0x38D through the hand-back state machine (the panda's hyundai_fca11_long_cam_step, released
    FCA11_CAM_QUIET_MARGIN frames later)."""
    if dat is None or len(dat) != 8:
      return
    if camera_braking(dat):
      if not self.cam_brake:
        self._refused_streak = 0         # refusals around the hand-back edge are the hand-back, not a wall
      self.cam_brake = True
      self._cam_quiet = 0
      self._cam_tail = 0                 # the hand-back itself owns them now; no tail is owed
    elif self._cam_quiet < FCA11_CAM_RESUME_FRAMES:
      self._cam_quiet += 1
      if self._cam_quiet >= FCA11_CAM_RESUME_FRAMES and self.cam_brake:
        self.cam_brake = False
        self._div_active = False         # a resync hold ends with the hand-back it adopted
        self._start_probation()
        # 16a: we released on OUR clock (camera frames), which can run ahead of the panda's (its ticks), so the panda may
        # still be handing back for up to its own quiet window: the camera keeps ownership of refusals until then.
        self._cam_tail = FCA11_CAM_RELEASE_TAIL_FRAMES
        # the FIRST host 0x38D after every release is a PASSIVE frame (the camera's idle shape, zero decel - not a
        # probe): it re-opens our stream cleanly at the panda AND is the divergence guard's evidence - the panda
        # refuses a well-formed passive frame ONLY while its own cam_brake still holds.
        self._release_pending = True

  def _start_probation(self) -> None:
    self._probation_left = FCA11_PROBATION_ECHOES
    self._probation_refused = 0
    self._probation_cut = False

  def _divergence_event(self) -> None:
    """The panda refused passive frames while we believed the camera was not braking: adopt the panda's view (the
    stricter direction), raise the alert, latch on the FCA11_DIVERGENCE_LATCH_EVENTS-th event this ignition."""
    self._div_streak = 0
    self._div_events += 1
    self._div_active = True
    self._attrib_hold = 0
    self._refused_streak = 0             # the refusals were the panda's hand-back, not a wall
    self._refused_cut = False
    self.cam_brake = True
    self._cam_quiet = 0
    if self._div_events >= FCA11_DIVERGENCE_LATCH_EVENTS:
      self._div_latched = True

  def ingest_camera(self, CS, frame: int) -> None:
    """0053: feed EVERY bus-2 camera 0x38D of this tick to the hand-back state machine, in order. Runs once per
    controller frame, BEFORE observe_echo and update (create_gas_command calls it first; update() calls it itself if
    nobody did). Never gated on engagement or on longActive: the drive-14d divergence was exactly such a gate.

    Real path: CS.fca11_cam_frames (CarStateExt, every frame of the tick). Fallback for callers that only carry the
    freshest frame (CS.fca11_cam_frame): a non-braking frame counts once per 50 Hz send slot (the camera's own rate),
    a BRAKING-class frame counts on every controller frame (the stricter direction: a brake is never skipped)."""
    if self._cam_ingested_frame == frame:
      return
    self._cam_ingested_frame = frame
    if not self.enabled:
      return
    frames = getattr(CS, "fca11_cam_frames", None)
    if frames is None:
      cam = getattr(CS, "fca11_cam_frame", None)
      frames = [cam] if (cam is not None and (frame % FCA11_SEND_EVERY == 0 or camera_braking(cam[1]))) else []
    for _t, dat in frames:
      self._cam_step(bytes(dat))

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

  def _shape_reset(self) -> None:
    """0043: arm a fresh shaped episode - the float LSB accumulator starts at the floor, the onset-hold
    sent-frame counter and the hold-band latch are cleared. Called on every (re)open, on disable, on the
    longActive-off exit, and on a cal-active EDGE (0043 v2: a cal->planner hand-off must not carry stale state)."""
    self._shape_acc = float(FCA11_SHAPE_FLOOR_LSB)
    self._shape_sent_n = 0
    self._shape_in_hold_band = False

  def _shape_level(self, accel_ask: float, v_kph: float, CS, frame: int,
                   personality=PERSONALITY_STANDARD) -> int:
    """0043: the raw CR_VSM_DecCmd the SHAPED law requests for one 100 Hz frame (ON path).

    State advances ONLY on the 50 Hz SENT slot (frame % FCA11_SEND_EVERY == 0): the emitted level is a staircase
    and the jerk slew counts per SENT frame. On a non-send frame the last accumulator value is re-offered (it is
    not transmitted, so this only feeds the alert/logger view). The return is capped by the panda contract - a
    sent frame may grow at most +RATE_STEP over the last ACCEPTED frame (0 at an episode open, which is why the
    FIRST sent frame is the floor 4). Never returns 0 within an episode; a release is the caller's job.
    """
    cap = personality_max_dec(personality)
    want = max(0.0, -accel_ask)
    # 0043 v2: hold-band EXIT hysteresis. ENTER at <= FCA11_SHAPE_HOLD_BAND_KPH (3 km/h) or CarState.standstill;
    # leave only at >= FCA11_SHAPE_HOLD_EXIT_KPH (5 km/h). Without it a DCT creeping across the 3 km/h edge steps the
    # command cap<->level (e.g. 20 -> 6 -> 20), a repeated ~1 m/s^2 step. The latch is updated every frame (it is
    # above the send-slot return) so the exit is seen promptly.
    standstill = bool(getattr(CS.out, "standstill", False))
    if standstill or v_kph <= FCA11_SHAPE_HOLD_BAND_KPH:
      self._shape_in_hold_band = True
    elif v_kph >= FCA11_SHAPE_HOLD_EXIT_KPH:
      self._shape_in_hold_band = False
    in_hold_band = self._shape_in_hold_band
    # level-law target (integer LSB): the tier cap in the stop/hold band, else the shaped inverse of the demand.
    target = cap if in_hold_band else shape_dec_cmd_from_extra(want, cap)
    if frame % FCA11_SEND_EVERY != 0:
      return int(self._shape_acc)
    hard = bool(accel_ask <= FCA11_ONSET_BYPASS_ACCEL)
    # 0043 v2: a HARD ask is an EMERGENCY. It bypasses the onset hold AND the comfort jerk slew: its growth runs at
    # up to the panda ceiling, the SAME +RATE_STEP/sent-frame rate the (pre-shaping) fast ramp uses, so a hard demand
    # reaches the tier cap in a few sent frames (~0.1 s) instead of seconds. The shaped LEVEL target is kept (an
    # honest inverse level once reached). The hold band is exempt for the same reason (a stop wants the cap promptly).
    fast = bool(hard or in_hold_band)
    prev = self._shape_acc
    if (not in_hold_band) and (not hard) and self._shape_sent_n < FCA11_SHAPE_ONSET_HOLD_SENT:
      # onset hold: NO GROWTH before the hold has elapsed. At an episode open `prev` IS the floor, so this holds the
      # floor exactly as before. 0043 v2: after a HARD phase (the fast lane grew `prev` above the floor) an ask that
      # eases into the soft band inside the onset hold must NOT be forced back down to the floor - that would be a
      # 12 -> 4 step across the delivered-decel cliff and a re-climb at the comfort slew (a release-and-reapply). Hold
      # the level, and follow the (lower) honest soft level down immediately (decreases are the safe direction).
      new_acc = prev if target >= prev else float(target)
      # ... and NO growth of the SENT value either. `_shape_acc` is stored BEFORE the panda clamp below, so after a
      # short HARD phase it sits up to one +RATE_STEP AHEAD of the last sent frame; without this cap a soft ask
      # that eases in during the hold would let the sent value CLIMB to it (4 -> 7 in one frame). Cap at the last
      # sent level (the floor at an episode open, where _dec_last_sent is 0). Only ever lowers the command.
      new_acc = min(new_acc, float(max(self._dec_last_sent, FCA11_SHAPE_FLOOR_LSB)))
    elif target >= prev:
      # grow toward the target: the panda's own +RATE_STEP on a hard ask / in the hold band, the comfort jerk slew
      # on a soft (comfort) moving ask.
      slew = float(HYUNDAI_FCA11_LONG_RATE_STEP) if fast else shape_slew_lsb_per_sent(prev)
      new_acc = min(float(target), prev + slew)
    else:
      new_acc = float(target)                           # decrease is immediate (safe direction; release ramp deferred)
    new_acc = max(float(FCA11_SHAPE_FLOOR_LSB), new_acc)
    self._shape_acc = new_acc
    self._shape_sent_n += 1
    # panda contract: never exceed +RATE_STEP over the last ACCEPTED frame (0 at an episode open -> the first
    # sent frame is the floor 4, which the panda accepts).
    return min(int(new_acc), self._dec_last_sent + HYUNDAI_FCA11_LONG_RATE_STEP)

  def _feel_reset(self, seed: float = float(FCA11_FEEL_FIRST_LSB)) -> None:
    """0049: arm the feel ramp for a (re)opened episode: the accumulator starts at ``seed`` (the first-frame level, or
    the last SENT level on a cal->planner hand-off so there is no step), the slope estimate is cleared."""
    self._feel_acc = float(seed)
    self._feel_slope = 0.0
    self._feel_prev_ask = None
    self._feel_slope_ant = 0.0         # 0051: the slow (anticipation-only) slope EMA
    self._feel_peak = float(seed)      # 0051: the re-rise deadband reference restarts with the episode
    self._feel_ll_frame = None         # 0051: no lead-loss hold survives into a new episode
    self._feel_standstill_hold = False  # 17c: nor a standstill hold (re-set at once if the car is still at standstill)

  def _feel_target(self, a_tgt: float, accel: float, personality, gain_speed: float = 1.0,
                   v_mps=None) -> int:
    """0051 (b): the feel target for the (possibly anticipated) ask ``a_tgt``, with the gentle-ask gain chosen by the RAW ask
    ``accel`` and the (b2) low-speed restore ``gain_speed``. Off the fix (feel_fix False) this is exactly the 0049
    feel_target_lsb(a_tgt).

    0054: on the MEASURED law (self.meas_inverse) this is the measured inverse of the ask and NOTHING ELSE. The 0051
    gentle gain (0.70) existed only to cancel the 0.67 inverse's ~2x over-delivery; applied on top of an honest inverse
    it would UNDER-deliver by 30%, so it is not applied. Same for the anticipation/low-speed-restore plumbing downstream:
    they keep working on the level, they just have a level that means what it says.
    0055: ``v_mps`` selects the speed-indexed floor (fca11_floor_lsb) instead of the fixed 4, so the level rises into the
    stop band as the car slows instead of stepping at band entry. None = at speed = the moving floor.
    """
    if self.meas_inverse:
      return min(personality_max_dec(personality), dec_cmd_from_accel_measured(a_tgt, v_mps))
    if not self.feel_fix:
      return feel_target_lsb(a_tgt, personality)
    return feel_gentle_target_lsb(a_tgt, personality, gain_accel=accel, gain_speed=gain_speed)

  def _feel_lead_loss_floor(self, frame: int) -> float:
    """0051 (c): the lead-loss floor (LSB) at ``frame``: the level frozen at the edge for LEAD_LOSS_HOLD_S, then linear to 0
    over LEAD_LOSS_TAPER_S; 0 when no hold is active (and the hold is cleared once it has run out)."""
    if self._feel_ll_frame is None:
      return 0.0
    dt = (frame - self._feel_ll_hold_from) * 0.01
    if dt < FCA11_FEEL_LEAD_LOSS_HOLD_S:
      return float(self._feel_ll_lsb)
    frac = (dt - FCA11_FEEL_LEAD_LOSS_HOLD_S) / FCA11_FEEL_LEAD_LOSS_TAPER_S
    if frac >= 1.0:
      self._feel_ll_frame = None
      return 0.0
    return float(self._feel_ll_lsb) * (1.0 - frac)

  def _feel_update_band(self, v_kph: float, CS) -> bool:
    """0049: update + return the hold-band latch (ENTER <= FCA11_FEEL_HOLD_BAND_KPH or standstill; EXIT >= EXIT_KPH)."""
    if bool(getattr(CS.out, "standstill", False)) or v_kph <= FCA11_FEEL_HOLD_BAND_KPH:
      self._feel_in_hold_band = True
    elif v_kph >= FCA11_FEEL_HOLD_EXIT_KPH:
      self._feel_in_hold_band = False
      self._feel_standstill_hold = False   # 17c: the standstill hold leaves with the band (the UNCHANGED exit hysteresis)
    return self._feel_in_hold_band

  def _feel_band_clamp(self, dec: int, personality) -> int:
    """17c: the hold band's clamp. Not latched -> clamp_dec_cmd (the deployed tier clamp, byte-for-byte). Latched at
    standstill -> the same clamp with the ceiling raised to max(tier cap, the standstill hold level)."""
    if not self._feel_standstill_hold:
      return clamp_dec_cmd(dec, personality)
    return max(1, min(max(personality_max_dec(personality), fca11_standstill_hold_lsb()), dec))

  def _feel_level(self, accel: float, frame: int, personality=PERSONALITY_STANDARD, v_kph=None,
                  standstill: bool = False) -> int:
    """0049: the raw CR_VSM_DecCmd the OLD-LAW feel layer requests for one 100 Hz frame (planner ask, moving).

    State advances only on the 50 Hz SENT slot (the ramp + slope are per SENT frame); a non-send frame re-offers the
    accumulator (not transmitted - alerts/logger view only). The return is ALWAYS capped by the panda contract (+RATE_STEP
    over the last SENT frame), and is never above the deployed law's request for the same ask.
    0051 (b2): ``v_kph`` feeds the low-speed restore (the gentle gain fades back as the car slows so the stop band cannot
    step the level up at band entry). v_kph None (unit callers) = at speed = the gain in full force."""
    if self._feel_in_hold_band:
      # the stop/hold band runs the DEPLOYED law exactly: request clamped like before, +RATE_STEP over the last sent frame.
      # 0055: its floor is now the SPEED-INDEXED one, taken EXPLICITLY (fca11_floor_lsb) instead of being inherited from
      # the -0.5 onset clamp at 8. At every band speed (<= FCA11_FEEL_HOLD_BAND_KPH = 3 km/h = 0.83 m/s <= 1.0 m/s) that
      # floor IS 8, so the band is unchanged here -- that is the point: the ramp finishes ABOVE the band edge, so entry
      # no longer steps the level. An ask the deployed request takes ABOVE the floor still commands what it commanded
      # (max(), never min()): the standstill hold is not weakened. Gated on meas_inverse so the 0054 kill switch still
      # restores the pre-0054 map byte-for-byte.
      # 17c: at TRUE standstill the band's ceiling (both tier clamps below) is raised to the standstill hold level, latched
      # until the band exits / the episode re-opens (FCA11_STANDSTILL_HOLD_*). The +RATE_STEP limiter is unchanged.
      if self.standstill_hold and standstill:
        self._feel_standstill_hold = True
      dec = self._feel_band_clamp(dec_cmd_from_accel(min(accel, FCA11_BRAKE_ONSET_ACCEL)), personality)
      if self.meas_inverse:
        band_floor = fca11_floor_lsb(None if v_kph is None else v_kph / 3.6)
        dec = max(dec, min(band_floor, personality_max_dec(personality)))
      dec = self._feel_band_clamp(min(dec, self._dec_last_sent + HYUNDAI_FCA11_LONG_RATE_STEP), personality)
      if frame % FCA11_SEND_EVERY == 0:
        self._feel_acc = float(dec)
        self._feel_prev_ask = accel
      return dec
    if frame % FCA11_SEND_EVERY != 0:
      return max(1, min(int(self._feel_acc), self._dec_last_sent + HYUNDAI_FCA11_LONG_RATE_STEP))
    gain_speed = feel_gentle_gain_speed_frac(v_kph) if (self.feel_fix and v_kph is not None) else 1.0
    # 0055: the speed-indexed floor's speed input. None (unit callers with no speed) = at speed = the moving floor 4.
    v_mps = None if v_kph is None else v_kph / 3.6
    if self._feel_prev_ask is not None:
      inst = (accel - self._feel_prev_ask) / (FCA11_SEND_EVERY * 0.01)
      self._feel_slope += FCA11_FEEL_SLOPE_ALPHA * (inst - self._feel_slope)
      # 0051 (a): anticipation reads a SLOWER EMA of the same slope; the fast one above still drives the deepening bypass
      self._feel_slope_ant += FCA11_FEEL_SLOPE_ALPHA_0051 * (inst - self._feel_slope_ant)
    self._feel_prev_ask = accel
    ll_floor = self._feel_lead_loss_floor(frame) if self.feel_fix else 0.0
    if ll_floor > 0.0:   # bounded by the PRE-0049 deployed request for this ask (never more than the old law would command)
      ll_floor = min(ll_floor, float(clamp_dec_cmd(dec_cmd_from_accel(min(accel, FCA11_BRAKE_ONSET_ACCEL)), personality)))
    # 0051 (c): while a lead-loss hold is active the ask is NOT anticipated (an ask easing because the track vanished is not
    # a trend to run ahead of).
    ant_slope = self._feel_slope_ant if self.feel_fix else self._feel_slope
    a_tgt = accel if ll_floor > 0.0 else feel_anticipated_accel(accel, ant_slope)
    target = self._feel_target(a_tgt, accel, personality, gain_speed, v_mps)
    prev = self._feel_acc
    if target > prev:
      rate = feel_rate_lsb_per_sent(accel, prev)
      if self._feel_slope <= FCA11_FEEL_DEEPEN_SLOPE:
        rate = float(HYUNDAI_FCA11_LONG_RATE_STEP)   # a fast-building demand is not throttled by the comfort ramp
      new_acc = min(float(target), prev + rate)
      # 0051 (a) RE-RISE DEADBAND: after the level came down from the episode peak, a GENTLE, not-fast-deepening ask must
      # call for >= RERISE_LSB more (un-anticipated) before the level climbs again; otherwise it holds (never lower).
      # 0051 (b2): the comparison uses the RESTORE target (gain_speed), so in the restore zone the fast-rising deployed
      # level nearly always clears RERISE_LSB and the restore proceeds; at worst the deadband holds the LAST 1 LSB, which is
      # the 0049 band-entry step shape (measured: rise med/max at band entry 1/4, +4 slot 4 of 36 entries, 0 of the 25
      # scored stops - identical to 0049). It is NOT suspended there: suspending it re-exposed ask ripple at 6-8 km/h
      # (30 dips vs 28 with it active, same band-entry result) for no benefit.
      if (self.feel_fix and prev < self._feel_peak - 0.5 and accel > FCA11_FEEL_RATE_SOFT_ACCEL and
          self._feel_slope > FCA11_FEEL_DEEPEN_SLOPE and
          self._feel_target(accel, accel, personality, gain_speed, v_mps) < prev + FCA11_FEEL_RERISE_LSB):
        new_acc = prev
    else:
      new_acc = float(target)          # decreases are immediate (the safe direction)
    # 0055: the mid-episode floor is the SPEED-INDEXED one on the measured law (so a level the deadband held at 4 cannot
    # lag under a floor that has since risen to 6-8); off the measured path it stays the 0049 constant 4. Never 0: 0 is
    # the release, and a release is the caller's job (see the FCA11_MEAS_* block).
    new_acc = max(float(fca11_floor_lsb(v_mps) if self.meas_inverse else FCA11_FEEL_FLOOR_LSB), new_acc, ll_floor)
    if self.feel_fix:
      self._feel_peak = max(self._feel_peak, new_acc)
    # the panda contract caps the SENT value at +RATE_STEP over the last SENT frame (0 at an episode open -> the first
    # frame is 4). The accumulator is held to the same ceiling so it can never run AHEAD of the wire: otherwise growth
    # earned on a capped frame would be released in one +4 jump on the next (the 4 -> 8 step this layer exists to remove).
    ceiling = self._dec_last_sent + HYUNDAI_FCA11_LONG_RATE_STEP
    self._feel_acc = min(new_acc, float(ceiling))
    return max(1, min(int(new_acc), ceiling))

  # ---- 0045 SHADOW: V3 computed post-send + logged, never sent ----

  def update(self, CC: structs.CarControl, CS, frame: int, personality=PERSONALITY_STANDARD) -> list[CanData]:
    sends = self._update_law(CC, CS, frame, personality)
    if self.shadow:   # 0045 SEAM: capture the V3 inputs only; V3 runs in v3_post_send() AFTER the send, never on the wire
      self._v3_pending = (float(CC.actuators.accel), bool(CC.enabled), bool(CC.longActive), CS, int(frame), personality, tuple(sends))
    return sends

  def v3_post_send(self) -> None:
    """0045: compute V3 for the frame update() just returned (called by card.py AFTER sendcan is published, via
    GasInterceptorCarController.fca11_post_send) and write the shadow log. NEVER raises; never touches OLD-LAW state."""
    p, self._v3_pending = self._v3_pending, None
    if p is None or not self.shadow:
      return
    accel, enabled, long_active, CS, frame, personality, oldlaw_sends = p
    v3_sends = None
    if not self._v3_disabled:
      t0 = time.monotonic()
      try:
        v3_sends = self._v3_step(accel, enabled, long_active, CS, frame, personality)
      except Exception as e:  # noqa: BLE001 - V3 must never take the OLD-LAW path (or card) down
        self._v3_fault("exception", frame, f"{type(e).__name__}: {e}"[:300], 1e3 * (time.monotonic() - t0))
        v3_sends = None
      else:
        dt = time.monotonic() - t0
        if dt > FCA11_V3_BUDGET_S:
          self.v3_overruns += 1
          self._v3_overrun_streak += 1
          self._v3_fault("overrun", frame, f"V3 step {1e3 * dt:.2f} ms > budget {1e3 * FCA11_V3_BUDGET_S:.1f} ms",
                         1e3 * dt, disable=self._v3_overrun_streak >= FCA11_V3_OVERRUN_TRIP)
        else:
          self._v3_overrun_streak = 0
    try:
      self._shadow_publish(accel, enabled, long_active, CS, frame, personality, oldlaw_sends, v3_sends)
    except Exception:  # noqa: BLE001 - a log must never take card down
      pass

  def _v3_step(self, accel: float, enabled: bool, long_active: bool, CS, frame: int, personality) -> list[CanData]:
    core = self._shadow_v3
    assert core is not None
    cc = _ShadowCC(enabled, long_active, max(accel, -FCA11_A_MAX_MSS2))
    core.observe_echo(list(core._sh_prev_echo))
    v3_sends = core.update(cc, _ShadowCSView(CS), frame, personality)
    core._sh_prev_echo = [(128, bytes(s.dat)) for s in v3_sends]
    return v3_sends

  def _v3_fault(self, kind: str, frame: int, err: str, elapsed_ms: float, disable: bool = True) -> None:
    self.v3_failures += 1
    if disable:
      self._v3_disabled = True
    try:
      if self.shadow_log is not None:
        self.shadow_log.write(dict(k="v3fail", t_wall=time.time(), frame=int(frame), kind=kind, err=err,
                                   elapsed_ms=round(float(elapsed_ms), 3), n_failures=self.v3_failures,
                                   n_overruns=self.v3_overruns, v3_disabled=bool(self._v3_disabled)))
        self.shadow_log.flush()
    except Exception:  # noqa: BLE001
      pass

  def _shadow_publish(self, accel, enabled, long_active, CS, frame, personality, oldlaw_sends, v3_sends) -> None:
    accel_law = max(accel, -FCA11_A_MAX_MSS2)
    ol = [s for s in oldlaw_sends if (s[0] if isinstance(s, tuple) else s.address) == FCA11_ADDR]
    ol_lsb = int(bytes(ol[-1][1] if isinstance(ol[-1], tuple) else ol[-1].dat)[1]) if ol else 0
    inst = clamp_dec_cmd(dec_cmd_from_accel(min(accel_law, FCA11_BRAKE_ONSET_ACCEL)), personality) \
      if accel_law <= FCA11_BRAKE_ONSET_ACCEL else 0
    v3_ok = v3_sends is not None
    self.shadow_out = ShadowOutput(non_actuating=True, frame=int(frame), v3_ok=v3_ok,
                                   v3_wouldbe_lsb=(int(v3_sends[-1].dat[1]) if v3_sends else 0) if v3_ok else None,
                                   v3_send_wouldbe=bool(v3_sends), wouldbe_sends=tuple(v3_sends or ()),
                                   oldlaw_cmd_lsb=ol_lsb, oldlaw_send=bool(ol), oldlaw_instant_lsb=int(inst))
    self._shadow_record(accel, enabled, long_active, CS, frame, personality, accel_law)

  def _shadow_diag_state(self, CS) -> dict:
    """0054: the diagnostic tuple the IDLE availability trace watches (and embeds in every idle record as ``diag``).

    These are the OLD-LAW instance's own availability inputs - the ones that actually raise/clear the driver-facing
    "FCA11 Braking Unavailable" - NOT the V3 core's mirrors the frame record's ``hc_unavailable`` / ``hard_cut`` /
    ``pedal_cut`` fields carry (V3 synthesises ACCEPTED echoes, so its refusal wall can never latch: route 16a's
    ``fca11_unavailable`` was True for 170 s while those three fields read false). ``cam_quiet`` is BUCKETED
    (2 = the camera is braking, 1 = its quiet tail is still running, 0 = quiet) so the once-per-camera-frame decay can
    never trigger an edge on its own - the budget stays 2 Hz.
    """
    quiet = int(self._cam_quiet)
    try:
      gear = int(getattr(CS.out.gearShifter, "raw", CS.out.gearShifter))
    except (TypeError, ValueError):
      gear = -1
    return dict(
      fca11_unavailable=bool(self.fca11_unavailable), pedal_cut=bool(self._pedal_cut),
      refused_cut=bool(self._refused_cut), cam_brake=bool(self.cam_brake),
      cam_quiet=(0 if quiet >= FCA11_CAM_RESUME_FRAMES else (1 if quiet > 0 else 2)),
      interceptor_state=int(getattr(CS, "interceptor_state", 0) or 0),
      interceptor_fault_seen=bool(getattr(CS, "interceptor_fault_seen", False)),
      brake_pressed=bool(CS.out.brakePressed), gas_pressed=bool(CS.out.gasPressed),
      gear=gear,
    )

  def _shadow_idle_due(self, CS, frame: int, long_active: bool) -> bool:
    """0054: is an IDLE (availability-trace) frame record due on THIS controller frame?

    True while the feature is ARMED and either longitudinal is NOT active or the surfaced loss (fca11_unavailable)
    stands; then one record every FCA11_SHADOW_IDLE_FRAMES (2 Hz) and, whatever the cadence, ONE on every change of the
    _shadow_diag_state tuple. Called on EVERY frame, before the 50 Hz slot gate, so a change is logged on the tick it
    happens. Cheap on the hot path: an armed + longActive + available frame returns after one boolean expression, and
    nothing here is read by a send decision or an alert (a log must never change control).
    """
    if self.shadow_log is None or not self.shadow_log.enabled or self.shadow_log.truncated:
      return False                    # nothing can be written: skip the tuple build entirely
    if not (self.enabled and (not long_active or self.fca11_unavailable)):
      return False
    diag = self._shadow_diag_state(CS)
    changed = self._sh_diag_prev is not None and diag != self._sh_diag_prev
    self._sh_diag_prev = diag
    last = self._sh_idle_last_frame
    due = bool(changed or last is None or frame < last or (frame - last) >= FCA11_SHADOW_IDLE_FRAMES)
    if due:
      self._sh_idle_last_frame = int(frame)
    return due

  def _shadow_record(self, accel, enabled, long_active, CS, frame: int, personality, accel_law: float) -> None:
    v3 = self._shadow_v3
    assert v3 is not None
    o = self.shadow_out
    if enabled and not self._sh_engaged_prev:
      self._sh_engage_frame = frame
    if not enabled:
      self._sh_engage_frame = None
    self._sh_engaged_prev = enabled
    v3_braking = bool(o.v3_ok and v3.braking)
    v = float(CS.out.vEgo)
    cap = personality_max_dec(personality)
    if v3_braking and self._sh_ep is None:
      self._sh_episode_id += 1
      self._sh_ep = dict(episode_id=self._sh_episode_id, frame_start=frame, t_wall_start=time.time(), n_frames=0,
                         n_sent=0, v3_min=None, v3_max=0, v3_last=0, oldlaw_instant_max=0, oldlaw_cmd_max=0,
                         cap_hit_frames=0, held_band_frames=0, v_start_mps=v, v_min_mps=v, ask_min_mps2=accel_law,
                         camera_request_seen=False, tier_cap=cap)
    # 0054: the IDLE availability-trace decision is taken on EVERY frame (not only the 50 Hz slot) so a state change
    # is recorded on the tick it happens; it is inert unless the feature is armed and either longitudinal is inactive
    # or fca11_unavailable holds, and it never reaches past the shadow log.
    idle_due = self._shadow_idle_due(CS, frame, long_active)
    if frame % FCA11_SEND_EVERY != 0 and not (self._sh_ep is not None and not v3_braking) \
       and not idle_due:
      return   # frame records ride the 50 Hz send slot; an episode close is never missed
    cam = getattr(CS, "fca11_cam_frame", None)
    dat = cam[1] if cam is not None and len(cam[1]) == 8 else None
    cam_req = bool(dat is not None and camera_requesting(dat))
    ep = self._sh_ep
    if ep is not None and v3_braking:
      ep["n_frames"] += 1
      if o.v3_send_wouldbe and o.v3_wouldbe_lsb:
        ep["n_sent"] += 1
        ep["v3_min"] = o.v3_wouldbe_lsb if ep["v3_min"] is None else min(ep["v3_min"], o.v3_wouldbe_lsb)
        ep["v3_max"] = max(ep["v3_max"], o.v3_wouldbe_lsb)
        ep["v3_last"] = o.v3_wouldbe_lsb
        ep["cap_hit_frames"] += int(o.v3_wouldbe_lsb >= cap)
      ep["oldlaw_instant_max"] = max(ep["oldlaw_instant_max"], o.oldlaw_instant_lsb)
      ep["oldlaw_cmd_max"] = max(ep["oldlaw_cmd_max"], o.oldlaw_cmd_lsb)
      ep["held_band_frames"] += int(bool(v3._shape_in_hold_band))
      ep["v_min_mps"] = min(ep["v_min_mps"], v)
      ep["ask_min_mps2"] = min(ep["ask_min_mps2"], accel_law)
      ep["camera_request_seen"] = bool(ep["camera_request_seen"] or cam_req)
    why = ((v3._last_why or None) if long_active else "long_inactive") if o.v3_ok else "v3_disabled"
    lead = getattr(CS, "fca11_lead_one", None)
    lead_present = bool(lead is not None and getattr(lead, "status", False))
    lead_vr = getattr(CS, "fca11_lead_vrel_mps", None) if lead_present else None

    def _lf(name, nd):
      x = getattr(lead, name, None) if lead_present else None
      return None if x is None else round(float(x), nd)
    rec = dict(
      t_wall=time.time(), t_mono_ns=int(getattr(CS, "fca11_now_nanos", 0) or 0), frame=int(frame),
      t_since_engage=(None if self._sh_engage_frame is None else round((frame - self._sh_engage_frame) * 0.01, 2)),
      v_ego_mps=round(v, 4), a_ego_mps2=round(float(getattr(CS.out, "aEgo", 0.0) or 0.0), 4),
      ask_mps2=round(accel, 4), ask_law_mps2=round(accel_law, 4), camera_requesting=cam_req,
      camera_request_level=(int(dat[1]) if dat is not None else None),
      camera_warn=(_get_bits(dat, CAM_ACT_WARN_BITS[0], CAM_ACT_WARN_BITS[1]) if dat is not None else None),
      cam_hex=(dat.hex() if dat is not None else None),
      oldlaw_armed=bool(self.enabled), oldlaw_cmd_lsb=int(o.oldlaw_cmd_lsb), oldlaw_send=bool(o.oldlaw_send),
      oldlaw_braking=bool(self.braking), oldlaw_instant_lsb=int(o.oldlaw_instant_lsb),
      v3_ok=bool(o.v3_ok), v3_wouldbe_lsb=o.v3_wouldbe_lsb, v3_send_wouldbe=bool(o.v3_send_wouldbe),
      v3_target_lsb=((int(cap) if v3._shape_in_hold_band else
                      int(shape_dec_cmd_from_extra(max(0.0, -min(accel_law, FCA11_BRAKE_ONSET_ACCEL)), cap)))
                     if v3_braking else 0) if o.v3_ok else None,
      tier_cap=int(cap), personality=int(getattr(personality, "raw", personality)),
      engaged=bool(enabled), long_active=bool(long_active), v3_braking=v3_braking,
      held_band=bool(o.v3_ok and v3._shape_in_hold_band), latched=bool(v3.blocked_until_resume),
      hc_unavailable=bool(v3.fca11_unavailable), hard_cut=bool(v3.cam_brake), pedal_cut=bool(v3._pedal_cut),
      excl_reason=why, episode_id=(ep["episode_id"] if ep is not None else None),
      standstill=bool(getattr(CS.out, "standstill", False)), gas_pressed=bool(CS.out.gasPressed),
      brake_pressed=bool(CS.out.brakePressed),
      lead_present=lead_present, lead_drel_m=_lf("dRel", 2),
      lead_vrel_mps=(None if lead_vr is None else round(float(lead_vr), 3)),
      lead_vlead_mps=_lf("vLead", 3), lead_alead_mps2=_lf("aLeadK", 3), lead_model_prob=_lf("modelProb", 3),
      lead_radar=(bool(getattr(lead, "radar", False)) if lead_present else None),
      v3_alert_bits_wouldbe=(int(v3.supervise_stop) | (int(v3.stop_complete) << 1) | (int(v3.brake_now) << 2)) if o.v3_ok else None,
    )
    if self.shadow_log is not None:
      window = bool(v3_braking or self.braking or cam_req or (long_active and accel_law <= FCA11_BRAKE_RELEASE_ACCEL))
      if window or not idle_due:
        self.shadow_log.frame(rec, window)        # the windowed 50 Hz trace, byte-for-byte as before
      else:
        # 0054: the IDLE record - the SAME record, written outside the window, plus `idle` and the diagnostic tuple.
        self.shadow_log.frame_idle(dict(rec, idle=True, diag=self._sh_diag_prev))
    if ep is not None and not v3_braking:
      ep.update(frame_end=int(frame), t_wall_end=time.time(), duration_s=round((frame - ep["frame_start"]) * 0.01, 2),
                v_end_mps=v, end_reason=(why or "release"), v3_min=(ep["v3_min"] or 0))
      ep = {kk: (round(x, 4) if isinstance(x, float) else x) for kk, x in ep.items()}
      if self.shadow_log is not None:
        self.shadow_log.epilogue(ep)
      self._sh_ep = None

  def _update_law(self, CC: structs.CarControl, CS, frame: int, personality=PERSONALITY_STANDARD) -> list[CanData]:
    """The FCA11 law path (OLD-LAW, or V3 when affine_gain - never in Shadow). Unchanged by 0045 except its name."""
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
      # G9b/0042: and no cut/re-arm state either (a disable is a fresh start, matching the panda's init).
      self._driver_cut = False
      self._pedal_cut = False
      self._rearm_pending = False
      self.blocked_until_resume = False
      # 0042: and nothing is UNAVAILABLE while the feature is off (the surfaced flag is the OR of the cut sources).
      self.fca11_unavailable = False
      # 0042 (B): clear the echo-confirmed refusal wall too.
      self._refused_streak = 0
      self._refused_cut = False
      # 0053: the camera hand-back, the probation and the divergence guard start clean too (the panda's hyundai_init).
      self.cam_brake = False
      self._cam_quiet = FCA11_CAM_RESUME_FRAMES
      self._cam_tail = 0
      self.camera_braking_alert = False
      self._probation_left = self._probation_refused = 0
      self._probation_cut = False
      self._div_armed = self._div_latched = self._div_active = self.fca11_divergence = False
      self._div_streak = self._div_events = self._attrib_hold = 0
      self._cal_active = False
      # 0038: no gas hold-off state either.
      self._gas_release_frame = None
      self._gas_prev = False
      # 0043: and no shaped-law state survives a disable (a re-arm starts at the floor). 0043 v2: nor the cal-edge tracker.
      self._shape_reset()
      self._cal_prev = False
      self._feel_reset()                     # 0049: no feel-ramp state survives a disable
      self._feel_cal_prev = False
      self._feel_in_hold_band = False
      self._feel_dwell = 0
      self._feel_dwell_slow = 0
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
      self._start_probation()          # 0053: every (re)engage opens a refusal-cut probation window
    self._engaged_prev = engaged

    # latch the cut on what the car layer CAN observe (driver brake, gear).
    # 0038: GAS is deliberately NOT part of _driver_cut any more (owner requires a gas press never lock out future
    # braking). It is enforced per-frame in fca11_ok below plus the elapsed-frame hold-off. The DRIVER latch
    # (_driver_cut) mirrors the panda's hyundai_fca11_long_driver_cut (brake / gear != D): it re-arms on the resume
    # edge but clears only on the first clean engaged frame. A pedal FAULT goes to _pedal_cut (0042), mirroring the
    # panda's hyundai_fca11_long_cut. 0053: the camera is NOT a latch any more (cam_brake, ingest_camera).
    driver_cut = (bool(CS.out.brakePressed) or
                  CS.out.gearShifter != structs.CarState.GearShifter.drive)
    if CC.longActive and driver_cut:
      self._driver_cut = True
    elif self._rearm_pending and engaged:
      self._driver_cut = False
      self._rearm_pending = False
    cam = getattr(CS, "fca11_cam_frame", None)
    # 0053: the camera hand-back. EVERY bus-2 0x38D of this tick steps the non-latching state machine REGARDLESS of
    # longActive / engagement, exactly like the panda's hyundai_fca11_long_cam_step (the 0030 D1-a lesson: the old
    # `CC.longActive and ...` gate on drive 14d let the two layers disagree for 16 minutes). Idempotent per frame: the
    # real path already ran it in create_gas_command BEFORE observe_echo. Only the BRAKING class sets cam_brake; a
    # warning (the whole of 14d and 15b) leaves openpilot braking. While cam_brake holds, nothing of ours goes out on
    # 0x38D (see `withhold` below) and the pedal is forced to 0 (gas_interceptor.py).
    self.ingest_camera(CS, frame)
    withhold = bool(self.cam_brake or self._div_latched)
    self.camera_braking_alert = bool(self.enabled and self.cam_brake and not self._div_active)
    self.fca11_divergence = bool(self.enabled and (self._div_active or self._div_latched))
    # 0042 (A): latch the PEDAL FAULT exactly like the panda. The panda computes pedal_state = (data[4] >> 4) & 0xF
    # from ANY 0x701 frame and sets hyundai_interceptor_pedal_fault = state not in {0,4,5} (hyundai.h:962-963), which
    # hyundai_fca11_long_rx folds into hyundai_fca11_long_cut on ANY rx frame regardless of engagement/arm and only
    # hyundai_init clears it (hyundai.h:369-371). CS.interceptor_state is that same nibble (carstate_ext.py reads the
    # DBC STATE of GAS_SENSOR_R), so latching it here is EXACT panda equivalence: any frame, no longActive gate,
    # sticky against a clean pedal state and a resume edge (this instance's lifetime); the ONLY thing that clears it is the
    # proof-based heal in observe_echo (an accepted ACTUATING echo), or the end of the ignition.
    # 0042 v3: the panda faults on EVERY 0x701 frame, but the CANParser `vl` view keeps only the LAST value of the
    # window - so a single fault frame sandwiched between accepted states (route 156's boot handshake, STATE [5, 3, 5])
    # is invisible to CS.interceptor_state (its last value is an accepted 5). CS.interceptor_fault_seen carries the
    # all-frames verdict from carstate_ext.py (vl_all), so the latch is panda-equivalent on the real path too.
    pedal_state = int(getattr(CS, "interceptor_state", 0))
    if pedal_state not in HYUNDAI_FCA11_PEDAL_STATE_OK or bool(getattr(CS, "interceptor_fault_seen", False)):
      self._pedal_cut = True
    if not engaged:
      self._rearm_pending = False
    # 0042 v2: SEND GATE vs ALERT. blocked_until_resume stays the DRIVER cut ONLY (0053: the camera gate is cam_brake,
    # which releases on its own; it is not a resume-gated latch any more) - the pedal mirror
    # (_pedal_cut) must NEVER gate a send. The car-layer mirror has no per-frame ARM gate (its arm is the constant
    # `enabled`), so on route 155 it over-latches on a pre-arm STATE=1 blip the deployed panda never saw; letting
    # that fabricate a `blocked_until_resume` would invent a brake loss and suppress every send for the rest of the
    # ignition. Refused sends are inert (they never reach the car), so keeping ours LIVE preserves the ground truth
    # + the echo evidence, and there is no deadlock. _pedal_cut (and _refused_cut) only drive fca11_unavailable:
    # KNOW + ALERT (and the alert-state suppression in _alerts()), never control.
    self.blocked_until_resume = self._driver_cut
    # 0038: track the driver's gas RELEASE edge (for the elapsed-frame hold-off). CS.out.gasPressed is the driver's
    # physical pedal (the panda's gas source is the interceptor sensor, not our throttle command). On the frame the
    # pedal releases, stamp the frame; _gas_holdoff_ok refuses for GAS_HOLDOFF_FRAMES after it. No latch.
    gas_now = bool(CS.out.gasPressed)
    if self._gas_prev and not gas_now:
      self._gas_release_frame = frame
    self._gas_prev = gas_now
    # 0042: surface the fail-closed loss. _pedal_cut (pedal fault) is the hyundai_fca11_long_cut latch source and
    # _refused_cut (B) is the echo-confirmed refusal wall. 0053: these two - and ONLY these two - own the PERMANENT
    # "unavailable until restart" alert; the camera owns the transient camera_braking_alert instead.
    self.fca11_unavailable = bool(self.enabled and (self._pedal_cut or self._refused_cut))

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
      # 0043: and the shaped-law ramp state (the next episode re-opens at the floor with the onset hold armed).
      self._shape_reset()
      self._feel_reset()                     # 0049: and the feel ramp
      self._feel_cal_prev = False
      self._feel_dwell = 0
      self._feel_dwell_slow = 0
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
    # 0043 v2: reset the shaped ramp on a cal-active EDGE. While cal is active the shaped law takes the 0041-era
    # path; without this reset an episode that OPENS under cal and then continues on the planner ask would inherit
    # stale `_shape_acc`/`_shape_sent_n` (which can skip the onset hold). A cal->planner hand-off must start fresh.
    if self.affine_gain and bool(cal_on) != self._cal_prev:     # ON path only: the OFF path runs no shaped-law code here
      self._shape_reset()
      self._cal_prev = bool(cal_on)
    # 0049: the feel layer's cal edge tracker is read in the want_brake branch below; it is updated AFTER that use.

    # ---- G5 anti-flap decision ----------------------------------------------------------------
    # ask: the planner wants a brake THIS frame and every pre-condition (fca11_ok) holds. `ask` drives
    # the onset debounce only; the release/min-on logic below consumes the *raw* accel.
    # 0049 COAST-FIRST: on the OLD-LAW feel path, while moving above the hold band, a NEW episode needs the residual demand
    # beyond zero-throttle coast (open at FCA11_FEEL_OPEN_ACCEL, -0.60 by default); the release rule below is unchanged
    # (that is the hysteresis). Off the feel path / in the hold band the deployed -0.5 is used, byte-identical.
    open_accel = FCA11_BRAKE_ONSET_ACCEL
    if self.feel and not self._feel_update_band(v_kph_now, CS):
      open_accel = min(FCA11_BRAKE_ONSET_ACCEL, FCA11_FEEL_OPEN_ACCEL)
    ask = bool(ok and accel <= open_accel)
    self._feel_dwell = self._feel_dwell + 1 if (self.feel and ok and accel <= FCA11_BRAKE_ONSET_ACCEL) else 0
    _slow = bool(float(CS.out.vEgo) < FCA11_FEEL_DWELL_SLOW_MPS)
    self._feel_dwell_slow = self._feel_dwell_slow + 1 if (self.feel and ok and accel <= FCA11_BRAKE_ONSET_ACCEL and _slow) else 0
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
    if self.feel and self._feel_dwell >= FCA11_FEEL_DWELL_FRAMES:
      onset = True   # 0049 dwell escape (feel path only; `ok` is part of the dwell count): a SUSTAINED small ask opens
    if self.feel and self._feel_dwell_slow >= FCA11_FEEL_DWELL_SLOW_FRAMES:
      onset = True   # cycle 1 crawl escape: the same sustained ask opens after 0.7 s while every counted frame was slow
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
    # 0053: refusal attribution (see FCA11_ATTRIB_HOLD_FRAMES): end the episode briefly so the passive close frame is
    # the next thing the panda judges. Bounded, so it can never wedge braking.
    if self._attrib_hold > 0:
      self._attrib_hold -= 1
      want_brake = False
    # 0054: there is NO release-below-the-floor gate here. An ask softer than the ESC's weakest actuating level
    # (FCA11_MEAS_FLOOR_EXTRA) is commanded at FCA11_MEAS_FLOOR_LSB = 4, not released: the release variant was measured
    # and rejected on route 16a, where the last 2 s of all four stops asked 0.00-0.66 while the car delivered 1.19-1.71,
    # so releasing there sent nothing at exactly the asks that bring the car to standstill -- a brake loss at every stop,
    # and it disarmed the cycle-1 crawl-dwell escapes (they open on asks <= -0.5). See the FCA11_MEAS_* block.
    # 0051 (c): lead present -> absent EDGE while a feel episode is open and moving (not the hold band, not cal): freeze the
    # last SENT level as a decaying floor (see FCA11_FEEL_LEAD_LOSS_*). The edge is tracked on every frame so a lead lost
    # BEFORE an episode never arms it. Inert off the fix (CS without the lead attribute reads as no lead -> never an edge).
    if self.feel_fix:
      lead_now = getattr(CS, "fca11_lead_one", None) is not None
      if (self._feel_lead_prev and not lead_now and want_brake and self.braking and not cal_on and
          not self._feel_in_hold_band and self._dec_last_sent > 0):
        self._feel_ll_frame = self._feel_ll_hold_from = frame
        self._feel_ll_lsb = int(self._dec_last_sent)
      elif (self._feel_ll_frame is not None and not lead_now and
            (frame - self._feel_ll_frame) * 0.01 < FCA11_FEEL_LEAD_LOSS_MAX_ABSENT_S):
        self._feel_ll_hold_from = frame    # still absent (bounded): the HOLD phase is counted from the lead's return
      self._feel_lead_prev = lead_now

    dec_cmd_now = 0   # 0040: the level COMMANDED this frame (0 = no actuating frame); feeds the alerts + logger

    if want_brake:
      # 0029 CAL: latch whether THIS frame is part of a scripted hold AND the frame is being commanded
      # (ok True). This drives the gas neutralization in GasInterceptorCarController.create_gas_command.
      self._cal_active = bool(cal_on and ok)
      accel_ask = min(accel, FCA11_BRAKE_ONSET_ACCEL)
      # 0043: the OPT-IN brake-shaping law (self.affine_gain, DEFAULT OFF). On the planner-ask path it SUPERSEDES
      # the 0041 affine-only LEVEL path (and the 0.67 through-origin path): it owns its own jerk-limited slew and
      # the 4 LSB moving floor / tier-cap hold floor, so it replaces the whole request+floor+rate-limit block
      # below. cal_on still takes the 0041-era path UNCHANGED (cal is a subordinate test tool - a scripted level
      # with a clean +RATE_STEP entry step). OFF (no cal, no affine) is byte-identical to 0042.
      if self.feel and not cal_on:
        # 0049: the OLD-LAW feel layer (coast-first / lag-aware onset ramp / anticipatory release); see FCA11_FEEL_*.
        # A (re)opened episode starts at the first-frame level; a cal->planner hand-off re-seeds at the last SENT level.
        if not self.braking:
          self._feel_reset()
        elif self._feel_cal_prev:
          self._feel_reset(float(max(self._dec_last_sent, FCA11_FEEL_FLOOR_LSB)))
        dec_cmd = self._feel_level(accel, frame, personality, v_kph_now,
                                   standstill=bool(getattr(CS.out, "standstill", False)))
      elif self.affine_gain and not cal_on:
        if not self.braking:
          self._shape_reset()   # a (re)opened episode starts at the floor with the onset hold armed
        dec_cmd = self._shape_level(accel_ask, v_kph_now, CS, frame, personality)
      else:
        # Release hysteresis: inside the band [-0.50, -0.30] the command is floored at the ONSET value so
        # the ESC never sees a 0 mid-episode (a 0 would be an immediate release, defeating the hysteresis).
        # 0041: the OPT-IN affine law (self.affine_gain, DEFAULT OFF) replaces the through-origin inverse with the
        # fitted affine plant inverse (dec_cmd_from_accel_affine); OFF -> byte-identical to 0040. 0054: this branch is
        # only reached with the FEEL layer off, and 0054 rides the FEEL path, so it is byte-identical to 0040 still.
        # 0050 CAL: under cal the planner term is computed from the RAW ask, NOT the onset-clamped one. The scripted
        # level below enters as a FLOOR (`max()`), so a COASTING planner must contribute 0 -- but the -0.5 onset clamp
        # maps through the 0.67 inverse to 8, so every scripted level <= 8 became 8 on the wire (the degenerate class;
        # wsSTOP/m57 MEASUREMENT-5-7.md §0.3). A planner that IS braking (accel <= ONSET_ACCEL) is a no-op for the
        # clamp, so there the mapping is unchanged and `max()` still means "cal only ADDS braking". Gated on cal_on:
        # with cal off `_ask_for_law` IS `accel_ask`, so the planner path is byte-identical (identity.py).
        _ask_for_law = accel if cal_on else accel_ask
        if self.affine_gain:
          dec_cmd = dec_cmd_from_accel_affine(_ask_for_law)
        else:
          dec_cmd = dec_cmd_from_accel(_ask_for_law)
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
      # 0053: while the camera is braking (or the divergence latch holds) NOTHING of ours goes out on 0x38D, not even
      # the passive close frame (the panda refuses it then, and the camera's own frame is what the ESC must see). The
      # close frame stays OWED and goes out on the first send slot after the hand-back releases.
      if self._release_pending and not withhold:
        sends += self._passive_frame(CS, frame)
        if sends:  # emitted (frame % SEND_EVERY == 0); if [] we retry next frame
          self._release_pending = False
          if self._echo_wired:
            # 0030 (D2-iii): record the frame we just put on the wire and wait for the pandad echo (observe_echo):
            # refused (src 192) RE-OWES the close frame so the hand-back is retried.
            self._pending_echo = bytes(sends[-1].dat)
            self._pending_echo_frame = frame
            self._echo_rejected = False

    self._feel_cal_prev = bool(cal_on)   # 0049: previous frame's cal_on, for the feel ramp's cal->planner re-seed
    # episode age AFTER this frame's decision (consumed by `holding` on the next frame).
    self._on_frames = self._on_frames + 1 if want_brake else 0

    # 0040: alert state (over this frame's braking) and the per-episode log (one line on the true -> false edge).
    self._alerts(CS, frame, v_kph_now, dec_cmd_now)
    self._log_tick(CS, frame, dec_cmd_now)

    return sends
