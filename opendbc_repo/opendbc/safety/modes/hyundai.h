#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/hyundai_common.h"

#define HYUNDAI_LIMITS(steer, rate_up, rate_down) { \
  .max_torque = (steer), \
  .max_rate_up = (rate_up), \
  .max_rate_down = (rate_down), \
  .max_rt_delta = 112, \
  .driver_torque_allowance = 50, \
  .driver_torque_multiplier = 2, \
  .type = TorqueDriverLimited, \
   /* the EPS faults when the steering angle is above a certain threshold for too long. to prevent this, */ \
   /* we allow setting CF_Lkas_ActToi bit to 0 while maintaining the requested torque value for two consecutive frames */ \
  .min_valid_request_frames = 89, \
  .max_invalid_request_frames = 2, \
  .min_valid_request_rt_interval = 810000,  /* 810ms; a ~10% buffer on cutting every 90 frames */ \
  .has_steer_req_tolerance = true, \
}

extern const LongitudinalLimits HYUNDAI_LONG_LIMITS;
const LongitudinalLimits HYUNDAI_LONG_LIMITS = {
  .max_accel = 200,   // 1/100 m/s2
  .min_accel = -350,  // 1/100 m/s2
};

#define HYUNDAI_COMMON_TX_MSGS(scc_bus) \
  {0x340, 0,       8, .check_relay = true},   /* LKAS11 Bus 0                              */ \
  {0x4F1, scc_bus, 4, .check_relay = false},  /* CLU11 Bus 0 (radar-SCC) or 2 (camera-SCC) */ \
  {0x485, 0,       4, .check_relay = true},   /* LFAHDA_MFC Bus 0                          */ \

#define HYUNDAI_LONG_COMMON_TX_MSGS(scc_bus) \
  HYUNDAI_COMMON_TX_MSGS(scc_bus) \
  {0x420, 0,       8, .check_relay = true},   /* SCC11 Bus 0       */ \
  {0x421, 0,       8, .check_relay = true},   /* SCC12 Bus 0       */ \
  {0x50A, 0,       8, .check_relay = true},   /* SCC13 Bus 0       */ \
  {0x389, 0,       8, .check_relay = true},   /* SCC14 Bus 0       */ \
  {0x4A2, 0,       2, .check_relay = false},  /* FRT_RADAR11 Bus 0 */ \

#define HYUNDAI_COMMON_RX_CHECKS(legacy)                                                                                                                                               \
  {.msg = {{0x260, 0, 8, 100U, .max_counter = 3U, .ignore_quality_flag = true},                                                                                           \
           {0x371, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }}},                                                    \
  {.msg = {{0x386, 0, 8, 100U, .ignore_checksum = (legacy), .ignore_counter = (legacy), .max_counter = (legacy) ? 0U : 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \
  {.msg = {{0x394, 0, 8, 100U, .ignore_checksum = (legacy), .ignore_counter = (legacy), .max_counter = (legacy) ? 0U : 7U, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{0x251, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},                                              \
  {.msg = {{0x4F1, 0, 4, 50U, .ignore_checksum = true, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},                                                  \

#define HYUNDAI_SCC11_ADDR_CHECK(scc_bus)                                                                                                         \
  {.msg = {{0x420, (scc_bus), 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 50U}, { 0 }, { 0 }}}, \

#define HYUNDAI_SCC12_ADDR_CHECK(scc_bus)                                                                            \
  {.msg = {{0x421, (scc_bus), 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \

#define HYUNDAI_FCEV_GAS_ADDR_CHECK \
  {.msg = {{0x91,  0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \

#define HYUNDAI_LDA_BUTTON_ADDR_CHECK \
  {.msg = {{0x391, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 50U}, { 0 }, { 0 }}}, \

#define HYUNDAI_NON_SCC_HEV_ADDR_CHECK \
  {.msg = {{0x595U, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \

#define HYUNDAI_NON_SCC_EV_ADDR_CHECK \
  {.msg = {{0x592U, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \

// comma pedal GAS_SENSOR (0x201). Unlike upstream Toyota/Honda (.ignore_checksum/.ignore_counter), checksum AND counter are
// enforced: the pedal firmware always fills both (crc8 poly 0xD5 over bytes 0-4, 4-bit counter in byte 4), so a frozen
// (repeating), corrupted or absent pedal invalidates the RX checks and drops controls_allowed. The pedal transmits at ~49 Hz
// (TIM3: 48 MHz / 15 / 65536); a dead pedal fails the lag check within ~1 s (safety_tick), a stuck one within 5 frames.
#define HYUNDAI_GAS_INTERCEPTOR_ADDR_CHECK \
  {.msg = {{0x201, 0, 6, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \

// Same pedal, REMAPPED CAN IDs (custom pedal firmware: +0x500, GAS_SENSOR on 0x701, GAS_COMMAND on 0x700). Identical
// payload, crc8 and counter, so the identical strict check applies. Selected by HYUNDAI_PARAM_SP_GAS_INTERCEPTOR_REMAPPED.
#define HYUNDAI_GAS_INTERCEPTOR_REMAPPED_ADDR_CHECK \
  {.msg = {{0x701, 0, 6, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \

static const CanMsg HYUNDAI_TX_MSGS[] = {
  HYUNDAI_COMMON_TX_MSGS(0)
};

static bool hyundai_legacy = false;

// Must equal HYUNDAI_GAS_INTERCEPTOR_THRESHOLD in opendbc/sunnypilot/car/hyundai/gas_interceptor.py
// (a unit test enforces this). Raw GAS_SENSOR units = average of the pedal's two 12-bit ADC tracks of the DRIVER's pedal.
// The comma pedal firmware reports the driver's raw ADC on 0x201 and applies max(driver, openpilot) only at its DAC output
// (panda c076a9f2^:board/pedal/main.c pedal() + TIM3_IRQ_Handler), so openpilot's own command cannot raise this value.
// Bench-measured: foot-off rest A=465 B=241 -> average 353 (+/-12 noise). 420 = rest + 67, ~4% pedal travel, below the
// ~7% travel at which the car's own ECU flags gas pressed.
#define HYUNDAI_GAS_INTERCEPTOR_THRESHOLD 420

// FCA11 brake test (HYUNDAI_PARAM_SP_FCA11_BRAKE_TEST): max raw CR_VSM_DecCmd (0.01 g/LSB) -> 0.10 g.
// Must equal HYUNDAI_FCA11_TEST_MAX_DEC in opendbc/sunnypilot/car/hyundai/values.py (a unit test enforces this).
#define HYUNDAI_FCA11_TEST_MAX_DEC 10
// ROLLING mode only (64|128): raised cap for the dose-response (scaling) test -> 0.30 g. The parked mode (64 alone) keeps
// HYUNDAI_FCA11_TEST_MAX_DEC; with the test bits unset every decel stays blocked. Every other rolling guard (window, 1.2 s
// clock, latched cut, freshness, camera hand-back) is unchanged. Must equal HYUNDAI_FCA11_ROLL_MAX_DEC in
// opendbc/sunnypilot/car/hyundai/values.py (a unit test enforces this).
#define HYUNDAI_FCA11_ROLL_MAX_DEC 30

// FCA11 ROLLING test (HYUNDAI_PARAM_SP_FCA11_ROLLING_TEST, only on top of FCA11_BRAKE_TEST). Raw WHL_SPD11 units,
// 0.03125 km/h/LSB. Actuation is allowed only while EVERY wheel is above the floor and NO wheel is above the ceiling.
// Must equal HYUNDAI_FCA11_ROLL_MIN_SPEED_KPH / _MAX_SPEED_KPH in opendbc/sunnypilot/car/hyundai/values.py (unit tests
// pin both boundaries). Ceiling 35 km/h: the highest planned band is 20 mph (32.2 km/h). Floor 5 km/h: the runner stops
// commanding at 8 km/h (5 mph); the floor is the firmware backstop so the panda can never brake the car to a stop.
#define HYUNDAI_FCA11_ROLL_MIN_SPEED 160U   // 5 km/h
#define HYUNDAI_FCA11_ROLL_MAX_SPEED 1120U  // 35 km/h
// every cut input (wheel speeds, gear, brake, gas) must have been received within this long of each actuation frame
#define HYUNDAI_FCA11_ROLL_RX_MAX_AGE_US 100000U  // 100 ms = 10 missed frames at 100 Hz
#define HYUNDAI_FCA11_ROLL_GEAR_D 5U  // LVR12 CF_Lvr_Gear (hyundai_can.dbc VAL_ 871): 5 = D
// longest CONTINUOUS actuation (any actuating field set) before a passive frame is required again (runner pulses: <= 1.0 s)
#define HYUNDAI_FCA11_ROLL_MAX_ACT_US 1200000U
// auto hand-back: if panda has not transmitted an accepted 0x38D for this long, the camera's 0x38D is forwarded again
// (runner/USB death can never leave the ESC without FCA11, nor holding our last command)
#define HYUNDAI_FCA11_ROLL_TX_STALE_US 100000U

// the stock camera's own FCA11 (bus 2): watched so a REAL camera AEB/FCW request hands FCA11 back to the camera at once
#define HYUNDAI_FCA11_ROLL_CAMERA_ADDR_CHECK \
  {.msg = {{0x38D, 2, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \

// gear source for the rolling test only (this car: LVR12, 100 Hz on bus 0; no checksum/counter defined for it)
#define HYUNDAI_FCA11_ROLL_GEAR_ADDR_CHECK \
  {.msg = {{0x367, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \

// ******************** PRODUCTION FCA11 longitudinal braking (HYUNDAI_PARAM_SP_FCA11_LONG, bit 256) ********************
// Rides ON TOP of the gas interceptor (pedal-long), with openpilot ENGAGED. Proven on the car (roll-20261005T232357Z):
// variant B (CF_VSM_Warn=3, FCA_CmdAct=1, CF_VSM_DecCmdAct=0, CR_VSM_DecCmd=g*100, Prefill=1) braked through the ESC
// at ~31 km/h, 0.74/1.28/2.03 m/s^2 for 0.10/0.20/0.30 g (~2/3 of the command, linear); no response below ~8-10 km/h.
// Unlike the TEST bits this mode requires controls_allowed AND the openpilot heartbeat engaged, keeps the pedal
// active, and never bounds the speed from above (production braking must work at any D speed).

// max raw CR_VSM_DecCmd (0.01 g/LSB). Measured: 0.30 g -> 2.03 m/s^2 delivered (69%), ~= COMFORT_BRAKE. The car layer
// requests with a 0.67 gain correction, so a -2.0 m/s^2 accel ask maps to 0.30 g here. Must equal
// HYUNDAI_FCA11_LONG_MAX_DEC in opendbc/sunnypilot/car/hyundai/values.py (a unit test enforces this).
#define HYUNDAI_FCA11_LONG_MAX_DEC 30
// 0040: the speed FLOOR is DELETED. It was OUR assumption (never measured): we believed the ESC was inert below
// ~8-10 km/h, so we refused actuation there. A harvest of the owner's drives shows accepted production braking down
// to a 12.7 km/h plateau with NOTHING lower ever sent (OUR floor stopped it before the ESC was ever asked), so the
// belief was never tested. The owner's directive: treat FCA11 as a proper brake system, guide the car to a full stop,
// hold it, and LOG what the ESC actually does below 12 km/h and at standstill instead of pre-empting it. There is no
// lower speed bound on actuation any more, and no hold-duration cap. The bounds that REMAIN are: the 0.30 g cap
// (HYUNDAI_FCA11_LONG_MAX_DEC), the +RATE_STEP/period ramp (HYUNDAI_FCA11_LONG_RATE_STEP), the driver-cut latches
// (brake / gear != D), the camera-owns latch, the pedal-fault cut, the freshness checks, the gas per-frame window +
// hold-off, and the HOST_HB liveness watchdog - which is the REPLACEMENT for the old floor's assumed purpose (bound a
// live-but-wedged host), and is a liveness bound, never a floor. NO ceiling: production braking works at any D speed.
//
// decel RATE limit per camera period (20 ms): a frame's CR_VSM_DecCmd may exceed the last accepted one by at most
// 0.04 g -> 2.0 g/s of ramp, 0 -> 0.30 g in 150 ms. Release (smaller decel) is immediate, like the pedal slew.
#define HYUNDAI_FCA11_LONG_RATE_STEP 4
// 0035: there is NO actuation-duration budget and NO cooldown. The old 2.5 s budget / 3 s cooldown sat AT the typical
// event length, so it clipped real stops mid-brake and then refused new braking for 3 s. 0040: a braking command is NO
// LONGER kinematically self-terminating at any speed - the floor is gone, so it can now drive the car to a full stop
// and hold it there (the owner's directive); the hold ends only when the PLANNER releases it or a gate ends it (driver
// brake/gas, disengage, gear, camera, pedal fault, stale inputs, or the HOST_HB liveness watchdog). A live-but-wedged
// host is bounded by the cap, the rate limit, the stale-TX hand-back, the heartbeat, the HOST_HB watchdog, the
// driver-cut latch and the camera-owns hand-back. What the cooldown incidentally did - bound how often an episode
// can (re)open at a high level - is now done by the ONSET rule: the rate-limit reference restarts from 0 whenever the
// ESC is not currently seeing our command (no accepted frame this arm, the last accepted frame was PASSIVE, or our
// stream went stale and the camera's own FCA11 has been forwarded again), so every (re)opened episode starts at
// <= HYUNDAI_FCA11_LONG_RATE_STEP and climbs at most +RATE_STEP per accepted frame.
// freshness of every window input (wheels/gear/brake/gas), and the stale-TX hand-back, as the rolling test
#define HYUNDAI_FCA11_LONG_RX_MAX_AGE_US 100000U
#define HYUNDAI_FCA11_LONG_TX_STALE_US 100000U
#define HYUNDAI_FCA11_LONG_GEAR_D 5U  // LVR12 CF_Lvr_Gear: 5 = D (same decode as the rolling window)

// Per-arm state. Reset by every hyundai_init (each ignition / re-arm starts clean, un-cut, with no evidence yet).
enum {
  HYUNDAI_LONG_SEEN_SPEED = 1,
  HYUNDAI_LONG_SEEN_GEAR = 2,
  HYUNDAI_LONG_SEEN_BRAKE = 4,
  HYUNDAI_LONG_SEEN_GAS = 8,
  HYUNDAI_LONG_SEEN_ALL = 15,
};
static bool hyundai_fca11_long_cut;         // LATCHED: no further actuation until the next init
static uint32_t hyundai_fca11_long_seen;    // HYUNDAI_LONG_SEEN_* bits
static bool hyundai_fca11_long_gear_d;
static uint32_t hyundai_fca11_long_speed_min;  // raw, slowest wheel
static uint32_t hyundai_fca11_long_ts_speed;
static uint32_t hyundai_fca11_long_ts_gear;
static uint32_t hyundai_fca11_long_ts_brake;
static uint32_t hyundai_fca11_long_ts_gas;
static bool hyundai_fca11_long_camera_owns;   // LATCHED: camera requested actuation -> camera FCA11 forwarded, our TX blocked
static bool hyundai_fca11_long_tx_seen;       // panda has transmitted an accepted 0x38D since this arm
static uint32_t hyundai_fca11_long_ts_tx;     // ... last one at
static int hyundai_fca11_long_dec_last;       // last ACCEPTED CR_VSM_DecCmd (rate-limit reference; -1 = none this arm)
// G9 (R7-B): the DRIVER-INPUT cut (brake / gear != D) is kept in its own latch so it can re-arm on the
// openpilot resume edge (controls_allowed rising), mirroring the car layer's blocked_until_resume. The camera
// hand-back and the pedal-fault cut stay in hyundai_fca11_long_cut, latched for the whole arm (fail-closed).
// 0038: the GAS contribution to this latch is DELETED. Gas is now handled by the per-frame window check plus a
// short elapsed-time hold-off (hyundai_fca11_long_gas_holdoff_ok); see the block comment below. The latch now
// covers brake and gear != D ONLY - both are deliberate "I own this car right now" assertions, and a brake press
// already drops controls_allowed (generic_rx_checks), so the latch is mostly belt-and-braces on top of that.
static bool hyundai_fca11_long_driver_cut;    // LATCHED (brake / gear != D) until the first CLEAN frame after the next openpilot RESUME edge
static bool hyundai_fca11_long_controls_prev; // resume-edge tracker (controls_allowed prev sample)
// G9b (drive-14): the resume edge only ARMS the re-arm; the latch clears on the first CLEAN frame after the grant.
// A driver input merely HELD at the grant (the drive-14 case: engage with the foot still on the gas) therefore keeps
// the latch for that frame but stops barring actuation the moment it is actually released.
static bool hyundai_fca11_long_driver_cut_pending;

// *** 0038: gas = per-frame suppression + a short hold-off, NOT a latch ***
// The owner's requirement: hitting the gas to escape an over-brake must override the CURRENT frame (human always
// wins) but must NEVER lock out FUTURE braking events. The old rule latched a driver-cut on ANY RX frame with
// gas_pressed, cleared only by a deliberate disengage+re-engage, so a single gas tap killed braking for the rest of
// the engagement. The latch's only effect beyond the per-frame window was exactly that. It is DELETED for gas.
//
// Safety rest on: the per-frame window (`!gas_pressed`, in hyundai_fca11_long_window_ok) already refuses every
// actuating frame while the driver's physical pedal is down, so a wedged host can never brake through a gas press.
// The gas signal is the interceptor's own sensor frame (0x201/0x701 = the DRIVER's pedal, not openpilot's throttle
// command), so the gate cannot self-trigger.
//
// The hold-off is a plain elapsed-time check on the last gas_pressed sample: for a short window after the gas
// RELEASES, actuation stays refused. Purpose is ONLY to stop a 1-2 Hz brake/gas fight at the current ~3.5x
// over-delivery (braking returning the instant the pedal lifts made the driver press it again). It is NOT a latch
// and NOT a hold cap: it self-clears purely on elapsed time, needs no edge, and a NEW braking request after it is
// served normally (first frame at the onset floor, then ramping as usual).
#define HYUNDAI_FCA11_LONG_GAS_HOLDOFF_US 500000U  // 0.5 s
static uint32_t hyundai_fca11_long_gas_released_ts;  // ts of the last gas release edge
static bool hyundai_fca11_long_gas_release_seen;     // a release edge has been observed this arm (avoids the ts==0 sentinel aliasing a real release at t=0)
static bool hyundai_fca11_long_gas_prev;             // previous RX sample's gas_pressed (release-edge tracker)

// *** 0038: panda heartbeat / host-liveness watchdog (~500 ms), a LIVENESS bound, never a hold cap ***
// If the host stops streaming frames for this long, the controls loop that produces the brake command is gone;
// refuse actuation. Implemented as an additional clause in the TX hook, sampled BEFORE its own update so it
// measures the gap since the PREVIOUS host frame and self-clears the moment a fresh frame arrives (cannot wedge).
// It reuses the SAME observable the stale-TX hand-back uses - "is a stream of host frames still flowing" - but at a
// coarser bound: the hand-back (TX_STALE, 100 ms) fires only while an episode was open, so it ends an episode but
// does NOT stop a host that comes back after a gap and immediately re-demands at a high level. This watchdog refuses
// that first post-gap frame, so the 100 ms hand-back can land (camera re-owns FCA11 = the ESC sees explicit zeros)
// BEFORE any new command. It acts on ACTUATING frames only: the passive close frame (how the car layer releases)
// is never gated by it, and neither is any non-brake host frame.
#define HYUNDAI_FCA11_LONG_HOST_HB_TIMEOUT_US 500000U  // 0.5 s
static uint32_t hyundai_fca11_long_host_hb_ts;  // ts of the previous host TX hook call (0 = none yet this arm)

// *** 0038: throttle exclusivity ***
// While the host is commanding ACTUATING brake fields, strip throttle pass-through: a host frame at the pedal's
// command address (0x200/0x700) is refused unless it is a ZERO (clear) frame. This can NEVER reduce braking
// authority - it only ever rejects a THROTTLE command, and a zero/clear gas frame is always accepted. The window is
// a couple of camera periods wide so the FCA11 mirror and the GAS_COMMAND frame (separate TX-hook calls in the same
// ~20 ms tick) are caught in either order. A wedged host can therefore never brake AND throttle simultaneously.
#define HYUNDAI_FCA11_LONG_BRAKE_THROTTLE_EXCL_US 100000U  // 100 ms
static uint32_t hyundai_fca11_long_host_brake_active_ts;  // last host 0x38D frame that carried actuating brake fields (0 = none)

static void hyundai_fca11_long_reset(void) {
  hyundai_fca11_long_cut = false;
  // G9: the driver-input latch starts SET. hyundai_init also clears brake_pressed/gas_pressed, so a mid-drive
  // re-init (0xdc re-arm / pandad restart) while the driver physically holds the brake would otherwise read the
  // parsed brake "up" until the next pedal frame; the seed keeps that window closed. The edge tracker is seeded
  // from controls_allowed, the canonical "no edge until a genuine fresh sample" idiom: set_safety_hooks (the
  // function the panda's 0xdc re-arm calls) forces controls_allowed = false BEFORE the mode init runs, so at init
  // it is always false and this is exactly `controls_prev = false` today -- but writing it this way keeps the
  // "clearing driver_cut needs a genuine controls_allowed rising edge" property locally true, so it does not depend
  // on the init-time controls_allowed = false invariant holding forever.
  hyundai_fca11_long_driver_cut = true;
  hyundai_fca11_long_controls_prev = controls_allowed;
  // G9b: no resume edge has been OBSERVED yet, so no re-arm is pending. A cut carried across the re-init (driver
  // physically holding a pedal / the gear out of D) therefore stays latched, exactly like the G9 rule: only a
  // genuine controls_allowed rising edge this arm observed may arm its clearing.
  hyundai_fca11_long_driver_cut_pending = false;
  // 0038: a fresh arm starts with no gas release on record and no host-liveness evidence (host_hb_ts = 0 reads as
  // "not stale" so the FIRST brake of an engagement is never refused by the watchdog).
  hyundai_fca11_long_gas_released_ts = 0U;
  hyundai_fca11_long_gas_release_seen = false;
  hyundai_fca11_long_gas_prev = false;
  hyundai_fca11_long_host_hb_ts = 0U;
  hyundai_fca11_long_host_brake_active_ts = 0U;
  hyundai_fca11_long_seen = 0U;
  hyundai_fca11_long_gear_d = false;
  hyundai_fca11_long_speed_min = 0U;
  hyundai_fca11_long_ts_speed = 0U;
  hyundai_fca11_long_ts_gear = 0U;
  hyundai_fca11_long_ts_brake = 0U;
  hyundai_fca11_long_ts_gas = 0U;
  hyundai_fca11_long_camera_owns = false;
  hyundai_fca11_long_tx_seen = false;
  hyundai_fca11_long_ts_tx = 0U;
  hyundai_fca11_long_dec_last = -1;
}

// Called at the END of hyundai_rx_hook for valid, RX-checked frames. Records the window inputs and LATCHES the cut on:
// driver brake, driver gas, gear != D, pedal fault, or a real camera FCA11 request (which also hands FCA11 back).
// G9 (R7-B): the DRIVER-INPUT latch (brake/gas/gear) re-arms on the openpilot RESUME edge (controls_allowed rising,
// mirroring the car layer's blocked_until_resume); the camera hand-back and the pedal-fault cut stay latched for the
// whole arm (fail-closed). Any driver input still cuts the CURRENT episode immediately, and every actuating frame is
// re-checked against the live window.
static void hyundai_fca11_long_rx(const CANPacket_t *msg) {
  uint32_t ts = microsecond_timer_get();
  if ((msg->bus == 2U) && (msg->addr == 0x38DU)) {
    // the camera's OWN request: any actuation or warning field -> it is a real event, the camera gets FCA11 back for
    // the rest of this arm (latched; fwd hook stops blocking, tx hook rejects every 0x38D)
    bool cam_act = (msg->data[1] != 0U) || ((msg->data[0] & 0x1FU) != 0U) || GET_BIT(msg, 20U) || GET_BIT(msg, 21U) ||
                   GET_BIT(msg, 31U);
    if (cam_act) {
      hyundai_fca11_long_camera_owns = true;
      hyundai_fca11_long_cut = true;
    }
  }
  if (msg->bus == 0U) {
    if (msg->addr == 0x386U) {
      uint32_t fl = GET_BYTES(msg, 0, 2) & 0x3FFFU;
      uint32_t fr = GET_BYTES(msg, 2, 2) & 0x3FFFU;
      uint32_t rl = GET_BYTES(msg, 4, 2) & 0x3FFFU;
      uint32_t rr = GET_BYTES(msg, 6, 2) & 0x3FFFU;
      hyundai_fca11_long_speed_min = SAFETY_MIN(SAFETY_MIN(fl, fr), SAFETY_MIN(rl, rr));
      hyundai_fca11_long_ts_speed = ts;
      hyundai_fca11_long_seen |= HYUNDAI_LONG_SEEN_SPEED;
    }
    if (msg->addr == 0x367U) {
      hyundai_fca11_long_gear_d = (msg->data[4] & 0xFU) == HYUNDAI_FCA11_LONG_GEAR_D;  // CF_Lvr_Gear 32|4
      hyundai_fca11_long_ts_gear = ts;
      hyundai_fca11_long_seen |= HYUNDAI_LONG_SEEN_GEAR;
    }
    if (msg->addr == 0x394U) {
      hyundai_fca11_long_ts_brake = ts;
      hyundai_fca11_long_seen |= HYUNDAI_LONG_SEEN_BRAKE;
    }
    if (msg->addr == 0x260U) {
      hyundai_fca11_long_ts_gas = ts;
      hyundai_fca11_long_seen |= HYUNDAI_LONG_SEEN_GAS;
    }
  }

  // 0038: track the GAS RELEASE edge for the per-frame hold-off. gas_pressed is the driver's physical pedal
  // (interceptor 0x201/0x701). On every RX sample, if gas was pressed last sample and is now released, stamp the
  // release time; the actuating gate (hyundai_fca11_long_window_ok) refuses for GAS_HOLDOFF_US after it. This is a
  // plain elapsed-time hold-off, NOT a latch: nothing needs an edge to clear it, it just expires.
  if (hyundai_fca11_long_gas_prev && !gas_pressed) {
    hyundai_fca11_long_gas_released_ts = ts;
    hyundai_fca11_long_gas_release_seen = true;
  }
  hyundai_fca11_long_gas_prev = gas_pressed;

  // G9b (drive-14): the DRIVER-INPUT latch re-arms on the openpilot RESUME edge (controls_allowed rising, mirroring
  // the car layer's blocked_until_resume) - but only CLEARS on the first CLEAN frame after that grant. G9 cleared the
  // latch directly on the edge and then re-set it in the SAME call whenever a driver input was still held at that
  // instant, so engaging with the foot still on the gas (the drive-14 failure: gas_raw 574 > 420 for the first 80 ms
  // of a 24.7 s engagement) latched the cut for the whole episode and refused all 117 braking frames. Here the edge
  // only ARMS pending; the held input keeps the latch this frame, and the first frame with no input clears it.
  //
  // 0038: the driver-input latch covers BRAKE and GEAR != D ONLY. Gas is deliberately EXCLUDED (the owner requires
  // that a gas press never lock out future braking): it is enforced per-frame by the window (`!gas_pressed`) plus the
  // elapsed-time hold-off above. Brake and gear != D keep the G9/G9b semantics unchanged - both are deliberate "I
  // own this" assertions, and a brake press also drops controls_allowed via generic_rx_checks.
  bool controls_rising = controls_allowed && !hyundai_fca11_long_controls_prev;
  if (controls_rising) {
    hyundai_fca11_long_driver_cut_pending = true;
  }

  bool driver_cut = brake_pressed;
  if ((hyundai_fca11_long_seen & HYUNDAI_LONG_SEEN_GEAR) != 0U) {
    driver_cut |= !hyundai_fca11_long_gear_d;
  }
  if (driver_cut) {
    hyundai_fca11_long_driver_cut = true;
  } else if (hyundai_fca11_long_driver_cut_pending && controls_allowed) {
    // first CLEAN frame of the episode: the driver input that set the latch is gone, so the new engagement starts
    // fresh. Gated on controls_allowed ("after engaging"): a clean frame while disengaged does not re-arm, so a cut
    // held across a mid-drive re-init still refuses until a genuine resume edge (G9 item 2).
    hyundai_fca11_long_driver_cut = false;
    hyundai_fca11_long_driver_cut_pending = false;
  } else {
  }
  // a disengage abandons any armed-but-unused re-arm: it must be consumed by a clean frame of the SAME engagement.
  if (!controls_allowed) {
    hyundai_fca11_long_driver_cut_pending = false;
  }
  // Pedal FAULT stays hard-latched for the whole arm in hyundai_fca11_long_cut (see hyundai_rx_hook): the pedal is
  // passing through with no throttle authority, which a resume press must not paper over.
  if (hyundai_interceptor_pedal_fault) {
    hyundai_fca11_long_cut = true;
  }
  hyundai_fca11_long_controls_prev = controls_allowed;
}

static bool hyundai_fca11_long_fresh(uint32_t ts, uint32_t ts_last) {
  return safety_get_ts_elapsed(ts, ts_last) <= HYUNDAI_FCA11_LONG_RX_MAX_AGE_US;
}

// 0038: the GAS hold-off predicate for actuating frames. Refuse for GAS_HOLDOFF_US after the driver's pedal last
// released (a plain elapsed-time check on the last release sample; no edge, no latch, self-clearing). 0 = no release
// seen yet this arm -> allowed (the FIRST brake of an engagement is never held off).
static bool hyundai_fca11_long_gas_holdoff_ok(uint32_t ts) {
  return !hyundai_fca11_long_gas_release_seen ||
         (safety_get_ts_elapsed(ts, hyundai_fca11_long_gas_released_ts) >= HYUNDAI_FCA11_LONG_GAS_HOLDOFF_US);
}

// The window, evaluated for every actuating FCA11 frame in the long mode. The cut inputs are re-checked here (not
// only via the latch) so a frame can never race the latch. NOTE: the speed FLOOR is deliberately NOT a cut and NOT
// part of ok (it is still the SEEN_SPEED evidence the window needs). 0040: the floor is DELETED as a gate - the tx
// hook no longer refuses on the slowest wheel, so actuation is legal at any speed, down to and through zero. 0038:
// `!gas_pressed` (per-frame, the driver's live pedal) plus the short elapsed-time hold-off are the ONLY gas barriers;
// there is no gas latch.
static bool hyundai_fca11_long_window_ok(void) {
  uint32_t ts = microsecond_timer_get();
  bool ok = (hyundai_fca11_long_seen == (uint32_t)HYUNDAI_LONG_SEEN_ALL);
  ok = ok && hyundai_fca11_long_fresh(ts, hyundai_fca11_long_ts_speed) && hyundai_fca11_long_fresh(ts, hyundai_fca11_long_ts_gear) &&
       hyundai_fca11_long_fresh(ts, hyundai_fca11_long_ts_brake) && hyundai_fca11_long_fresh(ts, hyundai_fca11_long_ts_gas);
  ok = ok && !hyundai_fca11_long_cut && !hyundai_fca11_long_driver_cut && hyundai_fca11_long_gear_d && !brake_pressed && !gas_pressed;
  ok = ok && hyundai_fca11_long_gas_holdoff_ok(ts);
  return ok;
}

// Per-arm rolling-test state. Reset by every hyundai_init (each 0xdc re-arms clean, un-cut, with no evidence yet).
enum {
  HYUNDAI_ROLL_SEEN_SPEED = 1,
  HYUNDAI_ROLL_SEEN_GEAR = 2,
  HYUNDAI_ROLL_SEEN_BRAKE = 4,
  HYUNDAI_ROLL_SEEN_GAS = 8,
  HYUNDAI_ROLL_SEEN_ALL = 15,
};
static bool hyundai_fca11_roll_cut;        // LATCHED: no further actuation until the next init
static uint32_t hyundai_fca11_roll_seen;   // HYUNDAI_ROLL_SEEN_* bits
static bool hyundai_fca11_roll_gear_d;
static uint32_t hyundai_fca11_roll_speed_min;  // raw, slowest wheel
static uint32_t hyundai_fca11_roll_speed_max;  // raw, fastest wheel
static uint32_t hyundai_fca11_roll_ts_speed;
static uint32_t hyundai_fca11_roll_ts_gear;
static uint32_t hyundai_fca11_roll_ts_brake;
static uint32_t hyundai_fca11_roll_ts_gas;
static bool hyundai_fca11_roll_camera_owns;  // LATCHED: camera requested actuation -> camera FCA11 forwarded, our TX blocked
static bool hyundai_fca11_roll_tx_seen;      // panda has transmitted an accepted 0x38D since this arm
static uint32_t hyundai_fca11_roll_ts_tx;    // ... last one at
static bool hyundai_fca11_roll_act_active;   // inside a continuous actuation
static uint32_t hyundai_fca11_roll_ts_act;   // ... which started at

static void hyundai_fca11_rolling_reset(void) {
  hyundai_fca11_roll_cut = false;
  hyundai_fca11_roll_seen = 0U;
  hyundai_fca11_roll_gear_d = false;
  hyundai_fca11_roll_speed_min = 0U;
  hyundai_fca11_roll_speed_max = 0U;
  hyundai_fca11_roll_ts_speed = 0U;
  hyundai_fca11_roll_ts_gear = 0U;
  hyundai_fca11_roll_ts_brake = 0U;
  hyundai_fca11_roll_ts_gas = 0U;
  hyundai_fca11_roll_camera_owns = false;
  hyundai_fca11_roll_tx_seen = false;
  hyundai_fca11_roll_ts_tx = 0U;
  hyundai_fca11_roll_act_active = false;
  hyundai_fca11_roll_ts_act = 0U;
}

// Called at the END of hyundai_rx_hook (after brake_pressed / gas_pressed were updated from this frame) for valid,
// RX-checked bus-0 frames. Records the window inputs and LATCHES the cut on: driver brake, driver gas, gear != D,
// any wheel above the ceiling, or the slowest wheel at/below the floor. A cut never clears inside the armed session.
static void hyundai_fca11_rolling_rx(const CANPacket_t *msg) {
  uint32_t ts = microsecond_timer_get();
  if ((msg->bus == 2U) && (msg->addr == 0x38DU)) {
    // the camera's OWN request: any actuation or warning field -> it is a real event, the camera gets FCA11 back for
    // the rest of this arm (latched; fwd hook stops blocking, tx hook rejects every 0x38D)
    bool cam_act = (msg->data[1] != 0U) || ((msg->data[0] & 0x1FU) != 0U) || GET_BIT(msg, 20U) || GET_BIT(msg, 21U) ||
                   GET_BIT(msg, 31U);
    if (cam_act) {
      hyundai_fca11_roll_camera_owns = true;
      hyundai_fca11_roll_cut = true;
    }
  }
  if (msg->bus == 0U) {
    if (msg->addr == 0x386U) {
      uint32_t fl = GET_BYTES(msg, 0, 2) & 0x3FFFU;
      uint32_t fr = GET_BYTES(msg, 2, 2) & 0x3FFFU;
      uint32_t rl = GET_BYTES(msg, 4, 2) & 0x3FFFU;
      uint32_t rr = GET_BYTES(msg, 6, 2) & 0x3FFFU;
      hyundai_fca11_roll_speed_min = SAFETY_MIN(SAFETY_MIN(fl, fr), SAFETY_MIN(rl, rr));
      hyundai_fca11_roll_speed_max = SAFETY_MAX(SAFETY_MAX(fl, fr), SAFETY_MAX(rl, rr));
      hyundai_fca11_roll_ts_speed = ts;
      hyundai_fca11_roll_seen |= HYUNDAI_ROLL_SEEN_SPEED;
    }
    if (msg->addr == 0x367U) {
      hyundai_fca11_roll_gear_d = (msg->data[4] & 0xFU) == HYUNDAI_FCA11_ROLL_GEAR_D;  // CF_Lvr_Gear 32|4
      hyundai_fca11_roll_ts_gear = ts;
      hyundai_fca11_roll_seen |= HYUNDAI_ROLL_SEEN_GEAR;
    }
    if (msg->addr == 0x394U) {
      hyundai_fca11_roll_ts_brake = ts;
      hyundai_fca11_roll_seen |= HYUNDAI_ROLL_SEEN_BRAKE;
    }
    if (msg->addr == 0x260U) {
      hyundai_fca11_roll_ts_gas = ts;
      hyundai_fca11_roll_seen |= HYUNDAI_ROLL_SEEN_GAS;
    }
  }

  bool cut = brake_pressed || gas_pressed;
  if ((hyundai_fca11_roll_seen & HYUNDAI_ROLL_SEEN_GEAR) != 0U) {
    cut |= !hyundai_fca11_roll_gear_d;
  }
  if ((hyundai_fca11_roll_seen & HYUNDAI_ROLL_SEEN_SPEED) != 0U) {
    cut |= hyundai_fca11_roll_speed_max > HYUNDAI_FCA11_ROLL_MAX_SPEED;
    cut |= hyundai_fca11_roll_speed_min <= HYUNDAI_FCA11_ROLL_MIN_SPEED;
  }
  if (cut) {
    hyundai_fca11_roll_cut = true;
  }
}

// ******************** TEST-ONLY parked LKAS11 steering sweep (HYUNDAI_PARAM_SP_LKAS_PARK_TEST, bit 512) ********************
// Armed only by car-features/steer-test/steer_park_test.py (openpilot stopped, raw 0xdf + 0xdc). openpilot's own lateral
// can never steer here (controlsd gates latActive below 0.3 m/s; Hyundai has no steerAtStandstill), so the runner sends
// LKAS11 itself and panda is the ONLY enforcer. TX list = LKAS11 alone. Every ACTUATING frame (torque != 0 or
// CF_Lkas_ActToi = 1) needs the parked window; a PASSIVE frame (torque 0, ActToi 0) is the clean idle shape and is
// accepted until the per-arm time cap, so the runner can release without a source switch. Constants that the runner
// mirrors must equal the HYUNDAI_LKAS_PARK_* values in opendbc/sunnypilot/car/hyundai/values.py (unit tests pin them).
#define HYUNDAI_LKAS_PARK_MAX_TORQUE 384       // = STEER_MAX (CarControllerParams), never more
#define HYUNDAI_LKAS_PARK_RATE_UP 3            // per accepted frame, magnitude growth (= STEER_DELTA_UP)
#define HYUNDAI_LKAS_PARK_RATE_DOWN 7          // per accepted frame, magnitude decay (= STEER_DELTA_DOWN)
#define HYUNDAI_LKAS_PARK_RT_DELTA 112         // real-time bound per 250 ms, same as the normal Hyundai limits
#define HYUNDAI_LKAS_PARK_RT_INTERVAL_US 250000U
#define HYUNDAI_LKAS_PARK_MAX_ARM_US 60000000U // hard cap: 60 s after the arm (init); then EVERY 0x340 is refused
#define HYUNDAI_LKAS_PARK_RX_MAX_AGE_US 100000U  // every window input fresh within 100 ms of each actuating frame
#define HYUNDAI_LKAS_PARK_TX_STALE_US 100000U  // camera LKAS11 forwarded again 100 ms after our last accepted frame
// 5.0 km/h per wheel (raw WHL_SPD11, 0.03125 km/h/LSB). Was 12U = 0.375 km/h (STANDSTILL_THRSLD): on-car, steering
// torque at standstill scrubs the front tire and WHL_SPD11 spikes to 0.47-1.66 km/h, so the old standstill gate refused
// frames and latched the cut. Owner-authorized 2026-10-06 to relax it for the parked sweep; Park pawl + handbrake are
// the compensating control. Still far below any real movement.
#define HYUNDAI_LKAS_PARK_WHEEL_MAX 160U
#define HYUNDAI_LKAS_PARK_MAX_ANGLE 850        // raw SAS_Angle (0.1 deg): |angle| > 85 deg latches the cut
#define HYUNDAI_LKAS_PARK_MAX_DRIVER_TQ 500    // raw CR_Mdps_StrTq (0.01 Nm): |tq| > 5.00 Nm latches the cut
#define HYUNDAI_LKAS_PARK_GEAR_P 0U            // LVR12 CF_Lvr_Gear
#define HYUNDAI_LKAS_PARK_GEAR_N 6U

// SAS11 is 6 bytes on this car (route 131: 35868/35868 frames; ~600k across routes 131/133/134, all 6 bytes), NOT the
// DBC's 5: a 5-byte check never matches
#define HYUNDAI_LKAS_PARK_SAS_ADDR_CHECK \
  {.msg = {{0x2B0, 0, 6, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \

enum {
  HYUNDAI_PARK_SEEN_MDPS = 1,
  HYUNDAI_PARK_SEEN_SPEED = 2,
  HYUNDAI_PARK_SEEN_GEAR = 4,
  HYUNDAI_PARK_SEEN_ANGLE = 8,
  HYUNDAI_PARK_SEEN_GAS = 16,
  HYUNDAI_PARK_SEEN_ALL = 31,
};
static bool hyundai_lkas_park_cut;             // LATCHED: only passive frames until the next init (re-arm)
static uint32_t hyundai_lkas_park_seen;        // HYUNDAI_PARK_SEEN_* since this arm
static uint32_t hyundai_lkas_park_ts_arm;      // init time (the 60 s cap runs from here)
static uint32_t hyundai_lkas_park_ts_mdps;
static uint32_t hyundai_lkas_park_ts_speed;
static uint32_t hyundai_lkas_park_ts_gear;
static uint32_t hyundai_lkas_park_ts_angle;
static uint32_t hyundai_lkas_park_ts_gas;
static uint32_t hyundai_lkas_park_gear_ref;    // gear at the first LVR12 of this arm (must be P or N, must not change)
static uint32_t hyundai_lkas_park_gear;        // latest LVR12 gear
static uint32_t hyundai_lkas_park_wheel_max;   // latest WHL_SPD11, fastest wheel (raw)
static bool hyundai_lkas_park_tx_seen;         // an accepted 0x340 since this arm ...
static uint32_t hyundai_lkas_park_ts_tx;       // ... last one at
static int hyundai_lkas_park_torque_last;      // last ACCEPTED torque (rate reference)
static int hyundai_lkas_park_rt_last;          // real-time reference torque
static uint32_t hyundai_lkas_park_ts_rt;       // ... set at

static void hyundai_lkas_park_reset(void) {
  hyundai_lkas_park_cut = false;
  hyundai_lkas_park_seen = 0U;
  hyundai_lkas_park_ts_arm = microsecond_timer_get();
  hyundai_lkas_park_ts_mdps = 0U;
  hyundai_lkas_park_ts_speed = 0U;
  hyundai_lkas_park_ts_gear = 0U;
  hyundai_lkas_park_ts_angle = 0U;
  hyundai_lkas_park_ts_gas = 0U;
  hyundai_lkas_park_gear_ref = 0xFFU;
  hyundai_lkas_park_gear = 0xFFU;
  hyundai_lkas_park_wheel_max = 0xFFFFU;
  hyundai_lkas_park_tx_seen = false;
  hyundai_lkas_park_ts_tx = 0U;
  hyundai_lkas_park_torque_last = 0;
  hyundai_lkas_park_rt_last = 0;
  hyundai_lkas_park_ts_rt = hyundai_lkas_park_ts_arm;
}

static bool hyundai_lkas_park_capped(uint32_t ts) {
  return safety_get_ts_elapsed(ts, hyundai_lkas_park_ts_arm) > HYUNDAI_LKAS_PARK_MAX_ARM_US;
}

// Called at the END of hyundai_rx_hook for valid, RX-checked bus-0 frames while armed. Records the window inputs and
// LATCHES the cut on: any MDPS12 fault/unavailable bit (Def, ToiUnavail, ToiFlt, FailStat, SErr), |driver torque| over
// the limit, any wheel above standstill, a gear that is not P/N or differs from the arm's first gear, |angle| over the
// limit, or the driver's gas. A cut never clears inside the armed session (re-arm = the next init).
static void hyundai_lkas_park_rx(const CANPacket_t *msg) {
  uint32_t ts = microsecond_timer_get();
  bool cut = false;
  if (msg->bus == 0U) {
    if (msg->addr == 0x251U) {
      bool fault = GET_BIT(msg, 11U) || GET_BIT(msg, 12U) || GET_BIT(msg, 14U) || GET_BIT(msg, 15U) || GET_BIT(msg, 37U);
      uint32_t str_tq_raw = GET_BYTES(msg, 5, 2) & 0xFFFU;  // CR_Mdps_StrTq 40|12, 0.01 Nm, offset -20.48
      int str_tq = (int)str_tq_raw - 2048;
      cut = fault || (SAFETY_ABS(str_tq) > HYUNDAI_LKAS_PARK_MAX_DRIVER_TQ);
      hyundai_lkas_park_ts_mdps = ts;
      hyundai_lkas_park_seen |= HYUNDAI_PARK_SEEN_MDPS;
    }
    if (msg->addr == 0x386U) {
      uint32_t fl = GET_BYTES(msg, 0, 2) & 0x3FFFU;
      uint32_t fr = GET_BYTES(msg, 2, 2) & 0x3FFFU;
      uint32_t rl = GET_BYTES(msg, 4, 2) & 0x3FFFU;
      uint32_t rr = GET_BYTES(msg, 6, 2) & 0x3FFFU;
      hyundai_lkas_park_wheel_max = SAFETY_MAX(SAFETY_MAX(fl, fr), SAFETY_MAX(rl, rr));
      cut = cut || (hyundai_lkas_park_wheel_max > HYUNDAI_LKAS_PARK_WHEEL_MAX);
      hyundai_lkas_park_ts_speed = ts;
      hyundai_lkas_park_seen |= HYUNDAI_PARK_SEEN_SPEED;
    }
    if (msg->addr == 0x367U) {
      uint32_t gear = msg->data[4] & 0xFU;  // CF_Lvr_Gear 32|4
      if (hyundai_lkas_park_gear_ref == 0xFFU) {
        hyundai_lkas_park_gear_ref = gear;
      }
      hyundai_lkas_park_gear = gear;
      cut = cut || ((gear != HYUNDAI_LKAS_PARK_GEAR_P) && (gear != HYUNDAI_LKAS_PARK_GEAR_N)) || (gear != hyundai_lkas_park_gear_ref);
      hyundai_lkas_park_ts_gear = ts;
      hyundai_lkas_park_seen |= HYUNDAI_PARK_SEEN_GEAR;
    }
    if (msg->addr == 0x2B0U) {
      int angle = (int)(int16_t)(uint16_t)GET_BYTES(msg, 0, 2);  // SAS_Angle 0|16 signed, 0.1 deg
      cut = cut || (SAFETY_ABS(angle) > HYUNDAI_LKAS_PARK_MAX_ANGLE);
      hyundai_lkas_park_ts_angle = ts;
      hyundai_lkas_park_seen |= HYUNDAI_PARK_SEEN_ANGLE;
    }
    if (msg->addr == 0x260U) {
      hyundai_lkas_park_ts_gas = ts;
      hyundai_lkas_park_seen |= HYUNDAI_PARK_SEEN_GAS;
    }
  }
  if (cut || gas_pressed) {
    hyundai_lkas_park_cut = true;
  }
}

static bool hyundai_lkas_park_fresh(uint32_t ts, uint32_t ts_last) {
  return safety_get_ts_elapsed(ts, ts_last) <= HYUNDAI_LKAS_PARK_RX_MAX_AGE_US;
}

// The parked window for every actuating frame: every input seen this arm and fresh, not cut. The cut conditions
// themselves are latched at RX time (a frame can never race them: rx and tx hooks run in the same context).
static bool hyundai_lkas_park_window_ok(uint32_t ts) {
  bool ok = (hyundai_lkas_park_seen == (uint32_t)HYUNDAI_PARK_SEEN_ALL) && !hyundai_lkas_park_cut;
  ok = ok && hyundai_lkas_park_fresh(ts, hyundai_lkas_park_ts_mdps) && hyundai_lkas_park_fresh(ts, hyundai_lkas_park_ts_speed);
  ok = ok && hyundai_lkas_park_fresh(ts, hyundai_lkas_park_ts_gear) && hyundai_lkas_park_fresh(ts, hyundai_lkas_park_ts_angle);
  ok = ok && hyundai_lkas_park_fresh(ts, hyundai_lkas_park_ts_gas) && !gas_pressed;
  return ok;
}

// Rate law of opendbc apply_driver_steer_torque_limits (no driver term): growth <= +3, decay <= 7 per accepted frame,
// zero crossing in at most 3 per frame; plus the normal real-time bound.
static bool hyundai_lkas_park_rate_ok(int torque, uint32_t ts) {
  int last = hyundai_lkas_park_torque_last;
  int hi;
  int lo;
  if (last > 0) {
    hi = last + HYUNDAI_LKAS_PARK_RATE_UP;
    lo = SAFETY_MAX(last - HYUNDAI_LKAS_PARK_RATE_DOWN, -HYUNDAI_LKAS_PARK_RATE_UP);
  } else {
    hi = SAFETY_MIN(last + HYUNDAI_LKAS_PARK_RATE_DOWN, HYUNDAI_LKAS_PARK_RATE_UP);
    lo = last - HYUNDAI_LKAS_PARK_RATE_UP;
  }
  bool ok = (torque <= hi) && (torque >= lo);
  if (safety_get_ts_elapsed(ts, hyundai_lkas_park_ts_rt) > HYUNDAI_LKAS_PARK_RT_INTERVAL_US) {
    hyundai_lkas_park_rt_last = last;
    hyundai_lkas_park_ts_rt = ts;
  }
  int rt_hi = SAFETY_MAX(hyundai_lkas_park_rt_last, 0) + HYUNDAI_LKAS_PARK_RT_DELTA;
  int rt_lo = SAFETY_MIN(hyundai_lkas_park_rt_last, 0) - HYUNDAI_LKAS_PARK_RT_DELTA;
  ok = ok && (torque <= rt_hi) && (torque >= rt_lo);
  return ok;
}

// Parked right now (EVERY frame, passive included): wheel + gear seen this arm and fresh, the latest wheel sample at
// standstill, the latest gear P/N and equal to the arm's first gear. Not latched (the cut latch only gates actuation),
// so after a firmware cut the runner can keep the idle stream going until it disarms - no LKAS11 gap.
static bool hyundai_lkas_park_parked(uint32_t ts) {
  const uint32_t need = (uint32_t)HYUNDAI_PARK_SEEN_SPEED | (uint32_t)HYUNDAI_PARK_SEEN_GEAR;
  bool ok = ((hyundai_lkas_park_seen & need) == need);
  ok = ok && hyundai_lkas_park_fresh(ts, hyundai_lkas_park_ts_speed) && hyundai_lkas_park_fresh(ts, hyundai_lkas_park_ts_gear);
  ok = ok && (hyundai_lkas_park_wheel_max <= HYUNDAI_LKAS_PARK_WHEEL_MAX);
  ok = ok && ((hyundai_lkas_park_gear == HYUNDAI_LKAS_PARK_GEAR_P) || (hyundai_lkas_park_gear == HYUNDAI_LKAS_PARK_GEAR_N));
  ok = ok && (hyundai_lkas_park_gear == hyundai_lkas_park_gear_ref);
  return ok;
}

// LKAS11 TX rule in the parked test mode. PASSIVE frame = torque 0 and CF_Lkas_ActToi 0 (the idle shape openpilot
// itself sends when not steering): needs only the parked state and the time cap, and is exempt from the rate law (an
// immediate release to zero is the fail-safe direction - exactly what openpilot does on any disengage). ACTUATING
// frame (torque != 0 or ActToi = 1): additionally the full window (no latched cut, every input seen and fresh, no gas),
// |torque| <= 384, ActToi set whenever torque != 0, and the +3/-7 rate law plus the real-time bound.
static bool hyundai_lkas_park_tx(const CANPacket_t *msg) {
  uint32_t ts = microsecond_timer_get();
  uint32_t torque_raw = (GET_BYTES(msg, 0, 4) >> 16) & 0x7FFU;  // CR_Lkas_StrToqReq 16|11, offset -1024
  int torque = (int)torque_raw - 1024;
  bool steer_req = GET_BIT(msg, 27U);                                 // CF_Lkas_ActToi
  bool actuation = (torque != 0) || steer_req;

  bool violation = hyundai_lkas_park_capped(ts) || !hyundai_lkas_park_parked(ts);
  if (actuation && !violation) {
    violation = (SAFETY_ABS(torque) > HYUNDAI_LKAS_PARK_MAX_TORQUE) || ((torque != 0) && !steer_req);
    violation = violation || !hyundai_lkas_park_window_ok(ts);
    violation = violation || !hyundai_lkas_park_rate_ok(torque, ts);
  }
  if (!violation && actuation) {
    hyundai_lkas_park_torque_last = torque;
  } else {
    // passive frame, or ANY rejected frame: the rate reference drops to zero (as steer_torque_cmd_checks does on a
    // violation), so after a refusal the runner must ramp up from zero again - never resume from the old level
    hyundai_lkas_park_torque_last = 0;
    hyundai_lkas_park_rt_last = 0;
    hyundai_lkas_park_ts_rt = ts;
  }
  if (!violation) {
    hyundai_lkas_park_tx_seen = true;
    hyundai_lkas_park_ts_tx = ts;
  }
  return !violation;
}

// Rolling mode only: the camera's 0x38D (bus 2 -> 0) is blocked ONLY while panda is actively transmitting it (an accepted
// 0x38D within HYUNDAI_FCA11_ROLL_TX_STALE_US) and the camera has not requested actuation. Before our first frame, after
// we stop, or after a real camera request, the camera's own FCA11 reaches the ESC again within one camera period.
static bool hyundai_fwd_hook(int bus_num, int addr) {
  bool block = false;
  if (hyundai_fca11_rolling_test && (bus_num == 2) && (addr == 0x38D)) {
    block = hyundai_fca11_roll_tx_seen && !hyundai_fca11_roll_camera_owns &&
            (safety_get_ts_elapsed(microsecond_timer_get(), hyundai_fca11_roll_ts_tx) <= HYUNDAI_FCA11_ROLL_TX_STALE_US);
  }
  if (hyundai_fca11_long && (bus_num == 2) && (addr == 0x38D)) {
    // same dynamic-blocking model as the rolling test: while openpilot is braking it MIRRORS the camera's 0x38D on bus 0
    // (only the brake fields overridden), so the camera's own copy must not also reach the ESC (two FCA11 sources); the
    // block exists ONLY while panda transmitted an accepted 0x38D recently (<= TX_STALE) and the camera has not made its
    // own request. Host death / stale stream / a real camera request -> the camera's FCA11 forwards again within one
    // camera period (~20 ms), the same fail-open the test mode proved on the car.
    block = hyundai_fca11_long_tx_seen && !hyundai_fca11_long_camera_owns &&
            (safety_get_ts_elapsed(microsecond_timer_get(), hyundai_fca11_long_ts_tx) <= HYUNDAI_FCA11_LONG_TX_STALE_US);
  }
  if (hyundai_lkas_park_test && (bus_num == 2) && (addr == 0x340)) {
    // parked steering test: the camera's LKAS11 is blocked ONLY while panda transmitted an accepted 0x340 within
    // TX_STALE (never two LKAS11 sources at the MDPS) AND panda would still accept our next frame (parked, inside the
    // cap). Before our first frame and after runner death / disarm the camera is back within ~100 ms; the moment the
    // car leaves park (gear, wheel, stale input) or the 60 s cap expires - i.e. the moment panda starts REFUSING our
    // frames - the camera is forwarded on its very next frame, so the MDPS is never left without LKAS11.
    //
    // INVARIANT: this block deliberately mirrors the PASSIVE TX-accept predicate (parked && !capped), NOT the actuating
    // one (which additionally needs window_ok / !cut). The runner always maintains a passive floor (torque 0 / ActToi 0
    // frames during the run and for the 0.3 s release tail after any abort, until it disarms), and a latched cut bars
    // ACTUATING frames only - a passive frame is still accepted while parked and inside the cap. So "panda is taking our
    // frames right now" == parked && !capped, which is exactly when the camera must stay off. Adding !cut / window_ok
    // here would clear the block the instant a cut latches while the runner is still flowing ACCEPTED passive frames,
    // forwarding the camera alongside us: two LKAS11 sources at the MDPS for up to the release tail, the exact hazard
    // this mode forbids (harness invariant I4). The cost of keeping it is bounded and benign: while a cut is latched the
    // MDPS sees a <= ~20 ms LKAS11 gap (our actuating frame refused, the runner reflows accepted passive frames on its
    // next tick; the runner's own rejected-frame abort fires within that tick) - zero-source at standstill, well under
    // the harness's 50 ms gap allowance. Camera resume stays guaranteed by !parked, !capped and TX_STALE, so the block
    // can never wedge. Keep I4 in mind: it holds only while the runner's max inter-frame gap stays < TX_STALE (100 ms);
    // if the fallback latency ever drifts past that, I4 breaks even with this predicate.
    uint32_t now = microsecond_timer_get();
    block = hyundai_lkas_park_tx_seen &&
            (safety_get_ts_elapsed(now, hyundai_lkas_park_ts_tx) <= HYUNDAI_LKAS_PARK_TX_STALE_US) &&
            !hyundai_lkas_park_capped(now) && hyundai_lkas_park_parked(now);
  }
  return block;
}

static bool hyundai_fca11_roll_fresh(uint32_t ts, uint32_t ts_last) {
  return safety_get_ts_elapsed(ts, ts_last) <= HYUNDAI_FCA11_ROLL_RX_MAX_AGE_US;
}

// The rolling window, evaluated for every actuating FCA11 frame. Everything must hold; the cut inputs are also
// re-checked here (not only via the latch) so a frame can never race the latch.
static bool hyundai_fca11_rolling_window_ok(void) {
  uint32_t ts = microsecond_timer_get();
  bool ok = (hyundai_fca11_roll_seen == (uint32_t)HYUNDAI_ROLL_SEEN_ALL);
  ok = ok && hyundai_fca11_roll_fresh(ts, hyundai_fca11_roll_ts_speed) && hyundai_fca11_roll_fresh(ts, hyundai_fca11_roll_ts_gear) &&
       hyundai_fca11_roll_fresh(ts, hyundai_fca11_roll_ts_brake) && hyundai_fca11_roll_fresh(ts, hyundai_fca11_roll_ts_gas);
  ok = ok && !hyundai_fca11_roll_cut && hyundai_fca11_roll_gear_d && !brake_pressed && !gas_pressed;
  ok = ok && (hyundai_fca11_roll_speed_min > HYUNDAI_FCA11_ROLL_MIN_SPEED) && (hyundai_fca11_roll_speed_max <= HYUNDAI_FCA11_ROLL_MAX_SPEED);
  return ok;
}

// Ceiling on openpilot's pedal command, raw DAC counts per track. Must equal the packed GAS_COMMAND / GAS_COMMAND2 at
// MAX_INTERCEPTOR_GAS (0.35 of pedal travel) in opendbc/sunnypilot/car/hyundai/gas_interceptor.py (a unit test pins
// both against the packer). Bench scaling: rest A=465/B=241, full A=2616/B=1284, so 0.35 -> A 1218 / B 612
// (B on the measured line B = 0.494*A + 10.5). Without this, any value up to full throttle passed as long as
// longitudinal was allowed.
#define HYUNDAI_GAS_INTERCEPTOR_MAX_GAS_A 1218U
#define HYUNDAI_GAS_INTERCEPTOR_MAX_GAS_B 612U

static int hyundai_get_interceptor_gas(const CANPacket_t *msg) {
  uint16_t val1 = (uint16_t)((uint16_t)msg->data[0] << 8U) | (uint16_t)msg->data[1];
  uint16_t val2 = (uint16_t)((uint16_t)msg->data[2] << 8U) | (uint16_t)msg->data[3];
  uint16_t avg  = (uint16_t)((val1 + val2) / 2U);

  return (int)avg;
}

// comma pedal checksum: crc8, poly 0xD5, init 0xFF, over bytes 4..0 (processed last byte first), stored in byte 5
static uint8_t hyundai_interceptor_crc8(const CANPacket_t *msg) {
  uint8_t crc = 0xFFU;
  for (int i = 4; i >= 0; i--) {
    crc ^= msg->data[i];
    for (int j = 0; j < 8; j++) {
      if ((crc & 0x80U) != 0U) {
        crc = (uint8_t)((crc << 1) ^ 0xD5U);
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// active pedal dialect (see hyundai_gas_interceptor_remapped in hyundai_common.h)
static uint32_t hyundai_interceptor_sensor_addr(void) {
  return hyundai_gas_interceptor_remapped ? 0x701U : 0x201U;
}

static uint32_t hyundai_interceptor_command_addr(void) {
  return hyundai_gas_interceptor_remapped ? 0x700U : 0x200U;
}

// checksum/counter hooks only run for addresses in the active RX checks, of which at most one pedal sensor is present
static bool hyundai_is_interceptor_sensor(const CANPacket_t *msg) {
  return (msg->addr == 0x201U) || (msg->addr == 0x701U);
}

static uint8_t hyundai_get_counter(const CANPacket_t *msg) {

  uint8_t cnt = 0;
  if (msg->addr == 0x260U) {
    cnt = (msg->data[7] >> 4) & 0x3U;
  } else if (msg->addr == 0x386U) {
    cnt = ((msg->data[3] >> 6) << 2) | (msg->data[1] >> 6);
  } else if (msg->addr == 0x394U) {
    cnt = (msg->data[1] >> 5) & 0x7U;
  } else if (msg->addr == 0x421U) {
    cnt = msg->data[7] & 0xFU;
  } else if (msg->addr == 0x4F1U) {
    cnt = (msg->data[3] >> 4) & 0xFU;
  } else if (hyundai_is_interceptor_sensor(msg)) {
    // GAS_SENSOR PEDAL_COUNTER; the upper nibble is the pedal STATE (see hyundai_rx_hook)
    cnt = msg->data[4] & 0xFU;
  } else {
  }
  return cnt;
}

static uint32_t hyundai_get_checksum(const CANPacket_t *msg) {

  uint8_t chksum = 0;
  if (msg->addr == 0x260U) {
    chksum = msg->data[7] & 0xFU;
  } else if (msg->addr == 0x386U) {
    chksum = ((msg->data[7] >> 6) << 2) | (msg->data[5] >> 6);
  } else if (msg->addr == 0x394U) {
    chksum = msg->data[6] & 0xFU;
  } else if (msg->addr == 0x421U) {
    chksum = msg->data[7] >> 4;
  } else if (hyundai_is_interceptor_sensor(msg)) {
    chksum = msg->data[5];
  } else {
  }
  return chksum;
}

static uint32_t hyundai_compute_checksum(const CANPacket_t *msg) {
  uint8_t chksum = 0;
  if (hyundai_is_interceptor_sensor(msg)) {
    chksum = hyundai_interceptor_crc8(msg);
  } else if (msg->addr == 0x386U) {
    // count the bits
    for (int i = 0; i < 8; i++) {
      uint8_t b = msg->data[i];
      for (int j = 0; j < 8; j++) {
        uint8_t bit = 0;
        // exclude checksum and counter
        if (((i != 1) || (j < 6)) && ((i != 3) || (j < 6)) && ((i != 5) || (j < 6)) && ((i != 7) || (j < 6))) {
          bit = (b >> (uint8_t)j) & 1U;
        }
        chksum += bit;
      }
    }
    chksum = (chksum ^ 9U) & 15U;
  } else {
    // sum of nibbles
    for (int i = 0; i < 8; i++) {
      if ((msg->addr == 0x394U) && (i == 7)) {
        continue; // exclude
      }
      uint8_t b = msg->data[i];
      if (((msg->addr == 0x260U) && (i == 7)) || ((msg->addr == 0x394U) && (i == 6)) || ((msg->addr == 0x421U) && (i == 7))) {
        b &= (msg->addr == 0x421U) ? 0x0FU : 0xF0U; // remove checksum
      }
      chksum += (b % 16U) + (b / 16U);
    }
    chksum = (16U - (chksum %  16U)) % 16U;
  }

  return chksum;
}

static void hyundai_rx_hook(const CANPacket_t *msg) {

  // SCC12 is on bus 2 for camera-based SCC cars, bus 0 on all others
  if (msg->addr == 0x421U) {
    if (((msg->bus == 0U) && !hyundai_camera_scc) || ((msg->bus == 2U) && hyundai_camera_scc)) {
      // 2 bits: 13-14
      int cruise_engaged = (GET_BYTES(msg, 0, 4) >> 13) & 0x3U;
      hyundai_common_cruise_state_check(cruise_engaged);
    }
  }

  if (msg->addr == 0x420U) {
    if (((msg->bus == 0U) && !hyundai_camera_scc) || ((msg->bus == 2U) && hyundai_camera_scc)) {
      if (!hyundai_longitudinal) {
        acc_main_on = GET_BIT(msg, 0U);
      }
    }
  }

  if (msg->bus == 0U) {
    if (msg->addr == 0x251U) {
      int torque_driver_new = (GET_BYTES(msg, 0, 2) & 0x7ffU) - 1024U;
      // update array of samples
      update_sample(&torque_driver, torque_driver_new);
    }

    if (msg->addr == 0x391U) {
      mads_button_press = GET_BIT(msg, 4U) ? MADS_BUTTON_PRESSED : MADS_BUTTON_NOT_PRESSED;
    }

    // ACC steering wheel buttons
    if (msg->addr == 0x4F1U) {
      if (hyundai_gas_interceptor) {
        // timed factory-cruise cancel: the bus is idle right after this cluster frame (hyundai_common.h)
        hyundai_fc_cancel_cluster_frame(msg);
      }
      int cruise_button = msg->data[0] & 0x7U;
      bool main_button = GET_BIT(msg, 3U);
      hyundai_common_cruise_buttons_check(cruise_button, main_button);
    }

    // gas press, different for EV, hybrid, and ICE models
    if ((msg->addr == 0x371U) && hyundai_ev_gas_signal) {
      gas_pressed = (((msg->data[4] & 0x7FU) << 1) | (msg->data[3] >> 7)) != 0U;
    } else if ((msg->addr == 0x371U) && hyundai_hybrid_gas_signal) {
      gas_pressed = msg->data[7] != 0U;
    } else if ((msg->addr == 0x91U) && hyundai_fcev_gas_signal) {
      gas_pressed = msg->data[6] != 0U;
    } else if ((msg->addr == 0x260U) && !hyundai_ev_gas_signal && !hyundai_hybrid_gas_signal && !hyundai_gas_interceptor) {
      gas_pressed = (msg->data[7] >> 6) != 0U;
    } else if (hyundai_gas_interceptor && (msg->addr == hyundai_interceptor_sensor_addr())) {
      // gas interceptor replaces EMS16 CF_Ems_AclAct as the gas source: the pedal sits between the driver's pedal and the
      // ECU, so the ECU's view of the pedal (EMS16) includes openpilot's command, while 0x201 is the driver's input only.
      // Byte 4 upper nibble = pedal STATE (0 NO_FAULT, 1 BAD_CHECKSUM, 2 SEND, 3 SCE, 4 STARTUP, 5 TIMEOUT, 6 INVALID);
      // in any fault state the pedal passes the driver's pedal straight through and ignores 0x200 (open item: disengage on fault).
      int gas_interceptor = hyundai_get_interceptor_gas(msg);
      gas_pressed = gas_interceptor > HYUNDAI_GAS_INTERCEPTOR_THRESHOLD;
      gas_interceptor_prev = gas_interceptor;
      // FCA11 long (bit 256): a pedal FAULT means openpilot throttle authority is gone (passthrough), so the ESC brake
      // must not mix with a throttle we do not control. LIVE from the pedal's STATE nibble; any observed fault then
      // LATCHES the long cut (hyundai_fca11_long_rx) until the next init - a pedal fault event is rare and serious,
      // and re-arm-by-ignition is the fail-closed choice. BOOT states do NOT cut: 4 = STARTUP (first ~1 s after
      // pedal power-up) and 5 = TIMEOUT (powered, no valid command yet) are present at EVERY ignition before card
      // starts commanding, so latching on them would disable the feature for the whole drive. 1 = BAD_CHECKSUM,
      // 2 = SEND, 3 = SCE, 6 = INVALID (unknown, treated as a fault) do cut. While a fault is latched the pedal
      // passes the driver's pedal straight through and 0x701 still reports the driver's raw ADC, so gas_pressed
      // stays truthful - this cut is belt-and-braces on top of the gas_pressed window check.
      uint8_t pedal_state = (uint8_t)((msg->data[4] >> 4) & 0xFU);
      hyundai_interceptor_pedal_fault = (pedal_state != 0U) && (pedal_state != 4U) && (pedal_state != 5U);
    } else {
    }

    // sample wheel speed, averaging opposite corners
    if (msg->addr == 0x386U) {
      uint32_t front_left_speed = GET_BYTES(msg, 0, 2) & 0x3FFFU;
      uint32_t rear_right_speed = GET_BYTES(msg, 6, 2) & 0x3FFFU;
      vehicle_moving = (front_left_speed > HYUNDAI_STANDSTILL_THRSLD) || (rear_right_speed > HYUNDAI_STANDSTILL_THRSLD);
    }

    if (msg->addr == 0x394U) {
      brake_pressed = ((msg->data[5] >> 5U) & 0x3U) == 0x2U;
    }

    if (msg->addr == 0x592U) {
      acc_main_on = GET_BIT(msg, 34U);
      bool cruise_engaged = GET_BIT(msg, 35U);
      hyundai_common_cruise_state_check(cruise_engaged);
    }

    if (msg->addr == 0x595U) {
      acc_main_on = GET_BIT(msg, 50U);
      bool cruise_engaged = GET_BIT(msg, 51U);
      hyundai_common_cruise_state_check(cruise_engaged);
    }

    if ((msg->addr == 0x260U) && hyundai_non_scc && !hyundai_ev_gas_signal && !hyundai_hybrid_gas_signal) {
      if (hyundai_gas_interceptor) {
        // CRUISE_LAMP_M = factory cruise MAIN armed. Pedal-long does not use it as its main switch. Instead it is a
        // lockout: the factory cruise can only engage while armed. openpilot itself never sends CLU11 in this mode.
        // Safety net: factory cruise ACTIVE while openpilot long is engaged -> panda's own timed CANCEL (hyundai_common.h).
        // Evaluated BEFORE the lockout below clears controls_allowed on this same frame.
        hyundai_fc_cancel_ems16(GET_BIT(msg, 26U));
        hyundai_factory_main_on = GET_BIT(msg, 25U);
        // factory cruise active (CRUISE_LAMP_S, implies MAIN armed): also a lockout, belt and braces
        if (hyundai_factory_main_on || GET_BIT(msg, 26U)) {
          controls_allowed = false;
        }
        // openpilot reports cruiseState.available = True in this mode (carstate_ext.py); mirror it so the MADS acc_main
        // edges (lateral request / exit) agree between panda and openpilot. Arming MAIN never affects lateral.
        acc_main_on = true;
      } else {
        acc_main_on = GET_BIT(msg, 25U);
      }
      bool cruise_engaged = GET_BIT(msg, 26U);
      hyundai_common_cruise_state_check(cruise_engaged);
    }
  }

  if (hyundai_gas_interceptor) {
    hyundai_gas_interceptor_pedal_check();
  }

  if (hyundai_fca11_rolling_test) {
    hyundai_fca11_rolling_rx(msg);
  }

  if (hyundai_fca11_long) {
    hyundai_fca11_long_rx(msg);
  }

  if (hyundai_lkas_park_test) {
    hyundai_lkas_park_rx(msg);
  }

  hyundai_common_reset_acc_main_on_mismatches();
}

static bool hyundai_tx_hook(const CANPacket_t *msg) {
  const TorqueSteeringLimits HYUNDAI_STEERING_LIMITS = HYUNDAI_LIMITS(384, 3, 7);
  const TorqueSteeringLimits HYUNDAI_STEERING_LIMITS_ALT = HYUNDAI_LIMITS(270, 2, 3);
  const TorqueSteeringLimits HYUNDAI_STEERING_LIMITS_ALT_2 = HYUNDAI_LIMITS(170, 2, 3);
  // CN7 Elantra N: same 384 ceiling, faster ramp-up (see hyundai_cn7_steer_ramp in hyundai_common.h)
  const TorqueSteeringLimits HYUNDAI_STEERING_LIMITS_CN7_RAMP = HYUNDAI_LIMITS(384, 4, 7);

  bool tx = true;

  // 0038: host-liveness (panda heartbeat) watchdog. Sample the gap since the PREVIOUS host TX-hook call BEFORE
  // updating, so `host_hb_fresh` measures the last inter-frame gap and self-clears the instant the host resumes
  // (cannot wedge). `0` means no host frame seen yet this arm -> read as fresh, so the FIRST brake of an engagement
  // is never refused by this clause. A gap over HOST_HB_TIMEOUT_US means the host/controls loop went silent, so the
  // actuating FCA11 frame is refused; the host's own passive close frame and every non-brake frame still pass, and
  // the very next host frame (a few ms later) is fresh again. This is the LIVENESS bound the owner asked for - it is
  // NOT a duration cap on any hold; while the host keeps streaming, a brake may run unbounded.
  uint32_t host_hb_now = microsecond_timer_get();
  bool host_hb_fresh = (hyundai_fca11_long_host_hb_ts == 0U) ||
                       (safety_get_ts_elapsed(host_hb_now, hyundai_fca11_long_host_hb_ts) <= HYUNDAI_FCA11_LONG_HOST_HB_TIMEOUT_US);
  hyundai_fca11_long_host_hb_ts = host_hb_now;

  // FCA11: Block any potential actuation
  if (msg->addr == 0x38DU) {
    int CR_VSM_DecCmd = msg->data[1];
    bool FCA_CmdAct = GET_BIT(msg, 20U);
    bool CF_VSM_DecCmdAct = GET_BIT(msg, 31U);

    if (hyundai_fca11_long) {
      // PRODUCTION FCA11 long braking (bit 256, on top of the gas interceptor). Openpilot is ENGAGED: every frame
      // (actuating or passive) requires controls_allowed AND the openpilot heartbeat engaged, on top of the
      // window/cap/rate rules. Allowed shapes:
      //   * ACTUATING: the car layer's mirror of the camera frame (variant B) with CR_VSM_DecCmd 1..30 (0.01 g/LSB),
      //     CF_VSM_Prefill (bit 0), CF_VSM_Warn = 3 (bits 3-4), FCA_CmdAct (bit 20), CF_VSM_DecCmdAct = 0;
      //   * PASSIVE: the camera's own idle shape (all brake fields 0, Warn = 0) - the clean release/hand-back frame,
      //     byte-identical to what the ESC sees from the camera every idle frame.
      // Always blocked: decel above the cap, CF_VSM_HBACmd (bits 1-2), FCA_StopReq (bit 21), CF_VSM_DecCmdAct,
      // Warn 1/2 (never a shape this car answered), Warn 3 on a passive frame (a warning we have no reason to send),
      // any ACTUATION outside the window / after a cut / while the camera owns FCA11 / from a host
      // that has gone silent past HOST_HB_TIMEOUT_US (0038 liveness watchdog). 0040: there is NO speed floor any more
      // (the deleted HYUNDAI_FCA11_LONG_MIN_SPEED); actuation is legal down to and through zero.
      int CF_VSM_HBACmd = (msg->data[0] >> 1) & 0x3U;
      bool CF_VSM_Prefill = GET_BIT(msg, 0U);
      int CF_VSM_Warn = (msg->data[0] >> 3) & 0x3U;
      bool FCA_StopReq = GET_BIT(msg, 21U);
      bool actuation = (CR_VSM_DecCmd != 0) || FCA_CmdAct || CF_VSM_DecCmdAct || CF_VSM_Prefill;

      // 0038 throttle exclusivity: record that the host is commanding brake fields (this frame, accepted or not) so
      // the GAS_COMMAND branch below strips its throttle for the next BRAKE_THROTTLE_EXCL window.
      if (actuation) {
        hyundai_fca11_long_host_brake_active_ts = microsecond_timer_get();
      }

      bool violation = (CR_VSM_DecCmd > HYUNDAI_FCA11_LONG_MAX_DEC);
      violation |= (CF_VSM_HBACmd != 0);
      violation |= FCA_StopReq;
      violation |= CF_VSM_DecCmdAct;           // variant B: FCA_CmdAct carries the command, not DecCmdAct
      violation |= actuation ? (CF_VSM_Warn != 3) : (CF_VSM_Warn != 0);
      violation |= hyundai_fca11_long_camera_owns;
      // 0030 (D2-i): the controls_allowed / heartbeat gate applies to ACTUATING frames only. A PASSIVE frame is the
      // camera's own idle shape (all brake fields 0, Warn 0, DecCmd 0); accepting it can only ever CLOSE an episode
      // (the rate-limit reference drops to 0, so the next episode restarts its onset ramp) and can never command the
      // ESC. Requiring the engagement gate for it meant a close frame sent at an engage edge (!heartbeat_engaged, the
      // heartbeat only lands at pandad's 10 Hz step) or a disengage edge (!controls_allowed) was REFUSED, so the panda
      // episode never closed (the 860-frame stale act_active defect, 14f). The CAMERA-OWNS hand-back still blocks
      // EVERY host frame (a passive host frame injected while the camera is actuating could cancel its AEB), and
      // !camera_owns is checked before any state change.
      violation |= actuation && (!controls_allowed || !heartbeat_engaged);
      violation |= actuation && !hyundai_fca11_long_window_ok();
      // 0040: the speed floor clause is DELETED. `hyundai_fca11_long_speed_min` is still tracked (the slowest-wheel
      // observable, kept for parity with the SEEN_SPEED evidence the window requires) but it NO LONGER refuses
      // actuation - the ESC is allowed to be asked to brake at any speed, down to and through zero.
      // 0038: host-liveness watchdog. `host_hb_fresh` was sampled at the top of this hook (gap since the PREVIOUS
      // host frame); a stale host may not command actuation. It self-clears on the next host frame (a few ms later),
      // and it never gates a passive frame (the release path).
      violation |= actuation && !host_hb_fresh;

      uint32_t ts = microsecond_timer_get();
      if (actuation && !violation) {
        // rate limit on decel growth: at most +0.04 g above the last ACCEPTED frame (release is immediate, and an
        // accepted smaller value RESETS the reference down, so the next ramp starts from the released level).
        // 0035 ONSET rule: the reference is 0 - so this frame is capped at RATE_STEP - whenever the ESC is not
        // currently seeing our command: no accepted frame yet this arm (dec_last < 0), the last accepted frame was
        // PASSIVE (dec_last == 0), or our stream went STALE (the SAME predicate hyundai_fwd_hook uses to forward the
        // camera's own FCA11 again, so the ESC has been back on the camera's idle frame). Without the stale term a
        // host could brake at 30, go silent past TX_STALE (camera re-owns, ESC sees 0) and reopen at 30: a 0 -> 30
        // step at the ESC, repeatable without bound now that no cooldown spaces episodes apart.
        bool stream_stale = !hyundai_fca11_long_tx_seen ||
                            (safety_get_ts_elapsed(ts, hyundai_fca11_long_ts_tx) > HYUNDAI_FCA11_LONG_TX_STALE_US);
        int dec_ref = ((hyundai_fca11_long_dec_last < 0) || stream_stale) ? 0 : hyundai_fca11_long_dec_last;
        violation |= (CR_VSM_DecCmd > (dec_ref + HYUNDAI_FCA11_LONG_RATE_STEP));
      }
      if (!violation) {
        hyundai_fca11_long_tx_seen = true;
        hyundai_fca11_long_ts_tx = ts;
        hyundai_fca11_long_dec_last = CR_VSM_DecCmd;
      }
      if (violation) {
        tx = false;
      }
    } else if (!hyundai_fca11_brake_test) {
      if ((CR_VSM_DecCmd != 0) || FCA_CmdAct || CF_VSM_DecCmdAct) {
        tx = false;
      }
    } else {
      // TEST-ONLY (parked brake-injection research). Deliberately independent of controls_allowed: openpilot is
      // disengaged during the test. Allowed: CR_VSM_DecCmd <= cap (0.10 g parked, 0.30 g rolling), CF_VSM_Prefill (bit 0), FCA_CmdAct (bit 20),
      // CF_VSM_DecCmdAct (bit 31). Always blocked: decel above the cap, CF_VSM_HBACmd (bits 1-2, brake assist),
      // FCA_StopReq (bit 21), and ANY actuation (decel/act/prefill) while the vehicle is moving.
      // ROLLING extension (bit 128): the moving/standstill rule is REPLACED by the rolling window: actuation ONLY while
      // rolling inside [floor, ceiling] in D with no pedal and fresh inputs, never after a latched cut, and never at
      // standstill. Cap, HBA and StopReq rules are identical.
      int CF_VSM_HBACmd = (msg->data[0] >> 1) & 0x3U;
      bool CF_VSM_Prefill = GET_BIT(msg, 0U);
      bool FCA_StopReq = GET_BIT(msg, 21U);
      bool actuation = (CR_VSM_DecCmd != 0) || FCA_CmdAct || CF_VSM_DecCmdAct || CF_VSM_Prefill;

      int dec_cap = HYUNDAI_FCA11_TEST_MAX_DEC;
      if (hyundai_fca11_rolling_test) {
        dec_cap = HYUNDAI_FCA11_ROLL_MAX_DEC;
      }
      bool violation = (CR_VSM_DecCmd > dec_cap);
      violation |= (CF_VSM_HBACmd != 0);
      violation |= FCA_StopReq;
      if (hyundai_fca11_rolling_test) {
        uint32_t ts = microsecond_timer_get();
        // CF_VSM_Warn (bits 3-4) raises the dash FCW: in the rolling mode a Warn-only frame (no actuation bits) is
        // policed exactly like actuation: same rolling window, same latched cut, and it runs the same 1.2 s clock
        // (a Warn lead-in followed by actuation counts as ONE continuous episode)
        bool CF_VSM_Warn = ((msg->data[0] >> 3) & 0x3U) != 0U;
        bool gated = actuation || CF_VSM_Warn;
        violation |= hyundai_fca11_roll_camera_owns;  // real camera request: never a second FCA11 source
        violation |= gated && !hyundai_fca11_rolling_window_ok();
        violation |= gated && hyundai_fca11_roll_act_active &&
                     (safety_get_ts_elapsed(ts, hyundai_fca11_roll_ts_act) > HYUNDAI_FCA11_ROLL_MAX_ACT_US);
        if (!violation) {
          hyundai_fca11_roll_tx_seen = true;
          hyundai_fca11_roll_ts_tx = ts;
          if (gated && !hyundai_fca11_roll_act_active) {
            hyundai_fca11_roll_act_active = true;
            hyundai_fca11_roll_ts_act = ts;
          } else if (!gated) {
            hyundai_fca11_roll_act_active = false;
          } else {
          }
        }
      } else {
        violation |= actuation && vehicle_moving;
      }
      if (violation) {
        tx = false;
      }
    }
  }

  if (msg->addr == 0x420U) {
    acc_main_on_tx = GET_BIT(msg, 0U);
    hyundai_common_acc_main_on_sync();
  }

  // ACCEL: safety check
  if (msg->addr == 0x421U) {
    int desired_accel_raw = (((msg->data[4] & 0x7U) << 8) | msg->data[3]) - 1023U;
    int desired_accel_val = ((msg->data[5] << 3) | (msg->data[4] >> 5)) - 1023U;

    int aeb_decel_cmd = msg->data[2];
    bool aeb_req = GET_BIT(msg, 54U);
    bool aeb_stop_req = GET_BIT(msg, 55U);

    bool violation = false;

    violation |= longitudinal_accel_checks(desired_accel_raw, HYUNDAI_LONG_LIMITS);
    violation |= longitudinal_accel_checks(desired_accel_val, HYUNDAI_LONG_LIMITS);
    if (!hyundai_escc) {
      violation |= (aeb_decel_cmd != 0);
      violation |= aeb_req;
      violation |= aeb_stop_req;
    }

    if (violation) {
      tx = false;
    }
  }

  // LKA STEER: safety check
  if (msg->addr == 0x340U) {
    int desired_torque = ((GET_BYTES(msg, 0, 4) >> 16) & 0x7ffU) - 1024U;
    bool steer_req = GET_BIT(msg, 27U);

    const bool cn7_ramp = hyundai_cn7_steer_ramp && hyundai_non_scc;
    const TorqueSteeringLimits limits = hyundai_alt_limits_2 ? HYUNDAI_STEERING_LIMITS_ALT_2 :
                                        hyundai_alt_limits ? HYUNDAI_STEERING_LIMITS_ALT :
                                        cn7_ramp ? HYUNDAI_STEERING_LIMITS_CN7_RAMP : HYUNDAI_STEERING_LIMITS;

    if (hyundai_lkas_park_test) {
      // TEST-ONLY parked sweep (bit 512): openpilot is stopped and controls_allowed plays no part; panda's own parked
      // rules (hyundai_lkas_park_tx) are the only enforcer. The normal path below is untouched when the bit is unset.
      tx = hyundai_lkas_park_tx(msg);
    } else if (steer_torque_cmd_checks(desired_torque, steer_req, limits)) {
      tx = false;
    } else {
    }
  }

  // UDS: Only tester present ("\x02\x3E\x80\x00\x00\x00\x00\x00") allowed on diagnostics address
  if (msg->addr == 0x7D0U) {
    if ((GET_BYTES(msg, 0, 4) != 0x00803E02U) || (GET_BYTES(msg, 4, 4) != 0x0U)) {
      tx = false;
    }
  }

  // GAS: safety check (interceptor). Any non-zero GAS_COMMAND requires longitudinal to be allowed. Only the active
  // dialect's command id is in the TX allowlist; the other one is already rejected before this hook.
  if (hyundai_gas_interceptor && (msg->addr == hyundai_interceptor_command_addr())) {
    if (longitudinal_interceptor_checks(msg)) {
      tx = false;
    }
    // magnitude: either track above the openpilot cap is never allowed (a zero / clear frame is always within it)
    uint16_t gas_a = (uint16_t)((uint16_t)msg->data[0] << 8U) | (uint16_t)msg->data[1];
    uint16_t gas_b = (uint16_t)((uint16_t)msg->data[2] << 8U) | (uint16_t)msg->data[3];
    if ((gas_a > HYUNDAI_GAS_INTERCEPTOR_MAX_GAS_A) || (gas_b > HYUNDAI_GAS_INTERCEPTOR_MAX_GAS_B)) {
      tx = false;
    }
    // 0038 throttle exclusivity (FCA11 long only): while the host is commanding ACTUATING brake fields, strip
    // throttle pass-through for this frame. A non-zero GAS_COMMAND within the brake window is refused; a zero
    // (clear) frame is always accepted, and this clause never touches 0x38D - so braking authority is UNCHANGED, it
    // only ever removes throttle while braking. A wedged host can therefore never brake AND throttle at once.
    if (hyundai_fca11_long && hyundai_fca11_long_host_brake_active_ts != 0U &&
        (safety_get_ts_elapsed(microsecond_timer_get(), hyundai_fca11_long_host_brake_active_ts) <= HYUNDAI_FCA11_LONG_BRAKE_THROTTLE_EXCL_US)) {
      if ((gas_a != 0U) || (gas_b != 0U)) {
        tx = false;
      }
    }
  }

  // BUTTONS: used for resume spamming and cruise cancellation
  // (gas interceptor: CLU11 is not in the TX allowlist at all, so openpilot/USB can never send one; the only CLU11 in
  // this mode is panda's own timed CANCEL, built and policed in hyundai_common.h, which does not pass through here)
  if ((msg->addr == 0x4F1U) && !hyundai_longitudinal) {
    int button = msg->data[0] & 0x7U;

    bool allowed_resume = (button == 1) && controls_allowed;
    bool allowed_set = (button == 2) && controls_allowed;
    bool allowed_cancel = (button == 4) && cruise_engaged_prev;
    if (!(allowed_resume || allowed_set || allowed_cancel)) {
      tx = false;
    }
  }

  return tx;
}

static safety_config hyundai_init(uint16_t param) {
  static const CanMsg HYUNDAI_LONG_TX_MSGS[] = {
    HYUNDAI_LONG_COMMON_TX_MSGS(0)
    {0x38D, 0, 8, .check_relay = false}, // FCA11 Bus 0
    {0x483, 0, 8, .check_relay = false}, // FCA12 Bus 0
    {0x7D0, 0, 8, .check_relay = false}, // radar UDS TX addr Bus 0 (for radar disable)
  };

  static const CanMsg HYUNDAI_CAMERA_SCC_TX_MSGS[] = {
    HYUNDAI_COMMON_TX_MSGS(2)
  };

  static const CanMsg HYUNDAI_CAMERA_SCC_LONG_TX_MSGS[] = {
    HYUNDAI_LONG_COMMON_TX_MSGS(2)
  };

  static const CanMsg HYUNDAI_LONG_ESCC_TX_MSGS[] = {
    HYUNDAI_LONG_COMMON_TX_MSGS(0)
  };

  // LKAS11 + LFAHDA_MFC + the pedal command only: no SCC/FCA/radar-UDS actuation messages, and NO CLU11 (0x4F1).
  // A CLU11 from us shares the ID with the cluster's own 50 Hz CLU11, so the two can win arbitration together and then
  // bit-error in the data field; every such error frame can latch the pedal's FAULT_SCE (routes 11c and 123: all
  // commanded SCE onsets 3-17 ms after one of our CLU11 frames). The factory cruise is locked out via its MAIN lamp
  // instead (hyundai_factory_main_on), so there is nothing to cancel.
  static const CanMsg HYUNDAI_GAS_INTERCEPTOR_TX_MSGS[] = {
    {0x340, 0, 8, .check_relay = true},   // LKAS11 Bus 0
    {0x485, 0, 4, .check_relay = true},   // LFAHDA_MFC Bus 0
    {0x200, 0, 6, .check_relay = false},  // GAS_COMMAND Bus 0 (comma pedal)
  };

  // same, remapped pedal IDs: GAS_COMMAND_R on 0x700 INSTEAD OF 0x200 (never both)
  static const CanMsg HYUNDAI_GAS_INTERCEPTOR_REMAPPED_TX_MSGS[] = {
    {0x340, 0, 8, .check_relay = true},   // LKAS11 Bus 0
    {0x485, 0, 4, .check_relay = true},   // LFAHDA_MFC Bus 0
    {0x700, 0, 6, .check_relay = false},  // GAS_COMMAND_R Bus 0 (comma pedal, remapped IDs)
  };

  // TEST-ONLY FCA11 brake test: FCA11 with check_relay=true, unlike HYUNDAI_LONG_TX_MSGS (false). That makes fwd_hook
  // drop the stock camera's FCA11 (bus 2 -> 0), so there is never more than one FCA11 source on the car bus.
  // It is relay-fault safe on the target car (2022 Elantra N, non-SCC): full route logs show 0x38D received ONLY on
  // bus 2 (camera side, src 2/128), never on bus 0, and stock_ecu_check (safety.h) only trips relay_malfunction when a
  // check_relay addr is RECEIVED on the same bus as the TX entry (bus 0).
  // Deliberately NOT HYUNDAI_COMMON_TX_MSGS: its LKAS11 (0x340) / LFAHDA_MFC (0x485) entries are check_relay=true
  // because in normal operation openpilot REPLACES the camera's steering messages; here that would block the camera's
  // own lane-keeping stream (bus 2 -> 0) for the whole test with nothing replacing it. Both are left out entirely, so
  // the camera's 0x340/0x485 keep forwarding and panda can't add a second source for them. Only FCA11 is replaced.
  static const CanMsg HYUNDAI_FCA11_TEST_TX_MSGS[] = {
    {0x4F1, 0, 4, .check_relay = false},  // CLU11 Bus 0 (cancel only, policed in hyundai_tx_hook)
    {0x38D, 0, 8, .check_relay = true},   // FCA11 Bus 0 (replaces the stock camera's)
  };

  // ROLLING test: same two messages, but FCA11's camera blocking is DYNAMIC (hyundai_fwd_hook): blocked only while panda
  // is actively transmitting 0x38D, so a dead host hands FCA11 back to the camera automatically. Still relay-checked.
  static const CanMsg HYUNDAI_FCA11_ROLL_TX_MSGS[] = {
    {0x4F1, 0, 4, .check_relay = false},  // CLU11 Bus 0 (cancel only, policed in hyundai_tx_hook)
    {0x38D, 0, 8, .check_relay = true, .disable_static_blocking = true},  // FCA11 Bus 0, see hyundai_fwd_hook
  };

  // PRODUCTION FCA11 long braking (bit 256): the FULL pedal TX list (LKAS11 + LFAHDA with check_relay, which is correct
  // in production because openpilot REPLACES the camera's steering messages) + the active pedal command + FCA11 with
  // DYNAMIC camera blocking (hyundai_fwd_hook) and check_relay: 0x38D never appears on bus 0 from the car (route logs),
  // so the relay check only trips on a real harness fault. Two dialect variants, exactly like the pedal lists.
  static const CanMsg HYUNDAI_FCA11_LONG_TX_MSGS[] = {
    {0x340, 0, 8, .check_relay = true},   // LKAS11 Bus 0
    {0x485, 0, 4, .check_relay = true},   // LFAHDA_MFC Bus 0
    {0x200, 0, 6, .check_relay = false},  // GAS_COMMAND Bus 0 (comma pedal)
    {0x38D, 0, 8, .check_relay = true, .disable_static_blocking = true},  // FCA11 Bus 0, see hyundai_fwd_hook
  };

  static const CanMsg HYUNDAI_FCA11_LONG_REMAPPED_TX_MSGS[] = {
    {0x340, 0, 8, .check_relay = true},   // LKAS11 Bus 0
    {0x485, 0, 4, .check_relay = true},   // LFAHDA_MFC Bus 0
    {0x700, 0, 6, .check_relay = false},  // GAS_COMMAND_R Bus 0 (comma pedal, remapped IDs)
    {0x38D, 0, 8, .check_relay = true, .disable_static_blocking = true},  // FCA11 Bus 0, see hyundai_fwd_hook
  };

  // TEST-ONLY parked LKAS11 sweep (bit 512): LKAS11 ALONE. check_relay stays true (a bus-0 0x340 from the car is still
  // a relay fault), but the camera's copy is blocked DYNAMICALLY (hyundai_fwd_hook): forwarded until panda transmits,
  // and again within 100 ms after panda stops. LFAHDA_MFC (0x485), CLU11, the pedal and every SCC/FCA message are NOT
  // transmittable; the camera's 0x485 keeps forwarding.
  static const CanMsg HYUNDAI_LKAS_PARK_TX_MSGS[] = {
    {0x340, 0, 8, .check_relay = true, .disable_static_blocking = true},  // LKAS11 Bus 0, see hyundai_fwd_hook
  };

  hyundai_common_init(param);
  hyundai_legacy = false;
  hyundai_fca11_rolling_reset();
  hyundai_fca11_long_reset();
  hyundai_lkas_park_reset();

  safety_config ret;
  if (hyundai_longitudinal) {
    // Use CLU11 (buttons) to manage controls allowed instead of SCC cruise state
    static RxCheck hyundai_long_rx_checks[] = {
      HYUNDAI_COMMON_RX_CHECKS(false)
    };

    static RxCheck hyundai_lda_button_long_rx_checks[] = {
      HYUNDAI_COMMON_RX_CHECKS(false)
      HYUNDAI_LDA_BUTTON_ADDR_CHECK
    };

    static RxCheck hyundai_fcev_long_rx_checks[] = {
      HYUNDAI_COMMON_RX_CHECKS(false)
      HYUNDAI_FCEV_GAS_ADDR_CHECK
    };

    static RxCheck hyundai_fcev_lda_button_long_rx_checks[] = {
      HYUNDAI_COMMON_RX_CHECKS(false)
      HYUNDAI_FCEV_GAS_ADDR_CHECK
      HYUNDAI_LDA_BUTTON_ADDR_CHECK
    };

    if (hyundai_fcev_gas_signal) {
      if (hyundai_has_lda_button) {
        SET_RX_CHECKS(hyundai_fcev_lda_button_long_rx_checks, ret);
      } else {
        SET_RX_CHECKS(hyundai_fcev_long_rx_checks, ret);
      }
    } else {
      if (hyundai_has_lda_button) {
        SET_RX_CHECKS(hyundai_lda_button_long_rx_checks, ret);
      } else {
        SET_RX_CHECKS(hyundai_long_rx_checks, ret);
      }
    }
    if (hyundai_escc) {
      SET_TX_MSGS(HYUNDAI_LONG_ESCC_TX_MSGS, ret);
    } else if (hyundai_camera_scc) {
      SET_TX_MSGS(HYUNDAI_CAMERA_SCC_LONG_TX_MSGS, ret);
    } else {
      SET_TX_MSGS(HYUNDAI_LONG_TX_MSGS, ret);
    }

  } else if (hyundai_camera_scc) {
    static RxCheck hyundai_cam_scc_rx_checks[] = {
      HYUNDAI_COMMON_RX_CHECKS(false)
      HYUNDAI_SCC12_ADDR_CHECK(2)
      HYUNDAI_SCC11_ADDR_CHECK(2)
      HYUNDAI_LDA_BUTTON_ADDR_CHECK
    };

    ret = BUILD_SAFETY_CFG(hyundai_cam_scc_rx_checks, HYUNDAI_CAMERA_SCC_TX_MSGS);
  } else {
    static RxCheck hyundai_rx_checks[] = {
       HYUNDAI_COMMON_RX_CHECKS(false)
       HYUNDAI_SCC12_ADDR_CHECK(0)
       HYUNDAI_SCC11_ADDR_CHECK(0)
    };

    static RxCheck hyundai_lda_button_rx_checks[] = {
      HYUNDAI_COMMON_RX_CHECKS(false)
      HYUNDAI_SCC12_ADDR_CHECK(0)
      HYUNDAI_SCC11_ADDR_CHECK(0)
      HYUNDAI_LDA_BUTTON_ADDR_CHECK
    };

    static RxCheck hyundai_fcev_rx_checks[] = {
      HYUNDAI_COMMON_RX_CHECKS(false)
      HYUNDAI_SCC12_ADDR_CHECK(0)
      HYUNDAI_SCC11_ADDR_CHECK(0)
      HYUNDAI_FCEV_GAS_ADDR_CHECK
    };

    static RxCheck hyundai_fcev_lda_button_rx_checks[] = {
      HYUNDAI_COMMON_RX_CHECKS(false)
      HYUNDAI_SCC12_ADDR_CHECK(0)
      HYUNDAI_SCC11_ADDR_CHECK(0)
      HYUNDAI_FCEV_GAS_ADDR_CHECK
      HYUNDAI_LDA_BUTTON_ADDR_CHECK
    };

    static RxCheck hyundai_non_scc_addr_checks[] = {
      HYUNDAI_COMMON_RX_CHECKS(false)
    };

    static RxCheck hyundai_non_scc_lda_button_addr_checks[] = {
      HYUNDAI_COMMON_RX_CHECKS(false)
      HYUNDAI_LDA_BUTTON_ADDR_CHECK
    };

    static RxCheck hyundai_hev_non_scc_addr_checks[] = {
      HYUNDAI_COMMON_RX_CHECKS(false)
      HYUNDAI_NON_SCC_HEV_ADDR_CHECK
    };

    static RxCheck hyundai_hev_non_scc_lda_button_addr_checks[] = {
      HYUNDAI_COMMON_RX_CHECKS(false)
      HYUNDAI_NON_SCC_HEV_ADDR_CHECK
      HYUNDAI_LDA_BUTTON_ADDR_CHECK
    };

    static RxCheck hyundai_ev_non_scc_addr_checks[] = {
      HYUNDAI_COMMON_RX_CHECKS(false)
      HYUNDAI_NON_SCC_EV_ADDR_CHECK
    };

    static RxCheck hyundai_ev_non_scc_lda_button_addr_checks[] = {
      HYUNDAI_COMMON_RX_CHECKS(false)
      HYUNDAI_NON_SCC_EV_ADDR_CHECK
      HYUNDAI_LDA_BUTTON_ADDR_CHECK
    };

    static RxCheck hyundai_non_scc_fca11_rolling_addr_checks[] = {
      HYUNDAI_COMMON_RX_CHECKS(false)
      HYUNDAI_FCA11_ROLL_GEAR_ADDR_CHECK
      HYUNDAI_FCA11_ROLL_CAMERA_ADDR_CHECK
    };

    static RxCheck hyundai_non_scc_fca11_rolling_lda_button_addr_checks[] = {
      HYUNDAI_COMMON_RX_CHECKS(false)
      HYUNDAI_FCA11_ROLL_GEAR_ADDR_CHECK
      HYUNDAI_FCA11_ROLL_CAMERA_ADDR_CHECK
      HYUNDAI_LDA_BUTTON_ADDR_CHECK
    };

    static RxCheck hyundai_non_scc_interceptor_addr_checks[] = {
      HYUNDAI_COMMON_RX_CHECKS(false)
      HYUNDAI_GAS_INTERCEPTOR_ADDR_CHECK
    };

    static RxCheck hyundai_non_scc_interceptor_lda_button_addr_checks[] = {
      HYUNDAI_COMMON_RX_CHECKS(false)
      HYUNDAI_GAS_INTERCEPTOR_ADDR_CHECK
      HYUNDAI_LDA_BUTTON_ADDR_CHECK
    };

    static RxCheck hyundai_non_scc_interceptor_remapped_addr_checks[] = {
      HYUNDAI_COMMON_RX_CHECKS(false)
      HYUNDAI_GAS_INTERCEPTOR_REMAPPED_ADDR_CHECK
    };

    static RxCheck hyundai_non_scc_interceptor_remapped_lda_button_addr_checks[] = {
      HYUNDAI_COMMON_RX_CHECKS(false)
      HYUNDAI_GAS_INTERCEPTOR_REMAPPED_ADDR_CHECK
      HYUNDAI_LDA_BUTTON_ADDR_CHECK
    };

    // PRODUCTION FCA11 long (bit 256): the pedal's RX check + gear (LVR12, the D gate) + the camera's own FCA11
    // (bus 2; lag-policed and the source of the camera-request hand-back latch). Both pedal dialects.
    static RxCheck hyundai_non_scc_fca11_long_addr_checks[] = {
      HYUNDAI_COMMON_RX_CHECKS(false)
      HYUNDAI_GAS_INTERCEPTOR_ADDR_CHECK
      HYUNDAI_FCA11_ROLL_GEAR_ADDR_CHECK
      HYUNDAI_FCA11_ROLL_CAMERA_ADDR_CHECK
    };

    static RxCheck hyundai_non_scc_fca11_long_lda_button_addr_checks[] = {
      HYUNDAI_COMMON_RX_CHECKS(false)
      HYUNDAI_GAS_INTERCEPTOR_ADDR_CHECK
      HYUNDAI_FCA11_ROLL_GEAR_ADDR_CHECK
      HYUNDAI_FCA11_ROLL_CAMERA_ADDR_CHECK
      HYUNDAI_LDA_BUTTON_ADDR_CHECK
    };

    static RxCheck hyundai_non_scc_fca11_long_remapped_addr_checks[] = {
      HYUNDAI_COMMON_RX_CHECKS(false)
      HYUNDAI_GAS_INTERCEPTOR_REMAPPED_ADDR_CHECK
      HYUNDAI_FCA11_ROLL_GEAR_ADDR_CHECK
      HYUNDAI_FCA11_ROLL_CAMERA_ADDR_CHECK
    };

    static RxCheck hyundai_non_scc_fca11_long_remapped_lda_button_addr_checks[] = {
      HYUNDAI_COMMON_RX_CHECKS(false)
      HYUNDAI_GAS_INTERCEPTOR_REMAPPED_ADDR_CHECK
      HYUNDAI_FCA11_ROLL_GEAR_ADDR_CHECK
      HYUNDAI_FCA11_ROLL_CAMERA_ADDR_CHECK
      HYUNDAI_LDA_BUTTON_ADDR_CHECK
    };

    // parked LKAS11 sweep: gear (LVR12) + steering angle (SAS11) join the RX checks so their lag/validity is policed
    static RxCheck hyundai_non_scc_lkas_park_addr_checks[] = {
      HYUNDAI_COMMON_RX_CHECKS(false)
      HYUNDAI_FCA11_ROLL_GEAR_ADDR_CHECK
      HYUNDAI_LKAS_PARK_SAS_ADDR_CHECK
    };

    static RxCheck hyundai_non_scc_lkas_park_lda_button_addr_checks[] = {
      HYUNDAI_COMMON_RX_CHECKS(false)
      HYUNDAI_FCA11_ROLL_GEAR_ADDR_CHECK
      HYUNDAI_LKAS_PARK_SAS_ADDR_CHECK
      HYUNDAI_LDA_BUTTON_ADDR_CHECK
    };

    SET_TX_MSGS(HYUNDAI_TX_MSGS, ret);
    if (hyundai_lkas_park_test) {  // hyundai_common_init: non-SCC ICE only, no longitudinal, no pedal, no FCA11 bits
      SET_TX_MSGS(HYUNDAI_LKAS_PARK_TX_MSGS, ret);
      if (hyundai_has_lda_button) {
        SET_RX_CHECKS(hyundai_non_scc_lkas_park_lda_button_addr_checks, ret);
      } else {
        SET_RX_CHECKS(hyundai_non_scc_lkas_park_addr_checks, ret);
      }
    } else if (hyundai_fca11_brake_test) {  // hyundai_common_init: non-SCC ICE only, never with any openpilot longitudinal
      SET_TX_MSGS(HYUNDAI_FCA11_TEST_TX_MSGS, ret);
      if (hyundai_fca11_rolling_test) {
        SET_TX_MSGS(HYUNDAI_FCA11_ROLL_TX_MSGS, ret);
        // same TX list; RX adds the gear (LVR12) so its lag/validity is policed like every other window input
        if (hyundai_has_lda_button) {
          SET_RX_CHECKS(hyundai_non_scc_fca11_rolling_lda_button_addr_checks, ret);
        } else {
          SET_RX_CHECKS(hyundai_non_scc_fca11_rolling_addr_checks, ret);
        }
      } else if (hyundai_has_lda_button) {
        SET_RX_CHECKS(hyundai_non_scc_lda_button_addr_checks, ret);
      } else {
        SET_RX_CHECKS(hyundai_non_scc_addr_checks, ret);
      }
    } else if (hyundai_fca11_long) {  // implies hyundai_gas_interceptor (hyundai_common_init); never with the test bits
      // PRODUCTION FCA11 long: the pedal TX list + FCA11 (dynamic camera blocking); RX adds gear (LVR12) and the
      // camera's own FCA11 (bus 2) so the window inputs are policed for lag/validity and a camera request latches
      // the hand-back, exactly like the rolling test. The pedal sensor check stays (whichever dialect is active).
      if (hyundai_gas_interceptor_remapped) {
        SET_TX_MSGS(HYUNDAI_FCA11_LONG_REMAPPED_TX_MSGS, ret);
      } else {
        SET_TX_MSGS(HYUNDAI_FCA11_LONG_TX_MSGS, ret);
      }
      if (hyundai_has_lda_button) {
        if (hyundai_gas_interceptor_remapped) {
          SET_RX_CHECKS(hyundai_non_scc_fca11_long_remapped_lda_button_addr_checks, ret);
        } else {
          SET_RX_CHECKS(hyundai_non_scc_fca11_long_lda_button_addr_checks, ret);
        }
      } else {
        if (hyundai_gas_interceptor_remapped) {
          SET_RX_CHECKS(hyundai_non_scc_fca11_long_remapped_addr_checks, ret);
        } else {
          SET_RX_CHECKS(hyundai_non_scc_fca11_long_addr_checks, ret);
        }
      }
    } else if (hyundai_gas_interceptor_remapped) {  // implies hyundai_gas_interceptor (hyundai_common_init)
      // remapped pedal IDs: TX 0x700 / RX 0x701, mutually exclusive with the standard set below
      SET_TX_MSGS(HYUNDAI_GAS_INTERCEPTOR_REMAPPED_TX_MSGS, ret);
      if (hyundai_has_lda_button) {
        SET_RX_CHECKS(hyundai_non_scc_interceptor_remapped_lda_button_addr_checks, ret);
      } else {
        SET_RX_CHECKS(hyundai_non_scc_interceptor_remapped_addr_checks, ret);
      }
    } else if (hyundai_gas_interceptor) {
      // hyundai_common_init only allows this for non-SCC ICE (EMS16) cars; RX is the non-SCC set + the pedal
      SET_TX_MSGS(HYUNDAI_GAS_INTERCEPTOR_TX_MSGS, ret);
      if (hyundai_has_lda_button) {
        SET_RX_CHECKS(hyundai_non_scc_interceptor_lda_button_addr_checks, ret);
      } else {
        SET_RX_CHECKS(hyundai_non_scc_interceptor_addr_checks, ret);
      }
    } else if (hyundai_fcev_gas_signal) {
      if (hyundai_has_lda_button) {
        SET_RX_CHECKS(hyundai_fcev_lda_button_rx_checks, ret);
      } else {
        SET_RX_CHECKS(hyundai_fcev_rx_checks, ret);
      }
    } else if (hyundai_non_scc) {
      if (hyundai_ev_gas_signal) {
        if (hyundai_has_lda_button) {
          SET_RX_CHECKS(hyundai_ev_non_scc_lda_button_addr_checks, ret);
        } else {
          SET_RX_CHECKS(hyundai_ev_non_scc_addr_checks, ret);
        }
      } else if (hyundai_hybrid_gas_signal) {
        if (hyundai_has_lda_button) {
          SET_RX_CHECKS(hyundai_hev_non_scc_lda_button_addr_checks, ret);
        } else {
          SET_RX_CHECKS(hyundai_hev_non_scc_addr_checks, ret);
        }
      } else {
        if (hyundai_has_lda_button) {
          SET_RX_CHECKS(hyundai_non_scc_lda_button_addr_checks, ret);
        } else {
          SET_RX_CHECKS(hyundai_non_scc_addr_checks, ret);
        }
      }
    } else {
      if (hyundai_has_lda_button) {
        SET_RX_CHECKS(hyundai_lda_button_rx_checks, ret);
      } else {
        SET_RX_CHECKS(hyundai_rx_checks, ret);
      }
    }
  }
  return ret;
}

static safety_config hyundai_legacy_init(uint16_t param) {
  // older hyundai models have less checks due to missing counters and checksums
  static RxCheck hyundai_legacy_rx_checks[] = {
    HYUNDAI_COMMON_RX_CHECKS(true)
    HYUNDAI_SCC12_ADDR_CHECK(0)
    HYUNDAI_SCC11_ADDR_CHECK(0)
  };

  hyundai_common_init(param);
  hyundai_legacy = true;
  hyundai_longitudinal = false;
  hyundai_gas_interceptor = false;
  hyundai_gas_interceptor_remapped = false;
  hyundai_fca11_brake_test = false;
  hyundai_fca11_rolling_test = false;
  hyundai_fca11_long = false;
  hyundai_lkas_park_test = false;
  hyundai_camera_scc = false;
  return BUILD_SAFETY_CFG(hyundai_legacy_rx_checks, HYUNDAI_TX_MSGS);
}

const safety_hooks hyundai_hooks = {
  .init = hyundai_init,
  .rx = hyundai_rx_hook,
  .tx = hyundai_tx_hook,
  .fwd = hyundai_fwd_hook,
  .get_counter = hyundai_get_counter,
  .get_checksum = hyundai_get_checksum,
  .compute_checksum = hyundai_compute_checksum,
};

const safety_hooks hyundai_legacy_hooks = {
  .init = hyundai_legacy_init,
  .rx = hyundai_rx_hook,
  .tx = hyundai_tx_hook,
  .get_counter = hyundai_get_counter,
  .get_checksum = hyundai_get_checksum,
  .compute_checksum = hyundai_compute_checksum,
};
