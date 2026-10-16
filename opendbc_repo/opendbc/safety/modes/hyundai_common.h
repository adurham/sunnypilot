#pragma once

#include "opendbc/safety/declarations.h"

extern uint16_t hyundai_canfd_crc_lut[256];
uint16_t hyundai_canfd_crc_lut[256];

static const uint8_t HYUNDAI_PREV_BUTTON_SAMPLES = 8;  // roughly 160 ms

extern const uint32_t HYUNDAI_STANDSTILL_THRSLD;
const uint32_t HYUNDAI_STANDSTILL_THRSLD = 12;  // 0.375 kph

enum {
  HYUNDAI_BTN_NONE = 0,
  HYUNDAI_BTN_RESUME = 1,
  HYUNDAI_BTN_SET = 2,
  HYUNDAI_BTN_CANCEL = 4,
};

enum {
  HYUNDAI_PARAM_SP_ESCC = 1,
  HYUNDAI_PARAM_SP_LONGITUDINAL_MAIN_CRUISE_TOGGLEABLE = 2,
  HYUNDAI_PARAM_SP_HAS_LDA_BUTTON = 4,
  HYUNDAI_PARAM_SP_NON_SCC = 8,
  HYUNDAI_PARAM_SP_GAS_INTERCEPTOR = 16,  // comma pedal longitudinal (non-SCC ICE only), see hyundai.h
  HYUNDAI_PARAM_SP_GAS_INTERCEPTOR_REMAPPED = 32,  // pedal on remapped IDs: cmd 0x700 / sensor 0x701 (needs GAS_INTERCEPTOR)
  HYUNDAI_PARAM_SP_FCA11_BRAKE_TEST = 64,  // TEST-ONLY: parked FCA11 (0x38D) brake-injection research, see hyundai.h
  HYUNDAI_PARAM_SP_FCA11_ROLLING_TEST = 128,  // TEST-ONLY: extends 64 to a low-speed rolling window (needs 64), see hyundai.h
  HYUNDAI_PARAM_SP_FCA11_LONG = 256,  // PRODUCTION (toggle-gated): FCA11 decel on top of the gas interceptor, see hyundai.h
  HYUNDAI_PARAM_SP_LKAS_PARK_TEST = 512,  // TEST-ONLY: parked LKAS11 (0x340) steering-torque sweep, see hyundai.h
  HYUNDAI_PARAM_SP_CN7_STEER_RAMP = 1024,  // Elantra N (CN7 non-SCC): LKAS11 torque rate-up 4/frame instead of 3, see hyundai.h
};

// common state
extern bool hyundai_ev_gas_signal;
bool hyundai_ev_gas_signal = false;

extern bool hyundai_hybrid_gas_signal;
bool hyundai_hybrid_gas_signal = false;

extern bool hyundai_longitudinal;
bool hyundai_longitudinal = false;

extern bool hyundai_camera_scc;
bool hyundai_camera_scc = false;

extern bool hyundai_canfd_lka_steer_msg;
bool hyundai_canfd_lka_steer_msg = false;

extern bool hyundai_alt_limits;
bool hyundai_alt_limits = false;

extern bool hyundai_fcev_gas_signal;
bool hyundai_fcev_gas_signal = false;

extern bool hyundai_alt_limits_2;
bool hyundai_alt_limits_2 = false;

// ESCC
extern bool hyundai_escc;
bool hyundai_escc = false;

extern bool hyundai_longitudinal_main_cruise_toggleable;
bool hyundai_longitudinal_main_cruise_toggleable = false;

extern bool hyundai_has_lda_button;
bool hyundai_has_lda_button = false;

extern bool hyundai_non_scc;
bool hyundai_non_scc = false;

// CN7 (Elantra N, non-SCC) faster steering-torque ramp: max_rate_up 4 (default HKG 3). Same 384 max, same rate-down 7,
// same max_rt_delta 112 / 250 ms (4 x 25 frames = 100 < 112, so 4 is the largest rate the real-time check permits).
// Honored only together with NON_SCC and never with ALT_LIMITS / ALT_LIMITS_2 (see hyundai.h).
extern bool hyundai_cn7_steer_ramp;
bool hyundai_cn7_steer_ramp = false;

// Gas interceptor (comma pedal) longitudinal. Deliberately a separate mode from hyundai_longitudinal:
// hyundai_longitudinal selects the SCC11/SCC12/SCC14/FCA11/radar-UDS TX allowlist, which pedal-long must NOT get.
// Pedal-long only adds 0x200 (GAS_COMMAND) TX + 0x201 (GAS_SENSOR) RX, and uses button-based controls_allowed.
extern bool hyundai_gas_interceptor;
bool hyundai_gas_interceptor = false;

// Pedal CAN ID dialect, only meaningful while hyundai_gas_interceptor. Standard comma pedal: GAS_COMMAND 0x200 TX /
// GAS_SENSOR 0x201 RX. Remapped (custom pedal firmware, +0x500 to dodge the 0x200 EMS20 address): 0x700 TX / 0x701 RX.
// Exactly one dialect is active: the other dialect's command is not in the TX allowlist and its sensor is not parsed.
extern bool hyundai_gas_interceptor_remapped;
bool hyundai_gas_interceptor_remapped = false;

// TEST-ONLY, NOT FOR ROAD USE. Parked/standstill research mode for FCA11 (0x38D) brake-message injection on non-SCC
// cars (forward camera -> ESC). When armed, panda owns FCA11 on bus 0 (the stock camera's FCA11 is no longer forwarded)
// and openpilot may request a small, capped CR_VSM_DecCmd at standstill only. Mutually exclusive with every
// openpilot longitudinal mode (SCC-replacement and the gas interceptor). Limits/rules: hyundai_tx_hook.
extern bool hyundai_fca11_brake_test;
bool hyundai_fca11_brake_test = false;

// TEST-ONLY extension of hyundai_fca11_brake_test (never on its own): instead of standstill-only, the SAME capped decel
// is allowed ONLY inside a low-speed rolling window (D, no pedals, fresh inputs), with a latching cut. Rules and per-arm
// state: hyundai.h (hyundai_fca11_rolling_*). Never set on CAN-FD / legacy.
extern bool hyundai_fca11_rolling_test;
bool hyundai_fca11_rolling_test = false;

// PRODUCTION FCA11 longitudinal braking (HYUNDAI_PARAM_SP_FCA11_LONG, bit 256), a fork feature armed by CarParamsSP
// from the HyundaiFca11Brake param (default OFF). Unlike the TEST bits (64/128), this rides ON TOP of the gas
// interceptor (pedal-long): the pedal TX/RX path stays fully active and openpilot is ENGAGED (controls_allowed +
// heartbeat_engaged) while it sends FCA11 decel. All rules and per-arm state: hyundai.h (hyundai_fca11_long_*).
// Never set on CAN-FD / legacy; never together with the test bits (64/128) or SCC-replacement longitudinal.
extern bool hyundai_fca11_long;
bool hyundai_fca11_long = false;

// TEST-ONLY, NOT FOR ROAD USE (HYUNDAI_PARAM_SP_LKAS_PARK_TEST, bit 512). Parked steering-torque sweep: armed ONLY by the
// steer-test runner (raw 0xdf/0xdc, openpilot stopped). While armed the ONLY transmittable message is LKAS11 (0x340),
// and only while the car is parked (gear P or N, every wheel at standstill, inputs fresh), within a 60 s cap per arm,
// with |torque| <= 384 and the +3/-7 per-frame rate law enforced by panda itself; any MDPS12 fault bit, driver torque,
// gas, wheel motion, gear change or the cap LATCHES a cut until the next init. Non-SCC ICE only, never with any
// openpilot longitudinal mode or the FCA11 test/long bits. Rules and per-arm state: hyundai.h (hyundai_lkas_park_*).
extern bool hyundai_lkas_park_test;
bool hyundai_lkas_park_test = false;

// Gas interceptor only: factory (conventional) cruise MAIN armed, EMS16 CRUISE_LAMP_M. The factory cruise can only engage
// while MAIN is armed, and openpilot never sends CLU11 in this mode (no CANCEL to fight it), so pedal-long is locked out
// while it is armed: controls_allowed is cleared and no button may grant it (fail closed, never two speed controllers).
extern bool hyundai_factory_main_on;
bool hyundai_factory_main_on = false;

// Gas interceptor only: the steering-wheel pause/resume button (CLU11 CF_Clu_CruiseSwState 4, "CANCEL" in the DBC) is
// the ONLY way controls_allowed is granted in pedal mode. The up/down arrows (CruiseSwState 1 RES/ACCEL and 2 SET/DECEL)
// only change openpilot's set speed and never grant or clear controls here (buttons-v3; openpilot's buttonEnable mirrors
// this: opendbc/car/hyundai/carstate.py update_button_enable).
// Holding pause/resume clears controls (the disengage). Its debounced release (HYUNDAI_PAUSE_RELEASE_SAMPLES consecutive
// NONE samples) grants controls_allowed, but ONLY when the press started with the brake released and the factory MAIN
// off, the brake was not pressed at any sample of the press or its debounce, and MAIN is still off at the release. The
// grant is a PERMISSION superset of openpilot: openpilot itself only engages on it when it was disengaged at the press
// start (pause_resume.py); after a disengage press openpilot stays off, sends zero gas, and the 1 Hz heartbeat clears the
// unused grant. Deliberately NOT conditioned on controls_allowed (route 00000128 @139.2 s: a stale grant made panda judge
// a press 'disengage' while openpilot judged it 'resume'). Gas may be held (engages into override). Speed is not checked:
// there is no engage floor at all (green-light resume from standstill). Nothing else grants controls: no timer, brake
// release, speed recovery or up/down press.
// MUST equal PAUSE_RELEASE_SAMPLES in opendbc/sunnypilot/car/hyundai/pause_resume.py (a unit test enforces this).
#define HYUNDAI_PAUSE_RELEASE_SAMPLES 3U  // CLU11 is 50 Hz: 60 ms
static bool hyundai_pause_armed;             // the current/last press will resume on its debounced release
static uint8_t hyundai_pause_released_cnt;   // consecutive NONE samples since the button was last 4

// ********** Gas interceptor: timed factory-cruise CANCEL (the ONLY CLU11 this panda ever transmits in pedal mode) **********
// Safety net for the MAIN lockout: if the factory (non-adaptive) cruise reports ACTIVE (EMS16 CRUISE_LAMP_S) while
// openpilot longitudinal is engaged (controls_allowed AND the openpilot heartbeat says engaged), panda cancels it with
// CLU11 (0x4F1) CF_Clu_CruiseSwState = 4 frames that it builds and transmits ITSELF, from the bus-0 RX path:
//  * Timing: one frame, queued synchronously while a CLUSTER CLU11 is being received (rx hook), i.e. right after that
//    frame's EOF. The cluster sends CLU11 every 19.8 ms, so its next frame is ~19.8 ms away and the two can never contend
//    for the same arbitration slot (routes 11c/123: every bus error, bus-off and pedal SCE onset came from our old,
//    unsynchronised CLU11 frames arbitrating together with the cluster's). No RX of a cluster CLU11 -> no TX, ever.
//  * Payload: the cluster's frame just received, with ONLY the button field set to 4, CF_Clu_AliveCnt1 = cluster + 1 and
//    CF_Clu_ParityBit1 recomputed over the final payload (the old create_clu11 copied the cluster's parity after
//    rewriting the counter: 1998/3983 of its frames on 11c had a wrong parity bit).
//  * Schedule: up to 4 attempts of 3 frames (a 3-sample press, like the driver's; odd because CruiseSwState 4 is a
//    pause/resume TOGGLE on this ECM; one well-formed frame is NOT enough on the logs), at most ONE frame per cluster
//    frame, >= 400 ms between attempts to watch CRUISE_LAMP_S, at most 12 frames, >= 15 ms apart, stop on the
//    first EMS16 with CRUISE_LAMP_S = 0, give up 2 s after the trigger (openpilot raises an audible alert).
//    Every frame needs FRESH positive evidence that the factory cruise is still on: the latest bus-0 EMS16 (10 ms period)
//    is <= 30 ms old and says CRUISE_LAMP_S = 1. A later attempt additionally needs >= 2 EMS16 frames with
//    CRUISE_LAMP_S = 1 received >= 250 ms after the previous attempt's last frame (lamp confirmation). Toggle-back margin
//    (integration-3b-report.md, routes 11c/123/127/128): the lamp falls <= 150.5 ms after the frame the ECM acted on
//    (44/45 <= 50.5 ms), and the ECM's torque request drops <= 99.7 ms before the lamp, so the confirmation can only
//    complete while the cruise is still really on. Inside one attempt, frames 2-3 (+19.8 / +39.6 ms) may still land
//    after the ECM acted on frame 1: unavoidable for a press shape, and the same exposure as a driver's 3-frame press.
//    On the logs no 4 alone ever re-engaged the cruise sooner than 1.74 s after it went off (331 trailing frames 0-995 ms
//    after a fall: 0 re-engagements without a driver SET/RES). A driver
//    SET/RES/pause press on the cluster aborts it (the driver is operating the factory cruise himself); frames with
//    MAIN held are skipped. No frame while the brake is pressed (the ECM cancels on the brake itself).
//  * Not reachable over USB: 0x4F1 is not in the pedal-mode TX allowlist, so openpilot can't send CLU11 at all.
// The lockout itself is unchanged: controls_allowed is still cleared on the same EMS16 (two speed controllers never run
// together), so the trigger is evaluated on that EMS16 frame BEFORE the lockout clears it.
#define HYUNDAI_FC_CANCEL_BUTTON 4U
#define HYUNDAI_FC_CANCEL_WINDOW_US 2000000U       // give up this long after the trigger
#define HYUNDAI_FC_CANCEL_MIN_GAP_US 15000U        // hard rate cap: never two frames closer than this (cluster: 19.8 ms)
#define HYUNDAI_FC_CANCEL_ATTEMPT_GAP_US 400000U   // between attempts: > worst lamp lag (150.5 ms) + 2 confirmations, << 1.74 s
#define HYUNDAI_FC_CANCEL_LAMP_LAG_US 250000U      // lamp confirmation samples count only this long after an attempt
#define HYUNDAI_FC_CANCEL_LAMP_CONFIRM 2U          // EMS16 samples with CRUISE_LAMP_S = 1 needed before the next attempt
#define HYUNDAI_FC_CANCEL_LAMP_FRESH_US 30000U     // the latest EMS16 (10 ms period) must be this recent for every frame
#define HYUNDAI_FC_CANCEL_ATTEMPTS 4U
#define HYUNDAI_FC_CANCEL_FRAMES_PER_ATTEMPT 3U    // odd (toggle), = a short driver press (3 frames of 4, ~60 ms)
#define HYUNDAI_FC_CANCEL_MAX_FRAMES 12U           // == ATTEMPTS * FRAMES_PER_ATTEMPT (independent hard cap)

// provided by the panda board (board/drivers/can_common.h); libsafety provides recording stubs
void can_send(CANPacket_t *to_push, uint8_t bus_number, bool skip_tx_hook);
void can_set_checksum(CANPacket_t *packet);

static bool hyundai_fc_active;                // EMS16 CRUISE_LAMP_S, latest bus-0 sample
static bool hyundai_fc_cancel_armed;          // a cancel episode is running
static bool hyundai_fc_cancel_done;           // episode over (gave up / aborted): no new one until CRUISE_LAMP_S falls
static uint32_t hyundai_fc_cancel_start_ts;   // trigger time
static uint32_t hyundai_fc_cancel_next_us;    // earliest TX, microseconds after the trigger (attempt gap)
static uint32_t hyundai_fc_cancel_last_tx_ts;
static uint8_t hyundai_fc_cancel_attempt;
static uint8_t hyundai_fc_cancel_attempt_frames;
static uint8_t hyundai_fc_cancel_frames;      // frames transmitted this episode
static uint32_t hyundai_fc_ems16_ts;          // time of the latest bus-0 EMS16 (lamp freshness)
static uint8_t hyundai_fc_cancel_confirm;     // CRUISE_LAMP_S = 1 samples >= LAMP_LAG after the last attempt

static void hyundai_fc_cancel_reset(void) {
  hyundai_fc_active = false;
  hyundai_fc_cancel_armed = false;
  hyundai_fc_cancel_done = false;
  hyundai_fc_cancel_start_ts = 0U;
  hyundai_fc_cancel_next_us = 0U;
  hyundai_fc_cancel_last_tx_ts = 0U;
  hyundai_fc_cancel_attempt = 0U;
  hyundai_fc_cancel_attempt_frames = 0U;
  hyundai_fc_cancel_frames = 0U;
  hyundai_fc_ems16_ts = 0U;
  hyundai_fc_cancel_confirm = 0U;
}

// CF_Clu_ParityBit1 (bit 5) = parity over CF_Clu_CruiseSwState (bits 0-2) + CF_Clu_CruiseSwMain (bit 3) +
// CF_Clu_AliveCnt1 (byte 3 bits 4-7): bit5 == popcount(those 8 bits) % 2. Holds for 174557/174557 cluster frames on
// routes 11c/123/128 (CF_Clu_SldMainSW, bit 4, was 0 in all of them; frames with it set are never copied).
static uint8_t hyundai_clu11_parity(uint8_t byte0, uint8_t byte3) {
  uint8_t bits = (uint8_t)((byte0 & 0x0FU) | (byte3 & 0xF0U));
  uint8_t n = 0U;
  for (uint8_t i = 0U; i < 8U; i++) {
    n += (uint8_t)((bits >> i) & 1U);
  }
  return (uint8_t)(n & 1U);
}

// TX policy for the firmware-built frame (the only CLU11 source in pedal mode). Every condition is re-checked on the
// final packet, independent of how it was built.
static bool hyundai_fc_cancel_tx_allowed(const CANPacket_t *pkt, const CANPacket_t *cluster) {
  bool ok = hyundai_gas_interceptor && hyundai_fc_cancel_armed && hyundai_fc_active && !relay_malfunction;
  ok = ok && (pkt->addr == 0x4F1U) && (pkt->bus == 0U) && (pkt->extended == 0U) && (pkt->fd == 0U) && (GET_LEN(pkt) == 4U);
  ok = ok && ((pkt->data[0] & 0x1FU) == HYUNDAI_FC_CANCEL_BUTTON);  // CANCEL, MAIN 0, SldMainSW 0
  ok = ok && (((pkt->data[0] >> 5) & 1U) == hyundai_clu11_parity(pkt->data[0], pkt->data[3]));
  ok = ok && ((pkt->data[3] >> 4) == (((cluster->data[3] >> 4) + 1U) & 0xFU));
  ok = ok && ((pkt->data[0] & 0xC0U) == (cluster->data[0] & 0xC0U)) && (pkt->data[1] == cluster->data[1]) &&
            (pkt->data[2] == cluster->data[2]) && ((pkt->data[3] & 0x0FU) == (cluster->data[3] & 0x0FU));
  return ok;
}

// EMS16 (bus 0), gas interceptor only. Must run BEFORE the MAIN/active lockout clears controls_allowed.
static void hyundai_fc_cancel_ems16(bool factory_active) {
  uint32_t ts = microsecond_timer_get();
  if (!factory_active) {
    // factory cruise off: success (or never needed). Re-arm.
    hyundai_fc_cancel_armed = false;
    hyundai_fc_cancel_done = false;
  } else if (!hyundai_fc_cancel_armed && !hyundai_fc_cancel_done && controls_allowed && heartbeat_engaged) {
    hyundai_fc_cancel_armed = true;
    hyundai_fc_cancel_start_ts = ts;
    hyundai_fc_cancel_next_us = 0U;
    hyundai_fc_cancel_attempt = 0U;
    hyundai_fc_cancel_attempt_frames = 0U;
    hyundai_fc_cancel_frames = 0U;
    hyundai_fc_cancel_confirm = 0U;
  } else if (hyundai_fc_cancel_armed && (safety_get_ts_elapsed(ts, hyundai_fc_cancel_start_ts) >= HYUNDAI_FC_CANCEL_WINDOW_US)) {
    hyundai_fc_cancel_armed = false;  // give up (openpilot alerts on its own: CRUISE_LAMP_S still on)
    hyundai_fc_cancel_done = true;
  } else if (hyundai_fc_cancel_armed && (hyundai_fc_cancel_attempt > 0U) && (hyundai_fc_cancel_attempt_frames == 0U) &&
             (safety_get_ts_elapsed(ts, hyundai_fc_cancel_last_tx_ts) >= HYUNDAI_FC_CANCEL_LAMP_LAG_US) &&
             (hyundai_fc_cancel_confirm < HYUNDAI_FC_CANCEL_LAMP_CONFIRM)) {
    // still on well after the last attempt could have shown on the lamp: confirmation sample for the next attempt
    hyundai_fc_cancel_confirm += 1U;
  } else {
  }
  hyundai_fc_ems16_ts = ts;
  hyundai_fc_active = factory_active;
}

// cluster CLU11 just received on bus 0 (gas interceptor only): the only place a CLU11 is ever transmitted.
// INVARIANT: only a frame the cluster put on the bus may trigger (and be copied into) a CANCEL; our own transmitted
// frame must never feed back into this function. Today that holds without the guard below: the panda board's TX echo
// (process_can, returned = 1) goes only to the USB rx queue, never through safety_rx_hook, and FDCAN does not loop a
// self-transmitted frame into the RX FIFO (can_loopback defaults to false). The guard makes it hold by construction, so
// a future board/harness change that routes echoes or loopback frames through the RX hook can't make panda answer its
// own CANCEL with another one (a self-sustaining loop of a pause/resume toggle).
//
// Call path (board/drivers/fdcan.h): FDCANx_IT0 IRQ -> can_rx() -> safety_rx_hook() -> hyundai_rx_hook() -> here ->
// can_send(skip_tx_hook) -> can_push() onto the bus-0 TX ring (ENTER_CRITICAL) -> process_can(0) (ENTER_CRITICAL) moves
// at most ONE ring element into the FDCAN TX FIFO if it has room, else returns; the TX-FIFO-empty IRQ (IT1) drains the
// rest. The ring is FIFO, shared with USB-sent pedal/LKAS frames, so our frame keeps its order relative to those but can
// wait behind them: worst-case latency is the ring backlog at that moment, not the 30-400 us of an idle ring. A frame
// that waits longer than one cluster period (19.8 ms) loses the collision-free slot; the 15 ms rate cap is measured
// from queue time, not wire time. Road checklist: integration-3b-report.md (TX latency under USB load).
static void hyundai_fc_cancel_cluster_frame(const CANPacket_t *msg) {
  if (hyundai_fc_cancel_armed && (msg->returned == 0U)) {
    uint32_t ts = microsecond_timer_get();
    uint32_t elapsed = safety_get_ts_elapsed(ts, hyundai_fc_cancel_start_ts);
    bool driver_cruise_button = ((msg->data[0] & 0x07U) != 0U);  // CruiseSwState: SET / RES / pause-resume
    bool driver_main_button = ((msg->data[0] & 0x18U) != 0U);    // CruiseSwMain / SldMainSW
    if (!hyundai_fc_active) {
      hyundai_fc_cancel_armed = false;
    } else if ((elapsed >= HYUNDAI_FC_CANCEL_WINDOW_US) || driver_cruise_button) {
      // window over, or the driver is operating the factory cruise himself (his own pause/resume press is a toggle
      // too: never interleave ours with it). Done until CRUISE_LAMP_S falls.
      hyundai_fc_cancel_armed = false;
      hyundai_fc_cancel_done = true;
    } else {
      // MAIN held (e.g. the MAIN press that made the lamp come on): never copy it, wait for a released frame
      // only a bus-0 CLU11 from the cluster (the RX whitelist already guarantees this; checked again here)
      bool send = (msg->addr == 0x4F1U) && (msg->bus == 0U) && (GET_LEN(msg) == 4U) && !driver_main_button &&
                  !brake_pressed && (elapsed >= hyundai_fc_cancel_next_us) &&
                  (hyundai_fc_cancel_attempt < HYUNDAI_FC_CANCEL_ATTEMPTS) &&
                  (hyundai_fc_cancel_frames < HYUNDAI_FC_CANCEL_MAX_FRAMES) &&
                  (((msg->data[0] >> 5) & 1U) == hyundai_clu11_parity(msg->data[0], msg->data[3]));
      // fresh lamp: CRUISE_LAMP_S = 1 (checked above) from an EMS16 at most LAMP_FRESH old; a stale lamp skips this
      // cluster frame (no abort), so a lost EMS16 can never let a frame through on old information
      send = send && (safety_get_ts_elapsed(ts, hyundai_fc_ems16_ts) <= HYUNDAI_FC_CANCEL_LAMP_FRESH_US);
      // a new attempt only after the lamp confirmed (twice, after LAMP_LAG) that the previous one did not cancel
      send = send && ((hyundai_fc_cancel_attempt == 0U) || (hyundai_fc_cancel_attempt_frames > 0U) ||
                      (hyundai_fc_cancel_confirm >= HYUNDAI_FC_CANCEL_LAMP_CONFIRM));
      if (send && (hyundai_fc_cancel_frames > 0U)) {
        send = safety_get_ts_elapsed(ts, hyundai_fc_cancel_last_tx_ts) >= HYUNDAI_FC_CANCEL_MIN_GAP_US;
      }
      if (send) {
        CANPacket_t pkt;
        pkt.fd = 0U;
        pkt.returned = 0U;
        pkt.rejected = 0U;
        pkt.extended = 0U;
        pkt.addr = 0x4F1U;
        pkt.bus = 0U;
        pkt.data_len_code = msg->data_len_code;
        uint8_t counter = (uint8_t)(((msg->data[3] >> 4) + 1U) & 0xFU);
        pkt.data[0] = (uint8_t)((msg->data[0] & 0xC0U) | HYUNDAI_FC_CANCEL_BUTTON);
        pkt.data[1] = msg->data[1];
        pkt.data[2] = msg->data[2];
        pkt.data[3] = (uint8_t)((uint8_t)(counter << 4) | (msg->data[3] & 0x0FU));
        pkt.data[0] |= (uint8_t)(hyundai_clu11_parity(pkt.data[0], pkt.data[3]) << 5);
        if (hyundai_fc_cancel_tx_allowed(&pkt, msg)) {
          can_set_checksum(&pkt);
          can_send(&pkt, 0U, true);  // policy enforced right above; CLU11 is deliberately not in the TX allowlist
          hyundai_fc_cancel_frames += 1U;
          hyundai_fc_cancel_attempt_frames += 1U;
          hyundai_fc_cancel_last_tx_ts = ts;
          if (hyundai_fc_cancel_attempt_frames >= HYUNDAI_FC_CANCEL_FRAMES_PER_ATTEMPT) {
            hyundai_fc_cancel_attempt += 1U;
            hyundai_fc_cancel_attempt_frames = 0U;
            hyundai_fc_cancel_confirm = 0U;
            hyundai_fc_cancel_next_us = elapsed + HYUNDAI_FC_CANCEL_ATTEMPT_GAP_US;
          }
        }
      }
    }
  }
}

static uint8_t hyundai_last_button_interaction;  // button messages since the user pressed an enable button

static bool main_button_prev;
static bool acc_main_on_prev;
static bool acc_main_on_tx;
static uint32_t acc_main_on_mismatches;

// Gas interceptor: called on every RX message (hyundai_rx_hook), so a brake press is caught even when it falls
// between two CLU11 samples. Brake during a pause/resume press (or its release debounce) cancels that resume.
// FCA11 long (bit 256) ALSO uses this: a pedal fault means the pedal is in passthrough (driver's pedal straight
// to the ECU) and openpilot throttle authority is gone, so panda latches the FCA11 long cut (hyundai.h) - mixing
// an ESC brake with a throttle we no longer control is the two-controllers case. Defined BEFORE hyundai_common_init
// (which clears it) and before hyundai_rx_hook reads it.
extern bool hyundai_interceptor_pedal_fault;
bool hyundai_interceptor_pedal_fault = false;

void hyundai_common_init(uint16_t param) {
  const uint16_t HYUNDAI_PARAM_EV_GAS = 1;
  const uint16_t HYUNDAI_PARAM_HYBRID_GAS = 2;
  const uint16_t HYUNDAI_PARAM_CAMERA_SCC = 8;
  const uint16_t HYUNDAI_PARAM_CANFD_LKA_STEER_MSG = 16;
  const uint16_t HYUNDAI_PARAM_ALT_LIMITS = 64; // TODO: shift this down with the rest of the common flags
  const uint16_t HYUNDAI_PARAM_FCEV_GAS = 256;
  const uint16_t HYUNDAI_PARAM_ALT_LIMITS_2 = 512;

  hyundai_ev_gas_signal = GET_FLAG(param, HYUNDAI_PARAM_EV_GAS);
  hyundai_hybrid_gas_signal = !hyundai_ev_gas_signal && GET_FLAG(param, HYUNDAI_PARAM_HYBRID_GAS);
  hyundai_camera_scc = GET_FLAG(param, HYUNDAI_PARAM_CAMERA_SCC);
  hyundai_canfd_lka_steer_msg = GET_FLAG(param, HYUNDAI_PARAM_CANFD_LKA_STEER_MSG);
  hyundai_alt_limits = GET_FLAG(param, HYUNDAI_PARAM_ALT_LIMITS);
  hyundai_fcev_gas_signal = GET_FLAG(param, HYUNDAI_PARAM_FCEV_GAS);
  hyundai_alt_limits_2 = GET_FLAG(param, HYUNDAI_PARAM_ALT_LIMITS_2);

  hyundai_escc = GET_FLAG(current_safety_param_sp, HYUNDAI_PARAM_SP_ESCC);
  hyundai_longitudinal_main_cruise_toggleable = GET_FLAG(current_safety_param_sp, HYUNDAI_PARAM_SP_LONGITUDINAL_MAIN_CRUISE_TOGGLEABLE);
  hyundai_has_lda_button = GET_FLAG(current_safety_param_sp, HYUNDAI_PARAM_SP_HAS_LDA_BUTTON);
  hyundai_non_scc = GET_FLAG(current_safety_param_sp, HYUNDAI_PARAM_SP_NON_SCC);
  hyundai_cn7_steer_ramp = GET_FLAG(current_safety_param_sp, HYUNDAI_PARAM_SP_CN7_STEER_RAMP);

  hyundai_last_button_interaction = HYUNDAI_PREV_BUTTON_SAMPLES;

  main_button_prev = false;
  hyundai_factory_main_on = false;
  hyundai_pause_armed = false;
  hyundai_pause_released_cnt = HYUNDAI_PAUSE_RELEASE_SAMPLES;
  hyundai_fc_cancel_reset();
  hyundai_interceptor_pedal_fault = false;
  acc_main_on_prev = false;
  acc_main_on_tx = false;
  acc_main_on_mismatches = 0U;

#ifdef ALLOW_DEBUG
  const uint16_t HYUNDAI_PARAM_LONGITUDINAL = 4;
  hyundai_longitudinal = GET_FLAG(param, HYUNDAI_PARAM_LONGITUDINAL);
#else
  hyundai_longitudinal = false;
#endif

  // Pedal-long and the FCA11 brake test are only defined for non-SCC ICE cars (EMS16 gas/cruise signals, no SCC ECU to
  // fight with) and never coexist with SCC-replacement longitudinal. Safety modes that don't support them reset them after init.
  // cppcheck-suppress knownConditionTrueFalse ; hyundai_longitudinal is always false in non-ALLOW_DEBUG builds
  const bool non_scc_ice_no_long = hyundai_non_scc && !hyundai_longitudinal && !hyundai_camera_scc &&
                                   !hyundai_ev_gas_signal && !hyundai_hybrid_gas_signal && !hyundai_fcev_gas_signal;
  hyundai_gas_interceptor = GET_FLAG(current_safety_param_sp, HYUNDAI_PARAM_SP_GAS_INTERCEPTOR) && non_scc_ice_no_long;
  hyundai_gas_interceptor_remapped = hyundai_gas_interceptor &&
                                     GET_FLAG(current_safety_param_sp, HYUNDAI_PARAM_SP_GAS_INTERCEPTOR_REMAPPED);

  // FCA11 brake test (TEST-ONLY) takes precedence over the pedal: while armed there is no openpilot accel path at all
  // (no 0x200/0x700 TX).
  hyundai_fca11_brake_test = GET_FLAG(current_safety_param_sp, HYUNDAI_PARAM_SP_FCA11_BRAKE_TEST) && non_scc_ice_no_long;
  if (hyundai_fca11_brake_test) {
    hyundai_gas_interceptor = false;
    hyundai_gas_interceptor_remapped = false;
  }
  // rolling extension: only on top of an armed bit 64 (so non-SCC ICE, no openpilot longitudinal, no pedal)
  hyundai_fca11_rolling_test = hyundai_fca11_brake_test && GET_FLAG(current_safety_param_sp, HYUNDAI_PARAM_SP_FCA11_ROLLING_TEST);

  // parked LKAS11 sweep (TEST-ONLY, bit 512): non-SCC ICE, no openpilot longitudinal, never together with the FCA11
  // test bits (64/128 win: one test mode at a time). Like the FCA11 test it removes the pedal (no 0x200/0x700 TX) and
  // therefore FCA11 long (bit 256, which needs the pedal) as well.
  hyundai_lkas_park_test = GET_FLAG(current_safety_param_sp, HYUNDAI_PARAM_SP_LKAS_PARK_TEST) && non_scc_ice_no_long &&
                           !hyundai_fca11_brake_test;
  if (hyundai_lkas_park_test) {
    hyundai_gas_interceptor = false;
    hyundai_gas_interceptor_remapped = false;
  }

  // PRODUCTION FCA11 long braking (bit 256) rides ON TOP of the gas interceptor: it needs the pedal active
  // (non-SCC ICE, no SCC-replacement longitudinal) and is mutually exclusive with the TEST bits (64/128), which
  // take over FCA11 for runner-driven research with openpilot disengaged. While armed, panda polices FCA11 decel
  // with its own window/cap/rate/budget rules (hyundai.h) IN ADDITION to the pedal checks; nothing about the pedal
  // path changes.
  hyundai_fca11_long = hyundai_gas_interceptor && !hyundai_fca11_brake_test &&
                       GET_FLAG(current_safety_param_sp, HYUNDAI_PARAM_SP_FCA11_LONG);
}

void hyundai_common_cruise_state_check(const bool cruise_engaged) {
  // some newer HKG models can re-enable after spamming cancel button,
  // so keep track of user button presses to deny engagement if no interaction

  // enter controls on rising edge of ACC and recent user button press, exit controls when ACC off
  // (openpilot-managed longitudinal, incl. gas interceptor, uses the buttons instead: see hyundai_common_cruise_buttons_check)
  if (!(hyundai_longitudinal || hyundai_gas_interceptor)) {
    if (cruise_engaged && !cruise_engaged_prev && (hyundai_last_button_interaction < HYUNDAI_PREV_BUTTON_SAMPLES)) {
      controls_allowed = true;
    }

    if (!cruise_engaged) {
      controls_allowed = false;
    }
    cruise_engaged_prev = cruise_engaged;
  } else if (hyundai_gas_interceptor) {
    // The factory (non-adaptive) cruise never grants controls here; it is locked out via its MAIN lamp instead
    // (hyundai_factory_main_on, hyundai.h). Tracked for visibility only: openpilot sends no CLU11 in this mode.
    cruise_engaged_prev = cruise_engaged;
  } else {
  }
}

// Gas interceptor: called on every RX message (hyundai_rx_hook), so a brake press is caught even when it falls
// between two CLU11 samples. Brake during a pause/resume press (or its release debounce) cancels that resume.
// FCA11 long (bit 256) also reads hyundai_interceptor_pedal_fault (defined above, before hyundai_common_init).
static void hyundai_gas_interceptor_pedal_check(void) {
  if (brake_pressed) {
    hyundai_pause_armed = false;
  }
}

// pause/resume button state machine, gas interceptor only. Must run BEFORE the CANCEL-held clear below: a new press
// samples whether longitudinal was engaged at the moment the press began.
static void hyundai_gas_interceptor_pause_button_check(const int cruise_button) {
  if (cruise_button == HYUNDAI_BTN_CANCEL) {
    if (hyundai_pause_released_cnt >= HYUNDAI_PAUSE_RELEASE_SAMPLES) {
      // new press (not a bounce of the previous one): may grant on its release if the brake is up and MAIN is off
      hyundai_pause_armed = !brake_pressed && !hyundai_factory_main_on;
    }
    hyundai_pause_released_cnt = 0U;
  } else if (cruise_button != HYUNDAI_BTN_NONE) {
    // rolled onto another cruise button: the pause press is abandoned (that button follows its own rules)
    hyundai_pause_armed = false;
    hyundai_pause_released_cnt = HYUNDAI_PAUSE_RELEASE_SAMPLES;
  } else if (hyundai_pause_released_cnt < HYUNDAI_PAUSE_RELEASE_SAMPLES) {
    hyundai_pause_released_cnt += 1U;
    if (hyundai_pause_armed && !brake_pressed && !hyundai_factory_main_on &&
        (hyundai_pause_released_cnt == HYUNDAI_PAUSE_RELEASE_SAMPLES)) {
      controls_allowed = true;
      hyundai_pause_armed = false;
    }
  } else {
  }
  // brake at any time during the press or its release debounce cancels the resume
  if (brake_pressed) {
    hyundai_pause_armed = false;
  }
}

void hyundai_common_cruise_buttons_check(const int cruise_button, const bool main_button) {
  if ((cruise_button == HYUNDAI_BTN_RESUME) || (cruise_button == HYUNDAI_BTN_SET) || (cruise_button == HYUNDAI_BTN_CANCEL) || main_button) {
    hyundai_last_button_interaction = 0U;
  } else {
    hyundai_last_button_interaction = SAFETY_MIN(hyundai_last_button_interaction + 1U, HYUNDAI_PREV_BUTTON_SAMPLES);
  }

  if (hyundai_longitudinal || hyundai_gas_interceptor) {
    if (hyundai_gas_interceptor) {
      // pedal mode: the pause/resume release is the only grant; up/down (RES/SET) edges never grant (buttons-v3)
      hyundai_gas_interceptor_pause_button_check(cruise_button);
    } else {
      // enter controls on falling edge of resume or set
      bool set = (cruise_button != HYUNDAI_BTN_SET) && (cruise_button_prev == HYUNDAI_BTN_SET);
      bool res = (cruise_button != HYUNDAI_BTN_RESUME) && (cruise_button_prev == HYUNDAI_BTN_RESUME);
      if (set || res) {
        controls_allowed = true;
      }
    }

    // exit controls on cancel press
    if (cruise_button == HYUNDAI_BTN_CANCEL) {
      controls_allowed = false;
    }

    // toggle main cruise state on rising edge of main cruise button
    // not for the gas interceptor: on non-SCC cars acc_main_on stays sourced from the factory cruise main lamp
    // (EMS16 CRUISE_LAMP_M), matching what sunnypilot reports as cruiseState.available, so there is a single writer
    if (main_button && !main_button_prev && hyundai_longitudinal_main_cruise_toggleable && !hyundai_gas_interceptor) {
      acc_main_on = !acc_main_on;
    }

    cruise_button_prev = cruise_button;
    main_button_prev = main_button;
  }
}

uint32_t hyundai_common_canfd_compute_checksum(const CANPacket_t *msg) {
  int len = GET_LEN(msg);
  uint32_t address = msg->addr;

  uint16_t crc = 0;

  for (int i = 2; i < len; i++) {
    crc = (crc << 8U) ^ hyundai_canfd_crc_lut[(crc >> 8U) ^ msg->data[i]];
  }

  // Add address to crc
  crc = (crc << 8U) ^ hyundai_canfd_crc_lut[(crc >> 8U) ^ ((address >> 0U) & 0xFFU)];
  crc = (crc << 8U) ^ hyundai_canfd_crc_lut[(crc >> 8U) ^ ((address >> 8U) & 0xFFU)];

  if (len == 24) {
    crc ^= 0x819dU;
  } else if (len == 32) {
    crc ^= 0x9f5bU;
  } else {

  }

  return crc;
}

// reset mismatches on rising edge of acc_main_on to avoid rare race conditions when using non-PCM main cruise state
void hyundai_common_reset_acc_main_on_mismatches(void) {
  if (acc_main_on && !acc_main_on_prev) {
    acc_main_on_mismatches = 0U;
  }

  acc_main_on_prev = acc_main_on;
}

// exit lateral controls allowed if sunnypilot and panda main cruise states are desynced
void hyundai_common_acc_main_on_sync(void) {
  if (acc_main_on && !acc_main_on_tx) {
    acc_main_on_mismatches += 1U;

    if (acc_main_on_mismatches >= 3U) {  // desync by 3 frames
      acc_main_on = false;
      mads_exit_controls(MADS_DISENGAGE_REASON_NON_PCM_ACC_MAIN_DESYNC);
    }
  } else {
    acc_main_on_mismatches = 0U;
  }
}

uint32_t get_acc_main_on_mismatches(void) {
  return acc_main_on_mismatches;
}
