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
      acc_main_on = GET_BIT(msg, 25U);
      bool cruise_engaged = GET_BIT(msg, 26U);
      hyundai_common_cruise_state_check(cruise_engaged);
    }
  }

  hyundai_common_reset_acc_main_on_mismatches();
}

static bool hyundai_tx_hook(const CANPacket_t *msg) {
  const TorqueSteeringLimits HYUNDAI_STEERING_LIMITS = HYUNDAI_LIMITS(384, 3, 7);
  const TorqueSteeringLimits HYUNDAI_STEERING_LIMITS_ALT = HYUNDAI_LIMITS(270, 2, 3);
  const TorqueSteeringLimits HYUNDAI_STEERING_LIMITS_ALT_2 = HYUNDAI_LIMITS(170, 2, 3);

  bool tx = true;

  // FCA11: Block any potential actuation
  if (msg->addr == 0x38DU) {
    int CR_VSM_DecCmd = msg->data[1];
    bool FCA_CmdAct = GET_BIT(msg, 20U);
    bool CF_VSM_DecCmdAct = GET_BIT(msg, 31U);

    if (!hyundai_fca11_brake_test) {
      if ((CR_VSM_DecCmd != 0) || FCA_CmdAct || CF_VSM_DecCmdAct) {
        tx = false;
      }
    } else {
      // TEST-ONLY (parked brake-injection research). Deliberately independent of controls_allowed: openpilot is
      // disengaged during the test. Allowed: CR_VSM_DecCmd <= cap, CF_VSM_Prefill (bit 0), FCA_CmdAct (bit 20),
      // CF_VSM_DecCmdAct (bit 31). Always blocked: decel above the cap, CF_VSM_HBACmd (bits 1-2, brake assist),
      // FCA_StopReq (bit 21), and ANY actuation (decel/act/prefill) while the vehicle is moving.
      int CF_VSM_HBACmd = (msg->data[0] >> 1) & 0x3U;
      bool CF_VSM_Prefill = GET_BIT(msg, 0U);
      bool FCA_StopReq = GET_BIT(msg, 21U);
      bool actuation = (CR_VSM_DecCmd != 0) || FCA_CmdAct || CF_VSM_DecCmdAct || CF_VSM_Prefill;

      bool violation = (CR_VSM_DecCmd > HYUNDAI_FCA11_TEST_MAX_DEC);
      violation |= (CF_VSM_HBACmd != 0);
      violation |= FCA_StopReq;
      violation |= actuation && vehicle_moving;
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

    bool violation = false;

    violation |= longitudinal_accel_checks(desired_accel_raw, HYUNDAI_LONG_LIMITS);
    violation |= longitudinal_accel_checks(desired_accel_val, HYUNDAI_LONG_LIMITS);
    if (!hyundai_escc) {
      violation |= (aeb_decel_cmd != 0);
      violation |= aeb_req;
    }

    if (violation) {
      tx = false;
    }
  }

  // LKA STEER: safety check
  if (msg->addr == 0x340U) {
    int desired_torque = ((GET_BYTES(msg, 0, 4) >> 16) & 0x7ffU) - 1024U;
    bool steer_req = GET_BIT(msg, 27U);

    const TorqueSteeringLimits limits = hyundai_alt_limits_2 ? HYUNDAI_STEERING_LIMITS_ALT_2 :
                                        hyundai_alt_limits ? HYUNDAI_STEERING_LIMITS_ALT : HYUNDAI_STEERING_LIMITS;

    if (steer_torque_cmd_checks(desired_torque, steer_req, limits)) {
      tx = false;
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
  }

  // BUTTONS: used for resume spamming and cruise cancellation
  // (also policed with the gas interceptor: openpilot only sends CANCEL there, to stop the factory conventional cruise)
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

  // base TX + the pedal command only: no SCC/FCA/radar-UDS actuation messages
  static const CanMsg HYUNDAI_GAS_INTERCEPTOR_TX_MSGS[] = {
    HYUNDAI_COMMON_TX_MSGS(0)
    {0x200, 0, 6, .check_relay = false},  // GAS_COMMAND Bus 0 (comma pedal)
  };

  // same, remapped pedal IDs: GAS_COMMAND_R on 0x700 INSTEAD OF 0x200 (never both)
  static const CanMsg HYUNDAI_GAS_INTERCEPTOR_REMAPPED_TX_MSGS[] = {
    HYUNDAI_COMMON_TX_MSGS(0)
    {0x700, 0, 6, .check_relay = false},  // GAS_COMMAND_R Bus 0 (comma pedal, remapped IDs)
  };

  // TEST-ONLY FCA11 brake test: base TX + FCA11 with check_relay=true, unlike HYUNDAI_LONG_TX_MSGS (false). That makes
  // fwd_hook drop the stock camera's FCA11 (bus 2 -> 0), so there is never more than one FCA11 source on the car bus.
  // It is relay-fault safe on the target car (2022 Elantra N, non-SCC): full route logs show 0x38D received ONLY on
  // bus 2 (camera side, src 2/128), never on bus 0, and stock_ecu_check (safety.h) only trips relay_malfunction when a
  // check_relay addr is RECEIVED on the same bus as the TX entry (bus 0).
  static const CanMsg HYUNDAI_FCA11_TEST_TX_MSGS[] = {
    HYUNDAI_COMMON_TX_MSGS(0)
    {0x38D, 0, 8, .check_relay = true},  // FCA11 Bus 0 (replaces the stock camera's)
  };

  hyundai_common_init(param);
  hyundai_legacy = false;

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

    SET_TX_MSGS(HYUNDAI_TX_MSGS, ret);
    if (hyundai_fca11_brake_test) {  // hyundai_common_init: non-SCC ICE only, never with any openpilot longitudinal
      SET_TX_MSGS(HYUNDAI_FCA11_TEST_TX_MSGS, ret);
      if (hyundai_has_lda_button) {
        SET_RX_CHECKS(hyundai_non_scc_lda_button_addr_checks, ret);
      } else {
        SET_RX_CHECKS(hyundai_non_scc_addr_checks, ret);
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
  hyundai_camera_scc = false;
  return BUILD_SAFETY_CFG(hyundai_legacy_rx_checks, HYUNDAI_TX_MSGS);
}

const safety_hooks hyundai_hooks = {
  .init = hyundai_init,
  .rx = hyundai_rx_hook,
  .tx = hyundai_tx_hook,
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
