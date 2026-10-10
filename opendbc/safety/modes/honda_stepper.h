#pragma once

#include "opendbc/safety/declarations.h"

// 2008-12 Honda Accord (no factory steering motor) steered by a StepperServoCAN motor through printed teeth
// zip-tied to the top of the steering wheel.
//  - bus 0: car F-CAN (500 kbps). Nothing is ever sent there. Signal locations come from the generated header below.
//  - bus 1: the motor alone. openpilot sends STEERING_COMMAND (0x22E), the motor answers with STEERING_STATUS (0x22F).
// Torque is limited against the torque the motor reports (TorqueMotorLimited).
//
// The lift lever is the only switch for lane keeping. Down, the motor has power and reports STEERING_STATUS 100 times
// a second; up, its power is cut and it is silent. Steering may be allowed once the motor has reported for 2 s without
// a gap, every message saying its controller is running (CONTROL_STATUS bit 0, set once it has started up). It stops
// being allowed after 100 ms of silence or at a message without that bit (the motor restarted). MADS takes this as its
// main switch (acc_main_on). openpilot's car port (opendbc/car/hondastepper/carstate.py) counts the same messages the
// same way, starting once this safety mode has passed one of its commands, so it turns on just after the panda does.
// The cruise MAIN button and the car's own cruise control play no part.
//
// Stand-alone (the safety param's bit 0, the default build): the comma device isn't connected to the car at all. Only
// the motor's status is checked; nothing is read from bus 0, so there is no brake or speed here, and the steering
// limits and the lift lever logic above, which come from the motor, are the checks. With the motor silent (lever up)
// no message arrives to run the rx hook, so the 1 s rx check lag (safety_tick) ends steering too; the motor itself has
// no power then.

#define HONDA_STEPPER_PARAM_STANDALONE 1U

#define HONDA_STEPPER_FCAN_BUS 0U
#define HONDA_STEPPER_SERVO_BUS 1U

#define HONDA_STEPPER_STEERING_COMMAND 0x22EU
#define HONDA_STEPPER_STEERING_STATUS 0x22FU
#define HONDA_STEPPER_COMMAND_LEN 5
#define HONDA_STEPPER_STATUS_LEN 8

// STEERING_COMMAND.STEER_MODE
#define HONDA_STEPPER_MODE_OFF 0U
#define HONDA_STEPPER_MODE_TORQUE 1U
#define HONDA_STEPPER_MODE_SOFT_OFF 3U

#define HONDA_STEPPER_STATUS_GAP_US 100000U  // no STEERING_STATUS for 100 ms: the lift lever is up
#define HONDA_STEPPER_LEVER_DOWN_MSGS 200U   // 2 s of STEERING_STATUS at 100 Hz: lever down and the motor started
#define HONDA_STEPPER_STATUS_RUNNING 0x10U   // STEERING_STATUS byte 1: CONTROL_STATUS bit 0, the motor's controller runs

#include "opendbc/safety/modes/honda_stepper_fcan.h"

static uint32_t honda_stepper_status_ts = 0U;    // when the last STEERING_STATUS came
static uint32_t honda_stepper_status_msgs = 0U;  // running STEERING_STATUS messages without a gap, up to LEVER_DOWN_MSGS
static bool honda_stepper_standalone = false;    // nothing is read from the car's network

// Reads an unsigned signal using DBC bit numbering, for both byte orders.
// The rx hook only sees messages whose length matched an rx check, and config.py verified every
// signal fits inside its message, so no bit read here can run past the message data.
static uint32_t honda_stepper_get_signal(const CANPacket_t *msg, uint32_t start_bit, uint32_t size, bool big_endian) {
  uint32_t value = 0U;
  uint32_t bit = start_bit;
  for (uint32_t i = 0U; i < size; i++) {
    uint32_t bit_value = ((uint32_t)msg->data[bit / 8U] >> (bit % 8U)) & 1U;
    if (big_endian) {
      // most significant bit first, walking down each byte and then into the next byte
      value = (value << 1U) | bit_value;
      bit = ((bit % 8U) == 0U) ? (bit + 15U) : (bit - 1U);
    } else {
      value |= bit_value << i;
      bit += 1U;
    }
  }
  return value;
}

static uint32_t honda_stepper_get_checksum(const CANPacket_t *msg) {
  uint32_t checksum;
  if (msg->addr == HONDA_STEPPER_STEERING_STATUS) {
    checksum = msg->data[0];
  } else {
    int len = GET_LEN(msg);
    checksum = (uint32_t)msg->data[len - 1] & 0xFU;
  }
  return checksum;
}

static uint32_t honda_stepper_compute_checksum(const CANPacket_t *msg) {
  int len = GET_LEN(msg);
  uint32_t checksum = 0U;
  if (msg->addr == HONDA_STEPPER_STEERING_STATUS) {
    // StepperServoCAN: id plus every data byte except the checksum byte, high byte folded into low byte
    checksum = (uint32_t)msg->addr;
    for (int i = 1; i < len; i++) {
      checksum += msg->data[i];
    }
    checksum = ((checksum & 0xFFU) + (checksum >> 8U)) & 0xFFU;
  } else {
    // Honda: nibble sum of id and data, minus the checksum nibble, subtracted from 8
    uint8_t sum = 0U;
    uint32_t addr = (uint32_t)msg->addr;
    while (addr > 0U) {
      sum += (uint8_t)(addr & 0xFU);
      addr >>= 4U;
    }
    for (int j = 0; j < len; j++) {
      uint8_t byte = msg->data[j];
      sum += (uint8_t)(byte & 0xFU) + (uint8_t)(byte >> 4U);
      if (j == (len - 1)) {
        sum -= (uint8_t)(byte & 0xFU);
      }
    }
    checksum = ((uint32_t)8U - (uint32_t)sum) & 0xFU;
  }
  return checksum;
}

static uint8_t honda_stepper_get_counter(const CANPacket_t *msg) {
  uint8_t cnt;
  if (msg->addr == HONDA_STEPPER_STEERING_STATUS) {
    cnt = msg->data[1] & 0xFU;
  } else {
    int len = GET_LEN(msg);
    cnt = (msg->data[len - 1] >> 4U) & 0x3U;
  }
  return cnt;
}

// Runs on every message the rx hook accepts. The car's messages keep coming while the motor is silent, so a
// lifted lever is noticed within 100 ms. Before the motor has ever reported, this leaves acc_main_on alone.
static void honda_stepper_lever_update(bool status_received, bool motor_running) {
  uint32_t ts = microsecond_timer_get();
  if ((honda_stepper_status_msgs > 0U) && (safety_get_ts_elapsed(ts, honda_stepper_status_ts) > HONDA_STEPPER_STATUS_GAP_US)) {
    // lever up, or the motor dropped out: steering ends, and the count starts again
    honda_stepper_status_msgs = 0U;
    acc_main_on = false;
  }
  if (status_received) {
    honda_stepper_status_ts = ts;
    if (!motor_running) {
      // the motor is starting up (or restarted after losing power): steering ends, and the count starts again
      honda_stepper_status_msgs = 0U;
      acc_main_on = false;
    } else {
      if (honda_stepper_status_msgs < HONDA_STEPPER_LEVER_DOWN_MSGS) {
        honda_stepper_status_msgs += 1U;
      }
      if (honda_stepper_status_msgs >= HONDA_STEPPER_LEVER_DOWN_MSGS) {
        acc_main_on = true;
      }
    }
  }
}

static void honda_stepper_rx_hook(const CANPacket_t *msg) {
  bool status_received = false;
  bool motor_running = false;
  if (msg->bus == HONDA_STEPPER_SERVO_BUS) {
    if (msg->addr == HONDA_STEPPER_STEERING_STATUS) {
      // torque the motor is producing, 0.125 Nm units at the firmware's gearing
      int torque_meas_new = to_signed(msg->data[2], 8);
      update_sample(&torque_meas, torque_meas_new);
      status_received = true;
      motor_running = ((uint32_t)msg->data[1] & HONDA_STEPPER_STATUS_RUNNING) != 0U;
    }
  }
  honda_stepper_lever_update(status_received, motor_running);

  if (!honda_stepper_standalone && (msg->bus == HONDA_STEPPER_FCAN_BUS)) {
    if (msg->addr == HONDA_STEPPER_SPEED_ADDR) {
      vehicle_moving = honda_stepper_get_signal(msg, HONDA_STEPPER_SPEED_START_BIT, HONDA_STEPPER_SPEED_SIZE,
                                                HONDA_STEPPER_SPEED_BIG_ENDIAN) != 0U;
    }

    if (msg->addr == HONDA_STEPPER_BRAKE_ADDR) {
      brake_pressed = honda_stepper_get_signal(msg, HONDA_STEPPER_BRAKE_START_BIT, HONDA_STEPPER_BRAKE_SIZE,
                                               HONDA_STEPPER_BRAKE_BIG_ENDIAN) != 0U;
    }

#if HONDA_STEPPER_GAS_PRESENT
    if (msg->addr == HONDA_STEPPER_GAS_ADDR) {
      gas_pressed = honda_stepper_get_signal(msg, HONDA_STEPPER_GAS_START_BIT, HONDA_STEPPER_GAS_SIZE,
                                             HONDA_STEPPER_GAS_BIG_ENDIAN) != 0U;
    }
#endif
  }
}

static bool honda_stepper_tx_hook(const CANPacket_t *msg) {
  // units are 0.125 Nm at the motor firmware's gearing: with the 45 Ncm motor and the printed gears, a unit is 0.094 Nm
  // at the steering wheel (opendbc/car/hondastepper/config.json, torque_cmd_scale 1.33)
  const TorqueSteeringLimits HONDA_STEPPER_STEERING_LIMITS = {
    .max_torque = 47,           // 4.4 Nm at the steering wheel; openpilot itself stops at 43 (4.0 Nm)
    .max_rate_up = 2,           // per 20 ms command (openpilot ramps at half that)
    .max_rate_down = 2,
    .max_rt_delta = 15,         // openpilot's 1 unit per command is 13 per 250 ms, plus a buffer
    .max_torque_error = 8,      // 0.75 Nm at the steering wheel above what the motor reports
    .type = TorqueMotorLimited,
  };

  bool tx = false;
  if ((msg->bus == HONDA_STEPPER_SERVO_BUS) && (msg->addr == HONDA_STEPPER_STEERING_COMMAND)) {
    uint32_t mode = ((uint32_t)msg->data[1] >> 4U) & 0x3U;
    uint32_t angle_raw = (uint32_t)msg->data[2] | ((uint32_t)msg->data[3] << 8U);
    int angle = to_signed((int)angle_raw, 16);
    int desired_torque = to_signed(msg->data[4], 8);

    // angle mode would let the motor hold a position on its own, outside these torque limits
    bool mode_allowed = (mode == HONDA_STEPPER_MODE_OFF) || (mode == HONDA_STEPPER_MODE_TORQUE) || (mode == HONDA_STEPPER_MODE_SOFT_OFF);
    int steer_req = (mode == HONDA_STEPPER_MODE_TORQUE) ? 1 : 0;
    bool violation = steer_torque_cmd_checks(desired_torque, steer_req, HONDA_STEPPER_STEERING_LIMITS);
    tx = mode_allowed && (angle == 0) && !violation;
  }
  return tx;
}

static safety_config honda_stepper_init(uint16_t param) {
  static const CanMsg HONDA_STEPPER_TX_MSGS[] = {
    {HONDA_STEPPER_STEERING_COMMAND, HONDA_STEPPER_SERVO_BUS, HONDA_STEPPER_COMMAND_LEN, .check_relay = true},
  };

  honda_stepper_status_ts = 0U;
  honda_stepper_status_msgs = 0U;
  honda_stepper_standalone = GET_FLAG(param, HONDA_STEPPER_PARAM_STANDALONE);
  safety_config ret;
  if (honda_stepper_standalone) {
    // stand-alone: the motor's status only
    static RxCheck honda_stepper_standalone_rx_checks[] = {
      {.msg = {{HONDA_STEPPER_STEERING_STATUS, HONDA_STEPPER_SERVO_BUS, HONDA_STEPPER_STATUS_LEN, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    };
    ret = BUILD_SAFETY_CFG(honda_stepper_standalone_rx_checks, HONDA_STEPPER_TX_MSGS);
  } else {
    static RxCheck honda_stepper_rx_checks[] = {
      HONDA_STEPPER_FCAN_RX_CHECKS
      {.msg = {{HONDA_STEPPER_STEERING_STATUS, HONDA_STEPPER_SERVO_BUS, HONDA_STEPPER_STATUS_LEN, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    };
    ret = BUILD_SAFETY_CFG(honda_stepper_rx_checks, HONDA_STEPPER_TX_MSGS);
  }
  // nothing is forwarded: there is no camera or other module between the car and the motor
  ret.disable_forwarding = true;
  return ret;
}

const safety_hooks honda_stepper_hooks = {
  .init = honda_stepper_init,
  .rx = honda_stepper_rx_hook,
  .tx = honda_stepper_tx_hook,
  .get_checksum = honda_stepper_get_checksum,
  .compute_checksum = honda_stepper_compute_checksum,
  .get_counter = honda_stepper_get_counter,
};
