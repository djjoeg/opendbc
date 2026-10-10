#!/usr/bin/env python3
import random
import unittest

import opendbc.safety.tests.common as common
from opendbc.car.hondastepper.config import DBC_NAME, SERVO_DBC_NAME, load_config
from opendbc.car.hondastepper.values import HONDA_STEPPER_PARAM_STANDALONE, CarControllerParams
from opendbc.car.hondastepper.discovery import with_honda_checksum
from opendbc.car.structs import CarParams
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.safety.tests.libsafety import libsafety_py

CONFIG = load_config()

FCAN_BUS = 0
SERVO_BUS = 1
STEERING_COMMAND = 0x22E
STEERING_STATUS = 0x22F

MODE_OFF = 0
MODE_TORQUE = 1
MODE_ANGLE = 2
MODE_SOFT_OFF = 3

# safety-relevant signals and the libsafety getter that holds their current state
STATE_SIGNALS = {
  "BRAKE_PRESSED": "get_brake_pressed_prev",
  "GAS_PEDAL": "get_gas_pressed_prev",
}

# the lift lever: the motor's status messages without a 100 ms gap, 2 s of them to switch lane keeping on
LEVER_DOWN_MSGS = CarControllerParams.LEVER_DOWN_MSGS
STATUS_GAP_US = 100_000
# STEERING_STATUS.CONTROL_STATUS: bit 0 the motor's controller runs, bit 1 sensored, bit 2 soft-off, bit 3 closed loop
RUNNING = 1 | 2
STARTING = 2 | 4 | 8   # every bit but bit 0


class TestHondaStepperSafety(common.CarSafetyTest, common.MotorTorqueSteeringSafetyTest):
  TX_MSGS = [[STEERING_COMMAND, SERVO_BUS]]
  RELAY_MALFUNCTION_ADDRS = {SERVO_BUS: (STEERING_COMMAND,)}
  FWD_BUS_LOOKUP: dict[int, int] = {}  # nothing is forwarded
  FWD_BLACKLISTED_ADDRS: dict[int, list[int]] = {}

  MAX_RATE_UP = 2
  MAX_RATE_DOWN = 2
  MAX_TORQUE_LOOKUP = [0], [47]
  MAX_RT_DELTA = 15
  MAX_TORQUE_ERROR = 8
  TORQUE_MEAS_TOLERANCE = 0

  def setUp(self):
    self.packer = CANPackerSafety(DBC_NAME)
    self.servo_packer = CANPackerSafety(SERVO_DBC_NAME)
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hondaStepper, 0)
    self.safety.init_tests()
    self.speed = 0.

  # ***** message builders *****

  def _fcan_msg(self, **signals):
    """Builds the F-CAN message holding the given signals, keeping every other safety-relevant
    signal in that message at its current state so sharing a message doesn't change anything else."""
    msg_name = CONFIG.signals[next(iter(signals))].message
    values = {}
    for name, getter in STATE_SIGNALS.items():
      if CONFIG.has(name) and CONFIG.signals[name].message == msg_name:
        values[name] = float(bool(getattr(self.safety, getter)()))
    if CONFIG.signals["VEHICLE_SPEED"].message == msg_name:
      values["VEHICLE_SPEED"] = self.speed
    values.update(signals)
    return self.packer.make_can_msg_safety(msg_name, FCAN_BUS, values)

  def _speed_msg(self, speed):
    self.speed = speed * 3.6  # m/s to kph, the default map's unit
    return self._fcan_msg(VEHICLE_SPEED=self.speed)

  def _speed_msg_2(self, speed):
    return None

  def _user_brake_msg(self, brake):
    return self._fcan_msg(BRAKE_PRESSED=brake)

  def _user_gas_msg(self, gas):
    return self._fcan_msg(GAS_PEDAL=gas)

  def _pcm_status_msg(self, enable):
    raise unittest.SkipTest("the car's own cruise control never allows steering: only the lift lever does")

  def _acc_state_msg(self, enabled):
    # MADS's main switch is the lift lever: 2 s of motor status messages, not one message (tests below)
    raise NotImplementedError

  def _lkas_button_msg(self, enabled):
    raise NotImplementedError  # no lane keeping button: the lift lever is the only switch

  def _torque_cmd_msg(self, torque, steer_req=1, mode=None, angle=0):
    if mode is None:
      mode = MODE_TORQUE if steer_req else MODE_OFF
    values = {"STEER_MODE": mode, "STEER_TORQUE": torque * 0.125, "STEER_ANGLE": angle}
    return self.servo_packer.make_can_msg_safety("STEERING_COMMAND", SERVO_BUS, values)

  def _torque_meas_msg(self, torque, control_status=RUNNING):
    values = {"STEERING_TORQUE": torque * 0.125, "CONTROL_STATUS": control_status}
    return self.servo_packer.make_can_msg_safety("STEERING_STATUS", SERVO_BUS, values)

  # ***** tests for this mode *****

  def test_torque_absolute_limits(self):
    # STEER_TORQUE is 8 bits, so the base test's +-1000 sweep would wrap; sweep everything the field can carry
    for controls_allowed in (True, False):
      for torque in range(-128, 128):
        self.safety.set_controls_allowed(controls_allowed)
        self.safety.set_rt_torque_last(torque)
        self.safety.set_torque_meas(torque, torque)
        self.safety.set_desired_torque_last(torque - self.MAX_RATE_UP)
        send = (-self.MAX_TORQUE <= torque <= self.MAX_TORQUE) if controls_allowed else torque == 0
        self.assertEqual(send, self._tx(self._torque_cmd_msg(torque)), torque)

  def test_only_torque_off_and_soft_off_modes(self):
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self.assertTrue(self._tx(self._torque_cmd_msg(0, mode=MODE_OFF)))
    self.assertTrue(self._tx(self._torque_cmd_msg(0, mode=MODE_SOFT_OFF)))
    self.assertTrue(self._tx(self._torque_cmd_msg(0, mode=MODE_TORQUE)))
    # angle mode would hold a position with the motor's own controller, outside the torque limits
    self.assertFalse(self._tx(self._torque_cmd_msg(0, mode=MODE_ANGLE)))

  def test_torque_only_in_torque_mode(self):
    self.safety.set_controls_allowed(True)
    for mode in (MODE_OFF, MODE_SOFT_OFF):
      self._set_prev_torque(2)
      self.assertFalse(self._tx(self._torque_cmd_msg(2, mode=mode)))

  def test_angle_field_must_be_zero(self):
    self.safety.set_controls_allowed(True)
    for angle in (-90., -0.125, 0.125, 90.):
      self._set_prev_torque(0)
      self.assertFalse(self._tx(self._torque_cmd_msg(0, mode=MODE_TORQUE, angle=angle)))
      self.assertFalse(self._tx(self._torque_cmd_msg(0, mode=MODE_OFF, angle=angle)))

  def test_off_allowed_without_controls(self):
    self.safety.set_controls_allowed(False)
    self.safety.set_controls_allowed_lateral(False)
    self.assertTrue(self._tx(self._torque_cmd_msg(0, mode=MODE_OFF)))
    self.assertTrue(self._tx(self._torque_cmd_msg(0, mode=MODE_SOFT_OFF)))
    self.assertFalse(self._tx(self._torque_cmd_msg(1, mode=MODE_TORQUE)))

  def test_lateral_only_allows_torque(self):
    # MADS lane keeping without stock cruise engaged
    self.safety.set_controls_allowed(False)
    self.safety.set_controls_allowed_lateral(True)
    self._set_prev_torque(0)
    self.assertTrue(self._tx(self._torque_cmd_msg(self.MAX_RATE_UP)))

  def test_nothing_sent_on_car_bus(self):
    self.safety.set_controls_allowed(True)
    for addr in (STEERING_COMMAND, 0x158, 0x17C, 0x1A6, 0xE4):
      self.assertFalse(self._tx(common.make_msg(FCAN_BUS, addr, 8)))
      self.assertFalse(self._tx(common.make_msg(2, addr, 8)))

  def test_car_bus_never_allows_steering(self):
    # nothing the car sends can allow steering, whatever its buttons or cruise control do: only the lift lever can
    self.safety.set_mads_params(True, False, False)
    rng = random.Random(0)
    for _ in range(2000):
      msg = rng.choice(list(CONFIG.messages.values()))
      data = bytes(rng.randrange(256) for _ in range(msg.length))
      if msg.honda_checksum:
        data = with_honda_checksum(msg.addr, data, rng.randrange(4) if msg.counter else None)
      self._rx(libsafety_py.make_CANPacket(msg.addr, FCAN_BUS, data))
      self.assertFalse(self.safety.get_controls_allowed())
      self.assertFalse(self.safety.get_controls_allowed_lateral())
      self.assertFalse(self.safety.get_acc_main_on())

  # ***** the lift lever *****

  def _lever_setup(self):
    self.safety.set_mads_params(True, False, False)
    self.now = 0
    self.safety.set_timer(0)

  def _after(self, us):
    self.now += us
    self.safety.set_timer(self.now)

  def _motor_status(self, n=1, control_status=RUNNING):
    """n motor status messages 10 ms apart, each followed 5 ms later by a message from the car."""
    for _ in range(n):
      self._after(5_000)
      self.assertTrue(self._rx(self._torque_meas_msg(0, control_status)))
      self._after(5_000)
      self._rx(self._speed_msg(0))

  def _silence(self, us):
    """The motor silent for us microseconds while the car keeps sending."""
    for _ in range(us // 10_000):
      self._after(10_000)
      self._rx(self._speed_msg(0))

  def test_lever_down_allows_steering_after_2_s(self):
    self._lever_setup()
    self._motor_status(LEVER_DOWN_MSGS - 1)
    self.assertFalse(self.safety.get_acc_main_on())
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self._after(5_000)
    self._rx(self._torque_meas_msg(0))
    self.assertTrue(self.safety.get_acc_main_on())
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self.safety.get_controls_allowed())   # only lane keeping, never the car's speed
    self._set_prev_torque(0)
    self.assertTrue(self._tx(self._torque_cmd_msg(self.MAX_RATE_UP)))

  def test_lever_needs_mads(self):
    self._lever_setup()
    self.safety.set_mads_params(False, False, False)
    self._motor_status(LEVER_DOWN_MSGS + 10)
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_short_gap_keeps_steering(self):
    self._lever_setup()
    self._motor_status(LEVER_DOWN_MSGS)
    self._silence(90_000)
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self._motor_status(5)
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_lever_up_ends_steering(self):
    self._lever_setup()
    self._motor_status(LEVER_DOWN_MSGS)
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self._silence(STATUS_GAP_US + 10_000)
    self.assertFalse(self.safety.get_acc_main_on())
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self._set_prev_torque(0)
    self.assertFalse(self._tx(self._torque_cmd_msg(self.MAX_RATE_UP)))
    # pushed down again: another 2 s before steering
    self._motor_status(LEVER_DOWN_MSGS - 1)
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self._motor_status(1)
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_gap_noticed_by_the_next_status(self):
    # nothing else came during the gap: the next status message itself shows it
    self._lever_setup()
    self._motor_status(LEVER_DOWN_MSGS)
    self._after(STATUS_GAP_US + 50_000)
    self.assertTrue(self._rx(self._torque_meas_msg(0)))
    self.assertFalse(self.safety.get_acc_main_on())
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_motor_starting_up_is_not_counted(self):
    # the motor reports before its controller runs; the 2 s start when it runs
    self._lever_setup()
    self._motor_status(300, control_status=STARTING)
    self.assertFalse(self.safety.get_acc_main_on())
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self._motor_status(LEVER_DOWN_MSGS - 1)
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self._motor_status(1)
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_motor_restart_ends_steering(self):
    # a message from a motor that restarted (it lost power for a moment, too briefly for a 100 ms gap) ends steering
    # at once, and the count starts again
    self._lever_setup()
    self._motor_status(LEVER_DOWN_MSGS)
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self._motor_status(1, control_status=0)
    self.assertFalse(self.safety.get_acc_main_on())
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self._motor_status(LEVER_DOWN_MSGS - 1)
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self._motor_status(1)
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_lever_down_while_braking_waits_for_release(self):
    # "Steering Mode on Brake Pedal: Pause"
    self._lever_setup()
    self.safety.set_mads_params(True, False, True)
    self._rx(self._user_brake_msg(True))
    self._motor_status(LEVER_DOWN_MSGS)
    self.assertTrue(self.safety.get_acc_main_on())
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self._rx(self._user_brake_msg(False))
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_motor_status_checksum(self):
    msg = self._torque_meas_msg(10)
    self.assertTrue(self._rx(msg))
    bad = self._torque_meas_msg(10)
    bad[0].data[0] = (bad[0].data[0] + 1) % 256
    self.assertFalse(self._rx(bad))

  def test_motor_status_counter(self):
    for _ in range(10):
      self.assertTrue(self._rx(self._torque_meas_msg(0)))
    # a frozen counter is caught after a few frames
    stuck = self._torque_meas_msg(0)
    for _ in range(common.MAX_WRONG_COUNTERS + 1):
      self._rx(stuck)
    self.assertFalse(self._rx(stuck))

  def test_fcan_checksum(self):
    msg = self._speed_msg(10)
    self.assertTrue(self._rx(msg))
    if not CONFIG.message_of("VEHICLE_SPEED").honda_checksum:
      raise unittest.SkipTest("speed message has no checksum")
    bad = self._speed_msg(10)
    last = CONFIG.message_of("VEHICLE_SPEED").length - 1
    bad[0].data[last] = (bad[0].data[last] & 0xF0) | ((bad[0].data[last] + 1) & 0x0F)
    self.assertFalse(self._rx(bad))

  def test_motor_lag_ends_lateral(self):
    self.safety.set_mads_params(True, False, False)
    self.safety.set_controls_allowed_lateral(True)
    self.safety.set_timer(int(2e6))
    self.safety.safety_tick_current_safety_config()
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_signal_reader_byte_orders(self):
    # Intel (little endian) 12 bit value starting at bit 4 of byte 1
    msg = libsafety_py.make_CANPacket(0x100, 0, bytes([0x00, 0xB0, 0x5A, 0, 0, 0, 0, 0]))
    self.assertEqual(0x5AB, self.safety.honda_stepper_get_signal_test(msg, 12, 12, False))
    # Motorola (big endian) 16 bit value starting at bit 7, Honda speed style
    msg = libsafety_py.make_CANPacket(0x100, 0, bytes([0x12, 0x34, 0, 0, 0, 0, 0, 0]))
    self.assertEqual(0x1234, self.safety.honda_stepper_get_signal_test(msg, 7, 16, True))
    # Motorola value crossing a byte boundary mid-byte
    msg = libsafety_py.make_CANPacket(0x100, 0, bytes([0x03, 0xC0, 0, 0, 0, 0, 0, 0]))
    self.assertEqual(0xF, self.safety.honda_stepper_get_signal_test(msg, 1, 4, True))


class TestHondaStepperStandaloneSafety(common.SafetyTest, common.MotorTorqueSteeringSafetyTest):
  """Stand-alone (the default): the comma device isn't connected to the car. Only the motor's status is checked and
  nothing from bus 0 is read; the torque limits and the lift lever logic are the same."""
  TX_MSGS = [[STEERING_COMMAND, SERVO_BUS]]
  FWD_BUS_LOOKUP: dict[int, int] = {}
  FWD_BLACKLISTED_ADDRS: dict[int, list[int]] = {}

  MAX_RATE_UP = 2
  MAX_RATE_DOWN = 2
  MAX_TORQUE_LOOKUP = [0], [47]
  MAX_RT_DELTA = 15
  MAX_TORQUE_ERROR = 8
  TORQUE_MEAS_TOLERANCE = 0

  def setUp(self):
    self.packer = CANPackerSafety(DBC_NAME)
    self.servo_packer = CANPackerSafety(SERVO_DBC_NAME)
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hondaStepper, HONDA_STEPPER_PARAM_STANDALONE)
    self.safety.init_tests()

  def _torque_cmd_msg(self, torque, steer_req=1, mode=None, angle=0):
    if mode is None:
      mode = MODE_TORQUE if steer_req else MODE_OFF
    values = {"STEER_MODE": mode, "STEER_TORQUE": torque * 0.125, "STEER_ANGLE": angle}
    return self.servo_packer.make_can_msg_safety("STEERING_COMMAND", SERVO_BUS, values)

  def _torque_meas_msg(self, torque, control_status=RUNNING):
    values = {"STEERING_TORQUE": torque * 0.125, "CONTROL_STATUS": control_status}
    return self.servo_packer.make_can_msg_safety("STEERING_STATUS", SERVO_BUS, values)

  def _car_msg(self, **signals):
    return self.packer.make_can_msg_safety(CONFIG.signals[next(iter(signals))].message, FCAN_BUS, signals)

  def _speed_msg(self, speed):
    # the car's speed message: stand-alone it is never read
    return self._car_msg(VEHICLE_SPEED=speed * 3.6)

  def _lever_setup(self):
    self.safety.set_mads_params(True, False, False)
    self.now = 0
    self.safety.set_timer(0)

  def _after(self, us):
    self.now += us
    self.safety.set_timer(self.now)

  def _motor_status(self, n=1, control_status=RUNNING):
    """n motor status messages 10 ms apart, nothing else on the network."""
    for _ in range(n):
      self._after(10_000)
      self.assertTrue(self._rx(self._torque_meas_msg(0, control_status)))

  def test_relay_malfunction(self):
    # openpilot's motor command seen coming back on the motor's bus means the harness is wired wrong
    for bus in range(3):
      self.safety.set_relay_malfunction(False)
      self._rx(common.make_msg(bus, STEERING_COMMAND, 8))
      self.assertEqual(bus == SERVO_BUS, self.safety.get_relay_malfunction())

  def test_the_cars_messages_are_ignored(self):
    # a car message that does arrive (the OBD port plugged in after all) is never read: no brake, gas or speed here
    self.safety.set_mads_params(True, False, False)
    self._rx(self._car_msg(VEHICLE_SPEED=100.0))
    self._rx(self._car_msg(BRAKE_PRESSED=1))
    self._rx(self._car_msg(GAS_PEDAL=50))
    self.assertFalse(self.safety.get_vehicle_moving())
    self.assertFalse(self.safety.get_brake_pressed_prev())
    self.assertFalse(self.safety.get_gas_pressed_prev())
    rng = random.Random(0)
    for _ in range(500):
      msg = rng.choice(list(CONFIG.messages.values()))
      self._rx(libsafety_py.make_CANPacket(msg.addr, FCAN_BUS, bytes(rng.randrange(256) for _ in range(msg.length))))
      self.assertFalse(self.safety.get_controls_allowed_lateral())
      self.assertFalse(self.safety.get_acc_main_on())

  def test_only_the_motor_is_checked(self):
    # with the motor reporting and nothing at all from the car, the rx checks are met
    self._lever_setup()
    self._motor_status(150)
    self.safety.safety_tick_current_safety_config()
    self.assertTrue(self.safety.safety_config_valid())

  def test_lever_down_allows_steering_after_2_s(self):
    self._lever_setup()
    self._motor_status(LEVER_DOWN_MSGS - 1)
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self._motor_status(1)
    self.assertTrue(self.safety.get_acc_main_on())
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self.safety.get_controls_allowed())
    self._set_prev_torque(0)
    self.assertTrue(self._tx(self._torque_cmd_msg(self.MAX_RATE_UP)))

  def test_lever_up_ends_steering(self):
    # lever up: nothing at all arrives. The 1 s rx check lag ends steering, and the motor's next message after the gap
    # (lever down again) restarts the 2 s count
    self._lever_setup()
    self._motor_status(LEVER_DOWN_MSGS)
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self._after(1_100_000)
    self.safety.safety_tick_current_safety_config()
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self._set_prev_torque(0)
    self.assertFalse(self._tx(self._torque_cmd_msg(self.MAX_RATE_UP)))
    self._motor_status(1)
    self.assertFalse(self.safety.get_acc_main_on())
    self._motor_status(LEVER_DOWN_MSGS - 2)
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self._motor_status(1)
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_motor_restart_ends_steering(self):
    self._lever_setup()
    self._motor_status(LEVER_DOWN_MSGS)
    self._motor_status(1, control_status=0)
    self.assertFalse(self.safety.get_acc_main_on())
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  # the same commands and limits as connected to the car
  test_torque_absolute_limits = TestHondaStepperSafety.test_torque_absolute_limits
  test_only_torque_off_and_soft_off_modes = TestHondaStepperSafety.test_only_torque_off_and_soft_off_modes
  test_torque_only_in_torque_mode = TestHondaStepperSafety.test_torque_only_in_torque_mode
  test_angle_field_must_be_zero = TestHondaStepperSafety.test_angle_field_must_be_zero
  test_off_allowed_without_controls = TestHondaStepperSafety.test_off_allowed_without_controls
  test_lateral_only_allows_torque = TestHondaStepperSafety.test_lateral_only_allows_torque
  test_nothing_sent_on_car_bus = TestHondaStepperSafety.test_nothing_sent_on_car_bus
  test_motor_status_checksum = TestHondaStepperSafety.test_motor_status_checksum
  test_motor_status_counter = TestHondaStepperSafety.test_motor_status_counter

if __name__ == "__main__":
  unittest.main()
