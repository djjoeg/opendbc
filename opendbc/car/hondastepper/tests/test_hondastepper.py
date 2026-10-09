import contextlib
import copy
import json
import os
import struct
import tempfile
import unittest
from unittest import mock

from opendbc.can import CANPacker
from opendbc.car import DT_CTRL, gen_empty_fingerprint, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.car_helpers import interfaces
from opendbc.car import Bus
from opendbc.car.hondastepper import carstate as carstate_mod, config as cfgmod, interface as interface_mod, values as values_mod
from opendbc.car.hondastepper.carstate import TORQUE_UNIT
from opendbc.car.hondastepper.config import CONFIG_PATH, ConfigError, parse_config
from opendbc.car.hondastepper.tests.sim import FREE_SPIN_FW_DEG_S, SimMotor
from opendbc.car.hondastepper.values import CAR, CONFIG, CanBus, CarControllerParams, HondaStepperFlags, StepperMode

ButtonType = structs.CarState.ButtonEvent.Type
PLATFORM = CAR.HONDA_ACCORD_8G_STEPPER


def raw_default_config() -> dict:
  with open(CONFIG_PATH) as f:
    return json.load(f)


def raw_config_with_fcan_angle() -> dict:
  """The default map plus an F-CAN steering angle, laid out like comma's STEERING_SENSORS (0x156)."""
  raw = raw_default_config()
  raw["fcan"]["messages"].append({"name": "STEERING_SENSORS", "addr": "0x156", "len": 6, "freq": 100, "checksum": "honda", "counter": True})
  raw["fcan"]["signals"]["STEER_ANGLE"] = {"message": "STEERING_SENSORS", "start_bit": 7, "size": 16, "byte_order": "big",
                                           "signed": True, "factor": -0.1, "unit": "deg"}
  return raw


def raw_config_with_cruise_buttons() -> dict:
  """The default map plus the cruise buttons at comma's Honda Nidec location. Supported, but not in the default
  config; they only ever give button events."""
  raw = raw_default_config()
  raw["fcan"]["signals"]["CRUISE_BUTTONS"] = {"message": "SCM_BUTTONS", "start_bit": 7, "size": 3, "byte_order": "big",
                                              "values": {"0": "none", "1": "main", "2": "cancel", "3": "decel_set", "4": "accel_res"}}
  return raw


@contextlib.contextmanager
def variant(raw: dict):
  """Runs the car port with another config: its DBC goes to a temp file and the modules see the config."""
  cfg = parse_config(raw)
  with tempfile.TemporaryDirectory() as d:
    dbc_path = os.path.join(d, "hondastepper_fcan.dbc")
    with open(dbc_path, "w") as f:
      f.write(cfgmod.generate_dbc(cfg))
    dbc_map = {PLATFORM: {**values_mod.DBC[PLATFORM], Bus.pt: dbc_path}}
    with mock.patch.object(carstate_mod, "CONFIG", cfg), mock.patch.object(interface_mod, "CONFIG", cfg), \
         mock.patch.object(carstate_mod, "DBC", dbc_map):
      yield cfg, dbc_path


def firmware_command_bytes(counter: int, mode: int, torque_nm: float) -> bytes:
  """Independent copy of StepperServoPandaCli.steering_msg_cmd_data (dzid26/StepperServoCAN)."""
  packed = struct.pack('<Bhb', (mode << 4) | counter, 0, max(min(int(torque_nm * 8), 127), -128))
  checksum = 0x22E
  for b in packed:
    checksum += b
  checksum = ((checksum & 0xFF) + (checksum >> 8)) & 0xFF
  return struct.pack('<B', checksum) + packed


class TestConfig(unittest.TestCase):
  def setUp(self):
    with open(CONFIG_PATH) as f:
      self.raw = json.load(f)

  def test_generated_files_current(self):
    self.assertEqual(cfgmod.stale_files(CONFIG), [], "run python -m opendbc.car.hondastepper.config --write")

  def test_checked_in_config_parses(self):
    cfg = parse_config(self.raw)
    for name in cfgmod.REQUIRED_SIGNALS:
      self.assertTrue(cfg.has(name))

  def _expect_error(self, mutate):
    raw = copy.deepcopy(self.raw)
    mutate(raw)
    with self.assertRaises(ConfigError):
      parse_config(raw)

  def test_rejects_bad_configs(self):
    self._expect_error(lambda r: r["fcan"]["signals"].pop("BRAKE_PRESSED"))
    self._expect_error(lambda r: r["fcan"]["signals"]["BRAKE_PRESSED"].update(start_bit=59))     # Honda checksum bits
    self._expect_error(lambda r: r["fcan"]["signals"]["BRAKE_PRESSED"].update(size=2))           # flags are 1 bit
    self._expect_error(lambda r: r["fcan"]["signals"]["GAS_PEDAL"].update(start_bit=70))         # past the end of the message
    self._expect_error(lambda r: r["fcan"]["signals"]["BRAKE_PRESSED"].update(start_bit=7))      # same bits as GAS_PEDAL
    self._expect_error(lambda r: r["fcan"]["signals"]["VEHICLE_SPEED"].update(unit="furlongs"))
    self._expect_error(lambda r: r["fcan"]["signals"].update(MADE_UP={"message": "ENGINE_DATA", "start_bit": 0, "size": 1}))
    self._expect_error(lambda r: r["fcan"]["messages"][0].update(addr="0x17C"))                  # duplicate address
    self._expect_error(lambda r: r["fcan"]["messages"][0].update(checksum="crc"))
    self._expect_error(lambda r: r["actuator"].update(direction=0))
    self._expect_error(lambda r: r["actuator"].update(mechanical_ratio=40))                      # too far from the firmware
    self._expect_error(lambda r: r["fcan"].update(mads_button={"signal": "BRAKE_PRESSED", "value": 1}))  # the lever is the switch
    for name in ("CRUISE_MAIN", "CRUISE_ACTIVE"):                                                   # not read any more
      self._expect_error(lambda r, name=name: r["fcan"]["signals"].update(
        {name: {"message": "SCM_BUTTONS", "start_bit": 47, "size": 1, "byte_order": "big"}}))
    self._expect_error(lambda r: r["actuator"].update(tracked_arc_deg=5))
    self._expect_error(lambda r: r["fcan"]["messages"][1].update(freq=5))                        # brake too slow for the panda
    self._expect_error(lambda r: r.update(setup={"parked": True}))
    self._expect_error(lambda r: r.update(setup={"someday": "now"}))

  def test_verified_needs_both_setup_checks(self):
    self.assertFalse(CONFIG.verified)  # the default in the code was never checked on the car
    for setup, verified in (({"parked": "x"}, False), ({"drive": "x"}, False), ({"parked": "x", "drive": "y"}, True)):
      raw = copy.deepcopy(self.raw)
      raw["setup"] = setup
      self.assertEqual(parse_config(raw).verified, verified)

  def test_device_config_replaces_default(self):
    with tempfile.TemporaryDirectory() as d:
      path = os.path.join(d, "hondastepper", "config.json")
      with mock.patch.dict(os.environ, {"HONDASTEPPER_CONFIG": path}):
        self.assertEqual(cfgmod.active_config_path(), CONFIG_PATH)  # nothing saved yet
        raw = copy.deepcopy(self.raw)
        raw["setup"] = {"parked": "2026-10-07 10:00", "drive": "2026-10-07 10:30"}
        raw["actuator"]["direction"] = -1
        self.assertEqual(cfgmod.save_device_config(raw), path)
        self.assertEqual(cfgmod.active_config_path(), path)
        cfg = cfgmod.load_config()
        self.assertTrue(cfg.verified)
        self.assertEqual(cfg.actuator.direction, -1)
        self.assertEqual(os.listdir(os.path.dirname(path)), ["config.json"])  # no temp files left behind

  def test_broken_device_config_falls_back_to_the_default(self):
    with tempfile.TemporaryDirectory() as d:
      path = os.path.join(d, "config.json")
      with mock.patch.dict(os.environ, {"HONDASTEPPER_CONFIG": path}):
        for content in ("{not json", json.dumps({"schema_version": 1})):
          with open(path, "w") as f:
            f.write(content)
          cfg, problem = cfgmod.load_config_or_default()
          self.assertIn(path, problem)
          self.assertFalse(cfg.verified)      # the default was never checked, so lane keeping stays off
          self.assertEqual(cfg.actuator, CONFIG.actuator)
        os.remove(path)
        self.assertEqual(cfgmod.load_config_or_default(), (CONFIG, ""))

  def test_save_device_config_rejects_invalid(self):
    with tempfile.TemporaryDirectory() as d:
      path = os.path.join(d, "config.json")
      raw = copy.deepcopy(self.raw)
      raw["actuator"]["direction"] = 0
      with self.assertRaises(ConfigError):
        cfgmod.save_device_config(raw, path)
      self.assertFalse(os.path.exists(path))

  def test_write_generated_rewrites_only_stale_files(self):
    with tempfile.TemporaryDirectory() as d:
      dbc, header = os.path.join(d, "a.dbc"), os.path.join(d, "a.h")
      with mock.patch.object(cfgmod, "DBC_PATH", dbc), mock.patch.object(cfgmod, "SAFETY_HEADER_PATH", header):
        self.assertEqual(sorted(cfgmod.write_generated(CONFIG)), sorted([dbc, header]))
        self.assertEqual(cfgmod.write_generated(CONFIG), [])
        # the safety mode doesn't read the steering angle, so only the DBC changes
        angle_cfg = parse_config(raw_config_with_fcan_angle())
        self.assertEqual(cfgmod.write_generated(angle_cfg), [dbc])
        self.assertEqual(cfgmod.stale_files(angle_cfg), [])
        self.assertEqual(cfgmod.stale_files(CONFIG), [dbc])
        # moving a signal the safety mode reads changes both
        raw = raw_default_config()
        raw["fcan"]["signals"]["BRAKE_PRESSED"]["start_bit"] = 32
        self.assertEqual(sorted(cfgmod.write_generated(parse_config(raw))), sorted([dbc, header]))

  def test_scales(self):
    a = CONFIG.actuator
    # the firmware default (5.18:1 gearbox x 2, 39 Ncm 1.3 A motor) driving the 45 Ncm 2 A motor through the 197:19
    # printed gears: the firmware needs 1.33 times the torque asked at the steering wheel
    self.assertAlmostEqual(a.torque_cmd_scale, 1.333, delta=0.005)
    # lane keeping's own most and the panda's limit, at the steering wheel
    params = CarControllerParams(None)
    self.assertEqual(params.STEER_MAX, 41)
    self.assertAlmostEqual(params.STEER_MAX * 0.125 / a.torque_cmd_scale, 3.85, delta=0.01)
    # the grip test's push (the board's 3.3 A) is at least 1.4 times lane keeping's most
    self.assertGreaterEqual(3.3 * a.motor_k_torque * a.mechanical_ratio / (params.STEER_MAX * 0.125 / a.torque_cmd_scale), 1.4)
    self.assertAlmostEqual(CarControllerParams.SAFETY_MAX_TORQUE * 0.125 / a.torque_cmd_scale, 4.41, delta=0.01)
    self.assertAlmostEqual(a.angle_scale, a.firmware_gearing_ratio / a.mechanical_ratio)


class TestStepperMessages(unittest.TestCase):
  def test_command_matches_firmware_tool(self):
    packer = CANPacker(cfgmod.SERVO_DBC_NAME)
    from opendbc.car.hondastepper.steppercan import create_steer_command
    for counter in (0, 7, 15):
      for mode in (StepperMode.OFF, StepperMode.TORQUE_CONTROL, StepperMode.SOFT_OFF):
        for torque in (-4.125, -0.125, 0.0, 0.125, 4.125):
          addr, dat, bus = create_steer_command(packer, counter, mode, torque)
          expected = firmware_command_bytes(counter, int(mode), torque if mode == StepperMode.TORQUE_CONTROL else 0.0)
          self.assertEqual((addr, bus), (0x22E, 1))
          self.assertEqual(bytes(dat), expected, (counter, mode, torque))


class HondaStepperHarness:
  def __init__(self, direction: int = 1, fcan_dbc: str = cfgmod.DBC_NAME, cfg=CONFIG):
    CI = interfaces[PLATFORM]
    fingerprint = gen_empty_fingerprint()
    self.CP = CI.get_params(PLATFORM, fingerprint, [], False, False, False)
    self.CP_SP = CI.get_params_sp(self.CP, PLATFORM, fingerprint, [], False, False, False)
    self.CI = CI(self.CP, self.CP_SP)
    self.CI.CS.direction = direction
    self.CI.CC.direction = direction
    self.cfg = cfg
    self.fcan = CANPacker(fcan_dbc)
    self.servo = CANPacker(cfgmod.SERVO_DBC_NAME)
    self.nanos = 0
    self.fcan_values: dict[str, dict[str, float]] = {m: {} for m in cfg.messages}
    self.status = {"CONTROL_STATUS": 3, "STEERING_TORQUE": 0.0, "STEERING_ANGLE": 0.0, "STEERING_SPEED": 0.0}
    self.motor_rev_s = 0.0     # how fast the motor really turns (firmware turns per second): its angle advances with it
    self.panda_passes = True   # the panda runs this car's safety mode and passes the motor commands on
    self.passed: list[CanData] = []

  def set(self, **signals):
    for name, value in signals.items():
      self.fcan_values[self.cfg.signals[name].message][name] = value

  def turning(self, rev_s: float):
    """The motor turns this fast from now on (firmware turns per second). Its angle advances with every status
    message, and its speed field reads what the firmware would put there: 8 bits that wrap above 2 turns a second."""
    self.motor_rev_s = rev_s

  def step(self, motor: bool = True, CC=None, motor_frames=None, stall: bool = False):
    """One 10 ms step. The motor status comes from self.status, or from motor_frames (a simulated motor) when given.
    The panda's report of the commands it passed on in the step before comes along. stall: no messages at all reach
    openpilot in this step (a hold-up in the comma device's own message handling)."""
    frames = [CanData(*self.fcan.make_can_msg(m, 0, v)) for m, v in self.fcan_values.items()]
    if motor_frames is not None:
      frames += [CanData(addr, dat, 1) for _, addr, dat in motor_frames]
    elif motor:
      self.status["STEERING_ANGLE"] += self.motor_rev_s * 360.0 * DT_CTRL
      self.status["STEERING_SPEED"] = wrapped_speed_field(self.motor_rev_s)
      frames.append(CanData(*self.servo.make_can_msg("STEERING_STATUS", 1, self.status)))
    frames += [CanData(addr, dat, CanBus.SERVO_SENT) for addr, dat, _ in self.passed]
    self.passed = []
    self.nanos += int(DT_CTRL * 1e9)
    CS, _ = self.CI.update([] if stall else [(self.nanos, frames)])
    sends = []
    if CC is not None:
      _, sends = self.CI.apply(CC.as_reader(), structs.CarControlSP(), self.nanos)
      if self.panda_passes:
        self.passed = list(sends)
    return CS, sends


def wrapped_speed_field(rev_s: float) -> float:
  """STEERING_SPEED as the firmware writes it: the value in 1/64 turn per second cast to a signed byte, so anything
  past 2 turns a second wraps round."""
  raw = int(rev_s / 0.015625)
  return ((raw + 128) % 256 - 128) * 0.015625


# The motor's first message makes the car port send a command; the panda's report of it comes with the next
# message, and counting starts with the one after that.
LEVER_ON_STEPS = CarControllerParams.LEVER_DOWN_MSGS + 2


def lat_active_cc(torque: float):
  CC = structs.CarControl()
  CC.enabled = True
  CC.latActive = True
  CC.actuators.torque = torque
  return CC


class TestCarState(unittest.TestCase):
  def test_basic_signals(self):
    h = HondaStepperHarness()
    h.set(VEHICLE_SPEED=72.0, BRAKE_PRESSED=1, GAS_PEDAL=20, GEAR=4, LEFT_BLINKER=1)
    for _ in range(5):
      CS, _ = h.step()
    self.assertAlmostEqual(CS.vEgoRaw, 20.0, places=2)
    self.assertTrue(CS.brakePressed)
    self.assertTrue(CS.gasPressed)
    self.assertFalse(CS.cruiseState.available)   # the motor has reported for less than 2 s
    self.assertFalse(CS.cruiseState.enabled)
    self.assertEqual(CS.gearShifter, structs.CarState.GearShifter.drive)
    self.assertTrue(CS.leftBlinker)
    self.assertFalse(CS.rightBlinker)
    self.assertTrue(CS.canValid)

  def _lever_down(self, h: HondaStepperHarness, steps: int = LEVER_ON_STEPS):
    """Lift lever down: the motor reports, the car port sends its (off) commands, and lane keeping turns on at the
    last of these steps."""
    idle = structs.CarControl()
    for _ in range(steps - 1):
      CS, _ = h.step(CC=idle)
      self.assertFalse(CS.cruiseState.available)
    CS, _ = h.step(CC=idle)
    self.assertTrue(CS.cruiseState.available)
    return CS

  def test_lift_lever_turns_lane_keeping_on_after_2_s(self):
    # sunnypilot's MADS ("Toggle with Main Cruise") takes cruiseState.available as its main switch
    h = HondaStepperHarness()
    idle = structs.CarControl()
    CS = self._lever_down(h)
    self.assertFalse(CS.cruiseState.enabled)   # the car's own cruise control never engages openpilot
    # a short gap in the motor's messages changes nothing
    for _ in range(5):
      CS, _ = h.step(motor=False, CC=idle)
    self.assertTrue(CS.cruiseState.available)
    CS, _ = h.step(CC=idle)
    self.assertTrue(CS.cruiseState.available)
    # lever up: off after 100 ms of silence, and the next lever down waits its own 2 s
    for _ in range(10):
      CS, _ = h.step(motor=False, CC=idle)
    self.assertFalse(CS.cruiseState.available)
    self.assertTrue(CS.steeringDisengage)
    self._lever_down(h)

  def test_waits_until_the_panda_passes_commands(self):
    # The panda runs this car's safety mode only once openpilot has started up. Before that it passes no motor
    # commands and counts nothing: with the lift lever already down as the car starts, lane keeping waits for it.
    h = HondaStepperHarness()
    h.panda_passes = False
    idle = structs.CarControl()
    commands = 0
    for _ in range(500):
      CS, sends = h.step(CC=idle)
      commands += len(sends)
      self.assertFalse(CS.cruiseState.available)
    self.assertEqual(commands, 250)   # the car port keeps sending its off commands meanwhile
    h.panda_passes = True
    self._lever_down(h)

  def test_motor_starting_up_is_not_counted(self):
    # the motor reports before its controller runs (CONTROL_STATUS bit 0); the 2 s start when it runs
    h = HondaStepperHarness()
    idle = structs.CarControl()
    h.status["CONTROL_STATUS"] = 0
    for _ in range(300):
      CS, _ = h.step(CC=idle)
      self.assertFalse(CS.cruiseState.available)
    h.status["CONTROL_STATUS"] = 3
    self._lever_down(h, steps=CarControllerParams.LEVER_DOWN_MSGS)

  def test_motor_restart_turns_lane_keeping_off(self):
    # a message from a motor that restarted (it lost power for a moment) ends lane keeping at once; it comes back on
    # by itself 2 s after the motor runs again, like the panda
    h = HondaStepperHarness()
    idle = structs.CarControl()
    self._lever_down(h)
    h.status["CONTROL_STATUS"] = 0
    CS, _ = h.step(CC=idle)
    self.assertFalse(CS.cruiseState.available)
    h.status["CONTROL_STATUS"] = 3
    self._lever_down(h, steps=CarControllerParams.LEVER_DOWN_MSGS)

  def test_a_hold_up_in_openpilot_is_not_a_lifted_lever(self):
    # no messages at all for 300 ms, then they all come at once: nothing changes. The panda times the motor's
    # silence against the car's messages too.
    h = HondaStepperHarness()
    idle = structs.CarControl()
    self._lever_down(h)
    for _ in range(30):
      CS, _ = h.step(stall=True, CC=idle)
    self.assertTrue(CS.cruiseState.available)
    self.assertTrue(h.CI.CS.motor_present)
    CS, _ = h.step(CC=idle)
    self.assertTrue(CS.cruiseState.available)

  def test_cruise_buttons_never_switch_lane_keeping(self):
    with variant(raw_config_with_cruise_buttons()) as (cfg, dbc_path):
      h = HondaStepperHarness(fcan_dbc=dbc_path, cfg=cfg)
      h.set(CRUISE_BUTTONS=0)
      h.step()
      for value, button in ((1, ButtonType.mainCruise), (2, ButtonType.cancel)):
        h.set(CRUISE_BUTTONS=value)
        CS, _ = h.step()
        self.assertEqual([(b.type, b.pressed) for b in CS.buttonEvents], [(button, True)])
        self.assertFalse(CS.cruiseState.available)
        h.set(CRUISE_BUTTONS=0)
        h.step()

  def test_motor_presence(self):
    h = HondaStepperHarness()
    CS, _ = h.step(motor=True)
    self.assertTrue(h.CI.CS.motor_present)
    self.assertFalse(CS.steerFaultTemporary)
    for _ in range(9):
      CS, _ = h.step(motor=False)
    self.assertTrue(h.CI.CS.motor_present)  # under 100 ms of silence
    CS, _ = h.step(motor=False)
    self.assertFalse(h.CI.CS.motor_present)
    self.assertTrue(CS.steerFaultTemporary)
    self.assertTrue(CS.canValid)  # the lift lever up is not a CAN fault
    self.assertFalse(CS.canTimeout)
    CS, _ = h.step(motor=True)
    self.assertTrue(h.CI.CS.motor_present)

  def test_lift_lever_up_turns_lane_keeping_off(self):
    h = HondaStepperHarness()
    CS, _ = h.step(motor=True)
    self.assertFalse(CS.steeringDisengage)
    self.assertFalse(CS.blockPcmEnable)
    for _ in range(10):
      CS, _ = h.step(motor=False)
    self.assertTrue(CS.steeringDisengage)   # rising edge -> steerDisengage, the normal disengage sound
    self.assertFalse(CS.cruiseState.available)
    self.assertTrue(CS.blockPcmEnable)
    CS, _ = h.step(motor=True)
    self.assertFalse(CS.steeringDisengage)
    self.assertFalse(CS.blockPcmEnable)

  def test_soft_off_is_temporary_fault(self):
    h = HondaStepperHarness()
    h.status["CONTROL_STATUS"] = 3 | 4
    CS, _ = h.step()
    self.assertTrue(h.CI.CS.motor_soft_off)
    self.assertTrue(CS.steerFaultTemporary)

  def test_free_spin_is_silent(self):
    p = CarControllerParams
    scale = CONFIG.actuator.angle_scale
    fast = (p.FREE_SPIN_RATE_DEG_S + 30.0) / 360.0 / scale   # turns per second, as the motor counts them
    for direction in (1, -1):
      h = HondaStepperHarness(direction)
      # these are not free spin: turning against the push (the driver), slower than the limit, or not pushing at all
      for torque, rev_s in ((1.0, -fast), (1.0, 0.5 * fast), (0.0, fast), (0.0, 1.9)):
        h.status.update(STEERING_TORQUE=torque)
        h.turning(rev_s)
        for _ in range(3 * p.FREE_SPIN_FRAMES):
          CS, _ = h.step()
        self.assertFalse(h.CI.CS.motor_free_spin, (direction, torque, rev_s))
        self.assertFalse(CS.steeringPressed, (direction, torque, rev_s))
      # pushing and turning the same way fast, for 0.1 s
      h.status.update(STEERING_TORQUE=-1.0)
      h.turning(-fast)
      for _ in range(p.FREE_SPIN_FRAMES - 1):
        CS, _ = h.step()
      self.assertFalse(CS.steeringPressed)
      CS, _ = h.step()
      self.assertTrue(h.CI.CS.motor_free_spin)
      # openpilot sees the driver steering (the wheel is past the teeth): it holds back without a warning
      self.assertTrue(CS.steeringPressed)
      self.assertFalse(CS.steerFaultTemporary)
      # the motor is off and has stopped; as long as the steering wheel doesn't turn it, its gear is still off the
      # teeth and lane keeping holds back for the whole turn
      h.status.update(STEERING_TORQUE=0.0, STEERING_ANGLE=40.0)
      h.turning(0.0)
      for _ in range(500):
        CS, _ = h.step()
        self.assertTrue(CS.steeringPressed)
        self.assertFalse(CS.steerFaultTemporary)
      # the steering wheel turns the motor again: the gear is back on the teeth, lane keeping may steer 1 s later
      h.status.update(STEERING_ANGLE=40.0 - 2.0 / scale)
      for _ in range(p.FREE_SPIN_HOLD_FRAMES):
        CS, _ = h.step()
        self.assertTrue(CS.steeringPressed)
      CS, _ = h.step()
      self.assertFalse(CS.steeringPressed)
      self.assertFalse(h.CI.CS.motor_free_spin)

  def test_free_spin_past_the_speed_field_limit(self):
    # the motor's speed field wraps above 2 turns a second: a motor spinning at 2.8 reads as turning the other way
    # at 1.2. The rate comes from the angle instead, so the spin is still caught
    p = CarControllerParams
    for rev_s in (2.8, 3.9, 1.0):
      h = HondaStepperHarness()
      h.status.update(STEERING_TORQUE=1.0)
      h.turning(rev_s)
      CS, _ = h.step()
      if rev_s > 2.0:
        self.assertLess(h.status["STEERING_SPEED"], 0.0, rev_s)   # what the field says
      for _ in range(p.FREE_SPIN_FRAMES):
        CS, _ = h.step()
      self.assertTrue(CS.steeringPressed, rev_s)
      self.assertAlmostEqual(h.CI.CS.motor_rate_fw, rev_s * 360.0, delta=15.0)

  def test_motor_rate_from_the_angle(self):
    # the rate is the angle between status messages over their time apart, the angle field's wrap undone, and it
    # starts again from nothing each time the motor comes on
    h = HondaStepperHarness()
    h.turning(0.5)
    for _ in range(3):
      h.step()
    self.assertAlmostEqual(h.CI.CS.motor_rate_fw, 180.0, delta=13.0)   # the angle field counts in 0.125 degrees
    h.status["STEERING_ANGLE"] = 4095.0        # about to wrap round to -4096
    h.step()
    CS, _ = h.step()
    self.assertAlmostEqual(h.CI.CS.motor_rate_fw, 180.0, delta=13.0)
    self.assertLess(CS.steeringAngleDeg, 0.0)   # the field has wrapped; the rate hasn't noticed
    for _ in range(12):
      h.step(motor=False)                      # lift lever up
    self.assertEqual(h.CI.CS.motor_rate_fw, 0.0)
    h.status["STEERING_ANGLE"] = 0.0
    h.step()                                   # the first message after power-up: no rate yet
    self.assertEqual(h.CI.CS.motor_rate_fw, 0.0)
    h.step()
    self.assertAlmostEqual(h.CI.CS.motor_rate_fw, 180.0, delta=13.0)   # the angle field counts in 0.125 degrees

  def test_free_spin_retries_after_10_s_and_lever_up_resets(self):
    p = CarControllerParams
    h = HondaStepperHarness()
    h.status.update(STEERING_TORQUE=1.0)
    h.turning(1.0)
    for _ in range(p.FREE_SPIN_FRAMES + 1):
      h.step()
    self.assertTrue(h.CI.CS.motor_free_spin)
    h.status.update(STEERING_TORQUE=0.0)
    h.turning(0.0)
    for _ in range(p.FREE_SPIN_REST_FRAMES + p.FREE_SPIN_RETRY_FRAMES + 2):
      CS, _ = h.step()
    self.assertFalse(CS.steeringPressed)   # nothing seen of the teeth for 10 s: lane keeping may try again
    # caught again, then the lift lever goes up and down: the gear was put back on the teeth by hand
    h.status.update(STEERING_TORQUE=1.0)
    h.turning(1.0)
    for _ in range(p.FREE_SPIN_FRAMES + 1):
      h.step()
    self.assertTrue(h.CI.CS.motor_free_spin)
    for _ in range(12):
      h.step(motor=False)
    h.status.update(STEERING_TORQUE=0.0)
    h.turning(0.0)
    CS, _ = h.step()
    self.assertFalse(h.CI.CS.motor_free_spin)
    self.assertFalse(CS.steeringPressed)

  def test_angle_and_torque_scaling(self):
    for direction in (1, -1):
      h = HondaStepperHarness(direction)
      h.status.update(STEERING_ANGLE=10.0, STEERING_TORQUE=1.0)
      CS, _ = h.step()
      self.assertAlmostEqual(CS.steeringAngleDeg, direction * 10.0 * CONFIG.actuator.angle_scale, places=3)
      self.assertAlmostEqual(CS.steeringTorqueEps, direction * 1.0 / CONFIG.actuator.torque_cmd_scale, places=3)
      self.assertEqual(h.CI.CS.motor_torque_units, direction * 8)


class TestInterface(unittest.TestCase):
  def _params(self, docs=False):
    CI = interfaces[PLATFORM]
    return CI.get_params(PLATFORM, gen_empty_fingerprint(), [], False, False, docs)

  def test_default_measures_turning_from_gyro(self):
    CP = self._params()
    self.assertTrue(CP.lateralTuning.torque.useYawRate)
    self.assertEqual(CP.minSteerSpeed, interface_mod.YAW_RATE_MIN_STEER_SPEED)
    self.assertFalse(CP.flags & HondaStepperFlags.FCAN_STEER_ANGLE)
    self.assertFalse(CP.openpilotLongitudinalControl)

  def test_lane_keeping_off_until_setup_checks_pass(self):
    self.assertTrue(self._params().dashcamOnly)
    self.assertFalse(self._params(docs=True).dashcamOnly)  # still listed in the vehicle picker
    raw = raw_default_config()
    raw["setup"] = {"parked": "2026-10-07 10:00", "drive": "2026-10-07 10:30"}
    with mock.patch.object(interface_mod, "CONFIG", parse_config(raw)):
      self.assertFalse(self._params().dashcamOnly)

  def test_fcan_angle_and_stale_generated_files(self):
    # a checked config with an F-CAN steering angle that the generated files don't match yet
    raw = raw_config_with_fcan_angle()
    raw["setup"] = {"parked": "2026-10-07 10:00", "drive": "2026-10-07 10:30"}
    with mock.patch.object(interface_mod, "CONFIG", parse_config(raw)):
      CP = self._params()
    self.assertTrue(CP.flags & HondaStepperFlags.FCAN_STEER_ANGLE)
    self.assertFalse(CP.lateralTuning.torque.useYawRate)
    self.assertEqual(CP.minSteerSpeed, 0.0)
    self.assertTrue(CP.dashcamOnly)


class TestCarStateFcanAngle(unittest.TestCase):
  def test_reads_fcan_angle(self):
    with variant(raw_config_with_fcan_angle()) as (cfg, dbc_path):
      h = HondaStepperHarness(fcan_dbc=dbc_path, cfg=cfg)
      h.set(STEER_ANGLE=-12.3)
      h.status["STEERING_ANGLE"] = 50.0  # the motor encoder is ignored when the car reports the angle
      CS, _ = h.step()
    self.assertAlmostEqual(CS.steeringAngleDeg, -12.3, places=3)


class TestCarController(unittest.TestCase):
  def _commands(self, sends):
    out = []
    for addr, dat, bus in sends:
      self.assertEqual((addr, bus), (0x22E, 1))
      mode = (dat[1] >> 4) & 0x3
      torque = struct.unpack('<b', bytes([dat[4]]))[0]
      out.append((mode, torque))
    return out

  def test_ramps_and_rate(self):
    h = HondaStepperHarness()
    params = CarControllerParams(h.CP)
    commanded = []
    for frame in range(200):
      # the motor reports what it was last told
      if commanded:
        h.status["STEERING_TORQUE"] = commanded[-1][1] * TORQUE_UNIT
      _, sends = h.step(CC=lat_active_cc(1.0))
      cmds = self._commands(sends)
      self.assertEqual(len(cmds), 1 if frame % params.STEER_STEP == 0 else 0)
      commanded += cmds
    torques = [t for _, t in commanded]
    self.assertTrue(all(m == StepperMode.TORQUE_CONTROL for m, _ in commanded))
    self.assertTrue(all(0 <= b - a <= params.STEER_DELTA_UP for a, b in zip(torques, torques[1:], strict=False)))
    self.assertEqual(torques[-1], params.STEER_MAX)

  def test_waits_for_motor_torque(self):
    h = HondaStepperHarness()
    params = CarControllerParams(h.CP)
    h.status["STEERING_TORQUE"] = 0.0  # motor not following
    last = 0
    for _ in range(100):
      _, sends = h.step(CC=lat_active_cc(1.0))
      for _, t in self._commands(sends):
        last = t
    self.assertEqual(last, params.STEER_ERROR_MAX)

  def test_disengage_releases_immediately(self):
    h = HondaStepperHarness()
    for _ in range(40):
      h.status["STEERING_TORQUE"] = h.CI.CC.apply_torque_last * TORQUE_UNIT
      h.step(CC=lat_active_cc(1.0))
    self.assertGreater(h.CI.CC.apply_torque_last, 0)
    CC = structs.CarControl()
    _, sends = h.step(CC=CC)
    _, sends2 = h.step(CC=CC)
    self.assertEqual(self._commands(sends + sends2), [(StepperMode.OFF, 0)])

  def test_soft_off_rearms_with_off(self):
    h = HondaStepperHarness()
    h.status["CONTROL_STATUS"] = 3 | 4
    h.step()
    _, sends = h.step(CC=lat_active_cc(1.0))
    _, sends2 = h.step(CC=lat_active_cc(1.0))
    self.assertEqual(self._commands(sends + sends2), [(StepperMode.OFF, 0)])

  def test_free_spin_turns_motor_off(self):
    h = HondaStepperHarness()
    p = CarControllerParams
    h.status.update(STEERING_TORQUE=1.0)   # pushing left ...
    h.turning(1.0)                         # ... and spinning left at 360 degrees per second
    cmds = []
    for _ in range(p.FREE_SPIN_FRAMES + 3):
      _, sends = h.step(CC=lat_active_cc(1.0))
      cmds += self._commands(sends)
    self.assertTrue(h.CI.CS.motor_free_spin)
    self.assertEqual(cmds[0][0], StepperMode.TORQUE_CONTROL)
    self.assertEqual(cmds[-1], (StepperMode.OFF, 0))
    self.assertEqual(h.CI.CC.apply_torque_last, 0)

  def test_silent_without_motor(self):
    h = HondaStepperHarness()
    for _ in range(12):
      h.step(motor=False)
    for _ in range(4):
      _, sends = h.step(motor=False, CC=lat_active_cc(1.0))
      self.assertEqual(sends, [])

  def test_direction(self):
    for direction in (1, -1):
      h = HondaStepperHarness(direction)
      h.step()
      _, sends = h.step(CC=lat_active_cc(1.0))
      sends += h.step(CC=lat_active_cc(1.0))[1]
      torques = [t for _, t in self._commands(sends)]
      self.assertEqual(torques[-1], direction * 1)


class TestWithSimMotor(unittest.TestCase):
  """The car port against the simulated motor: its status frames in, the car port's commands back to it."""

  def _run(self, h: HondaStepperHarness, motor: SimMotor, CC, seconds: float) -> list:
    states = []
    for _ in range(round(seconds / DT_CTRL)):
      frames = motor.advance(h.nanos * 1e-9 + DT_CTRL)
      CS, sends = h.step(CC=CC, motor_frames=frames)
      for addr, dat, bus in sends:
        self.assertEqual((addr, bus), (0x22E, 1))
        motor.command(motor.t, bytes(dat))
      states.append(CS)
    return states

  def test_lane_keeping_on_the_teeth_is_not_free_spin(self):
    h, motor = HondaStepperHarness(), SimMotor()
    states = self._run(h, motor, lat_active_cc(1.0), 1.0)
    self.assertFalse(any(cs.steeringPressed or cs.steerFaultTemporary for cs in states[2:]))
    self.assertGreater(motor.wheel_deg, 10.0)   # lane keeping turned the wheel left
    self.assertTrue(motor.in_teeth(motor.wheel_deg))
    self.assertEqual(motor.free_spin_deg, 0.0)

  def test_motor_off_the_teeth_is_stopped(self):
    h, motor = HondaStepperHarness(), SimMotor()
    motor.wheel_deg = motor.teeth_center_deg + 50.0   # the driver turned the wheel past the end of the teeth
    states = self._run(h, motor, lat_active_cc(1.0), 0.5)
    # it spun for about 0.1 s before the car port turned it off, and stays off
    self.assertGreater(motor.free_spin_deg, 0.0)
    self.assertLess(motor.free_spin_deg, 0.2 * FREE_SPIN_FW_DEG_S)
    self.assertEqual(motor.mode, StepperMode.OFF)
    self.assertTrue(states[-1].steeringPressed)
    self.assertFalse(any(cs.steerFaultTemporary for cs in states[2:]))   # no warning
    spun = motor.free_spin_deg
    # held through the whole turn: no retries while the steering wheel stays past the teeth
    states = self._run(h, motor, lat_active_cc(1.0), 3.0)
    self.assertEqual(motor.free_spin_deg, spun)
    self.assertTrue(all(cs.steeringPressed and not cs.steerFaultTemporary for cs in states))

    # the driver turns the wheel back: the teeth turn the motor again, and 1 s later lane keeping steers again
    motor.turn_wheel(motor.teeth_center_deg)
    states = self._run(h, motor, lat_active_cc(1.0), 0.9)
    self.assertTrue(all(cs.steeringPressed for cs in states))
    states = self._run(h, motor, lat_active_cc(1.0), 0.6)
    self.assertFalse(states[-1].steeringPressed)
    self.assertEqual(motor.mode, StepperMode.TORQUE_CONTROL)
    self.assertEqual(motor.free_spin_deg, spun)
    self.assertGreater(motor.wheel_deg, motor.teeth_center_deg)

  def test_a_fast_spin_is_caught_too(self):
    # the motor spinning freely faster than its speed field can show (2 turns a second): the car port works the
    # rate out from the angle, so it still catches the spin within about 0.1 s
    for fw_deg_s in (1000.0, 1400.0):
      h, motor = HondaStepperHarness(), SimMotor()
      motor.free_spin_fw_deg_s = fw_deg_s
      motor.wheel_deg = motor.teeth_center_deg + 50.0
      states = self._run(h, motor, lat_active_cc(1.0), 0.5)
      self.assertGreater(motor.free_spin_deg, 0.0)
      self.assertLess(motor.free_spin_deg, 0.2 * fw_deg_s, fw_deg_s)
      self.assertEqual(motor.mode, StepperMode.OFF)
      self.assertTrue(states[-1].steeringPressed)

  def test_one_hold_per_turn_then_a_retry_after_10_s(self):
    h, motor = HondaStepperHarness(), SimMotor()
    motor.wheel_deg = motor.teeth_center_deg - 50.0
    states = self._run(h, motor, lat_active_cc(-1.0), 12.0)
    held = [cs.steeringPressed for cs in states]
    starts = sum(1 for a, b in zip(held, held[1:], strict=False) if b and not a)
    self.assertEqual(starts, 2)   # caught once, held 10 s, one retry
    self.assertFalse(any(cs.steerFaultTemporary for cs in states[2:]))
    self.assertLess(motor.free_spin_deg, 2 * 0.2 * FREE_SPIN_FW_DEG_S)

  def test_fast_turn_by_the_driver_recovers(self):
    # the driver turns the wheel fast the same way lane keeping pushes: that looks like free spin, so the motor is
    # turned off, and lane keeping steers again about 1 s after the driver's next small correction
    h, motor = HondaStepperHarness(), SimMotor()
    self._run(h, motor, lat_active_cc(1.0), 0.5)
    for _ in range(15):                       # 130 degrees per second to the left for 0.15 s, staying on the teeth
      motor.turn_wheel(motor.wheel_deg + 1.3, steps=1)
      self._run(h, motor, lat_active_cc(1.0), 0.01)
    self.assertTrue(motor.in_teeth(motor.wheel_deg))
    self.assertTrue(h.CI.CS.motor_free_spin)
    self._run(h, motor, lat_active_cc(1.0), 0.3)      # the driver holds the wheel still
    self.assertTrue(h.CI.CS.motor_free_spin)
    motor.turn_wheel(motor.wheel_deg - 3.0)            # a small correction
    states = self._run(h, motor, lat_active_cc(1.0), 1.2)
    self.assertFalse(states[-1].steeringPressed)
    self.assertEqual(motor.free_spin_deg, 0.0)


if __name__ == "__main__":
  unittest.main()
