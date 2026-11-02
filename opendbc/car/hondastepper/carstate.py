import math
from collections import deque
from enum import IntEnum

from opendbc.can import CANParser
from opendbc.car import Bus, create_button_events, structs
from opendbc.car.carlog import carlog
from opendbc.car.hondastepper.config import SPEED_UNITS
from opendbc.car.hondastepper.steppercan import STEERING_COMMAND, STEERING_STATUS
from opendbc.car.hondastepper.values import CONFIG, DBC, CanBus, CarControllerParams, StepperControlStatus, StepperDebugStates
from opendbc.car.interfaces import CarStateBase

ButtonType = structs.CarState.ButtonEvent.Type
GearShifter = structs.CarState.GearShifter

TORQUE_UNIT = 0.125      # Nm per STEERING_TORQUE / STEER_TORQUE bit
ANGLE_WRAP_DEG = 8192.0  # STEERING_ANGLE is 16 bits of 0.125 degrees: it wraps round every 8192 firmware degrees
BLINKER_HOLD_FRAMES = 50  # a flashing lamp reads as on for 0.5 s after each flash

# CRUISE_BUTTONS values (Honda layout): 1 main, 2 cancel, 3 decel/set, 4 accel/resume
CRUISE_BUTTONS_DICT = {1: ButtonType.mainCruise, 2: ButtonType.cancel, 3: ButtonType.decelCruise, 4: ButtonType.accelCruise}


class FreeSpin(IntEnum):
  NONE = 0      # normal: the motor gear is in the steering wheel teeth
  SETTLING = 1  # caught spinning: the motor is off and coasting to a stop
  WAITING = 2   # stopped, or just switched on: waiting for the steering wheel to turn the motor (gear in the teeth)
  HOLD = 3      # in the teeth; the motor stays off a little longer


STANDSTILL_MPS = 0.3      # stand-alone: the comma device's own speed under this counts as stopped


class CarState(CarStateBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.cfg = CONFIG
    self.car_network = CONFIG.car_network
    # stand-alone: the comma device's own speed (m/s) and whether it can be used, handed in by openpilot's card before
    # every update (set_device_speed). Until it is, there is no speed and the motor doesn't push.
    self.device_speed = 0.0
    self.device_speed_valid = False
    # stand-alone: the driver turning the wheel against the motor; the motor lets go (CarControllerParams.OVERRIDE_*)
    self.driver_override = False
    self.override_frames = 0
    self.override_window: deque[tuple[float, float]] = deque(maxlen=60)   # (steering wheel deg, motor torque units)
    self.direction = CONFIG.actuator.direction
    self.angle_scale = CONFIG.actuator.angle_scale
    self.torque_meas_scale = 1.0 / CONFIG.actuator.torque_cmd_scale

    sigs = CONFIG.signals
    self.gear_values = sigs["GEAR"].values if "GEAR" in sigs else {}
    self.speed_unit = SPEED_UNITS[sigs["VEHICLE_SPEED"].unit]
    self.wheel_speed_unit = SPEED_UNITS[sigs["WHEEL_SPEED_FL"].unit] if CONFIG.has_wheel_speeds else 1.0

    # motor state, used by the car controller
    self.motor_present = False
    self.motor_last_nanos = 0     # when the last motor status came
    self.lever_counting = False   # the panda has passed a motor command since the motor came on
    self.motor_status_msgs = 0    # running motor status messages without a gap, up to LEVER_DOWN_MSGS
    self.lever_down = False       # the motor has reported for 2 s: lane keeping is on
    self.motor_soft_off = False
    self.motor_torque_units = 0   # STEERING_TORQUE in CAN units, openpilot direction (left positive)
    # The motor gear isn't known to be in the steering wheel teeth until the wheel has turned the motor: keep the
    # motor off till then (update_free_spin)
    self.motor_free_spin = True
    self.free_spin_state = FreeSpin.WAITING
    self.free_spin_frames = 0     # frames in a row of spinning (NONE), or frames spent in the current state
    self.free_spin_rest_frames = 0
    self.free_spin_ref_angle: float | None = None
    self.debug_states_logged = 0
    self.angle_fw_prev: float | None = None   # the motor's angle and the time of its last message, for its rate
    self.angle_ts_prev = 0
    self.motor_rate_fw = 0.0      # firmware degrees per second, from the angle

    self.cruise_buttons = 0

  def _sig(self, cp: CANParser, name: str) -> float:
    return cp.vl[self.cfg.signals[name].message][name]

  def set_device_speed(self, speed: float, valid: bool) -> None:
    """Stand-alone: the comma device's own speed (m/s, already scaled to GPS by the drive check's result) and
    whether it can be used now. openpilot's card calls this before every update."""
    self.device_speed = max(float(speed), 0.0) if valid else 0.0
    self.device_speed_valid = bool(valid)

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp = can_parsers[Bus.pt]
    cp_servo = can_parsers[Bus.alt]
    cp_sent = can_parsers[Bus.loopback]
    sigs = self.cfg.signals
    ret = structs.CarState()

    if self.car_network:
      self.update_car_data(ret, cp)
    else:
      # Stand-alone: nothing from the car. The speed is the comma device's own; the brake, gas, turn signals, gear
      # and doors aren't known, so they read as released, off, drive and closed (the events they drive never come).
      ret.vEgoRaw = self.device_speed
      ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
      ret.standstill = ret.vEgo < STANDSTILL_MPS
      ret.gearShifter = GearShifter.drive

    # ***** motor and the lift lever *****
    status = cp_servo.vl[STEERING_STATUS]
    control_statuses = cp_servo.vl_all[STEERING_STATUS]["CONTROL_STATUS"]
    if len(control_statuses):
      self.motor_last_nanos = cp_servo.ts_nanos[STEERING_STATUS]["CONTROL_STATUS"]
    # The motor silent for 100 ms while the car's own messages keep coming: the lift lever is up (its power is cut)
    # or the motor is unplugged. Timed against the car's messages like the panda does, so a hold-up in the comma
    # device's own message handling can't look like a lifted lever. Stand-alone, against the comma device's CAN
    # packets, which its panda daemon sends 100 times a second whether they hold any message or not; their times are
    # when the daemon sent them, so a hold-up here doesn't count either.
    clock_ns = cp.last_nonempty_nanos if self.car_network else cp_servo._last_update_nanos
    silent_ns = clock_ns - self.motor_last_nanos
    self.motor_present = self.motor_last_nanos > 0 and silent_ns < CarControllerParams.MOTOR_TIMEOUT_NS
    self.update_lever(control_statuses, len(cp_sent.vl_all[STEERING_COMMAND]["COUNTER"]) > 0)
    ret.cruiseState.available = self.lever_down
    ret.cruiseState.enabled = False
    control_status = StepperControlStatus(int(status["CONTROL_STATUS"]))
    self.motor_soft_off = bool(control_status & StepperControlStatus.SOFT_OFF)
    motor_ready = bool(control_status & StepperControlStatus.ENABLED)

    debug_states = int(status["DEBUG_STATES"])
    sticky_warnings = debug_states & (StepperDebugStates.CAN_RX_CHECKSUM_ERROR | StepperDebugStates.MOTION_TASK_OVERRUN |
                                      StepperDebugStates.SERVICE_TASK_OVERRUN)
    if self.motor_present and sticky_warnings and sticky_warnings != self.debug_states_logged:
      carlog.warning(f"StepperServoCAN reports DEBUG_STATES {debug_states:#x} (sticky until the motor restarts)")
      self.debug_states_logged = sticky_warnings

    self.motor_torque_units = self.direction * round(status["STEERING_TORQUE"] / TORQUE_UNIT)
    self.update_motor_rate(status["STEERING_ANGLE"], len(control_statuses) > 0)
    motor_rate = self.direction * self.motor_rate_fw * self.angle_scale        # steering wheel deg/s
    motor_angle = self.direction * status["STEERING_ANGLE"] * self.angle_scale  # steering wheel deg
    self.update_free_spin(motor_rate, motor_angle)
    if not self.car_network:
      self.update_driver_override(cp_servo.vl_all[STEERING_STATUS]["STEERING_ANGLE"], cp_servo.vl_all[STEERING_STATUS]["STEERING_TORQUE"])
    ret.steeringTorqueEps = self.direction * status["STEERING_TORQUE"] * self.torque_meas_scale
    ret.steeringTorque = 0.0       # no driver torque sensor on a hydraulic rack
    # The steering wheel turned past the end of the teeth is the driver steering: lane keeping holds back without a
    # warning, and picks up again once the motor gear is back on the teeth. Stand-alone, so is the wheel turned
    # against the motor.
    ret.steeringPressed = self.motor_free_spin or self.driver_override
    ret.steerFaultTemporary = not self.motor_present or not motor_ready or self.motor_soft_off
    if not self.car_network and not self.device_speed_valid:
      # no speed to steer by (the comma device's own estimate isn't ready, or disagrees with GPS)
      ret.steerFaultTemporary = True
    # The motor losing power (the car switched off, or the lift lever lifted on the earlier mounts): lane keeping
    # turns off with the normal disengage sound.
    ret.steeringDisengage = not self.motor_present
    ret.blockPcmEnable = not self.motor_present

    # ***** steering angle *****
    if self.car_network and "STEER_ANGLE" in sigs:
      ret.steeringAngleDeg = self._sig(cp, "STEER_ANGLE")
    else:
      # motor encoder: zero is wherever the wheel was when the motor powered up
      ret.steeringAngleDeg = motor_angle
    ret.steeringRateDeg = motor_rate

    return ret, structs.CarStateSP()

  def update_car_data(self, ret: structs.CarState, cp: CANParser) -> None:
    """Speed, pedals, gear, doors, turn signals and buttons from the car's F-CAN (car_network)."""
    sigs = self.cfg.signals
    # ***** speed *****
    if self.cfg.has_wheel_speeds:
      self.parse_wheel_speeds(ret, self._sig(cp, "WHEEL_SPEED_FL"), self._sig(cp, "WHEEL_SPEED_FR"),
                              self._sig(cp, "WHEEL_SPEED_RL"), self._sig(cp, "WHEEL_SPEED_RR"), unit=self.wheel_speed_unit)
    else:
      ret.vEgoRaw = self._sig(cp, "VEHICLE_SPEED") * self.speed_unit
      ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = self._sig(cp, "VEHICLE_SPEED") < 1e-3

    # ***** pedals, gear, doors *****
    ret.brakePressed = bool(self._sig(cp, "BRAKE_PRESSED"))
    ret.gasPressed = "GAS_PEDAL" in sigs and self._sig(cp, "GAS_PEDAL") > 0
    if "GEAR" in sigs:
      ret.gearShifter = self.parse_gear_shifter(self.gear_values.get(int(self._sig(cp, "GEAR"))))
    else:
      ret.gearShifter = GearShifter.drive
    ret.doorOpen = "DOOR_OPEN" in sigs and bool(self._sig(cp, "DOOR_OPEN"))
    ret.seatbeltUnlatched = "SEATBELT_UNLATCHED" in sigs and bool(self._sig(cp, "SEATBELT_UNLATCHED"))

    # ***** blinkers *****
    if "LEFT_BLINKER" in sigs and "RIGHT_BLINKER" in sigs:
      ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(BLINKER_HOLD_FRAMES, bool(self._sig(cp, "LEFT_BLINKER")),
                                                                        bool(self._sig(cp, "RIGHT_BLINKER")))

    # ***** buttons *****
    if "CRUISE_BUTTONS" in sigs:
      prev_cruise_buttons = self.cruise_buttons
      self.cruise_buttons = int(self._sig(cp, "CRUISE_BUTTONS"))
      ret.buttonEvents = create_button_events(self.cruise_buttons, prev_cruise_buttons, CRUISE_BUTTONS_DICT)

  def update_lever(self, control_statuses: list[float], command_passed: bool) -> None:
    """The lift lever is the lane keeping switch: on once the motor has reported for 2 s without a gap, every
    message saying its controller is running (CONTROL_STATUS bit 0, set once it has started up); off the moment it goes
    silent (lever up) or reports a restart. The panda's safety mode counts the same messages the same way
    (opendbc/safety/modes/honda_stepper.h). This side starts counting only after the panda has passed one of
    openpilot's motor commands since the motor came on, which it does only once it runs this car's safety mode and
    counts too. So lane keeping turns on here just after the panda allows it, also when the lever was already down as
    the car started. sunnypilot's MADS, with "Toggle with Main Cruise" on, takes cruiseState.available as its main
    switch. The car's cruise MAIN button and its own cruise control play no part."""
    p = CarControllerParams
    if not self.motor_present:
      self.lever_counting = False
      self.motor_status_msgs = 0
    else:
      if self.lever_counting:
        for control_status in control_statuses:
          running = bool(int(control_status) & StepperControlStatus.ENABLED)
          self.motor_status_msgs = min(self.motor_status_msgs + 1, p.LEVER_DOWN_MSGS) if running else 0
      # from the next message on: messages that came with the first passed command may be older than the panda's count
      self.lever_counting = self.lever_counting or command_passed
    self.lever_down = self.motor_status_msgs >= p.LEVER_DOWN_MSGS

  def update_motor_rate(self, angle_fw: float, new_message: bool) -> None:
    """The motor's rate (firmware degrees per second) from its angle between two of its status messages. The
    status's own speed field is 8 bits, 2 turns a second at most, and wraps past that, so a motor spinning freely
    could read as turning slowly the other way. The angle field wraps every 8192 degrees; that is undone here."""
    if not self.motor_present:
      self.angle_fw_prev, self.motor_rate_fw = None, 0.0
      return
    if not new_message:
      return
    if self.angle_fw_prev is not None and self.motor_last_nanos > self.angle_ts_prev:
      delta = angle_fw - self.angle_fw_prev
      if abs(delta) > ANGLE_WRAP_DEG / 2:
        delta -= math.copysign(ANGLE_WRAP_DEG, delta)
      self.motor_rate_fw = delta / ((self.motor_last_nanos - self.angle_ts_prev) * 1e-9)
    self.angle_fw_prev, self.angle_ts_prev = angle_fw, self.motor_last_nanos

  def update_free_spin(self, motor_rate: float, motor_angle: float) -> None:
    """The motor turning fast in the direction it pushes means its gear is off the steering wheel teeth (the wheel
    is turned past their end, or the motor was slid back out of them) and spinning freely. Then the motor is kept
    off, with no warning, until the gear is back in the teeth: the stopped motor starts turning again because the
    steering wheel turns it. One second later lane keeping may steer again. This keeps the gear from running into
    the teeth while it spins.

    The motor has power whenever the car is on, and the driver switches lane keeping on and off by sliding the motor
    gear into and out of the teeth. So when the motor comes on, its gear isn't known to be in the teeth either: lane
    keeping holds back the same way, silently, until the steering wheel has turned the motor. With the gear out it
    never pushes; there are no retries to spin it."""
    p = CarControllerParams
    if not self.motor_present:
      # no motor (car off, or unplugged): when it reports again, wait for the steering wheel to turn it
      self.free_spin_state, self.free_spin_frames, self.free_spin_ref_angle = FreeSpin.WAITING, 0, None
      self.motor_free_spin = True
      return

    state = self.free_spin_state
    self.free_spin_frames += 1
    if state == FreeSpin.NONE:
      pushing = self.motor_torque_units != 0
      spinning = pushing and self.motor_torque_units * motor_rate > 0 and abs(motor_rate) > p.FREE_SPIN_RATE_DEG_S
      if not spinning:
        self.free_spin_frames = 0
      elif self.free_spin_frames >= p.FREE_SPIN_FRAMES:
        carlog.warning(f"hondastepper: motor spinning freely at {motor_rate:.0f} deg/s: off until the gear is back on the teeth")
        state, self.free_spin_frames, self.free_spin_rest_frames = FreeSpin.SETTLING, 0, 0
    elif state == FreeSpin.SETTLING:
      self.free_spin_rest_frames = self.free_spin_rest_frames + 1 if abs(motor_rate) < p.FREE_SPIN_REST_DEG_S else 0
      if self.free_spin_rest_frames >= p.FREE_SPIN_REST_FRAMES or self.free_spin_frames >= p.FREE_SPIN_SETTLE_MAX_FRAMES:
        state, self.free_spin_frames, self.free_spin_ref_angle = FreeSpin.WAITING, 0, motor_angle
    elif state == FreeSpin.WAITING:
      if self.free_spin_ref_angle is None:
        self.free_spin_ref_angle = motor_angle      # the motor just came on: where it stands now
      elif abs(motor_angle - self.free_spin_ref_angle) > p.FREE_SPIN_REMESH_DEG:
        state, self.free_spin_frames = FreeSpin.HOLD, 0
    elif state == FreeSpin.HOLD:
      if self.free_spin_frames >= p.FREE_SPIN_HOLD_FRAMES:
        state, self.free_spin_frames = FreeSpin.NONE, 0

    self.free_spin_state = state
    self.motor_free_spin = state != FreeSpin.NONE

  def update_driver_override(self, angles_fw: list[float], torques_nm: list[float]) -> None:
    """Stand-alone: the driver turning the steering wheel against the motor (there are no turn signals or brake to
    pause lane keeping). The motor pushing hard one way while the wheel turns the other way: the motor lets go, and
    stays off until the wheel has been still for a while. Worked out per motor status message (10 ms) from the
    wheel's travel over a window of them: the angle field's 0.125 degree step is too coarse for a rate from two
    messages in a row."""
    p = CarControllerParams
    if not self.motor_present or self.motor_free_spin:
      self.driver_override, self.override_frames = False, 0
      self.override_window.clear()
      return
    max_units = p.STEER_MAX_WHEEL_NM * self.cfg.actuator.torque_cmd_scale / TORQUE_UNIT
    for angle_fw, torque_nm in zip(angles_fw, torques_nm, strict=True):
      angle = self.direction * angle_fw * self.angle_scale
      torque = self.direction * torque_nm / TORQUE_UNIT       # CAN units, openpilot direction
      self.override_window.append((angle, torque))
      if not self.driver_override:
        n = p.OVERRIDE_FRAMES
        if len(self.override_window) <= n:
          continue
        recent = list(self.override_window)[-(n + 1):]
        travel = recent[-1][0] - recent[0][0]                 # steering wheel degrees over n messages
        pushes = [t for _, t in recent[1:]]
        hard = all(abs(t) >= p.OVERRIDE_TORQUE_SHARE * max_units for t in pushes)
        same_way = all(t > 0 for t in pushes) or all(t < 0 for t in pushes)
        if hard and same_way and pushes[-1] * travel < 0 and abs(travel) > p.OVERRIDE_RATE_DEG_S * n * 0.01:
          carlog.warning(f"hondastepper: the wheel turned {travel:.1f} deg against the motor in {n * 10} ms: the motor lets go")
          self.driver_override, self.override_frames = True, 0
      else:
        n = p.OVERRIDE_STILL_WINDOW
        window = list(self.override_window)[-(n + 1):]
        still = len(window) > n and abs(window[-1][0] - window[0][0]) < p.OVERRIDE_STILL_DEG_S * n * 0.01
        self.override_frames = self.override_frames + 1 if still else 0
        if self.override_frames >= p.OVERRIDE_STILL_FRAMES:
          self.driver_override, self.override_frames = False, 0

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    # stand-alone: nothing is read from the car's network (an empty parser, always valid)
    pt_messages = [(m.name, m.freq) for m in CONFIG.messages.values()] if CONFIG.car_network else []
    # NaN frequency: a missing motor status is normal (lift lever up), carstate tracks it instead of failing CAN validity
    servo_messages = [(STEERING_STATUS, float("nan"))]
    # openpilot's own motor commands as the panda passed them on: it reports every message it sends
    sent = CANParser(DBC[CP.carFingerprint][Bus.alt], [(STEERING_COMMAND, float("nan"))], CanBus.SERVO_SENT)
    # a command the panda refused leaves a gap in the counter here, which is no CAN fault
    sent.message_states[sent.dbc.name_to_msg[STEERING_COMMAND].address].ignore_counter = True
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, CanBus.FCAN),
      Bus.alt: CANParser(DBC[CP.carFingerprint][Bus.alt], servo_messages, CanBus.SERVO),
      Bus.loopback: sent,
    }
