from enum import IntEnum, IntFlag

from opendbc.car import Bus, CarSpecs, PlatformConfig, Platforms
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts, SupportType
from opendbc.car.hondastepper.config import DBC_NAME, SERVO_DBC_NAME, load_config_or_default

# Single source of truth for signal locations and actuator setup, see config.json. A saved config that can't be
# used falls back to the default in the code (lane keeping then stays off) instead of stopping openpilot for every car.
CONFIG, CONFIG_PROBLEM = load_config_or_default()


# the panda safety mode's param: stand-alone, nothing read from the car (opendbc/safety/modes/honda_stepper.h)
HONDA_STEPPER_PARAM_STANDALONE = 1


class CanBus:
  FCAN = 0   # car F-CAN through the harness CAN0 pair (OBD pins 6/14), listen only; unused stand-alone
  SERVO = 1  # StepperServoCAN motor alone on the harness CAN1 pair
  SERVO_SENT = SERVO + 128  # openpilot's messages to the motor, reported back by the panda as it sends them


class StepperMode(IntEnum):
  OFF = 0             # motor driver disabled, wheel free
  TORQUE_CONTROL = 1  # apply STEER_TORQUE
  ANGLE_CONTROL = 2   # never used, blocked by the panda safety mode
  SOFT_OFF = 3        # firmware ramps torque to zero, then locks until OFF is sent


# STEERING_STATUS.CONTROL_STATUS bits (StepperServoCAN control_api.c)
class StepperControlStatus(IntFlag):
  ENABLED = 1
  SENSORED = 2
  SOFT_OFF = 4
  CLOSED_LOOP = 8


# STEERING_STATUS.DEBUG_STATES bits (StepperServoCAN control_api.c)
class StepperDebugStates(IntFlag):
  DRIVER_ENABLED = 1
  CAN_RX_CHECKSUM_ERROR = 32
  MOTION_TASK_OVERRUN = 64
  SERVICE_TASK_OVERRUN = 128


class CarControllerParams:
  # Torque values are CAN units of the motor command: 0.125 Nm at the gearing the motor firmware assumes.
  # The panda safety mode (opendbc/safety/modes/honda_stepper.h) enforces the same numbers.
  STEER_STEP = 2            # one command every 20 ms; the motor soft-offs after 60 ms without one
  STEER_DELTA_UP = 1        # per command: 6.25 Nm/s
  STEER_DELTA_DOWN = 2      # per command: 12.5 Nm/s
  STEER_ERROR_MAX = 8       # 1 Nm the command may run ahead of the torque the motor reports
  # lane keeping's most at the steering wheel; one hand overpowers it easily. The car setup's grip test pushes with the
  # most the motor's board gives (about 5.4 Nm), 1.41 times this, so passing it proves the motor gear stays in the teeth
  STEER_MAX_WHEEL_NM = 3.85
  SAFETY_MAX_TORQUE = 47    # the panda's hard limit: 5.9 Nm in firmware units, 4.4 Nm at the steering wheel
  MOTOR_TIMEOUT_NS = 100_000_000  # treat the motor as gone (lift lever up) after 100 ms without status
  # lane keeping turns on after this many motor status messages without a 100 ms gap, each saying the motor's
  # controller runs: 2 s at 100 Hz. The panda's safety mode uses the same count (HONDA_STEPPER_LEVER_DOWN_MSGS).
  LEVER_DOWN_MSGS = 200

  # Free spin: the steering wheel is turned further than the steering wheel teeth reach, so the motor gear is off
  # them and spins freely in the direction it pushes. On the teeth the motor turns only as fast as the steering
  # wheel, and lane keeping alone turns the wheel well under this rate. The motor is then kept off, silently, until
  # its gear is back on the teeth. Not measured on the car yet.
  # Rates are steering wheel degrees per second, worked out from the motor's angle between two of its status
  # messages (its speed field is 8 bits and wraps above 2 turns a second, which a spinning motor may well pass; the
  # angle field only wraps after 11 turns, and that wrap is undone). Frames are 10 ms status messages.
  FREE_SPIN_RATE_DEG_S = 100.0     # turning this fast in the direction it pushes ...
  FREE_SPIN_FRAMES = 10            # ... for 0.1 s counts as free spin
  FREE_SPIN_REST_DEG_S = 10.0      # after the motor is turned off it coasts; under this rate ...
  FREE_SPIN_REST_FRAMES = 10       # ... for 0.1 s, it has stopped
  FREE_SPIN_SETTLE_MAX_FRAMES = 200   # stop waiting for it to stop after 2 s
  # the stopped motor then turning this far means the steering wheel turns it again: its gear is back on the teeth
  FREE_SPIN_REMESH_DEG = 1.0
  FREE_SPIN_HOLD_FRAMES = 100      # the motor stays off for 1 s after that
  FREE_SPIN_RETRY_FRAMES = 1000    # with no sign of the teeth for 10 s, let lane keeping try again

  # Stand-alone there are no turn signals or brake to pause lane keeping, so the motor lets go when the driver turns
  # the wheel against it: the motor pushing hard one way (a share of lane keeping's most) while the steering wheel
  # turns the other way faster than lane keeping lets it unwind. It stays off until the wheel has been still for a
  # while (the driver has finished, or holds it), then lane keeping takes over again, its torque ramping up from zero.
  # Not measured on the car yet: how fast the wheel unwinds by itself out of a curve against the motor sets how low
  # these can go.
  OVERRIDE_TORQUE_SHARE = 0.6      # the motor pushing at least this share of lane keeping's most, the same way, ...
  OVERRIDE_RATE_DEG_S = 20.0       # ... while the steering wheel turns the other way faster than this on average ...
  OVERRIDE_FRAMES = 20             # ... over 0.2 s (motor status messages): the driver is steering
  OVERRIDE_STILL_DEG_S = 5.0       # then the motor stays off until the wheel turns slower than this on average ...
  OVERRIDE_STILL_WINDOW = 50       # ... over 0.5 s ...
  OVERRIDE_STILL_FRAMES = 150      # ... for 1.5 s in a row

  def __init__(self, CP):
    max_units = round(self.STEER_MAX_WHEEL_NM * CONFIG.actuator.torque_cmd_scale / 0.125)
    self.STEER_MAX = int(min(max_units, self.SAFETY_MAX_TORQUE))


class HondaStepperFlags(IntFlag):
  # Absolute steering angle is broadcast on F-CAN (VSA). Otherwise the motor's encoder angle is used.
  FCAN_STEER_ANGLE = 1
  # Not connected to the car: speed from the comma device itself (openpilot's card hands it to the car port)
  STANDALONE = 2


class CAR(Platforms):
  HONDA_ACCORD_8G_STEPPER = PlatformConfig(
    [CarDocs("Honda Accord 2008-12", "2.4L sedan with the add-on steering motor",
             car_parts=CarParts.common([CarHarness.custom]), support_type=SupportType.COMMUNITY,
             support_link="#community")],
    # mass: 2010 LX 2.4 curb weight. steerRatio: Honda lists 13.08:1 overall, 2.56 turns lock to lock ("variable"
    # in its spec sheet is the power assist, not the ratio). openpilot's ratios are usually higher than listed (comma uses
    # 16.33 for the 2018 Accord, listed at 11.82), so 15.3 is a guess between the two. It matters little here: the
    # steering angle is only relative to where the motor started, so above walking pace the steering controller
    # measures the car's turning with the comma device's gyro, and paramsd doesn't learn this value.
    CarSpecs(mass=3269 * CV.LB_TO_KG, wheelbase=2.799, steerRatio=15.3, centerToFrontRatio=0.39),
    {Bus.pt: DBC_NAME, Bus.alt: SERVO_DBC_NAME},
  )


DBC = CAR.create_dbc_map()
