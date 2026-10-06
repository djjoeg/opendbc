from opendbc.can import CANPacker
from opendbc.car.hondastepper.values import CanBus, StepperMode

STEERING_COMMAND = "STEERING_COMMAND"   # 0x22E, openpilot -> motor
STEERING_STATUS = "STEERING_STATUS"     # 0x22F, motor -> openpilot


def create_steer_command(packer: CANPacker, counter: int, mode: StepperMode, torque_fw_nm: float):
  """Motor command. Torque is in firmware units (Nm at the firmware's assumed output).
  The angle field is always zero: angle mode is never used and the panda rejects non-zero angles."""
  values = {
    "COUNTER": counter % 16,
    "STEER_MODE": int(mode),
    "STEER_ANGLE": 0,
    "STEER_TORQUE": torque_fw_nm if mode == StepperMode.TORQUE_CONTROL else 0.0,
  }
  return packer.make_can_msg(STEERING_COMMAND, CanBus.SERVO, values)
