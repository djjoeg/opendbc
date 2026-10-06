from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.hondastepper.carstate import TORQUE_UNIT
from opendbc.car.hondastepper.steppercan import create_steer_command
from opendbc.car.hondastepper.values import CONFIG, CarControllerParams, StepperMode
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_meas_steer_torque_limits


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(dbc_names[Bus.alt])
    self.direction = CONFIG.actuator.direction
    self.apply_torque_last = 0   # CAN units, openpilot direction
    self.command_counter = 0

  def update(self, CC, CC_SP, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    if self.frame % self.params.STEER_STEP == 0:
      if not CS.motor_present:
        # lift lever up or motor unplugged: nothing to talk to
        mode = StepperMode.OFF
        apply_torque = 0
      elif CS.motor_soft_off:
        # the firmware soft-offs after 60 ms without commands and stays off until it gets Off
        mode = StepperMode.OFF
        apply_torque = 0
      elif CS.motor_free_spin:
        # the motor gear is off the steering wheel teeth and spinning: let it stop (openpilot sees the driver steering)
        mode = StepperMode.OFF
        apply_torque = 0
      elif CC.latActive:
        mode = StepperMode.TORQUE_CONTROL
        new_torque = int(round(actuators.torque * self.params.STEER_MAX))
        apply_torque = apply_meas_steer_torque_limits(new_torque, self.apply_torque_last, CS.motor_torque_units, self.params)
      else:
        # any disengagement releases the wheel at once
        mode = StepperMode.OFF
        apply_torque = 0

      if CS.motor_present:
        torque_fw_nm = self.direction * apply_torque * TORQUE_UNIT
        can_sends.append(create_steer_command(self.packer, self.command_counter, mode, torque_fw_nm))
        self.command_counter = (self.command_counter + 1) % 16
      self.apply_torque_last = apply_torque

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / self.params.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last

    self.frame += 1
    return new_actuators, can_sends
