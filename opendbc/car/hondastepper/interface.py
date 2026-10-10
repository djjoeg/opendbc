from opendbc.car import get_safety_config, structs
from opendbc.car.carlog import carlog
from opendbc.car.hondastepper.carcontroller import CarController
from opendbc.car.hondastepper.carstate import CarState
from opendbc.car.hondastepper.config import SETUP_STEPS, active_config_path, stale_files
from opendbc.car.hondastepper.values import CONFIG, CONFIG_PROBLEM, HONDA_STEPPER_PARAM_STANDALONE, HondaStepperFlags
from opendbc.car.interfaces import CarInterfaceBase

# Without a steering angle from the car, lateral control works from the gyro, which is only good above walking pace
YAW_RATE_MIN_STEER_SPEED = 2.0  # m/s


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "hondastepper"
    if CONFIG.car_network:
      ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.hondaStepper)]
    else:
      # stand-alone: the panda checks only the motor; openpilot's card hands the comma device's own speed to the car port
      ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.hondaStepper, HONDA_STEPPER_PARAM_STANDALONE)]
      ret.flags |= HondaStepperFlags.STANDALONE.value

    # no radar, no openpilot gas or brake: speed stays with the driver or the stock cruise control
    ret.radarUnavailable = True
    ret.openpilotLongitudinalControl = False
    ret.pcmCruise = True
    ret.autoResumeSng = False

    ret.steerControlType = structs.CarParams.SteerControlType.torque
    # stepper through a hydraulic rack, value from dzid26's BMW stepper port; sunnypilot's live delay learner refines it
    ret.steerActuatorDelay = 0.4
    ret.steerLimitTimer = 1.0
    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    if CONFIG.car_network and CONFIG.has("STEER_ANGLE"):
      ret.flags |= HondaStepperFlags.FCAN_STEER_ANGLE.value
    else:
      # The motor encoder only knows the angle relative to where the motor powered up, and loses count when the
      # motor gear runs off the end of the steering wheel teeth, so the steering controller measures the car's
      # turning from the comma device's gyro instead. Below walking pace the gyro says little, so no steering there.
      ret.lateralTuning.torque.useYawRate = True
      ret.minSteerSpeed = YAW_RATE_MIN_STEER_SPEED

    if not docs:
      if CONFIG_PROBLEM:
        carlog.error(f"hondastepper: {CONFIG_PROBLEM}. Lane keeping is off; run the car setup again.")
      # Lane keeping stays off (openpilot only records) until the car setup tool has checked the motor's direction
      # (and with car_network this car's map), and the drive check has checked the speed against GPS and, against
      # the GPS course, the motor's direction again. The drive check reads the logs openpilot records meanwhile.
      if not CONFIG.verified:
        missing = [step for step in SETUP_STEPS if not CONFIG.setup.get(step)]
        carlog.error(f"hondastepper: setup checks not done yet: {missing}. Lane keeping is off until they pass.")
        ret.dashcamOnly = True
      # The panda firmware and the DBC are generated from the config. If they don't match the config in use
      # (the setup tool saved a new one and the device hasn't rebuilt yet), lane keeping stays off.
      stale = stale_files(CONFIG)
      if stale:
        carlog.error(f"hondastepper: generated files don't match {active_config_path()}: {stale}. Reboot the device to rebuild.")
        ret.dashcamOnly = True

    return ret
