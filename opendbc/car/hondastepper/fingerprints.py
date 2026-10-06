from opendbc.car.hondastepper.values import CAR, CONFIG

# The car is picked by hand in sunnypilot (Settings > Vehicle). A config that lists the car's F-CAN messages under
# "fingerprint" also lets openpilot recognize it automatically; the car setup tool doesn't write that list.
FINGERPRINTS = {CAR.HONDA_ACCORD_8G_STEPPER: [CONFIG.fingerprint[0]]} if CONFIG.fingerprint.get(0) else {}
