"""
Simulated car and motor for the hondastepper tests and the setup tool's tests: F-CAN frames like a 2008-12 Accord
would send them under a given signal layout, and a StepperServoCAN motor turning a steering wheel through the printed
steering wheel teeth.
"""
import math
import random

from opendbc.car.hondastepper import discovery as dsc
from opendbc.car.hondastepper.checksums import stepper_checksum
from opendbc.car.hondastepper.config import TEETH_SETS

# where a simulated car keeps each signal: name -> (addr, start_bit, size); big endian like Honda
DEFAULT_LAYOUT = {
  "VEHICLE_SPEED": (0x158, 7, 16),
  "BRAKE_PRESSED": (0x17C, 53, 1),
  "GAS_PEDAL": (0x17C, 7, 8),
  "GEAR": (0x1A3, 35, 4),
  "DOOR_OPEN": (0x1A6, 63, 1),
  "LEFT_BLINKER": (0x294, 5, 1),
  "RIGHT_BLINKER": (0x294, 6, 1),
}
DEFAULT_GEARS = {"P": 1, "R": 2, "N": 3, "D": 4}

# a car that keeps things somewhere else, with another speed scale and gear coding
MOVED_LAYOUT = {
  "VEHICLE_SPEED": (0x309, 23, 16),   # bytes 2-3, 0.005 kph per step
  "BRAKE_PRESSED": (0x17C, 32, 1),
  "GAS_PEDAL": (0x17C, 15, 8),
  "GEAR": (0x191, 3, 3),
  "DOOR_OPEN": (0x405, 1, 1),
  "LEFT_BLINKER": (0x294, 13, 1),
  "RIGHT_BLINKER": (0x294, 14, 1),
}
MOVED_GEARS = {"P": 0, "R": 7, "N": 6, "D": 1}

# (length, Hz)
MESSAGES = {0x158: (8, 100), 0x17C: (8, 100), 0x1A3: (8, 100), 0x1A6: (8, 25), 0x294: (8, 10),
            0x309: (8, 50), 0x191: (8, 100), 0x326: (8, 10), 0x405: (8, 10), 0x13F: (8, 100), 0x255: (5, 20),
            0x3D9: (8, 5)}


class SimCar:
  """Makes F-CAN frames like a car: every message at its rate with Honda checksum and counter, signals at their
  layout positions, and some busy fields (engine speed, a rolling byte, a 1 Hz heartbeat bit) that change by themselves."""

  def __init__(self, layout=DEFAULT_LAYOUT, gears=DEFAULT_GEARS, speed_factor_kph=0.01, flashing_blinkers=True, messages=None,
               copies=None):
    self.messages = messages or MESSAGES
    self.layout = layout
    self.copies = copies or {}   # name -> more places the car also sends that signal, like layout entries
    self.gears = gears
    self.speed_factor_kph = speed_factor_kph
    self.flashing_blinkers = flashing_blinkers
    self.state = {"VEHICLE_SPEED": 0.0, "BRAKE_PRESSED": 0, "GAS_PEDAL": 0, "GEAR": "P", "DOOR_OPEN": 0,
                  "LEFT_BLINKER": 0, "RIGHT_BLINKER": 0}
    self.counters = dict.fromkeys(self.messages, 0)
    self.next_t = dict.fromkeys(self.messages, 0.0)
    self.t = 0.0
    self.rng = random.Random(1)
    self.history: list[dsc.Frame] = []

  def _value(self, name: str) -> int:
    v = self.state[name]
    if name == "VEHICLE_SPEED":
      return round(v / self.speed_factor_kph)
    if name == "GEAR":
      return self.gears[v]
    if name in ("LEFT_BLINKER", "RIGHT_BLINKER") and v and self.flashing_blinkers:
      return 1 if (self.t % 0.7) < 0.35 else 0
    return int(v)

  def _frame(self, addr: int) -> bytes:
    length = self.messages[addr][0]
    value = 0
    places = [(name, place) for name, place in self.layout.items()]
    places += [(name, place) for name, more in self.copies.items() for place in more]
    for name, (a, start_bit, size) in places:
      if a == addr:
        shift = length * 8 - dsc.BE_BITS.index(start_bit) - size
        value |= (self._value(name) & ((1 << size) - 1)) << shift
    data = bytearray(value.to_bytes(length, "big"))
    if addr == 0x17C:
      rpm = 750 + int(30 * math.sin(self.t * 3)) + 4 * self.state["GAS_PEDAL"]
      data[2], data[3] = rpm >> 8, rpm & 0xFF          # ENGINE_RPM 23|16
    if addr == 0x13F:
      data[0] = int(self.t * 100) & 0xFF                # rolling byte
      data[1] = int(self.t) & 1                         # 1 Hz heartbeat bit
      data[4] = self.rng.randrange(256)                 # noise
    out = dsc.with_honda_checksum(addr, bytes(data), self.counters[addr])
    self.counters[addr] = (self.counters[addr] + 1) % 4
    return out

  def run(self, seconds: float, **state) -> list[dsc.Frame]:
    self.state.update(state)
    return self.advance(self.t + seconds)

  def advance(self, t: float) -> list[dsc.Frame]:
    """Frames sent from now until time t."""
    frames = []
    while self.t < t:
      for addr, (_, freq) in self.messages.items():
        if self.t >= self.next_t[addr]:
          frames.append((self.t, addr, self._frame(addr)))
          self.next_t[addr] += 1.0 / freq
      self.t = round(self.t + 0.002, 6)
    self.history += frames
    return frames


def turning_drive(seconds: float, true_direction: int = 1, angle_scale: float = 1.0, turning: bool = True,
                  speed: float = 20.0, t0: float = 0.0) -> tuple[list[tuple[float, float]], list[tuple[float, float, float]]]:
  """A drive at a steady speed with the steering wheel weaving through gentle curves: the motor's angle readings
  (time, firmware degrees; its zero isn't straight ahead) at 100 Hz, and the GPS course (time, degrees clockwise from
  north, m/s) at 10 Hz. true_direction is the motor's real direction, as in SimMotor."""
  motor, headings = [], []
  bearing = 90.0
  for i in range(int(seconds * 100)):
    t = t0 + i * 0.01
    steer = 8.0 * math.sin(2 * math.pi * i * 0.01 / 30.0) if turning else 0.0   # steering wheel degrees, left positive
    bearing = (bearing - 0.6 * steer * 0.01) % 360.0                              # left turns lower the course
    motor.append((t, true_direction * (steer + 3.0) / angle_scale))
    if i % 10 == 0:
      headings.append((t, bearing, speed))
  return motor, headings


class _ChecksumSig:
  start_bit = 0


# the printed parts and the motor firmware's setting (see config.json)
FIRMWARE_RATIO = 10.363636     # gearing the motor firmware assumes
PRINTED_RATIO = 197 / 19       # steering wheel teeth (counted as a full circle) / motor gear teeth
TRACKED_ARC_DEG = TEETH_SETS[29]   # steering wheel turn the motor gear follows on the long teeth: teeth plus rolling band
FREE_SPIN_FW_DEG_S = 360.0     # motor speed off the teeth while pushing, firmware degrees per second (an estimate)


class SimMotor:
  """StepperServoCAN on the steering wheel teeth. Status (0x22F) at 100 Hz while powered (lift lever down). Commands
  (0x22E) in torque mode turn the wheel while the motor gear is on the teeth; off them the motor spins freely in the
  direction it pushes. true_direction is +1 when positive firmware torque turns the wheel left. Starts powered with
  its controller running; lever() lifts and lowers it."""

  def __init__(self, true_direction: int = 1, angle_scale: float = FIRMWARE_RATIO / PRINTED_RATIO, arc_deg: float = TRACKED_ARC_DEG):
    self.true_direction = true_direction
    self.angle_scale = angle_scale       # steering wheel degrees per firmware degree
    self.arc_deg = arc_deg
    self.powered = True
    self.wheel_deg = 0.0                 # steering wheel, left positive
    self.teeth_center_deg = 0.0          # wheel angle with the motor gear in the middle of the steering wheel teeth
    self.encoder_fw_deg = 0.0            # what the firmware reports
    self.speed_fw_deg_s = 0.0            # what the firmware reports as its speed, firmware degrees per second
    self.free_spin_deg = 0.0             # how far the motor has spun off the teeth, firmware degrees
    self.mode = 0
    self.torque_fw = 0.0
    self.soft_off = False
    self.last_cmd_t = -1.0
    self.counter = 0
    self.next_t = 0.0
    self.t = 0.0
    self._status_encoder = 0.0
    self._status_t = 0.0
    self.boot_s = 0.05      # after power-up it reports this long before its controller runs (the firmware's start-up)
    self.running_t = 0.0    # when its controller runs

  def lever(self, down: bool):
    """The lift lever, now. Down puts the motor gear on the teeth and powers the motor: it reports 10 ms later and
    its controller runs boot_s after power-up, its angle counted from there. Up lifts the gear off and cuts the power."""
    if down and not self.powered:
      self.mode, self.torque_fw, self.soft_off, self.last_cmd_t = 0, 0.0, False, -1.0
      self.encoder_fw_deg = 0.0
      self.counter = 0
      self.next_t = self.t + 0.01
      self.running_t = self.t + self.boot_s
      self._status_encoder, self._status_t = 0.0, self.t
    if not down:
      self.mode, self.torque_fw, self.soft_off = 0, 0.0, False
    self.powered = down

  def running(self) -> bool:
    return self.powered and self.t >= self.running_t

  def in_teeth(self, wheel_deg: float) -> bool:
    return abs(wheel_deg - self.teeth_center_deg) <= self.arc_deg / 2

  def turn_wheel(self, new_wheel_deg: float, steps: int = 50):
    """The driver turns the wheel; the encoder follows only while the motor gear is on the teeth (lift lever down)."""
    start = self.wheel_deg
    for i in range(1, steps + 1):
      w = start + (new_wheel_deg - start) * i / steps
      if self.powered and self.in_teeth(w) and self.in_teeth(self.wheel_deg):
        self.encoder_fw_deg += self.true_direction * (w - self.wheel_deg) / self.angle_scale
      self.wheel_deg = w

  def command(self, t: float, data: bytes):
    if not self.running():
      return
    mode = (data[1] >> 4) & 0x3
    torque = int.from_bytes(data[4:5], "little", signed=True) * 0.125
    self.last_cmd_t = t
    if mode == 0:
      self.soft_off = False
    if not self.soft_off:
      self.mode, self.torque_fw = mode, torque

  def advance(self, t: float) -> list[tuple[float, int, bytes]]:
    frames = []
    while self.t < t:
      dt = 0.002
      if self.powered and self.last_cmd_t >= 0 and self.t - self.last_cmd_t > 0.06 and self.mode == 1:
        self.soft_off, self.mode, self.torque_fw = True, 3, 0.0
      pushing = self.powered and self.mode == 1 and self.torque_fw != 0
      if pushing and self.in_teeth(self.wheel_deg):
        # hydraulic steering at standstill: about 8 degrees per second per Nm at the wheel
        self.turn_wheel(self.wheel_deg + self.true_direction * self.torque_fw * 8.0 * dt, steps=1)
      elif pushing:
        # off the teeth nothing holds the motor gear back
        self.encoder_fw_deg += math.copysign(FREE_SPIN_FW_DEG_S * dt, self.torque_fw)
        self.free_spin_deg += FREE_SPIN_FW_DEG_S * dt
      if self.powered and self.t >= self.next_t:
        frames.append((self.t, 0x22F, self._status()))
        self.next_t += 0.01
      self.t = round(self.t + dt, 6)
    return frames

  def _status(self) -> bytes:
    if self.t > self._status_t:
      self.speed_fw_deg_s = (self.encoder_fw_deg - self._status_encoder) / (self.t - self._status_t)
    self._status_encoder, self._status_t = self.encoder_fw_deg, self.t
    control_status = (1 | 2 if self.running() else 0) | (4 if self.soft_off else 0)
    torque = round(self.torque_fw / 0.125) if self.mode == 1 else 0
    speed = max(-128, min(127, round(self.speed_fw_deg_s / 360.0 / 0.015625)))
    angle = round(self.encoder_fw_deg / 0.125)
    data = bytearray(8)
    data[1] = (self.counter & 0xF) | (control_status << 4)
    data[2] = torque & 0xFF
    data[3] = speed & 0xFF
    data[4] = 30 + 60
    data[5:7] = (angle & 0xFFFF).to_bytes(2, "little")
    self.counter = (self.counter + 1) % 16
    data[0] = stepper_checksum(0x22F, _ChecksumSig, data)
    return bytes(data)
