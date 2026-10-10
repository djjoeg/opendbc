#!/usr/bin/env python3
"""
Single source of truth for the hondastepper port.

config.json describes:
  * fcan:     where each needed signal lives on the car's F-CAN (500 kbps, OBD pins 6/14)
  * actuator: the StepperServoCAN motor setup (direction, firmware gearing, real gearing)

From it this module generates two checked-in files that must always match:
  * opendbc/dbc/hondastepper_fcan.dbc              (used by carstate and the tests)
  * opendbc/safety/modes/honda_stepper_fcan.h      (compiled into the panda firmware)

The car setup tools on the comma device save the checked map to /data/hondastepper/config.json, outside the
code, so software updates keep it. When that file exists it is used instead of config.json, and every build on
the device regenerates the two files from it (the SConstruct calls write_generated()).

Run `python -m opendbc.car.hondastepper.config --write` after editing a config,
and `--check` to verify the generated files are current.
"""
import argparse
import json
import math
import os
import sys
import tempfile
from dataclasses import dataclass, field

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(HERE, "config.json")
# written by the setup tools on the comma device; HONDASTEPPER_CONFIG points somewhere else (used by the tests)
DEVICE_CONFIG_PATH = "/data/hondastepper/config.json"
OPENDBC_DIR = os.path.abspath(os.path.join(HERE, "..", ".."))
DBC_NAME = "hondastepper_fcan"
SERVO_DBC_NAME = "hondastepper_servo"
DBC_PATH = os.path.join(OPENDBC_DIR, "dbc", f"{DBC_NAME}.dbc")
SAFETY_HEADER_PATH = os.path.join(OPENDBC_DIR, "safety", "modes", "honda_stepper_fcan.h")

# Signals the port knows how to use. Required ones must exist for the safety mode to engage at all. Lane keeping is
# switched by the lift lever (the motor reporting or not), so the cruise MAIN button and the car's own cruise state
# aren't read at all.
REQUIRED_SIGNALS = ("VEHICLE_SPEED", "BRAKE_PRESSED")
OPTIONAL_SIGNALS = ("GAS_PEDAL", "CRUISE_BUTTONS", "GEAR", "DOOR_OPEN", "SEATBELT_UNLATCHED",
                    "LEFT_BLINKER", "RIGHT_BLINKER", "STEER_ANGLE",
                    "WHEEL_SPEED_FL", "WHEEL_SPEED_FR", "WHEEL_SPEED_RL", "WHEEL_SPEED_RR")
KNOWN_SIGNALS = REQUIRED_SIGNALS + OPTIONAL_SIGNALS
WHEEL_SPEED_SIGNALS = ("WHEEL_SPEED_FL", "WHEEL_SPEED_FR", "WHEEL_SPEED_RL", "WHEEL_SPEED_RR")
SPEED_UNITS = {"kph": 1. / 3.6, "mph": 0.44704, "m/s": 1.0}

# Honda F-CAN integrity fields live in the last byte: checksum = low nibble, counter = bits 4-5
HONDA_COUNTER_MAX = 3

# The panda's safety mode reads these signals. It checks every message they are in for presence and rate, and it
# refuses messages slower than 10 per second (opendbc/safety/safety.h), so those can't hold them.
SAFETY_SIGNALS = ("VEHICLE_SPEED", "BRAKE_PRESSED")
SAFETY_OPTIONAL_SIGNALS = ("GAS_PEDAL",)
SAFETY_MIN_FREQ = 10


def _be_bits() -> list[int]:
  # DBC big-endian (Motorola) bit order, as used by opendbc/can/dbc.py
  return [j + i * 8 for i in range(64) for j in range(7, -1, -1)]


BE_BITS = _be_bits()


@dataclass(frozen=True)
class MessageDef:
  name: str
  addr: int
  length: int
  freq: int
  honda_checksum: bool
  counter: bool

  @property
  def checksum_start_bit(self) -> int:
    return (self.length - 1) * 8 + 3

  @property
  def counter_start_bit(self) -> int:
    return (self.length - 1) * 8 + 5


@dataclass(frozen=True)
class SignalDef:
  name: str
  message: str
  start_bit: int
  size: int
  big_endian: bool
  signed: bool = False
  factor: float = 1.0
  offset: float = 0.0
  unit: str = ""
  values: dict[int, str] = field(default_factory=dict)

  def occupied_bits(self) -> set[int]:
    if not self.big_endian:
      return set(range(self.start_bit, self.start_bit + self.size))
    idx = BE_BITS.index(self.start_bit)
    return set(BE_BITS[idx:idx + self.size])

  @property
  def lsb_byte_index(self) -> int:
    return min(self.occupied_bits()) // 8


# The printed steering wheel teeth, by how far they reach each way from straight ahead (degrees of steering wheel
# turn): the steering wheel turn over which the motor gear turns with the wheel on each, end to end. That is the teeth
# (16 teeth, 29.2 degrees; 32 teeth, 58.5 degrees; the long back teeth, 42 teeth of a face gear, 91.1 degrees) plus
# the flat of the rolling band past each end (6.5 degrees each side); the roller follows the band a little way down its
# slope too, so a hand turn measures a few degrees more. The short ones are for getting used to lane keeping and for
# the highway: the motor pushes with its full strength only as far as the teeth reach (past them, only weakly through
# the roller's grip on the band). 15 also stands for the column mounts' ring teeth, which match the short teeth, and
# the short back teeth (14 teeth of the face gear: 30.4 degrees, 43.4 tracked, so their reach reads 0.6 degrees short).
TEETH_SETS = {15: 42.2, 29: 71.5, 45: 104.1}
# Steering wheel teeth (as a full circle) over the motor gear's: 197 / 19 on top of the wheel and in the ring teeth,
# 166 / 16 for the long back teeth (a 16-tooth motor gear on a 166-tooth face gear); 0.07% apart, so the short back
# teeth's 15 takes 197 / 19 as well.
MECHANICAL_RATIOS = {15: 197 / 19, 29: 197 / 19, 45: 166 / 16}
ROLL_PAST_TEETH_DEG = 6.5   # the flat of the rolling band past the last tooth, each side (the printed parts' design)


@dataclass(frozen=True)
class ActuatorConfig:
  direction: int
  firmware_gearing_ratio: float  # gearing the motor firmware was flashed with (StepperServoCAN default 5.18 * 2)
  firmware_k_torque: float       # Nm/A the firmware assumes for its motor
  mechanical_ratio: float        # steering wheel teeth (as a full circle: 197) / teeth on the motor gear (19); see MECHANICAL_RATIOS
  motor_k_torque: float          # Nm/A of the motor actually fitted
  # steering wheel turn over which the motor gear turns with the wheel, end to end, for the steering wheel teeth
  # fitted (see TEETH_SETS); the car setup saves the one it was told is on
  tracked_arc_deg: float = 71.5

  @property
  def torque_cmd_scale(self) -> float:
    """Firmware torque units per Nm wanted at the steering wheel."""
    return (self.firmware_gearing_ratio * self.firmware_k_torque) / (self.mechanical_ratio * self.motor_k_torque)

  @property
  def angle_scale(self) -> float:
    """Steering wheel degrees per firmware-reported degree."""
    return self.firmware_gearing_ratio / self.mechanical_ratio

  @property
  def full_push_reach_deg(self) -> float:
    """How far from straight ahead, each way, the motor can push with its full strength: the last tooth."""
    return self.tracked_arc_deg / 2.0 - ROLL_PAST_TEETH_DEG


SETUP_STEPS = ("parked", "drive")  # the car setup tool's two checks, see openpilot/sunnypilot/hondastepper


# The comma device on its own, the default: not connected to the car's network at all. Speed comes from the comma
# device's own estimate of its motion (camera and motion sensors), checked against GPS by the drive check, which saves
# the GPS's verdict as a scale; the brake, gas, turn signals, gear and doors aren't known. car_network: true uses the
# F-CAN map below instead, through the OBD port, as before.
DEVICE_SPEED_SCALE_RANGE = (0.85, 1.15)


@dataclass(frozen=True)
class Config:
  setup: dict[str, str]  # setup check -> when it passed; written by the setup tools
  messages: dict[str, MessageDef]
  signals: dict[str, SignalDef]
  actuator: ActuatorConfig
  fingerprint: dict[int, dict[int, int]]
  car_network: bool = False        # read the car's F-CAN through the OBD port; False: stand-alone
  device_speed_scale: float = 1.0  # stand-alone: GPS speed / the device's own speed, from the drive check

  @property
  def verified(self) -> bool:
    """Both setup checks passed on this car: the motor (and with car_network the F-CAN map) parked, and the speed
    and the motor's direction on a drive."""
    return all(self.setup.get(step) for step in SETUP_STEPS)

  def has(self, signal: str) -> bool:
    return signal in self.signals

  def safety_signal_names(self) -> list[str]:
    """The signals the panda's safety mode reads, as this config places them."""
    return list(SAFETY_SIGNALS) + [s for s in SAFETY_OPTIONAL_SIGNALS if s in self.signals]

  def message_of(self, signal: str) -> MessageDef:
    return self.messages[self.signals[signal].message]

  @property
  def has_wheel_speeds(self) -> bool:
    return all(s in self.signals for s in WHEEL_SPEED_SIGNALS)


class ConfigError(ValueError):
  pass


def _parse_int(value) -> int:
  if isinstance(value, int):
    return value
  return int(str(value), 0)


def parse_config(raw: dict) -> Config:
  if raw.get("schema_version") != 1:
    raise ConfigError("unsupported schema_version")

  fcan = raw["fcan"]
  messages: dict[str, MessageDef] = {}
  for m in fcan["messages"]:
    checksum = m.get("checksum", "none")
    if checksum not in ("honda", "none"):
      raise ConfigError(f"{m['name']}: checksum must be 'honda' or 'none'")
    msg = MessageDef(name=m["name"], addr=_parse_int(m["addr"]), length=int(m["len"]), freq=int(m["freq"]),
                     honda_checksum=checksum == "honda", counter=bool(m.get("counter", False)))
    if msg.name in messages:
      raise ConfigError(f"duplicate message name {msg.name}")
    if not 0 < msg.addr < 0x800:
      raise ConfigError(f"{msg.name}: only 11-bit addresses are supported")
    if not 1 <= msg.length <= 8:
      raise ConfigError(f"{msg.name}: length must be 1-8")
    if not 1 <= msg.freq <= 1000:
      raise ConfigError(f"{msg.name}: freq must be 1-1000 Hz")
    if msg.counter and not msg.honda_checksum:
      raise ConfigError(f"{msg.name}: a counter is only supported together with the Honda checksum")
    messages[msg.name] = msg
  if len({m.addr for m in messages.values()}) != len(messages):
    raise ConfigError("two messages share an address")

  signals: dict[str, SignalDef] = {}
  for name, s in fcan["signals"].items():
    if name not in KNOWN_SIGNALS:
      raise ConfigError(f"unknown signal {name}")
    if s["message"] not in messages:
      raise ConfigError(f"{name}: unknown message {s['message']}")
    byte_order = s.get("byte_order", "big")
    if byte_order not in ("big", "little"):
      raise ConfigError(f"{name}: byte_order must be 'big' or 'little'")
    sig = SignalDef(name=name, message=s["message"], start_bit=int(s["start_bit"]), size=int(s["size"]),
                    big_endian=byte_order == "big", signed=bool(s.get("signed", False)),
                    factor=float(s.get("factor", 1.0)), offset=float(s.get("offset", 0.0)), unit=str(s.get("unit", "")),
                    values={_parse_int(k): str(v) for k, v in s.get("values", {}).items()})
    msg = messages[sig.message]
    if not 1 <= sig.size <= 16:
      raise ConfigError(f"{name}: size must be 1-16 bits")
    try:
      bits = sig.occupied_bits()
    except ValueError as e:
      raise ConfigError(f"{name}: invalid start bit") from e
    if len(bits) != sig.size or max(bits) >= msg.length * 8:
      raise ConfigError(f"{name}: does not fit in {msg.name} ({msg.length} bytes)")
    reserved = set()
    if msg.honda_checksum:
      reserved |= set(range((msg.length - 1) * 8, (msg.length - 1) * 8 + 4))
    if msg.counter:
      reserved |= {(msg.length - 1) * 8 + 4, (msg.length - 1) * 8 + 5}
    if bits & reserved:
      raise ConfigError(f"{name}: overlaps the checksum/counter bits of {msg.name}")
    signals[name] = sig

  for name in REQUIRED_SIGNALS:
    if name not in signals:
      raise ConfigError(f"required signal {name} missing")
  if signals["VEHICLE_SPEED"].unit not in SPEED_UNITS:
    raise ConfigError(f"VEHICLE_SPEED unit must be one of {sorted(SPEED_UNITS)}")
  for name in WHEEL_SPEED_SIGNALS:
    if name in signals and signals[name].unit not in SPEED_UNITS:
      raise ConfigError(f"{name} unit must be one of {sorted(SPEED_UNITS)}")
  if any(s in signals for s in WHEEL_SPEED_SIGNALS) and not all(s in signals for s in WHEEL_SPEED_SIGNALS):
    raise ConfigError("give all four wheel speeds or none")
  for name in ("BRAKE_PRESSED", "LEFT_BLINKER", "RIGHT_BLINKER", "DOOR_OPEN", "SEATBELT_UNLATCHED"):
    if name in signals and (signals[name].size != 1 or signals[name].signed):
      raise ConfigError(f"{name} must be an unsigned 1-bit flag")
  # signals in the same message must not overlap
  for a in signals.values():
    for b in signals.values():
      if a.name < b.name and a.message == b.message and a.occupied_bits() & b.occupied_bits():
        raise ConfigError(f"{a.name} and {b.name} overlap")

  if "mads_button" in fcan:
    raise ConfigError("mads_button isn't supported: the lift lever turns lane keeping on and off")

  safety_names = list(SAFETY_SIGNALS) + [n for n in SAFETY_OPTIONAL_SIGNALS if n in signals]
  for name in safety_names:
    msg = messages[signals[name].message]
    if msg.freq < SAFETY_MIN_FREQ:
      raise ConfigError(f"{name} is in {msg.name}, which comes {msg.freq} times a second; the panda needs {SAFETY_MIN_FREQ}")

  a = raw["actuator"]
  actuator = ActuatorConfig(direction=int(a["direction"]), firmware_gearing_ratio=float(a["firmware_gearing_ratio"]),
                            firmware_k_torque=float(a["firmware_k_torque"]), mechanical_ratio=float(a["mechanical_ratio"]),
                            motor_k_torque=float(a["motor_k_torque"]),
                            tracked_arc_deg=float(a.get("tracked_arc_deg", ActuatorConfig.tracked_arc_deg)))
  if actuator.direction not in (-1, 1):
    raise ConfigError("actuator.direction must be 1 or -1")
  for key in ("firmware_gearing_ratio", "firmware_k_torque", "mechanical_ratio", "motor_k_torque"):
    v = getattr(actuator, key)
    if not (math.isfinite(v) and v > 0):
      raise ConfigError(f"actuator.{key} must be positive")
  if not 10.0 <= actuator.tracked_arc_deg <= 360.0:
    raise ConfigError("actuator.tracked_arc_deg must be 10-360")
  if not 0.5 < actuator.torque_cmd_scale < 2.0:
    raise ConfigError("actuator scales differ by more than 2x from the firmware; reflash the motor firmware instead")

  fingerprint = {int(bus): {_parse_int(addr): int(ln) for addr, ln in msgs.items()}
                 for bus, msgs in raw.get("fingerprint", {}).items()}

  setup = raw.get("setup", {})
  if not isinstance(setup, dict) or not set(setup) <= set(SETUP_STEPS) or not all(isinstance(v, str) for v in setup.values()):
    raise ConfigError(f"setup must map {SETUP_STEPS} to text")

  car_network = raw.get("car_network", False)
  if not isinstance(car_network, bool):
    raise ConfigError("car_network must be true or false")
  scale = raw.get("device_speed_scale", 1.0)
  if isinstance(scale, bool) or not isinstance(scale, int | float) or not \
     DEVICE_SPEED_SCALE_RANGE[0] <= float(scale) <= DEVICE_SPEED_SCALE_RANGE[1]:
    raise ConfigError(f"device_speed_scale must be {DEVICE_SPEED_SCALE_RANGE[0]} to {DEVICE_SPEED_SCALE_RANGE[1]}")

  return Config(setup=dict(setup), messages=messages, signals=signals, actuator=actuator, fingerprint=fingerprint,
                car_network=car_network, device_speed_scale=float(scale))


def device_config_path() -> str:
  return os.environ.get("HONDASTEPPER_CONFIG", DEVICE_CONFIG_PATH)


def active_config_path() -> str:
  """The device config when the setup tools have saved one, otherwise the default in the code."""
  path = device_config_path()
  return path if path and os.path.isfile(path) else CONFIG_PATH


def load_raw_config(path: str | None = None) -> dict:
  with open(path or active_config_path()) as f:
    return json.load(f)


def load_config(path: str | None = None) -> Config:
  return parse_config(load_raw_config(path))


def load_config_or_default() -> tuple[Config, str]:
  """The active config, or the default in the code when the saved one can't be read or doesn't pass the checks.
  The default was never checked on the car, so lane keeping stays off with it. Returns (config, problem); problem
  is empty when the active config loaded."""
  path = active_config_path()
  try:
    return load_config(path), ""
  except (OSError, ValueError, KeyError, TypeError) as e:  # ConfigError and json errors are ValueErrors
    if path == CONFIG_PATH:
      raise
    return load_config(CONFIG_PATH), f"{path} can't be used ({e}); using the default in the code"


def save_device_config(raw: dict, path: str | None = None) -> str:
  """Validates raw and writes it atomically as the device config. Returns the path written."""
  parse_config(raw)
  path = path or device_config_path()
  os.makedirs(os.path.dirname(path), exist_ok=True)
  fd, tmp = tempfile.mkstemp(dir=os.path.dirname(path), prefix=".config.", suffix=".json")
  with os.fdopen(fd, "w") as f:
    json.dump(raw, f, indent=2)
    f.write("\n")
    f.flush()
    os.fsync(f.fileno())
  os.replace(tmp, path)
  return path


# ********** code generation **********

GENERATED_NOTE = "AUTO-GENERATED by opendbc/car/hondastepper/config.py from config.json or /data/hondastepper/config.json. Do not edit."

def _fmt_num(v: float) -> str:
  return f"{v:g}"


def _signal_range(sig: SignalDef) -> tuple[float, float]:
  if sig.signed:
    lo, hi = -(1 << (sig.size - 1)), (1 << (sig.size - 1)) - 1
  else:
    lo, hi = 0, (1 << sig.size) - 1
  a, b = lo * sig.factor + sig.offset, hi * sig.factor + sig.offset
  return min(a, b), max(a, b)


def generate_dbc(cfg: Config) -> str:
  lines = [
    'VERSION ""',
    "",
    "",
    "NS_ :",
    "    NS_DESC_",
    "    CM_",
    "    BA_DEF_",
    "    BA_",
    "    VAL_",
    "",
    "BS_:",
    "",
    "BU_: XXX",
    "",
    f'CM_ "{GENERATED_NOTE}";',
    "",
  ]
  for msg in sorted(cfg.messages.values(), key=lambda m: m.addr):
    lines.append(f"BO_ {msg.addr} {msg.name}: {msg.length} XXX")
    for sig in sorted((s for s in cfg.signals.values() if s.message == msg.name), key=lambda s: s.start_bit):
      lo, hi = _signal_range(sig)
      order = "0" if sig.big_endian else "1"
      sign = "-" if sig.signed else "+"
      scale = f"({_fmt_num(sig.factor)},{_fmt_num(sig.offset)})"
      limits = f"[{_fmt_num(lo)}|{_fmt_num(hi)}]"
      lines.append(f' SG_ {sig.name} : {sig.start_bit}|{sig.size}@{order}{sign} {scale} {limits} "{sig.unit}" XXX')
    if msg.counter:
      lines.append(f' SG_ COUNTER : {msg.counter_start_bit}|2@0+ (1,0) [0|3] "" XXX')
    if msg.honda_checksum:
      lines.append(f' SG_ CHECKSUM : {msg.checksum_start_bit}|4@0+ (1,0) [0|15] "" XXX')
    lines.append("")
  for sig in sorted(cfg.signals.values(), key=lambda s: (cfg.messages[s.message].addr, s.name)):
    if sig.values:
      vals = " ".join(f'{k} "{v}"' for k, v in sorted(sig.values.items()))
      lines.append(f"VAL_ {cfg.messages[sig.message].addr} {sig.name} {vals} ;")
  lines.append("")
  return "\n".join(lines)


def _c_signal_block(prefix: str, sig: SignalDef, msg: MessageDef) -> list[str]:
  return [
    f"#define {prefix}_ADDR 0x{msg.addr:X}U",
    f"#define {prefix}_START_BIT {sig.start_bit}U",
    f"#define {prefix}_SIZE {sig.size}U",
    f"#define {prefix}_BIG_ENDIAN {'true' if sig.big_endian else 'false'}",
  ]


def generate_safety_header(cfg: Config) -> str:
  out = [
    "#pragma once",
    "",
    f"// {GENERATED_NOTE}",
    "",
  ]
  safety_signals = [("HONDA_STEPPER_SPEED", "VEHICLE_SPEED"), ("HONDA_STEPPER_BRAKE", "BRAKE_PRESSED")]
  optional = [("HONDA_STEPPER_GAS", "GAS_PEDAL")]
  for prefix, name in safety_signals:
    out += [f"// {name}"] + _c_signal_block(prefix, cfg.signals[name], cfg.message_of(name)) + [""]
  for prefix, name in optional:
    present = name in cfg.signals
    out += [f"// {name}", f"#define {prefix}_PRESENT {'1' if present else '0'}"]
    if present:
      out += _c_signal_block(prefix, cfg.signals[name], cfg.message_of(name))
    out.append("")

  # one rx check per F-CAN message the safety mode reads
  used_msgs = sorted({cfg.signals[s].message for s in cfg.safety_signal_names()}, key=lambda n: cfg.messages[n].addr)
  out.append("// F-CAN messages checked for presence, rate, checksum and counter")
  out.append(f"#define HONDA_STEPPER_FCAN_RX_CHECK_COUNT {len(used_msgs)}")
  entries = []
  for name in used_msgs:
    m = cfg.messages[name]
    opts = []
    if m.counter:
      opts.append(f".max_counter = {HONDA_COUNTER_MAX}U")
    else:
      opts.append(".ignore_counter = true")
    if not m.honda_checksum:
      opts.append(".ignore_checksum = true")
    opts.append(".ignore_quality_flag = true")
    entries.append(f"  /* {m.name} */ {{.msg = {{{{0x{m.addr:X}U, HONDA_STEPPER_FCAN_BUS, {m.length}, {m.freq}U, {', '.join(opts)}}}, {{ 0 }}, {{ 0 }}}}}},")
  out.append("#define HONDA_STEPPER_FCAN_RX_CHECKS \\")
  out.append(" \\\n".join(entries))
  out.append("")
  return "\n".join(out)


def generated_files(cfg: Config) -> dict[str, str]:
  return {DBC_PATH: generate_dbc(cfg), SAFETY_HEADER_PATH: generate_safety_header(cfg)}


def stale_files(cfg: Config) -> list[str]:
  stale = []
  for path, content in generated_files(cfg).items():
    try:
      with open(path) as f:
        if f.read() != content:
          stale.append(path)
    except FileNotFoundError:
      stale.append(path)
  return stale


def write_generated(cfg: Config | None = None) -> list[str]:
  """Rewrites the generated files that differ from cfg (default: the active config, or the default in the code when
  that can't be used). Returns the paths written."""
  if cfg is None:
    cfg, _ = load_config_or_default()
  written = []
  for path in stale_files(cfg):
    with open(path, "w") as f:
      f.write(generated_files(cfg)[path])
    written.append(path)
  return written


def main(argv=None) -> int:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("--config", default=None, help="config file (default: the device config if saved, else config.json)")
  group = parser.add_mutually_exclusive_group(required=True)
  group.add_argument("--write", action="store_true", help="regenerate the DBC and safety header")
  group.add_argument("--check", action="store_true", help="exit 1 if the generated files are stale")
  args = parser.parse_args(argv)

  path = args.config or active_config_path()
  cfg = load_config(path)
  print(f"config: {path}")
  if args.write:
    written = write_generated(cfg)
    for p in written:
      print(f"wrote {os.path.relpath(p, OPENDBC_DIR)}")
    if not written:
      print("generated files already current")
    return 0
  stale = stale_files(cfg)
  for path in stale:
    print(f"stale: {os.path.relpath(path, OPENDBC_DIR)}")
  return 1 if stale else 0


if __name__ == "__main__":
  sys.exit(main())
