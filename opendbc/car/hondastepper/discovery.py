"""
Analysis behind the car setup tool (openpilot/sunnypilot/hondastepper/setup_car.py).

The setup tool records the car's F-CAN while the driver does simple things (press the brake, switch the
blinker on, shift gears), and records the motor's status while the driver turns the steering wheel. The
functions here check each signal location in the config against those recordings, and search for the right
location when a check fails. They work on plain recorded frames, so they are tested without a car.

A frame is (seconds, address, data bytes). F-CAN frames come from panda bus 0.
"""
import copy
import math
from collections import defaultdict
from collections.abc import Iterable, Sequence
from dataclasses import dataclass, field
from typing import NamedTuple

from opendbc.car.hondastepper.config import (BE_BITS, DEVICE_SPEED_SCALE_RANGE, REQUIRED_SIGNALS, SAFETY_MIN_FREQ, SAFETY_OPTIONAL_SIGNALS, SAFETY_SIGNALS,
                                             SPEED_UNITS, Config, SignalDef)

Frame = tuple[float, int, bytes]
Window = Sequence[Frame]

STANDARD_FREQS = (1, 2, 5, 10, 20, 25, 33, 50, 100)
INTEGRITY_MIN_FRACTION = 0.98  # share of frames that must pass the Honda checksum and counter checks

# what the setup reports call each signal
PLAIN_NAMES = {
  "VEHICLE_SPEED": "speed", "BRAKE_PRESSED": "brake pedal", "GAS_PEDAL": "gas pedal",
  "CRUISE_BUTTONS": "cruise buttons", "GEAR": "shifter", "DOOR_OPEN": "driver's door",
  "SEATBELT_UNLATCHED": "seatbelt", "LEFT_BLINKER": "left turn signal", "RIGHT_BLINKER": "right turn signal",
  "STEER_ANGLE": "steering angle", "WHEEL_SPEED_FL": "front left wheel speed", "WHEEL_SPEED_FR": "front right wheel speed",
  "WHEEL_SPEED_RL": "rear left wheel speed", "WHEEL_SPEED_RR": "rear right wheel speed",
}


def plain_name(signal: str) -> str:
  return PLAIN_NAMES.get(signal, signal)


# ********** low level **********

def honda_checksum(addr: int, data: bytes) -> int:
  """Honda F-CAN checksum: low nibble of the last byte. Same as honda_stepper_compute_checksum in the panda."""
  s = 0
  a = addr
  while a > 0:
    s += a & 0xF
    a >>= 4
  for b in data:
    s += (b & 0xF) + (b >> 4)
  s -= data[-1] & 0xF
  return (8 - s) & 0xF


def with_honda_checksum(addr: int, data: bytes, counter: int | None = None) -> bytes:
  """Returns data with the counter (bits 4-5 of the last byte, when given) and the checksum filled in."""
  out = bytearray(data)
  if counter is not None:
    out[-1] = (out[-1] & 0xCF) | ((counter & 0x3) << 4)
  out[-1] &= 0xF0
  out[-1] |= honda_checksum(addr, bytes(out))
  return bytes(out)


def honda_counter(data: bytes) -> int:
  return (data[-1] >> 4) & 0x3


def read_raw(data: bytes, start_bit: int, size: int, big_endian: bool) -> int:
  """Unsigned value of a signal in DBC bit numbering (same walk as honda_stepper_get_signal in the panda)."""
  if big_endian:
    # with the bytes read as one big-endian number, a Motorola field is a run of adjacent bits
    shift = len(data) * 8 - BE_BITS.index(start_bit) - size
    return (int.from_bytes(data, "big") >> shift) & ((1 << size) - 1)
  return (int.from_bytes(data, "little") >> start_bit) & ((1 << size) - 1)


def read_signal(data: bytes, sig: SignalDef) -> float:
  raw = read_raw(data, sig.start_bit, sig.size, sig.big_endian)
  if sig.signed and raw >= 1 << (sig.size - 1):
    raw -= 1 << sig.size
  return raw * sig.factor + sig.offset


def by_addr(frames: Iterable[Frame]) -> dict[int, list[Frame]]:
  out: dict[int, list[Frame]] = defaultdict(list)
  for f in frames:
    out[f[1]].append(f)
  return out


def between(frames: Sequence[Frame], t0: float, t1: float) -> list[Frame]:
  return [f for f in frames if t0 <= f[0] <= t1]


def _parse_addr(v) -> int:
  return v if isinstance(v, int) else int(str(v), 0)


# ********** message inventory **********

@dataclass
class MessageStats:
  addr: int
  length: int
  count: int
  freq: float
  honda_integrity: bool  # Honda checksum and counter valid on (almost) every frame


def honda_integrity(frames: Sequence[Frame]) -> bool:
  if len(frames) < 4:
    return False
  addr = frames[0][1]
  good_checksum = sum(1 for _, _, d in frames if d[-1] & 0xF == honda_checksum(addr, d))
  good_counter = sum(1 for (_, _, a), (_, _, b) in zip(frames, frames[1:], strict=False)
                     if honda_counter(b) == (honda_counter(a) + 1) % 4)
  return good_checksum >= INTEGRITY_MIN_FRACTION * len(frames) and good_counter >= INTEGRITY_MIN_FRACTION * (len(frames) - 1)


def message_stats(frames: Sequence[Frame], duration: float) -> dict[int, MessageStats]:
  stats = {}
  for addr, fs in by_addr(frames).items():
    lengths = {len(d) for _, _, d in fs}
    if 0 in lengths:
      continue
    stats[addr] = MessageStats(addr=addr, length=max(lengths), count=len(fs), freq=len(fs) / max(duration, 1e-3),
                               honda_integrity=len(lengths) == 1 and honda_integrity(fs))
  return stats


def nearest_standard_freq(freq: float) -> int:
  return min(STANDARD_FREQS, key=lambda f: abs(math.log(max(freq, 0.1) / f)))


def fingerprint(frames: Iterable[Frame]) -> dict[int, int]:
  fp: dict[int, int] = {}
  for _, addr, data in frames:
    if addr < 0x800:
      fp[addr] = max(fp.get(addr, 0), len(data))
  return dict(sorted(fp.items()))


def integrity_bits(length: int) -> set[int]:
  """Checksum and counter bits of a Honda F-CAN message (DBC bit numbers)."""
  last = (length - 1) * 8
  return set(range(last, last + 6))


# ********** results **********

@dataclass
class Finding:
  signal: str
  ok: bool
  detail: str                       # plain words for the person running the tool
  signal_def: dict | None = None    # config "signals" entry to use (set when found somewhere else)
  message_def: dict | None = None   # config "messages" entry the new location needs, when it isn't configured yet
  candidates: list = field(default_factory=list)


def message_entry(addr: int, stats: dict[int, MessageStats]) -> dict:
  s = stats[addr]
  return {"name": f"MSG_{addr:03X}", "addr": f"0x{addr:X}", "len": s.length, "freq": nearest_standard_freq(s.freq),
          "checksum": "honda" if s.honda_integrity else "none", "counter": s.honda_integrity}


def _relocated(cfg: Config, finding: Finding, addr: int, sig: dict, stats: dict[int, MessageStats], where: str) -> Finding:
  existing = next((m for m in cfg.messages.values() if m.addr == addr), None)
  message_def = None
  if existing is not None:
    sig["message"] = existing.name
  else:
    message_def = message_entry(addr, stats)
    sig["message"] = message_def["name"]
  return Finding(finding.signal, True, f"{finding.detail}; {where}", signal_def=sig, message_def=message_def,
                 candidates=finding.candidates)


# ********** single-bit flags (brake, cruise main, door, blinkers, reverse) **********

def _masks(window: Window, addr: int, length: int) -> tuple[int, int] | None:
  """AND and OR of every frame of addr in the window, as little-endian integers (bit k = DBC bit k).
  None when the message is missing from the window."""
  and_mask, or_mask, seen = (1 << (length * 8)) - 1, 0, False
  for _, a, d in window:
    if a == addr and len(d) == length:
      v = int.from_bytes(d, "little")
      and_mask &= v
      or_mask |= v
      seen = True
  return (and_mask, or_mask) if seen else None


def _bit_series(window: Window, addr: int, bit: int) -> list[int]:
  return [(d[bit // 8] >> (bit % 8)) & 1 for _, a, d in window if a == addr]


def _flashes(window: Window, addr: int, bit: int) -> bool:
  """True when the bit switches on and off repeatedly, like a blinker lamp (two or more rising edges)."""
  values = _bit_series(window, addr, bit)
  return sum(1 for x, y in zip(values, values[1:], strict=False) if not x and y) >= 2


def check_flag(cfg: Config, name: str, off: Sequence[Window], on: Sequence[Window], flashing: bool = False) -> Finding:
  """The configured flag must read 0 throughout every off window and 1 throughout every on window
  (or switch on and off repeatedly, for a blinker lamp)."""
  if not cfg.has(name):
    return Finding(name, False, "not in the config")
  sig, msg = cfg.signals[name], cfg.message_of(name)
  for windows, want in ((off, 0), (on, 1)):
    for w in windows:
      values = [int(read_signal(d, sig)) for _, a, d in w if a == msg.addr and len(d) == msg.length]
      if not values:
        return Finding(name, False, f"its message ({msg.addr:#x}) was not in the recording")
      if want == 0 and any(values):
        return Finding(name, False, "reads on when it should be off")
      if want == 1 and not all(values) and not (flashing and _flashes(w, msg.addr, sig.start_bit)):
        return Finding(name, False, "did not come on")
  return Finding(name, True, "matches")


class FlagCandidate(NamedTuple):
  addr: int
  bit: int
  active_high: bool
  flashing: bool


def find_flags(off: Sequence[Window], on: Sequence[Window], stats: dict[int, MessageStats],
               flashing: bool = False) -> list[FlagCandidate]:
  """Bits that are steady in every off window, and in every on window are steady at the other value (or, with
  flashing=True, switch on and off repeatedly)."""
  found = []
  for addr, s in sorted(stats.items()):
    skip = integrity_bits(s.length) if s.honda_integrity else set()
    off_masks = [m for m in (_masks(w, addr, s.length) for w in off) if m is not None]
    on_masks = [m for m in (_masks(w, addr, s.length) for w in on) if m is not None]
    if len(off_masks) != len(off) or len(on_masks) != len(on):
      continue  # the message is missing from a window
    nbits = s.length * 8
    full = (1 << nbits) - 1
    steady1_off, steady0_off = full, full
    for and_m, or_m in off_masks:
      steady1_off &= and_m
      steady0_off &= ~or_m & full
    steady1_on, steady0_on, any1_on = full, full, full
    for and_m, or_m in on_masks:
      steady1_on &= and_m
      steady0_on &= ~or_m & full
      any1_on &= or_m
    for bit in range(nbits):
      if bit in skip:
        continue
      b = 1 << bit
      if steady0_off & b and steady1_on & b:
        found.append(FlagCandidate(addr, bit, True, False))
      elif steady1_off & b and steady0_on & b:
        found.append(FlagCandidate(addr, bit, False, False))
      elif flashing and steady0_off & b and any1_on & b and all(_flashes(w, addr, bit) for w in on):
        found.append(FlagCandidate(addr, bit, True, True))
  return found


def _is_safety_signal(cfg: Config, name: str) -> bool:
  return name in SAFETY_SIGNALS or name in SAFETY_OPTIONAL_SIGNALS


def rank_places(cfg: Config, name: str, places: Sequence, stats: dict[int, MessageStats]) -> tuple[list, int]:
  """Orders candidate places (anything with .addr or a first item that is the address) for a signal: the configured
  message first, then the message that comes most often, then the lowest address. A signal the panda reads can't
  live in a message slower than it accepts, so those are dropped. Returns (ranked places, number dropped as slow)."""
  def addr_of(c):
    return c.addr if hasattr(c, "addr") else c[0]
  configured = cfg.message_of(name).addr if cfg.has(name) else None
  usable, slow = [], 0
  for c in places:
    if _is_safety_signal(cfg, name) and nearest_standard_freq(stats[addr_of(c)].freq) < SAFETY_MIN_FREQ:
      slow += 1
    else:
      usable.append(c)
  usable.sort(key=lambda c: (addr_of(c) != configured, -stats[addr_of(c)].freq, addr_of(c), tuple(c)[1:]))
  return usable, slow


def _search_detail(found: int, slow: int, what: str) -> str:
  if found:
    return ""
  if slow:
    return f"; it shows up only in messages that come less than {SAFETY_MIN_FREQ} times a second, too slow for lane keeping's checks"
  return f"; searched the recording and found no {what} that matches"


def _also(n: int) -> str:
  return f" ({n} other place{'s' if n > 1 else ''} matched too; this one is used)" if n else ""


def resolve_flag(cfg: Config, name: str, off: Sequence[Window], on: Sequence[Window], stats: dict[int, MessageStats],
                 flashing: bool = False) -> Finding:
  """Checks the configured location, and searches the recording when it doesn't match."""
  finding = check_flag(cfg, name, off, on, flashing)
  if finding.ok:
    return finding
  candidates = [c for c in find_flags(off, on, stats, flashing) if c.active_high]  # the only kind the config holds
  finding.candidates = candidates
  steady = [c for c in candidates if not c.flashing]
  # a blinker can show up twice, as the turn signal switch position (steady) and as the lamp (flashing): the switch is cleaner
  pick, slow = rank_places(cfg, name, steady if steady else candidates, stats)
  if not pick:
    finding.detail += _search_detail(0, slow, "place")
    return finding
  c = pick[0]
  # a 1-bit signal starts at its own bit in either byte order
  sig = {"message": "", "start_bit": c.bit, "size": 1, "byte_order": "big"}
  return _relocated(cfg, finding, c.addr, sig, stats, f"found it elsewhere (message {c.addr:#x}, bit {c.bit}){_also(len(pick) - 1)}")


# ********** multi-bit fields (shifter, pedal position) **********

def _be_values(window: Window, addr: int, length: int) -> set[int]:
  return {int.from_bytes(d, "big") for _, a, d in window if a == addr and len(d) == length}


def _be_field(value: int, length: int, idx: int, size: int) -> int:
  return (value >> (length * 8 - idx - size)) & ((1 << size) - 1)


# openpilot's gear letters; any other shifter position (D3, 2, 1) is a low forward gear
OPENPILOT_GEARS = ("P", "R", "N", "D", "S", "L")


def gear_letter(shifter_label: str) -> str:
  label = shifter_label.strip().upper()
  return label if label in OPENPILOT_GEARS else "L"


def check_gear(cfg: Config, windows: dict[str, Window]) -> Finding:
  """windows: shifter position as shown on the dash ('P', 'R', 'N', 'D', 'D3', '2'...) -> recording made there."""
  if not cfg.has("GEAR"):
    return Finding("GEAR", False, "not in the config")
  sig, msg = cfg.signals["GEAR"], cfg.message_of("GEAR")
  for label, w in windows.items():
    values = {int(read_signal(d, sig)) for _, a, d in w if a == msg.addr and len(d) == msg.length}
    if not values:
      return Finding("GEAR", False, f"its message ({msg.addr:#x}) was not in the recording")
    if len(values) != 1 or sig.values.get(next(iter(values))) != gear_letter(label):
      shown = ", ".join(str(sig.values.get(v, v)) for v in sorted(values))
      return Finding("GEAR", False, f"in {label} it reads {shown}")
  return Finding("GEAR", True, "matches")


def find_value_fields(windows: dict[str, Window], stats: dict[int, MessageStats], max_size: int = 6,
                      prefer_addr: int | None = None) -> list[tuple[int, int, int, dict[int, str]]]:
  """Big-endian fields that hold one steady value per window and a different value in every window. Per message,
  the field is the shortest run of bits covering every bit that changes between windows, so a bit that only
  changes in a shifter position that wasn't recorded may be missing. Returns (addr, start_bit, size,
  {raw value: window label}), fields in prefer_addr first, then the narrowest."""
  labels = list(windows)
  found = []
  for addr, s in sorted(stats.items()):
    nbits = s.length * 8
    full = (1 << nbits) - 1
    # integrity bits as positions in the big-endian number: DBC bit k sits at (length-1-k//8)*8 + k%8
    skip = 0
    if s.honda_integrity:
      for k in integrity_bits(s.length):
        skip |= 1 << ((s.length - 1 - k // 8) * 8 + k % 8)
    values = {label: _be_values(windows[label], addr, s.length) for label in labels}
    if any(not v for v in values.values()):
      continue
    steady = full
    for vs in values.values():
      and_m, or_m = full, 0
      for v in vs:
        and_m &= v
        or_m |= v
      steady &= ~(and_m ^ or_m) & full
    first = next(iter(values[labels[0]]))
    varying = 0
    for label in labels[1:]:
      varying |= (next(iter(values[label])) ^ first)
    varying &= steady & ~skip
    if not varying:
      continue
    hi, lo = varying.bit_length() - 1, (varying & -varying).bit_length() - 1
    size = hi - lo + 1
    run = ((1 << size) - 1) << lo
    if size > max_size or run & ~steady or run & skip:
      continue
    mapping = {(next(iter(values[label])) >> lo) & ((1 << size) - 1): label for label in labels}
    if len(mapping) != len(labels):
      continue
    found.append((addr, BE_BITS[nbits - 1 - hi], size, mapping))
  found.sort(key=lambda c: (c[0] != prefer_addr, c[2], c[0]))
  return found


def resolve_gear(cfg: Config, windows: dict[str, Window], stats: dict[int, MessageStats]) -> Finding:
  finding = check_gear(cfg, windows)
  if finding.ok:
    return finding
  prefer = cfg.message_of("GEAR").addr if cfg.has("GEAR") else None
  fields = find_value_fields(windows, stats, prefer_addr=prefer)
  finding.candidates = fields
  # a manual gearbox only shows neutral and reverse; anything that isn't reverse is then a forward gear
  manual = set(windows) <= {"N", "R"}
  if fields:
    addr, start_bit, size, mapping = fields[0]
    letters = {v: ("R" if label == "R" else "D") if manual else gear_letter(label) for v, label in mapping.items()}
    sig = {"message": "", "start_bit": start_bit, "size": size, "byte_order": "big",
           "values": {str(v): letter for v, letter in sorted(letters.items())}}
    others = f" ({len(fields) - 1} other messages also change with the shifter)" if len(fields) > 1 else ""
    return _relocated(cfg, finding, addr, sig, stats, f"found it elsewhere (message {addr:#x}, a {size}-bit field){others}")
  if "R" in windows and len(windows) > 1:
    # no field: settle for a reverse flag, every other position then reads as drive
    off = [w for g, w in windows.items() if g != "R"]
    flags = [c for c in find_flags(off, [windows["R"]], stats) if c.active_high]
    if len(flags) == 1:
      c = flags[0]
      sig = {"message": "", "start_bit": c.bit, "size": 1, "byte_order": "big", "values": {"0": "D", "1": "R"}}
      return _relocated(cfg, finding, c.addr, sig, stats, f"only found a reverse flag, at {c.addr:#x} bit {c.bit}")
  finding.detail += "; searched the recording and found nothing that changes with the shifter"
  return finding


def check_pedal(cfg: Config, name: str, idle: Sequence[Window], pressed: Sequence[Window], min_raw: int = 10) -> Finding:
  """A pedal position: exactly zero at rest, clearly above zero in every press."""
  if not cfg.has(name):
    return Finding(name, False, "not in the config")
  sig, msg = cfg.signals[name], cfg.message_of(name)
  for windows, resting in ((idle, True), (pressed, False)):
    for w in windows:
      values = [read_raw(d, sig.start_bit, sig.size, sig.big_endian) for _, a, d in w if a == msg.addr and len(d) == msg.length]
      if not values:
        return Finding(name, False, f"its message ({msg.addr:#x}) was not in the recording")
      if resting and any(values):
        return Finding(name, False, "is not zero with the pedal released")
      if not resting and max(values) < min_raw:
        return Finding(name, False, "did not rise when the pedal was pressed")
  return Finding(name, True, "matches")


def find_pedal_bytes(idle: Sequence[Window], pressed: Sequence[Window], stats: dict[int, MessageStats],
                     min_raw: int = 10) -> list[tuple[int, int]]:
  """Whole bytes that are zero at rest and rise in every press. Returns (addr, big-endian start bit)."""
  found = []
  for addr, s in sorted(stats.items()):
    for byte in range(s.length - (1 if s.honda_integrity else 0)):
      rest = [d[byte] for w in idle for _, a, d in w if a == addr and len(d) == s.length]
      if not rest or any(rest):
        continue
      if all(max((d[byte] for _, a, d in w if a == addr and len(d) == s.length), default=0) >= min_raw for w in pressed):
        found.append((addr, byte * 8 + 7))
  return found


def resolve_pedal(cfg: Config, name: str, idle: Sequence[Window], pressed: Sequence[Window], stats: dict[int, MessageStats]) -> Finding:
  finding = check_pedal(cfg, name, idle, pressed)
  if finding.ok:
    return finding
  candidates = find_pedal_bytes(idle, pressed, stats)
  finding.candidates = candidates
  pick, slow = rank_places(cfg, name, candidates, stats)
  if not pick:
    finding.detail += _search_detail(0, slow, "place")
    return finding
  addr, start_bit = pick[0]
  sig = {"message": "", "start_bit": start_bit, "size": 8, "byte_order": "big"}
  return _relocated(cfg, finding, addr, sig, stats, f"found it elsewhere (message {addr:#x}, byte {start_bit // 8}){_also(len(pick) - 1)}")


def check_standstill_speed(cfg: Config, window: Window) -> Finding:
  """Parked: the speed message must be there and read zero."""
  sig, msg = cfg.signals["VEHICLE_SPEED"], cfg.message_of("VEHICLE_SPEED")
  values = [read_signal(d, sig) for _, a, d in window if a == msg.addr and len(d) == msg.length]
  if not values:
    return Finding("VEHICLE_SPEED", False, f"its message ({msg.addr:#x}) was not in the recording")
  if max(abs(v) for v in values) > 0.5:
    return Finding("VEHICLE_SPEED", False, f"reads {max(values):.1f} {sig.unit} while parked")
  return Finding("VEHICLE_SPEED", True, "reads zero while parked (the drive check confirms the scale)")


# ********** vehicle speed over a drive **********

@dataclass
class SpeedFit:
  ok: bool
  detail: str
  ratio: float = float("nan")       # configured CAN speed / GPS speed
  samples: int = 0
  signal_def: dict | None = None    # replacement VEHICLE_SPEED entry when the configured one is wrong
  message_def: dict | None = None
  scale: float = float("nan")       # stand-alone: GPS speed / the comma device's own speed, to save in the config


def _pair_with_gps(frames: Sequence[Frame], gps: Sequence[tuple[float, float]], min_speed: float) -> list[tuple[bytes, float]]:
  """(frame data, GPS speed m/s): each GPS fix above min_speed with the nearest frame, if within 0.1 s."""
  frames = sorted(frames, key=lambda f: f[0])
  pairs = []
  j = 0
  for t, v_gps in sorted(gps):
    if v_gps < min_speed or not frames:
      continue
    while j + 1 < len(frames) and abs(frames[j + 1][0] - t) <= abs(frames[j][0] - t):
      j += 1
    if abs(frames[j][0] - t) <= 0.1:
      pairs.append((frames[j][2], v_gps))
  return pairs


def _fit_through_zero(pairs: Sequence[tuple[float, float]]) -> tuple[float, float]:
  """Least squares y = k * x. Returns (k, r2)."""
  sxx = sum(x * x for x, _ in pairs)
  if not pairs or sxx <= 0:
    return float("nan"), 0.0
  k = sum(x * y for x, y in pairs) / sxx
  mean_y = sum(y for _, y in pairs) / len(pairs)
  ss_tot = sum((y - mean_y) ** 2 for _, y in pairs)
  ss_res = sum((y - k * x) ** 2 for x, y in pairs)
  return k, (1.0 - ss_res / ss_tot) if ss_tot > 0 else 0.0


def nice_factor(kph_per_unit: float) -> float:
  """Snaps a fitted factor to a round value when it is within 1.5% of one (including mph-based ones)."""
  for nice in (0.01, 0.005, 0.02, 0.05, 0.1, 0.0625, 1 / 128, 0.5, 1.0, 0.01 * 1.609344, 0.1 * 1.609344, 1.609344):
    if abs(kph_per_unit / nice - 1) < 0.015:
      return nice
  return round(kph_per_unit, 6)


def check_speed(cfg: Config, frames: Sequence[Frame], gps: Sequence[tuple[float, float]], stats: dict[int, MessageStats],
                min_speed: float = 5.0, min_samples: int = 60, tolerance: float = 0.04) -> SpeedFit:
  """Compares VEHICLE_SPEED with GPS speed over a drive, and searches for the speed when they don't agree.
  gps: (seconds, m/s), good fixes only, on the same clock as the frames."""
  sig, msg = cfg.signals["VEHICLE_SPEED"], cfg.message_of("VEHICLE_SPEED")
  unit = SPEED_UNITS[sig.unit]
  moving = [p for p in gps if p[1] >= min_speed]
  if len(moving) < min_samples:
    return SpeedFit(False, f"only {len(moving)} GPS readings above {min_speed * 2.237:.0f} mph; drive longer, where the comma device can see the sky")
  groups = by_addr(frames)
  own = [f for f in groups.get(msg.addr, []) if len(f[2]) == msg.length]
  pairs = [(read_signal(d, sig) * unit, v) for d, v in _pair_with_gps(own, gps, min_speed)]
  if len(pairs) >= min_samples:
    k, r2 = _fit_through_zero(pairs)
    ratio = 1 / k if k else float("nan")
    if r2 > 0.98 and abs(ratio - 1) < tolerance:
      return SpeedFit(True, f"matches the GPS speed within {abs(ratio - 1) * 100:.1f}%", ratio, len(pairs))
    detail = f"reads {ratio:.3f} times the GPS speed, and follows it {max(r2, 0.0):.0%} closely"
  else:
    detail = f"its message ({msg.addr:#x}) was missing during the drive"

  # search: 16-bit unsigned big-endian fields that follow GPS speed
  best = None
  for addr, s in sorted(stats.items()):
    if nearest_standard_freq(s.freq) < SAFETY_MIN_FREQ:
      continue  # too slow for the panda's checks
    paired = _pair_with_gps([f for f in groups.get(addr, []) if len(f[2]) == s.length], gps, min_speed)
    if len(paired) < min_samples:
      continue
    for byte in range(s.length - (2 if s.honda_integrity else 1)):
      xy = [((d[byte] << 8) | d[byte + 1], v) for d, v in paired]
      k, r2 = _fit_through_zero(xy)
      if k > 0 and r2 > 0.995 and (best is None or r2 > best[3]):
        best = (addr, byte * 8 + 7, k, r2)
  if best is None:
    return SpeedFit(False, detail + "; found nothing else in the car's data that follows the GPS speed")
  addr, start_bit, k, r2 = best
  factor = nice_factor(k * 3.6)  # kph per unit
  sig_def = {"message": "", "start_bit": start_bit, "size": 16, "byte_order": "big", "factor": factor, "unit": "kph"}
  existing = next((m for m in cfg.messages.values() if m.addr == addr), None)
  message_def = None
  if existing is not None:
    sig_def["message"] = existing.name
  else:
    message_def = message_entry(addr, stats)
    sig_def["message"] = message_def["name"]
  return SpeedFit(True, f"{detail}; found the speed elsewhere (message {addr:#x}, byte {start_bit // 8}, {factor:g} kph per step)",
                  samples=len(pairs), signal_def=sig_def, message_def=message_def)


DEVICE_SPEED_MIN_R2 = 0.95   # the comma device's own speed must follow the GPS speed this closely


def check_device_speed(device: Sequence[tuple[float, float]], gps: Sequence[tuple[float, float]], min_speed: float = 5.0,
                       min_samples: int = 60) -> SpeedFit:
  """Stand-alone: compares the comma device's own speed (its estimate from the camera and motion sensors) with GPS
  speed over a drive. It passes when they follow each other closely and differ by a steady factor inside
  DEVICE_SPEED_SCALE_RANGE; that factor is saved, and the speed lane keeping uses is the device's own times it.
  device, gps: (seconds, m/s) on the same clock, gps good fixes only."""
  moving = [p for p in gps if p[1] >= min_speed]
  if len(moving) < min_samples:
    return SpeedFit(False, f"only {len(moving)} GPS readings above {min_speed * 2.237:.0f} mph; drive longer, where the comma device can see the sky")
  frames = [(t, 0, v) for t, v in device]      # _pair_with_gps pairs by time; the "data" here is the speed itself
  pairs = [(v_dev, v_gps) for v_dev, v_gps in _pair_with_gps(frames, gps, min_speed)]
  if len(pairs) < min_samples:
    return SpeedFit(False, f"the comma device's own speed was missing for most of the drive ({len(pairs)} readings next to GPS)")
  k, r2 = _fit_through_zero(pairs)
  lo, hi = DEVICE_SPEED_SCALE_RANGE
  detail = f"the comma device's own speed reads {1 / k:.3f} times the GPS speed, and follows it {max(r2, 0.0):.0%} closely"
  if r2 < DEVICE_SPEED_MIN_R2:
    hint = f" (needs {DEVICE_SPEED_MIN_R2:.0%}): check the comma device sits firmly on its mount and can see the road"
    return SpeedFit(False, detail + hint, 1 / k, len(pairs))
  if not lo <= k <= hi:
    return SpeedFit(False, detail + f": more than {round((hi - 1) * 100)}% off. Check the comma device's mount and calibration",
                    1 / k, len(pairs))
  return SpeedFit(True, detail + "; lane keeping uses it scaled to the GPS", 1 / k, len(pairs), scale=round(k, 4))


# ********** steering angle on the car's network **********

ANGLE_MIN_FREQ = 50           # a steering angle slower than this is too stale for the steering controller
ANGLE_MIN_FRAMES = 20         # frames of a message during the motor's travel, for a fit
ANGLE_MIN_R2 = 0.97
ANGLE_MAX_DEG_PER_UNIT = 0.5  # coarser than this is no angle sensor
ANGLE_REST_MAX_UNITS = 2      # how much the field may wander with the wheel still
ANGLE_MIN_LOW_BYTES = 8       # distinct low bytes seen during a turn: fewer means a field read in the wrong byte order

MotorSeries = Sequence[tuple[float, float]]   # (seconds, motor angle in firmware degrees)


def nice_angle_factor(deg_per_unit: float) -> float:
  """Snaps a fitted factor (steering wheel degrees per step, signed) to a round value within 2% of one."""
  sign = -1.0 if deg_per_unit < 0 else 1.0
  for nice in (0.1, 0.05, 0.02, 0.01, 0.0625, 0.03125, 0.015625, 0.125, 0.25, 0.5, 1 / 3, 0.2):
    if abs(abs(deg_per_unit) / nice - 1) < 0.02:
      return sign * nice
  return round(deg_per_unit, 6)


def _interp(series: MotorSeries, t: float) -> float | None:
  """The motor's angle at time t, between its samples; None outside them."""
  if not series or t < series[0][0] or t > series[-1][0]:
    return None
  lo, hi = 0, len(series) - 1
  while hi - lo > 1:
    mid = (lo + hi) // 2
    if series[mid][0] <= t:
      lo = mid
    else:
      hi = mid
  (t0, a0), (t1, a1) = series[lo], series[hi]
  return a0 if t1 == t0 else a0 + (a1 - a0) * (t - t0) / (t1 - t0)


def _travel(series: MotorSeries, min_move_fw: float = 1.0) -> tuple[float, float] | None:
  """When the motor was turning: from its first move to its last change. None if it never moved."""
  if len(series) < 3:
    return None
  start = series[0][1]
  first = next((t for t, a in series if abs(a - start) >= min_move_fw), None)
  if first is None:
    return None
  end = series[-1][1]
  last = next(t for t, a in reversed(series) if abs(a - end) >= 0.5)
  return first - 0.2, last + 0.2


def _fit_line(xy: Sequence[tuple[float, float]]) -> tuple[float, float, float]:
  """Least squares y = a x + b. Returns (a, b, r2)."""
  n = len(xy)
  mx = sum(x for x, _ in xy) / n
  my = sum(y for _, y in xy) / n
  sxx = sum((x - mx) ** 2 for x, _ in xy)
  if sxx <= 0:
    return 0.0, my, 0.0
  a = sum((x - mx) * (y - my) for x, y in xy) / sxx
  b = my - a * mx
  ss_tot = sum((y - my) ** 2 for _, y in xy)
  ss_res = sum((y - a * x - b) ** 2 for x, y in xy)
  return a, b, (1.0 - ss_res / ss_tot) if ss_tot > 0 else 0.0


def _field_value(d: bytes, byte: int, big_endian: bool, signed: bool) -> int:
  return int.from_bytes(d[byte:byte + 2], "big" if big_endian else "little", signed=signed)


def find_angle_signal(cfg: Config, rest: Window, turns: Sequence[tuple[Window, MotorSeries]], stats: dict[int, MessageStats],
                      angle_scale: float) -> Finding:
  """Looks for the steering wheel's angle in the car's data: a 16-bit field that follows the motor's own angle while
  the motor gear is on the steering wheel teeth, during the car setup's turns (the first turn is to the LEFT), and
  holds still with the wheel still. turns: (the F-CAN frames during a turn, the motor's angle meanwhile), left turn
  first. angle_scale: steering wheel degrees per firmware degree. The field's scale comes from the motor's travel,
  and its sign from the left turn; the value with the wheel at rest reads 0."""
  name = "STEER_ANGLE"
  configured = cfg.signals[name] if cfg.has(name) else None
  travels = [(frames, series, _travel(series)) for frames, series in turns]
  if any(tr is None for _, _, tr in travels):
    return Finding(name, False, "not looked for: the motor didn't report the wheel turning")
  rest_groups = by_addr(rest)
  best = None
  slow = 0
  for addr, s in sorted(stats.items()):
    if s.length < 2:
      continue
    too_slow = nearest_standard_freq(s.freq) < ANGLE_MIN_FREQ
    turn_frames = [[(t, d) for t, a, d in frames if a == addr and len(d) == s.length] for frames, _, _ in travels]
    if any(len(tf) < ANGLE_MIN_FRAMES for tf in turn_frames):
      continue
    rest_frames = [d for _, a, d in rest_groups.get(addr, []) if len(d) == s.length]
    last_byte = s.length - (2 if s.honda_integrity else 1)
    for byte in range(last_byte - 1):
      for big_endian in (True, False):
        for signed in (True, False):
          fits = []
          for (_, series, (t0, t1)), tf in zip(travels, turn_frames, strict=True):
            xy = []
            for t, d in tf:
              if t0 <= t <= t1:
                m = _interp(series, t)
                if m is not None:
                  xy.append((m, float(_field_value(d, byte, big_endian, signed))))
            if len(xy) < ANGLE_MIN_FRAMES:
              break
            # a 16-bit field read in the wrong byte order still follows the wheel, in jumps of 256: its low byte
            # hardly changes. A real angle's low byte runs through many values during a turn
            if len({int(v) & 0xFF for _, v in xy}) < ANGLE_MIN_LOW_BYTES:
              break
            a, b, r2 = _fit_line(xy)
            if r2 < ANGLE_MIN_R2 or a == 0.0:
              break
            fits.append((a, b, r2, len(xy)))
          if len(fits) != len(travels):
            continue
          slopes = [a for a, _, _, _ in fits]
          if any(sl * slopes[0] <= 0 for sl in slopes) or max(slopes) / min(slopes) > 1.25 or min(slopes) / max(slopes) < 0.8:
            continue
          if rest_frames:
            values = [_field_value(d, byte, big_endian, signed) for d in rest_frames]
            if max(values) - min(values) > ANGLE_REST_MAX_UNITS:
              continue
          a = sum(slopes) / len(slopes)
          if abs(angle_scale / a) > ANGLE_MAX_DEG_PER_UNIT:
            continue
          if too_slow:
            slow += 1
            continue
          score = min(r2 for _, _, r2, _ in fits)
          if best is None or score > best[0]:
            best = (score, addr, byte, big_endian, signed, a)
  if best is None:
    detail = "not on the car's network (the motor's own angle is used; nothing to do)"
    if slow:
      detail = f"found only in a message that comes less than {ANGLE_MIN_FREQ} times a second, too slow to use; the motor's own angle is used"
    return Finding(name, False, detail)
  _, addr, byte, big_endian, signed, a = best
  # the first turn is to the left, which openpilot counts as positive: the motor moved left by |its travel|
  left_series = travels[0][1]
  left_sign = 1.0 if left_series[-1][1] - left_series[0][1] > 0 else -1.0
  factor = nice_angle_factor(left_sign * angle_scale / a)
  rest_values = sorted(_field_value(d, byte, big_endian, signed) for _, _, d in rest_groups.get(addr, []) if len(d) == stats[addr].length)
  rest_value = rest_values[len(rest_values) // 2] if rest_values else 0
  offset = round(-factor * rest_value, 4)
  start_bit = byte * 8 + 7 if big_endian else byte * 8
  sig = {"message": "", "start_bit": start_bit, "size": 16, "byte_order": "big" if big_endian else "little", "signed": signed,
         "factor": factor, "offset": offset, "unit": "deg"}
  where = f"found (message {addr:#x}, byte {byte}, {abs(factor):g} degrees per step)"
  if configured is not None and cfg.message_of(name).addr == addr and configured.start_bit == start_bit:
    where = f"matches ({abs(factor):g} degrees per step)"
  return _relocated(cfg, Finding(name, False, "follows the motor's angle while the gear is on the teeth"), addr, sig, stats, where)


def angle_from_drive(cfg: Config, frames: Sequence[Frame]) -> list[tuple[float, float]]:
  """(seconds, steering wheel degrees, left positive) from a drive's F-CAN frames, for the configured STEER_ANGLE."""
  sig, msg = cfg.signals["STEER_ANGLE"], cfg.message_of("STEER_ANGLE")
  return [(t, read_signal(d, sig)) for t, a, d in frames if a == msg.addr and len(d) == msg.length]


# ********** motor and steering wheel teeth **********

@dataclass
class MotorCheck:
  ok: bool
  detail: str
  direction: int = 0       # value for actuator.direction, 0 when unknown
  wrong_way: bool = False  # the motor turned the wheel the opposite way to what it was told


def motor_direction_from_turn(start_deg: float, left_deg: float, right_deg: float, angle_scale: float,
                              tracked_arc_deg: float, arc_tolerance: float = 0.35) -> MotorCheck:
  """The driver turned the steering wheel left until the motor gear ran off the end of the steering wheel teeth and
  stopped turning, then right past the middle to the other end. Inputs are the motor's reported angle
  (STEERING_ANGLE) at the start and at each end. The motor turns with the wheel over the teeth, and the rolling band
  past each end when its gear has a roller (tracked_arc_deg), so the two ends are that far apart when the gearing
  settings are right.
  openpilot counts a left turn as positive, so direction is the sign that makes left positive."""
  moved_left = left_deg - start_deg
  moved_right = right_deg - start_deg
  if abs(moved_left) < 3.0 or abs(moved_right) < 3.0:
    return MotorCheck(False, "the motor didn't report the wheel turning both ways; is the lift lever down, with the motor gear on the teeth?")
  if moved_left * moved_right > 0:
    return MotorCheck(False, "the left and right turns moved the motor the same way; turn left first, then right")
  span = abs(left_deg - right_deg) * angle_scale
  if abs(span / tracked_arc_deg - 1) > arc_tolerance:
    detail = f"the motor turned with the steering wheel for {span:.0f} degrees, but the steering wheel teeth cover {tracked_arc_deg:.0f}"
    return MotorCheck(False, detail + "; the motor's gearing setting doesn't match the printed gears")
  direction = 1 if moved_left > 0 else -1
  setting = "normal" if direction == 1 else "reversed"
  return MotorCheck(True, f"the motor turned with the steering wheel for {span:.0f} degrees; motor direction setting: {setting}",
                    direction)


def motor_push_agrees(direction: int, angle_before: float, angle_after: float, min_move_deg: float = 2.0) -> MotorCheck:
  """The motor was told to turn the wheel left (openpilot's positive torque, sent as direction * torque). Left reads
  as a positive change once multiplied by the same direction."""
  moved = direction * (angle_after - angle_before)
  if abs(moved) < min_move_deg:
    return MotorCheck(False, f"the wheel moved only {abs(moved):.1f} degrees", direction)
  if moved < 0:
    return MotorCheck(False, "the motor turned the wheel right when told left", direction, wrong_way=True)
  return MotorCheck(True, f"the motor turned the wheel {moved:.0f} degrees left when told left", direction)


MOTOR_STATUS_ADDR = 0x22F
MSG_NO_MOTOR_IN_DRIVE = " ".join((
  "the motor wasn't heard during the drive: drive with the lift lever down",
  "(lane keeping stays off until this check passes)",
))


def motor_angle_fw(data: bytes) -> float:
  """STEERING_STATUS.STEERING_ANGLE (firmware degrees) from a motor status frame."""
  return int.from_bytes(data[5:7], "little", signed=True) * 0.125


def steer_agrees_with_course(wheel: Sequence[tuple[float, float]], headings: Sequence[tuple[float, float, float]],
                             min_speed: float = 8.0, min_turn_deg_s: float = 2.0, min_steer_deg: float = 0.5) -> tuple[int, int]:
  """How often a steering wheel angle reading (seconds, degrees, left positive; its zero needn't be straight ahead)
  was on the same side as the car's turning, from the GPS course: (agreeing moments, turning moments).
  headings: (seconds, GPS course in degrees clockwise from north, speed m/s), good fixes only, on the same clock."""
  wheel = sorted(wheel)
  times = [t for t, _ in wheel]
  angles = [a for _, a in wheel]
  centre = sorted(angles)[len(angles) // 2]   # where the steering wheel sits on straight road
  agree = total = 0
  hs = sorted(headings)
  j = 0
  for (t0, b0, v0), (t1, b1, v1) in zip(hs, hs[1:], strict=False):
    dt = t1 - t0
    if not 0.05 <= dt <= 2.0 or min(v0, v1) < min_speed:
      continue
    turn = ((b1 - b0 + 180.0) % 360.0 - 180.0) / dt   # degrees per second, clockwise (right) positive
    if abs(turn) < min_turn_deg_s:
      continue
    tm = (t0 + t1) / 2
    while j + 1 < len(times) and times[j + 1] <= tm:
      j += 1
    if abs(times[j] - tm) > 0.5:
      continue
    steer = angles[j] - centre                          # left positive
    if abs(steer) < min_steer_deg:
      continue
    total += 1
    agree += 1 if steer * -turn > 0 else 0
  return agree, total


def motor_direction_from_drive(motor: Sequence[tuple[float, float]], headings: Sequence[tuple[float, float, float]],
                               direction: int, angle_scale: float, min_speed: float = 8.0, min_turn_deg_s: float = 2.0,
                               min_steer_deg: float = 0.5, min_samples: int = 20) -> MotorCheck:
  """Checks the saved motor direction against the car's own turning, with nothing from the person doing the setup.
  During a drive with the lift lever down and lane keeping off, the motor's angle follows the steering wheel. When
  the GPS course swings left, the steering wheel must be left of where it sits on straight road, and the saved
  direction must make that read as left (positive).
  motor: (seconds, motor angle in firmware degrees). headings: (seconds, GPS course in degrees clockwise from north,
  speed m/s), good fixes only. Both on the same clock."""
  if len(motor) < 1000:
    return MotorCheck(False, MSG_NO_MOTOR_IN_DRIVE, direction)
  agree, total = steer_agrees_with_course([(t, direction * a * angle_scale) for t, a in motor], headings, min_speed,
                                          min_turn_deg_s, min_steer_deg)
  if total < min_samples:
    detail = f"too little turning to check the motor's direction ({total} turning moments, need {min_samples})"
    return MotorCheck(False, f"{detail}: include some curves or lane changes above {min_speed * 2.237:.0f} mph", direction)
  share = agree / total
  if share >= 0.8:
    return MotorCheck(True, f"the steering wheel turned the same way as the car in {share:.0%} of {total} turning moments",
                      direction)
  if share <= 0.2:
    detail = f"the motor reads the steering wheel the opposite way to how the car turned ({1 - share:.0%} of {total})"
    return MotorCheck(False, f"{detail}: its saved direction is wrong. Run part 1 again", direction, wrong_way=True)
  detail = f"the steering wheel and the car's turning didn't line up clearly ({share:.0%} of {total})"
  return MotorCheck(False, f"{detail}: drive again with the lift lever down", direction)


def angle_direction_from_drive(cfg: Config, frames: Sequence[Frame], headings: Sequence[tuple[float, float, float]],
                               min_samples: int = 20) -> Finding:
  """Checks the steering angle the car setup found on the car's network against the car's own turning over a drive.
  ok: it reads left when the car turns left. Not ok: it reads backwards, or doesn't follow the car, or there wasn't
  enough turning to tell; the drive check then drops it, and the motor's own angle is used."""
  name = "STEER_ANGLE"
  wheel = angle_from_drive(cfg, frames)
  if len(wheel) < 500:
    return Finding(name, False, "its message was hardly seen during the drive: dropped, the motor's own angle is used")
  agree, total = steer_agrees_with_course(wheel, headings)
  if total < min_samples:
    detail = f"too little turning to check it ({total} turning moments, need {min_samples}): dropped, the motor's own angle is used"
    return Finding(name, False, detail)
  share = agree / total
  if share >= 0.8:
    return Finding(name, True, f"reads the same way as the car turned in {share:.0%} of {total} turning moments",
                   signal_def=_signal_entry(cfg, name))
  if share <= 0.2:
    detail = f"reads the opposite way to how the car turned ({1 - share:.0%} of {total})"
  else:
    detail = f"doesn't follow the car's turning clearly ({share:.0%} of {total})"
  return Finding(name, False, f"{detail}: dropped, the motor's own angle is used")


def _signal_entry(cfg: Config, name: str) -> dict:
  """The config's own entry for a signal, as it is written."""
  sig = cfg.signals[name]
  out = {"message": sig.message, "start_bit": sig.start_bit, "size": sig.size, "byte_order": "big" if sig.big_endian else "little"}
  if sig.signed:
    out["signed"] = True
  if sig.factor != 1.0:
    out["factor"] = sig.factor
  if sig.offset != 0.0:
    out["offset"] = sig.offset
  if sig.unit:
    out["unit"] = sig.unit
  if sig.values:
    out["values"] = {str(k): v for k, v in sig.values.items()}
  return out


# ********** writing the result **********

def apply_findings(raw: dict, findings: Sequence[Finding]) -> dict:
  """Returns a copy of raw with each finding applied: relocated signals updated, optional signals that failed and
  weren't found removed, messages added or removed to match."""
  out = copy.deepcopy(raw)
  signals = out["fcan"]["signals"]
  messages = out["fcan"]["messages"]
  for f in findings:
    if f.ok and f.signal_def is not None:
      if f.message_def is not None and not any(_parse_addr(m["addr"]) == _parse_addr(f.message_def["addr"]) for m in messages):
        messages.append(dict(f.message_def))
      signals[f.signal] = dict(f.signal_def)
    elif not f.ok and f.signal not in REQUIRED_SIGNALS:
      signals.pop(f.signal, None)
  used = {s["message"] for s in signals.values()}
  out["fcan"]["messages"] = [m for m in messages if m["name"] in used]
  return out


def update_message_rates(raw: dict, stats: dict[int, MessageStats]) -> list[str]:
  """Sets each configured message's rate, length and integrity checks to what the car actually sends.
  Returns plain-word notes about what changed."""
  notes = []
  for m in raw["fcan"]["messages"]:
    addr = _parse_addr(m["addr"])
    s = stats.get(addr)
    if s is None:
      continue
    freq = nearest_standard_freq(s.freq)
    if freq != m["freq"]:
      notes.append(f"{m['name']} ({addr:#x}) comes {freq} times a second, not {m['freq']}")
      m["freq"] = freq
    if s.length != m["len"]:
      notes.append(f"{m['name']} ({addr:#x}) is {s.length} bytes long, not {m['len']}")
      m["len"] = s.length
    if (m.get("checksum") == "honda") != s.honda_integrity:
      notes.append(f"{m['name']} ({addr:#x}) {'has' if s.honda_integrity else 'does not have'} a Honda checksum and counter")
      m["checksum"] = "honda" if s.honda_integrity else "none"
      m["counter"] = s.honda_integrity
  return notes


def missing_messages(cfg: Config, stats: dict[int, MessageStats]) -> list[str]:
  """Configured messages the car didn't send."""
  return [f"{m.name} ({m.addr:#x})" for m in cfg.messages.values() if m.addr not in stats]


def required_ok(findings: Sequence[Finding]) -> bool:
  by_name = {f.signal: f for f in findings}
  return all(name in by_name and by_name[name].ok for name in REQUIRED_SIGNALS)
