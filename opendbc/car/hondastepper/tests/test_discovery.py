import copy
import json
import math
import random
import unittest

from opendbc.can import CANPacker
from opendbc.car.hondastepper import discovery as dsc
from opendbc.car.hondastepper.config import CONFIG_PATH, DBC_NAME, parse_config
from opendbc.car.hondastepper.tests.sim import DEFAULT_LAYOUT, MESSAGES, MOVED_GEARS, MOVED_LAYOUT, SimCar, turning_drive

DEFAULT_RAW = json.load(open(CONFIG_PATH))
DEFAULT_CFG = parse_config(DEFAULT_RAW)

def parked_session(car: SimCar):
  """The setup tool's parked steps, as windows of frames: each action done twice, with rest in between."""
  rec: dict = {"rest": car.run(3.0)}

  def twice(name, on_value=1):
    on1 = car.run(3.0, **{name: on_value})
    off1 = car.run(3.0, **{name: 0})
    on2 = car.run(3.0, **{name: on_value})
    off2 = car.run(3.0, **{name: 0})
    return [off1, off2], [on1, on2]

  rec["brake_off"], rec["brake"] = twice("BRAKE_PRESSED")
  rec["gas_off"], rec["gas"] = twice("GAS_PEDAL", 70)
  rec["left_off"], rec["left_on"] = twice("LEFT_BLINKER")
  rec["right_off"], rec["right_on"] = twice("RIGHT_BLINKER")
  rec["door_off"], rec["door_on"] = twice("DOOR_OPEN")
  rec["gears"] = {g: car.run(3.0, GEAR=g) for g in ("P", "R", "N", "D")}
  car.run(1.0, GEAR="P")
  rec["stats"] = dsc.message_stats(car.history, car.t)
  return rec


def resolve_all(cfg, rec):
  stats = rec["stats"]
  return [
    dsc.check_standstill_speed(cfg, rec["rest"]),
    dsc.resolve_flag(cfg, "BRAKE_PRESSED", rec["brake_off"], rec["brake"], stats),
    dsc.resolve_pedal(cfg, "GAS_PEDAL", rec["gas_off"], rec["gas"], stats),
    dsc.resolve_flag(cfg, "LEFT_BLINKER", rec["left_off"], rec["left_on"], stats, flashing=True),
    dsc.resolve_flag(cfg, "RIGHT_BLINKER", rec["right_off"], rec["right_on"], stats, flashing=True),
    dsc.resolve_flag(cfg, "DOOR_OPEN", rec["door_off"], rec["door_on"], stats),
    dsc.resolve_gear(cfg, rec["gears"], stats),
  ]


class TestLowLevel(unittest.TestCase):
  def test_checksum_matches_packer(self):
    packer = CANPacker(DBC_NAME)
    for counter in range(4):
      addr, dat, _ = packer.make_can_msg("ENGINE_DATA", 0, {"VEHICLE_SPEED": 51.23, "COUNTER": counter})
      self.assertEqual(dat[-1] & 0xF, dsc.honda_checksum(addr, dat))
      self.assertEqual(dsc.with_honda_checksum(addr, bytes(dat[:-1]) + b"\x00", counter), bytes(dat))

  def test_read_raw_matches_packer(self):
    packer = CANPacker(DBC_NAME)
    _, dat, _ = packer.make_can_msg("GEARBOX", 0, {"GEAR": 10})
    self.assertEqual(dsc.read_raw(dat, 35, 4, True), 10)
    _, dat, _ = packer.make_can_msg("ENGINE_DATA", 0, {"VEHICLE_SPEED": 123.45})
    self.assertAlmostEqual(dsc.read_signal(dat, DEFAULT_CFG.signals["VEHICLE_SPEED"]), 123.45, places=2)
    self.assertEqual(dsc.read_raw(bytes([0x00, 0xB0, 0x5A, 0, 0, 0, 0, 0]), 12, 12, False), 0x5AB)

  def test_inventory(self):
    car = SimCar()
    frames = car.run(2.0)
    stats = dsc.message_stats(frames, 2.0)
    self.assertEqual(dsc.nearest_standard_freq(stats[0x158].freq), 100)
    self.assertEqual(dsc.nearest_standard_freq(stats[0x294].freq), 10)
    self.assertTrue(stats[0x158].honda_integrity)
    self.assertEqual(stats[0x255].length, 5)
    self.assertEqual(dsc.fingerprint(frames)[0x255], 5)
    # a message without the Honda checksum
    stats2 = dsc.message_stats([(i * 0.01, 0x123, bytes([i % 256, 0, 0, 0])) for i in range(100)], 1.0)
    self.assertFalse(stats2[0x123].honda_integrity)

  def test_update_message_rates(self):
    raw = copy.deepcopy(DEFAULT_RAW)
    car = SimCar()
    stats = dsc.message_stats(car.run(2.0), 2.0)
    stats[0x1A6] = dsc.MessageStats(0x1A6, 8, 20, 10.0, True)
    stats[0x294] = dsc.MessageStats(0x294, 8, 20, 10.0, False)
    notes = dsc.update_message_rates(raw, stats)
    by_name = {m["name"]: m for m in raw["fcan"]["messages"]}
    self.assertEqual(by_name["SCM_BUTTONS"]["freq"], 10)
    self.assertEqual(by_name["SCM_FEEDBACK"]["checksum"], "none")
    self.assertFalse(by_name["SCM_FEEDBACK"]["counter"])
    self.assertEqual(len(notes), 2)
    parse_config(raw)
    self.assertEqual(dsc.missing_messages(DEFAULT_CFG, {}), [f"{m.name} ({m.addr:#x})" for m in DEFAULT_CFG.messages.values()])


class TestParkedChecks(unittest.TestCase):
  def test_default_layout_matches_config(self):
    rec = parked_session(SimCar())
    findings = resolve_all(DEFAULT_CFG, rec)
    for f in findings:
      self.assertTrue(f.ok, f"{f.signal}: {f.detail}")
      self.assertIsNone(f.signal_def, f.signal)
    self.assertTrue(dsc.required_ok(findings))
    self.assertEqual(dsc.apply_findings(DEFAULT_RAW, findings), DEFAULT_RAW)

  def test_steady_blinker_lever_also_matches(self):
    rec = parked_session(SimCar(flashing_blinkers=False))
    for f in resolve_all(DEFAULT_CFG, rec):
      self.assertTrue(f.ok, f"{f.signal}: {f.detail}")

  def test_moved_layout_is_found(self):
    messages = {a: v for a, v in MESSAGES.items() if a not in (0x158, 0x1A3)}  # this car has no ENGINE_DATA or GEARBOX
    rec = parked_session(SimCar(layout=MOVED_LAYOUT, gears=MOVED_GEARS, messages=messages))
    findings = resolve_all(DEFAULT_CFG, rec)
    by_name = {f.signal: f for f in findings}
    for name in ("BRAKE_PRESSED", "GAS_PEDAL", "LEFT_BLINKER", "RIGHT_BLINKER", "DOOR_OPEN", "GEAR"):
      f = by_name[name]
      self.assertTrue(f.ok, f"{name}: {f.detail}")
      self.assertIsNotNone(f.signal_def, name)
      addr, start_bit, size = MOVED_LAYOUT[name]
      self.assertEqual((f.signal_def["start_bit"], f.signal_def["size"]), (start_bit, size), name)
    self.assertEqual(by_name["GEAR"].signal_def["values"], {"0": "P", "1": "D", "6": "N", "7": "R"})
    # the speed message isn't there while parked: found later by the drive check
    self.assertFalse(by_name["VEHICLE_SPEED"].ok)

    raw = dsc.apply_findings(DEFAULT_RAW, findings)
    cfg = parse_config(raw)
    self.assertEqual(cfg.message_of("GEAR").addr, 0x191)
    self.assertEqual(cfg.message_of("DOOR_OPEN").addr, 0x405)
    self.assertEqual(cfg.message_of("BRAKE_PRESSED").name, "POWERTRAIN_DATA")  # same message, other bit
    self.assertNotIn("GEARBOX", cfg.messages)  # nothing reads it any more
    # the updated config passes the same checks without searching
    for f in resolve_all(cfg, rec)[1:]:
      self.assertTrue(f.ok, f"{f.signal}: {f.detail}")
      self.assertIsNone(f.signal_def, f.signal)

  def test_missing_signal_is_dropped(self):
    car = SimCar(layout={k: v for k, v in DEFAULT_LAYOUT.items() if k != "DOOR_OPEN"})
    rec = parked_session(car)
    findings = resolve_all(DEFAULT_CFG, rec)
    door = next(f for f in findings if f.signal == "DOOR_OPEN")
    self.assertFalse(door.ok)
    raw = dsc.apply_findings(DEFAULT_RAW, findings)
    self.assertNotIn("DOOR_OPEN", raw["fcan"]["signals"])
    parse_config(raw)

  def test_required_signal_failure_is_reported(self):
    car = SimCar(layout={k: v for k, v in DEFAULT_LAYOUT.items() if k != "BRAKE_PRESSED"})
    findings = resolve_all(DEFAULT_CFG, parked_session(car))
    self.assertFalse(dsc.required_ok(findings))
    # a required signal is kept rather than dropped, so a failed setup never writes an incomplete config
    self.assertIn("BRAKE_PRESSED", dsc.apply_findings(DEFAULT_RAW, findings)["fcan"]["signals"])

  def test_every_lever_position(self):
    # a 4-bit gear field where P/R/N/D alone never set the top bit, so the extra positions are needed to see it all
    layout = {**DEFAULT_LAYOUT, "GEAR": (0x191, 7, 4)}
    gears = {"P": 0, "R": 1, "N": 2, "D": 3, "D3": 9, "2": 10, "1": 11}
    car = SimCar(layout=layout, gears=gears)
    windows = {g: car.run(3.0, GEAR=g) for g in gears}
    f = dsc.resolve_gear(DEFAULT_CFG, windows, dsc.message_stats(car.history, car.t))
    self.assertTrue(f.ok, f.detail)
    self.assertEqual((f.signal_def["start_bit"], f.signal_def["size"]), (7, 4))
    self.assertEqual(f.signal_def["values"], {"0": "P", "1": "R", "2": "N", "3": "D", "9": "L", "10": "L", "11": "L"})
    raw = dsc.apply_findings(DEFAULT_RAW, [f])
    self.assertTrue(dsc.check_gear(parse_config(raw), windows).ok)

  def test_reverse_flag_fallback(self):
    # a manual gearbox: only a reverse switch
    layout = {k: v for k, v in DEFAULT_LAYOUT.items() if k != "GEAR"}
    layout["GEAR"] = (0x1A6, 18, 1)
    car = SimCar(layout=layout, gears={"N": 0, "R": 1, "P": 0, "D": 0})
    windows = {"N": car.run(3.0, GEAR="N"), "R": car.run(3.0, GEAR="R")}
    stats = dsc.message_stats(windows["N"] + windows["R"], 6.0)
    f = dsc.resolve_gear(DEFAULT_CFG, windows, stats)
    self.assertTrue(f.ok, f.detail)
    self.assertEqual(f.signal_def["start_bit"], 18)
    self.assertEqual(f.signal_def["values"], {"0": "D", "1": "R"})

  def test_noise_is_not_a_flag(self):
    car = SimCar()
    off = [car.run(3.0), car.run(3.0)]
    on = [car.run(3.0), car.run(3.0)]  # nothing actually changes
    stats = dsc.message_stats(off[0] + on[0], 6.0)
    self.assertEqual(dsc.find_flags(off, on, stats, flashing=True), [])


class TestSeveralPlaces(unittest.TestCase):
  def test_two_matching_places_picks_one(self):
    # the brake isn't where the map says, and the car sends it in two places: the configured message wins
    layout = dict(DEFAULT_LAYOUT)
    layout["BRAKE_PRESSED"] = (0x17C, 32, 1)
    car = SimCar(layout=layout, copies={"BRAKE_PRESSED": [(0x1A6, 20, 1)]})
    rec = parked_session(car)
    f = dsc.resolve_flag(DEFAULT_CFG, "BRAKE_PRESSED", rec["brake_off"], rec["brake"], rec["stats"])
    self.assertTrue(f.ok, f.detail)
    self.assertEqual(f.signal_def["start_bit"], 32)
    self.assertEqual(f.signal_def["message"], "POWERTRAIN_DATA")
    self.assertIn("1 other place matched too", f.detail)

  def test_safety_signal_in_a_slow_message_is_refused(self):
    # the panda can't check a message that comes 5 times a second, so the brake can't be taken from there
    layout = dict(DEFAULT_LAYOUT)
    layout["BRAKE_PRESSED"] = (0x3D9, 4, 1)
    car = SimCar(layout=layout)
    rec = parked_session(car)
    f = dsc.resolve_flag(DEFAULT_CFG, "BRAKE_PRESSED", rec["brake_off"], rec["brake"], rec["stats"])
    self.assertFalse(f.ok)
    self.assertIn("less than 10 times a second", f.detail)
    # a signal the panda doesn't read may live there
    layout = dict(DEFAULT_LAYOUT)
    layout["DOOR_OPEN"] = (0x3D9, 4, 1)
    rec = parked_session(SimCar(layout=layout))
    self.assertTrue(dsc.resolve_flag(DEFAULT_CFG, "DOOR_OPEN", rec["door_off"], rec["door_on"], rec["stats"]).ok)


class TestDriveDirection(unittest.TestCase):
  def test_right_direction_passes(self):
    for d in (1, -1):
      motor, headings = turning_drive(300.0, d)
      c = dsc.motor_direction_from_drive(motor, headings, d, 1.0)
      self.assertTrue(c.ok, c.detail)

  def test_reversed_direction_fails(self):
    motor, headings = turning_drive(300.0, 1)
    c = dsc.motor_direction_from_drive(motor, headings, -1, 1.0)
    self.assertFalse(c.ok)
    self.assertTrue(c.wrong_way)
    self.assertIn("Run part 1 again", c.detail)

  def test_needs_the_motor_and_some_turning(self):
    _, headings = turning_drive(300.0)
    c = dsc.motor_direction_from_drive([], headings, 1, 1.0)
    self.assertFalse(c.ok)
    self.assertIn("lift lever down", c.detail)
    motor, headings = turning_drive(300.0, turning=False)
    c = dsc.motor_direction_from_drive(motor, headings, 1, 1.0)
    self.assertFalse(c.ok)
    self.assertIn("too little turning", c.detail)


class TestSpeedCheck(unittest.TestCase):
  def _drive(self, car: SimCar, gps_scale: float = 1.0):
    frames, gps = [], []
    for i in range(240):  # 2 minutes of speed changes, GPS at 2 Hz
      v_mps = 8 + 10 * (0.5 + 0.5 * math.sin(i / 20))
      chunk = car.run(0.5, VEHICLE_SPEED=v_mps * 3.6)
      frames += chunk
      gps.append((chunk[0][0], v_mps * gps_scale))
    return frames, gps

  def test_configured_speed_matches(self):
    car = SimCar()
    frames, gps = self._drive(car)
    stats = dsc.message_stats(frames, frames[-1][0] - frames[0][0])
    fit = dsc.check_speed(DEFAULT_CFG, frames, gps, stats)
    self.assertTrue(fit.ok, fit.detail)
    self.assertIsNone(fit.signal_def)
    self.assertAlmostEqual(fit.ratio, 1.0, delta=0.01)

  def test_wrong_scale_is_found(self):
    car = SimCar(layout=MOVED_LAYOUT, gears=MOVED_GEARS, speed_factor_kph=0.005,
                 messages={a: v for a, v in MESSAGES.items() if a != 0x158})
    frames, gps = self._drive(car)
    stats = dsc.message_stats(frames, frames[-1][0] - frames[0][0])
    fit = dsc.check_speed(DEFAULT_CFG, frames, gps, stats)
    self.assertTrue(fit.ok, fit.detail)
    self.assertEqual(fit.signal_def["factor"], 0.005)
    self.assertEqual(fit.signal_def["start_bit"], 23)
    self.assertEqual(fit.message_def["addr"], "0x309")

  def test_mph_car(self):
    car = SimCar(speed_factor_kph=0.01 * 1.609344)
    frames, gps = self._drive(car)
    stats = dsc.message_stats(frames, frames[-1][0] - frames[0][0])
    fit = dsc.check_speed(DEFAULT_CFG, frames, gps, stats)
    self.assertTrue(fit.ok, fit.detail)
    self.assertAlmostEqual(fit.signal_def["factor"], 0.01 * 1.609344)
    self.assertEqual(fit.signal_def["message"], "ENGINE_DATA")

  def test_not_enough_driving(self):
    car = SimCar()
    frames = car.run(5.0, VEHICLE_SPEED=50.0)
    fit = dsc.check_speed(DEFAULT_CFG, frames, [(0.0, 14.0)] * 10, dsc.message_stats(frames, 5.0))
    self.assertFalse(fit.ok)

  def test_nice_factor(self):
    self.assertEqual(dsc.nice_factor(0.01004), 0.01)
    self.assertEqual(dsc.nice_factor(0.0161), 0.01 * 1.609344)
    self.assertEqual(dsc.nice_factor(0.0137), 0.0137)


class TestDeviceSpeedCheck(unittest.TestCase):
  """Stand-alone: the comma device's own speed (20 times a second) against GPS (2 times a second)."""

  def _drive(self, device_scale: float = 1.0, noise: float = 0.2, gps_hz: float = 2.0, seconds: float = 120.0, seed: int = 1):
    rng = random.Random(seed)
    device, gps = [], []
    for i in range(int(seconds * 20)):
      t = i * 0.05
      v = 8 + 10 * (0.5 + 0.5 * math.sin(t / 10))
      device.append((t, v * device_scale + rng.gauss(0, noise)))
      if i % int(20 / gps_hz) == 0:
        gps.append((t, v))
    return device, gps

  def test_matching_speed_passes_with_its_scale(self):
    for scale in (1.0, 0.95, 1.08):
      with self.subTest(scale=scale):
        fit = dsc.check_device_speed(*self._drive(device_scale=scale))
        self.assertTrue(fit.ok, fit.detail)
        self.assertAlmostEqual(fit.scale, 1 / scale, delta=0.01)

  def test_far_off_or_noisy_speed_fails(self):
    fit = dsc.check_device_speed(*self._drive(device_scale=1.3))
    self.assertFalse(fit.ok)
    self.assertIn("off", fit.detail)
    fit = dsc.check_device_speed(*self._drive(noise=4.0))
    self.assertFalse(fit.ok)
    self.assertIn("follows it", fit.detail)

  def test_not_enough_driving_or_no_device_speed(self):
    device, gps = self._drive(seconds=20.0)
    self.assertFalse(dsc.check_device_speed(device, gps).ok)
    _, gps = self._drive()
    fit = dsc.check_device_speed([], gps)
    self.assertFalse(fit.ok)
    self.assertIn("missing", fit.detail)


class TestMotorChecks(unittest.TestCase):
  def test_direction_from_turn(self):
    angle_scale = 10.363636 / 10.368421
    arc = 71.5
    ok = dsc.motor_direction_from_turn(0.0, 35.0, -36.0, angle_scale, arc)
    self.assertTrue(ok.ok, ok.detail)
    self.assertEqual(ok.direction, 1)
    flipped = dsc.motor_direction_from_turn(5.0, -30.0, 40.0, angle_scale, arc)
    self.assertTrue(flipped.ok, flipped.detail)
    self.assertEqual(flipped.direction, -1)
    # stopping as soon as the motor gear leaves the teeth, before the rolling band ends, still passes
    self.assertTrue(dsc.motor_direction_from_turn(0.0, 29.0, -29.5, angle_scale, arc).ok)
    self.assertFalse(dsc.motor_direction_from_turn(0.0, 1.0, -1.0, angle_scale, arc).ok)       # didn't move
    self.assertFalse(dsc.motor_direction_from_turn(0.0, 20.0, 30.0, angle_scale, arc).ok)      # same way twice
    self.assertFalse(dsc.motor_direction_from_turn(0.0, 300.0, -300.0, angle_scale, arc).ok)   # gearing mismatch

  def test_push(self):
    self.assertTrue(dsc.motor_push_agrees(1, 0.0, 6.0).ok)
    self.assertTrue(dsc.motor_push_agrees(-1, 0.0, -6.0).ok)
    self.assertFalse(dsc.motor_push_agrees(1, 0.0, -6.0).ok)
    self.assertTrue(dsc.motor_push_agrees(1, 0.0, -6.0).wrong_way)
    self.assertFalse(dsc.motor_push_agrees(1, 0.0, 0.5).wrong_way)
    self.assertFalse(dsc.motor_push_agrees(1, 0.0, 0.5).ok)


if __name__ == "__main__":
  unittest.main()


class TestAngleSearch(unittest.TestCase):
  """The steering wheel's angle on the car's network, found from the car setup's turns."""

  def _turns(self, car: SimCar, wheel):
    """The setup's recordings: 3 s at rest, then a left turn and a right turn of the wheel, with the motor's angle
    following the wheel over the teeth (the motor counts the other way, and stops once the gear runs off)."""
    def motor_fw(deg):   # the motor's angle in firmware degrees: -1 direction, follows the wheel within +-35
      return -max(-35.0, min(35.0, deg)) / 1.0
    wheel.deg = 0.0
    rest = car.run(3.0)
    turns = []
    for target in (45.0, -45.0):
      frames, series = [], []
      steps = 60
      for i in range(steps + 1):
        wheel.deg += (target - wheel.deg) / (steps - i) if i < steps else 0.0
        frames += car.run(0.1)
        series.append((car.t, motor_fw(wheel.deg)))
      for _ in range(15):      # still for 1.5 s
        frames += car.run(0.1)
        series.append((car.t, motor_fw(wheel.deg)))
      turns.append((frames, series))
    return rest, turns

  def _car(self, start_bit, factor, freq=100):
    class Wheel:
      deg = 0.0
    wheel = Wheel()
    layout = {**DEFAULT_LAYOUT, "STEER_ANGLE": (0x1C2, start_bit, 16)}
    car = SimCar(layout=layout, messages={**MESSAGES, 0x1C2: (8, freq)}, wheel_deg_fn=lambda: wheel.deg, angle_factor_deg=factor)
    return car, wheel

  def _find(self, car, wheel):
    rest, turns = self._turns(car, wheel)
    stats = dsc.message_stats(car.history, car.t)
    cfg = parse_config(json.load(open(CONFIG_PATH)))
    return dsc.find_angle_signal(cfg, rest, turns, stats, 1.0)

  def test_found_in_either_byte_order_and_sign(self):
    # big-endian signed at byte 2 (Honda style, -0.1 a step), then an unsigned field counting up from 30000
    car, wheel = self._car(23, -0.1)
    f = self._find(car, wheel)
    self.assertTrue(f.ok, f.detail)
    self.assertEqual((f.signal_def["start_bit"], f.signal_def["byte_order"], f.signal_def["signed"]), (23, "big", True))
    self.assertEqual(f.signal_def["factor"], -0.1)
    self.assertEqual(f.message_def["freq"], 100)
    car, wheel = self._car(7, 0.05)
    car.wheel_deg_fn = lambda: wheel.deg + 1618.4   # the car's count sits at 32368 straight ahead and passes 32767: unsigned
    f = self._find(car, wheel)
    self.assertTrue(f.ok, f.detail)
    self.assertEqual(f.signal_def["signed"], False)
    self.assertEqual(f.signal_def["factor"], 0.05)
    self.assertAlmostEqual(f.signal_def["offset"], -1618.4, delta=0.2)   # reads 0 at rest
    # little-endian
    car, wheel = self._car(7, -0.1)
    car._frame_be = car._frame
    car._frame = lambda addr: car._frame_be(addr) if addr != 0x1C2 else dsc.with_honda_checksum(
      0x1C2, bytes([car._frame_be(0x1C2)[1], car._frame_be(0x1C2)[0]]) + car._frame_be(0x1C2)[2:], 0)
    f = self._find(car, wheel)
    self.assertTrue(f.ok, f.detail)
    self.assertEqual((f.signal_def["start_bit"], f.signal_def["byte_order"]), (0, "little"))
    self.assertEqual(f.signal_def["factor"], -0.1)

  def test_nothing_else_matches(self):
    car = SimCar()   # busy fields (engine speed, a rolling byte, noise), no angle

    class Wheel:
      deg = 0.0
    f = self._find(car, Wheel())
    self.assertFalse(f.ok)
    self.assertIn("not on the car's network", f.detail)

  def test_too_coarse_or_too_slow_is_rejected(self):
    car, wheel = self._car(7, 1.0)      # a whole degree a step: no angle sensor
    self.assertFalse(self._find(car, wheel).ok)
    car, wheel = self._car(7, -0.1, freq=25)
    f = self._find(car, wheel)
    self.assertFalse(f.ok)
    self.assertIn("less than 50 times a second", f.detail)
