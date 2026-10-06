def stepper_checksum(address: int, sig, d: bytearray) -> int:
  """StepperServoCAN checksum: 16-bit sum of the message id and all data bytes with the
  checksum byte zeroed, then the high byte added to the low byte, keep 8 bits.
  Matches Msg_calc_checksum_8bit() in StepperServoCAN firmware/src/BSP/can.c."""
  s = address
  checksum_byte = sig.start_bit // 8
  for i, b in enumerate(d):
    if i != checksum_byte:
      s += b
  s = (s & 0xFF) + (s >> 8)
  return s & 0xFF
