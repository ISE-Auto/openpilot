import struct

def toyota_fix(msg, addr):
  checksum = 0
  idh = (addr & 0xff00) >> 8
  idl = (addr & 0xff)

  checksum = idh + idl + len(msg) + 1
  for d_byte in msg:
    checksum += d_byte

  #return msg + chr(checksum & 0xFF)
  return msg + struct.pack("B", checksum & 0xFF)


def make_toyota_can_msg(addr, dat, alt, cks=False):
  if cks:
    dat = fix(dat, addr)
  return [addr, 0, dat, alt]


def crc8_trq(data):
  crc = 0xFF    # standard init value
  poly = 0xD5   # standard crc8: x8+x7+x6+x4+x2+1
  size = len(data)
  for i in range(size-1, -1, -1):
    crc ^= data[i]
    for j in range(8):
      if ((crc & 0x80) != 0):
        crc = ((crc << 1) ^ poly) & 0xFF
      else:
        crc <<= 1
  return crc


def create_steer_command(packer, trq_command, enable_steer, direction, idx):
  # riped from pedal
  enable = enable_steer

  values = {
    "ENABLE": enable,
    "COUNTER_TRQ": idx & 0xF,
  }

  if enable:
    values["STEER_COMMAND"] = trq_command
    values["DIRECTION"] = direction

  dat = packer.make_can_msg("STEER_TORQUE_COMMAND", 2, values)[2]

  #dat = [ord(i) for i in dat]
  checksum = crc8_trq(dat[:-1])
  values["CHECKSUM_TRQ"] = checksum

  return packer.make_can_msg("STEER_TORQUE_COMMAND", 2, values)
