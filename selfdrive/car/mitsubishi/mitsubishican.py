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

# this is the old checksum calculator, new one lives in proces_dbc.py
# throttle seems to work with ID at 0x450 because it's defined in DBC as a Pedal. This may need to change.
# TODO: try to use the new calculator with the custom interceptor msgs

def crc8(data):
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


def create_steer_command(packer, trq_command, enable_steer, idx):
  # riped from pedal
  enable = enable_steer

  values = {
    "ENABLE": enable,
    "COUNTER_TRQ": idx & 0xF,
  }

  if enable:
    values["STEER_COMMAND"] = trq_command
    values["STEER_COMMAND2"] = trq_command

  dat = packer.make_can_msg("STEER_TORQUE_COMMAND", 0, values)[2]

  #dat = [ord(i) for i in dat]
  checksum = crc8(dat[:-1])
  values["CHECKSUM_TRQ"] = checksum

  return packer.make_can_msg("STEER_TORQUE_COMMAND", 0, values) #TODO: change this back to bus 0 after tests

def create_brake_command(packer, brake_amount, idx):
  # same as gas command
  enable = brake_amount > 0.001

  values = {
    "ENABLE": enable,
    "COUNTER_BRAKE": idx & 0xF,
  }

  if enable:
    values["BRAKE_COMMAND"] = brake_amount * 255.
    values["BRAKE_COMMAND2"] = 0.

  dat = packer.make_can_msg("BRAKE_COMMAND", 0, values)[2]

  checksum = crc8(dat[:-1])
  values["CHECKSUM_BRAKE"] = checksum

  return packer.make_can_msg("BRAKE_COMMAND", 0, values)

# TODO: create a master PCM_CANCEL that kills all the interceptors
