from selfdrive.car import dbc_dict

class CAR:
  MIEV = "MITSUBISHI MIEV"

class ECU:
#   CAM = 0 # camera
  DSU = 1 # driving support unit
#   APGS = 2 # advanced parking guidance system


# addr: (ecu, cars, bus, 1/freq*100, vl)
STATIC_MSGS = [
  (0x128, ECU.DSU, (CAR.MIEV), 1,   3, b'\xf4\x01\x90\x83\x00\x37'),
  (0x141, ECU.DSU, (CAR.MIEV), 1,   2, b'\x00\x00\x00\x46'),
  (0x160, ECU.DSU, (CAR.MIEV), 1,   7, b'\x00\x00\x08\x12\x01\x31\x9c\x51'),
  (0x161, ECU.DSU, (CAR.MIEV), 1,   7, b'\x00\x1e\x00\x00\x00\x80\x07'),
  (0x344, ECU.DSU, (CAR.MIEV), 2,   5, b'\x00\x00\x01\x00\x00\x00\x00\x50'),
  (0x365, ECU.DSU, (CAR.MIEV), 2,  20, b'\x00\x00\x00\x80\xfc\x00\x08'),
  (0x366, ECU.DSU, (CAR.MIEV), 2,  20, b'\x00\x72\x07\xff\x09\xfe\x00'),
  (0x4CB, ECU.DSU, (CAR.MIEV), 2, 100, b'\x0c\x00\x00\x00\x00\x00\x00\x00'),
]

ECU_FINGERPRINT = {
  ECU.DSU: 0x343,   # accel cmd
}


def check_ecu_msgs(fingerprint, ecu):
  # return True if fingerprint contains messages normally sent by a given ecu
  return ECU_FINGERPRINT[ecu] in fingerprint

FINGERPRINTS = {
  CAR.MIEV: [{
    256:6, 257:6
  }],
}

STEER_THRESHOLD = 0.5

DBC = {
   CAR.MIEV: dbc_dict('toyota_celica_2003_can', 'toyota_adas'),
}
