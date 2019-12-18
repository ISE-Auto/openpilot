from cereal import car
from common.numpy_fast import mean
from common.kalman.simple_kalman import KF1D
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from selfdrive.config import Conversions as CV
from selfdrive.car.mitsubishi.values import DBC, STEER_THRESHOLD #, NO_DSU_CAR, CAR, 
# from selfdrive.udp.udpserver import Server

def parse_gear_shifter(gear, vals):
  return 'drive'


def get_can_parser(CP):

  signals = [
    # sig_name, sig_address, default
    ("STEER_ANGLE", "STEER_ANGLE_SENSOR", 0),
    ("STEER_RATE", "STEER_ANGLE_SENSOR", 0),
    ("GEAR", "GEAR_PACKET", 0),
    ("SPEED", "COMBOMETER", 0),
    ("RPM", "MOTOR", 0),
    ("INTERCEPTOR_TRQ", "STEER_SENSOR", 0),
    ("INTERCEPTOR_TRQ2", "STEER_SENSOR", 0),
    ("STATE", "STEER_SENSOR", 0),
    ("INTERCEPTOR_GAS", "GAS_SENSOR", 0),
    ("INTERCEPTOR_GAS2", "GAS_SENSOR", 0),
    ("STATE", "GAS_SENSOR", 0),
    ("INTERCEPTOR_BRAKE", "BRAKE_SENSOR", 0),
    ("STATE", "BRAKE_SENSOR", 0),
    ("WHEEL_SPEED_FL", "ABS_FRONT", 0),
    ("WHEEL_SPEED_FR", "ABS_FRONT", 0),
    ("BRAKE_PEDAL", "ABS_REAR", 0),
    ("WHEEL_SPEED_RL", "ABS_REAR", 0),
    ("WHEEL_SPEED_RR", "ABS_REAR", 0),
  ]

  checks = [
    ("GAS_SENSOR", 50),
    ("STEER_SENSOR", 50),
    ("BRAKE_SENSOR", 50),
    #("STEER_ANGLE_SENSOR", 100),
  ]

  return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)

class CarState():
  def __init__(self, CP):

    self.CP = CP
    self.can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    print("DBC PARSED: %s" % DBC[CP.carFingerprint]['pt'])
    self.shifter_values = 'D' #self.can_define.dv["GEAR_PACKET"]['GEAR']

    self.ENGAGED = False

     # initialize can parser
    self.car_fingerprint = CP.carFingerprint

    # vEgo kalman filter
    dt = 0.01
    # Q = np.matrix([[10.0, 0.0], [0.0, 100.0]])
    # R = 1e3
    self.v_ego_kf = KF1D(x0=[[0.0], [0.0]],
                         A=[[1.0, dt], [0.0, 1.0]],
                         C=[1.0, 0.0],
                         K=[[0.12287673], [0.29666309]])
    self.v_ego = 0.0
    self.pcm_acc_status = 0
    self.pcm_acc_active = False
    self.v_cruise_pcm = 0

  def update(self, cp):
    # update prevs, update must run once per loop
#     self.prev_left_blinker_on = self.left_blinker_on
#     self.prev_right_blinker_on = self.right_blinker_on

    self.door_all_closed = True #not any([cp.vl["SEATS_DOORS"]['DOOR_OPEN_FL'], cp.vl["SEATS_DOORS"]['DOOR_OPEN_FR'],
#                                     cp.vl["SEATS_DOORS"]['DOOR_OPEN_RL'], cp.vl["SEATS_DOORS"]['DOOR_OPEN_RR']])
    self.seatbelt = True
    self.brake_pressed = (cp.vl["BRAKE_SENSOR"]['INTERCEPTOR_BRAKE'] > 200) #TODO - brake pressed value
#     if self.CP.enableGasInterceptor:
#       self.pedal_gas = (cp.vl["GAS_SENSOR"]['INTERCEPTOR_GAS'] + cp.vl["GAS_SENSOR"]['INTERCEPTOR_GAS2']) / 2.
#     else:
    self.pedal_gas = 0 #cp.vl["GAS_PEDAL"]['GAS_PEDAL']
    self.car_gas = 0
    self.esp_disabled = False
    # # calc best v_ego estimate, by averaging two opposite corners
    self.v_wheel_fl = cp.vl["ABS_FRONT"]['WHEEL_SPEED_FL'] * CV.KPH_TO_MS
    self.v_wheel_fr = cp.vl["ABS_FRONT"]['WHEEL_SPEED_FR'] * CV.KPH_TO_MS
    self.v_wheel_rl = cp.vl["ABS_REAR"]['WHEEL_SPEED_RL'] * CV.KPH_TO_MS
    self.v_wheel_rr = cp.vl["ABS_REAR"]['WHEEL_SPEED_RR'] * CV.KPH_TO_MS
    v_wheel = mean([self.v_wheel_fl, self.v_wheel_fr, self.v_wheel_rl, self.v_wheel_rr])

    # Kalman filter
    if abs(v_wheel - self.v_ego) > 2.0:  # Prevent large accelerations when car starts at non zero speed
      self.v_ego_kf.x = [[v_wheel], [0.0]]

    self.v_ego_raw = v_wheel
    v_ego_x = self.v_ego_kf.update(v_wheel)
    self.v_ego = float(v_ego_x[0])
    self.a_ego = float(v_ego_x[1])
    self.standstill = not v_wheel > 0.001

    self.angle_steers = cp.vl["STEER_ANGLE_SENSOR"]['STEER_ANGLE']
    self.angle_steers_rate = cp.vl["STEER_ANGLE_SENSOR"]['STEER_RATE']
    self.gear_shifter = 'drive' #parse_gear_shifter(can_gear, self.shifter_values)
    self.main_on = True #cp.vl["PCM_CRUISE_2"]['MAIN_ON']
#     self.left_blinker_on = cp.vl["STEERING_LEVERS"]['TURN_SIGNALS'] == 1
#     self.right_blinker_on = cp.vl["STEERING_LEVERS"]['TURN_SIGNALS'] == 2

#     # 2 is standby, 10 is active. TODO: check that everything else is really a faulty state
    self.steer_state = cp.vl["STEER_SENSOR"]['STATE']
    self.steer_error = cp.vl["STEER_SENSOR"]['STATE'] not in [0]
    self.ipas_active = False
    self.brake_error = 0
    self.steer_torque_driver = (cp.vl["STEER_SENSOR"]['INTERCEPTOR_TRQ'] - 0x060A) #TODO: - integrate into FW
    self.steer_torque_motor = 0 #TODO: find an EPS torque amount
#     # we could use the override bit from dbc, but it's triggered at too high torque values
    self.steer_override = abs(self.steer_torque_driver) > STEER_THRESHOLD

    self.user_brake = 0

    #cruise control
    self.pcm_acc_active = not self.brake_pressed

    if not self.pcm_acc_active:
      self.pcm_acc_status = not self.user_brake #cruise activates as soon as user stops pressing the brake
      self.v_cruise_pcm = cp.vl["COMBOMETER"]["SPEED"]
    else:
      self.pcm_acc_status = 8 # cp.vl["PCM_CRUISE"]['CRUISE_STATE'] # 8
    
    self.brake_lights = False #bool(cp.vl["ESP_CONTROL"]['BRAKE_LIGHTS_ACC'] or self.brake_pressed)
   
#thos is the old network conection stuff
    # try:
    #   data, addr = self.sock.recvfrom(1) # buffer size is 1 byte and unpacks to bool
    #   self.pcm_acc_active = (bool(struct.unpack('b', data)[0]))
    # except Exception as e:
    #   print e
    #   pass
