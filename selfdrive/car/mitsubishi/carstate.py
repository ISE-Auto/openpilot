# from cereal import car
# from common.numpy_fast import mean
from common.kalman.simple_kalman import KF1D
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
# from selfdrive.config import Conversions as CV
from selfdrive.car.mitsubishi.values import DBC, STEER_THRESHOLD #, NO_DSU_CAR, CAR, 
# from selfdrive.udp.udpserver import Server

def parse_gear_shifter(gear, vals):
  return 'drive'


def get_can_parser(CP):

  signals = [
    # sig_name, sig_address, default
    ("CRUISE_BUTTONS", "CRUISE_STALK", 0),
    ("STEER_ANGLE", "STEER_ANGLE_SENSOR", 0),
    ("STEER_RATE", "STEER_ANGLE_SENSOR", 0),
    ("INTERCEPTOR_TRQ", "STEER_SENSOR", 0),
    ("INTERCEPTOR_TRQ2", "STEER_SENSOR", 0),
    ("STATE", "STEER_SENSOR", 0),
    ("INTERCEPTOR_GAS", "GAS_SENSOR", 0),
    ("INTERCEPTOR_GAS2", "GAS_SENSOR", 0),
    ("STATE", "GAS_SENSOR", 0),
    ("INTERCEPTOR_BRAKE", "BRAKE_SENSOR", 0),
    ("STATE", "BRAKE_SENSOR", 0),
    ("STEER_TORQUE_EPS", "STEER_TORQUE_SENSOR", 0)
  ]

  checks = [
    ("GAS_SENSOR", 50),
    ("STEER_SENSOR", 50),
    ("BRAKE_SENSOR", 50),
    ("STEER_ANGLE_SENSOR", 50),
  ]

  return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)

class CarState():
  def __init__(self, CP):

    self.CP = CP
    self.can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    print("DBC PARSED: %s" % DBC[CP.carFingerprint]['pt'])
    self.shifter_values = 'D' #self.can_define.dv["GEAR_PACKET"]['GEAR']

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
    self.main_on = False
    self.pcm_acc_active = False
    self.pcm_acc_status = 0
    self.v_cruise_pcm = 0
    self.speed_offset = 0
    self.cruise_button_state_last = 0

  def update(self, cp):
    # update prevs, update must run once per loop
#     self.prev_left_blinker_on = self.left_blinker_on
#     self.prev_right_blinker_on = self.right_blinker_on

    self.door_all_closed = True #not any([cp.vl["SEATS_DOORS"]['DOOR_OPEN_FL'], cp.vl["SEATS_DOORS"]['DOOR_OPEN_FR'],
#                                     cp.vl["SEATS_DOORS"]['DOOR_OPEN_RL'], cp.vl["SEATS_DOORS"]['DOOR_OPEN_RR']])
    self.seatbelt = True
    self.brake_pressed = (cp.vl["BRAKE_SENSOR"]['INTERCEPTOR_BRAKE'] > 75)
    self.pedal_gas = cp.vl["GAS_SENSOR"]['INTERCEPTOR_GAS']
    self.car_gas = self.pedal_gas
    self.esp_disabled = False
    
    # # calc best v_ego estimate, by averaging two opposite corners
    # self.v_wheel_fl = cp.vl["ABS_FRONT"]['WHEEL_SPEED_FL'] * CV.KPH_TO_MS
    # self.v_wheel_fr = cp.vl["ABS_FRONT"]['WHEEL_SPEED_FR'] * CV.KPH_TO_MS
    # self.v_wheel_rl = cp.vl["ABS_REAR"]['WHEEL_SPEED_RL'] * CV.KPH_TO_MS
    # self.v_wheel_rr = cp.vl["ABS_REAR"]['WHEEL_SPEED_RR'] * CV.KPH_TO_MS
    # v_wheel = mean([self.v_wheel_fl, self.v_wheel_fr, self.v_wheel_rl, self.v_wheel_rr])

    # Kalman filter
    # if abs(v_wheel - self.v_ego) > 2.0:  # Prevent large accelerations when car starts at non zero speed
    #   self.v_ego_kf.x = [[v_wheel], [0.0]]

    # self.v_ego_raw = v_wheel
    # v_ego_x = self.v_ego_kf.update(v_wheel)
    # self.v_ego = float(v_ego_x[0])
    # self.a_ego = float(v_ego_x[1])
    # self.standstill = not v_wheel > 0.001

    self.angle_steers = cp.vl["STEER_ANGLE_SENSOR"]['STEER_ANGLE']
    self.angle_steers_rate = cp.vl["STEER_ANGLE_SENSOR"]['STEER_RATE']
    self.gear_shifter = 'drive' #parse_gear_shifter(can_gear, self.shifter_values)

#     # 2 is standby, 10 is active. TODO: check that everything else is really a faulty state
    self.steer_state = cp.vl["STEER_SENSOR"]['STATE']
    self.steer_error = cp.vl["STEER_SENSOR"]['STATE'] not in [0]
    self.ipas_active = False
    self.brake_error = 0
    self.steer_torque_driver = (cp.vl["STEER_SENSOR"]['INTERCEPTOR_TRQ'] - cp.vl['STEER_SENSOR']['INTERCEPTOR_TRQ2']) * 256
    self.steer_torque_motor = cp.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_EPS"]
    # print("motor:", self.steer_torque_motor, "driver:", self.steer_torque_driver)
    # TODO: check this math
    # we could use the override bit from dbc, but it's triggered at too high torque values
    self.steer_override = abs(self.steer_torque_driver) > STEER_THRESHOLD

    self.user_brake = 0

    # cruise control
    cruise_button_state = cp.vl["CRUISE_STALK"]['CRUISE_BUTTONS']
    # self.pcm_acc_active = (not self.brake_pressed) and (self.pedal_gas < 50)

    # only change states if button state changed
    # welcome to IF hell. TODO: take some time and do this better
    if cruise_button_state != self.cruise_button_state_last:
      if self.main_on:
        # only change these other states if main_on AND the state changed.
        if cruise_button_state == 0x0a:
          if self.pcm_acc_active: 
            self.speed_offset -= 1.
          else:
            self.pcm_acc_active = True
            self.pcm_acc_status = 1
        elif cruise_button_state == 0x0e:
          if self.pcm_acc_active:
            self.speed_offset += 1.
          elif self.pcm_acc_status == 1:
            self.pcm_acc_active = True       
        elif cruise_button_state == 0x07:
          self.pcm_acc_active = False

      if cruise_button_state == 0x0f:
        self.main_on = not self.main_on

    if not self.main_on:
      self.speed_offset = 0
      self.pcm_acc_status = 0
    self.cruise_button_state_last = cruise_button_state

    # if not self.pcm_acc_active:
    #   self.pcm_acc_status = not self.user_brake #cruise activates as soon as user stops pressing the brake
    #   self.v_cruise_pcm = cp.vl["COMBOMETER"]["SPEED"]
    # else:
    #   self.pcm_acc_status = 8 # cp.vl["PCM_CRUISE"]['CRUISE_STATE'] # 8
    
    self.brake_lights = False #bool(cp.vl["ESP_CONTROL"]['BRAKE_LIGHTS_ACC'] or self.brake_pressed)
