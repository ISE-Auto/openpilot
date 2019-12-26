#!/usr/bin/env python3
# from common.realtime import sec_since_boot
from cereal import car
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.drive_helpers import EventTypes as ET, create_event
from selfdrive.controls.lib.vehicle_model import VehicleModel
from selfdrive.car.mitsubishi.carstate import CarState, get_can_parser
from selfdrive.car.mitsubishi.values import ECU, check_ecu_msgs, CAR
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, is_ecu_disconnected, gen_empty_fingerprint
from selfdrive.swaglog import cloudlog
from selfdrive.car.interfaces import CarInterfaceBase
import cereal.messaging as messaging

# lifted from mock car because no speed sensor.. TODO: add speed sensor
# mocked car interface to work with chffrplus
TS = 0.01  # 100Hz
YAW_FR = 0.2 # ~0.8s time constant on yaw rate filter
# low pass gain
LPG = 2 * 3.1415 * YAW_FR * TS / (1 + 2 * 3.1415 * YAW_FR * TS)


class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController):
    self.CP = CP
    self.VM = VehicleModel(CP)

    self.frame = 0
    self.gas_pressed_prev = False
    self.brake_pressed_prev = False
    self.cruise_enabled_prev = False

    # *** init the major players ***
    self.CS = CarState(CP)

    self.cp = get_can_parser(CP)
    #failing here

    self.forwarding_camera = False

    self.CC = None
    if CarController is not None:
      self.CC = CarController(self.cp.dbc_name, CP.carFingerprint, CP.enableCamera, CP.enableDsu, CP.enableApgs)

    self.sensor = messaging.sub_sock('sensorEvents')
    self.gps = messaging.sub_sock('gpsLocation')

    self.speed = 0.
    self.prev_speed = 0.
    self.yaw_rate = 0.
    self.yaw_rate_meas = 0.
    
    self.cruisespeed = 0.

  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 3.0

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), vin="", has_relay=False):
    print("CarParams new message")
    ret = car.CarParams.new_message()
    print("CarParams setting fingerprint and safety")
    ret.carName = "mitsubishi"
    ret.carFingerprint = candidate
    ret.carVin = vin
    ret.isPandaBlack = False
    
    ret.safetyModel = car.CarParams.SafetyModel.toyota
    print("CarParams done settng safety model")
    # # pedal
    ret.enableCruise = True #not ret.enableGasInterceptor

    print("CarParams setting car tuning")

    ret.steerActuatorDelay = 0.1  # Default delay
    ret.lateralTuning.init('pid')   
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
    
    print("CarParams set up PID") 
    
    stop_and_go = True
    ret.safetyParam = 100
    ret.wheelbase = 2.55
    ret.steerRatio = 17.
    tire_stiffness_factor = 0.444
    ret.mass = 1080. + STD_CARGO_KG
    ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.01], [0.005]]
    ret.lateralTuning.pid.kf = 0.001388889 # 1 / max angle
    
    print("CarParams done setting tuning")
    
    ret.steerRateCost = 1.0
    print("CarParams steerRateCost set") #% ret.steerRateCost
    ret.centerToFront = ret.wheelbase * 0.44
    print("CarParams centerToFront set") #% ret.centerToFront
    ret.enableGasInterceptor = True
    print("CarParams enableGasInterceptor set to True")

    ret.minEnableSpeed = -1.
    print("CarParams done setting ratecost and min enable speed")

    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                          tire_stiffness_factor=tire_stiffness_factor)
    ret.steerRatioRear = 0.
    ret.steerControlType = car.CarParams.SteerControlType.angle

    # steer, gas, brake limitations VS speed
    ret.steerMaxBP = [16. * CV.KPH_TO_MS, 45. * CV.KPH_TO_MS]  # breakpoints at 1 and 40 kph
    ret.steerMaxV = [1., 1.]  # 2/3rd torque allowed above 45 kph
    ret.brakeMaxBP = [0.]
    ret.brakeMaxV = [1.]
    
    ret.enableCamera = True #not check_ecu_msgs(fingerprint, ECU.CAM) or is_panda_black
    ret.enableDsu = True #not check_ecu_msgs(fingerprint, ECU.DSU)
    ret.enableApgs = False #not check_ecu_msgs(fingerprint, ECU.APGS)
    ret.openpilotLongitudinalControl = True #ret.enableCamera and ret.enableDsu
    cloudlog.warn("CarParams ECU DSU Simulated: %r", ret.enableDsu)

    ret.steerLimitAlert = False

    ret.longitudinalTuning.deadzoneBP = [0., 9.]
    ret.longitudinalTuning.deadzoneV = [0., .15]
    ret.longitudinalTuning.kpBP = [0., 5., 35.]
    ret.longitudinalTuning.kiBP = [0., 35.]
    ret.stoppingControl = False
    ret.startAccel = 0.0

    ret.gasMaxBP = [0., 9., 35]
    ret.gasMaxV = [0.2, 0.5, 0.7]
    ret.longitudinalTuning.kpV = [1.2, 0.8, 0.5]
    ret.longitudinalTuning.kiV = [0.18, 0.12]
    return ret

  # returns a car.CarState
  def update(self, c, can_strings):
    # ******************* do can recv *******************
    self.cp.update_strings(can_strings)


    self.CS.update(self.cp)

  # get basic data from phone and gps since no wheel speeds
  # TODO: speed sensor from VSS

    sensors = messaging.recv_sock(self.sensor)
    if sensors is not None:
      for sensor in sensors.sensorEvents:
        if sensor.type == 4:  # gyro
          self.yaw_rate_meas = -sensor.gyro.v[0]

    gps = messaging.recv_sock(self.gps)
    if gps is not None:
      self.prev_speed = self.speed
      self.speed = gps.gpsLocation.speed

    # create message
    ret = car.CarState.new_message()
    ret.canValid = self.cp.can_valid

    # speeds
    ret.vEgo = self.speed #self.CS.v_ego
    ret.vEgoRaw = self.speed #self.CS.v_ego_raw
    
    a = self.speed - self.prev_speed
    ret.aEgo = a

    self.yawRate = LPG * self.yaw_rate_meas + (1. - LPG) * self.yaw_rate
    ret.yawRate = self.yawRate #self.VM.yaw_rate(self.CS.angle_steers * CV.DEG_TO_RAD, self.CS.v_ego)
    ret.standstill = self.speed < 0.01 #self.CS.standstill

    # gear shifter
    ret.gearShifter = self.CS.gear_shifter

    # gas pedal
    ret.gas = self.CS.car_gas
    ret.gasPressed = self.CS.pedal_gas > 50

    # brake pedal
    ret.brake = self.CS.user_brake
    ret.brakePressed = self.CS.brake_pressed != 0
    ret.brakeLights = False #self.CS.brake_lights

    # steering wheel
    ret.steeringAngle = self.CS.angle_steers
    ret.steeringRate = self.CS.angle_steers_rate

    ret.steeringTorque = self.CS.steer_torque_driver
    ret.steeringPressed = self.CS.steer_override

    # cruise state
    if not self.CS.pcm_acc_active:
      self.cruisespeed = self.speed
    ret.cruiseState.enabled = self.CS.pcm_acc_active
    ret.cruiseState.speed = self.cruisespeed + self.CS.speed_offset #self.CS.v_cruise_pcm * CV.KPH_TO_MS
    ret.cruiseState.available = bool(self.CS.main_on)
    ret.cruiseState.speedOffset = 0.
    ret.cruiseState.standstill = False
    ret.doorOpen = False #not self.CS.door_all_closed
    ret.seatbeltUnlatched = False #not self.CS.seatbelt
  
    # events
    events = []

    if not ret.gearShifter == 'drive' and self.CP.enableDsu:
      events.append(create_event('wrongGear', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    if ret.doorOpen:
      events.append(create_event('doorOpen', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    if ret.seatbeltUnlatched:
      events.append(create_event('seatbeltNotLatched', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    # if self.CS.esp_disabled and self.CP.enableDsu:
    #   events.append(create_event('espDisabled', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    # if not self.CS.main_on and self.CP.enableDsu:
    #   events.append(create_event('wrongCarMode', [ET.NO_ENTRY, ET.USER_DISABLE]))
    if ret.gearShifter == 'reverse' and self.CP.enableDsu:
      events.append(create_event('reverseGear', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE]))
    if self.CS.steer_error:
      events.append(create_event('steerTempUnavailable', [ET.NO_ENTRY, ET.WARNING]))
    # if ret.vEgo < self.CP.minEnableSpeed and self.CP.enableDsu:
    #   events.append(create_event('speedTooLow', [ET.NO_ENTRY]))
    #   if ret.vEgo < 0.001:
    #     # while in standstill, send a user alert
    #     events.append(create_event('manualRestart', [ET.WARNING]))

    ret.events = events

    self.gas_pressed_prev = ret.gasPressed
    self.brake_pressed_prev = ret.brakePressed
    self.cruise_enabled_prev = ret.cruiseState.enabled
    return ret.as_reader()

  # pass in a car.CarControl
  # to be called @ 100hz
  def apply(self, c):

    can_sends = self.CC.update(c.enabled, self.CS, self.frame,
                              c.actuators, c.cruiseControl.cancel, c.hudControl.visualAlert,
                              self.forwarding_camera, c.hudControl.leftLaneVisible,
                              c.hudControl.rightLaneVisible, c.hudControl.leadVisible,
                              c.hudControl.leftLaneDepart, c.hudControl.rightLaneDepart)

    self.frame += 1
    return can_sends
