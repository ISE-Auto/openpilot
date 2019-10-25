from cereal import car
from common.numpy_fast import clip
from selfdrive.car import create_gas_command
from selfdrive.car.mitsubishi.mitsubishican import make_toyota_can_msg, create_steer_command, create_brake_command
from selfdrive.car.mitsubishi.values import ECU, STATIC_MSGS
from selfdrive.can.packer import CANPacker

VisualAlert = car.CarControl.HUDControl.VisualAlert

# Accel limits
ACCEL_HYST_GAP = 0.02  # don't change accel command for small oscilalitons within this value
ACCEL_MAX = 1.5  # 1.5 m/s2
ACCEL_MIN = -3.0 # 3   m/s2
ACCEL_SCALE = max(ACCEL_MAX, -ACCEL_MIN)

# Steer torque limits
class SteerLimitParams:
  STEER_MAX = 1500
  STEER_DELTA_UP = 10       # 1.5s time to peak torque
  STEER_DELTA_DOWN = 25     # always lower than 45 otherwise the Rav4 faults (Prius seems ok with 50)
  STEER_ERROR_MAX = 350     # max delta between torque cmd and torque motor

# Steer angle limits (tested at the Crows Landing track and considered ok)
ANGLE_MAX_BP = [0., 5.]
ANGLE_MAX_V = [510., 300.]
ANGLE_DELTA_BP = [0., 5., 15.]
ANGLE_DELTA_V = [5., .8, .15]     # windup limit
ANGLE_DELTA_VU = [5., 3.5, 0.4]   # unwind limit

# TARGET_IDS = [0x340, 0x341, 0x342, 0x343, 0x344, 0x345,
#               0x363, 0x364, 0x365, 0x370, 0x371, 0x372,
#               0x373, 0x374, 0x375, 0x380, 0x381, 0x382,
#               0x383]


def accel_hysteresis(accel, accel_steady, enabled):

  # for small accel oscillations within ACCEL_HYST_GAP, don't change the accel command
  if not enabled:
    # send 0 when disabled, otherwise acc faults
    accel_steady = 0.
  elif accel > accel_steady + ACCEL_HYST_GAP:
    accel_steady = accel - ACCEL_HYST_GAP
  elif accel < accel_steady - ACCEL_HYST_GAP:
    accel_steady = accel + ACCEL_HYST_GAP
  accel = accel_steady

  return accel, accel_steady


def process_hud_alert(hud_alert):
  # initialize to no alert
  steer = 0
  fcw = 0

  if hud_alert == VisualAlert.fcw:
    fcw = 1
  elif hud_alert == VisualAlert.steerRequired:
    steer = 1

  return steer, fcw


# def ipas_state_transition(steer_angle_enabled, enabled, ipas_active, ipas_reset_counter):

#   if enabled and not steer_angle_enabled:
#     ipas_reset_counter = max(0, ipas_reset_counter - 1)
#     if ipas_reset_counter == 0:
#       steer_angle_enabled = True
#     else:
#       steer_angle_enabled = False
#       return steer_angle_enabled, ipas_reset_counter
#     return True, 0

#   elif enabled and steer_angle_enabled: 
#     if steer_angle_enabled and not ipas_active:
#        ipas_reset_counter += 1  
#     else:
#        ipas_reset_counter = 0 
#     if ipas_reset_counter > 10:  # try every 0.1s
#        steer_angle_enabled = False
#     return steer_angle_enabled, ipas_reset_counter

#   else:
#     return False, 0


class CarController():
  def __init__(self, dbc_name=None, car_fingerprint = "MIEV", enable_camera=1, enable_dsu=1, enable_apg=1):
    self.braking = False
    # redundant safety check with the board
    self.controls_allowed = True
    self.last_steer = 0
    self.last_angle = 0
    self.accel_steady = 0.
    self.car_fingerprint = car_fingerprint
    self.alert_active = False
    self.last_standstill = False
    self.standstill_req = False
    self.angle_control = False
    self.steer_angle_enabled = False
    self.last_fault_frame = -200

    self.fake_ecus = set()
    if enable_dsu: self.fake_ecus.add(ECU.DSU)

    self.packer = CANPacker(dbc_name)

  def update(self, enabled, CS, frame, actuators,
             pcm_cancel_cmd, hud_alert, forwarding_camera, left_line,
             right_line, lead, left_lane_depart, right_lane_depart):
    can_sends = []

    # gas and brake

    apply_gas = clip(actuators.gas, 0., 1.)
    apply_brake = clip(actuators.gas, 0., 1.)

    # steer torque
    apply_steer = int(round(actuators.steer * SteerLimitParams.STEER_MAX))
    
    # TODO: determine actual directions and test on an EPS
    if (apply_steer > 0.0):
        steer_dir = 0x08
    elif (apply_steer < 0.0):
        steer_dir = 0x0F
    else:
        steer_dir = 0x0

    #only cut torque when steer state is a known fault
    if (CS.steer_state >= 1):
     self.last_fault_frame = frame

    #Cut steering for 2s after fault
    if not enabled or (frame - self.last_fault_frame < 200):
      apply_steer = 0
      apply_steer_req = 0
    else:
      apply_steer_req = 1

    #if not enabled and CS.pcm_acc_status:
      # send pcm acc cancel cmd if drive is disabled but pcm is still on, or if the system can't be activated
    #  pcm_cancel_cmd = 1

    self.last_steer = apply_steer
    self.last_standstill = CS.standstill

    can_sends = []

    if (frame % 2 == 0):
      # send exactly zero if apply_gas is zero. Interceptor will send the max between read value and apply_gas.
      # This prevents unexpected pedal range rescaling
      can_sends.append(create_gas_command(self.packer, apply_gas, frame//2))
      can_sends.append(create_brake_command(self.packer, apply_brake, frame//2))
      can_sends.append(create_steer_command(self.packer, apply_steer, apply_steer_req, steer_dir, frame//2))
    
    # ui mesg is at 100Hz but we send asap if:
    # - there is something to display
    # - there is something to stop displaying
    alert_out = process_hud_alert(hud_alert)
    steer, fcw = alert_out

    if (any(alert_out) and not self.alert_active) or \
       (not any(alert_out) and self.alert_active):
      send_ui = True
      self.alert_active = not self.alert_active
    else:
      send_ui = False

    # disengage msg causes a bad fault sound so play a good sound instead
    if pcm_cancel_cmd:
      send_ui = True

    #*** static msgs ***
    for (addr, ecu, cars, bus, fr_step, vl) in STATIC_MSGS:
      if frame % fr_step == 0:
        can_sends.append(make_toyota_can_msg(addr, vl, bus, False))

    return can_sends
