from cereal import car
from common.numpy_fast import clip
from selfdrive.car import apply_toyota_steer_torque_limits, create_gas_command, make_can_msg
from selfdrive.car.mitsubishi.mitsubishican import create_steer_command, create_brake_command
from selfdrive.car.mitsubishi.values import ECU, STATIC_MSGS
from opendbc.can.packer import CANPacker

VisualAlert = car.CarControl.HUDControl.VisualAlert

# Accel limits
ACCEL_HYST_GAP = 0.02  # don't change accel command for small oscilalitons within this value
ACCEL_MAX = 1.5  # 1.5 m/s2
ACCEL_MIN = -3.0 # 3   m/s2
ACCEL_SCALE = max(ACCEL_MAX, -ACCEL_MIN)

# Steer torque limits
class SteerLimitParams:
  STEER_MAX = 1500
  STEER_DELTA_UP = 25       # 1.5s time to peak torque
  STEER_DELTA_DOWN = 50     # always lower than 45 otherwise the Rav4 faults (Prius seems ok with 50)
  STEER_ERROR_MAX = 600     # max delta between torque cmd and torque motor

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
    #self.last_standstill = False
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

    # gas and brake

    apply_gas = clip(actuators.gas, 0., 1.)
    apply_brake = clip(actuators.brake, 0., 1.)

    # steer torque
    new_steer = int(round(actuators.steer * SteerLimitParams.STEER_MAX))
    apply_steer = apply_toyota_steer_torque_limits(new_steer, self.last_steer, CS.steer_torque_motor, SteerLimitParams)
    self.steer_rate_limited = new_steer != apply_steer

    # only cut torque when steer state is a known fault
    if CS.steer_state in [9, 25]:
      self.last_fault_frame = frame

    # Cut steering for 2s after fault
    if not enabled or (frame - self.last_fault_frame < 200):
      apply_steer = 0
      apply_steer_req = 0
    else:
      apply_steer_req = 1

    self.last_steer = apply_steer

    can_sends = []

    if (frame % 2 == 0):
      # send exactly zero if apply_gas is zero. Interceptor will send the max between read value and apply_gas.
      # This prevents unexpected pedal range rescaling
      can_sends.append(create_gas_command(self.packer, apply_gas, frame//2))
      can_sends.append(create_brake_command(self.packer, apply_brake, frame//2))
      can_sends.append(create_steer_command(self.packer, apply_steer, apply_steer_req, frame//2))

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
    for (addr, bus, fr_step, vl) in STATIC_MSGS:
      if frame % fr_step == 0:
        can_sends.append(make_can_msg(addr, vl, bus))

    return can_sends
