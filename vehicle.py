import fancyyaml as yaml
import numpy as np

"""
Vehicle Parameters Schema:
mass: [float]
downforce_35mph: [float]
drag_35mph: [float]
mu: [float]
tire_radius: [float]
engine_rpms: [float-array]
engine_torque: [float-array]
engine_reduction: [float]
gears: [float-array]
final_drive_reduction: [float]
shift_time: [float]
"""

class Vehicle(object):
  def alpha_downforce(self):
    return v.downforce_35mph / (51.33333 ** 2); # lbf/(ft/s)^2

  def alpha_drag(self):
    return v.drag_35mph / (51.33333 ** 2); # lbf/(ft/s)^2

  def front_brake_bias(self):
    return v.brake_bias

  def rear_brake_bias(self):
    return 1-v.brake_bias

  def eng_force(self, vel, gear):
    gear_ratio = v.gears[gear]
    eng_output_rpm = vel / v.tire_radius * 9.5493 * v.final_drive_reduction
    crank_rpm = eng_output_rpm * v.engine_reduction * gear_ratio

    if crank_rpm <= v.engine_rpms[0]:
      # cap to lowest hp
      return (v.engine_torque[0] * v.engine_reduction * gear_ratio * v.final_drive_reduction / v.tire_radius, crank_rpm)
    elif crank_rpm > v.engine_rpms[-1]:
      return (0,crank_rpm) # simulate hitting the rev limiter
    else:
      for i in range(1, len(v.engine_rpms)):
        if crank_rpm < v.engine_rpms[i]:
          torque = v.engine_torque[i] + (crank_rpm - v.engine_rpms[i]) * (v.engine_torque[i-1] - v.engine_torque[i]) / (v.engine_rpms[i-1] - v.engine_rpms[i])
          return (torque * v.engine_reduction * gear_ratio * v.final_drive_reduction / v.tire_radius, crank_rpm)

  def best_gear(self, v, fr_limit):
    opts = [min(self.eng_force(v, int(gear))[0], fr_limit) for gear in range(len(self.gears))]
    best = 0
    besti = -1
    for i in range(len(opts)):
      if opts[i] >= best:
        best = opts[i]
        besti = i
    return besti



  def __init__(self):
    pass

v = Vehicle()
g = 32.2 # ft/s^2

def load(filename):
  vehicle_YAML = './Vehicles/' + filename
  with open(vehicle_YAML) as data:
    v_OBJ = yaml.load(data)

  for key in v_OBJ:
    if isinstance(v_OBJ[key], int):
      v_OBJ[key] = float(v_OBJ[key])
    setattr(v, key, v_OBJ[key])
  v.mass /= g
  v.g = g

  v.v_OBJ = v_OBJ

  # TODO: be smart about this (setattr)

def getOriginalVal(name):
  return v.v_OBJ[name]

def setVar(name, val):
  exec("v." + name + " = " + str(val)) # jank in preparation for even better code structure


# mass = (550*2)/g #slug

# downforce_35mph = 60; # lbf
# drag_35mph = 0 # lbf

# mu = 2.0; #coeff
# tire_radius = 0.75 #ft

# engine_rpms = [1000.0, 3000.0, 9000.0] #rev/min
# engine_torque = [36.88*0.9, 36.88*2.0, 36.88*0.8] # ft-lbf
# engine_reduction = 2.81
# gears = [2.416, 1.92, 1.562, 1.277, 1.05]
# final_drive_reduction = 36.0/13.0