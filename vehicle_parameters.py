import json

"""
Vehicle Parameters Schema:
mass: [float]
downforce_35mph: [float]
drag_35mph: [float]
mu: [float]
tire_radius: [float]
engine_rpms: [array]
engine_torque: [float]
engine_reduction: [float]
gears: [array]
final_drive_reduction: [float]
"""

vehicle_JSON = './Vehicles/aero_efficiency_v.json'
g = 32.2 # ft/s^2

with open(vehicle_JSON) as data:
  v_OBJ = json.load(data)

mass = (v_OBJ["mass"] * 2) / g
downforce_35mph = v_OBJ["downforce_35mph"]
drag_35mph = v_OBJ["drag_35mph"]
mu = v_OBJ["mu"]
tire_radius = v_OBJ["tire_radius"]
engine_rpms = v_OBJ["engine_rpms"]
engine_torque = v_OBJ["engine_torque"]
engine_reduction = v_OBJ["engine_reduction"]
gears = v_OBJ["gears"]
final_drive_reduction = v_OBJ["final_drive_reduction"]

def getOriginalVal(name):
  return v_OBJ[name]

def setVar(name, val):
  exec("global " + name + "\n" + name + " = " + str(val)) # jank in preparation for better code structure


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

def alpha_downforce():
    return downforce_35mph/(51.33333**2); # lbf/(ft/s)^2

def alpha_drag():
    return drag_35mph/(51.33333**2); # lbf/(ft/s)^2

def eng_force(v,gear):
  eng_output_rpm = v/tire_radius*9.5493*final_drive_reduction
  crank_rpm = eng_output_rpm*engine_reduction*gears[gear]
  if crank_rpm <= engine_rpms[0]:
    # cap to lowest hp
    return engine_torque[0]*engine_reduction*gears[gear]*final_drive_reduction/tire_radius
    #hp = engine_hps[0] + (crank_rpm-engine_rpms[0])*(engine_hps[1]-engine_hps[0])/(engine_rpms[1]-engine_rpms[0])
  elif crank_rpm > engine_rpms[-1]:
    return 0 # simulate hitting the rev limiter
  else:
    for i in range(1,len(engine_rpms)):
      if crank_rpm<engine_rpms[i]:
        torque = engine_torque[i] + (crank_rpm-engine_rpms[i])*(engine_torque[i-1]-engine_torque[i])/(engine_rpms[i-1]-engine_rpms[i])
        return torque*engine_reduction*gears[gear]*final_drive_reduction/tire_radius
    #return torque*engine_reduction*gears[gear]*final_drive_reduction/tire_radius