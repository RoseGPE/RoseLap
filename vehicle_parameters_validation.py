g = 32.2 # ft/s^2
mass = (550*2)/g #slug

downforce_35mph = 60; # lbf
def alpha_downforce():
    return downforce_35mph/(51.33333**2); # lbf/(ft/s)^2

drag_35mph = 0 # lbf
def alpha_drag():
    return drag_35mph/(51.33333**2); # lbf/(ft/s)^2

mu = 2.0; #coeff
tire_radius = 0.75 #ft

engine_rpms = [1000.0, 3000.0, 9000.0] #rev/min
#engine_hps  = [10.0, 25.0, 45.0, 30.0]         #horsepower
engine_torque = [36.88*0.9, 36.88*2.0, 36.88*0.8] # ft-lbf
engine_reduction = 2.81
gears = [2.416, 1.92, 1.562, 1.277, 1.05]
final_drive_reduction = 36.0/13.0

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
