g = 32.2 # ft/s^2
m = (300.0+150.0)/g #slug

downforce_35mph = 65.0; # lbf
def alpha_downforce():
    return downforce_35mph/(51.33333**2); # lbf/(ft/s)^2

drag_35mph = 44.0 # lbf
def alpha_drag():
    return drag_35mph/(51.33333**2); # lbf/(ft/s)^2


def slip_angle_front():
	return 0;

def slip_angle_rear():
	return 0;

def height_CG():
	return 0.8 # idk

def weight_bias():
	return 0.55 #idk

def downforce_bias():
	return 0.6 # idk

def height_drag():
	return 2 #idk

def mass():
	return m

def CG_dist_from_front():
	return 2.0 # idk

def vehicle_length():
	return 62.0/12.0

def brake_bias():
	return 0.55

mu = 2.0; #coeff
tire_radius = 0.75 #ft

engine_rpms = [4500.0, 6000.0, 8000.0, 9000.0] #rev/min
engine_hps  = [10.0, 25.0, 45.0, 30.0]         #horsepower
engine_reduction = 2.81
gears = [2.416, 1.92, 1.562, 1.277, 1.05]
final_drive_reduction = 36.0/13.0

def eng_force(v,gear):
	eng_output_rpm = v/tire_radius*9.5493*final_drive_reduction
	crank_rpm = eng_output_rpm*engine_reduction*gears[gear]
	if crank_rpm < engine_rpms[0]:
		# cap to lowest hp
		hp = engine_hps[0]
		return 550.0*hp/(engine_rpms[0]/engine_reduction/gears[gear]*tire_radius/9.5493/final_drive_reduction)
		#hp = engine_hps[0] + (crank_rpm-engine_rpms[0])*(engine_hps[1]-engine_hps[0])/(engine_rpms[1]-engine_rpms[0])
	elif crank_rpm > engine_rpms[-1]:
		hp = 0 # simulate hitting the rev limiter
	else:
		for i in reversed(range(1,len(engine_rpms))):
			if crank_rpm<engine_rpms[i]:
				hp = engine_hps[i] + (crank_rpm-engine_rpms[i-1])*(engine_hps[i]-engine_hps[i-1])/(engine_rpms[i]-engine_rpms[i-1])
				break
	return 550.0*hp/v;