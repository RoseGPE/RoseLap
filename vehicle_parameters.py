g = 32.2 # ft/s^2
mass = 350/g #slug

downforce_35mph = 0; # lbf
alpha_downforce = 0; # lbf/(ft/s)^2

mu = 1.5; #coeff
tire_radius = 0.75 #ft

engine_rpms = [4500, 6000, 8000, 9000] #rev/min
engine_hps  = [10, 30, 45, 40]         #horsepower
engine_reduction = 2.81
#gears = [2.416, 1.92, 1.562, 1.277, 1.05]
final_drive_reduction = 32/13

def eng_force(v):
	eng_output_rpm = v/tire_radius*final_drive_reduction
	crank_rpm = eng_output_rpm*engine_reduction
	if crank_rpm < engine_rpms[0]:
		# cap to lowest hp
		hp = engine_hps[0]
		#hp = engine_hps[0] + (crank_rpm-engine_rpms[0])*(engine_hps[1]-engine_hps[0])/(engine_rpms[1]-engine_rpms[0])
	elif crank_rpm > engine_rpms[-1]:
		hp = 0 # simulate hitting the rev limiter
	else:
		for i in range(1,len(engine_rpms)):
			if crank_rpm>engine_rpms[i]:
				hp = engine_hps[i] + (crank_rpm-engine_rpms[i-1])*(engine_hps[i]-engine_hps[i-1])/(engine_rpms[i]-engine_rpms[i-1])
				break
	if v == 0:
		return 1
	return 550*hp/v;