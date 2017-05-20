g = 32.2 # ft/s^2
mass = (300+150)/g #slug

downforce_35mph = 65; # lbf
alpha_downforce = downforce_35mph/(51.33333**2); # lbf/(ft/s)^2

drag_35mph = 44 # lbf
alpha_drag = drag_35mph/(51.33333**2); # lbf/(ft/s)^2

mu = 1.5; #coeff
tire_radius = 0.75 #ft

engine_rpms = [4500, 6000, 8000, 9000] #rev/min
engine_hps  = [10, 25, 45, 30]         #horsepower
engine_reduction = 2.81
gears = [2.416, 1.92, 1.562, 1.277, 1.05]
final_drive_reduction = 36/13

def eng_force(v,gear):
	eng_output_rpm = v/tire_radius*9.5493*final_drive_reduction
	crank_rpm = eng_output_rpm*engine_reduction*gears[gear]
	if crank_rpm < engine_rpms[0]:
		# cap to lowest hp
		hp = engine_hps[0]
		550*hp/10
		#hp = engine_hps[0] + (crank_rpm-engine_rpms[0])*(engine_hps[1]-engine_hps[0])/(engine_rpms[1]-engine_rpms[0])
	elif crank_rpm > engine_rpms[-1]:
		hp = 0 # simulate hitting the rev limiter
	else:
		for i in reversed(range(1,len(engine_rpms))):
			if crank_rpm<engine_rpms[i]:
				hp = engine_hps[i] + (crank_rpm-engine_rpms[i-1])*(engine_hps[i]-engine_hps[i-1])/(engine_rpms[i]-engine_rpms[i-1])
				break
	if v == 0:
		return 1
	return 550*hp/v;