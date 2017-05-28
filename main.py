from sim import *
import vehicle_parameters as vehicle
track = './DXFs/ax.dxf'

if False:

	#track_segmentation.plot_segments(segs)

	possibilities = np.array([6.5,5.0,2.0,1.0,0.5,0.35,0.3,0.25,0.2,0.15,0.1])
	output = []
	times = np.zeros(len(possibilities))

	for i in range(len(possibilities)):
		#vehicle.final_drive_reduction=possibilities[i]
		segs = track_segmentation.dxf_to_segments(track, possibilities[i])
		output.append(steady_solve(vehicle,segs))
		print('DX=%.2f, took %.3f seconds to travel %.1f feet' % (possibilities[i],output[i][-1,0],output[i][-1,1]) )
		times[i]=output[i][-1,0]

	fig, ax = plt.subplots()
	print(possibilities,times)

	ax.semilogx(possibilities,times,label='Track Time',marker='x',linestyle='-')

	ax.grid(True)
	ax.legend()
	plt.gca().invert_xaxis()
	plt.xlabel('Segment Size')
	plt.ylabel('Track Time')
	plt.draw()

if True:
	segs = track_segmentation.dxf_to_segments(track, 0.25)
	possibilities = np.array([0.0,0.25,0.5,0.7,0.8,0.9,1.0,1.1,1.2,1.3,1.5,2.0,2.5])
	output = []
	times = np.zeros(len(possibilities))

	for i in range(len(possibilities)):
		vehicle.downforce_35mph=65.0 *possibilities[i]
		vehicle.drag_35mph=44.0      *possibilities[i]
		output.append(steady_solve(vehicle,segs))
		print('Scale factor: %.2f, took %.3f seconds to travel %.1f feet' % (possibilities[i],output[i][-1,0],output[i][-1,1]) )
		times[i]=output[i][-1,0]

	fig, ax = plt.subplots()
	print(possibilities,times)

	ax.plot(possibilities,times,label='Track Time',marker='x',linestyle='-')

	ax.grid(True)
	ax.legend()
	#plt.gca().invert_xaxis()
	plt.xlabel('Wing Scaling Factor')
	plt.ylabel('Track Time')
	plt.draw()

if False:
	segs = track_segmentation.dxf_to_segments(track, 0.25)
	possibilities = np.array([0.7,0.8,0.9,1.0,1.1,1.2,1.3,1.4])
	output = []
	times = np.zeros(len(possibilities))

	for i in range(len(possibilities)):
		vehicle.downforce_35mph=65.0 *math.sqrt(possibilities[i])
		vehicle.drag_35mph=44.0      *math.sqrt(1/possibilities[i])
		output.append(steady_solve(vehicle,segs))
		print('Aero Efficiency: %.2f, took %.3f seconds to travel %.1f feet' % (possibilities[i],output[i][-1,0],output[i][-1,1]) )
		times[i]=output[i][-1,0]

	fig, ax = plt.subplots()
	print(possibilities,times)

	ax.plot(possibilities,times,label='Track Time',marker='x',linestyle='-')

	ax.grid(True)
	ax.legend()
	#plt.gca().invert_xaxis()
	plt.xlabel('Wing Efficiency')
	plt.ylabel('Track Time')
	plt.draw()

if False:
	segs = track_segmentation.dxf_to_segments(track, 0.25)
	possibilities = np.array([38.0/13,  36.0/13,  34.0/13,  32.0/13, 30.0/13,  30.0/14, 28.0/14, 26.0/14, 24.0/14])
	output = []
	times = np.zeros(len(possibilities))

	for i in range(len(possibilities)):
		vehicle.final_drive_reduction=possibilities[i]
		output.append(steady_solve(vehicle,segs))
		print('FDR: %.2f, took %.3f seconds to travel %.1f feet' % (possibilities[i],output[i][-1,0],output[i][-1,1]) )
		times[i]=output[i][-1,0]

	fig, ax = plt.subplots()
	print(possibilities,times)

	ax.plot(possibilities,times,label='Track Time',marker='x',linestyle='-')

	ax.grid(True)
	ax.legend()
	#plt.gca().invert_xaxis()
	plt.xlabel('Final Drive Ratio')
	plt.ylabel('Track Time')
	plt.draw()

if False:
	segs = track_segmentation.dxf_to_segments(track, 0.25)
	possibilities = np.array([[10.0, 25.0, 45.0, 30.0],[10.0, 25.0, 35.0, 45.0],[10.0, 25.0, 40.0, 40.0]])

	output = []
	times = np.zeros(len(possibilities))

	for i in range(len(possibilities)):
		vehicle.engine_hps=possibilities[i]
		output.append(steady_solve(vehicle,segs))
		print('Curve: %.2f, took %.3f seconds to travel %.1f feet' % (i,output[i][-1,0],output[i][-1,1]) )
		plot_velocity_and_events(output[i],'time')
		times[i]=output[i][-1,0]

	fig, ax = plt.subplots()
	print(possibilities,times)

	ax.plot(possibilities,times,label='Track Time',marker='x',linestyle='-')

	ax.grid(True)
	ax.legend()
	#plt.gca().invert_xaxis()
	plt.xlabel('Final Drive Ratio')
	plt.ylabel('Track Time')
	plt.draw()

if False:

	segs = track_segmentation.dxf_to_segments(track, 0.25)
	track_segmentation.plot_segments(segs)
	output = steady_solve(vehicle,segs)
	print('Took %.3f seconds to travel %.1f feet' % (output[-1,0],output[-1,1]) )


	plot_velocity_and_events(output,'time')

#print(output[:,5])

plt.show()