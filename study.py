from sim import *
import vehicle
import track_segmentation
import json

"""
Study Schema:
vehicle: [string] filename for .json in the Vehicles directory
track: [string] filename for .dxf in the DXFs directory
segment_distance: [float-array]
tests: [object-array]
	target: [string] vehicle parameter to alter
	operation: 'replace' || 'product' || 'inverse-product'
	test_vals: [float-array]
tests2: [object-array] (optional)
	target: [string] vehicle parameter to alter
	operation: 'replace' || 'product' || 'inverse-product'
	test_vals: [float-array]
plot_style: 'semilog' || 'basic' || 'heatmap' || 'bars'
plot_title: [string]
plot_x_label: [string]
plot_y_label: [string]
plot_points: [float-array]
"""

print("Loading test...")

# load the study JSON into s_OBJ
study_JSON = './Studies/shift_vs_mu_s.json'
with open(study_JSON) as data:
  s_OBJ = json.load(data)

# load vehicle
vehicle.load(s_OBJ["vehicle"])

print("Setting up tests...")

# set up track
tracks = s_OBJ["track"]
meshes = s_OBJ["segment_distance"]
segList = [track_segmentation.dxf_to_segments("./DXFs/" + tracks[i], meshes[i]) for i in range(len(tracks))]

# # the following lines were lost under the sweeping branch of the new regime
# tests = np.array(s_OBJ["test_points"])
# test_op = s_OBJ["test_operation"]

# set up tests
tests = s_OBJ["tests"]
targets = [tests[x]["target"] for x in range(len(tests))]
operations = [tests[x]["operation"] for x in range(len(tests))]
test_points = [tests[x]["test_vals"] for x in range(len(tests))]

try: # run 2D test
	tests2 = s_OBJ["tests2"]
	targets2 = [tests2[x]["target"] for x in range(len(tests2))]
	operations2 = [tests2[x]["operation"] for x in range(len(tests2))]
	test_points2 = [tests2[x]["test_vals"] for x in range(len(tests2))]

	num_xtests = len(test_points[0])
	num_ytests = len(test_points2[0])
	output = []

	# set up some preliminary values
	times = np.zeros((len(segList), num_xtests, num_ytests))

	for seg_no in range(len(segList)):
		print("\tTesting track " + str(seg_no + 1) + "...")

		for test_no in range(num_xtests):
			# alter the test variables as need be
			for var_no, var in enumerate(targets):
				test_op = operations[var_no]
				test_vals = test_points[var_no]

				if test_op == "product":
					vehicle.setVar(var, vehicle.getOriginalVal(var) * test_vals[test_no])
				elif test_op == "inverse-product":
					vehicle.setVar(var, vehicle.getOriginalVal(var) / test_vals[test_no])
				elif test_op == "replace":
					vehicle.setVar(var, test_vals[test_no])

				# alter the test2 variables as need be
				for test2_no in range(num_ytests):
					# alter the variables as need be
					for var2_no, var2 in enumerate(targets2):
						test_op2 = operations2[var2_no]
						test_vals2 = test_points2[var2_no]

						if test_op2 == "product":
							vehicle.setVar(var2, vehicle.getOriginalVal(var2) * test_vals2[test2_no])
						elif test_op2 == "inverse-product":
							vehicle.setVar(var2, vehicle.getOriginalVal(var2) / test_vals2[test2_no])
						elif test_op2 == "replace":
							vehicle.setVar(var2, test_vals[test2_no])

					# solve under the new conditions
					output.append(steady_solve(vehicle.v, segList[seg_no]))
					times[seg_no, test_no, test2_no] = output[test_no][-1, O_TIME]

			print("\t\tTest row " + str(test_no + 1) + " complete!")
			# plot_velocity_and_events(output[test_no], "time")

		output = []

	for seg_no in range(len(segList)):
		print("Plotting results...")

		plt.imshow(times[seg_no], cmap='hot')
		plt.colorbar(im, orientation='horizontal')
		plt.xlabel(s_OBJ["plot_x_label"])
		plt.ylabel(s_OBJ["plot_y_label"])
	
	plt.show()

except Exception as e: # run 1D test
	print("Running tests...")

	# set up some preliminary values
	num_tests = len(test_points[0])
	plot_points = np.array(s_OBJ["plot_points"])
	times = np.zeros((len(segList), num_tests))
	output = []

	# run 1D study
	for seg_no in range(len(segList)):
		print("\tTesting track " + str(seg_no + 1) + "...")

		for test_no in range(num_tests):
			# alter the variables as need be
			for var_no, var in enumerate(targets):
				test_op = operations[var_no]
				test_vals = test_points[var_no]

				if test_op == "product":
					vehicle.setVar(var, vehicle.getOriginalVal(var) * test_vals[test_no])
				elif test_op == "inverse-product":
					vehicle.setVar(var, vehicle.getOriginalVal(var) / test_vals[test_no])
				elif test_op == "replace":
					vehicle.setVar(var, test_vals[test_no])

			# solve under the new conditions
			output.append(steady_solve(vehicle.v, segList[seg_no]))
			times[seg_no, test_no] = output[test_no][-1, O_TIME]

			print("\t\tTest " + str(test_no + 1) + " complete!")
			# plot_velocity_and_events(output[test_no], "time")
		output = []

	print("Plotting results...")

	# plot the study
	fig, ax = plt.subplots()

	plot_style = s_OBJ["plot_style"]

	for i, track in enumerate(tracks):
		title = s_OBJ["plot_title"] + " for " + track + " at mesh size " + str(meshes[i])
		if plot_style == "basic":
			ax.plot(plot_points, times[i], label=title, marker='x', linestyle='-')
		elif plot_style == "semilog":
			ax.semilogx(plot_points, times[i], label=title, marker='x', linestyle='-')

	ax.grid(True)
	ax.legend()

	plt.xlabel(s_OBJ["plot_x_label"])
	plt.ylabel(s_OBJ["plot_y_label"])

	plt.draw()

	print("Done!")

	plt.show()