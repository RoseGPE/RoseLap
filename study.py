from sim import *
import vehicle
import track_segmentation
import json

"""
Study Schema:
vehicle: [string] filename for .json in the Vehicles directory
track: [string] filename for .dxf in the DXFs directory
segment_distance: [float]
tests: [object-array]
	target: [string] vehicle parameter to alter
	operation: 'replace' || 'product' || 'inverse-product'
	test_vals: [float-array]
plot_style: 'semilog' || 'basic'
plot_title: [string]
plot_x_label: [string]
plot_y_label: [string]
plot_points: [float-array]
"""

print("Loading test...")

# load the study JSON into s_OBJ
# study_JSON = './Studies/aero_scale_factor_s.json'
study_JSON = './Studies/test.json'
with open(study_JSON) as data:
  s_OBJ = json.load(data)

# load vehicle
vehicle.load(s_OBJ["vehicle"])

print("Setting up tests...")

# set up track
track = './DXFs/' + s_OBJ["track"]
segments = track_segmentation.dxf_to_segments(track, s_OBJ["segment_distance"])

# # the following lines were lost under the sweeping branch of the new regime
# tests = np.array(s_OBJ["test_points"])
# test_op = s_OBJ["test_operation"]

# set up tests
tests = s_OBJ["tests"]
targets = list(map(lambda x: tests[x]["target"], range(len(tests)))) # blame PLC for this
operations = list(map(lambda x: tests[x]["operation"], range(len(tests)))) # I'm so sorry
test_points = list(map(lambda x: tests[x]["test_vals"], range(len(tests)))) # I've forgotten how to do this the normal way

# set up some preliminary values
num_tests = len(test_points[0])
output = []
plot_points = np.array(s_OBJ["plot_points"])
times = np.zeros(num_tests)

print("Running tests...")

# run the study
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
	output.append(steady_solve(vehicle.v, segments))
	times[test_no] = output[test_no][-1, O_TIME]

	print("\tTest " + str(test_no + 1) + " complete!")
	plot_velocity_and_events(output[test_no])

print("Plotting results...")

# plot the study
fig, ax = plt.subplots()

plot_style = s_OBJ["plot_style"]
if plot_style == "basic":
	ax.plot(plot_points, times, label=s_OBJ["plot_title"], marker='x', linestyle='-')
elif plot_style == "semilog":
	ax.semilogx(plot_points, times, label=s_OBJ["plot_title"], marker='x', linestyle='-')

ax.grid(True)
ax.legend()

plt.xlabel(s_OBJ["plot_x_label"])
plt.ylabel(s_OBJ["plot_y_label"])

plt.draw()

print("Done!")

plt.show()

# track = './DXFs/ax.dxf'
# segs = track_segmentation.dxf_to_segments(track, 0.25)
# possibilities = np.array([0.7,0.8,0.9,1.0,1.1,1.2,1.3,1.4])
# output = []
# times = np.zeros(len(possibilities))

# for i in range(len(possibilities)):
# 	vehicle.downforce_35mph=65.0 *math.sqrt(possibilities[i])
# 	vehicle.drag_35mph=44.0      *math.sqrt(1/possibilities[i])
# 	output.append(steady_solve(vehicle,segs))
# 	print('Aero Efficiency: %.2f, took %.3f seconds to travel %.1f feet' % (possibilities[i],output[i][-1,0],output[i][-1,1]) )
# 	times[i]=output[i][-1,0]

# fig, ax = plt.subplots()
# print(possibilities,times)

# ax.plot(possibilities,times,label='Track Time',marker='x',linestyle='-')

# ax.grid(True)
# ax.legend()
# #plt.gca().invert_xaxis()
# plt.xlabel('Wing Efficiency')
# plt.ylabel('Track Time')
# plt.draw()

# plt.show()