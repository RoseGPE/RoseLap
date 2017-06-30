import sim_twotires
import sim_pointmass
from constants import *
from plottools import *
import vehicle
import track_segmentation
import fancyyaml as yaml
import cPickle as pickle
import time
import pointsim
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

def load(filename):
	with open('./Results/'+filename, 'rb') as f:
		return pickle.load(f)

class StudyRecord:
	def __init__(self, filename, study_text, timestamp_start, timestamp_end, output, times, co2s, lat_accels, segList, sobj, kind="2D"):
		self.filename=filename
		self.study_text=study_text
		self.output = output
		self.co2s = co2s
		self.lat_accels = lat_accels
		self.times = times
		self.segList = segList
		self.sobj = sobj
		self.kind = kind
		self.timestamp_start = timestamp_start
		self.timestamp_end = timestamp_end

		for key in sobj:
			setattr(self, key, sobj[key])

	def save(self):
		print('Saving...')
		with open('./Results/'+self.filename, 'wb') as f:
			pickle.dump(self, f, protocol=2)
		print('Saved.')

	def plot(self):
		print("Plotting results...")

		if self.kind == "2D":
			if 'time' in self.plot_outputs:
				fig, ax = plt.subplots()
				fig.canvas.set_window_title('Laptime Results')

				# plot the study
				for i, track in enumerate(self.track):
					title = self.track[i] + " (mesh size: " + str(self.segment_distance[i]) + ")" 

					if self.plot_style == "basic":
						ax.plot(self.plot_points, self.times[i], label=title, marker='x', linestyle='-', picker=5)
					elif self.plot_style == "semilog":
						ax.semilogx(self.plot_points, self.times[i], label=title, marker='x', linestyle='-', picker=5)

				ax.grid(True)
				ax.legend()

				plt.title(self.plot_title+" (Laptimes)")
				plt.xlabel(self.plot_x_label)
				plt.ylabel("Laptime")


				# interactivity, maybe
				if len(self.tests) == 1:
					print("we doin this")
				details = DetailZoom(self, 0)
				fig.canvas.mpl_connect('pick_event', details.onpick)
				fig.canvas.show()

			if 'lateral_acceleration' in self.plot_outputs:
				fig, ax = plt.subplots()
				fig.canvas.set_window_title('Lat. Accel Results')

				# plot the study
				for i, track in enumerate(self.track):
					title = self.track[i] + " (mesh size: " + str(self.segment_distance[i]) + ")" 

					if self.plot_style == "basic":
						ax.plot(self.plot_points, self.lat_accels[i], label=title, marker='x', linestyle='-', picker=5)
					elif self.plot_style == "semilog":
						ax.semilogx(self.plot_points, self.lat_accels[i], label=title, marker='x', linestyle='-', picker=5)

				ax.grid(True)
				ax.legend()

				plt.title(self.plot_title+" (Lat. Accel)")
				plt.xlabel(self.plot_x_label)
				plt.ylabel("Lateral Acceleration (G's)")


				# interactivity, maybe
				if len(self.tests) == 1:
					print("we doin this")
				details = DetailZoom(self, 0)
				fig.canvas.mpl_connect('pick_event', details.onpick)
				fig.canvas.show()

			if 'points' in self.plot_outputs:
				fig, ax = plt.subplots()
				fig.canvas.set_window_title('Points For Each Track')
				
				print(self.plot_points)
				# plot the study
				pts_total = None
				for i, track in enumerate(self.track):
					title = self.track[i] + " (mesh size: " + str(self.segment_distance[i]) + ")" 

					pts = pointsim.compute_points(self.point_formulas[i],self.min_times[i],self.min_co2[i],self.times[i],self.co2s[i])
					if pts_total is None:
						pts_total = pts
					else:
						pts_total += pts

					if self.plot_style == "basic":
						ax.plot(self.plot_points, pts, label=title, marker='x', linestyle='-', picker=5)
					elif self.plot_style == "semilog":
						ax.semilogx(self.plot_points, pts, label=title, marker='x', linestyle='-', picker=5)

				ax.plot(self.plot_points, pts_total, label='Total Points', marker='x', linestyle='-', picker=5)

				ax.grid(True)
				ax.legend()

				plt.title(self.plot_title+" (Points)")
				plt.xlabel(self.plot_x_label)
				plt.ylabel("Points")


				# interactivity, maybe
				if len(self.tests) == 1:
					print("we doin this")
				details = DetailZoom(self, 0)
				fig.canvas.mpl_connect('pick_event', details.onpick)
				fig.canvas.show()

			plt.show()



		elif self.kind == "3D":
			total_points = None
			axes = []
			details = []
			for seg_no in range(len(self.segList)):
				if 'time' in self.plot_outputs:
					fig, ax = plt.subplots()
					axes.append(ax)
					fig.canvas.set_window_title('3D Study Results (Times)')
					# data setup
					X1 = np.array(self.plot_x_points)
					Y1 = np.array(self.plot_y_points)
					X, Y = np.meshgrid(X1, Y1)
					Z = np.transpose(self.times[seg_no])

					# plotting shaded regions
					CS = plt.contourf(X, Y, Z, 200, cmap="plasma_r")
					cbar = plt.colorbar(CS)

					# plotting min track time
					minval = Z.min()
					itemindex = np.where(Z==minval)
					ys, xs = itemindex
					minx = X1[xs[0]]
					miny = Y1[ys[0]]

					plt.scatter(X, Y, marker="x", label="Details", picker=20)
					plt.scatter(minx, miny, marker="o", s=20, label="Min Track Time", zorder=10, picker=5)

					# adding labels + legibility
					plt.legend()

					plt.xticks(X1)
					plt.yticks(Y1)
					plt.grid(True)

					plt.title(self.plot_title + " (Times) on " + self.track[seg_no] + " (mesh size: " + str(self.segment_distance[seg_no]) + ")" )
					plt.xlabel(self.plot_x_label)
					plt.ylabel(self.plot_y_label)

					# interactivity, maybe
					# if len(self.tests) == 1:
						#print("we doin this")
					

					details.append(DetailZoom(self, seg_no))
					fig.canvas.mpl_connect('pick_event', details[-1].onpick)

					fig.canvas.show()

				if 'points' in self.plot_outputs:
					fig, ax = plt.subplots()
					axes.append(ax)
					fig.canvas.set_window_title('3D Study Results (Points)')
					# data setup
					X1 = np.array(self.plot_x_points)
					Y1 = np.array(self.plot_y_points)
					X, Y = np.meshgrid(X1, Y1)
					Z = np.transpose(pointsim.compute_points(self.point_formulas[seg_no],self.min_times[seg_no],self.min_co2[seg_no],self.times[seg_no],self.co2s[seg_no]))
					if total_points is None:
						total_points = Z
					else:
						total_points += Z

					# plotting shaded regions
					CS = plt.contourf(X, Y, Z, 200, cmap="viridis")
					cbar = plt.colorbar(CS)

					# plotting min track time
					minval = Z.max()
					itemindex = np.where(Z==minval)
					ys, xs = itemindex
					maxx = X1[xs[0]]
					maxy = Y1[ys[0]]

					plt.scatter(X, Y, marker="x", label="Details", picker=20)
					plt.scatter(maxx, maxy, marker="o", s=20, label="Max Points", zorder=10, picker=5)

					# adding labels + legibility
					plt.legend()

					plt.xticks(X1)
					plt.yticks(Y1)
					plt.grid(True)

					plt.title(self.plot_title + " (Points) on " + self.track[seg_no] + " (mesh size: " + str(self.segment_distance[seg_no]) + ")" )
					plt.xlabel(self.plot_x_label)
					plt.ylabel(self.plot_y_label)

					# interactivity, maybe
					# if len(self.tests) == 1:
						#print("we doin this")
					

					details.append(DetailZoom(self, seg_no))
					fig.canvas.mpl_connect('pick_event', details[-1].onpick)

					fig.canvas.show()

				if 'lateral_acceleration' in self.plot_outputs:
					fig, ax = plt.subplots()
					axes.append(ax)
					fig.canvas.set_window_title('3D Study Results (Lat. Accel.)')
					# data setup
					X1 = np.array(self.plot_x_points)
					Y1 = np.array(self.plot_y_points)
					X, Y = np.meshgrid(X1, Y1)
					Z = np.transpose(self.lat_accels[seg_no])

					# plotting shaded regions
					CS = plt.contourf(X, Y, Z, 200, cmap="inferno")
					cbar = plt.colorbar(CS)

					# plotting min track time
					minval = Z.max()
					itemindex = np.where(Z==minval)
					ys, xs = itemindex
					maxx = X1[xs[0]]
					maxy = Y1[ys[0]]

					plt.scatter(X, Y, marker="x", label="Details", picker=20)
					plt.scatter(maxx, maxy, marker="o", s=20, label="Max Points", zorder=10, picker=5)

					# adding labels + legibility
					plt.legend()

					plt.xticks(X1)
					plt.yticks(Y1)
					plt.grid(True)

					plt.title(self.plot_title + " (Lat. Accel.) on " + self.track[seg_no] + " (mesh size: " + str(self.segment_distance[seg_no]) + ")" )
					plt.xlabel(self.plot_x_label)
					plt.ylabel(self.plot_y_label)

					# interactivity, maybe
					# if len(self.tests) == 1:
						#print("we doin this")
					

					details.append(DetailZoom(self, seg_no))
					fig.canvas.mpl_connect('pick_event', details[-1].onpick)

					fig.canvas.show()


			if 'points' in self.plot_outputs:
				fig, ax = plt.subplots()
				axes.append(ax)
				fig.canvas.set_window_title('3D Study Results (Overall Points)')
				# data setup
				X1 = np.array(self.plot_x_points)
				Y1 = np.array(self.plot_y_points)
				X, Y = np.meshgrid(X1, Y1)
				Z = total_points

				# plotting shaded regions
				CS = plt.contourf(X, Y, Z, 200, cmap="viridis")
				cbar = plt.colorbar(CS)

				# plotting min track time
				minval = Z.max()
				itemindex = np.where(Z==minval)
				ys, xs = itemindex
				maxx = X1[xs[0]]
				maxy = Y1[ys[0]]

				plt.scatter(X, Y, marker="x", label="Details", picker=20)
				plt.scatter(maxx, maxy, marker="o", s=20, label="Max Points", zorder=10, picker=5)

				# adding labels + legibility
				plt.legend()

				plt.xticks(X1)
				plt.yticks(Y1)
				plt.grid(True)

				plt.title(self.plot_title + " (Points) on " + self.track[seg_no] + " (mesh size: " + str(self.segment_distance[seg_no]) + ")" )
				plt.xlabel(self.plot_x_label)
				plt.ylabel(self.plot_y_label)

				# interactivity, maybe
				# if len(self.tests) == 1:
					#print("we doin this")
				

				details.append(DetailZoom(self, seg_no))
				fig.canvas.mpl_connect('pick_event', details[-1].onpick)

				fig.canvas.show()



			plt.show()
		else:
			print("Invalid Study")

		print("Done!")

def run(filename):
	timestamp_start = time.time()
	print("Loading test...")

	# load the study YAML into s_OBJ
	study_YAML = './Studies/' + filename
	with open(study_YAML) as data:
	  s_OBJ = yaml.load(data)
	with open(study_YAML) as data:
	  study_text = data.read()

	# load vehicle
	vehicle.load(s_OBJ["vehicle"])

	print("Setting up tests...")

	if not 'model' in s_OBJ:
		s_OBJ['model'] = 'twotires'
	model = s_OBJ['model']
	print(repr(model))

	if model == 'pointmass':
		print('point mass model')
		sim_pkg = sim_pointmass
	else:
		print('two tire model')
		sim_pkg = sim_twotires

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
	output = []

	try: # run 3D test
		tests2 = s_OBJ["tests2"]
		targets2 = [tests2[x]["target"] for x in range(len(tests2))]
		operations2 = [tests2[x]["operation"] for x in range(len(tests2))]
		test_points2 = [tests2[x]["test_vals"] for x in range(len(tests2))]

		num_xtests = len(test_points[0])
		num_ytests = len(test_points2[0])

		if "plot_x_points" in s_OBJ and isinstance(s_OBJ["plot_x_points"],list) :
			pass
		else:
			s_OBJ['plot_x_points'] = tests[0]["test_vals"]
		if "plot_y_points" in s_OBJ and isinstance(s_OBJ["plot_y_points"],list) :
			pass
		else:
			s_OBJ['plot_y_points'] = tests2[0]["test_vals"]

		# set up some preliminary values
		times = np.zeros((len(segList), num_xtests, num_ytests))
		co2s = np.zeros((len(segList), num_xtests, num_ytests))
		lat_accels = np.zeros((len(segList), num_xtests, num_ytests))

		for seg_no in range(len(segList)):
			print("\tTesting track " + str(seg_no + 1) + "...")

			for test_no in range(num_xtests):
				print("\t\tTesting parameter row " + str(test_no + 1) + "...")

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
								vehicle.setVar(var2, test_vals2[test2_no])

						# solve under the new conditions
						if s_OBJ["steady_state"][seg_no]:
							#print('steady as she goes')
							output.append(sim_pkg.steady_solve(vehicle.v, segList[seg_no]))
						else:
							output.append(sim_pkg.solve(vehicle.v, segList[seg_no]))
						
						times[seg_no, test_no, test2_no] = output[-1][-1, O_TIME]
						co2s[seg_no, test_no, test2_no]  = output[-1][-1, O_CO2]
						lat_accels[seg_no, test_no, test2_no] = output[-1][-1, O_LAT_ACC]

						print("\t\t\tTest parameter " + str(test2_no + 1) + " complete!")

		print("Done!")
		return StudyRecord(filename, study_text, timestamp_start, time.time(), output, times, co2s, lat_accels, segList, s_OBJ, "3D")

	except KeyError as e: # run 2D test
		print("Running tests...")

		# set up some preliminary values
		num_tests = len(test_points[0])
		if "plot_points" in s_OBJ and isinstance(s_OBJ["plot_points"],list) :
			pass
		else:
			s_OBJ['plot_points'] = tests[0]["test_vals"]


		times = np.zeros((len(segList), num_tests))
		co2s = np.zeros((len(segList), num_tests))

		lat_accels = np.zeros((len(segList), num_tests))

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
				if s_OBJ["steady_state"][seg_no]:
					#print('steady as she goes')
					output.append(sim_pkg.steady_solve(vehicle.v, segList[seg_no]))
				else:
					output.append(sim_pkg.solve(vehicle.v, segList[seg_no]))
				times[seg_no, test_no] = output[-1][-1, O_TIME]
				co2s[seg_no, test_no] = output[-1][-1, O_CO2]
				lat_accels[seg_no, test_no] = output[-1][-1, O_LAT_ACC]
				

				print("\t\tTest " + str(test_no + 1) + " complete!")
				# plot_velocity_and_events(output[test_no], "time")

		return StudyRecord(filename, study_text, timestamp_start, time.time(), output, times, co2s, lat_accels, segList, s_OBJ)
