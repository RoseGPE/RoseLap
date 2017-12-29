import numpy as np

def batch(tests, vehicle, tracks, model, include_output):
	permutations, targets, flatTargets = listify(tests, vehicle)
	data = batch_run(flatTargets, permutations, vehicle, tracks, model, include_output)
	return (data, targets)

def listify(tests, vehicle):
	targets = [[t.target for t in test.axis] for test in tests]
	test_vals = [[t.test_vals for t in test.axis] for test in tests]
	ops = [[t.operation for t in test.axis] for test in tests]
	scaled_vals = [scale(targets[i], test_vals[i], ops[i], vehicle) for i in range(len(tests))]

	# list of pivoted test values
	values = [[[t[i] for t in axis] for i in range(len(axis[0]))] for axis in scaled_vals]

	bases = values[0]
	extensions = values[1 :]
	permutations = []

	for base in bases:
		permutations.extend(permutation_extend(base, extensions))

	# list of flat lists of values that line up with targets
	flatTargets = sum(targets, [])
	return (permutations, targets, flatTargets)

def scale(targets, vals, ops, vehicle):
	for i in range(len(ops)):
		op = ops[i]
		if op == 'product':
			base = getattr(vehicle, targets[i])
			vals[i] = [base * val for val in vals[i]]

	return vals

def permutation_extend(base, extensions):
	if len(extensions) == 0:
		return base

	res = []
	for extension in extensions[0]:
		res.append(base + extension)

	return permutation_extend(res, extensions[1 :])

def batch_run(targets, permutations, vehicle, tracks, model, include_output):
	test_data = []

	for track in tracks:
		track_data = [] # time or output matrix, depending on include_output
		segments, steady_state, name = track
		info = (name, steady_state)

		for permutation in permutations:
			vehicle = set_values(vehicle, targets, permutation)
			data = model.steady_solve(vehicle, segments) if steady_state else model.solve(vehicle, segments)
			
			if include_output:
				track_data.append((permutation, data.tolist()))
			else:
				track_data.append((permutation, float(data[-1, 0])))

		test_data.append((track_data, info))

	return test_data

def set_values(vehicle, targets, permutation):
	for i, target in enumerate(targets):
		setattr(vehicle, target, permutation[i])

	vehicle.prep()
	return vehicle