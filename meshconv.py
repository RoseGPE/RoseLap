import sim_twotires
import sim_pointmass
from plottools import *
import vehicle
import track_segmentation
import fancyyaml as yaml
import cPickle as pickle
import time

def run(filename):
	print('Beginning Mesh Convergence Study')
	mcstudy = {}
	with open('./Studies/'+filename) as f:
		mcstudy = yaml.load(f)

	vehicle.load(mcstudy["vehicle"])

	if not "model" in mcstudy:
		mcstudy['model'] = 'twotires'
	model = mcstudy['model']

	if model == 'twotires':
		sim_pkg = sim_pointmass
	else:
		sim_pkg = sim_twotires

	fig, ax = plt.subplots()
	fig.canvas.set_window_title('Mesh Convergence Results')

	for track in mcstudy['track']:
		meshes = mcstudy['track'][track]
		tf = []
		print('    Track: '+track)
		for mesh in meshes:
			print('        Segment Size: '+str(mesh))
			segs = track_segmentation.dxf_to_segments("./DXFs/" + track, mesh)
			output = sim_pkg.solve(vehicle.v, segs)
			tf.append(output[-1,O_TIME])

		ax.plot(meshes, tf, label=track, marker='x', linestyle='-', picker=5)

	ax.grid(True)
	ax.legend()

	plt.title("Mesh Convergence Study")
	plt.xlabel("Segment Size")
	plt.ylabel("Track Time")

	print('Finished!')

	plt.show()
