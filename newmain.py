from sim import *
import study

# vehicle.load("basic.json")

# track = './DXFs/loop.dxf'
# segments = track_segmentation.dxf_to_segments(track, 0.25)

# output = steady_solve(vehicle.v, segments)

# plot_velocity_and_events(output)
# plt.show()

esf3D = study.run("esf3D.yaml")
esf3D.plot()