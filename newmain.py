from sim import *
import vehicle
import track_segmentation
import study

# vehicle.load("basic.json")

# track = './DXFs/loop.dxf'
# segments = track_segmentation.dxf_to_segments(track, 0.25)

# output = steady_solve(vehicle.v, segments)

# plot_velocity_and_events(output)
# plt.show()

aero = study.run("aero_efficiency_s.json")
fdr = study.run("fdr_s.json")

aero.plot()
fdr.plot()
