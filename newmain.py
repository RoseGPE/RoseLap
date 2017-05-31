from sim import *
import vehicle
import track_segmentation

vehicle.load("basic.json")

track = './DXFs/loop.dxf'
segments = track_segmentation.dxf_to_segments(track, 0.25)

output = steady_solve(vehicle.v, segments)

plot_velocity_and_events(output)
plt.show()