from sim import *
import study

# vehicle.load("basic.json")

# track = './DXFs/loop.dxf'
# segments = track_segmentation.dxf_to_segments(track, 0.25)

# output = steady_solve(vehicle.v, segments)

# plot_velocity_and_events(output)
# plt.show()

# esf = study.run("esf.json")
# esf.plot()
print('Enter study name...\n> ')
esf3D = study.run(raw_input())
esf3D.plot()

'''
better detail plot titles
data toggle
optional plot points

'''