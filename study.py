from sim import *
import vehicle_parameters as vehicle
track = './DXFs/ax.dxf'

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

plt.show()