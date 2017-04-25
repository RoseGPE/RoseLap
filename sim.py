import numpy as np
import vehicle_parameters as vehicle
import math
import track_segmentation
import matplotlib.pyplot as plt

def step(v0,segment,brake):
  """
  Takes a vehicle step. Returns (see last line) if successful, returns None if vehicle skids off into a wall.
  @param v0 the initial vehicle speed for this step
  @param segment the Segment of the track the vehicle is on
  @param brake a boolean value specifying whether or not to apply the brakes (with full available force)
  """
  # Regardless of what you do, this much is true.
  F_lateral = vehicle.mass*v0**2*segment.curvature
  F_total_available = vehicle.mu*vehicle.mass*vehicle.g # Will include aero later
  
  # If there isn't even enough grip to turn, you're in a bad situation.
  if (F_lateral > F_total_available):
    return None

  # Compute remaining available tire grip
  F_ground_longitudinal_available = math.sqrt(F_total_available**2-F_lateral**2)


  # Accelerate, or decelerate. Why do anything else?
  if brake:
  	F_ground_longitudinal = -F_ground_longitudinal_available # Assume we can lock up tires and bias is perfect
  else:
  	F_ground_longitudinal = min([vehicle.eng_force(v0), F_ground_longitudinal_available]) # Assume driver always goes hard, but never burns out
  #print('ok', vehicle.eng_force(v0))
  F_longitudinal = F_ground_longitudinal # Will include aero later
  
  a = F_longitudinal/vehicle.mass
  try:
    vf = math.sqrt(v0**2 + 2*a*segment.length)
  except:
    vf=0

  return np.array([vf,a,F_ground_longitudinal,F_lateral,segment.sector])

def solve(segments):
  output = np.zeros((len(segments), 5))
  
  step_result = step(0, segments[0], False)
  output[0] = step_result
  i=1
  brake = False
  failpt = -1
  lastsafept = -1
  while i<len(segments):
    step_result = step(output[i-1,0], segments[i], brake)
    #print 'Segment %d' % i
    #print step_result
    if i<failpt:
      output[i] = step_result
      i+=1
      brake = True
    elif i == failpt and step_result is None:
      # evaulate; this braking didn't work
      lastsafept-=1
      i=lastsafept
      #raw_input()

    elif step_result is None:
      # initiate braking algorithm
      brake = True
      failpt = i
      lastsafept = i-1
      i = lastsafept
      #i-=1
    else:
      output[i] = step_result
      i+=1
      brake = False
  

  return output

if __name__ == '__main__':
  segs = track_segmentation.dxf_to_segments('./track.dxf', 0.01)
  #print 'fuk'
  #[print(x.x, x.y, x.curvature, x.sector) for x in segs]
  #track_segmentation.plot_segments(segs)
  #print 'why'
  output = solve(segs)
  #np.set_printoptions(threshold=np.nan)
  #print output
  fig, ax = plt.subplots()
  ax.plot(output[:,0],label='v')
  #ax.plot(output[:,1],label='a')
  #ax.plot(output[:,2],label='Flong')
  #ax.plot(output[:,3],label='Flat')
  #ax.plot(output[:,4],label='sector')

  
  ax.grid(True)
  ax.legend()

  plt.show()