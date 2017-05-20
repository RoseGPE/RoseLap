import numpy as np
import vehicle_parameters as vehicle
import math
import track_segmentation
import matplotlib.pyplot as plt
import matplotlib.cm as cmx
import matplotlib.colors as colors

"""
Statuses:
1. braking
2. engine limited acceleration
3. tire grip limited acceleration
4. sustaining speed to prevent going out of curve
5. topped out; drag limitation
"""

def step(prior_result,segment,brake):
  """
  Takes a vehicle step. Returns (see last line) if successful, returns None if vehicle skids off into a wall.
  @param v0 the initial vehicle speed for this step
  @param segment the Segment of the track the vehicle is on
  @param brake a boolean value specifying whether or not to apply the brakes (with full available force)
  """

  v0 = prior_result[2];
  x0 = prior_result[1];
  t0 = prior_result[0];

  # Regardless of what you do, this much is true.
  F_lateral = vehicle.mass*v0**2*segment.curvature
  F_down = vehicle.alpha_downforce*v0**2;
  F_total_available = vehicle.mu*(vehicle.mass*vehicle.g + F_down) # Will include aero later
  
  # If there isn't even enough grip to turn, you're in a bad situation.
  if (F_lateral > F_total_available):
    return None

  # Compute remaining available tire grip
  F_ground_longitudinal_available = math.sqrt(F_total_available**2-F_lateral**2)

  gear = None

  # Accelerate, or decelerate. Why do anything else?
  if brake:
    F_ground_longitudinal = -F_ground_longitudinal_available # Assume we can lock up tires and bias is perfect
    status = 1
  else:
    output_forces = [vehicle.eng_force(v0,gear) for gear in range(len(vehicle.gears))]
    #print(output_forces)
    gear = np.argmax(output_forces)
    F_ground_longitudinal = output_forces[gear] # Assume driver always goes hard, but never burns out
    status = 2
    if F_ground_longitudinal > F_ground_longitudinal_available:
      F_ground_longitudinal = F_ground_longitudinal_available
      status = 3
  #print('ok', vehicle.eng_force(v0))
  F_drag = vehicle.alpha_drag*v0**2
  F_longitudinal = F_ground_longitudinal - F_drag
  
  
  a = F_longitudinal/vehicle.mass
  try:
    vf = math.sqrt(v0**2 + 2*a*segment.length)
  except:
    vf=0
  if abs(F_longitudinal) < 1e-3:
    status=5
  if vehicle.mass*vf**2*segment.curvature > vehicle.mu*(vehicle.mass*vehicle.g + vehicle.alpha_downforce*vf**2):
    vf=v0
    status = 4

  tf = t0+segment.length/v0 if v0!=0 else 0;
  xf = x0+segment.length;

  return np.array([tf,xf,vf,segment.sector,status,gear,  F_total_available,F_ground_longitudinal_available, F_ground_longitudinal, F_lateral, F_drag, F_down])

def solve(segments):
  output = np.zeros((len(segments), 12))
  
  output[0,2] = 10
  step_result = step(output[0], segments[0], False)
  output[0] = step_result
  i=1
  brake = False
  failpt = -1
  lastsafept = -1
  while i<len(segments):
    step_result = step(output[i-1,:], segments[i], brake)
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

def colorgen(num_colors, idx):
  color_norm  = colors.Normalize(vmin=0, vmax=num_colors-1)
  scalar_map = cmx.ScalarMappable(norm=color_norm, cmap='hsv') 
  def map_index_to_rgb_color(index):
    return scalar_map.to_rgba(index)
  return map_index_to_rgb_color(idx)

def plot_velocity_and_events(output,axis='x'):
  fig, ax = plt.subplots()
  t = output[:,0]
  x = output[:,1]
  v = output[:,2]
  sectors = output[:,3]
  status = output[:,4]
  

  if axis=='time':
    xaxis = t
  else:
    xaxis = x

  ax.plot(xaxis,v,lw=5,label='Velocity')

  lim = max(v)

  alpha = 0.5
  ax.fill_between(xaxis, 0, lim, where= status==1, facecolor='#e23030', alpha=alpha)
  ax.fill_between(xaxis, 0, lim, where= status==2, facecolor='#50d21d', alpha=alpha)
  ax.fill_between(xaxis, 0, lim, where= status==3, facecolor='#1d95d2', alpha=alpha)
  ax.fill_between(xaxis, 0, lim, where= status==4, facecolor='#d2c81c', alpha=alpha)
  ax.fill_between(xaxis, 0, lim, where= status==5, facecolor='#e2a52b', alpha=alpha)

  sector = sectors[0]
  for idx,sec in enumerate(sectors):
    if sec!=sector:
      ax.axvline(xaxis[idx], color='black', lw=2, alpha=0.9)
      sector=sec
  plt.ylim((0,lim))
  plt.xlim((0,xaxis[-1]))

  #sectors = set(output[:,3])
  #for sector in sectors:
  #  ax.fill_between(t, -100, 100, where=output[:,3]==sector, facecolor=colorgen(len(sectors), sector), alpha=0.3)

  
  ax.grid(True)
  ax.legend()

  plt.show()

if __name__ == '__main__':
  segs = track_segmentation.dxf_to_segments('./track.dxf', 0.1)
  #print 'fuk'
  #[print(x.x, x.y, x.curvature, x.sector) for x in segs]
  #track_segmentation.plot_segments(segs)
  #print 'why'
  output = solve(segs)
  
  print('Took %.3f seconds to travel %.1f feet' % (output[-1,0],output[-1,1]))

  plot_velocity_and_events(output,'x')

  #print(output[:,5])