import numpy as np
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
# Status Constant Definition
S_BRAKING = 1
S_ENG_LIM_ACC = 2
S_TIRE_LIM_ACC = 3
S_SUSTAINING = 4
S_DRAG_LIM = 5

# Output Index Constant Definitions (Columns)
O_TIME = 0
O_DISTANCE = 1
O_VELOCITY = 2
O_NF = 3
O_NR = 4
O_SECTORS = 5
O_STATUS = 6
O_GEAR = 7
O_LONG_ACC = 8
O_LAT_ACC = 9


def step(vehicle,prior_result,segment,brake):
  status = -1
  """
  Takes a vehicle step. Returns (see last line) if successful, returns None if vehicle skids off into a wall.
  @param v0 the initial vehicle speed for this step
  @param segment the Segment of the track the vehicle is on
  @param brake a boolean value specifying whether or not to apply the brakes (with full available force)
  """

  Nf = prior_result[3];
  Nr = prior_result[4];
  v0 = prior_result[2];
  x0 = prior_result[1];
  t0 = prior_result[0];

  Ff_lat = (1-vehicle.weight_bias)*segment.curvature*vehicle.mass*v0**2
  Fr_lat = vehicle.weight_bias*segment.curvature*vehicle.mass*v0**2
  
  Fr_lim = (vehicle.mu*Nr)
  Ff_lim = (vehicle.mu*Nf) 

  if Fr_lat > Fr_lim or Ff_lat > Ff_lim :
    # You have lost control!!
    return None

  Fr_remaining = np.sqrt(Fr_lim**2 - Fr_lat**2)
  Fr_engine_limits = [vehicle.eng_force(v0,g) for g in range(len(vehicle.gears))]
  gear = np.argmax(Fr_engine_limits)
  Fr_engine_limit = Fr_engine_limits[gear]
  Ff_remaining = np.sqrt(Ff_lim**2 - Ff_lat**2)

  Fdown = vehicle.alpha_downforce()*v0**2;
  Fdrag = vehicle.alpha_drag()*v0**2;

  if brake:
    status = S_BRAKING
    F_brake = min(Ff_remaining/vehicle.front_brake_bias(), Fr_remaining/vehicle.rear_brake_bias())
    Fr_long = -F_brake*vehicle.rear_brake_bias()
    Ff_long = -F_brake*vehicle.front_brake_bias()
    gear = -1
  else:
    status = S_TIRE_LIM_ACC
    Fr_long = min(Fr_engine_limit, Fr_remaining);
    Ff_long = 0

  a_long = (Fr_long+Ff_long-Fdrag)/vehicle.mass

  try:
    vf = math.sqrt(v0**2 + 2*a_long*segment.length)
  except:
    vf=0

  tf = t0+segment.length/((v0+vf)/2) if v0!=0 else 0;
  xf = x0+segment.length;

  Nf = ( -vehicle.weight_bias*vehicle.g*vehicle.mass
      - Fdown*vehicle.weight_bias
      - vehicle.mass*a_long*vehicle.cg_height/vehicle.wheelbase_length
      + vehicle.mass*vehicle.g
      + Fdown
      - Fdrag*vehicle.cp_height/vehicle.wheelbase_length )

  Nr = ( vehicle.weight_bias*vehicle.g*vehicle.mass
      + Fdown*vehicle.cp_bias
      + vehicle.mass*a_long*vehicle.cg_height/vehicle.wheelbase_length
      + Fdrag*vehicle.cg_height/vehicle.wheelbase_length )

  Ff_lat = (1-vehicle.weight_bias)*segment.curvature*vehicle.mass*v0**2
  Fr_lat = vehicle.weight_bias*segment.curvature*vehicle.mass*v0**2
  
  Fr_lim = (vehicle.mu*Nr)
  Ff_lim = (vehicle.mu*Nf) 

  if Fr_lat > Fr_lim or Ff_lat > Ff_lim :
    vf=v0
    a_long=0
    Nf = ( -vehicle.weight_bias*vehicle.g*vehicle.mass
      - Fdown*vehicle.weight_bias
      - vehicle.mass*a_long*vehicle.cg_height/vehicle.wheelbase_length
      + vehicle.mass*vehicle.g
      + Fdown
      - Fdrag*vehicle.cp_height/vehicle.wheelbase_length )

    Nr = ( vehicle.weight_bias*vehicle.g*vehicle.mass
        + Fdown*vehicle.cp_bias
        + vehicle.mass*a_long*vehicle.cg_height/vehicle.wheelbase_length
        + Fdrag*vehicle.cg_height/vehicle.wheelbase_length )

  output = np.array([tf, xf, vf, Nf, Nr, segment.sector, status, gear, a_long / vehicle.g, (v0 ** 2) * segment.curvature / vehicle.g, Ff_remaining, Fr_remaining])
  return output

def solve(vehicle, segments, output_0 = None):
  # set up output matrix
  output = np.zeros((len(segments), 12))
  
  if output_0 is None:
    output[0,3] = vehicle.mass*(1-vehicle.weight_bias)*vehicle.g
    output[0,4] = vehicle.mass*vehicle.weight_bias*vehicle.g
  else:
    output[0,:] = output_0
    output[0,0] = 0
    output[0,1] = 0

  step_result = step(vehicle, output[0], segments[0], False)
  output[0] = step_result

  i = 1
  brake = False
  failpt = -1
  lastsafept = -1
  while i<len(segments):
    if i<0:
      print('damnit bobby')
      return None
    step_result = step(vehicle,output[i-1,:], segments[i], brake)
    if step_result is None:
      if not brake:
        # Start braking
        brake = True
        failpt = i
        lastsafept = i-1
        i = lastsafept
      else:
        # Try again from an earlier point
        lastsafept-=1
        i=lastsafept
    elif i<failpt:
      output[i] = step_result
      i+=1
      brake = True
    else:
      output[i] = step_result
      i+=1
      brake = False
      failpt = -1
      lastsafept = -1
  # while i < len(segments):
  #   step_result = step(vehicle, output[i-1], segments[i], brake)

  #   # we've been told to try again, this time with brakes
  #   if i < failpt:
  #     if step_result is None:
  #       print "wuh oh"
  #       np.savetxt('dump.csv', output, delimiter=",")
  #       return output
  #     output[i] = step_result
  #     i += 1
  #     brake = True

  #   # we caught the issue too late, we need to back up even more
  #   elif i == failpt and step_result is None:
  #     lastsafept -= 1
  #     i = lastsafept

  #   # the curve was too much, we need to back up and brake
  #   elif step_result is None:
  #     brake = True
  #     failpt = i
  #     lastsafept = i-1
  #     i = lastsafept

  #   # nothing's wrong, just how we like it :)
  #   else:
  #     output[i] = step_result
  #     i += 1
  #     brake = False

  np.savetxt('dump.csv', output, delimiter=",")
  return output

def steady_solve(vehicle,segments):
  # TODO: Find a better way to do this #sketch
  output = solve(vehicle,segments)
  return solve(vehicle,segments,output[-1, :])

def colorgen(num_colors, idx):
  color_norm  = colors.Normalize(vmin=0, vmax=num_colors-1)
  scalar_map = cmx.ScalarMappable(norm=color_norm, cmap='hsv') 
  def map_index_to_rgb_color(index):
    return scalar_map.to_rgba(index)
  return map_index_to_rgb_color(idx)

def plot_velocity_and_events(output, axis='x'):
  fig, ax = plt.subplots(2, sharex=True)

  t = output[:, O_TIME]
  x = output[:, O_DISTANCE]
  v = output[:, O_VELOCITY]

  sectors = output[:, O_SECTORS]
  status = output[:, O_STATUS]
  gear = output[:, O_GEAR]

  along = output[:, O_LONG_ACC]
  alat = output[:, O_LAT_ACC]

  if axis == 'time':
    plt.xlabel('Elapsed time')
    xaxis = t
  else:
    xaxis = x
    plt.xlabel('Distance travelled')

  ax[0].plot(xaxis,v,lw=5,label='Velocity')
  ax[1].plot(xaxis,along,lw=4,label='Longitudinal g\'s')
  ax[1].plot(xaxis,alat,lw=4,label='Lateral g\'s')
  ax[1].plot(xaxis,gear,lw=4,label='Gear')

  lim = max(v)
  alpha = 0.5
  ax[0].fill_between(xaxis, 0, lim, where= status==1, facecolor='#e23030', alpha=alpha)
  ax[0].fill_between(xaxis, 0, lim, where= status==2, facecolor='#50d21d', alpha=alpha)
  ax[0].fill_between(xaxis, 0, lim, where= status==3, facecolor='#1d95d2', alpha=alpha)
  ax[0].fill_between(xaxis, 0, lim, where= status==4, facecolor='#d2c81c', alpha=alpha)
  ax[0].fill_between(xaxis, 0, lim, where= status==5, facecolor='#e2a52b', alpha=alpha)

  sector = sectors[0]
  for idx,sec in enumerate(sectors):
    if sec!=sector:
      ax[0].axvline(xaxis[idx], color='black', lw=2, alpha=0.9)
      sector=sec
  ax[0].set_ylim((0,lim+1))
  #ax[1].set_ylim((min((min(along),min(alat)))-0.1,0.1+max((max(along),max(alat)))))
  ax[1].set_ylim(-5,5)
  plt.xlim((0,xaxis[-1]))

  #sectors = set(output[:,3])
  #for sector in sectors:
  #  ax.fill_between(t, -100, 100, where=output[:,3]==sector, facecolor=colorgen(len(sectors), sector), alpha=0.3)

  
  ax[0].grid(True)
  ax[0].legend()
  ax[1].legend()

  plt.draw()

if __name__ == '__main__':
  import vehicle
  import track_segmentation

  vehicle.load("basic.json")

  track = './DXFs/loop.dxf'
  segments = track_segmentation.dxf_to_segments(track, 0.25)

  output = solve(vehicle.v, segments)

  plot_velocity_and_events(output)
  plt.show()