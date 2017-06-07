import numpy as np
import math
import track_segmentation
import matplotlib.pyplot as plt
import matplotlib.cm as cmx
import matplotlib.colors as colors

# Status Constant Definition
S_BRAKING = 1
S_ENG_LIM_ACC = 2
S_TIRE_LIM_ACC = 3
S_SUSTAINING = 4
S_DRAG_LIM = 5
S_SHIFTING = 6
S_TOPPED_OUT = 7

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
O_CURVATURE = 12

# Shifting status codes
IN_PROGRESS = 0
JUST_FINISHED = 1
NOT_SHIFTING = 2


def step(vehicle, prior_result, segment, segment_next, brake, shifting, gear):
  """
  Takes a vehicle step. Returns (see last line) if successful, returns None if vehicle skids off into a wall.
  @param v0 the initial vehicle speed for this step
  @param segment the Segment of the track the vehicle is on
  @param brake a boolean value specifying whether or not to apply the brakes (with full available force)
  @param shifting a shifting status code
  """

  # init values
  Nf = prior_result[O_NF];
  Nr = prior_result[O_NR];
  v0 = prior_result[O_VELOCITY];
  x0 = prior_result[O_DISTANCE];
  t0 = prior_result[O_TIME];
  status = S_TOPPED_OUT
  F_longitudinal = 0

  Ff_lat = (1-vehicle.weight_bias)*segment.curvature*vehicle.mass*v0**2
  Fr_lat = vehicle.weight_bias*segment.curvature*vehicle.mass*v0**2
  
  Fr_lim = (vehicle.mu*Nr)
  Ff_lim = (vehicle.mu*Nf) 

  if Fr_lat > Fr_lim or Ff_lat > Ff_lim :
    return None

  Fr_remaining = np.sqrt(Fr_lim**2 - Fr_lat**2)

  Fr_engine_limit = vehicle.eng_force(v0, int(gear))

  Ff_remaining = np.sqrt(Ff_lim**2 - Ff_lat**2)

  Fdown = vehicle.alpha_downforce()*v0**2;
  Fdrag = vehicle.alpha_drag()*v0**2;

  if brake:
    status = S_BRAKING
    F_brake = min(Ff_remaining/vehicle.front_brake_bias(), Fr_remaining/vehicle.rear_brake_bias())
    Fr_long = -F_brake*vehicle.rear_brake_bias()
    Ff_long = -F_brake*vehicle.front_brake_bias()
    # Fr_long = -Fr_remaining
    # Ff_long = -Ff_remaining
    gear = np.nan
  elif shifting:
    status = S_SHIFTING
    Fr_long = 0
    Ff_long = 0
    gear = np.nan
  else:
    status = S_ENG_LIM_ACC
    Fr_long = Fr_engine_limit
    if Fr_long > Fr_remaining:
      status = S_TIRE_LIM_ACC
      Fr_long = Fr_remaining
    Ff_long = 0


  a_long = (Fr_long+Ff_long-Fdrag)/vehicle.mass

  F_longitudinal = Ff_long+Fr_long - Fdrag
  a = F_longitudinal / vehicle.mass

  try:
    vf = math.sqrt(v0**2 + 2*a_long*segment.length)
  except:
    a_long=0
    vf=0

  if abs(F_longitudinal) < 1e-3 and shifting != IN_PROGRESS:
    status = S_DRAG_LIM
  if abs(Fr_engine_limit) < 1e-3 :
    status = S_TOPPED_OUT

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

  Ff_lat = (1-vehicle.weight_bias)*segment_next.curvature*vehicle.mass*vf**2
  Fr_lat = vehicle.weight_bias*segment_next.curvature*vehicle.mass*vf**2
  
  Fr_lim = (vehicle.mu*Nr)
  Ff_lim = (vehicle.mu*Nf) 

  a_long_start = a_long
  
  nmax = 10
  n = 0
  while Fr_lat > Fr_lim-1e-2 or Ff_lat > Ff_lim-1e-2 :
    #return None
    a_long-=a_long_start*1/nmax
    vf = math.sqrt(v0**2 + 2*a_long*segment.length)

    Fdown = vehicle.alpha_downforce()*vf**2;
    Fdrag = vehicle.alpha_drag()*vf**2;

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

    Ff_lat = (1-vehicle.weight_bias)*segment_next.curvature*vehicle.mass*vf**2
    Fr_lat = vehicle.weight_bias*segment_next.curvature*vehicle.mass*vf**2
    
    Fr_lim = (vehicle.mu*Nr)
    Ff_lim = (vehicle.mu*Nf)

    n+=1
    if n > nmax:
      return None

  try:
    tf = t0 + segment.length/((v0+vf)/2)
  except:
    tf = t0
  xf = x0 + segment.length

  output = np.array([
    tf,
    xf,
    vf,
    Nf,
    Nr, 
    segment.sector,
    status,
    gear,
    a_long / vehicle.g, 
    (v0 ** 2) * segment.curvature / vehicle.g, 
    Ff_remaining, 
    Fr_remaining, 
    segment.curvature
  ])

  return output

def solve(vehicle, segments, output_0 = None):
  # set up initial stuctures
  output = np.zeros((len(segments), 13))
  shifting = NOT_SHIFTING
  
  if output_0 is None:
    output[0,3] = vehicle.mass*(1-vehicle.weight_bias)*vehicle.g
    output[0,4] = vehicle.mass*vehicle.weight_bias*vehicle.g
    gear = vehicle.best_gear(output[0,O_VELOCITY])
  else:
    output[0,:] = output_0
    output[0,0] = 0
    output[0,1] = 0
    gear = vehicle.best_gear(output_0[O_VELOCITY])

  brake = False
  shiftpt = -1
  shift_v_req = 0
  
  step_result = step(vehicle, output[0], segments[0], segments[1], brake, shiftpt>=0, gear)

  output[0] = step_result

  # step loop set up
  i = 1
  
  failpt = -1
  lastsafept = -1
  while i<len(segments):
    if i<0:
      print('damnit bobby')
      return None
    if (gear is None) and shiftpt < 0:
      gear = vehicle.best_gear(output[i-1,O_VELOCITY])

    step_result = step(vehicle,output[i-1,:], segments[i], (segments[i+1] if i+1<len(segments) else segments[i]), brake, shiftpt>=0, gear)
    if step_result is None:
      if not brake:
        # Start braking
        brake = True
        failpt = i
        lastsafept = i-1
        i = lastsafept
        #plot_velocity_and_events(output)
        #plt.show()
      else:
        # Try again from an earlier point
        lastsafept-=1
        i=lastsafept
      # reset shifting params
      gear = None
      shiftpt = -1
    elif i<failpt:
      output[i] = step_result
      i+=1
      brake = True
      # reset shifting params
      gear = None
      shiftpt = -1
      shift_v_req = 0
    else:
      # normal operation

      # quit braking
      brake = False # problematic??
      failpt = -1
      lastsafept = -1

      output[i] = step_result

      better_gear = vehicle.best_gear(output[i,O_VELOCITY]*(1-vehicle.shift_extra_factor))

      if output[i,O_STATUS]==S_ENG_LIM_ACC and shiftpt < 0 and gear != better_gear and output[i,O_VELOCITY]>shift_v_req:
        
        gear += int((better_gear-gear)/abs(better_gear-gear))
        shiftpt = i
        shift_v_req = output[i,O_VELOCITY]*1.01

      if shiftpt >= 0 and output[i,O_TIME] > output[shiftpt,O_TIME]+vehicle.shift_time:
        shiftpt = -1
        i-=1
      
      i+=1

      

  np.savetxt('dump.csv', output, delimiter=",")
  return output

def steady_solve(vehicle,segments):
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

  curv = output[:, O_CURVATURE]*100

  if axis == 'time':
    plt.xlabel('Elapsed time')
    xaxis = t
  else:
    xaxis = x
    plt.xlabel('Distance travelled')

  ax[0].plot(xaxis,v,lw=5,label='Velocity')
  ax[0].plot(xaxis,curv,lw=5,label='Curvature',marker='.',linestyle='none')
  ax[1].plot(xaxis,along,lw=4,label='Longitudinal g\'s')
  ax[1].plot(xaxis,alat,lw=4,label='Lateral g\'s')
  ax[1].plot(xaxis,gear,lw=4,label='Gear')

  lim = max(v)
  alpha =  1

  ax[0].fill_between(xaxis, 0, lim, where= status==S_BRAKING,      facecolor='#e22030', alpha=alpha)
  ax[0].fill_between(xaxis, 0, lim, where= status==S_ENG_LIM_ACC,  facecolor='#50d21d', alpha=alpha)
  ax[0].fill_between(xaxis, 0, lim, where= status==S_TIRE_LIM_ACC, facecolor='#1d95d2', alpha=alpha)
  ax[0].fill_between(xaxis, 0, lim, where= status==S_SUSTAINING,   facecolor='#d2c81c', alpha=alpha)
  ax[0].fill_between(xaxis, 0, lim, where= status==S_DRAG_LIM,     facecolor='#e2952b', alpha=alpha)
  ax[0].fill_between(xaxis, 0, lim, where= status==S_SHIFTING,     facecolor='#454545', alpha=alpha)
  ax[0].fill_between(xaxis, 0, lim, where= status==S_TOPPED_OUT,   facecolor='#7637a2', alpha=alpha)

  sector = sectors[0]
  for idx,sec in enumerate(sectors):
    if sec!=sector:
      ax[0].axvline(xaxis[idx], color='black', lw=2, alpha=0.9)
      sector=sec
  ax[0].set_ylim((0,lim+1))
  #ax[1].set_ylim((min((min(along),min(alat)))-0.1,0.1+max((max(along),max(alat)))))
  ax[1].set_ylim(-5,5)
  plt.xlim((0,max(xaxis)))

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

  track = './DXFs/accel.dxf'
  segments = track_segmentation.dxf_to_segments(track, 0.1)

  #track_segmentation.plot_segments(segments)

  output = solve(vehicle.v, segments)

  plot_velocity_and_events(output)

  plt.show()