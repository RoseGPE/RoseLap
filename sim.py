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
S_SHIFTING = 6

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

# Shifting status codes
IN_PROGRESS = 0
JUST_FINISHED = 1
NOT_SHIFTING = 2


def step(vehicle, prior_result, segment, brake, shifting, shift_goal):
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
  gear = None
  F_longitudinal = 0
  shift_force = prior_result[O_ENG_FORCE]

  Ff_lat = (1-vehicle.weight_bias)*segment.curvature*vehicle.mass*v0**2
  Fr_lat = vehicle.weight_bias*segment.curvature*vehicle.mass*v0**2
  
  Fr_lim = (vehicle.mu*Nr)
  Ff_lim = (vehicle.mu*Nf) 

  if Fr_lat > Fr_lim or Ff_lat > Ff_lim :
    # You have lost control!!
    return None

  Fr_remaining = np.sqrt(Fr_lim**2 - Fr_lat**2)
  if shifting == IN_PROGRESS:
    gear = -1
  else:
    gear = prior_result[O_GEAR]

   elif shifting == IN_PROGRESS:
    gear = -1
    Fr_engine_limit = 0
    status = S_SHIFTING

  elif shifting == JUST_FINISHED:
    gear = shift_goal
    Fr_engine_limit = vehicle.eng_force(v0, int(gear))
    status = S_ENG_LIM_ACC

  elif shifting == NOT_SHIFTING:
    output_forces = [vehicle.eng_force(v0, gear_index) for gear_index in range(len(vehicle.gears))]
    gear = np.argmax(output_forces) # index tho
    Fr_engine_limit = output_forces[gear] # Assume driver always goes hard, but never burns out
    shift_force = Fr_engine_limit
    status = S_ENG_LIM_ACC

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

  

 

  F_drag = vehicle.alpha_drag() * (v0 ** 2)
  F_longitudinal = F_ground_longitudinal - F_drag
  a = F_longitudinal / vehicle.mass

  try:
    vf = math.sqrt(v0**2 + 2*a_long*segment.length)
  except:
    vf=0

  if abs(F_longitudinal) < 1e-3 and shifting != IN_PROGRESS:
    status = S_DRAG_LIM

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

  output = np.array([tf, xf, vf, Nf, Nr, segment.sector, status, gear, a_long / vehicle.g, (v0 ** 2) * segment.curvature / vehicle.g, Ff_remaining, Fr_remaining, shift_force])
  return output

def solve(vehicle, segments, output_0 = None):
  # set up initial stuctures
  output = np.zeros((len(segments), 13))
  shifting = NOT_SHIFTING
  
  if output_0 is None:
    output[0,3] = vehicle.mass*(1-vehicle.weight_bias)*vehicle.g
    output[0,4] = vehicle.mass*vehicle.weight_bias*vehicle.g
  else:
    output[0,:] = output_0
    output[0,0] = 0
    output[0,1] = 0

  step_result = step(vehicle, output[0], segments[0], False, shifting, 0)

  output[0] = step_result

  # step loop set up
  i = 1

  # shifting set up
  shift_start = 0
  shift_goal = 0
  pow_goal = 0
  last_gear = output[0, O_GEAR]
  curr_gear = last_gear
  doshift = False

  # braking set up
  brake = False
  failpt = -1
  lastsafept = -1
<<<<<<< HEAD
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
      brake = False
      doshift = True

      if i == failpt:
        output[i - 1][O_GEAR] = curr_gear
        shifting = NOT_SHIFTING
        doshift = False
        shift_start = 0
        shift_goal = 0
        pow_goal = 0
        last_gear = output[0, O_GEAR]
        curr_gear = last_gear
        doshift = False
        #shift freely
      
      output[i] = step_result
      i+=1
      brake = False
      failpt = -1
      lastsafept = -1


    ### SHIFTING ###
    if doshift:
      i -= 1
      last_gear = curr_gear
      curr_gear = output[i, O_GEAR]

      # we just got done shifting
      if shifting == JUST_FINISHED:
        output[i - 1][O_GEAR] = curr_gear # only necessary the step after

        if output[i][O_ENG_FORCE] >= pow_goal:
          shifting = NOT_SHIFTING

      # we are currently shifting
      elif shifting == IN_PROGRESS:
        if output[i][O_TIME] - output[shift_start][O_TIME] >= vehicle.shift_time:
          shifting = JUST_FINISHED

      # we'd like to change gears
      elif last_gear != curr_gear and shifting == NOT_SHIFTING:
        shifting = IN_PROGRESS
        shift_start = i - 1
        shift_goal = curr_gear
        pow_goal = output[i][O_ENG_FORCE]
        i -= 1

      i += 1
      doshift = False

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
  ax[0].plot(xaxis,t,lw=5,label='Time')
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
  ax[0].fill_between(xaxis, 0, lim, where= status==6, facecolor='#b666d2', alpha=alpha)

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