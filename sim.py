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
O_SECTORS = 3
O_STATUS = 4
O_GEAR = 5
O_LONG_ACC = 6
O_LAT_ACC = 7

# Shifting status codes
IN_PROGRESS = 0
JUST_FINISHED = 1
NOT_SHIFTING = 2


def step(vehicle, prior_result, segment, brake, shifting):
  """
  Takes a vehicle step. Returns (see last line) if successful, returns None if vehicle skids off into a wall.
  @param v0 the initial vehicle speed for this step
  @param segment the Segment of the track the vehicle is on
  @param brake a boolean value specifying whether or not to apply the brakes (with full available force)
  @param shifting a shifting status code
  """

  # init values
  v0 = prior_result[O_VELOCITY];
  x0 = prior_result[O_DISTANCE];
  t0 = prior_result[O_TIME];
  gear = None
  F_longitudinal = 0

  # Regardless of what you do, this much is true.
  F_lateral = vehicle.mass * (v0 ** 2) * segment.curvature
  F_down = vehicle.alpha_downforce() * (v0 ** 2);
  F_total_available = vehicle.mu * (vehicle.mass * vehicle.g + F_down) # Will include aero later
  
  # If there isn't even enough grip to turn, you're in a bad situation.
  if (F_lateral > F_total_available):
    return None

  # Compute remaining available tire grip
  F_ground_longitudinal_available = math.sqrt((F_total_available ** 2) - (F_lateral ** 2))

  # Accelerate, or decelerate. Why do anything else? Besides shift
  if brake:
    F_ground_longitudinal = -F_ground_longitudinal_available # Assume we can lock up tires and bias is perfect
    status = S_BRAKING
    gear = prior_result[O_GEAR]
  elif shifting == IN_PROGRESS:
    gear = -1
    F_ground_longitudinal = 0
    status = S_SHIFTING
  else:
    output_forces = [vehicle.eng_force(v0, gear_index) for gear_index in range(len(vehicle.gears))]
    gear = np.argmax(output_forces) # index tho
    F_ground_longitudinal = output_forces[gear] # Assume driver always goes hard, but never burns out
    status = S_ENG_LIM_ACC

    if F_ground_longitudinal > F_ground_longitudinal_available:
      F_ground_longitudinal = F_ground_longitudinal_available
      status = S_TIRE_LIM_ACC

  F_drag = vehicle.alpha_drag() * (v0 ** 2)
  F_longitudinal = F_ground_longitudinal - F_drag
  a = F_longitudinal / vehicle.mass

  try:
    vf = math.sqrt((v0 ** 2) + 2 * a * segment.length)
  except:
    vf = 0

  if abs(F_longitudinal) < 1e-3 and shifting != IN_PROGRESS:
    status = S_DRAG_LIM

  if vehicle.mass*vf**2*segment.curvature > vehicle.mu*(vehicle.mass*vehicle.g + vehicle.alpha_downforce()*vf**2):
    vf = v0
    status = S_SUSTAINING

  tf = t0 + segment.length / ((v0 + vf) / 2) if v0 != 0 else 0
  xf = x0 + segment.length

  return np.array([tf, xf, vf, segment.sector, status, gear, a / vehicle.g, (v0 ** 2) * segment.curvature / vehicle.g])

def solve(vehicle, segments, v0 = 0):
  # set up initial stuctures
  output = np.zeros((len(segments), 8))
  shifting = NOT_SHIFTING
  
  # get first output row
  output[0, O_VELOCITY] = v0
  step_result = step(vehicle, output[0], segments[0], False, shifting)
  output[0] = step_result

  # step loop set up
  i = 1

  # braking set up
  brake = False
  failpt = -1
  lastsafept = -1

  # shifting set up
  shift_start = 0
  last_gear = output[0, O_GEAR]
  curr_gear = last_gear
  doshift = False

  while i < len(segments):
    step_result = step(vehicle, output[i-1], segments[i], brake, shifting)

    ### BRAKING ###
    # in a message from the future we've been told to try again, this time with brakes. otherwise everything's fine
    if i < failpt:
      brake = True
      doshift = True
      
      output[i] = step_result
      i += 1

    # we caught the issue too late, we need to back up even more
    elif i == failpt and step_result is None:
      lastsafept -= 1
      i = lastsafept

    # the curve was too much, we need to back up and brake
    elif step_result is None:
      brake = True

      failpt = i
      lastsafept = i - 1
      i = lastsafept

    # nothing's wrong, just how we like it :)
    else:
      brake = False
      doshift = True
      
      output[i] = step_result
      i += 1      

    ### SHIFTING ###
    if doshift:
      i -= 1
      last_gear = curr_gear
      curr_gear = output[i, O_GEAR]

      # we just got done shifting
      if shifting == JUST_FINISHED:
        output[i - 1][O_GEAR] = curr_gear
        shifting = NOT_SHIFTING

      # we are currently shifting
      elif shifting == IN_PROGRESS:
        if output[i][O_TIME] - output[shift_start][O_TIME] >= vehicle.shift_time:
          shifting = JUST_FINISHED

      # we'd like to change gears
      elif last_gear != curr_gear:
        shifting = IN_PROGRESS
        shift_start = i
        i -= 1

      i += 1
      doshift = False

  return output

def steady_solve(vehicle,segments,v0=0):
  # TODO: Find a better way to do this #sketch
  output = solve(vehicle,segments,v0)
  return solve(vehicle,segments,output[-1, O_VELOCITY])

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
  #ax[0].plot(xaxis,t,lw=5,label='Time')
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