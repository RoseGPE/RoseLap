import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.cm as cmx
import matplotlib.colors as colors
import sim

def plot_velocity_and_events(output, axis='x', title='Velocity and Events'):
  fig, ax = plt.subplots(2, sharex=True)
  fig.canvas.set_window_title(title)

  fig.suptitle(title)

  t = output[:, sim.O_TIME]
  x = output[:, sim.O_DISTANCE]
  v = output[:, sim.O_VELOCITY]

  sectors = output[:, sim.O_SECTORS]
  status = output[:, sim.O_STATUS]
  gear = output[:, sim.O_GEAR]

  along = output[:, sim.O_LONG_ACC]
  alat = output[:, sim.O_LAT_ACC]
  eng_rpm = output[:, sim.O_ENG_RPM]

  curv = output[:, sim.O_CURVATURE]*100

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
  ax[1].plot(xaxis,eng_rpm/1000, lw=4, label='RPM x1000')

  lim = max(v)
  alpha =  1

  ax[0].fill_between(xaxis, 0, lim, where= status==sim.S_BRAKING,      facecolor='#e22030', alpha=alpha)
  ax[0].fill_between(xaxis, 0, lim, where= status==sim.S_ENG_LIM_ACC,  facecolor='#50d21d', alpha=alpha)
  ax[0].fill_between(xaxis, 0, lim, where= status==sim.S_TIRE_LIM_ACC, facecolor='#1d95d2', alpha=alpha)
  ax[0].fill_between(xaxis, 0, lim, where= status==sim.S_SUSTAINING,   facecolor='#d2c81c', alpha=alpha)
  ax[0].fill_between(xaxis, 0, lim, where= status==sim.S_DRAG_LIM,     facecolor='#e2952b', alpha=alpha)
  ax[0].fill_between(xaxis, 0, lim, where= status==sim.S_SHIFTING,     facecolor='#454545', alpha=alpha)
  ax[0].fill_between(xaxis, 0, lim, where= status==sim.S_TOPPED_OUT,   facecolor='#7637a2', alpha=alpha)

  sector = sectors[0]
  for idx,sec in enumerate(sectors):
    if sec!=sector:
      ax[0].axvline(xaxis[idx], color='black', lw=2, alpha=0.9)
      sector=sec
  ax[0].set_ylim((0,lim+1))
  #ax[1].set_ylim((min((min(along),min(alat)))-0.1,0.1+max((max(along),max(alat)))))
  ax[1].set_ylim(-5,12)
  plt.xlim((0,max(xaxis)))

  #sectors = set(output[:,3])
  #for sector in sectors:
  #  ax.fill_between(t, -100, 100, where=output[:,3]==sector, facecolor=colorgen(len(sectors), sector), alpha=0.3)

  
  ax[0].grid(True)
  ax[0].legend()
  ax[1].legend()

  plt.draw()


class DetailZoom:
  def __init__(self, record, seg_no):
    self.record = record
    self.outputs = record.output
    self.seg_no = seg_no

  def onpick(self, event):
    # get mouse data
    x = event.mouseevent.xdata
    y = event.mouseevent.ydata
    

    # find closest point
    distances = []
    if self.record.kind == "2D":
      print('okay now')
      distances = np.array([abs(p - x) for p in self.record.plot_points])
      minXIndex = distances.argmin()
      # if distances[minXIndex] > 0.1:
      #   print('x prob')
      #   return

      relevantTimes = np.transpose(self.record.times[:, minXIndex])
      distances = np.array([abs(t - y) for t in relevantTimes])
      minYIndex = distances.argmin()
      # if distances[minYIndex] > 0.1:
      #   print('y prob')
      #   return


      outputIndex = minYIndex * len(self.record.plot_points) + minXIndex
      title = 'Details for ' + self.record.track[minYIndex] + ', ' + self.record.plot_x_label + '= ' + ("%.3f"%self.record.plot_points[minXIndex]) + " (" + ("%.3f"%self.outputs[outputIndex][-1,sim.O_TIME]) + "s)"
      print(title)
      self.plotDetail(outputIndex, title)

    elif self.record.kind == "3D":
      offset = len(self.record.plot_x_points)*len(self.record.plot_y_points)*self.seg_no
      distances = np.array([abs(p - x) for p in self.record.plot_x_points])
      minXIndex = distances.argmin()
      # if distances[minXIndex] > 0.1:
      #   return

      distances = np.array([abs(p - y) for p in self.record.plot_y_points])
      minYIndex = distances.argmin()
      # if distances[minYIndex] > 0.1:
      #   return

      outputIndex = minXIndex * len(self.record.plot_y_points) + minYIndex + offset
      title = 'Details for ' + self.record.track[self.seg_no] + ', ' + self.record.plot_x_label + "= " + ("%.3f"%self.record.plot_x_points[minXIndex]) + ', ' +  self.record.plot_y_label + ": " + ("%.3f"%self.record.plot_y_points[minYIndex]) + " (" + ("%.3f"%self.outputs[outputIndex][-1,sim.O_TIME]) + "s)"
      self.plotDetail(outputIndex, title)

  def plotDetail(self, i, title='Details'):
    plot_velocity_and_events(self.outputs[i], title=title)
    plt.show()
