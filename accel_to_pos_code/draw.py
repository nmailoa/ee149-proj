import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

from numpy import genfromtxt

data = genfromtxt('v.csv', delimiter=',')
ax = np.round((data[:,1]-data[0,1])*5)/5
ay = np.round((data[:,2]-data[0,2])*5)/5
az = -np.round((data[:,3]-data[0,3])*5)/5
ez = -(data[:,4]-data[0,4])/360*2*math.pi
ey = (data[:,5]-data[0,5])/360*2*math.pi
ex = (data[:,6]-data[0,6])/360*2*math.pi
time = data[:,7]/1000

print(az)

sex = np.sin(ex)
sey = np.sin(ey)
sez = np.sin(ez)
cex = np.cos(ex)
cey = np.cos(ey)
cez = np.cos(ez)

ori_ax

for i in range(len(ax)-1):
  ax[i+1] = ax[i]*0.8 + ax[i+1]*0.2
  ay[i+1] = ay[i]*0.8 + ay[i+1]*0.2
  az[i+1] = az[i]*0.8 + az[i+1]*0.2

absolute_ax = cey*cez*ax + (sex*sey*cez - cex*sez)*ay + (cex*sey*cez + sex*sez)*az
absolute_ay = cey*sez*ax + (sex*sey*sez + cex*sez)*ay + (cex*sey*sez - sex*cez)*az
absolute_az = -sey*ax + sex*cey*ay + cex*cey*az


vx = np.zeros(len(ax))
vy = np.zeros(len(ay))
vz = np.zeros(len(az))

x = np.zeros(len(ax))
y = np.zeros(len(ax))
z = np.zeros(len(ax))

anchor = np.zeros(len(ax))
anchor[0] = 1

fig = plt.figure()
figA = fig.add_subplot(111, projection = "3d")
figA.scatter(x[0],y[0],z[0])
#figA.view_init(elev=20, azim=47)

last_zero = time[0];

for i in range(len(ax)-1):
  t = (time[i+1] - time[i])


  # update velocity
  vx[i+1] = vx[i] + absolute_ax[i]*t
  vy[i+1] = vy[i] + absolute_ay[i]*t
  vz[i+1] = vz[i] + absolute_az[i]*t
  # update position
  x[i+1] = x[i] + vx[i]*t
  y[i+1] = y[i] + vy[i]*t
  z[i+1] = z[i] + vz[i]*t

  if (vx[i+1] <= 0.1 and vy[i+1] <= 0.1 and vz[i+1] <= 0.1):
    if last_zero == 0:
      last_zero = time[i]
      anchor[i] = 1

  else:
    last_zero = 0    

  """
  # calibrate vel if more than half a second w no accel
  if ((time[i+1] - last_zero) > .5):
    last_anchor = [i for i, e in enumerate(anchor) if e != 0]
    last_anchor = last_anchor[-1]
    print(vz[i])
    for j in range(i - last_anchor+3):
      vx[j+last_anchor-1] = vx[j+last_anchor-1] - j/(i-last_anchor)*vx[i+1]
      vy[j+last_anchor-1] = vy[j+last_anchor-1] - j/(i-last_anchor)*vy[i+1]
      vz[j+last_anchor-1] = vz[j+last_anchor-1] - j/(i-last_anchor)*vz[i+1]
      x[j+last_anchor] = x[j+last_anchor-1] + vx[j+last_anchor-1]*t
      y[j+last_anchor] = y[j+last_anchor-1] + vy[j+last_anchor-1]*t
      z[j+last_anchor] = z[j+last_anchor-1] + vz[j+last_anchor-1]*t
    print(vz[i-1])
    x[i+1] = x[i] + vx[i]*t
    y[i+1] = y[i] + vy[i]*t
    z[i+1] = z[i] + vz[i]*t
    last_zero = time[i+1]
    anchor[i] = 1
    print("calibrate " + str(i))

      
  #figA.scatter(x[i+1],y[i+1],z[i+1])
  #figA.plot([x[i],x[i+1]],[y[i],y[i+1]],[z[i],z[i+1]])
  """

figA.plot(x,y,z)
figA.set_xlabel('x')
figA.set_ylabel('y')
figA.set_zlabel('z')
plt.show()

fig = plt.figure()
#plt.plot(x)
#plt.plot(y)
#plt.plot(z)
#plt.plot(vx)
#plt.plot(vy)
#plt.plot(vz)
figA = fig.add_subplot(311)
figA.plot(ax)
figA.plot(ay)
figA.plot(az)
plt.legend(['ax', 'ay', 'az'],loc='center left', bbox_to_anchor=(1, 0.5))

figB = fig.add_subplot(312)
figB.plot(absolute_ax)
figB.plot(absolute_ay)
figB.plot(absolute_az)
plt.legend(['absolute_ax', 'absolute_ay', 'absolute_az'],loc='center left', bbox_to_anchor=(1, 0.5))


figC = fig.add_subplot(313)
figC.plot(vx)
figC.plot(vy)
figC.plot(vz)
plt.legend(['vx', 'vy', 'vz'],loc='center left', bbox_to_anchor=(1, 0.5))

fig = plt.figure()
figD = fig.add_subplot(111)
figD.plot(x)
figD.plot(y)
figD.plot(z)
plt.legend(['x', 'y', 'z'],loc='center left', bbox_to_anchor=(1, 0.5))
plt.show()
