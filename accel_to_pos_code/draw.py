import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

from numpy import genfromtxt

data = genfromtxt('v.csv', delimiter=',')
ax = np.round(data[:,1]-data[0,1],1)
ay = np.round(data[:,2]-data[0,2],1)
az = -np.round(data[:,3]-data[0,3],1)
ez = -(data[:,4]-data[0,4])/360*2*math.pi
ey = (data[:,5]-data[0,5])/360*2*math.pi
ex = (data[:,6]-data[0,6])/360*2*math.pi
time = data[:,7]/100

print(az)

sex = np.sin(ex)
sey = np.sin(ey)
sez = np.sin(ez)
cex = np.cos(ex)
cey = np.cos(ey)
cez = np.cos(ez)

absolute_ax = cey*cez*ax + (sex*sey*cez - cex*sez)*ay + (cex*sey*cez + sex*sez)*az
absolute_ay = cey*sez*ax + (sex*sey*sez + cex*sez)*ay + (cex*sey*sez - sex*cez)*az
absolute_az = -sey*ax + sex*cey*ay + cex*cey*az

vx = np.zeros(len(ax))
vy = np.zeros(len(ay))
vz = np.zeros(len(az))

x = np.zeros(len(ax))
y = np.zeros(len(ax))
z = np.zeros(len(ax))

fig = plt.figure()
figA = fig.add_subplot(111, projection = "3d")
figA.scatter(x[0],y[0],z[0])
#figA.view_init(elev=20, azim=47)

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
      
  #figA.scatter(x[i+1],y[i+1],z[i+1])
  #figA.plot([x[i],x[i+1]],[y[i],y[i+1]],[z[i],z[i+1]])



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
plt.legend(['ax', 'ay', 'az'])

figB = fig.add_subplot(312)
figB.plot(absolute_ax)
figB.plot(absolute_ay)
figB.plot(absolute_az)
plt.legend(['absolute_ax', 'absolute_ay', 'absolute_az'])


figC = fig.add_subplot(313)
figC.plot(vx)
figC.plot(vy)
figC.plot(vz)

plt.legend(['vx', 'vy', 'vz'])
plt.show()
