import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
import sys

from numpy import genfromtxt

if (len(sys.argv) != 2):
  print("Usage: python draw.py filename.csv")

data = genfromtxt(sys.argv[1], delimiter=',')
#ax = np.round((data[:,1]-data[0,1])*5)/5
#ay = np.round((data[:,2]-data[0,2])*5)/5
#az = -np.round((data[:,3]-data[0,3])*5)/5
ax = (data[:,1]-data[0,1])
ay = (data[:,2]-data[0,2])
az = (data[:,3]-data[0,3])
ez = -(data[:,4]-data[0,4])/360*2*math.pi
ey = (data[:,5]-data[0,5])/360*2*math.pi
ex = (data[:,6]-data[0,6])/360*2*math.pi
time = data[:,7]/1000

sex = np.sin(ex)
sey = np.sin(ey)
sez = np.sin(ez)
cex = np.cos(ex)
cey = np.cos(ey)
cez = np.cos(ez)

ori_ax = np.copy(ax)
ori_ay = np.copy(ay)
ori_az = np.copy(az)

# 2nd level calibration
mean_x = 0
mean_y = 0
mean_z = 0
for i in range(len(ax)-1):
  if (abs(ax[i+1] - mean_x) < 0.2):
    mean_x = mean_x * 0.99 + ax[i+1] * 0.01
  if (abs(ay[i+1] - mean_y) < 0.2):
    mean_y = mean_y * 0.99 + ay[i+1] * 0.01
  if (abs(az[i+1] - mean_z) < 0.2):
    mean_z = mean_z * 0.99 + az[i+1] * 0.01
  ax[i+1] = ax[i+1] - mean_x
  ay[i+1] = ay[i+1] - mean_y
  az[i+1] = az[i+1] - mean_z


ax = [i if (abs(i)>0.15) else 0 for i in ax]
ay = [i if (abs(i)>0.15) else 0 for i in ay]
az = [i if (abs(i)>0.15) else 0 for i in az]

# IIR low pass
alpha = .9
for i in range(len(ax)-1):
  ax[i+1] = ax[i]*alpha + ax[i+1]*(1-alpha)
  ay[i+1] = ay[i]*alpha + ay[i+1]*(1-alpha)
  az[i+1] = az[i]*alpha + az[i+1]*(1-alpha)

#ax = ax - np.mean(ax)
#ay = ay - np.mean(ay)
#az = az - np.mean(az)


# Move to real coordinates
absolute_ax = cez*cex*ax + (sey*sez*cex - cey*sex)*ay + (cey*sez*cex + sey*sex)*az
absolute_ay = cez*sex*ax + (sey*sez*sex + cey*cex)*ay + (cey*sez*sex - sey*cex)*az
absolute_az = -sez*ax + sey*cez*ay + cey*cez*az
#absolute_ax = cey*cez*ax + (sex*sey*cez - cex*sez)*ay + (cex*sey*cez + sex*sez)*az
#absolute_ay = cey*sez*ax + (sex*sey*sez + cex*sez)*ay + (cex*sey*sez - sex*cez)*az
#absolute_az = -sey*ax + sex*cey*ay + cex*cey*az


vx = np.zeros(len(ax))
vy = np.zeros(len(ay))
vz = np.zeros(len(az))

x = np.zeros(len(ax))
y = np.zeros(len(ax))
z = np.zeros(len(ax))

anchor = np.zeros(len(ax))
anchor[0] = 1



last_zero = time[0];


for i in range(len(ax)-2):
  t = (time[i+1] - time[i])
  # update velocity
  vx[i+1] = vx[i] + absolute_ax[i]*t
  vy[i+1] = vy[i] + absolute_ay[i]*t
  vz[i+1] = vz[i] + absolute_az[i]*t

ori_vx = np.copy(vx)
ori_vy = np.copy(vy)
ori_vz = np.copy(vz)
  

for i in range(len(ax)-2):
  t = (time[i+1] - time[i])

  # update velocity
  vx[i+1] = vx[i] + absolute_ax[i]*t
  vy[i+1] = vy[i] + absolute_ay[i]*t
  vz[i+1] = vz[i] + absolute_az[i]*t

  # update position
  x[i+1] = x[i] + vx[i]*t
  y[i+1] = y[i] + vy[i]*t
  z[i+1] = z[i] + vz[i]*t

  if (abs(absolute_ax[i+1]) <= 0.2 and abs(absolute_ay[i+1]) <= 0.2 and abs(absolute_az[i+1]) <= 0.2):
    if last_zero == 0:
      last_zero = time[i]
  else:
    last_zero = 0    

  # calibrate vel if more than half a second w no accel
  if (last_zero != 0 and (time[i+1] - last_zero) > 0.5):
    print("calibrate " + str(time[i+1]))
    last_anchor = [i for i, e in enumerate(anchor) if e != 0]
    last_anchor = last_anchor[-1]
    for j in range(i - last_anchor+3):
      vx[j+last_anchor-1] = vx[j+last_anchor-1] - j/(i-last_anchor)*vx[i+1]
      vy[j+last_anchor-1] = vy[j+last_anchor-1] - j/(i-last_anchor)*vy[i+1]
      vz[j+last_anchor-1] = vz[j+last_anchor-1] - j/(i-last_anchor)*vz[i+1]
      x[j+last_anchor] = x[j+last_anchor-1] + vx[j+last_anchor-1]*t
      y[j+last_anchor] = y[j+last_anchor-1] + vy[j+last_anchor-1]*t
      z[j+last_anchor] = z[j+last_anchor-1] + vz[j+last_anchor-1]*t
    x[i+1] = x[i] + vx[i]*t
    y[i+1] = y[i] + vy[i]*t
    z[i+1] = z[i] + vz[i]*t
    last_zero = time[i+1]
    anchor[i] = 1

      
  #figA.scatter(x[i+1],y[i+1],z[i+1])
  #figA.plot([x[i],x[i+1]],[y[i],y[i+1]],[z[i],z[i+1]])

m = max(max(max(abs(x)), max(abs(y))), max(abs(z)))

fig = plt.figure()
figA = fig.add_subplot(111, projection = "3d")
figA.set_xlim([-m, m])
figA.set_ylim([-m, m])
figA.set_zlim([-m, m])
figA.scatter(x[0],y[0],z[0])
#figA.view_init(elev=20, azim=47)

figA.plot(x[:-1],y[:-1],z[:-1])
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
figA = fig.add_subplot(321)
figA.plot(time, ori_ax)
figA.plot(time, ori_ay)
figA.plot(time, ori_az)
#plt.xlim([0,100])
figA.axis('tight')
plt.title('raw accel')
plt.legend(['x', 'y', 'z'],loc='center left', bbox_to_anchor=(1, 0.5))

figB = fig.add_subplot(322)
figB.plot(time, ax)
figB.plot(time, ay)
figB.plot(time, az)
#plt.xlim([0,100])
figB.axis('tight')
plt.title('filtered accel')
plt.legend(['x', 'y', 'z'],loc='center left', bbox_to_anchor=(1, 0.5))

figB = fig.add_subplot(323)
figB.plot(time, absolute_ax)
figB.plot(time, absolute_ay)
figB.plot(time, absolute_az)
#plt.xlim([0,100])
figB.axis('tight')
plt.title('absolute accel')
plt.legend(['x', 'y', 'z'],loc='center left', bbox_to_anchor=(1, 0.5))

figC = fig.add_subplot(324)
figC.plot(time, ori_vx)
figC.plot(time, ori_vy)
figC.plot(time, ori_vz)
plt.title('unadjusted vel')
figC.axis('tight')
plt.legend(['x', 'y', 'z'],loc='center left', bbox_to_anchor=(1, 0.5))

figC = fig.add_subplot(325)
figC.plot(time, vx)
figC.plot(time, vy)
figC.plot(time, vz)
plt.title('adjusted vel')
figC.axis('tight')
plt.legend(['x', 'y', 'z'],loc='center left', bbox_to_anchor=(1, 0.5))

figD = fig.add_subplot(326)
figD.plot(time, x)
figD.plot(time, y)
figD.plot(time, z)
figD.axis('tight')
plt.title('position')
plt.legend(['x', 'y', 'z'],loc='center left', bbox_to_anchor=(1, 0.5))

plt.tight_layout()
plt.show()
