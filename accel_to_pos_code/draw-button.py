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
button = data[:,7]
time = data[:,8]/1000

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

v_anchor = np.zeros(len(ax))
v_anchor[0] = 1

p_anchor = np.zeros(len(ax))
p_anchor[0] = 1


last_zero = time[0];

# No correction
for i in range(len(ax)-2):
  t = (time[i+1] - time[i])
  # update velocity
  vx[i+1] = vx[i] + absolute_ax[i]*t
  vy[i+1] = vy[i] + absolute_ay[i]*t
  vz[i+1] = vz[i] + absolute_az[i]*t

ori_vx = np.copy(vx)
ori_vy = np.copy(vy)
ori_vz = np.copy(vz)
  


vx = np.zeros(len(ax))
vy = np.zeros(len(ay))
vz = np.zeros(len(az))

x = np.zeros(len(ax))
y = np.zeros(len(ax))
z = np.zeros(len(ax))

v_anchor = np.zeros(len(ax))
v_anchor[0] = 1

p_anchor = np.zeros(len(ax))
p_anchor[0] = 1

# Velocity correction
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
    last_v_anchor = [i for i, e in enumerate(v_anchor) if e != 0]
    last_v_anchor = last_v_anchor[-1]
    found = 0
    idx = last_v_anchor
    while(not found):
      if (abs(vx[idx+1]-x[idx]) < 0.01 and abs(y[idx+1]-y[idx]) < 0.01 and abs(z[idx+1]-z[idx]) < 0.01 and idx < i):
        idx = idx + 1
      else:
        found = 1
    last_v_anchor = idx
    if (i != last_v_anchor):
      for j in range(i - last_v_anchor+3):
        vx[j+last_v_anchor-1] = vx[j+last_v_anchor-1] - j/(i-last_v_anchor)*vx[i+1]
        vy[j+last_v_anchor-1] = vy[j+last_v_anchor-1] - j/(i-last_v_anchor)*vy[i+1]
        vz[j+last_v_anchor-1] = vz[j+last_v_anchor-1] - j/(i-last_v_anchor)*vz[i+1]
        x[j+last_v_anchor] = x[j+last_v_anchor-1] + vx[j+last_v_anchor-1]*t
        y[j+last_v_anchor] = y[j+last_v_anchor-1] + vy[j+last_v_anchor-1]*t
        z[j+last_v_anchor] = z[j+last_v_anchor-1] + vz[j+last_v_anchor-1]*t
      x[i+1] = x[i] + vx[i]*t
      y[i+1] = y[i] + vy[i]*t
      z[i+1] = z[i] + vz[i]*t
    last_zero = time[i+1]
    v_anchor[i] = 1

ori_x = np.copy(x)
ori_y = np.copy(y)
ori_z = np.copy(z)




vx = np.zeros(len(ax))
vy = np.zeros(len(ay))
vz = np.zeros(len(az))

x = np.zeros(len(ax))
y = np.zeros(len(ax))
z = np.zeros(len(ax))

v_anchor = np.zeros(len(ax))
v_anchor[0] = 1

p_anchor = np.zeros(len(ax))
p_anchor[0] = 1

# Velocity and position correction
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
    last_v_anchor = [i for i, e in enumerate(v_anchor) if e != 0]
    last_v_anchor = last_v_anchor[-1]
    found = 0
    idx = last_v_anchor
    while(not found):
      if (abs(vx[idx+1]-x[idx]) < 0.01 and abs(y[idx+1]-y[idx]) < 0.01 and abs(z[idx+1]-z[idx]) < 0.01 and idx < i):
        idx = idx + 1
      else:
        found = 1
    last_v_anchor = idx
    if (i != last_v_anchor):
      for j in range(i - last_v_anchor+3):
        vx[j+last_v_anchor-1] = vx[j+last_v_anchor-1] - j/(i-last_v_anchor)*vx[i+1]
        vy[j+last_v_anchor-1] = vy[j+last_v_anchor-1] - j/(i-last_v_anchor)*vy[i+1]
        vz[j+last_v_anchor-1] = vz[j+last_v_anchor-1] - j/(i-last_v_anchor)*vz[i+1]
        x[j+last_v_anchor] = x[j+last_v_anchor-1] + vx[j+last_v_anchor-1]*t
        y[j+last_v_anchor] = y[j+last_v_anchor-1] + vy[j+last_v_anchor-1]*t
        z[j+last_v_anchor] = z[j+last_v_anchor-1] + vz[j+last_v_anchor-1]*t
      x[i+1] = x[i] + vx[i]*t
      y[i+1] = y[i] + vy[i]*t
      z[i+1] = z[i] + vz[i]*t
    last_zero = time[i+1]
    v_anchor[i] = 1

  if (button[i-1]==0 and button[i]==1 ):
    last_p_anchor = [i for i, e in enumerate(p_anchor) if e != 0]
    last_p_anchor = last_p_anchor[-1]
    found = 0
    idx = i
    while(not found):
      if (abs(x[idx+1]-x[idx]) < 0.001 and abs(y[idx+1]-y[idx]) < 0.001 and abs(z[idx+1]-z[idx]) < 0.001 and idx > 0):
        idx = idx - 1
      else:
        found = 1

    found = 0
    idx2 = last_p_anchor
    while(not found):
      if (abs(x[idx2+1]-x[idx2]) < 0.001 and abs(y[idx2+1]-y[idx2]) < 0.001 and abs(z[idx2+1]-z[idx2]) < 0.001 and idx2 < idx):
        idx2 = idx2 + 1
      else:
        found = 1

    print(time[i])
    print(time[last_p_anchor])
    print(time[idx])
    print(time[idx2])
    orix = x[idx]
    oriy = y[idx]
    oriz = z[idx]

    if (idx != idx2):
      for j in range(idx - idx2+1):
        x[j+idx2] = x[j+idx2] - j/(idx-idx2)*(x[idx+1]-x[idx2])
        y[j+idx2] = y[j+idx2] - j/(idx-idx2)*(y[idx+1]-y[idx2])
        z[j+idx2] = z[j+idx2] - j/(idx-idx2)*(z[idx+1]-z[idx2])
    if (idx != i):
      for j in range(i - idx+2):
        print("modifying2 " + str(time[j+idx-1]))
        x[j+idx] = 0
        y[j+idx] = 0
        z[j+idx] = 0
    p_anchor[i] = 1

  #elif (p_anchor[i-1] and abs(x[i+1]-x[i]) < 0.01 and abs(y[i+1]-y[i]) < 0.01 and abs(z[i+1]-z[i]) < 0.01):
  #  p_anchor[i] = 1
      
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

"""
figB = fig.add_subplot(322)
figB.plot(time, ax)
figB.plot(time, ay)
figB.plot(time, az)
#plt.xlim([0,100])
figB.axis('tight')
plt.title('filtered accel')
plt.legend(['x', 'y', 'z'],loc='center left', bbox_to_anchor=(1, 0.5))
"""

figB = fig.add_subplot(322)
figB.plot(time, absolute_ax)
figB.plot(time, absolute_ay)
figB.plot(time, absolute_az)
#plt.xlim([0,100])
figB.axis('tight')
plt.title('absolute accel')
plt.legend(['x', 'y', 'z'],loc='center left', bbox_to_anchor=(1, 0.5))

figC = fig.add_subplot(323)
figC.plot(time, ori_vx)
figC.plot(time, ori_vy)
figC.plot(time, ori_vz)
plt.title('unadjusted vel')
figC.axis('tight')
plt.legend(['x', 'y', 'z'],loc='center left', bbox_to_anchor=(1, 0.5))

figC = fig.add_subplot(324)
figC.plot(time, vx)
figC.plot(time, vy)
figC.plot(time, vz)
plt.title('adjusted vel')
figC.axis('tight')
plt.legend(['x', 'y', 'z'],loc='center left', bbox_to_anchor=(1, 0.5))

figD = fig.add_subplot(325)
figD.plot(time, ori_x)
figD.plot(time, ori_y)
figD.plot(time, ori_z)
figD.axis('tight')
plt.title('unadjusted position')
plt.legend(['x', 'y', 'z'],loc='center left', bbox_to_anchor=(1, 0.5))


figD = fig.add_subplot(326)
figD.plot(time, x)
figD.plot(time, y)
figD.plot(time, z)
figD.axis('tight')
plt.title('adjusted position')
plt.legend(['x', 'y', 'z'],loc='center left', bbox_to_anchor=(1, 0.5))

plt.tight_layout()
plt.show()
