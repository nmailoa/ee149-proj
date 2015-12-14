import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
import sys
import serial
import glob
import time
import re
import os
from numpy import genfromtxt
import pickle


ax = np.array([])
ay = np.array([])
az = np.array([])
vx = np.array([])
vy = np.array([])
vz = np.array([])
x = np.array([])
y = np.array([])
z = np.array([])
t = np.array([])
tipx = np.array([])
tipy = np.array([])
forceSensor = np.array([])
len_pen = 0.1
accel_thres = 0.15
accel_time_lim = 0.1

CHUNKS = 20

scale_factor = [10/9.5, 10/7.5]



def run():
  global ax,ay,az,vx,vy,vz,x,y,z,t,v_anchor,tipx,tipy,forceSensor

  cur_idx = 0
  mean_x = 0
  mean_y = 0
  mean_z = 0
  lpf_alpha = 0.9
  hpf_alpha = 0.99
  cal_pow = 2
  last_ax = 0
  last_ay = 0
  last_az = 0
  last_zero = [1,1,1]
  first_nonzero = [0,0,0]
  base_ex = 0
  base_ey = 0
  base_ez = 0

  failcount = 0




  linearray = pickle.load(open(sys.argv[1], 'rb'))
  
  #Throw away first 20

  base_time = 0
  while(cur_idx < len(linearray)):
    print(cur_idx)
    ex = np.zeros(CHUNKS)
    ey = np.zeros(CHUNKS)
    ez = np.zeros(CHUNKS)
    tax = np.zeros(CHUNKS)
    tay = np.zeros(CHUNKS)
    taz = np.zeros(CHUNKS)
    count = 0
    while count < CHUNKS:
      if (failcount > 20):
        print("Failed more than 20 times")
        exit()
      else:
      #try:


        line = linearray[cur_idx+ count]

        force = (line[0] >> 5) & 0x1;

        axt = (line[0] & 0xf) << 8 | line[1]
        if (axt & 0x800): axt = -1*int((axt ^ 0xfff) + 1)
        axt = axt/100

        ayt = line[2] << 4 | (line[3] >> 8)
        if (ayt & 0x800): ayt = -1*int((ayt ^ 0xfff) + 1)
        ayt = ayt/100

        azt = (line[3] & 0xf) << 8 | line[4]
        if (azt & 0x800): azt = -1*int((azt ^ 0xfff) + 1)
        azt = azt/100

        ezt = line[5] << 8 | line[6]
        ezt = int(ezt)/100

        eyt = line[7] << 8 | line[8]
        if (eyt & 0x8000): eyt = -1*int((eyt ^ 0xffff) + 1)
        eyt = int(eyt)/100

        ext = line[9] << 8 | line[10]
        if (ext & 0x8000): ext = -1*int((ext ^ 0xffff) + 1)
        ext = int(ext)/100

        timetemp = int(line[11])
        if t != [] and timetemp+base_time - t[-1] < 0:
          base_time = base_time + 256
        timetemp = timetemp + base_time
        # Get rid of mean and threshold

        temp = float(axt)
        if (cur_idx == 0 and count == 0):
          mean_x = temp
        mean_x = mean_x * hpf_alpha + temp * (1-hpf_alpha)
        temp = temp - mean_x 
        last_ax = last_ax*lpf_alpha + temp*(1-lpf_alpha)
        tax[count] = last_ax if abs(last_ax) > accel_thres/3 else 0

        temp = float(ayt)
        if (cur_idx == 0 and count == 0):
          mean_y = temp
        mean_y = mean_y * hpf_alpha + temp * (1-hpf_alpha)
        temp = temp - mean_y
        last_ay = last_ay*lpf_alpha + temp*(1-lpf_alpha)
        tay[count] = last_ay if abs(last_ay) > accel_thres/3 else 0

        temp = float(azt)
        if (cur_idx == 0 and count == 0):
          mean_z = temp
        mean_z = mean_z * hpf_alpha + temp * (1-hpf_alpha)
        temp = temp - mean_z
        last_az = last_az*lpf_alpha + temp*(1-lpf_alpha)
        taz[count] = last_az if abs(last_az) > accel_thres/3 else 0

        print(str(last_ax) + '\t' + str(last_ay) + '\t' + str(last_az) + '\t' + str(timetemp) + '\t' + str(force))

        if (cur_idx == 0 and count == 0):
          ex[count] = 0
          ey[count] = 0
          ez[count] = 0
          base_ez = ezt
          base_ey = eyt
          base_ex = ext
        else:
          ez[count] = -(ezt - base_ez)/360*2*math.pi
          ey[count] = (eyt - base_ey)/360*2*math.pi
          ex[count] = (ext - base_ex)/360*2*math.pi

        
        t = np.append(t, timetemp)
        forceSensor = np.append(forceSensor, force)

        count = count + 1

      #except:
      #  failcount = failcount + 1
      #  pass

   
    sex = np.sin(ex)
    sey = np.sin(ey)
    sez = np.sin(ez)
    cex = np.cos(ex)
    cey = np.cos(ey)
    cez = np.cos(ez)

    # Convert to real coordinates
    ax = np.append(ax, cey*cez*tax + (sex*sey*cez - cex*sez)*tay + (cex*sey*cez + sex*sez)*taz)
    ay = np.append(ay, cey*sez*tax + (sex*sey*sez + cex*cez)*tay + (cex*sey*sez - sex*cez)*taz)
    az = np.append(az, -sey*tax + sex*cey*tay + cex*cey*taz)

    for i in range(cur_idx, cur_idx+CHUNKS):
      if (i == 0):
        vx = np.array([0])
        vy = np.array([0])
        vz = np.array([0])
        x = np.array([0])
        y = np.array([0])
        z = np.array([0])
        v_anchor = np.array([1])
        continue

      timestep = (t[i] - t[i-1])*15/1000

      # update velocity
      vx = np.append(vx, vx[i-1]+ax[i-1]*timestep)
      vy = np.append(vy, vy[i-1]+ay[i-1]*timestep)
      vz = np.append(vz, vz[i-1]+az[i-1]*timestep)

      # update position
      x = np.append(x, x[i-1]+vx[i-1]*timestep)
      y = np.append(y, y[i-1]+vy[i-1]*timestep)
      z = np.append(z, z[i-1]+vz[i-1]*timestep)

      if (abs(ax[i]) <= accel_thres):
        if first_nonzero[0] != -1 and last_zero[0] == -1:
          last_zero[0] = i   
      else:
        last_zero[0] = -1    
        if first_nonzero[0] == -1:
          first_nonzero[0] = i   

      # calibrate vel if more than half a second w no accel
      if (last_zero[0] != -1 and (t[i] - t[last_zero[0]]) > accel_time_lim*1000/15):
        for j in range(first_nonzero[0], last_zero[0]+1):
          timestep = (t[j] - t[j-1])*15/1000
          vx[j] = vx[j] - ((j-first_nonzero[0])/(last_zero[0]-first_nonzero[0]))**cal_pow*vx[last_zero[0]]
          x[j] = x[j-1] + vx[j-1]*timestep
        for j in range(last_zero[0], i+1):
          vx[j] = 0
          x[j] = x[j-1]
        first_nonzero[0] = i-1
        last_zero[0] = -1




      if (abs(ay[i]) <= accel_thres):
        if first_nonzero[1] != -1 and last_zero[1] == -1:
          last_zero[1] = i   
      else:
        last_zero[1] = -1    
        if first_nonzero[1] == -1:
          first_nonzero[1] = i   

      # calibrate vel if more than half a second w no accel
      if (last_zero[1] != -1 and (t[i] - t[last_zero[1]]) > accel_time_lim*1000/15):
        for j in range(first_nonzero[1], last_zero[1]+1):
          timestep = (t[j] - t[j-1])*15/1000
          vy[j] = vy[j] - ((j-first_nonzero[1])/(last_zero[1]-first_nonzero[1]))**cal_pow*vy[last_zero[1]]
          y[j] = y[j-1] + vy[j-1]*timestep
        for j in range(last_zero[1], i+1):
          vy[j] = 0
          y[j] = y[j-1]
        first_nonzero[1] = i-1
        last_zero[1] = -1




      if (abs(az[i]) <= accel_thres):
        if first_nonzero[2] != -1 and last_zero[2] == -1:
          last_zero[2] = i   
      else:
        last_zero[2] = -1    
        if first_nonzero[2] == -1:
          first_nonzero[2] = i   

      # calibrate vel if more than half a second w no accel
      if (last_zero[2] != -1 and (t[i] - t[last_zero[2]]) > accel_time_lim*1000/15):
        for j in range(first_nonzero[2], last_zero[2]+1):
          timestep = (t[j] - t[j-1])*15/1000
          vz[j] = vz[j] - ((j-first_nonzero[2])/(last_zero[2]-first_nonzero[2]))**cal_pow*vz[last_zero[2]]
          z[j] = z[j-1] + vz[j-1]*timestep
        for j in range(last_zero[2], i+1):
          vz[j] = 0
          z[j] = z[j-1]
        first_nonzero[2] = i-1
        last_zero[2] = -1       
    
    
    tipx = np.append(tipx, x[cur_idx:] - len_pen*sey)
    tipy = np.append(tipy, y[cur_idx:] - len_pen*sex)

    m = 10000
    #figA.set_xlim([-m, m])
    #figA.set_ylim([-m, m])
    #figA.cla()
    #starts = np.where(forceSensor[1:] - forceSensor[:-1] == 1)[0]
    #ends = np.where(forceSensor[1:] - forceSensor[:-1] == -1)[0]
    #print(starts)
    #for a,j in enumerate(starts):
    #  if (a != len(starts) - 1 and j - ends[a] < 10):
    #    pass
    #  if a == len(starts) - 1:
    #    figA.plot(tipx[j:],tipy[j:])
    #  else:
    #    figA.plot(tipx[j:ends[a]],tipy[j:ends[a]])
    cur_idx = cur_idx + CHUNKS 



      

run()

x = x * scale_factor[0]
y = y * scale_factor[1]

fig = plt.figure()
#figA = fig.add_subplot(111, projection = "3d")
figA = fig.add_subplot(111)
figA.set_xlabel('x')
figA.set_ylabel('y') 
print(x)
print(y)
figA.plot(x,y)
plt.axis('equal')
plt.show()


time = t*15/1000

fig = plt.figure()
#plt.plot(x)
#plt.plot(y)
#plt.plot(z)
#plt.plot(vx)
#plt.plot(vy)
#plt.plot(vz)
#figA = fig.add_subplot(321)
#figA.plot(time, ori_ax)
#figA.plot(time, ori_ay)
#figA.plot(time, ori_az)
#plt.xlim([0,100])
#figA.axis('tight')
#plt.title('raw accel')
#plt.legend(['x', 'y', 'z'],loc='center left', bbox_to_anchor=(1, 0.5))

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

figB = fig.add_subplot(311)
figB.plot(time, ax)
figB.plot(time, ay)
figB.plot(time, az)
plt.xlim([0,100])
figB.axis('tight')
plt.title('absolute accel')
plt.legend(['x', 'y', 'z'],loc='center left', bbox_to_anchor=(1, 0.5))

#figC = fig.add_subplot(323)
#figC.plot(time, ori_vx)
#figC.plot(time, ori_vy)
#figC.plot(time, ori_vz)
#plt.title('unadjusted vel')
#figC.axis('tight')
#plt.legend(['x', 'y', 'z'],loc='center left', bbox_to_anchor=(1, 0.5))

figC = fig.add_subplot(312)
figC.plot(time, vx)
figC.plot(time, vy)
figC.plot(time, vz)
plt.title('adjusted vel')
figC.axis('tight')
plt.legend(['x', 'y', 'z'],loc='center left', bbox_to_anchor=(1, 0.5))

#figD = fig.add_subplot(325)
#figD.plot(time, ori_x)
#figD.plot(time, ori_y)
#figD.plot(time, ori_z)
#figD.axis('tight')
#plt.title('unadjusted position')
#plt.legend(['x', 'y', 'z'],loc='center left', bbox_to_anchor=(1, 0.5))


figD = fig.add_subplot(313)
figD.plot(time, x)
figD.plot(time, y)
figD.plot(time, z)
figD.axis('tight')
plt.title('adjusted position')
plt.legend(['x', 'y', 'z'],loc='center left', bbox_to_anchor=(1, 0.5))

plt.tight_layout()
plt.show()
