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
v_anchor = np.array([])

CHUNKS = 20


def serial_ports():
  """Lists serial ports
  Raises:
  EnvironmentError:
      On unsupported or unknown platforms
  Returns:
      A list of available serial ports
  """
  if sys.platform.startswith('win'):
    ports = ['COM' + str(i + 1) for i in range(256)]
  elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
      # this is to exclude your current terminal "/dev/tty"
    ports = glob.glob('/dev/tty[A-Za-z]*')
  elif sys.platform.startswith('darwin'):
    ports = glob.glob('/dev/tty.*')
  else:
    raise EnvironmentError('Unsupported platform')
  result = []
  for port in ports:
    try:
      s = serial.Serial(port)
      s.close()
      result.append(port)
    except (OSError, serial.SerialException):
      pass
  return result



def run():
  global ax,ay,az,vx,vy,vz,x,y,z,t,v_anchor
  print("reWRITE Position Reconstruction")

  ports = serial_ports()
  if ports:
    print("Available serial ports:")
    for (i,p) in enumerate(ports):
      print("%d) %s"%(i+1,p))
  else:
    print("No ports available. Check serial connection and try again.")
    print("Exiting...")
    return

  portNo = input("Select the port to use: ")
  ser = serial.Serial(ports[int(portNo)-1])
  ser.baudrate=57600 
  ser.timeout=1
  ser.write("5".encode())
  ser.readline()
  ser.timeout=5

  cur_idx = 0
  mean_x = 0
  mean_y = 0
  mean_z = 0
  alpha = 0.9
  last_ax = 0
  last_ay = 0
  last_az = 0
  last_zero = 0

  ser.flush()
  failcount = 0

  calibrated = 0
  while(calibrated != 3):
    try:
      line = ser.readline().decode().rstrip('\n')
      data = re.split(",", line)
      if (data[0][2] == '3'):
        calibrated = 1
      if (data[0][3] == '3'):
        calibrated = 2
      if (data[0][1] == '3'):
        calibrated = 3
      print(calibrated)
    except:
      pass

  print("Recording in 3...")
  time.sleep(1)
  print("2...")
  time.sleep(1)
  print("1...")
  time.sleep(1)
  print("Start")
  ser.flushInput()
  print(ser.inWaiting())
  t0 = time.time()

  while(True):
    print(cur_idx)
    ex = np.zeros(CHUNKS)
    ey = np.zeros(CHUNKS)
    ez = np.zeros(CHUNKS)
    tax = np.zeros(CHUNKS)
    tay = np.zeros(CHUNKS)
    taz = np.zeros(CHUNKS)
    count = 0
    while count < CHUNKS:
      print(count)
      if (failcount > 20):
        break
      try:
        line = ser.readline().decode().rstrip('\n')
        data = re.split(",", line)

        # Get rid of mean and threshold

        temp = float(data[1])
        if (abs(temp - mean_x) < 0.2):
          mean_x = mean_x * 0.99 + temp * 0.01
        temp = temp - mean_x if abs(temp-mean_x) > 0.15 else 0
        last_ax = last_ax*alpha + temp*(1-alpha)
        tax[count] = last_ax

        temp = float(data[2])
        if (abs(temp - mean_y) < 0.2):
          mean_y = mean_y * 0.99 + temp * 0.01
        temp = temp - mean_y if abs(temp-mean_y) > 0.15 else 0
        last_ay = last_ay*alpha + temp*(1-alpha)
        tay[count] = last_ay

        temp = float(data[3])
        if (abs(temp - mean_z) < 0.2):
          mean_z = mean_z * 0.99 + temp * 0.01
        temp = temp - mean_z if abs(temp-mean_z) > 0.15 else 0
        last_az = last_az*alpha + temp*(1-alpha)
        taz[count] = last_az

        ex[count] = float(data[4])
        ey[count] = float(data[5])
        ez[count] = float(data[6])
        t = np.append(t, int(data[8])/1000)
        count = count + 1

      except:
        failcount = failcount + 1
        pass

   
    sex = np.sin(ex)
    sey = np.sin(ey)
    sez = np.sin(ez)
    cex = np.cos(ex)
    cey = np.cos(ey)
    cez = np.cos(ez)

    # Convert to real coordinates
    ax = np.append(ax, cey*cez*tax + (sex*sey*cez - cex*sez)*tay + (cex*sey*cez + sex*sez)*taz)
    ay = np.append(ay, cey*sez*tax + (sex*sey*sez + cex*sez)*tay + (cex*sey*sez - sex*cez)*taz)
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

      timestep = (t[i] - t[i-1])

      # update velocity
      #vx[i+1] = vx[i] + ax[i]*timestep
      #vy[i+1] = vy[i] + ay[i]*timestep
      #vz[i+1] = vz[i] + az[i]*timestep
      vx = np.append(vx, ax[i-1]*timestep)
      vy = np.append(vy, ay[i-1]*timestep)
      vz = np.append(vz, az[i-1]*timestep)

      # update position
      #x[i+1] = x[i] + vx[i]*timestep
      #y[i+1] = y[i] + vy[i]*timestep
      #z[i+1] = z[i] + vz[i]*timestep
      x = np.append(x, vx[i-1]*timestep)
      y = np.append(y, vy[i-1]*timestep)
      z = np.append(z, vz[i-1]*timestep)

      if (abs(ax[i]) <= 0.2 and abs(ay[i]) <= 0.2 and abs(az[i]) <= 0.2):
        if last_zero == 0:
          last_zero = t[i]
        else:
          last_zero = 0    

      # calibrate vel if more than half a second w no accel
      v_anchor = np.append(v_anchor, [0])
      if (last_zero != 0 and (t[i] - last_zero) > 0.5):
        last_v_anchor = [j for j, e in enumerate(v_anchor) if e != 0]
        last_v_anchor = last_v_anchor[-1]
        if (i != last_v_anchor):
          for j in range(i - last_v_anchor+2):
            vx[j+last_v_anchor-1] = vx[j+last_v_anchor-1] - j/(i-last_v_anchor)*vx[i]
            vy[j+last_v_anchor-1] = vy[j+last_v_anchor-1] - j/(i-last_v_anchor)*vy[i]
            vz[j+last_v_anchor-1] = vz[j+last_v_anchor-1] - j/(i-last_v_anchor)*vz[i]
            if (j+last_v_anchor < len(x)):
              x[j+last_v_anchor] = x[j+last_v_anchor-1] + vx[j+last_v_anchor-1]*timestep
              y[j+last_v_anchor] = y[j+last_v_anchor-1] + vy[j+last_v_anchor-1]*timestep
              z[j+last_v_anchor] = z[j+last_v_anchor-1] + vz[j+last_v_anchor-1]*timestep
          #x[i] = x[i-1] + vx[i]*timestep
          #y[i] = y[i-1] + vy[i]*timestep
          #z[i] = z[i-1] + vz[i]*timestep
        last_zero = t[i]
        v_anchor[i] = 1



    cur_idx = cur_idx + CHUNKS 
    
    if (cur_idx >= 500):
      print(time.time() - t0)
      break

  m = max(max(max(abs(x)), max(abs(y))), max(abs(z)))

  fig = plt.figure()
  figA = fig.add_subplot(111, projection = "3d")
  figA.set_xlim([-m, m])
  figA.set_ylim([-m, m])
  figA.set_zlim([-m, m])
  figA.scatter(x[0],y[0],z[0])
  figA.plot(x,y,z)
  figA.set_xlabel('x')
  figA.set_ylabel('y')
  figA.set_zlabel('z')
  plt.show()

  fig = plt.figure()
  figB = fig.add_subplot(311)
  figB.plot(t, ax)
  figB.plot(t, ay)
  figB.plot(t, az)
  figB.axis('tight')
  plt.title('filtered accel')
  plt.legend(['x', 'y', 'z'],loc='center left', bbox_to_anchor=(1, 0.5))

  figC = fig.add_subplot(312)
  figC.plot(t, vx)
  figC.plot(t, vy)
  figC.plot(t, vz)
  plt.title('adjusted vel')
  figC.axis('tight')
  plt.legend(['x', 'y', 'z'],loc='center left', bbox_to_anchor=(1, 0.5))

  figD = fig.add_subplot(313)
  figD.plot(t, x)
  figD.plot(t, y)
  figD.plot(t, z)
  figD.axis('tight')
  plt.title('position')
  plt.legend(['x', 'y', 'z'],loc='center left', bbox_to_anchor=(1, 0.5))

  plt.tight_layout()
  plt.show()



run()

"""
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
#absolute_ax = cez*cex*ax + (sey*sez*cex - cey*sex)*ay + (cey*sez*cex + sey*sex)*az
#absolute_ay = cez*sex*ax + (sey*sez*sex + cey*cex)*ay + (cey*sez*sex - sey*cex)*az
#absolute_az = -sez*ax + sey*cez*ay + cey*cez*az
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

"""
