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
  base_ex = 0
  base_ey = 0
  base_ez = 0

  ser.flush()
  failcount = 0

  calibrated = 0
  while(calibrated != 3):
    try:
      line = ser.readline().decode().rstrip('\n')
      data = re.split(",", line)
      if (data[0][2] == '3'):
        calibrated = 1
      if (calibrated == 1 and data[0][3] == '3'):
        calibrated = 2
      if (calibrated == 2 and data[0][1] == '3'):
        calibrated = 3
      print(data[0])
    except:
      pass

  # plt.ion()
  # fig = plt.figure()
  # figA = fig.add_subplot(111, projection = "3d")
  # figA.set_xlabel('x')
  # figA.set_ylabel('y')
  # figA.set_zlabel('z')

  print("Recording in 3...")
  time.sleep(1)
  print("2...")
  time.sleep(1)
  print("1...")
  time.sleep(1)
  print("Start")
  ser.write('1'.encode())
  ser.flushInput()
  print(ser.inWaiting())
  t0 = time.time()


  data_file = open('data', 'w')
  plt.ion()
  fig = plt.figure()
  #figA = fig.add_subplot(111, projection = "3d")
  figA = fig.add_subplot(111)
  figA.set_xlabel('x')
  figA.set_ylabel('y')

  
  
  #Throw away first 20
  for i in range(20):
    line = ser.readline()

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
      if (failcount > 20):
        break
      try:
        line = ser.readline().decode().rstrip('\n')
        data_file.write(line)
        data_file.write('\n')
        data = re.split(",", line)

        # Get rid of mean and threshold

        temp = float(data[1])
        #if (abs(temp - mean_x) < 0.2):
        mean_x = mean_x * 0.999 + temp * 0.001
        temp = temp - mean_x if abs(temp-mean_x) > 0.15 else 0
        last_ax = last_ax*alpha + temp*(1-alpha)
        tax[count] = last_ax

        temp = float(data[2])
        #if (abs(temp - mean_y) < 0.2):
        mean_y = mean_y * 0.999 + temp * 0.001
        temp = temp - mean_y if abs(temp-mean_y) > 0.15 else 0
        last_ay = last_ay*alpha + temp*(1-alpha)
        tay[count] = last_ay

        temp = float(data[3])
        #if (abs(temp - mean_z) < 0.2):
        mean_z = mean_z * 0.999 + temp * 0.001
        temp = temp - mean_z if abs(temp-mean_z) > 0.15 else 0
        last_az = last_az*alpha + temp*(1-alpha)
        taz[count] = last_az

        if (cur_idx == 0 and count == 0):
          ex[count] = 0
          ey[count] = 0
          ez[count] = 0
          base_ez = float(data[4])
          base_ey = float(data[5])
          base_ex = float(data[6])
        else:
          ez[count] = -(float(data[4]) - base_ez)/360*2*math.pi
          ey[count] = (float(data[5]) - base_ey)/360*2*math.pi
          ex[count] = (float(data[6]) - base_ex)/360*2*math.pi

        t = np.append(t, int(data[8])*25/1000)
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

      timestep = (t[i] - t[i-1])

      # update velocity
      #vx[i+1] = vx[i] + ax[i]*timestep
      #vy[i+1] = vy[i] + ay[i]*timestep
      #vz[i+1] = vz[i] + az[i]*timestep
      vx = np.append(vx, vx[i-1]+ax[i-1]*timestep)
      vy = np.append(vy, vy[i-1]+ay[i-1]*timestep)
      vz = np.append(vz, vz[i-1]+az[i-1]*timestep)

      # update position
      #x[i+1] = x[i] + vx[i]*timestep
      #y[i+1] = y[i] + vy[i]*timestep
      #z[i+1] = z[i] + vz[i]*timestep
      x = np.append(x, x[i-1]+vx[i-1]*timestep)
      y = np.append(y, y[i-1]+vy[i-1]*timestep)
      z = np.append(z, z[i-1]+vz[i-1]*timestep)

      if (abs(ax[i]) <= 0.2 and abs(ay[i]) <= 0.2 and abs(az[i]) <= 0.2):
        if last_zero == 0:
          last_zero = t[i]
      else:
        last_zero = 0    

      # calibrate vel if more than half a second w no accel
      v_anchor = np.append(v_anchor, [0])
      if (last_zero != 0 and (t[i] - last_zero) > 0.3):
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
        v_anchor[i-1] = 1



    cur_idx = cur_idx + CHUNKS 
    
    if (cur_idx >= 5000):
      print(time.time() - t0)
      break
    
    m = max(max(abs(x)), max(abs(y)))
    figA.set_xlim([-m, m])
    figA.set_ylim([-m, m])
    figA.cla()
    figA.plot(x,y)
    fig.canvas.draw()
        
  plt.show()

"""

    m = max(max(max(abs(x)), max(abs(y))), max(abs(z)))
    figA.set_xlim([-m, m])
    figA.set_ylim([-m, m])
    figA.set_zlim([-m, m])
    figA.plot(x[cur_idx:cur_idx+CHUNKS],y[cur_idx:cur_idx+CHUNKS],z[cur_idx:cur_idx+CHUNKS])
    #plt.show()
    fig.canvas.draw()
"""



"""
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

  text_file = open('log', 'w')
  text_file.write(', '.join([str(x) for x in ax]))
  text_file.write('\n')
  text_file.write(', '.join([str(x) for x in ay]))
  text_file.write('\n')
  text_file.write(', '.join([str(x) for x in az]))
  text_file.write('\n')
  text_file.write(', '.join([str(x) for x in vx]))
  text_file.write('\n')
  text_file.write(', '.join([str(x) for x in vy]))
  text_file.write('\n')
  text_file.write(', '.join([str(x) for x in vz]))
  text_file.write('\n')
  text_file.write(', '.join([str(x) for x in x]))
  text_file.write('\n')
  text_file.write(', '.join([str(x) for x in y]))
  text_file.write('\n')
  text_file.write(', '.join([str(x) for x in t]))
  text_file.write('\n')
  text_file.close()
  data_file.close()
"""

run()

