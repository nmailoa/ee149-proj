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


  #dump_file = open('dump', 'w')
  
  while(True):
    count = 0
    while count < CHUNKS:
      if (failcount > 20):
        break
      try:
        line = (ser.read(12))
        print(line[0] >> 7)
        print("Calib")
        c = line[1]
        print(str((c & 0xc0) >> 6) + str((c & 0x30) >> 4) + str((c & 0x0c) >> 2) + str(c & 0x03))

        print("Button")
        print(line[0] >> 4 & 0x7)

        print("Accel")
        ax = (line[0] & 0xf) << 8 | line[1]
        if (ax & 0x800): ax = -1*int((ax ^ 0xfff) + 1)
        ax = ax/100
        print(ax)
        ay = line[2] << 4 | (line[3] >> 8)
        if (ay & 0x800): ay = -1*int((ay ^ 0xfff) + 1)
        ay = ay/100
        print(ay)
        az = (line[3] & 0xf) << 8 | line[4]
        if (az & 0x800): az = -1*int((az ^ 0xfff) + 1)
        az = az/100
        print(az)

        print("Euler")
        ex = line[5] << 8 | line[6]
        ex = int(ex)/100
        print(ex)
        ey = line[7] << 8 | line[8]
        if (ey & 0x8000): ey = -1*int((ey ^ 0xffff) + 1)
        ey = int(ey)/100
        print(ey)
        ez = line[9] << 8 | line[10]
        if (ez & 0x8000): ez = -1*int((ez ^ 0xffff) + 1)
        ez = int(ez)/100
        print(ez)

        print("Timer")
        print(line[11])
        print("End\n")
        
        count = count + 1

      except:
        failcount = failcount + 1
        pass

    cur_idx = cur_idx + CHUNKS 
    
    if (cur_idx >= 10000):
      break
    

run()


