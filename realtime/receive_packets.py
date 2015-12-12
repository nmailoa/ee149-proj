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
        line = ser.read(12)
        print(line)
        print(hex(line >> 11*8))
        print('\t')
        print(hex(line >> 77 & 0x3FF))
        print('\t')
        print(hex(line >> 66 & 0x3FF))
        print('\t')
        print(hex(line >> 55 & 0x3FF))
        print('\t')
        print(hex(line >> 40 & 0x7FF))
        print('\t')
        print(hex(line >> 25 & 0x7FF))
        print('\t')
        print(hex(line >> 10 & 0x7FF))
        print('\t')
        print(hex(line >> 8 & 0x3))
        print('\t')
        print(hex(line & 0xFF))
        print('\n')
        
        count = count + 1

      except:
        failcount = failcount + 1
        pass

    cur_idx = cur_idx + CHUNKS 
    
    if (cur_idx >= 700):
      break
    

run()


