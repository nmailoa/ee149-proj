#!/usr/bin/python
from __future__ import unicode_literals
import sys
import argparse
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
import time
import glob
import serial
import struct
from mpl_toolkits.mplot3d import Axes3D
import csv

from matplotlib.backends import qt_compat
use_pyside = qt_compat.QT_API == qt_compat.QT_API_PYSIDE
if use_pyside:
    from PySide import QtGui, QtCore
else:
    from PyQt4 import QtGui, QtCore

global EE16A_SCAN_DELAY_USER, EE16A_SCAN_DELAY
EE16A_SCAN_DELAY = 10000 #Total time to delay for at start of scan in miliseconds 
EE16A_SCAN_DELAY_USER = 0

DEFAULT_WIDTH = 1240
DEFAULT_HEIGHT = 760
global serial_count


help_menu = """Quit: press [Esc], [Ctrl+Q], or [Ctrl+W] at any time to exit\n
Help: press [H] to show this help menu\n
Fullscreen: press [F] to toggle full screen mode\n
Start scanning: press [Enter] start or continue scanning\n
Pause scanning: press [Space] to pause  scanning"""

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

def serial_write(ser, data):
  global serial_count
  if sys.platform.startswith('win'):
    ser.write([data,])
  else:
    ser.write(str(data))

def read_csv(filename):
    """
    Reads a csv file and returns the first 20 recordings from the file
    Input:
        filename: csv filename
    Output:
        data: a 20x66 matrix corresponding to the first 20 readings in the csv file. Each row corresponds
            to a reading; the first 33 values are x-coordinates while the second33 values are y-coordinates
    """
    data = []
    with open(filename, 'r') as f:
        reader = csv.reader(f)
        for i,row in enumerate(reader):
            if(row):
                if (data == []):
                    data = np.array([float(i) for i in row]).T
                else:
                    data = np.vstack((data, (np.array([float(i) for i in row]).T)))
    return data

def update_line(hl, new_data):
    hl.set_xdata(numpy.append(hl.get_xdata(), new_data))
    hl.set_ydata(numpy.append(hl.get_ydata(), new_data))
    ax.relim()
    ax.autoscale_view()
    plt.draw()
    plt.show()


def main():
  global serial_count
  print("EE16A Imaging Lab")

  ##LOOKING FOR PORTS
  print("Checking serial connections...")
  ports = serial_ports()
  if ports:
    print("Available serial ports:")
    for (i,p) in enumerate(ports):
      print("%d) %s"%(i+1,p))
  else:
    print("No ports available. Check serial connection and try again.")
    print("Exiting...")
    quit()
  selection = input("Select the port to use: ")
  ser = serial.Serial(ports[int(selection)-1],9600,timeout = .01)
  #ser.write([9,])
  # Read from serial port to check if it's getting cleared

  ##Creating Plots
  hl, = plt.plot([], [])
  #creating position and vector array
  pos = v = np.zeros((3,1))
  x = y = z = 0 # current position
  vx= vy= vz = 0 # current velocity
  a = np.zeros((3,1))
  xo= yo= zo = 0 # initial position
  t = 0.01 # sampling period
  fig = plt.figure()
  figA = fig.add_subplot(111, projection = '3d')
  figA.scatter(x,y,z)
  figA.view_init(elev=20, azim=47)
  plt.show()


  data = []
  while(1):
    serial_data = ser.readline().decode().rstrip('\n')
    if (serial_data != ''):
      serial_data = serial_data.split(',')
      if (len(serial_data) > 2):
        if (data == [] and len(serial_data) > 2):
          print (serial_data)
          data = np.array([float(i) for i in serial_data]).T
        else:
          print (serial_data)
          data = np.vstack((data, (np.array([float(i) for i in serial_data]).T)))

  ax = data[:,0] #np.random.rand(10) # x accel for time step 0, 1, 2, ...
  ay = data[:,1] #np.random.randn(10) # y accel
  az = data[:,2] #np.random.randn(10) # z accel

  for i in range(len(ax)):
      # update velocity
      vx = vx + ax[i]*t
      vy = vy + ay[i]*t
      vz = vz + az[i]*t
      # update position
      xOld = x
      yOld = y
      zOld = z
      x = x + vx*t
      y = y + vy*t
      z = z + vz*t
      
      figA.scatter(x,y,z)
      figA.plot([xOld,x],[yOld,y],[zOld,z])
      figA.view_init(elev=20, azim=47)
      



  app = QtGui.QApplication(sys.argv)
  print("Press [Esc], [Ctrl+Q], or [Ctrl+W] at any time to exit.")
  print("Press [F] for fullscreen and press [H] to show a help menu.\n")

if __name__ == '__main__':
  main()
