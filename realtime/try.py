import matplotlib.pyplot as plt
import time
import numpy as np

plt.ion()
fig = plt.figure()
#figA = fig.add_subplot(111, projection = "3d")
figA = fig.add_subplot(111)
figA.set_xlabel('x')
figA.set_ylabel('y')

#figA.set_zlim([-m, m])

x = np.array([0])
y = np.array([0])

for i in range(1,100):
  x = np.append(x, [i])
  y = np.append(y, [i])
  figA.set_xlim([-100, 100])
  figA.set_ylim([-100, 100])
  figA.plot(x[i-1:],y[i-1:])
  fig.canvas.draw()
  time.sleep(0.5)

