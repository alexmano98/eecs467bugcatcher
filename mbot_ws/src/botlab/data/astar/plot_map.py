import numpy as np
import matplotlib.pyplot as plt
import sys

# read the map file
mapName = str(sys.argv[1])

params = np.loadtxt(mapName, dtype=float, skiprows=0, max_rows=1)
map = np.loadtxt(mapName, dtype=int, skiprows=1)

# unpack parameters
origin = np.array([params[0] , params[1]])
size = np.array([params[2], params[3]])
resolution = params[4]

# read the astar path file
pathName = str(sys.argv[2])
path = np.loadtxt(pathName, dtype=float, usecols=(1,2,3), skiprows=1)

# translate the path poses into pixel coordinates
# u*res + originX = X
path[:,0:2] = (path[:,0:2] - origin) / resolution

# plot the map and path:
plt.imshow(map, cmap='gray_r', vmin = -127, vmax = 127)
plt.plot(path[:,0], path[:,1], c='r')
plt.scatter(x=path[:,0], y=path[:,1], c='b', s=3)
# green start
plt.scatter(x=[path[0,0]], y=path[0,1], c='g', s=8)
# Yellow Goal
plt.scatter(x=[path[-1,0]], y=path[-1,1], c='y', s=8)

plt.show()