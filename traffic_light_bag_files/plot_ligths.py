import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

data = np.loadtxt("lights.dat")

sns.set_style("white")
fig = plt.figure(1)
fig.clf()
ax = fig.add_subplot(111)

ax.scatter(data[:,0], data[:,2], marker='o', color="crimson")
ax.scatter(data[:,0], data[:,1], marker='o', color="seagreen")

ax.set_xlabel("waypoint index")
ax.set_ylabel("pixels count")

plt.draw()
plt.show()
