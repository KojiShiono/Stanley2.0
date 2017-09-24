import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

data = np.loadtxt("../data/sim_waypoints.csv", delimiter=',', skiprows=1)

def distance(p, q):
    return np.sqrt((p[0]-q[0])**2+(p[1]-q[1])**2+(p[2]-q[2])**2)

dist = np.zeros((data.shape[0]-1))
for i in range(data.shape[0]-1):
    p = (data[i,0], data[i,1], data[i,2])
    q = (data[i+1,0], data[i+1,1], data[i+1,2])
    dist[i] = distance(p, q)

sns.set_style("white")
fig = plt.figure(1)
fig.clf()
ax = fig.add_subplot(111)

ax.plot(np.linspace(0,len(dist), len(dist)), dist, 'o', color="crimson")

ax.set_xlabel("waypoints idx")
ax.set_ylabel("distance (m)")

ax.set_xscale("log")

plt.draw()
plt.show()
