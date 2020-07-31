import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
x=[]
y=[]
z=[]
with open("pos_leg5.txt", "r")as f:
    lines = f.readlines()
    for line in lines:
        pos0 = line.split(',')
        # print(float(pos0[0]))
        x.append(float(pos0[0]))
        y.append(float(pos0[1]))
        z.append(float(pos0[2]))
fig = plt.figure()
ax = Axes3D(fig)
ax.scatter(x, y, z)
ax.set_zlabel('Z', fontdict={'size': 15, 'color': 'red'})
ax.set_ylabel('Y', fontdict={'size': 15, 'color': 'red'})
ax.set_xlabel('X', fontdict={'size': 15, 'color': 'red'})
plt.show()