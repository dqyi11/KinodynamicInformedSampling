import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
import random

x1=10
v1=2
a_max=1
a1=a_max
a2=-a_max

# returns the minimum time between 2 points ([x1,y1] and [x2,y2]) in the state space
# returns -1 if the solution does not exist
def cal_min_time(x1, x2, v1, v2):
    global a1, a2
    dp_acc = 0.5*(v1+v2)*abs(v2-v1)/a_max
    sigma = 1 if(x2 - x1 - dp_acc)>0 else -1
    a2 = -sigma*a_max
    a1 = -a2
    ta1_a = (-2*v1 + (v1**2 - 4*a1*((v2**2-v1**2)/(2*a2) - (x2-x1))))/(2*a1)
    ta1_b = (-2*v1 - (v1**2 - 4*a1*((v2**2-v1**2)/(2*a2) - (x2-x1))))/(2*a1)
    if ta1_a > 0:
        ta1 = ta1_a
    elif ta1_a == 0 and ta1_b < 0:
        ta1 = ta1_a
    elif ta1_b >= 0:
        ta1 = ta1_b
    else:
        return -100
    T = (v2-v1)/a2 + 2*ta1
    return T

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
x = np.arange(x1-3.0, x1+3.0, 0.02)
y = np.arange(v1-1.0, v1+1.0, 0.02)
X, Y = np.meshgrid(x, y)
zs = np.array([cal_min_time(x1, x, v1, y) for x,y in zip(np.ravel(X), np.ravel(Y))])
Z = zs.reshape(X.shape)

ax.plot_surface(X, Y, Z, cmap=cm.coolwarm)
ax.set_xlabel('X')
ax.set_ylabel('X_dot')
ax.set_zlabel('Minimum Time')
plt.show()
