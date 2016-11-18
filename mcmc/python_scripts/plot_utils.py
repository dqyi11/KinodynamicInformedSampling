# 
# Plotting utilties in python
# 
import numpy as np
import distance_functions
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

# 
# Function to plot a 3D surf function
# 
# @param x_range Tuple specifying the xrange of the surf
# @param y_range Tuple specifying the yrange of the surf
# @param step Tupe for the step of the x and y meshgrid
# @param z_func Function to run x_range and y_range on
# 
def plot_surf(x_range, y_range, no_ele, z_func):
    x = np.linspace(x_range[0], x_range[1], no_ele[0])
    y = np.linspace(y_range[0], y_range[1], no_ele[1])

    X, Y = np.meshgrid(x, y)
    X = np.reshape(X, [X.shape[0]*X.shape[1], 1]);
    Y = np.reshape(Y, [Y.shape[0]*Y.shape[1], 1]);
    Z = z_func([X, Y])

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_surface(X, Y, Z, rstride=4, cstride=4, color='b')

    plt.show()
