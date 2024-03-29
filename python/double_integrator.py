# 
# Functions for getting the minimum time of the double integrator function
# 

import numpy as np 

# Maximum accelleration
a_max = 1

# 
# Our own sign function
# 
# @param x Value to find the sign of 
# @return Sign of the function
# 
def sign1(x):
    if(x >= 0):
    	return 1 
    else:
    	return -1

# 
# Returns the minimum time between 2 points ([x1,y1] and [x2,y2]) in the state space
# and returns -1 if the solution does not exist
# 
# @param x1 Initial position of the state
# @param v1 Initial velocity of the state
# @param x2 Final position of the state
# @param v2 Final velocity of the state
# @return Time to get from (x1,v1) to (x2,v2)
# 
def cal_min_time2(x1, v1, x2, v2):
	# Calculate the time to accelerate to highest point
	a_max = 1
	dp_acc = 0.5 * (v1 + v2) * abs(v2 - v1) / a_max
	sigma = sign1(x2 - x1 - dp_acc)
	a2 = -sigma * a_max
	a1 = -a2
	a = a1
	b = 2 * v1
	c = (v2**2 - v1**2) / (2 * a2) - (x2 - x1)
	q = -0.5 * (b + sign1(b) * (b**2 - 4 * a * c)**0.5)
	ta1_a = q / a
	ta1_b = c / q

	# Ensure that the time is feasible
	if(ta1_a > 0):
		ta1 = ta1_a
	elif(ta1_b > 0):
		ta1 = ta1_b
	elif(ta1_a == 0 or ta1_b == 0): 
		return 0
	else:
		return - 1

	# Return total time including time to decellerate 
	return (v2 - v1) / a2 + 2 * ta1

# 
# Returns the minimum time between 3 points ([x1,y1] -> [xi,yi] -> [x2,y2]) in the state space
# and returns -1 if the solution does not exist
# 
# @param x1 Initial position of the state
# @param v1 Initial velocity of the state
# @param x2 Final position of the state
# @param v2 Final velocity of the state
# @param xi Intermediate position
# @param vi Intermediate velocity
# @return Time to get from (x1,v1) to (x2,v2) through (xi, vi)
# 
def cal_min_time3(x1, v1, x2, v2, xi, vi):
	# Get the time from (x1,v1) to (xi,vi)
	T1 = cal_min_time2(x1, v1, xi, vi)
	# Get the time from (xi,vi) to (x2,v2)
	T2 = cal_min_time2(xi, vi, x2, v2)

	# Ensure that the times are valid
	# return -1 if not
	if (T1 < 0 or T2 < 0):
		return -1

	# Return total time
	return T1 + T2

# 
# This function calculates the maximum time given a set number of joints
# x = [x_1, x_1_dot,...,x_n,x_n_dot]
# @param x1 Initial state
# @param x2 Final state
# @param xi Intermediate state
# @return Maximum time for all joints
# 
def get_time(x1, x2, xi):
	Ts = []

	# Get the time for all of the joints
	for i in range(0, x1.shape[0], 2):
		Ts.append(cal_min_time3(x1[i], x1[i+1], x2[i], x2[i+1], xi[i], xi[i+1]))

	# Return maximum time
	return max(Ts)

# 
# Estimate the gradient
# 
# @param fun Function to estimate the gradient for
# @param x Input
# @param x1 Initial State
# @param x2 Final State
# @param h Derivative constant
# @return Gradient of the function at point x
# 
def gradient(fun, x, x1, x2, h):
	grad = np.zeros(x.shape)
	for dim in range(x.shape[0]):
		x_plus = np.copy(x)
		x_plus[dim] = x_plus[dim] + h
		x_min = np.copy(x)
		x_min[dim] = x_min[dim] - h
		grad[dim] = (fun(x1, x2, x_plus) - fun(x1, x2, x_min)) / (2*h)

	return grad

# 
# Function to ski down to the level set using gradient descent
#
# @param fun Function to perform gradient descent on
# @param epochs Max number of epochs
# @param x1 Initial state
# @param x2 Final state
# @param x Start of the ski
# @param alpha Learning rate for gradient descent
# @param level_set Level set to ski to
# @return Series of points that it ski'd down (no_results, dim of space + 1 (for time))
#
def grad_descent(fun, epochs, x1, x2, x, alpha, level_set):
	results = np.array([])
	count = 0
	h = 0.001
	for epoch in range(epochs):
		z = fun(x1, x2, x)

		if(results.size == 0): results = np.array([[x[0], x[1], z]])
		else: results = np.concatenate((results, np.array([[x[0], x[1], z]])), axis=0)

		if(z <= level_set):
			for i in range(3):
				# Take one more variable step toward the gradient and then be done
				# Sample alpha from Gaussian
				grad = gradient(fun, x, x1, x2, h)
				x = x - alpha*grad
				if(results.size == 0): results = np.array([[x[0], x[1], z]])
				else: results = np.concatenate((results, np.array([[x[0], x[1], z]])), axis=0)
			break

		grad = gradient(fun, x, x1, x2, h)
		x = x - alpha*grad

	return results

# 
# Function to plot the surface
# 
# @param x1 Start state
# @param x2 End state
# 
def plot_surface(x1, x2):
	# Assert that the dimension of the state is 2 for plotting surface
	assert(x1.size == 2  and x2.size == 2)

	# Import plotting stuff
	from mpl_toolkits.mplot3d import Axes3D
	from matplotlib import cm
	from matplotlib.ticker import LinearLocator, FormatStrFormatter
	import matplotlib.pyplot as plt

	# Get a meshgrid of the x and y values
	fig = plt.figure()
	ax = fig.gca(projection='3d')
	X = np.arange(-5, 5, 0.25)
	Y = np.arange(-5, 5, 0.25)
	X, Y = np.meshgrid(X, Y)
	X = np.reshape(X, (X.shape[0] * X.shape[1], 1))
	Y = np.reshape(Y, (Y.shape[0] * Y.shape[1], 1))
	x1 = np.tile(x1, [X.shape[0] * X.shape[1], 1])
	x2 = np.tile(x2, [X.shape[0] * X.shape[1], 1])
	T = np.zeros((X.shape[0] * X.shape[1], 1))
	for row in range(T.shape[0]):
		T[row,:] = get_time(x1[row,:], x2[row,:], np.array([X[row,:], Y[row,:]]))

	X = np.reshape(X, (40,40))
	Y = np.reshape(Y, (40,40))
	T = np.reshape(T, (40,40))
	surf = ax.plot_surface(X, Y, T, rstride=1, cstride=1, cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)

	ax.zaxis.set_major_locator(LinearLocator(10))
	ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))

	fig.colorbar(surf, shrink=0.5, aspect=5)

	plt.show()

# 
# Function to plot the surface
# 
# @param x1 Start state
# @param x2 End state
# @param results Samples along the path
# @param level_set Levelset of the cost function
# 
def plot_surface_with_path(x1, x2, results, level_set):
	# Assert that the dimension of the state is 2 for plotting surface
	assert(x1.size == 2  and x2.size == 2)

	# Import plotting stuff
	from mpl_toolkits.mplot3d import Axes3D
	from matplotlib import cm
	from matplotlib.ticker import LinearLocator, FormatStrFormatter
	import matplotlib.pyplot as plt

	# Get a meshgrid of the x and y values
	fig = plt.figure()
	ax = fig.gca(projection='3d')
	maxval = 5
	minval = -5
	step = 0.2
	X = np.arange(minval, maxval, step)
	Y = np.arange(minval, maxval, step)
	X, Y = np.meshgrid(X, Y)
	X = np.reshape(X, (X.shape[0] * X.shape[1], 1))
	Y = np.reshape(Y, (Y.shape[0] * Y.shape[1], 1))
	x1 = np.tile(x1, [X.shape[0] * X.shape[1], 1])
	x2 = np.tile(x2, [X.shape[0] * X.shape[1], 1])
	T = np.zeros((X.shape[0] * X.shape[1], 1))
	for row in range(T.shape[0]):
		T[row,:] = get_time(x1[row,:], x2[row,:], np.array([X[row,:], Y[row,:]]))

	size = ((maxval-minval)/step, (maxval-minval)/step)
	X = np.reshape(X, size)
	Y = np.reshape(Y, size)
	T = np.reshape(T, size)
	surf = ax.plot_surface(X, Y, T, rstride=1, cstride=1, cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)

	ax.zaxis.set_major_locator(LinearLocator(10))
	ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))

	fig.colorbar(surf, shrink=0.5, aspect=5)

	ax.plot(results[:,0], results[:,1], results[:,2], 
        label = 'Path',             # label of the curve
        color = 'DarkMagenta',      # colour of the curve
        linewidth = 3.2,            # thickness of the line
        linestyle = '--'            # available styles - -- -. :
        )

	plt.show()

# 
# Plot the contour and the levelset
# 
# @param x1 Start state
# @param x2 End state
# @param results Samples along the path
# @param level_set Levelset of the cost function
# 
def plot_contour_with_points(x1, x2, results, level_set):
	# Assert that the dimension of the state is 2 for plotting surface
	assert(x1.size == 2  and x2.size == 2)

	# Import plotting stuff
	from mpl_toolkits.mplot3d import Axes3D
	from matplotlib import cm
	from matplotlib.ticker import LinearLocator, FormatStrFormatter
	import matplotlib.pyplot as plt

	# Get a meshgrid of the x and y values
	fig = plt.figure()
	ax = fig.gca()
	maxval = 5
	minval = -5
	step = 0.2
	X = np.arange(minval, maxval, step)
	Y = np.arange(minval, maxval, step)
	X, Y = np.meshgrid(X, Y)
	X = np.reshape(X, (X.shape[0] * X.shape[1], 1))
	Y = np.reshape(Y, (Y.shape[0] * Y.shape[1], 1))
	x1 = np.tile(x1, [X.shape[0] * X.shape[1], 1])
	x2 = np.tile(x2, [X.shape[0] * X.shape[1], 1])
	T = np.zeros((X.shape[0] * X.shape[1], 1))
	for row in range(T.shape[0]):
		T[row,:] = get_time(x1[row,:], x2[row,:], np.array([X[row,:], Y[row,:]]))

	size = ((maxval-minval)/step, (maxval-minval)/step)
	vals = np.concatenate((X,Y,T), axis=1)
	X = np.reshape(X, size)
	Y = np.reshape(Y, size)
	T = np.reshape(T, size)
	
	points = vals[vals[:, 2] < level_set]
	# ax.contour(X, Y, T, zdir='z', offset=level_set, cmap=cm.coolwarm)
	from scipy.spatial import ConvexHull
	hull = ConvexHull(points)
	for simplex in hull.simplices:
		plt.plot(points[simplex, 0], points[simplex, 1], 'k-')
	ax.scatter(results[:,0], results[:,1], c='r', marker='o')

	plt.show()

# Main function plots the surface
def main():
	plot_surface(np.array([0,0]), np.array([1,1]))

if __name__ == '__main__':
	main()