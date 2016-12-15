# 
# Functions for getting the minimum time of the double integrator function
# 

import numpy as np
import tensorflow as tf 

# Maximum accelleration
a_max = 1

# 
# Our own sign function
# 
# @param x Value to find the sign of 
# @return Sign of the function
# 
def sign1(x):
	if_positive = lambda: tf.constant(1, dtype=tf.float32)
	if_negative = lambda: tf.constant(-1, dtype=tf.float32)

	return tf.cond(tf.greater_equal(x,0), if_positive, if_negative)

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
	return tf.case(
			{
				tf.greater_equal(ta1_a,0) : lambda: (v2 - v1) / a2 + 2 * ta1_a,
				tf.greater_equal(ta1_b,0) : lambda: (v2 - v1) / a2 + 2 * ta1_b,
				tf.equal(ta1_a, 0) : lambda: tf.constant(0, dtype=tf.float32),
				tf.equal(ta1_a, 0) : lambda: tf.constant(0, dtype=tf.float32)
			}, default=lambda: tf.constant(-1, dtype=tf.float32))

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
	return tf.case(
			{
				tf.less(T1, 0) : lambda: tf.constant(-1, dtype=tf.float32),
				tf.less(T2, 0) : lambda: tf.constant(-1, dtype=tf.float32),
			}, default=lambda: T1 + T2)

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
# Plots the probability surface
# 
def plot_prob(x1, x2, model):
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
	X = np.arange(-2, 2, 0.1)
	Y = np.arange(-2, 2, 0.1)
	X, Y = np.meshgrid(X, Y)
	X = np.reshape(X, (X.shape[0] * X.shape[1], 1))
	Y = np.reshape(Y, (Y.shape[0] * Y.shape[1], 1))
	q1 = np.tile(x1, [X.shape[0] * X.shape[1], 1])
	q2 = np.tile(x2, [X.shape[0] * X.shape[1], 1])
	T = np.zeros((X.shape[0] * X.shape[1], 1))
	in_space = {}

	with tf.Session() as sess:
		# Initialize all variables
		init = tf.initialize_all_variables()
		sess.run(init)

		for row in range(T.shape[0]):
			next_q = np.array([X[row,:], Y[row,:]])
			next_q = np.reshape(next_q, (2,))
			prob = sess.run(model.prob, feed_dict={model.q: next_q})
			T[row,:] = prob

	X = np.reshape(X, (40,40))
	Y = np.reshape(Y, (40,40))
	T = np.reshape(T, (40,40))
	surf = ax.plot_surface(X, Y, T, rstride=1, cstride=1, cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)

	ax.zaxis.set_major_locator(LinearLocator(10))
	ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))

	fig.colorbar(surf, shrink=0.5, aspect=5)

	plt.show()

# Main function plots the surface
def main():
	print("Nothing in main function.")

if __name__ == '__main__':
	main()