# 
# Test functions to try and implement our probability function in Edward
# 

# Import the uniform edward model
import edward as ed
from edward.models import Uniform, Empirical

# Import tensorflow
import tensorflow as tf

# Numpy
import numpy as np

# Import the double integrator model
import double_integrator_tf as di_tf
import double_integrator as di

# 
# Sigmoid function
# 
# @param x Input
# @param a Controls shape of sigmoid
# @param c Controls shape of sigmoid
# @return Output of sigmoid function
# 
def sigmoid(x, a=20, c=0):
	return 1 / (1 + tf.exp(-a * (x - c)))

# 
# Create a class that implements our probability distribution
# 
class PlanningPosterior:
	"""p(x, z) = p(z) = p(z | x) = exp(-E)"""
	def log_prob(self, xs, zs):
		# Get the latent param
		q = zs['z']

		# Get the max time for the model
		T = di_tf.get_time(q1, q2, q)

		# Get the energy function
		E = tf.tanh(T) + 1e4 * sigmoid(T - level_set)

		# return the probability
		return tf.exp(-E)

# Parameters
no_to_sample = 100
no_dimensions = 2
q1 = np.array([0,0])
q2 = np.array([1,1])
level_set = 2
minval = -5
maxval = 5
alpha = 0.5 # Gradient descent learning rate

# Surf down to the level_set
start = np.random.uniform(low=minval, high=maxval, size=2)
print(start)
results = di.grad_descent(di.get_time, 100, q1, q2, start, alpha, level_set)
print("Results: {}".format(results))

# Print the surf
di.plot_surface_with_path(q1, q2, results, level_set)

# # Model for the latent space (here it is just the joint space) - Must be Empirical 
# # qz = Empirical(params=tf.Variable(tf.random_uniform(shape=[no_to_sample, no_dimensions],\
# # 													minval=minval,
# # 													maxval=maxval)))
# qz = Empirical(params=tf.Variable(tf.random_normal(shape=[no_to_sample, no_dimensions])))


# # Model of the distribution
# model = PlanningPosterior()

# # Run inference on the model
# inference = ed.HMC({'z': qz}, model_wrapper=model)
# inference.initialize()
# inference.run()

# # Print 100 values and their time
# for i in range(100):
# 	sample = qz.value().eval()
# 	T = di.get_time(q1, q2, sample)

# 	print("Sample: {} | Time: {}".format(qz.value().eval(), T))