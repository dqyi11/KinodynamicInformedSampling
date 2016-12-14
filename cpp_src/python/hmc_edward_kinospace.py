# 
# Test functions to try and implement our probability function in Edward
# 

from datetime import datetime
FMT = '%H:%M:%S'

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

# Stop warnings. Probably shouldn't do this
import warnings
warnings.filterwarnings("ignore")
tf.logging.set_verbosity(tf.logging.ERROR)

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
		E = tf.tanh(T) + 1e30 * sigmoid(T - level_set)

		# return the probability
		return tf.exp(-E)

# Parameters
no_to_sample = 1
no_dimensions = 2
q1 = np.array([0,0])
q2 = np.array([0,1])
level_set = 5
minval = -10.0
maxval = 10.0
# Grad descent params
alpha = 0.05 # Gradient descent learning rate
epochs = 1000
# Sample z from the level set
qz_init = np.array([])
for i in range(no_to_sample):
	start = np.random.uniform(low=minval, high=maxval, size=2)
	results = di.grad_descent(di.get_time, epochs, q1, q2, start, alpha, level_set)

	if qz_init.size == 0:
		qz_init = np.array([results[-1,:-1]])
	else:
		qz_init = np.concatenate((qz_init, np.array([results[-1,:-1]])))

	# print("End: {}".format(results[-1,:]))
	# di.plot_surface_with_path(q1, q2, results, level_set)

di.plot_contour_with_points(q1, q2, qz_init, level_set)

# Model for the latent space (here it is just the joint space) - Must be Empirical 
# qz = Empirical(params=tf.Variable(tf.random_normal(shape=[no_to_sample, no_dimensions])))
qz = Empirical(params=tf.Variable(qz_init, dtype=tf.float32))

# # Model for the parameter space (Uniform)
# # x = Empirical(params=tf.Variable(tf.random_uniform(shape=[1, no_dimensions],\
# # 												   minval=minval,
# # 												   maxval=maxval)))

# Model of the distribution
model = PlanningPosterior()

# Run inference on the model
inference = ed.HMC({'z' : qz}, model_wrapper=model)
t1 = datetime.now()
inference.initialize(step_size=0.01, n_steps=1)
t2 = datetime.now()
inference.run()
t3 = datetime.now()

tdelta = t2 - t1 
tdelta2 = t3 - t2

print("Time For Initialization: {} | Time to Run Inference: {}".format(tdelta, tdelta2))

# Print 100 values and their time
results = np.array([])

for i in range(no_to_sample):
	sample = qz.value().eval()
	T = di.get_time(q1, q2, sample)

	sample = np.reshape(sample, (1, no_dimensions))
	if results.size == 0:
		results = np.array(sample)
	else:
		results = np.concatenate((results, sample))

	print("Sample: {} | Time: {}".format(qz.value().eval(), T))

# Print the surf
di.plot_contour_with_points(q1, q2, results, level_set)
