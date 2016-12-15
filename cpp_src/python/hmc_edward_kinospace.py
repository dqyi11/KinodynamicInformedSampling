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
	def __init__(self, q1, q2, level_set):
		self.q1 = q1
		self.q2 = q2
		self.level_set = level_set
		self.weight = 100

		self.q = tf.placeholder("float", shape=self.q1.shape)
		self.zs = {}
		self.zs['z'] = self.q

		self.prob = self.log_prob(None, self.zs)

	"""p(x, z) = p(z) = p(z | x) = exp(-E)"""
	def log_prob(self, xs, zs):
		# Get the latent param
		q = zs['z']

		# Get the max time for the model
		self.T = di_tf.get_time(self.q1, self.q2, q)

		# Get the energy function
		# self.E = tf.log(1 + tf.log(self.T + 1)) + self.weight * sigmoid(self.T - self.level_set)
		self.E = tf.tanh(self.T) + self.weight * sigmoid(self.T - self.level_set)

		# return the probability
		return tf.exp(-self.E)
		# return -self.E

# Parameters
no_to_sample = 200
no_dimensions = 2
q1 = np.array([0,0])
q2 = np.array([0,1])
level_set = 5
minval = -5.0
maxval = 5.0
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

# Print the points at the bottom
di.plot_contour_with_points(q1, q2, qz_init, level_set)

# Model for the latent space (here it is just the joint space) - Must be Empirical 
# qz = Empirical(params=tf.Variable(qz_init, dtype=tf.float32))
qz = Empirical(params=tf.Variable(tf.random_uniform(shape=(no_to_sample, no_dimensions),\
													minval=minval,\
													maxval=maxval)))

# Model of the distribution
model = PlanningPosterior(q1, q2, level_set)

di_tf.plot_prob(q1, q2, model)

# Run inference on the model
inference = ed.HMC({'z' : qz}, model_wrapper=model)
t1 = datetime.now()
inference.initialize(step_size=0.01, n_steps=2)
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