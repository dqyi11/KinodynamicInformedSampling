# Outside packages
import pymc3 as pm
import theano.tensor as T
import numpy as np

# Inside packages
import double_integrator as di
import double_integrator_theano as di_T

pi = 3.1415926

# 
# Sigmoid function
# 
# @param x Input
# @param a Controls shape of sigmoid
# @param c Controls shape of sigmoid
# @return Output of sigmoid function
# 
def sigmoid(x, a=20, c=0):
    return 1 / (1 + T.exp(-a * (x - c)))

# 
# Create a function that implements our probability distribution
# 
# @param q1 Start state
# @param q2 Goal state
# @param q Intermediate state to find the time for
# @param level_set Level set that you want to sample from
# 
def log_prob(q1, q2, q, level_set):
    # Get the max time for the model
    time = di_T.get_time(q1, q2, q)

    # Get the energy function
    E = T.tanh(time) + 100 * sigmoid(time - level_set)

    # return the probability
    return T.exp(-E)
 
# Set up constants and parameters
no_to_sample = 1
no_dimensions = 2
q1 = np.array([0,0])
q2 = np.array([0,1])
minval = -10.
maxval = 10.
level_set = 3.
# Grad descent params
alpha = 0.05 # Gradient descent learning rate
epochs = 1000

# Surf down to get to the level set
start = np.random.uniform(low=minval, high=maxval, size=2)
results = di.grad_descent(di.get_time, epochs, q1, q2, start, alpha, level_set)
start = results[-1,:-1]
print(start)

model = pm.Model()
with model:
    # Set up the model for the input variables
    q =  pm.Uniform("q", minval, maxval, shape=q1.shape)

    # Get the potential function
    prob = pm.Potential('prob',log_prob(q1, q2, q, level_set))
    step = pm.NUTS(model.vars, scaling=start)

n=100
with model:
    trace = pymc3.sample(n, step, start)
    print(trace)
