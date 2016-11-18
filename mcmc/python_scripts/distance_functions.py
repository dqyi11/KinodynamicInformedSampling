# 
# Utilities for distance functions
# 
import numpy as np 

# 
# Function to find the L2 distance of the functions
#
# @param x Point to calculate from the origin
# @return L2 Distance between x0 and x1
# 
def l2dist(x):
    zeros = np.zeros(x.shape)
    print("X Shape: {}".format(x.shape))
    print("Zeros Shape: {}".format(zeros.shape))
    return np.linalg.norm(x - zeros, 2)
def L2_distance(x):
    print("X shape before: {}".format(x.shape))
    l2_vec = np.vectorize(l2dist)
    return l2_vec(x)
