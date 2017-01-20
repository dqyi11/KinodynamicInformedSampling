import matplotlib.pyplot as plt
import numpy as np

with open('../test_mcmc.log') as f:
	mcmc = np.loadtxt(f)
with open('../test_rej.log') as f:
	rej = np.loadtxt(f)

print(mcmc.shape)
print(rej.shape)

for joint in range(0, mcmc.shape[1] - 1, 2):
	fig = plt.figure(joint / 2)
	plt.scatter(mcmc[:,joint], mcmc[:,joint+1], c='b', marker="o", label='MCMC')
	plt.scatter(rej[:,joint], rej[:,joint+1], c='r', marker="o", label='Rejection')
	plt.legend(loc='upper left')
	plt.title('Joint {}'.format(joint / 2))
	plt.xlabel('x')
	plt.ylabel('x_dot')

plt.show()