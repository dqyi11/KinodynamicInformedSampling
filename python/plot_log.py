import matplotlib.pyplot as plt
import numpy as np
import argparse

def main():
    parser = argparse.ArgumentParser(description='Plot the results from the sampling methods.')
    parser.add_argument('--f_rej', help='Filename of HRS or Rejection Sampling.')
    parser.add_argument('--f_mc', help='Filename of MCMC or HMC samples.')

    args = parser.parse_args()
    with open(args.f_mc) as f:
        mcmc = np.loadtxt(f)
    with open(args.f_rej) as f:
        rej = np.loadtxt(f)

    print(mcmc.shape)
    print(rej.shape)

    for joint in range(0, mcmc.shape[1] - 1, 2):
        joint_num = int(joint / 2)
        fig = plt.figure(joint_num)
        plt.scatter(mcmc[:,joint], mcmc[:,joint+1], c='b', marker="o", label='MCMC')
        plt.scatter(rej[:,joint], rej[:,joint+1], c='r', marker="o", label='Rejection')
        plt.legend(loc='upper left')
        plt.title('Joint {}'.format(joint_num))
        plt.xlabel('x')
        plt.ylabel('x_dot')

    plt.show()

if __name__ == '__main__':
    main()
