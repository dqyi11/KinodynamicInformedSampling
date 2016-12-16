#include <Sampler/MonteCarloSamplers.h>

// Standard library functions
#include <math.h>       /* exp, tanh, log */
#include <limits>
#include <algorithm>

///
/// Sigmoid function
/// 
/// @param x Input
/// @param a Controls shape of sigmoid
/// @param c Controls shape of sigmoid
/// @return Output of sigmoid function
/// 
inline double sigmoid(const double& x, const double& a = 20, const double& c = 0)
{
	return 1 / (1 + exp(-a * (x - c)));
}

///
/// Function to get some value between 0 and 1
///
/// @return A uniform value between 0 and 1
///
inline double rand_uni()
{
	// Set up the random number generator
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<double> dis(0.0, 1.0);

	return dis(gen);
}

// 
// MonteCarloSampler
//

///
/// Get the energy of the state from the cost function
///
/// @param curr_state Current state to get the energy for
/// @return Energy of the function
///
double MonteCarloSampler::get_energy(const VectorXd& curr_state) const
{
    double cost = problem().get_cost(curr_state);

    return tanh(cost) + 100 * sigmoid(cost - problem().level_set());
}

///
/// Get the probability of the state from the cost function
///
/// @param energy Energy of the state
/// @return Probability of the state
///
double MonteCarloSampler::get_prob(const double& energy) const
{
	return exp(-energy);
}

///
/// Get the probability of the state from the cost function
///
/// @param curr_state Current state to get the energy for
/// @return Probability of the state
///
double MonteCarloSampler::get_prob(const VectorXd& curr_state) const
{
	return exp(-MonteCarloSampler::get_energy(curr_state));
}

///
/// Get one random uniform sample from the space
///
/// @return Random uniform vector of length size
///
VectorXd MonteCarloSampler::get_random_sample() const
{
	// Set up the random number generator
	std::random_device rd;
	std::mt19937 gen(rd());
	// Get the limits of the space
	VectorXd max_vals, min_vals;
	std::tie(max_vals, min_vals) = problem().state_limits();

	int size = problem().space_dimension();
	VectorXd sample(size);
	for(int i = 0; i < size; i++)
	{
		// Get a random distribution between the values of the joint
		double min = min_vals(i); double max = max_vals(i);
		std::uniform_real_distribution<> dis(min, max);

		sample(i) = dis(gen);
	}

	return sample;
}

///
/// Function to concatenate a vector to back of matrix
///
/// @param matrix Matrix to concatenate to bottom of
/// @param vector Vector to contatenate to bottom of matrix
/// @return A new matrix with the concatenation
///
MatrixXd concatenate_matrix_and_vector(const MatrixXd& matrix, const VectorXd& vector)
{
	// Concatenate to results matrix
	MatrixXd new_results(matrix.rows()+1, matrix.cols());
	new_results << matrix,
				   vector.transpose();
	return new_results;
}

///
/// Surf down the cost function to get to the levelset
///
/// @param alpha Learning rate
/// @return Path to the level set
///
MatrixXd MonteCarloSampler::grad_descent(const double& alpha) const
{
	VectorXd start = MonteCarloSampler::get_random_sample();
	MatrixXd results(1, problem().space_dimension());
	results.row(results.rows()-1) = start;

	double cost = problem().get_cost(start);

	while(cost > problem().level_set())
	{
		VectorXd grad = problem().get_grad(start);
		start = start - alpha * grad;

		// Concatenate to results matrix
		// MatrixXd new_results(results.rows()+1, results.cols());
		// new_results << results,
		// 			   start.transpose();
		// results = new_results;
		results = concatenate_matrix_and_vector(results, start);

		cost = problem().get_cost(start);
	}

	return results;
}

///
/// Get a normal random vector for the momentum
///
/// @return A vector of momentum sampled from a random distribution
///
VectorXd MonteCarloSampler::sample_normal(const double& mean, const double& sigma) const
{
	// Set up the random number generator
	std::random_device rd;
	std::mt19937 gen(rd());
	std::normal_distribution<double> dis(mean, sigma);

	int size = problem().space_dimension();
	VectorXd sample(size);
	for(int i = 0; i < size; i++)
	{
		sample(i) = dis(gen);
	}

	return sample;
}

//
// HMCSampler
//

///
/// Get a series of samples for the problem space
///
/// @param no_samples Number of samples to get
/// @param time Boolean that determines if the time to run the proccess is displayed 
/// @return A series of samples of shape (number of samples, sample dimension)
///
MatrixXd HMCSampler::sample(const int& no_samples, const bool& time) const
{
	MatrixXd ski = HMCSampler::grad_descent();

	std::cout << "Got the gradient descent" << std::endl;

	// Last row of the ski is the start of the HMC algorithm
	VectorXd q = ski.row(ski.rows()-1);

	std::cout << "G0t last row of the gradient" << std::endl;

	// Store the samples
	MatrixXd samples(1, problem().space_dimension() + 1);
	samples << q.transpose(), problem().get_cost(q);

	std::cout << "Added the sample" << std::endl;

	for(int step = 0; step < no_samples; step++)
	{
		// Sample the momentum and set up the past and current state and momentum
		VectorXd q_last = q;
		VectorXd p = MonteCarloSampler::sample_normal(0, sigma());
		VectorXd p_last = p;

		// Make a half step for momentum at the beginning
		VectorXd grad = problem().get_grad(q);
		// Ensure that the gradient isn't two large
		if(grad.maxCoeff() > 1e2) std::cout << "Gradient too high" << std::endl; return samples;
		p -= epsilon() * grad / 2;

		// Alternate Full steps for q and p
		double dist = 0.0;
		while(dist < L())
		{
			q += epsilon() * p;
			if(dist != L()) p = p - epsilon() * grad;
			dist += epsilon();
		}

		// Make a half step for momentum at the end
		p -= epsilon() * grad / 2;

		// Negate the momentum at the end of the traj to make proposal
		// symmetric
		p = -p;

		// Evaluate potential and kinetic energies at start and end of traj
		double U_last = get_energy(q_last);
		double K_last = p_last.norm() / 2;        
		double U_proposed = get_energy(q);
		double K_proposed = p_last.norm() / 2;

		// Accept or reject the state at the end of trajectory
		double alpha = std::min(1.0, std::exp(U_last-U_proposed+K_last-K_proposed));
		if (rand_uni() <= alpha)
		{
			std::cout << "Accepted" << std::endl;
			VectorXd newsample(problem().start_state().size() + 1);
	    	newsample << q, problem().get_cost(q);
			samples = concatenate_matrix_and_vector(samples, newsample);
		}
		else
		{
			q = q_last;
		}
	}

	return samples;
}