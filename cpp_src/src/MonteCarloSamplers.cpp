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
inline double sigmoid(const double& x, const double& a = 200, const double& c = 0)
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
    // double E_grad = tanh(cost);
    double E_grad = log(1+log(1+cost));
    double E_informed = 100 * sigmoid(cost - problem().level_set());
    double E_region = 0;
    std::tuple<VectorXd, VectorXd> limits = problem().state_limits();
    for(int i=0; i < curr_state.size(); i++)
    {
    	E_region += 100 * sigmoid(std::get<0>(limits)(i) - curr_state(i)); // Lower Limits
    	E_region += 100 * sigmoid(curr_state(i) - std::get<1>(limits)(i)); // Higher Limits
  	}
    return  E_region + E_grad + E_informed;
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
	bool verbose = false;
	std::cout << "Number of samples: " << no_samples << std::endl;
	if(verbose) std::cout << "Surfing" << std::endl;
	MatrixXd ski = HMCSampler::grad_descent();
	if(verbose) std::cout << "Got Through Gradient Descent" << std::endl;

	// Last row of the ski is the start of the HMC algorithm
	VectorXd q = ski.row(ski.rows()-1);

	// Store the samples
	MatrixXd samples(1, problem().space_dimension() + 1);
	samples << q.transpose(), problem().get_cost(q);

	int accepted = 0;
	int rejected = 0;
	// If you want to time the sampling
	high_resolution_clock::time_point t1;
	if(time) t1 = high_resolution_clock::now();
	while(accepted < no_samples)
	{
		if(verbose) std::cout << "New start!" << std::endl;
		MatrixXd ski = HMCSampler::grad_descent();
		if(verbose) std::cout << "Got Through Gradient Descent in loop" << std::endl;

		// Last row of the ski is the start of the HMC algorithm
		VectorXd q = ski.row(ski.rows()-1);

		int curr_rejections = 0;
		int curr_step = 0;
		while(curr_rejections < 10 and accepted < no_samples and curr_step < steps())
		{
			// Sample the momentum and set up the past and current state and momentum
			VectorXd q_last = q;
			VectorXd p = MonteCarloSampler::sample_normal(0, sigma());
			VectorXd p_last = p;

			if(verbose) std::cout << "Sampled the momentum" << std::endl;

			// Make a half step for momentum at the beginning
			VectorXd grad = problem().get_grad(q);
			if(verbose) std::cout << "Got the gradient" << std::endl;

			// Ensure that the gradient isn't two large
			if(grad.maxCoeff() > 1e2) 
			{
				if(verbose) std::cout << "WARNING: Gradient too high" << std::endl;
				break;
			}

			p = p - epsilon() * grad / 2;

			// Alternate Full steps for q and p
			for(int i = 0; i < L(); i++)
			{
				q = q + epsilon() * p;
				if(i != L()) p = p - epsilon() * grad;
			}

			if(verbose) std::cout << "Integrated Along momentum" << std::endl;


			// Make a half step for momentum at the end
			p = p - epsilon() * grad / 2;

			// Negate the momentum at the end of the traj to make proposal
			// symmetric
			p = -p;

			// Evaluate potential and kinetic energies at start and end of traj
			double U_last = get_energy(q_last);
			double K_last = p_last.norm() / 2;        
			double U_proposed = get_energy(q);
			double K_proposed = p_last.norm() / 2;

			if(verbose) std::cout << "Got energies" << std::endl;


			// Accept or reject the state at the end of trajectory
			double alpha = std::min(1.0, std::exp(U_last-U_proposed+K_last-K_proposed));
			if (rand_uni() <= alpha)
			{
				VectorXd newsample(problem().start_state().size() + 1);
		    	newsample << q, problem().get_cost(q);
				samples = concatenate_matrix_and_vector(samples, newsample);
				accepted++;
			}
			else
			{
				q = q_last;
				rejected++;
				curr_rejections++;
				// std::cout << "Current Rejections: " << curr_rejections << std::endl;
			}

			curr_step++;
			if(verbose) std::cout << "Decided on rejection / acceptance" << std::endl;
			if(verbose) std::cout << "Number Accepted: " << accepted << std::endl;
			std::cout << "Number Accepted: " << accepted << std::endl;

		}
	}

	// If you want to time the sampling and display it
	if(time)
	{
		high_resolution_clock::time_point t2 = high_resolution_clock::now();
		auto duration_s = duration_cast<seconds>( t2 - t1 ).count();
		auto duration_ms = duration_cast<milliseconds>( t2 - t1 ).count();
		auto duration_us = duration_cast<microseconds>( t2 - t1 ).count();
		if (duration_s != 0)
			std::cout << "Total Sampling Time: " << duration_s << "s" << std::endl;
		else if (duration_ms != 0)
			std::cout << "Total Sampling Time: " << duration_ms << "ms" << std::endl;
		else 
			std::cout << "Total Sampling Time: " << duration_us << "us" << std::endl;
	}
	std::cout << "Percentage Accepted: " << (accepted + 0.0) / (rejected+accepted) << std::endl;

	return samples;
}

///
/// MCMC Sampler
///

///
/// Get a series of samples for the problem space
///
/// @param no_samples Number of samples to get
/// @param time Boolean that determines if the time to run the proccess is displayed 
/// @return A series of samples of shape (number of samples, sample dimension)
///
MatrixXd MCMCSampler::sample(const int& no_samples, const bool& time) const
{
	bool verbose = false;
	std::cout << "Number of samples: " << no_samples << std::endl;
	if(verbose) std::cout << "Surfing" << std::endl;
	MatrixXd ski = MCMCSampler::grad_descent(alpha());
	if(verbose) std::cout << "Got Through Gradient Descent" << std::endl;

	// Last row of the ski is the start of the HMC algorithm
	VectorXd q = ski.row(ski.rows()-1);

	// Store the samples
	MatrixXd samples(1, problem().space_dimension() + 1);
	samples << q.transpose(), problem().get_cost(q);

	int accepted = 0;
	int rejected = 0;
	// If you want to time the sampling
	high_resolution_clock::time_point t1;
	if(time) t1 = high_resolution_clock::now();
	while(accepted < no_samples)
	{
		if(samples.rows() > 1)
		{
			std::cout << "New start!" << std::endl;
			MatrixXd ski = MCMCSampler::grad_descent();
			if(verbose) std::cout << "Got Through Gradient Descent in loop" << std::endl;

			// Last row of the ski is the start of the HMC algorithm
			q = ski.row(ski.rows()-1);
		}

		int curr_rejections = 0;
		int curr_step = 0;
		while(curr_rejections < 10 and accepted < no_samples and curr_step < steps())
		{
			VectorXd q_proposed = q + sample_normal(0, sigma());
			double prob_proposed = get_prob(q_proposed);
			double prob_before = get_prob(q);

			// std::cout << "Prob Proposed: " << prob_proposed << 
			// 			" | Cost Proposed: " << problem().get_cost(q_proposed) <<
			// 			" | Energy Proposed: " << get_energy(q_proposed) <<
			// 			" | Prob Before: " << prob_before << 
			// 			" | Cost Before: " << problem().get_cost(q) <<
			// 			" | Energy Before: " << get_energy(q) << std::endl;
			if(prob_proposed / prob_before >= rand_uni())
			{
				VectorXd newsample(problem().start_state().size() + 1);
		    	newsample << q_proposed, problem().get_cost(q_proposed);
				samples = concatenate_matrix_and_vector(samples, newsample);
				accepted++;
				q = q_proposed;
			}
			else 
			{ 
				rejected++; curr_rejections++;
			}
			curr_step++;
		}
	}

	std::cout << "Number of accepted: " << accepted << std::endl;
	// If you want to time the sampling and display it
	if(time)
	{
		high_resolution_clock::time_point t2 = high_resolution_clock::now();
		auto duration_s = duration_cast<seconds>( t2 - t1 ).count();
		auto duration_ms = duration_cast<milliseconds>( t2 - t1 ).count();
		auto duration_us = duration_cast<microseconds>( t2 - t1 ).count();
		if (duration_s != 0)
			std::cout << "Total Sampling Time: " << duration_s << "s" << std::endl;
		else if (duration_ms != 0)
			std::cout << "Total Sampling Time: " << duration_ms << "ms" << std::endl;
		else 
			std::cout << "Total Sampling Time: " << duration_us << "us" << std::endl;
	}
	std::cout << "Percentage Accepted: " << (accepted + 0.0) / (rejected+accepted) << std::endl;

	return samples;
}
