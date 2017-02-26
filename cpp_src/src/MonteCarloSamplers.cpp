#include <Sampler/MonteCarloSamplers.h>

// Standard library functions
#include <math.h>       /* exp, tanh, log */
#include <limits>
#include <algorithm>

///
/// Helper functions
///

void print_out_states3(const VectorXd &state)
{
	std::cout << "[ ";
	for(uint i = 0; i < state.size(); i++)
	{
		std::cout << state[i] << " ";
	}
	std::cout << " ]" << std::endl;
}

// Verbose constant
const bool VERBOSE = false;

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
/// Function to determine if any of the joint limits are violated
/// @param sample Sample to check
/// @return Boolean that is true if any are in violation
///
bool MonteCarloSampler::any_dimensions_in_violation(const VectorXd sample) const
{
	const std::tuple<VectorXd, VectorXd> limits = problem().state_limits();
	const auto min_vals = std::get<0>(limits);
	const auto max_vals = std::get<1>(limits);

	for(uint i = 0; i < sample.size(); i++)
	{
		if(sample[i] > max_vals[i] or sample[i] < min_vals[i])
		{
			return true;
		}
	}

	return false;
}

///
/// Get the energy of the state from the cost function
///
/// @param curr_state Current state to get the energy for
/// @return Energy of the function
///
double MonteCarloSampler::get_energy(const VectorXd& curr_state) const
{
    const double cost = problem().get_cost(curr_state);
    // double E_grad = tanh(cost);
    const double E_grad = log(1+log(1+cost));
    const double E_informed = 100 * sigmoid(cost - problem().level_set());
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

// ///
// /// Surf down the cost function to get to the levelset
// ///
// /// @param alpha Learning rate
// /// @return Path to the level set
// ///
// VectorXd MonteCarloSampler::grad_descent(const double& alpha) const
// {
// 	VectorXd start = MonteCarloSampler::get_random_sample();
// 	double cost = problem().get_cost(start);

// 	int steps = 0;
//     while(cost > problem().level_set())
//     {
//         double last_cost = cost;
//         VectorXd inv_jacobian = problem().get_inv_jacobian(start);
//         //std::cout << "inv jacobian " << inv_jacobian << std::endl;
//         start = start - inv_jacobian * cost;
// 		cost = problem().get_cost(start);
// 		steps++;

// 		// If the number of steps reaches some threshold, start over
// 		const double thresh = 50;
// 		if( abs(last_cost - cost) < 0.0001 )
//         //if(steps > thresh)
// 		{
// 			//std::cout << "RESTART GRAD DESCENT" << std::endl;
// 			//return grad_descent();
// 			start = MonteCarloSampler::get_random_sample();
// 			cost = problem().get_cost(start);
// 		}
// 	}

// 	return start;
// }

///
/// Surf down the cost function to get to the levelset
///
/// @param alpha Learning rate
/// @return Path to the level set
///
VectorXd MonteCarloSampler::grad_descent(const double& alpha) const
{
	VectorXd start = MonteCarloSampler::get_random_sample();
	double cost = problem().get_cost(start);

	int steps = 0;
	while(cost > problem().level_set())
	{
		VectorXd grad = problem().get_grad(start);
		start = start - alpha * grad;

		cost = problem().get_cost(start);

		if(VERBOSE) std::cout << cost << std::endl;

		steps++;

		// If the number of steps reaches some threshold, start over
		const double thresh = 20;
		if(steps > thresh)
		{
			if(VERBOSE) std::cout << "Restarting!" << std::endl;
			return grad_descent(alpha);
		}
	}

	return start;
}

///
/// Surf down the cost function to get to the levelset
///
/// @param start Vector to start
/// @return A state in the level set
///
VectorXd MonteCarloSampler::newton_raphson(const VectorXd& start) const
{
    VectorXd end = start;
	double cost = problem().get_cost(end);

	int steps = 0;
    while(cost > problem().level_set())
    {
        double last_cost = cost;
        VectorXd inv_jacobian = problem().get_inv_jacobian(end);
        end = end - inv_jacobian * cost;
		cost = problem().get_cost(end);
		steps++;

		// If the number of steps reaches some threshold, start over
        const double trap_threshold = 0.0001;
		if( last_cost - cost < trap_threshold )
		{
            end = MonteCarloSampler::get_random_sample();
            cost = problem().get_cost(end);
		}
	}

	return end;
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

MatrixXd HMCSampler::sample_batch_memorized(const int& no_samples, const bool& time)
{
	MatrixXd samples(1, problem().space_dimension() + 1);
	// If you want to time the sampling
	high_resolution_clock::time_point t1;
	if(time) t1 = high_resolution_clock::now();

        for(unsigned int i=0;i<no_samples;i++)
	{
		VectorXd newsample = sample_memorized();
		samples = concatenate_matrix_and_vector(samples, newsample);
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

	return samples;
}

VectorXd HMCSampler::sample_memorized()
{
        // last sample
	VectorXd q = VectorXd(problem().start_state().size());
        for(unsigned int i=0;i<problem().start_state().size();i++)
        {
                q[i] = last_sample_[i];
        }

        if(current_step_ < 0 ){
		VectorXd start = MonteCarloSampler::get_random_sample();
                q = newton_raphson(start);
                //q = grad_descent(alpha());
        }
	current_step_++;

	// Sample the momentum and set up the past and current state and momentum
	VectorXd q_last = q;
	VectorXd p = MonteCarloSampler::sample_normal(0, sigma());
	VectorXd p_last = p;

	if(VERBOSE) std::cout << "Sampled the momentum" << std::endl;

	// Make a half step for momentum at the beginning
	VectorXd grad = problem().get_grad(q);
	if(VERBOSE) std::cout << "Got the gradient" << std::endl;

	// Ensure that the gradient isn't two large
	while(grad.maxCoeff() > 1e2)
	{
		if(VERBOSE) std::cout << "WARNING: Gradient too high" << std::endl;

		VectorXd start = MonteCarloSampler::get_random_sample();
                q = newton_raphson(start);
                grad = problem().get_grad(q);
	}

	p = p - epsilon() * grad / 2;

	// Alternate Full steps for q and p
	for(int i = 0; i < L(); i++)
	{
		q = q + epsilon() * p;
		if(i != L()) p = p - epsilon() * grad;
	}

	if(VERBOSE) std::cout << "Integrated Along momentum" << std::endl;


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

	if(VERBOSE) std::cout << "Got energies" << std::endl;

	// Accept or reject the state at the end of trajectory
	double alpha = std::min(1.0, std::exp(U_last-U_proposed+K_last-K_proposed));
	if (rand_uni() > alpha)
	{
		q = q_last;
	}

    VectorXd newsample(problem().start_state().size() + 1);
	newsample << q, problem().get_cost(q);

    last_sample_ = newsample;
    if (current_step_ >= steps())
    {
	   current_step_ = -1;
    }

	return newsample;
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
	if(VERBOSE) std::cout << "Number of samples: " << no_samples << std::endl;
	if(VERBOSE) std::cout << "Surfing" << std::endl;
	VectorXd q = HMCSampler::grad_descent(alpha());
	if(VERBOSE) std::cout << "Got Through Gradient Descent" << std::endl;

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
		if(VERBOSE) std::cout << "New start!" << std::endl;
		VectorXd q = HMCSampler::grad_descent(alpha());
		if(VERBOSE) std::cout << "Got Through Gradient Descent in loop" << std::endl;

		int curr_rejections = 0;
		int curr_step = 0;
		while(curr_rejections < 10 and accepted < no_samples and curr_step < steps())
		{
			// Sample the momentum and set up the past and current state and momentum
			VectorXd q_last = q;
			VectorXd p = MonteCarloSampler::sample_normal(0, sigma());
			VectorXd p_last = p;

			if(VERBOSE) std::cout << "Sampled the momentum" << std::endl;

			// Make a half step for momentum at the beginning
			VectorXd grad = problem().get_grad(q);
			if(VERBOSE) std::cout << "Got the gradient" << std::endl;

			// Ensure that the gradient isn't two large
			if(grad.maxCoeff() > 1e2)
			{
				if(VERBOSE) std::cout << "WARNING: Gradient too high" << std::endl;
				break;
			}

			p = p - epsilon() * grad / 2;

			// Alternate Full steps for q and p
			for(int i = 0; i < L(); i++)
			{
				q = q + epsilon() * p;
				if(i != L()) p = p - epsilon() * grad;
			}

			if(VERBOSE) std::cout << "Integrated Along momentum" << std::endl;


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

			if(VERBOSE) std::cout << "Got energies" << std::endl;


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
			if(VERBOSE) std::cout << "Decided on rejection / acceptance" << std::endl;
			if(VERBOSE) std::cout << "Number Accepted: " << accepted << std::endl;
			if(VERBOSE) std::cout << "Number Accepted: " << accepted << std::endl;

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
	if(VERBOSE) std::cout << "Percentage Accepted: " << (accepted + 0.0) / (rejected+accepted) << std::endl;

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
	if(VERBOSE) std::cout << "Number of samples: " << no_samples << std::endl;
	if(VERBOSE) std::cout << "Surfing" << std::endl;
	VectorXd q = MCMCSampler::grad_descent(alpha());
	if(VERBOSE) std::cout << "Got Through Gradient Descent" << std::endl;

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
			if(VERBOSE) std::cout << "New start!" << std::endl;
			VectorXd q = MCMCSampler::grad_descent(alpha());
			if(VERBOSE) std::cout << "Got Through Gradient Descent in loop" << std::endl;
		}

		int curr_rejections = 0;
		int curr_step = 0;
		while(curr_rejections < 10 and accepted < no_samples and curr_step < steps())
		{
			VectorXd q_proposed = q + sample_normal(0, sigma());
			double prob_proposed = get_prob(q_proposed);
			double prob_before = get_prob(q);

			// if(prob_proposed / prob_before >= rand_uni() and
			//    // !any_dimensions_in_violation(q_proposed))
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
				if(VERBOSE) std::cout << "Rejected!" << std::endl;
			}
			curr_step++;
		}

		if(VERBOSE) std::cout << "Number of accepted: " << accepted << std::endl;
	}

	if(VERBOSE) std::cout << "Number of accepted: " << accepted << std::endl;
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
	if(VERBOSE) std::cout << "Percentage Accepted: " << (accepted + 0.0) / (rejected+accepted) << std::endl;

	return samples;
}
