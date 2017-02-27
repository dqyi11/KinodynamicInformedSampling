// Standard Libraries
#pragma once

#include <iostream>

// Eigen namespace
#include <Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Include Sampler
#include <Sampler/Sampler.h>

class MonteCarloSampler: public Sampler
{
public:
	///
	/// Constructor for HMC Sampler (calls super class constructor)
	///
	/// @param problem Problem definition
	///
	MonteCarloSampler(const ProblemDefinition& problem, const double& alpha)
		: Sampler(problem), alpha_(alpha)
	{ }

	///
	/// Get alpha - learning rate for gradient descent
	///
	double alpha() const { return alpha_; }

	///
	/// Get a series of samples for the problem space
	///
	/// @param no_samples Number of samples to get
	/// @param time Boolean that determines if the time to run the proccess is displayed
	/// @return A series of samples of shape (number of samples, sample dimension)
	///
	virtual MatrixXd sample(const int& no_samples, high_resolution_clock::duration& duration) = 0;

	///
	/// Surf down the cost function to get to the levelset
	///
	/// @param alpha Learning rate
	/// @return Path to the level set
	///
	virtual VectorXd grad_descent(const double& alpha = 0.01) const;

	///
	/// Surf down the cost function to get to the levelset
	///
	/// @param start Starting state
	/// @return Path to the level set
	///
        virtual VectorXd newton_raphson(const VectorXd& start) const;

    ///
	/// Get the energy of the state from the cost function
	///
	/// @param curr_state Current state to get the energy for
	/// @return Energy of the state
	///
	virtual double get_energy(const VectorXd& curr_state) const;

	///
	/// Get the probability of the state from the cost function
	///
	/// @param energy Energy of the state
	/// @return Probability of the state
	///
	virtual double get_prob(const double& energy) const;

	///
	/// Get the probability of the state from the cost function
	///
	/// @param curr_state Current state to get the energy for
	/// @return Probability of the state
	///
	virtual double get_prob(const VectorXd& curr_state) const;

	///
	/// Get one random uniform sample from the space
	///
	/// @return Random uniform vector from the space
	///
	virtual VectorXd get_random_sample() const;

	///
	/// Get a normal random vector for the momentum
	///
	/// @param mean Mean of the normal distribution
	/// @paramm sigma Sigma of the normal distribution
	/// @return A vector of momentum sampled from a random distribution
	///
	VectorXd sample_normal(const double& mean, const double& sigma) const;

private:
	// Learning rate for gradient descent
	double alpha_;

	//

protected:
	///
	/// Function to determine if any of the joint limits are violated
	/// @param sample Sample to check
	/// @return Boolean that is true if any are in violation
	///
	bool any_dimensions_in_violation(const VectorXd sample) const;
};

class HMCSampler: public MonteCarloSampler
{
public:
	///
	/// Constructor for HMC Sampler (calls super class constructor)
	///
	/// @param problem Problem definition
	/// @param alpha Learning rate for gradient descent
	/// @param L Distance of integration for HMC step
	/// @param epsilon Integration constant for HMC
	/// @param sigma Sampling the momentum as a normal distribution
	/// @param steps Number of steps to run HMC for each chain
	///
	HMCSampler(const ProblemDefinition& problem, const double& alpha,
			   const double& L, const double& epsilon, const double& sigma,
			   const int& steps)
		: MonteCarloSampler(problem, alpha), L_(L), epsilon_(epsilon), sigma_(sigma),
		  steps_(steps), current_step_(-1)
	{
                last_sample_ = VectorXd(problem.start_state().size()+1);
        }

	///
	/// Get L - Distance of integration for HMC step
	///
	double L() const { return L_; }

	///
	/// Get epsilon - Integration constant for HMC
	///
	double epsilon() const { return epsilon_; }

	///
	/// Get sigma - Sampling the momentum as a normal distribution
	///
	double sigma() const { return sigma_; }

	///
	/// Get steps - Number of steps to run HMC for each chain
	///
	double steps() const { return steps_; }

	///
	/// Get a series of samples for the problem space
	///
	/// @param no_samples Number of samples to get
	/// @param time Boolean that determines if the time to run the proccess is displayed
	/// @return A series of samples of shape (number of samples, sample dimension)
	///
	virtual MatrixXd sample(const int& no_samples, high_resolution_clock::duration& duration);

        virtual MatrixXd sample_batch_memorized(const int& no_samples, high_resolution_clock::duration& duration);
        virtual VectorXd sample_memorized();

private:
	// Distance of integration for HMC step
	double L_;

	// Integration constant for HMC
	double epsilon_;

	// Sigma for sampling the momentum as a normal distribution
	double sigma_;

	// Number of steps to run HMC
	double steps_;

        int current_step_;
        // Last sample
        VectorXd last_sample_;
};

class MCMCSampler: public MonteCarloSampler
{
public:
	///
	/// Constructor for HMC Sampler (calls super class constructor)
	///
	/// @param problem Problem definition
	/// @param alpha Learning rate for gradient descent
	/// @param sigma Sigma for sampling the step
	/// @param steps Number of steps to run MCMC for each chain
	///
	MCMCSampler(const ProblemDefinition& prob, const double& alpha,
				const double& sigma, const double& steps)
	: MonteCarloSampler(prob, alpha), sigma_(sigma), steps_(steps)
	{ }

	///
	/// Get a series of samples for the problem space
	///
	/// @param no_samples Number of samples to get
	/// @param time Boolean that determines if the time to run the proccess is displayed
	/// @return A series of samples of shape (number of samples, sample dimension)
	///
	virtual MatrixXd sample(const int& no_samples, high_resolution_clock::duration& duration);

	///
	/// Get Sigma for sampling the step
	///
	double sigma() const { return sigma_; }

	///
	/// Get number of steps to run for each MCMC chain
	///
	double steps() const { return steps_; }

private:
	// Sigma for sampling the step
	double sigma_;

	// Number of steps to run MCMC for each chain
	double steps_;
};
