// Standard Libraries
#pragma once

#include <iostream>
#include <chrono>
using namespace std::chrono;

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
	virtual MatrixXd sample(const int& no_samples, const bool& time) const = 0;

	///
	/// Surf down the cost function to get to the levelset
	///
	/// @param alpha Learning rate
	/// @return Path to the level set
	///
	virtual MatrixXd grad_descent(const double& alpha = 0.5) const;

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
};

class HMCSampler: public MonteCarloSampler
{
public:
	/// 
	/// Constructor for HMC Sampler (calls super class constructor)
	///
	/// @param problem Problem definition
	///
	HMCSampler(const ProblemDefinition& problem, const double& alpha,
			   const double& L, const double& epsilon, const double& sigma,
			   const int& steps)
		: MonteCarloSampler(problem, alpha), L_(L), epsilon_(epsilon), sigma_(sigma),
		  steps_(steps)
	{ }

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
	/// Get steps - Number of steps to run HMC
	///
	double steps() const { return steps_; }

	///
	/// Get a series of samples for the problem space
	///
	/// @param no_samples Number of samples to get
	/// @param time Boolean that determines if the time to run the proccess is displayed 
	/// @return A series of samples of shape (number of samples, sample dimension)
	///
	virtual MatrixXd sample(const int& no_samples, const bool& time) const override;

private:
	// Distance of integration for HMC step
	double L_;

	// Integration constant for HMC
	double epsilon_;

	// Sigma for sampling the momentum as a normal distribution
	double sigma_;

	// Number of steps to run HMC
	double steps_;
};