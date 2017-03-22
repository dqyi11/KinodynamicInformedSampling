#pragma once

// Standard Library Functions
#include <iostream>
#include <chrono>
using namespace std::chrono;
#include <tuple>
#include <math.h>

// Eigen namespace
#include <Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Include Sampler
#include <Sampler/Sampler.h>

class GibbsSampler: public Sampler
{
public:
	///
	/// Constructor for Gibbs Sampler
    ///
    /// @param si Pointer to the space information
    /// @param problem Problem deficition pointer
    /// @param level_set Level set of the problem
	///
	GibbsSampler(const ompl::base::SpaceInformationPtr& si,
                 const ompl::base::ProblemDefinitionPtr& problem,
                 const double level_set)
		: Sampler(si, problem, level_set)
    {
		// Set up the random number generator
		std::random_device rd;
		gen_ = std::mt19937(rd());

		// Use the start positions as starting point of the algorithm
		// (This will always be inside the informed subspace)
		prev_sample_ = start_state();
    }

	///
	/// Get a series of samples for the problem space
	///
	/// @param no_samples Number of samples to get
	/// @param time Boolean that determines if the time to run the proccess is displayed
	/// @return A series of samples of shape (number of samples, sample dimension)
	///
	virtual MatrixXd sample(const int& no_samples, high_resolution_clock::duration& duration);

	virtual void update_level_set(const double& level_set) override;

private:
	std::mt19937 gen_;
	VectorXd prev_sample_;

	///
	/// Get one random uniform sample from the space
	///
	/// @return Random uniform vector from the space
	///
    virtual VectorXd get_random_sample(double min, double max, const int& dim);
};
