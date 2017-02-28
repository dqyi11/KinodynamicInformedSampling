// Problem Definition
#pragma once

#include <chrono>
#include <ProblemDefinition/ProblemDefinition.h>

using namespace std::chrono;
using Eigen::MatrixXd;

class Sampler
{
protected:
	ProblemDefinition problem_;

public:
	///
	/// Constructor
	/// 
	/// @param problem Problem definition
	///
	Sampler(const ProblemDefinition& problem)
		: problem_(problem)
	{

	}

	///
	/// Get a series of samples for the problem space
	///
	/// @param no_samples Number of samples to get
	/// @param time Boolean that determines if the time to run the proccess is displayed 
	/// @return A series of samples of shape (number of samples, sample dimension)
	///
	virtual MatrixXd sample(const int& no_samples, high_resolution_clock::duration& duration) = 0;

	///
	/// Get the problem definition for the problem
	/// 
	/// @return The problem definition
	///
	ProblemDefinition problem() const { return problem_; }

	///
	/// Update the level set of the problem definition
	///
	/// @param level_set The new level_set
	///
	virtual void update_level_set(const double& level_set) { problem_.update_level_set(level_set); }
};
