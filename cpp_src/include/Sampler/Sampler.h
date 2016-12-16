// Problem Definition
#pragma once

#include <ProblemDefinition/ProblemDefinition.h>

using Eigen::MatrixXd;

class Sampler
{
private:
	ProblemDefinition problem_;

public:
	///
	/// Constructor
	/// 
	/// @param problem Problem definition
	///
	Sampler(ProblemDefinition problem)
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
	virtual MatrixXd sample(const int& no_samples, const bool& time) const = 0;

	///
	/// Get the problem definition for the problem
	/// 
	/// @return The problem definition
	///
	ProblemDefinition problem() const { return problem_; }
};