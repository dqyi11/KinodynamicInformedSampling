#pragma once

// OMPL
#include <ompl/base/ProblemDefinition.h>
#include <ompl/control/SpaceInformation.h>

// Eigen
#include <Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::VectorXd;

// stdlib
#include <tuple>
#include <chrono>
using namespace std::chrono;

class Sampler
{
protected:
    ompl::base::SpaceInformationPtr si_;

    ompl::base::ProblemDefinitionPtr problem_;

    double level_set_;

public:
    ///
    /// Constructor
    ///
    /// @param si Pointer to the space information
    /// @param problem Problem deficition pointer
    /// @param level_set Level set of the problem
    ///
	Sampler(const ompl::base::SpaceInformationPtr& si,
            const ompl::base::ProblemDefinitionPtr& problem,
            const double level_set)
		: si_(si), problem_(problem), level_set_(level_set)
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
	ompl::base::ProblemDefinitionPtr problem() const { return problem_; }

    ///
    /// Get the space information pointer for the problem
    ///
    /// @return The space information
    ///
    ompl::base::SpaceInformationPtr si() const { return si_; }

	///
	/// Update the level set of the problem definition
	///
	/// @param level_set The new level_set
	///
	virtual void update_level_set(const double& level_set) { level_set_ = level_set; }

    ///
    /// Get the state limits of the space
    ///
    /// @return Tuple(state_max, state_min)
    ///
    std::tuple<Eigen::VectorXd, Eigen::VectorXd> state_limits() const;

    ///
    /// Get the start state
    ///
    /// @return Start state defined in the constructor
    ///
    VectorXd start_state() const;

    ///
    /// Get the goal state
    ///
    /// @return Start state defined in the constructor
    ///
    VectorXd goal_state() const;

    ///
    /// Get the level set
    ///
    /// @return Get the level set of the cost function you want to sample from
    ///
    double level_set() const { return level_set_; }

    ///
    /// Get the dimension of the space
    ///
    /// @return Get the level set of the cost function you want to sample from
    ///
    uint space_dimension() const { return si_->getStateSpace()->getDimension(); }

    ///
    /// Get the cost for a specific state
    ///
    /// @param curr_state Current state to get the cost for
    /// @return Cost at that state
    ///
    virtual double get_cost(const VectorXd& curr_state) const;

    ///
    /// Determines if a sample is within the cost function level set
    ///
    /// @param state State to test
    /// @return Boolean that is true if it is in the level set
    ///
    virtual bool is_in_level_set(const VectorXd& state) const { return get_cost(state) <= level_set_; }

    ///
    /// Get the gradient of the cost function at a specific state
    ///
    /// @param curr_state Current state to get the cost for
    /// @return Gradient of the function at the current state (same dimension as current state)
    ///
    virtual VectorXd get_grad(const VectorXd& curr_state) const;

    ///
    /// Get the Inverse Jacobian of the cost function at a specific state
    ///
    /// @param curr_state Current state to get the cost for
    /// @return Inverse Jacobian of the function at the current state (same dimension as current state)
    ///
    virtual VectorXd get_inv_jacobian(const VectorXd& curr_state) const;
};
