// Standard Library
#pragma once

#include <functional>
#include <stdexcept>      // std::invalid_argument
#include <tuple>

// Eigen
#include <Eigen/Dense>
using Eigen::VectorXd;

using CostFxn = std::function<double(VectorXd)>;

class ProblemDefinition
{
private:
	// The start of the plan 
	VectorXd start_state_;

	// The end of the plan
	VectorXd goal_state_;

	// Limits in each othe dimensions
	VectorXd state_max_;
	VectorXd state_min_;

	// Level set
	double level_set_;

	// Cost function
	CostFxn cost_;

	// Helper function to ensure that constructor is good
	// bool is_valid_constructor();

public:
	///
	/// Constructor 
	///
	/// @param start_state Start state of the plan
	/// @param goal_state Goal state of the plan
	/// @param state_min Minimum value of each state
	/// @param state_max Maximum value of each state
	/// @param level_set Levelset of the cost function that you want to sample from
	/// @param cost Function that determines the cost as a double given a VectorXd state
	/// 
	ProblemDefinition(VectorXd start_state, VectorXd goal_state, VectorXd state_min, 
					  VectorXd state_max, double level_set, CostFxn cost)
		: start_state_(start_state), goal_state_(goal_state), state_max_(state_max), 
		  state_min_(state_min), level_set_(level_set), cost_(cost)
	{
		// Assert that the start, goal, and limits are the same size
		if(!this->is_valid_constructor())
		{
			std::string error_msg = "Invalid Arguments of Shapes into Constructor.";
			throw std::invalid_argument(error_msg);
		}
	}

	///
	/// Get the start state
	///
	/// @return Start state defined in the constructor
	///
	VectorXd start_state() { return start_state_; }

	///
	/// Get the goal state
	///
	/// @return Start state defined in the constructor
	///
	VectorXd goal_state() { return goal_state_; }

	///
	/// Get the level set
	///
	/// @return Get the level set of the cost function you want to sample from
	///
	double level_set() { return level_set_; }

	///
	/// Get the state limits
	///
	/// @return Tuple(state_max, state_min)
	///
	std::tuple<VectorXd, VectorXd> state_limits() { return std::make_tuple(state_min_, state_max_); }

	///
	/// Get the cost for a specific state
	///
	/// @param curr_state Current state to get the cost for
	/// @return Cost at that state 
	///
	virtual double get_cost(VectorXd curr_state) { return this->cost_(curr_state); }

	///
	/// Helper function to help determine if constructor is invalid
	///
	/// @return Boolean to determine if the constructor is invalid
	///
	virtual bool is_valid_constructor();

	///
	/// Determines if a sample is within the cost function level set
	///
	/// @param state State to test
	/// @return Boolean that is true if it is in the level set
	///
	virtual bool is_in_level_set(VectorXd state) { return cost_(state) <= level_set_; }

	///
	/// Get the gradient of the cost function at a specific state
	///
	/// @param curr_state Current state to get the cost for
	/// @return Gradient of the function at the current state (same dimension as current state)
	///
	virtual VectorXd get_grad(VectorXd curr_state);
};