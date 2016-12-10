#include "ProblemDefinition.h"
#include <algorithm>    // std::max

VectorXd ProblemDefinition::get_grad(VectorXd curr_state) 
{
	// Assert that the matrix is not empty
	assert(curr_state.size() != 0 or curr_state.size() != 0)

	// Derivative constant 
	double h = 0.001;

	// Loop through and calculate the gradients
	VectorXd grad(curr_state.size())
	for(int dim = 0; dim < state.size(); dim++)
	{
		MatrixXd state_plus = state; MatrixXd state_min = state;
		state_plus(dim) = state(dim) + h;
		state_min(dim) = state(dim) - h;
		grad(dim) = (self.get_cost(state_plus) - self.get_cost(state_min)) / (2*h);
	}

	return grad;
}

// bool ProblemDefinition::is_valid_constructor()
// {
// 	return (start_state_.size() == goal_state_.size() and state_min_.size() == start_state_.size()
// 			state_max_.size() == start_state_.size()) and
// 		   (start_state_.size() != 0 and goal_state_.size() != 0 and state_min_.size() != 0
// 		   	and start_max_.size() != 0);
// }