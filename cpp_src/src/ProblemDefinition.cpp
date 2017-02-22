#include <ProblemDefinition/ProblemDefinition.h>
#include <algorithm>    // std::max
#include <iostream>

VectorXd ProblemDefinition::get_grad(const VectorXd& curr_state) const 
{
	// Assert that the matrix is not empty
	assert(curr_state.size() != 0 or curr_state.size() != 0);

	// Derivative constant 
	double h = 0.001;

	// Loop through and calculate the gradients
	VectorXd grad(curr_state.size());
	for(int dim = 0; dim < curr_state.size(); dim++)
	{
		VectorXd state_plus = curr_state; VectorXd state_min = curr_state;
		state_plus(dim) = curr_state(dim) + h;
		state_min(dim) = curr_state(dim) - h;
		grad(dim) = (get_cost(state_plus) - get_cost(state_min)) / (2*h);
	}

	return grad;
}

bool ProblemDefinition::is_valid_constructor() const
{
	return (start_state_.size() == goal_state_.size() and state_min_.size() == start_state_.size() and
			state_max_.size() == start_state_.size()) and
		   (start_state_.size() != 0 and goal_state_.size() != 0 and state_min_.size() != 0
		   	and state_max_.size() != 0);
}

VectorXd ProblemDefinition::get_inv_jacobian(const VectorXd& curr_state) const
{
	// Assert that the matrix is not empty
	assert(curr_state.size() != 0 or curr_state.size() != 0);

	// Derivative constant 
	double h = 0.001;

	// Loop through and calculate the gradients
	VectorXd inv_jacobian(curr_state.size());
	for(int dim = 0; dim < curr_state.size(); dim++)
	{
		VectorXd state_plus = curr_state; VectorXd state_min = curr_state;
		state_plus(dim) = curr_state(dim) + h;
		state_min(dim) = curr_state(dim) - h;
                double delta = get_cost(state_plus) - get_cost(state_min);
                //std::cout << "DELTA " << delta << std::endl;
                if(delta == 0.0) 
                {
                        inv_jacobian(dim) = 0.0;        
                } else 
                {
		        inv_jacobian(dim) = (2*h) / (delta);
                }
	}

	return inv_jacobian;

}
