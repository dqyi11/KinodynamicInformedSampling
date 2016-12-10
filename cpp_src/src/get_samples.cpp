#include <iostream>
#include <Eigen/Dense>
using Eigen::MatrixXd;

// #include <ProblemDefinition/ProblemDefinition.h>
#include <Sampler/Sampler.h>

int main()
{
	// 
	// Example for how to use the above sampler
	// 

	// Create a problem definition
	VectorXd start_state(2);
	start_state << 0, 0;
	VectorXd goal_state(2);
	goal_state << 0, 1;
	VectorXd state_min(2);
	state_min << -5, -5;
	VectorXd state_max(2);
	state_max << 5, 5;
	double level_set = 3;
	ProblemDefinition prob = ProblemDefinition(start_state, goal_state, state_min, state_max, level_set,
		[start_state, goal_state](const VectorXd& state)
		{
			return (start_state - state).norm() + (goal_state - state).norm();
		});

	// Initialize the sampler
	RejectionSampler s = RejectionSampler(prob);
	std::cout << "Created the sampler" << std::endl;

	// Sampler
	MatrixXd samples = s.sample(1000, true);

	std::cout << "Got the samples" << std::endl;

	// Print Samples
	// for(int i = 0; i < samples.rows(); i++)
	// {
	// 	std::cout << "Sample: (" << samples.row(i)(0) << "," << samples.row(i)(1) << ") | Cost: "
	// 			  << samples.row(i)(2) << std::endl;
	// }
}