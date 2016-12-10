#include <iostream>
#include <Eigen/Dense>
using Eigen::MatrixXd;

// #include "ProblemDefinition.h"
#include "Sampler.h"

// Example for how to inherit and create your own sampler
class RejectionSampler: public Sampler
{
public:
	RejectionSampler(ProblemDefinition problem)
		: Sampler(problem)
	{ }

	// Only function that you must implement
	MatrixXd sample(int no_samples) override
	{
		// Get the limits of the space
		VectorXd max_vals, min_vals;
		std::tie(max_vals, min_vals) = problem().state_limits();
		double max = max_vals(0); double min = min_vals(0);

	    // Run until you get the correct number of samples
	    int curr_no_samples = 0;
	    MatrixXd samples(no_samples, problem().start_state().size() + 1);

	    while(curr_no_samples < no_samples)
	    {
	    	VectorXd sample = get_random_sample(max, min, problem().start_state().size());

	    	if(problem().is_in_level_set(sample))
	    	{
	    		VectorXd newsample(problem().start_state().size() + 1);
	    		newsample << sample, problem().get_cost(sample);
	    		samples.row(curr_no_samples) = newsample;
	    		curr_no_samples++;
	    	}
	    }

	    return samples;
	}

private:
	// Can implement as many private functions as you want to help do the sampling
	VectorXd get_random_sample(double max, double min, int size)
	{
		// Set up the random number generator
		std::random_device rd;
		std::mt19937 gen(rd());
		std::uniform_real_distribution<> dis(min, max);

		VectorXd sample(size);

		for(int i = 0; i < size; i++)
		{
			sample(i) = dis(gen);
		}

  		return sample;
	}
};

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
	MatrixXd samples = s.sample(10);

	std::cout << "Got the samples" << std::endl;

	// Print Samples
	for(int i = 0; i < samples.rows(); i++)
	{
		std::cout << "Sample: (" << samples.row(i)(0) << "," << samples.row(i)(1) << ") | Cost: "
				  << samples.row(i)(2) << std::endl;
	}
}