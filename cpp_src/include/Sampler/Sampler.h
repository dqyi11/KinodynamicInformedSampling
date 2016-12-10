// Standard Libraries
#include <iostream>
#include <chrono>
using namespace std::chrono;

// Problem Definition
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
	virtual MatrixXd sample(int no_samples, bool time) = 0;

	///
	/// Get the problem definition for the problem
	/// 
	/// @return The problem definition
	///
	ProblemDefinition problem() { return problem_; }
};

// Example for how to inherit and create your own sampler
class RejectionSampler: public Sampler
{
public:
	RejectionSampler(ProblemDefinition problem)
		: Sampler(problem)
	{ }

	// Only function that you must implement
	MatrixXd sample(int no_samples, bool time) override
	{
		// Get the limits of the space
		VectorXd max_vals, min_vals;
		std::tie(max_vals, min_vals) = problem().state_limits();
		double max = max_vals(0); double min = min_vals(0);

	    // Run until you get the correct number of samples
	    int curr_no_samples = 0;
	    MatrixXd samples(no_samples, problem().start_state().size() + 1);

	    // If you want to time the sampling
	    high_resolution_clock::time_point t1;
	    if(time) t1 = high_resolution_clock::now();

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

	    // If you want to time the sampling and display it
	    if(time)
	    {
			high_resolution_clock::time_point t2 = high_resolution_clock::now();
			auto duration_s = duration_cast<seconds>( t2 - t1 ).count();
			auto duration_ms = duration_cast<milliseconds>( t2 - t1 ).count();
			auto duration_us = duration_cast<microseconds>( t2 - t1 ).count();
			if (duration_s != 0)
				std::cout << "Total Sampling Time: " << duration_s << "s" << std::endl;
			else if (duration_ms != 0)
				std::cout << "Total Sampling Time: " << duration_ms << "ms" << std::endl;
			else 
				std::cout << "Total Sampling Time: " << duration_us << "us" << std::endl;
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