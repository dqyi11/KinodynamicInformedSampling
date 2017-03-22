#include "Sampler/HitAndRun.h"

const bool VERBOSE = false;

VectorXd GibbsSampler::get_random_sample(double min, double max, const int& dim)
{
	std::uniform_real_distribution<> dis(min, max);
	// Updates the member variable of the class as well
	prev_sample_(dim) = dis(gen_);
	return prev_sample_;
}

MatrixXd GibbsSampler::sample(const int& no_samples, high_resolution_clock::duration& duration)
{
	// Get the limits of the space
	VectorXd max_vals, min_vals;
	const int dim = start_state().size();
	std::tie(max_vals, min_vals) = state_limits();

	// Run until you get the correct number of samples
	MatrixXd samples(no_samples, dim + 1);

	// If you want to time the sampling
	high_resolution_clock::time_point t1 = high_resolution_clock::now();

	VectorXd sample;
	unsigned int skip = 0, trys=0;
	for(int i=0; i<no_samples; i++)
	{
		trys=0;
		do
		{
			if (trys > 10000)
			{
				skip++;
				trys=0;
			}
			sample = get_random_sample(min_vals[(i+skip)%dim], max_vals[(i+skip)%dim], (i+skip)%dim);
			trys++;
			if (VERBOSE) std::cout << "Trys:" << trys << " Skip:" << skip << std::endl;
		}
		while(!is_in_level_set(sample));
		VectorXd newsample(start_state().size() + 1);
		newsample << sample, get_cost(sample);
		samples.row(i) = newsample;
	}

    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    duration = t2 - t1;
    return samples;
}

void GibbsSampler::update_level_set(const double& level_set)
{
	prev_sample_ = start_state();
	update_level_set(level_set);
	// std::cout << "Updated Level Set" << std::endl;
}
