#include <Sampler/RejectionSampler.h>

// Standard library
#include <limits>
#include <math.h>

///
/// RejectionSampler
///

///
/// Get a series of samples for the problem space
///
/// @param no_samples Number of samples to get
/// @param time Boolean that determines if the time to run the proccess is displayed
/// @return A series of samples of shape (number of samples, sample dimension)
///
MatrixXd RejectionSampler::sample(const int& no_samples, const bool& time) const
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

// Can implement as many private functions as you want to help do the sampling
VectorXd RejectionSampler::get_random_sample(const double& max, const double& min, const int& size) const
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

///
/// HierarchicalRejectionSampler
///

///
/// Get a series of samples for the problem space
///
/// @param no_samples Number of samples to get
/// @param time Boolean that determines if the time to run the proccess is displayed
/// @return A series of samples of shape (number of samples, sample dimension)
///
MatrixXd HierarchicalRejectionSampler::sample(const int& no_samples, const bool& time) const
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
		VectorXd sample(problem().start_state().size());
		HRS(0, problem().start_state().size() - 1, sample);

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

///
/// Get one sample using a recursive algorithm of heirarchical rejection sampling
///
/// @param start_index Start index of the hierarchical sample
/// @param end_index End index of the hierarchical sample
/// @param sample Reference to a sample that gets changed in place
/// @return (c_start, c_goal)
///
std::tuple<double, double>
HierarchicalRejectionSampler::HRS(const int &start_index, const int &end_index, VectorXd &sample) const
{
	// Initialize the costs
	double c_start = std::numeric_limits<double>::infinity();
	double c_goal = std::numeric_limits<double>::infinity();

	if(start_index == end_index)
	{
		while(get_cost(c_start) + get_cost(c_goal) > problem().level_set())
		{
			sample_leaf(sample, start_index);
			c_start = calculate_leaf(problem().start_state(), sample, start_index);
			c_goal = calculate_leaf(sample, problem().goal_state(), start_index);
		}
	}
	else
	{
		int mid_index = std::floor(start_index + end_index) / 2;
		double c_dash_start = std::numeric_limits<double>::infinity();
		double c_dash_goal = std::numeric_limits<double>::infinity();

		while(get_cost(c_start) + get_cost(c_goal) > problem().level_set())
		{
			std::tie(c_start, c_goal) = HRS(start_index, mid_index, sample);
			std::tie(c_dash_start, c_dash_goal) = HRS(mid_index + 1, end_index, sample);
			c_start = combine_costs(c_start, c_dash_start);
			c_goal = combine_costs(c_goal, c_dash_goal);
		}
	}

	return std::make_tuple(c_start, c_goal);
}

///
/// GeometricHierarchicalRejectionSampler
///

///
/// Calculates the cost of a leaf node
///
/// @param x1 First state
/// @param x2 Second state
/// @param i Index of the degree of freedom
/// @return Cost to go from x1 to x2
///
double GeometricHierarchicalRejectionSampler::calculate_leaf(const VectorXd &x1,
															 const VectorXd &x2,
															 const int &i) const
{
	return std::pow(x1[i] - x2[i], 2);
}

///
/// Combines the cost of two states
///
/// @param c1 Cost one
/// @param c2 Cost two
/// @return Combination of the costs
///
double GeometricHierarchicalRejectionSampler::combine_costs(const double &c1,
															const double &c2) const
{
	return c1 + c2;
}

///
/// How to sample a leaf (ex: geometric is one dimension and kino is 2)
///
/// @param sample A vector to the sample
/// @param dof An index to the degree of freedom to sample
/// @return A random vector in the space
///
void GeometricHierarchicalRejectionSampler::sample_leaf(VectorXd &sample,
													    const int dof) const
{
	// Get the limits of the state
	VectorXd max, min;
	std::tie(min, max) = problem().state_limits();

	// Set up the random number generator
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(min[dof], max[dof]);

	sample[dof] = dis(gen);
}
