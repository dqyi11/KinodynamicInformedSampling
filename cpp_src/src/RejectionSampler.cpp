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
MatrixXd RejectionSampler::sample(const int& no_samples, high_resolution_clock::duration& duration)
{
	// Get the limits of the space
	VectorXd max_vals, min_vals;
	std::tie(max_vals, min_vals) = problem().state_limits();
	double max = max_vals(0); double min = min_vals(0);

	// Run until you get the correct number of samples
	int curr_no_samples = 0;
	MatrixXd samples(no_samples, problem().start_state().size() + 1);

	// If you want to time the sampling
	high_resolution_clock::time_point t1 = high_resolution_clock::now();

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

    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    duration = t2 - t1;

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
MatrixXd HierarchicalRejectionSampler::sample(const int& no_samples, high_resolution_clock::duration& duration)
{
	// Get the limits of the space
	VectorXd max_vals, min_vals;
	std::tie(max_vals, min_vals) = problem().state_limits();
	double max = max_vals(0); double min = min_vals(0);

	// Run until you get the correct number of samples
	int curr_no_samples = 0;
	MatrixXd samples(no_samples, problem().start_state().size() + 1);

	// If you want to time the sampling
	high_resolution_clock::time_point t1 = high_resolution_clock::now();

	while(curr_no_samples < no_samples)
	{
		VectorXd sample(problem().start_state().size());
		HRS(0, dimension_ - 1, sample);

		if(problem().is_in_level_set(sample))
		{
			VectorXd newsample(problem().start_state().size() + 1);
			newsample << sample, problem().get_cost(sample);
			samples.row(curr_no_samples) = newsample;
			curr_no_samples++;
		}
	}

    // If you want to time the sampling and display it
    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    duration = t2 - t1;

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
HierarchicalRejectionSampler::HRS(const int &start_index, const int &end_index, VectorXd &sample)
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
			c_start = combine_costs(problem().start_state(), sample, start_index, mid_index,
                                    end_index, c_start, c_dash_start);
			c_goal = combine_costs(sample, problem().goal_state(), start_index, mid_index,
                                   end_index, c_goal, c_dash_goal);
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
															 const int &i)
{
	return std::pow(x1[i] - x2[i], 2);
}

///
/// Combines the cost of two states
///
/// @param x1 First state
/// @param x2 Second state
/// @param i Index of the degree of freedom for first state
/// @param m Mid degree of freedom
/// @param j Index of  the degree of freedom of the second state
/// @param c1 Cost one
/// @param c2 Cost two
/// @return Combination of the costs
///
double GeometricHierarchicalRejectionSampler::combine_costs(const VectorXd &x1,
                                                            const VectorXd &x2,
                                                            const int i,
                                                            const int m,
                                                            const int j,
                                                            const double &c1,
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
													    const int dof)
{
	std::uniform_real_distribution<double> dis(min_[dof], max_[dof]);

	sample[dof] = dis(gen_);
}

///
/// Dimt Hierarchical Rejection Sampler
///

///
/// Calculates the cost of a leaf node
///
/// @param x1 First state
/// @param x2 Second state
/// @param i Index of the degree of freedom
/// @return Cost to go from x1 to x2
///
double DimtHierarchicalRejectionSampler::calculate_leaf(const VectorXd &x1,
                                                        const VectorXd &x2,
                                                        const int &i)
{
    const auto state1 = x1.segment(i, 2);
    const auto state2 = x2.segment(i, 2);
    const Eigen::Matrix<double, 1, 1> distances = state2.template head<1>() - state1.template head<1>();
    Eigen::Matrix<double, 1, 1> firstAccelerations;

    double cost, infeasible_min, infeasible_max;
    std::tie(cost, infeasible_min, infeasible_max) =
        double_integrator_1dof_.getMinTimeAndIntervals(x1.segment(i, 2), x2.segment(i, 2));

    infeasible_intervals_[i] = std::make_pair(infeasible_min, infeasible_max);

    costs_[i] = cost;

    return costs_[i];
}

std::pair<size_t, double> max_in_range(const std::vector<double> &costs,
                                       const size_t start,
                                       const size_t end)
{
    double max = -1;
    size_t index = -1;
    for(size_t i = start; i <= end; i++)
    {
        if(costs[i] > max)
        {
            max = costs[i];
            index = i;
        }
    }

    return std::make_pair(index, max);
}

std::pair<bool, double> find_infeasible_intervals(const std::vector<std::pair<double,double>> &intervals,
                                                  const double max_val,
                                                  const size_t start,
                                                  const size_t end)
{
    for(size_t i = start; i <= end; i++)
    {
        if(max_val > std::get<0>(intervals[i]) and max_val < std::get<1>(intervals[i]))
        {
            return std::make_pair(true, std::get<1>(intervals[i]));
        }
    }

    return std::make_pair(false, max_val);
}
///
/// Combines the cost of two states
///
/// @param x1 First state
/// @param x2 Second state
/// @param i Index of the degree of freedom for first state
/// @param m Mid degree of freedom
/// @param j Index of  the degree of freedom of the second state
/// @param c1 Cost one
/// @param c2 Cost two
/// @return Combination of the costs
///
double DimtHierarchicalRejectionSampler::combine_costs(const VectorXd &x1,
                                                       const VectorXd &x2,
                                                       const int i,
                                                       const int m,
                                                       const int j,
                                                       const double &c1,
                                                       const double &c2) const
{
    // std::cout << "i: " << i << " | j: " << j << " | cost size: " << costs_.size() << std::endl;

    // Find the index and the max value of cost from the costs in the range
    size_t index;
    double max_val;
    std::tie(index, max_val) = max_in_range(costs_, i, j);

    bool is_invalid = true;
    while(is_invalid)
    {
        std::tie(is_invalid, max_val) = find_infeasible_intervals(infeasible_intervals_, max_val, i, j);
    }
    return std::max(c1, c2);
}

///
/// How to sample a leaf (ex: geometric is one dimension and kino is 2)
///
/// @param sample A vector to the sample
/// @param dof An index to the degree of freedom to sample
/// @return A random vector in the space
///
void DimtHierarchicalRejectionSampler::sample_leaf(VectorXd &sample,
                                                   const int dof)
{
    std::uniform_real_distribution<double> dis1(min_[dof], max_[dof]);
    std::uniform_real_distribution<double> dis2(min_[dof + 1], max_[dof + 1]);

    sample[dof] = dis1(gen_);
    sample[dof+1] = dis2(gen_);
}
