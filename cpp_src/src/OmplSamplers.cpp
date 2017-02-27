#include <OmplWrappers/OmplSamplers.h>

// stdlib
#include <vector>
#include <limits>

// OMPL
#include <ompl/base/spaces/RealVectorStateSpace.h>

// Eigen
using Eigen::VectorXd;

// Our stuff
#include <Dimt/Params.h>

///
/// Helper functions
///

bool same_cost(double a, double b)
{
    return std::fabs(a - b) < std::numeric_limits<double>::epsilon();
}

void print_out_states2(ompl::base::State *statePtr)
{
    double * val = static_cast<ompl::base::RealVectorStateSpace::StateType*>(statePtr)->values;

    std::cout << "Printing sample of size: " << param.dimensions << " | Vec: [ ";
    for(uint i = 0; i < param.dimensions; i++)
    {
        std::cout << val[i] << " ";
    }
    std::cout << "]" << std::endl;
}

void print_out_states2(const VectorXd &state)
{
	std::cout << "[ ";
	for(uint i = 0; i < state.size(); i++)
	{
		std::cout << state[i] << " ";
	}
	std::cout << " ]" << std::endl;
}

///
/// MyInformedSampler
///

// Can implement as many private functions as you want to help do the sampling
double ompl::base::MyInformedSampler::get_random_dimension(const double& max, const double& min) const
{
	// Set up the random number generator
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(min, max);

	return dis(gen);
}

///
/// Function to sample a state uniformly from the entire space before you have
/// a solution
///
/// @param statePtr Pointer to the state to sample
///
bool ompl::base::MyInformedSampler::sample_full_space(State *statePtr)
{
	// Get the limits of the space
	VectorXd max_vals, min_vals;
	std::tie(max_vals, min_vals) = sampler_->problem().state_limits();

	double * val = static_cast<ompl::base::RealVectorStateSpace::StateType*>(statePtr)->values;
	for(int i = 0; i < param.dimensions; i++)
	{
		val[i] = get_random_dimension(max_vals[i], min_vals[i]);
	}

	return true;
}

///
/// Function to sample uniformly from the informed subset
///
/// @param statePtr Pointer to the state to sample
/// @param maxCost Best cost found so far
///
bool ompl::base::MyInformedSampler::sample_informed_space(State *statePtr, const Cost maxCost)
{
	// if the informed subspace has changed or we've used all the samples
	// in the batch, resample
    if(!same_cost(maxCost.value(), prev_cost_) or sample_index_ >= sample_batch_size_)
	{
        std::cout << "Cost: " << maxCost.value();
        // std::cout << " | Sample Index: " << sample_index_;
        std::cout << " | Prev cost: " << prev_cost_;
        // std::cout << " | Sample batch size: " << sample_batch_size_;
        std::cout << " | Level set: " << sampler_->problem().level_set() << std::endl;

		if(maxCost.value() != prev_cost_) sampler_->update_level_set(maxCost.value());

        high_resolution_clock::duration duration;
		batch_samples_ = sampler_->sample(sample_batch_size_, duration);

		// for(uint i = 0; i < batch_samples_.rows(); i++)
		// {
		// 	print_out_states2(batch_samples_.row(i));
		// }

        prev_cost_ = maxCost.value();
		sample_index_ = 0;
	}

	auto sample = batch_samples_.row(sample_index_);

	double * val = static_cast<ompl::base::RealVectorStateSpace::StateType*>(statePtr)->values;
	for(int i = 0; i < sample.size() - 1; i++)
	{
		val[i] = sample(i);
	}

	// std::cout << "State after getting a sample: ";
	// print_out_states2(statePtr);

	sample_index_++;

	return true;
}

///
/// Sample uniformly from the informed space
///
/// @param statePtr Pointer of the state you're sampling
/// @param maxCost Max cost of the informed subspace
/// @return true if a sample is gotten false, if not
///
bool ompl::base::MyInformedSampler::sampleUniform(State *statePtr, const Cost &maxCost)
{
	if(maxCost.value() == std::numeric_limits<double>::infinity())
	{
		return sample_full_space(statePtr);
	}
	else
	{
		return sample_informed_space(statePtr, maxCost);
	}
}

///
/// Just call sampleUniform(statePtr, maxCost) - there is no mincost
///
/// @param statePtr Pointer of the state you're sampling
/// @param maxCost Max cost of the informed subspace
/// @param minCost Minimum cost of the informed subspace
/// @return true if a sample is gotten false, if not
///
bool ompl::base::MyInformedSampler::sampleUniform(State *statePtr,
												  const Cost &minCost,
												  const Cost &maxCost)
{
	return sampleUniform(statePtr, maxCost);
}

///
/// Function that lets the planner know if we have an informed measure
///
/// @return True if we have implemented an informed measure, false if not
///
bool ompl::base::MyInformedSampler::hasInformedMeasure() const
{
	return false;
}

///
/// Function to return the measure of the informed space
///
/// @param currentCost - Current cost of the best path
/// @return Measure of the informed space
///
double ompl::base::MyInformedSampler::getInformedMeasure(const Cost &currentCost) const
{
	return InformedSampler::space_->getMeasure();
}
