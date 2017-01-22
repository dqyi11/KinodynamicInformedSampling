#include <OmplWrappers/OmplSamplers.h>

// stdlib
#include <vector>

// OMPL
#include <ompl/base/spaces/RealVectorStateSpace.h>

///
/// MyInformedSampler
///

///
/// Sample uniformly from the informed space
///
/// @param statePtr Pointer of the state you're sampling
/// @param maxCost Max cost of the informed subspace
/// @return true if a sample is gotten false, if not
///
bool ompl::base::MyInformedSampler::sampleUniform(State *statePtr, const Cost &maxCost)
{
	// if the informed subspace has changed or we've used all the samples
	// in the batch, resample
	if(maxCost.value() != prev_cost_ or sample_index_ >= sample_batch_size_)
	{
		if(maxCost.value() != prev_cost_) sampler_->update_level_set(maxCost.value());

		batch_samples_ = sampler_->sample(sample_batch_size_, false);

		sample_index_ = 0;
	}

	auto sample = batch_samples_.row(sample_index_);

	for(int i = 0; i < sample.size(); i++)
	{
		statePtr->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = sample(i);
	}

	sample_index_++;

	return true;
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
