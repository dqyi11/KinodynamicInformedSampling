#include <OmplWrappers/OmplSamplers.h>

// stdlib
#include <vector>

// OMPL
#include <ompl/base/spaces/RealVectorStateSpace.h>

// DIMT stuff
#include <Dimt/Params.h>

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
	// auto state = statePtr->as<ompl::base::RealVectorStateSpace::StateType>()->values;
	// std::vector<double> state_vector_before(state, state + sizeof state / sizeof state[0]);
	// std::cout << "State vector size before: " << state_vector_before.size() << std::endl;

	// if the informed subspace has changed or we've used all the samples
	// in the batch, resample
	if(maxCost.value() != prev_cost_ or sample_index_ >= sample_batch_size_)
	{
		if(maxCost.value() != prev_cost_) sampler_->update_level_set(maxCost.value());

		batch_samples_ = sampler_->sample(sample_batch_size_, false);

		sample_index_ = 0;
	}

	// std::cout << "Got samples. About to give one!" << std::endl;

	auto sample = batch_samples_.row(sample_index_);

	std::cout << "Sample size: " << sample.size() << std::endl;

	// std::cout << "Got one sample!" << std::endl;
	double * val = static_cast<ompl::base::RealVectorStateSpace::StateType*>(statePtr)->values;
	for(int i = 0; i < sample.size(); i++)
	{
		val[i] = sample(i);
	}

	assert(si_->isValid(statePtr));
	// state = statePtr->as<ompl::base::RealVectorStateSpace::StateType>()->values;
	// std::vector<double> state_vector_after(state, state + sizeof state / sizeof state[0]);
	// std::cout << "State vector size after: " << state_vector_after.size() << std::endl;

	// std::cout << "Converted sample!" << std::endl;

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
