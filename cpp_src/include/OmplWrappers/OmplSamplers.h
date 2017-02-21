#pragma once

//
// Steps we need to take in order to implement this correclty
//
// 1. We need to implement the required functions for
//    InformedSampler(http://ompl.kavrakilab.org/InformedStateSampler_8h_source.html). Here
//    is where the informed sampling needs to take place (and the majority of our code)
// 2. We need to take this information and pass it to the the InformedStateSampler,
//    which inherits from StateSampler and takes in and InformedSampler as
//    a constructor option.
// 3. Get this information about the samplers into the SpaceInformationPtr
// 4. Pass the SpaceInformationPtr to Informed RRT* and then done
//

// stdlib
#include <iostream>
#include <memory>

// OMPL
#include <ompl/base/samplers/InformedStateSampler.h>

// Internal Includes
#include <Sampler/Sampler.h>

using Eigen::MatrixXd;

///
/// Class that inherits from InformedSampler
///
namespace ompl
{
	namespace base
	{
		class MyInformedSampler: public InformedSampler
		{
		private:

			// One of the Samplers we have created previously
			std::shared_ptr<Sampler> sampler_;

			// Holds the previous cost so that we know
			double prev_cost_ = -1;

			// Holds the index of the sample we've gotten
			int sample_index_ = 0;

			// Number of samples to call at each iteration
			int sample_batch_size_;

			// Samples in the batch
			MatrixXd batch_samples_;

			///
			/// Function to sample a state uniformly from the entire space before you have
			/// a solution
			///
			/// @param statePtr Pointer to the state to sample
			///
			virtual bool sample_full_space(State *statePtr);

			///
			/// Function to sample uniformly from the informed subset
			///
			/// @param statePtr Pointer to the state to sample
			/// @param maxCost Best cost found so far
			///
			virtual bool sample_informed_space(State *statePtr, const Cost maxCost);

			///
			/// Function to get a random sample in the limits
			///
			/// @param max Max value for the dimension
			/// @param min Min value for the dimension
			/// @return Uniform sample in one of the dimensions between min and max
			///
			double get_random_dimension(const double& max, const double& min) const;


		public:
			///
			/// Constructor
			///
			/// @param probDefn OMPL's problem definition
			/// @param maxNumberCalls Max number of calls to the sampler
			/// @param sampler Sampler that inherits from Sampler.h
			/// @param sample_batch_size How many samples to get each time a new
			/// batch of samples is gotten
			///
			MyInformedSampler(const ProblemDefinitionPtr probDefn,
							  unsigned int maxNumberCalls,
							  const std::shared_ptr<Sampler>& sampler,
							  const int& sample_batch_size = 100)
				: InformedSampler(probDefn, maxNumberCalls),
				  sampler_(sampler),
				  sample_batch_size_(sample_batch_size)
			{

			}

			///
			/// Sample uniformly from the informed space
			///
			/// @param statePtr Pointer of the state you're sampling
			/// @param maxCost Max cost of the informed subspace
			/// @return true if a sample is gotten false, if not
			///
			virtual bool sampleUniform(State *statePtr, const Cost &maxCost) override;

			///
			/// Just call sampleUniform(statePtr, maxCost) - there is no mincost
			///
			/// @param statePtr Pointer of the state you're sampling
			/// @param maxCost Max cost of the informed subspace
			/// @param minCost Minimum cost of the informed subspace
			/// @return true if a sample is gotten false, if not
			///
			virtual bool sampleUniform(State *statePtr, const Cost &minCost, const Cost &maxCost) override;

			///
			/// Function that lets the planner know if we have an informed measure
			///
			/// @return True if we have implemented an informed measure, false if not
			///
			virtual bool hasInformedMeasure() const override;

			///
			/// Function to return the measure of the informed space
			///
			/// @param currentCost - Current cost of the best path
			/// @return Measure of the informed space
			///
			virtual double getInformedMeasure(const Cost &currentCost) const override;
		};
	}
}
