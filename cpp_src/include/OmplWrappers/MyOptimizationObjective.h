#pragma once

// stdlib
#include <iostream>
#include <memory>

// Eigen
#include <Eigen/Dense>
using Eigen::VectorXd;

// OMPL
#include <ompl/base/OptimizationObjective.h>

// Internal packages
#include <Sampler/Sampler.h>

using MotionCostFxn = std::function<double(VectorXd, VectorXd)>;
using StateCostFxn = std::function<double(VectorXd)>;

///
/// Class that inherits from InformedSampler
///
namespace ompl
{
	namespace base
	{
		class MyOptimizationObjective: public OptimizationObjective
		{
		private:

			// Our problem definition
			StateCostFxn state_cost_function_;

			// Motion cost function
			MotionCostFxn motion_cost_function_;

			// Pointer to informed sampler
			std::shared_ptr<Sampler> sampler_ptr_;

			// Number of samples to get in a batch
			double batch_size_;

		public:

			///
			/// Constructor
			///
			/// @param si Space Information
			/// @param state_cost_function Cost to go of individual state
			/// @param motion_cost_function Cost function between two states
			/// @param informed_sampler Informed sampler
			///
			MyOptimizationObjective(const SpaceInformationPtr& si,
									const std::shared_ptr<Sampler>& sampler_ptr,
									const double& batch_size,
									const StateCostFxn& state_cost_function,
									const MotionCostFxn& motion_cost_function)
				: OptimizationObjective(si),
				  state_cost_function_(state_cost_function),
				  motion_cost_function_(motion_cost_function),
				  sampler_ptr_(sampler_ptr),
				  batch_size_(batch_size)
			{ }

            ///
            /// Constructor
            ///
            /// @param si Space Information
            /// @param state_cost_function Cost to go of individual state
            /// @param motion_cost_function Cost function between two states
            /// @param informed_sampler Informed sampler
            ///
            MyOptimizationObjective(const SpaceInformationPtr& si,
                                    const double& batch_size,
                                    const StateCostFxn& state_cost_function,
                                    const MotionCostFxn& motion_cost_function)
                : OptimizationObjective(si),
                  state_cost_function_(state_cost_function),
                  motion_cost_function_(motion_cost_function),
                  batch_size_(batch_size)
            { }


			///
			/// Return the cost of the state at this point
			///
			/// @param s State to get the cost for
			/// @return Cost of the state
			///
			virtual Cost stateCost(const State *s) const override;

			///
			/// Return the cost of moving from s1 to s2
			///
			/// @param s1 Start state
			/// @param s2 Goal state
			/// @return Cost of going from s1 to s2
			///
			virtual Cost motionCost(const State *s1, const State *s2) const override;

			///
			/// Function to get the informed sampler pointer
			///
			/// @param probDefn Problem definition pointer (OMPL)
			/// @param maxNumberCalls Maximum number of sampling calls
			/// @return Infromed sampler
			///
			virtual InformedSamplerPtr allocInformedStateSampler(const ProblemDefinitionPtr probDefn,
																 unsigned int maxNumberCalls) const override;
		};
	}
}
