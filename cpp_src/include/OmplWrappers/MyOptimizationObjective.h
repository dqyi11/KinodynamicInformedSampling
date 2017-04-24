#pragma once

// stdlib
#include <iostream>
#include <memory>

// Eigen
#include <Eigen/Dense>
using Eigen::VectorXd;

// OMPL
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

// Internal packages
#include <Sampler/Sampler.h>
#include <Dimt/Params.h>
#include <Dimt/Dimt.h>
#include <OmplWrappers/OmplHelpers.h>

using MotionCostFxn = std::function<double(VectorXd, VectorXd)>;
using StateCostFxn = std::function<double(VectorXd)>;

///
/// Class that inherits from InformedSampler
///
namespace ompl
{
    namespace base
    {
        class MyOptimizationObjective : public OptimizationObjective
        {
        private:
            // Pointer to informed sampler
            MyInformedSamplerPtr sampler_ = nullptr;

            // Base optimization objective that sets the cost function for
            // both the informed sampler and anything using this optimization
            // objective
            OptimizationObjectivePtr opt_ = nullptr;

        public:
            ///
            /// Constructor
            ///
            /// @param si Space Information
            /// @param sampler Informed Sampler
            /// @param sampleBatchSize How many samples to get each time a new
            /// batch of samples is gotten
            /// @param stateCostFn Cost function for a single point in space
            /// @param motionCostFn Cost function between two states
            ///
            MyOptimizationObjective(const SpaceInformationPtr &si, const MyInformedSamplerPtr sampler)
              : OptimizationObjective(si), sampler_(sampler), opt_(sampler_->problem()->getOptimizationObjective())
            {
            }

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

            ///
            /// Function to provide and informed sampler
            ///
            /// @param sampler
            ///
            virtual void setInformedStateSampler(const MyInformedSamplerPtr &sampler)
            {
                sampler_ = sampler;
            }
        };

        class GeometricObjective : public OptimizationObjective
        {
        private:
            const Eigen::VectorXd startState_;

            const Eigen::VectorXd goalState_;

        public:
            ///
            /// Constructor
            ///
            /// @param si Space Information
            /// @param startState Start state of the problem
            /// @param goalState Goal state of the problem
            ///
            GeometricObjective(const SpaceInformationPtr &si, const Eigen::VectorXd &startState,
                               const Eigen::VectorXd &goalState)
              : OptimizationObjective(si), startState_(startState), goalState_(goalState)
            {
            }

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
            /// Combines cost
            ///
            /// @param c1 cost one
            /// @param c2 cost two
            /// @return Combined cost (c1 + c2)
            ///
            virtual Cost combineCosts(Cost c1, Cost c2) const override;
        };

        template <int dof>
        class DimtObjective : public OptimizationObjective
        {
        private:
            const Eigen::VectorXd startState_;

            const Eigen::VectorXd goalState_;

            const Dimt di_;

            Eigen::VectorXd get_eigen_vector(const ompl::base::State *s) const
            {
                double *val = static_cast<const ompl::base::RealVectorStateSpace::StateType *>(s)->values;

                Eigen::VectorXd v(param.dimensions);

                for (uint i = 0; i < param.dimensions; i++)
                {
                    v[i] = val[i];
                }

                return v;
            }

        public:
            ///
            /// Constructor
            ///
            /// @param si Space Information
            /// @param startState Start state of the problem
            /// @param goalState Goal state of the problem
            /// @param di Double Integrator model
            ///
            DimtObjective<dof>(const SpaceInformationPtr &si, const Eigen::VectorXd &startState,
                               const Eigen::VectorXd &goalState, const Dimt di)
              : OptimizationObjective(si), startState_(startState), goalState_(goalState), di_(di)
            {
            }

            ///
            /// Return the cost of the state at this point
            ///
            /// @param s State to get the cost for
            /// @return Cost of the state
            ///
            virtual Cost stateCost(const State *s) const override
            {
                return Cost(di_.get_min_time(startState_, get_eigen_vector(s)) +
                            di_.get_min_time(get_eigen_vector(s), goalState_));
            }

            ///
            /// Return the cost of moving from s1 to s2
            ///
            /// @param s1 Start state
            /// @param s2 Goal state
            /// @return Cost of going from s1 to s2
            ///
            virtual Cost motionCost(const State *s1, const State *s2) const override
            {
                return Cost(di_.get_min_time(get_eigen_vector(s1), get_eigen_vector(s2)));
            }

            ///
            /// Combines cost
            ///
            /// @param c1 cost one
            /// @param c2 cost two
            /// @return Combined cost (c1 + c2)
            ///
            virtual Cost combineCosts(Cost c1, Cost c2) const override
            {
                return Cost(std::max(c1.value(), c2.value()));
            }
        };
    }
}
