#include <OmplWrappers/MyOptimizationObjective.h>

// Eigen
#include <Eigen/Dense>
using Eigen::VectorXd;

// OMPL
#include <ompl/base/State.h>
#include <ompl/base/Cost.h>
#include <ompl/base/samplers/InformedStateSampler.h>

// Internal Libraries
#include <Dimt/Params.h>
#include <Sampler/MonteCarloSamplers.h>

// stdlib
#include <exception>

//
// Return the cost of the state at this point
//
// @param s State to get the cost for
// @return Cost of the state
//
ompl::base::Cost ompl::base::MyOptimizationObjective::stateCost(const ompl::base::State *s) const
{
    if (sampler_ == nullptr or opt_ == nullptr)
    {
        throw std::runtime_error("An informed sampler with optimization objective "
                                 "must be provided or set.");
    }
    return opt_->stateCost(s);
}

//
// Return the cost of moving from s1 to s2
//
// @param s1 Start state
// @param s2 Goal state
// @return Cost of going from s1 to s2
//
ompl::base::Cost ompl::base::MyOptimizationObjective::motionCost(const ompl::base::State *s1,
                                                                 const ompl::base::State *s2) const
{
    if (sampler_ == nullptr)
    {
        throw std::runtime_error("An informed sampler with optimization objective "
                                 "must be provided or set.");
    }
    return opt_->motionCost(s1, s2);
}

//
// Function to get the informed sampler pointer
//
// @param probDefn Problem definition pointer (OMPL)
// @param maxNumberCalls Maximum number of sampling calls
// @return Infromed sampler
//
ompl::base::InformedSamplerPtr ompl::base::MyOptimizationObjective::allocInformedStateSampler(
    const ProblemDefinitionPtr probDefn, unsigned int maxNumberCalls) const
{
    if (sampler_ == nullptr)
    {
        throw std::runtime_error("An informed sampler with optimization objective "
                                 "must be provided or set.");
    }
    return sampler_;
}

namespace ompl
{
    namespace base
    {
        Cost GeometricObjective::stateCost(const State *s) const
        {
            return Cost((startState_ - get_eigen_vector(s)).norm() + (goalState_ - get_eigen_vector(s)).norm());
        }

        Cost GeometricObjective::motionCost(const State *s1, const State *s2) const
        {
            return Cost((get_eigen_vector(s1) - get_eigen_vector(s2)).norm());
        }

        Cost GeometricObjective::combineCosts(Cost c1, Cost c2) const
        {
            return Cost(c1.value() + c2.value());
        }
    }  // namespace base
}  // namespace ompl
