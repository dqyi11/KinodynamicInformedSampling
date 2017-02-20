#include <OmplWrappers/MyOptimizationObjective.h>

// Eigen
#include <Eigen/Dense>
using Eigen::VectorXd;

// OMPL
#include <ompl/base/State.h>
#include <ompl/base/Cost.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/samplers/InformedStateSampler.h>

// Internal Libraries
#include <OmplWrappers/OmplSamplers.h>
#include <Dimt/Params.h>

///
/// Function to convert a State to a VectorXd
///
/// @param s Ompl State
/// @return Eigen VectorXd
///
VectorXd get_eigen_vector(const ompl::base::State* s)
{
	auto state = s->as<ompl::base::RealVectorStateSpace::StateType>()->values;

	std::vector<double> state_vector(state, state + sizeof state / sizeof state[0]);

	// std::cout << "Created vector" << std::endl;

	// auto n = VectorXd::Map(state_vector.data(), state_vector.size());

	// std::cout << "Remapped vector" << std::endl;

	// return n;

	// std::cout << "State Vector Size: " << state_vector.size() << std::endl;

	return VectorXd::Map(state_vector.data(), state_vector.size());
}

///
/// MyOptimizationObjective
///

///
/// Return the cost of the state at this point
///
/// @param s State to get the cost for
/// @return Cost of the state
///
ompl::base::Cost ompl::base::MyOptimizationObjective::stateCost(const ompl::base::State *s) const
{
	return ompl::base::Cost(state_cost_function_(get_eigen_vector(s)));
}

///
/// Return the cost of moving from s1 to s2
///
/// @param s1 Start state
/// @param s2 Goal state
/// @return Cost of going from s1 to s2
///
ompl::base::Cost ompl::base::MyOptimizationObjective::motionCost(const ompl::base::State *s1,
										 			             const ompl::base::State *s2) const
{
	return ompl::base::Cost(motion_cost_function_(get_eigen_vector(s1), get_eigen_vector(s2)));
}

///
/// Function to get the informed sampler pointer
///
/// @param probDefn Problem definition pointer (OMPL)
/// @param maxNumberCalls Maximum number of sampling calls
/// @return Infromed sampler
///
ompl::base::InformedSamplerPtr ompl::base::MyOptimizationObjective::allocInformedStateSampler(
	const ProblemDefinitionPtr probDefn, unsigned int maxNumberCalls) const
{
	return ompl::base::InformedSamplerPtr(
		new ompl::base::MyInformedSampler(probDefn,
										  maxNumberCalls,
										  sampler_ptr_,
										  batch_size_));
}
