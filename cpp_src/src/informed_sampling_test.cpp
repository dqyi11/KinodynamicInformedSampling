// NOTE: Set "constexpr Params param = param_1dof;" in Params.h for 2d Narrow Corridor

// stdlib
#include <iostream>
#include <vector>
#include <tuple>
#include <memory>
#include <limits>

// Eigen
#include <Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::VectorXd;

// OMPL
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
namespace ob = ompl::base;
namespace og = ompl::geometric;

// Internal Libraries
#include <Sampler/Sampler.h>
#include <Dimt/Dimt.h>
#include <Sampler/RejectionSampler.h>
#include <Sampler/MonteCarloSamplers.h>
#include <OmplWrappers/OmplSamplers.h>
#include <OmplWrappers/MyOptimizationObjective.h>
#include <OmplWrappers/OmplHelpers.h>
#include "Dimt/Params.h"

///
/// Helper functions
///
std::tuple<VectorXd, VectorXd> get_random_start_and_goal(const int& dimension,
														 const double& minval,
														 const double& maxval)
{
	VectorXd start_state(dimension);
	VectorXd goal_state(dimension);

	// Randomly generate the start and goal states
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<double> dis(minval, maxval);
	for(int i = 0; i < dimension; i++)
	{
		start_state(i) = dis(gen);
		goal_state(i) = dis(gen);
	}

	return std::make_tuple(start_state, goal_state);
}

///
/// OMPL boilerplate code
/// Some from tutorial: http://ompl.kavrakilab.org/optimalPlanningTutorial.html
///

// Our collision checker. For this demo, our robot's state space
// lies in [0,1]x[0,1], with a circular obstacle of radius 0.25
// centered at (0.5,0.5). Any states lying in this circular region are
// considered "in collision".
class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr& si) :
        ob::StateValidityChecker(si) {}
    // Returns whether the given state's position overlaps the
    // circular obstacle
    bool isValid(const ob::State* state) const
    {
// Narrow Corridor problem
      const ob::RealVectorStateSpace::StateType* state_rv =
          state->as<ob::RealVectorStateSpace::StateType>();
      bool isInCollision = false;
      for (int i=0; i<param.dimensions; i++)
      {
      	if (i==0)
      		if(state_rv->values[i]<-0.3 ||  state_rv->values[i]>0.3)
      			return true;
      	else
      		if(state_rv->values[i] < -5 ||
      			(state_rv->values[i] > -0.1 && state_rv->values[i] < 0.1) ||
      			 state_rv->values[i] > 5)
      			return true;
      }
      return false;
// Circular Obstalce at Origin
      // return this->clearance(state) > 0.0;
    }
    // Returns the distance from the given state's position to the
    // boundary of the circular obstacle.
    double clearance(const ob::State* state) const
    {
      // We know we're working with a RealVectorStateSpace in this
      // example, so we downcast state into the specific type.
      const ob::RealVectorStateSpace::StateType* state_rv =
          state->as<ob::RealVectorStateSpace::StateType>();
      double sq_dist = 0; // squared distance from origin
      for (int i=0; i<param.dimensions; i++)
      	sq_dist += state_rv->values[i] * state_rv->values[i];
      return sq_dist - 3*3;
   }
};

const bool verbose = true;
int main(int argc, char const *argv[])
{
	if(verbose) std::cout << "Got in" << std::endl;

	///
	/// Set up the problem definition for geometric costs
	///
	const int dimension = param.dimensions; const double minval = -10; const double maxval = 10;
	ob::StateSpacePtr space(new ob::RealVectorStateSpace(param.dimensions));
	ompl::base::State *start_s = space->allocState();
	ompl::base::State *goal_s = space->allocState();
	VectorXd start_state(dimension);
	VectorXd goal_state(dimension);
	// Start and Goal Position For Narrow Corridor problem
	for (int i=0; i<param.dimensions; i++)
	{
	  start_s->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = 0;
	  goal_s->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = 0;
	  start_state[i] = 0;
	  goal_state[i] = 0;
	}
	start_s->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = -1.5;
  goal_s->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = 1.5;
  start_state[0] = -1.5;
  goal_state[0] = 1.5;
	// std::tie(start_state, goal_state) = get_random_start_and_goal(dimension,
	// 															  minval,
	// 															  maxval);

	std::cout << "Start: " << std::endl << start_state << std::endl;
	std::cout << "Goal: " << std::endl << goal_state << std::endl;

	const double level_set = std::numeric_limits<double>::infinity();

  	std::cout << "Level set: " << level_set << std::endl;

	///
	/// Set up the OMPL boilerplate space code
	///
	space->as<ob::RealVectorStateSpace>()->setBounds(minval, maxval);

	if(verbose) std::cout << "Created the space." << std::endl;

	// Construct a space information instance for this state space
	ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
	// Set the object used to check which states in the space are valid
	si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si)));
	si->setup();
	if(verbose) std::cout << "Set up the space information" << std::endl;

	// Create a problem instance
	ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

	// Set the start and goal states
	pdef->setStartAndGoalStates(start_s, goal_s);
	if(verbose) std::cout << "Set the start and goal" << std::endl;

	// Set up the optimization objective
	double sigma = 1; int max_steps = 20; double alpha = 1.0; double batch_size = 20;
	auto current_sampler = std::make_shared<MCMCSampler>(si, pdef, level_set, alpha, sigma, max_steps);
	// auto current_sampler = std::make_shared<GeometricHierarchicalRejectionSampler>(prob);
	auto opt = get_geom_opt_obj(si, start_state, goal_state, current_sampler, batch_size);
	opt->setCostThreshold(ob::Cost(1.51));

	if(verbose) std::cout << "Set up the optimizing objective" << std::endl;

	pdef->setOptimizationObjective(opt);

	if(verbose) std::cout << "Set up the problem definition" << std::endl;

	auto sampler = ob::InformedSamplerPtr(new ob::MyInformedSampler(pdef, 1000, current_sampler, 20));
	std::cout << "Running RRT* with mcmc Sampler..." << std::endl;

	if(verbose) std::cout << "Set up the sampler" << std::endl;

	// Construct our optimizing planner using the RRTstar algorithm.
	ob::PlannerPtr optimizingPlanner(new og::InformedRRTstar(si));

	// Set the problem instance for our planner to solve
	optimizingPlanner->setProblemDefinition(pdef);
	optimizingPlanner->setup();

	if(verbose) std::cout << "Set up the planner" << std::endl;

	// attempt to solve the planning problem within one second of
	// planning time
	ob::PlannerStatus solved = optimizingPlanner->solve(200);

	if(verbose) std::cout << "Solved!" << std::endl;

	if(solved)
	{
		std::cout << "Found solution:" << std::endl;
		ob::PathPtr path = pdef->getSolutionPath();
		path->print(std::cout);
	}
	else
	{
		std::cout << "Solution not found." << std::endl;
	}

	return 0;
}
