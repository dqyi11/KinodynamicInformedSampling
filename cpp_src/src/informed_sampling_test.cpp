// stdlib
#include <iostream>
#include <vector>
#include <tuple>
#include <memory>

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
#include <ProblemDefinition/ProblemDefinition.h>
#include <Dimt/Dimt.h>
#include <Sampler/RejectionSampler.h>
#include <Sampler/MonteCarloSamplers.h>
#include <OmplWrappers/OmplSamplers.h>
#include <OmplWrappers/MyOptimizationObjective.h>

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

ProblemDefinition create_prob_definition(const VectorXd& start_state,
										 const VectorXd& goal_state,
										 const int& dimension,
										 const double& minval,
										 const double& maxval,
										 const double& level_set,
										 const CostFxn& costfxn)
{
	VectorXd state_min(dimension);
	state_min << VectorXd::Constant(dimension, minval);

	VectorXd state_max(dimension);
	state_max << VectorXd::Constant(dimension, maxval);

	return ProblemDefinition(start_state, goal_state, state_min, 
							 state_max, level_set, costfxn);
}

ob::OptimizationObjectivePtr get_geom_opt_obj(const ob::SpaceInformationPtr& si,
											  const VectorXd& start_state,
											  const VectorXd& goal_state,
											  const std::shared_ptr<Sampler> sampler,
											  const double& batch_size)
{
	return 
	ob::OptimizationObjectivePtr(new ompl::base::MyOptimizationObjective(si, sampler, batch_size,
		[start_state, goal_state](const VectorXd& state)
		{
			return (start_state - state).norm() + (goal_state - state).norm();
		},
		[](const VectorXd& s1, const VectorXd& s2)
		{
			return (s2 - s1).norm();
		}));
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
        return this->clearance(state) > 0.0;
    }
    // Returns the distance from the given state's position to the
    // boundary of the circular obstacle.
    double clearance(const ob::State* state) const
    {
        // We know we're working with a RealVectorStateSpace in this
        // example, so we downcast state into the specific type.
        const ob::RealVectorStateSpace::StateType* state2D =
            state->as<ob::RealVectorStateSpace::StateType>();
        // Extract the robot's (x,y) position from its state
        double x = state2D->values[0];
        double y = state2D->values[1];
        // Distance formula between two points, offset by the circle's
        // radius
        return sqrt((x-0.5)*(x-0.5) + (y-0.5)*(y-0.5)) - 0.25;
    }
};

const bool verbose = true;
int main(int argc, char const *argv[])
{
	if(verbose) std::cout << "Got in" << std::endl;

	///
	/// Set up the problem definition for geometric costs
	///
	const int dimension = 2; const double minval = -25; const double maxval = 25;
	VectorXd start_state(dimension);
	VectorXd goal_state(dimension);
	start_state << 0.0, 0.0;
	goal_state << 1.0, 1.0;
	// std::tie(start_state, goal_state) = get_random_start_and_goal(dimension,
	// 															  minval,
	// 															  maxval);

	std::cout << "Start: " << std::endl << start_state << std::endl;
	std::cout << "Goal: " << std::endl << goal_state << std::endl;

	const double level_set = (goal_state - start_state).norm();
	auto prob = create_prob_definition(start_state, goal_state, dimension, minval, maxval, level_set,
		[start_state, goal_state](const VectorXd& state)
		{
			return (start_state - state).norm() + (goal_state - state).norm();
		});

  	std::cout << "Level set: " << level_set << std::endl;

	///
	/// Set up the OMPL boilerplate space code
	///

	// Construct the robot state space in which we're planning. We're
	// planning in [0,1]x[0,1], a subset of R^2.
	ob::StateSpacePtr space(new ob::RealVectorStateSpace(2));

	// Set the bounds of space to be in [0,1].
	space->as<ob::RealVectorStateSpace>()->setBounds(0.0, 1.0);

	if(verbose) std::cout << "Created the space." << std::endl;

	// Construct a space information instance for this state space
	ob::SpaceInformationPtr si(new ob::SpaceInformation(space));

	// Set the object used to check which states in the space are valid
	si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si)));
	si->setup();

	if(verbose) std::cout << "Set up the space information" << std::endl;

	// Set our robot's starting state to be the bottom-left corner of
	// the environment, or (0,0).
	ob::ScopedState<> start(space);
	start->as<ob::RealVectorStateSpace::StateType>()->values[0] = 0.0;
	start->as<ob::RealVectorStateSpace::StateType>()->values[1] = 0.0;

	// Set our robot's goal state to be the top-right corner of the
	// environment, or (1,1).
	ob::ScopedState<> goal(space);
	goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = 1.0;
	goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = 1.0;

	if(verbose) std::cout << "Got the start and goal" << std::endl;

	// Create a problem instance
	ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

	// Set the start and goal states
	pdef->setStartAndGoalStates(start, goal);

	if(verbose) std::cout << "Set the start and goal" << std::endl;

	// Set up the optimization objective
	double sigma = 1; int max_steps = 20; double alpha = 1.0; double batch_size = 20;
	auto mcmc_s = std::make_shared<MCMCSampler>(prob, alpha, sigma, max_steps);
	auto opt = get_geom_opt_obj(si, start_state, goal_state, mcmc_s, batch_size);
	opt->setCostThreshold(ob::Cost(1.51));

	if(verbose) std::cout << "Set up the optimizing objective" << std::endl;

	pdef->setOptimizationObjective(opt);

	if(verbose) std::cout << "Set up the problem definition" << std::endl;

	auto sampler = ob::InformedSamplerPtr(new ob::MyInformedSampler(pdef, 1000, mcmc_s, 20));
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
	ob::PlannerStatus solved = optimizingPlanner->solve(1.0);

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