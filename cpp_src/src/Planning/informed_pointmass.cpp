#include <iostream>
#include <fstream>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>

#include "Sampler/Sampler.h"
#include "Sampler/RejectionSampler.h"
#include "Sampler/MonteCarloSamplers.h"
#include "Sampler/HitAndRun.h"
#include "OmplWrappers/MyOptimizationObjective.h"
#include "OmplWrappers/MyInformedRRTstar.h"
#include "OmplWrappers/OmplHelpers.h"
#include "OmplWrappers/DimtStateSpace.h"
#include "Dimt/DoubleIntegrator.h"
#include "Dimt/Dimt.h"
#include "Dimt/Params.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

#include <limits>

class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr &si) : ob::StateValidityChecker(si)
    {
    }
    // rectangle obstacle
    bool isValid(const ob::State *state) const
    {
        const ob::RealVectorStateSpace::StateType *state_rv = state->as<ob::RealVectorStateSpace::StateType>();
        // for (int i=0; i<param.dimensions; i=i+2){
        //   if(state_rv->values[i]<-1 || state_rv->values[i]>1)
        //     return true;
        // }
        // return false;
        // Narrow Corridor problem
        for (int i = 0; i < param.dimensions; i = i + 2)
        {
            if (i == 0)
                if (state_rv->values[i] < -0.5 || state_rv->values[i] > 0.5)
                    return true;
                else if (state_rv->values[i] < -5 || (state_rv->values[i] > -0.05 && state_rv->values[i] < 0.05) ||
                         state_rv->values[i] > 5)
                    return true;
        }
        return false;
    }
};

bool MAIN_VERBOSE = true;

void planWithSimpleSetup(void)
{
    // Initializations
    Dimt dimt(param.a_max);
    DoubleIntegrator<param.dof>::Vector maxAccelerations, maxVelocities;
    for (unsigned int i = 0; i < param.dof; ++i)
    {
        maxVelocities[i] = 10;
        maxAccelerations[i] = param.a_max;
    }
    DoubleIntegrator<param.dof> double_integrator(maxAccelerations, maxVelocities);

    if (MAIN_VERBOSE)
        std::cout << "Created the double integrator model!" << std::endl;

    // Intiatilizations for sampler
    const int dimension = param.dimensions;
    const double minval = -10;
    const double maxval = 10;
    VectorXd start_state(dimension);
    VectorXd goal_state(dimension);
    VectorXd int_state(dimension);

    if (MAIN_VERBOSE)
        std::cout << "Got the start and goal states!" << std::endl;

    // Construct the state space we are planning in
    ompl::base::StateSpacePtr space(new ompl::base::DimtStateSpace(dimt, double_integrator, param.dimensions));
    ob::RealVectorBounds bounds(param.dimensions);
    bounds.setLow(-10);
    bounds.setHigh(10);
    space->as<ompl::base::DimtStateSpace>()->setBounds(bounds);
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
    si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si)));
    si->setStateValidityCheckingResolution(0.01);  // 3%
    si->setup();

    if (MAIN_VERBOSE)
        std::cout << "Set up the state space!" << std::endl;

    // Set custom start and goal
    ompl::base::State *start_s = space->allocState();
    ompl::base::State *goal_s = space->allocState();
    for (int i = 0; i < param.dimensions; i++)
    {
        if (i % 2 == 0)  // position
        {
            start_state[i] = 0;
            start_s->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = start_state[i];
            goal_state[i] = 0;
            goal_s->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = goal_state[i];
            int_state[i] = 0;
        }
        else  // velocity
        {
            start_state[i] = 2;
            start_s->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = start_state[i];
            goal_state[i] = 2;
            goal_s->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = goal_state[i];
            int_state[i] = 2;
        }
    }
    start_state[0] = -1.5;
    start_s->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = start_state[0];
    goal_state[0] = 1.5;
    goal_s->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = goal_state[0];
    start_state[2] = 1;
    start_s->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = start_state[2];
    goal_state[2] = 1;
    goal_s->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = goal_state[2];

    std::cout << "MinTime b/w start and goal = "
              << double_integrator.getMinTime(start_state, int_state) +
                     double_integrator.getMinTime(int_state, goal_state) << std::endl;
    std::cout << "Start_State: " << start_state << " Goal_State: " << goal_state << std::endl;
    ob::ScopedState<ompl::base::RealVectorStateSpace> start(space, start_s);
    ob::ScopedState<ompl::base::RealVectorStateSpace> goal(space, goal_s);

    if (MAIN_VERBOSE)
        std::cout << "Got the vector start and goal state space ompl!" << std::endl;
    if (MAIN_VERBOSE)
        std::cout << start_s << std::endl
                  << goal_s << std::endl;

    // Get a base problem definition that has the optimization objective with the
    // space
    // This probably should be changed
    ob::ProblemDefinitionPtr base_pdef(new ob::ProblemDefinition(si));
    base_pdef->setStartAndGoalStates(start, goal);

    const ompl::base::OptimizationObjectivePtr base_opt = ompl::base::OptimizationObjectivePtr(
        new ompl::base::DimtObjective<param.dof>(si, start_state, goal_state, double_integrator));
    base_pdef->setOptimizationObjective(base_opt);

    if (MAIN_VERBOSE)
        std::cout << "Set up the OMPL problem definition!" << std::endl;

    // Construct Sampler with the base pdef and base optimization objective
    double sigma = 1;
    int max_steps = 100;
    double alpha = 1.0;
    double batch_size = 1000;
    const double level_set = std::numeric_limits<double>::infinity();
    const auto sampler = ompl::base::MyInformedSamplerPtr(
        new ompl::base::MCMCSampler(si, base_pdef, level_set, 1000, batch_size, alpha, sigma, max_steps));

    // Set up the final problem with the full optimization objective
    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
    pdef->setStartAndGoalStates(start, goal);

    const ompl::base::OptimizationObjectivePtr opt =
        ompl::base::OptimizationObjectivePtr(new ompl::base::MyOptimizationObjective(si, sampler));
    if (MAIN_VERBOSE)
        std::cout << "Set up the MCMC sampler!" << std::endl;

    opt->setCostThreshold(ob::Cost(1.51));
    pdef->setOptimizationObjective(opt);

    if (MAIN_VERBOSE)
        std::cout << "Created the optimization objection!" << std::endl;

    if (MAIN_VERBOSE)
        std::cout << "Created the informed ompl sampler!" << std::endl;

    // ob::PlannerPtr planner(new ompl::geometric::RRTConnect(si));
    // ob::PlannerPtr planner(new og::RRTstar(si));
    // ob::PlannerPtr planner(new og::InformedRRTstar(si));
    ob::PlannerPtr planner(new MyInformedRRTstar(si));

    // Set the problem instance for our planner to solve
    planner->setProblemDefinition(pdef);
    planner->setup();

    if (MAIN_VERBOSE)
        std::cout << "Set up Informed RRT* planner!" << std::endl;

    // Run planner
    ob::PlannerStatus solved = planner->solve(15.0);

    if (MAIN_VERBOSE)
        std::cout << "Planner solved!" << std::endl;

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // get the goal representation from the problem definition (not the same as
        // the goal state)
        // and inquire about the found path
        ob::PathPtr path = pdef->getSolutionPath();
        // print the path to screen
        path->print(std::cout);
        // Print to File
        // std::ofstream myfile;
        // myfile.open("geometric_pointmass2d.txt");
        // path->printAsMatrix(std::cout);
        // myfile.close();
    }
}

int main()
{
    planWithSimpleSetup();
}