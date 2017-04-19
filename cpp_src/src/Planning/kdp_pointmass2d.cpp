#include <iostream>
#include <fstream>
#include "OmplWrappers/DimtStateSpace.h"
#include "Dimt/DoubleIntegrator.h"
#include "Dimt/Dimt.h"
#include "Dimt/Params.h"
#include "ompl/geometric/SimpleSetup.h"
#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include "ompl/geometric/planners/rrt/RRTstar.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

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
        for (int i = 0; i < param.dimensions; i = i + 2)
        {
            if (state_rv->values[i] < -1 || state_rv->values[i] > 1)
                return true;
        }
        return false;
    }
};

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
    // construct the state space we are planning in
    ompl::base::StateSpacePtr space(new ompl::base::DimtStateSpace(dimt, double_integrator, param.dimensions));
    ob::RealVectorBounds bounds(param.dimensions);
    bounds.setLow(-10);
    bounds.setHigh(10);
    space->as<ompl::base::DimtStateSpace>()->setBounds(bounds);
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
    si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si)));
    si->setStateValidityCheckingResolution(0.03);  // 3%
    si->setup();
    // Set custom start and goal
    ompl::base::State *start_s = space->allocState();
    ompl::base::State *goal_s = space->allocState();
    for (int i = 0; i < param.dimensions; i++)
    {
        start_s->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = -5;
        goal_s->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = 5;
    }
    ob::ScopedState<ompl::base::RealVectorStateSpace> start(space, start_s);
    ob::ScopedState<ompl::base::RealVectorStateSpace> goal(space, goal_s);
    // Set random start and goal
    // ob::ScopedState<> start(space);
    // start.random();
    // ob::ScopedState<> goal(space);
    // goal.random();
    // Setup Problem Definition
    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
    pdef->setStartAndGoalStates(start, goal);
    // Construct Planner
    // ob::PlannerPtr planner(new ompl::geometric::RRTConnect(si));
    ob::PlannerPtr planner(new og::RRTstar(si));
    // Set the problem instance for our planner to solve
    planner->setProblemDefinition(pdef);
    planner->setup();

    // Run planner
    ob::PlannerStatus solved = planner->solve(5.0);
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
