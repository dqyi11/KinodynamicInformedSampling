#include <iostream>
#include <fstream>
#include "DimtStateSpace.h"
#include "Dimt/DoubleIntegrator.h"
#include "Dimt/Dimt.h"
#include "Dimt/Params.h"
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid(const ob::State *state)
{
  // std::cout << "before" << std::endl;
  const double* val = static_cast<const ob::RealVectorStateSpace::StateType*>(state)->values;
  // std::cout << (*static_cast<const ob::RealVectorStateSpace::StateType*>(state))[0];
  // std::cout << "after" << std::endl;
  double x = (double)val[0]; 
  double y = (double)val[1];
  // std::cout << "after" << std::endl;
  // if (x>-1 && x<1 && y>-4 && y<4)
  //   return false;
  // else
	return true;
}

void planWithSimpleSetup(void)
{
  // Initializations
  double a_max = 1;
  int dimensions = 2*1;
  int dof = dimensions/2;
  Dimt dimt(a_max);
  DoubleIntegrator<PARAM_DOF>::Vector maxAccelerations, maxVelocities;
  for (unsigned int i = 0; i < dof; ++i)
  {
    maxVelocities[i] = 10;
    maxAccelerations[i] = a_max;
  }
  DoubleIntegrator<PARAM_DOF> double_integrator(maxAccelerations, maxVelocities);
  // construct the state space we are planning in
  ompl::base::StateSpacePtr space(new ompl::base::DimtStateSpace(dimt, double_integrator, 2));
	ob::RealVectorBounds bounds(2);
	bounds.setLow(-10);
	bounds.setHigh(10);
  space->as<ompl::base::DimtStateSpace>()->setBounds(bounds);
	ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
  si->setStateValidityChecker(std::bind(&isStateValid, std::placeholders::_1));
  si->setStateValidityCheckingResolution(0.03); // 3%
  si->setup();
// Set custom start and goal 
  ompl::base::State *start_s = space->allocState();
  start_s->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = -5;
  start_s->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = -5;
  ob::ScopedState<ompl::base::RealVectorStateSpace> start(space, start_s);
  ompl::base::State *goal_s = space->allocState();
  goal_s->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = 5;
  goal_s->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = 5;
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
    // get the goal representation from the problem definition (not the same as the goal state)
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