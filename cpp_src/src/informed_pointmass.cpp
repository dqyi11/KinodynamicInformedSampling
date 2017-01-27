#include <iostream>
#include <fstream>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>

#include <Sampler/Sampler.h>
#include <ProblemDefinition/ProblemDefinition.h>
#include <Sampler/RejectionSampler.h>
#include <Sampler/MonteCarloSamplers.h>
#include <OmplWrappers/OmplSamplers.h>
#include <OmplWrappers/MyOptimizationObjective.h>

#include "DimtStateSpace.h"
#include "Dimt/DoubleIntegrator.h"
#include "Dimt/Dimt.h"
#include "Dimt/Params.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr& si) :
        ob::StateValidityChecker(si) {}
    // rectangle obstacle
    bool isValid(const ob::State* state) const
    {
        const ob::RealVectorStateSpace::StateType* state_rv =
            state->as<ob::RealVectorStateSpace::StateType>();
        for (int i=0; i<param.dimensions; i=i+2){
          if(state_rv->values[i]<-1 || state_rv->values[i]>1)
            return true;
        }
        return false;
    }
};

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
  // Intiatilizations for sampler
  const int dimension = param.dimensions; const double minval = -10; const double maxval = 10;
  VectorXd start_state(dimension);
  VectorXd goal_state(dimension);
  // Construct the state space we are planning in
  ompl::base::StateSpacePtr space(new ompl::base::DimtStateSpace(dimt, double_integrator, param.dimensions));
  ob::RealVectorBounds bounds(param.dimensions);
  bounds.setLow(-10);
  bounds.setHigh(10);
  space->as<ompl::base::DimtStateSpace>()->setBounds(bounds);
  ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
  si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si)));
  si->setStateValidityCheckingResolution(0.03); // 3%
  si->setup();
  // Set custom start and goal 
  ompl::base::State *start_s = space->allocState();
  ompl::base::State *goal_s = space->allocState();
  for (int i=0; i<param.dimensions; i++)
  {
    start_s->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = -5;
    goal_s->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = 5;  
    start_state[i] = -5;
    goal_state[i] = 5;
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
  // Construct Sampler and Planner
  double sigma = 1; int max_steps = 20; double alpha = 1.0; double batch_size = 20;
  const double level_set = 100;
  auto prob = create_prob_definition(start_state, goal_state, dimension, minval, maxval, level_set,
    [start_state, goal_state](const VectorXd& state)
    {
      return (start_state - state).norm() + (goal_state - state).norm();
    });
  auto mcmc_s = std::make_shared<MCMCSampler>(prob, alpha, sigma, max_steps);
  auto opt = get_geom_opt_obj(si, start_state, goal_state, mcmc_s, batch_size);
  opt->setCostThreshold(ob::Cost(1.51));
  pdef->setOptimizationObjective(opt);
  auto sampler = ob::InformedSamplerPtr(new ob::MyInformedSampler(pdef, 1000, mcmc_s, 20));
  // ob::PlannerPtr planner(new ompl::geometric::RRTConnect(si));
  // ob::PlannerPtr planner(new og::RRTstar(si));
  ob::PlannerPtr planner(new og::InformedRRTstar(si));
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