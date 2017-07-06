#include <iostream>
#include <fstream>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>

#include "Sampler/Sampler.h"
#include "Sampler/RejectionSampler.h"
#include "Sampler/MonteCarloSampler.h"
#include "Sampler/HitAndRunSampler.h"
#include "OmplWrappers/MyOptimizationObjective.h"
#include "OmplWrappers/MyInformedRRTstar.h"
#include "OmplWrappers/OmplHelpers.h"
#include "OmplWrappers/DimtStateSpace.h"
#include "Dimt/Params.h"
#include "Dimt/DoubleIntegratorMinimumTime.h"
#include "create_obstacles.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

#include <limits>

typedef enum{ HNR, RS, HRS, HMC } SamplerType;

// Construct Sampler with the base pdef and base optimization objective
double sigma = 1;
int max_steps = 20;
double alpha = 0.5;
double max_call_num = 100;
double batch_size = 100;
double epsilon = 0.1;
double L = 5;
int num_trials = 5;
const double level_set = std::numeric_limits<double>::infinity();

bool MAIN_VERBOSE = true;

ompl::base::MyInformedRRTstarPtr createPlanner(std::string caseName, int index,
                                     SamplerType type, ompl::base::SpaceInformationPtr si,
                                     ompl::base::ProblemDefinitionPtr base_pdef, DIMTPtr dimt,
                                     const ompl::base::ScopedState<ompl::base::RealVectorStateSpace>& start,
                                     const ompl::base::ScopedState<ompl::base::RealVectorStateSpace>& goal,
                                     double singleSampleLimit)
{
    ompl::base::MyInformedSamplerPtr sampler;
    std::string samplerName;
    switch(type)
    {
        case HNR:
            std::cout << "Planning using Hit&Run Sampler" << std::endl;
            sampler = std::make_shared<ompl::base::HitAndRunSampler>(si, base_pdef, level_set, max_call_num, batch_size, num_trials);
            samplerName = "HNR";
            break;

        case RS:
            std::cout << "Planning using Rejection Sampler" << std::endl;
            sampler = std::make_shared<ompl::base::RejectionSampler>(si, base_pdef, level_set, max_call_num, batch_size);
            samplerName = "RS";
            break;

        case HRS:
            std::cout << "Planning using HRS Sampler" << std::endl;
            sampler = std::make_shared<ompl::base::DimtHierarchicalRejectionSampler>(si, base_pdef, dimt, level_set, max_call_num, batch_size);
            samplerName = "HRS";
            break;

        case HMC:
            std::cout << "Planning using HMC Sampler" << std::endl;
            sampler = std::make_shared<ompl::base::HMCSampler>(si, base_pdef, level_set, max_call_num, batch_size, alpha, L, epsilon, sigma, max_steps);
            samplerName = "HMC";
            break;
    }

    sampler->setSingleSampleTimelimit(singleSampleLimit);

    // Set up the final problem with the full optimization objective
    ob::ProblemDefinitionPtr pdef = std::make_shared<ob::ProblemDefinition>(si);
    pdef->setStartAndGoalStates(start, goal);

    ompl::base::OptimizationObjectivePtr opt = std::make_shared<ompl::base::MyOptimizationObjective>(si, sampler);

    pdef->setOptimizationObjective(opt);

    ob::MyInformedRRTstarPtr planner = std::make_shared<ob::MyInformedRRTstar>(si);

    // Set the problem instance for our planner to solve
    planner->setProblemDefinition(pdef);
    planner->setup();
    planner->initLogFile(caseName, samplerName, index);

    return planner;
}

void planWithSimpleSetup(void)
{
    // Initializations
    std::vector<double> maxVelocities(param.dof, param.v_max);
    std::vector<double> maxAccelerations(param.dof, param.a_max);
    DIMTPtr dimt = std::make_shared<DIMT>( maxAccelerations, maxVelocities );

    if (MAIN_VERBOSE)
        std::cout << "Created the double integrator model!" << std::endl;

    // Intiatilizations for sampler
    const int dimension = param.dimensions;
    VectorXd start_state(dimension);
    VectorXd goal_state(dimension);

    if (MAIN_VERBOSE)
        std::cout << "Got the start and goal states!" << std::endl;

    // Construct the state space we are planning in
    ob::StateSpacePtr space = std::make_shared< ob::DimtStateSpace >(dimt);
    ob::RealVectorBounds bounds(param.dimensions);
    for(uint i=0;i<param.dimensions;i++)
    {
        if(i%2==0)
        {
            bounds.setLow(-param.s_max);
            bounds.setHigh(param.s_max);
        }
        else
        {
            bounds.setLow(-param.v_max);
            bounds.setHigh(param.v_max);
        }
    }
    space->as<ompl::base::DimtStateSpace>()->setBounds(bounds);
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
    
    ob::StateValidityCheckerPtr svc = createStateValidityChecker(si, "obstacles.json");
    si->setStateValidityChecker(svc);
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
            start_state[i] = -4.;
            start_s->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = start_state[i];
            goal_state[i] = 4.;
            goal_s->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = goal_state[i];
        }
        else  // velocity
        {
            start_state[i] = 2.;
            start_s->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = start_state[i];
            goal_state[i] = 2.;
            goal_s->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = goal_state[i];
        }
    }

    std::cout << "Start_State: " << start_state << " Goal_State: " << goal_state << std::endl;
    ob::ScopedState<ompl::base::RealVectorStateSpace> start(space, start_s);
    ob::ScopedState<ompl::base::RealVectorStateSpace> goal(space, goal_s);

    if (MAIN_VERBOSE)
        std::cout << "Got the vector start and goal state space ompl!" << std::endl;

    // Get a base problem definition that has the optimization objective with the
    // space
    // This probably should be changed
    ob::ProblemDefinitionPtr base_pdef = std::make_shared<ob::ProblemDefinition>(si);
    base_pdef->setStartAndGoalStates(start, goal);

    const ompl::base::OptimizationObjectivePtr base_opt =
            std::make_shared<ob::DimtObjective>(si, start_state, goal_state, dimt);
    base_pdef->setOptimizationObjective(base_opt);

    int start_idx = 0;
    int iteration_num = 20;
    double duration = 45.0; //run time in seconds
    std::string caseName = "simple";

    for(int i=start_idx;i<iteration_num;i++)
    {
        // Hit And Run
        {
            std::cout << " Hit And Run " << std::endl;
            auto planner = createPlanner(caseName, i, HNR, si, base_pdef, dimt, start, goal, duration);
            ob::PlannerStatus solved = planner->solveAfterLoadingSamples("samples.txt", duration);
        }

        // HMC
        {
            std::cout << " HMC " << std::endl;
            auto planner = createPlanner(caseName, i, HMC, si, base_pdef, dimt, start, goal, duration);
            ob::PlannerStatus solved = planner->solveAfterLoadingSamples("samples.txt", duration);
        }

        // HRS
        {
            std::cout << " HRS " << std::endl;
            auto planner = createPlanner(caseName, i, HRS, si, base_pdef, dimt, start, goal, duration);
            ob::PlannerStatus solved = planner->solveAfterLoadingSamples("samples.txt", duration);
        }

        // Rejection
        {
            std::cout << " Rejection " << std::endl;
            auto planner = createPlanner(caseName, i, RS, si, base_pdef, dimt, start, goal, duration);
            ob::PlannerStatus solved = planner->solveAfterLoadingSamples("samples.txt", duration);
        }
    }

}

int main()
{
    planWithSimpleSetup();
}
