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
//int max_steps = 20;
double alpha = 0.5;
double max_call_num = 100;
double batch_size = 100;
//double epsilon = 0.1;
//double L = 5;
int num_trials = 5;
const double level_set = std::numeric_limits<double>::infinity();

bool MAIN_VERBOSE = true;

ompl::base::MyInformedRRTstarPtr createHMCPlanner(std::string caseName, int index, double hmcL, double hmcEpsilon, int hmcMaxSteps,
                                     ompl::base::SpaceInformationPtr si, DIMTPtr dimt,
                                     const ompl::base::State* start_state,
                                     const ompl::base::State* goal_state,
                                     double singleSampleLimit)
{
    ompl::base::MyInformedSamplerPtr sampler;
    std::string samplerName;

    ob::ScopedState<ompl::base::RealVectorStateSpace> start(si->getStateSpace(), start_state);
    ob::ScopedState<ompl::base::RealVectorStateSpace> goal(si->getStateSpace(), goal_state);

    // Set up the final problem with the full optimization objective
    ob::ProblemDefinitionPtr pdef = std::make_shared<ob::ProblemDefinition>(si);
    pdef->setStartAndGoalStates(start, goal);

    ob::ProblemDefinitionPtr base_pdef = std::make_shared<ob::ProblemDefinition>(si);
    base_pdef->setStartAndGoalStates(start, goal);

    const ompl::base::OptimizationObjectivePtr base_opt =
            std::make_shared<ob::DimtObjective>(si, start_state, goal_state, dimt);
    base_pdef->setOptimizationObjective(base_opt);

    sampler = std::make_shared<ompl::base::HMCSampler>(si, base_pdef, level_set, max_call_num, batch_size, alpha, hmcL, hmcEpsilon, sigma, hmcMaxSteps);
    samplerName = "HMC";

    sampler->setSingleSampleTimelimit(singleSampleLimit);

    ompl::base::OptimizationObjectivePtr opt = std::make_shared<ompl::base::MyOptimizationObjective>(si, sampler, start_state, goal_state);

    pdef->setOptimizationObjective(opt);

    ob::MyInformedRRTstarPtr planner = std::make_shared<ob::MyInformedRRTstar>(si);

    // Set the problem instance for our planner to solve
    planner->setProblemDefinition(pdef);
    planner->setup();
    planner->initLogFile(caseName, samplerName, index);

    return planner;
}

ompl::base::MyInformedRRTstarPtr createHRSPlanner(std::string caseName, int index,
                                     ompl::base::SpaceInformationPtr si,
                                     DIMTPtr dimt,
                                     const ompl::base::State* start_state,
                                     const ompl::base::State* goal_state,
                                     double singleSampleLimit)
{
    ompl::base::MyInformedSamplerPtr sampler;
    std::string samplerName;

    ob::ScopedState<ompl::base::RealVectorStateSpace> start(si->getStateSpace(), start_state);
    ob::ScopedState<ompl::base::RealVectorStateSpace> goal(si->getStateSpace(), goal_state);

    // Set up the final problem with the full optimization objective
    ob::ProblemDefinitionPtr pdef = std::make_shared<ob::ProblemDefinition>(si);
    pdef->setStartAndGoalStates(start, goal);

    ob::ProblemDefinitionPtr base_pdef = std::make_shared<ob::ProblemDefinition>(si);
    base_pdef->setStartAndGoalStates(start, goal);

    const ompl::base::OptimizationObjectivePtr base_opt =
            std::make_shared<ob::DimtObjective>(si, start_state, goal_state, dimt);
    base_pdef->setOptimizationObjective(base_opt);

    sampler = std::make_shared<ompl::base::DimtHierarchicalRejectionSampler>(si, base_pdef, dimt, level_set, max_call_num, batch_size);
    samplerName = "HRS";

    sampler->setSingleSampleTimelimit(singleSampleLimit);

    ompl::base::OptimizationObjectivePtr opt = std::make_shared<ompl::base::MyOptimizationObjective>(si, sampler, start_state, goal_state);

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


    // Intiatilizations for sampler
    const int dimension = param.dimensions;


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

    // Set custom start and goal
    ompl::base::State *start_state = space->allocState();
    ompl::base::State *goal_state = space->allocState();
    for (int i = 0; i < param.dimensions; i++)
    {
        if (i % 2 == 0)  // position
        {
            start_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = -4.;
            goal_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = 4.;
        }
        else  // velocity
        {
            start_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = 2.;
            goal_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = 2.;
        }
    }

    double duration = 45.0; //run time in second
    int start_idx = 0;
    int iteration_num = 10;

    int case_num = 3;
    double epsilon_set [] = {0.1, 0.1, 0.1};
    double L_set [] = {5, 5, 5};
    int maxSteps_set [] = {20, 200, 2000};


    std::string caseName = "";
    for(int i=start_idx;i<iteration_num;i++)
    {
        for(int caseIdx=0;caseIdx<case_num;caseIdx++)
        {
            std::stringstream ss;
            ss << "diffHMC" << caseIdx;
            caseName = ss.str();

            auto planner = createHMCPlanner(caseName, i, epsilon_set[caseIdx], L_set[caseIdx], maxSteps_set[caseIdx], si, dimt, start_state, goal_state, duration);
            std::cout << "HMC with epsilon = " << epsilon_set[caseIdx] << " L = " << L_set[caseIdx] << " MAXSTEP = " << maxSteps_set[caseIdx] << std::endl;
            ob::PlannerStatus solved = planner->solveAfterLoadingSamples("samples.txt", duration);
        }

        caseName = "HRS";
        auto planner2 = createHRSPlanner(caseName, i, si, dimt, start_state, goal_state, duration);
        ob::PlannerStatus solved2 = planner2->solveAfterLoadingSamples("samples.txt", duration);
    }

}

int main()
{
    planWithSimpleSetup();
}
