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
#include "load_problem.h"
#include "../External/multiLinkDI-dart/include/MultiLinkDIUtil.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

#include <limits>

typedef enum{ HNR, RS, HRS, HMC, MCMC } SamplerType;

// Construct Sampler with the base pdef and base optimization objective
double sigma = 1;
//int max_steps = 20;
int max_steps = 2000;
double alpha = 0.5;
double max_call_num = 100;
double batch_size = 100;
double epsilon = 2;//0.2;
double L = 1;
int num_trials = 5;
const double level_set = std::numeric_limits<double>::infinity();

bool MAIN_VERBOSE = true;

ompl::base::MyInformedRRTstarPtr createPlanner(std::string caseName, int index,
                                     SamplerType type, ompl::base::SpaceInformationPtr si,
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
    ob::ProblemDefinitionPtr base_pdef = std::make_shared<ob::ProblemDefinition>(si);
    base_pdef->setStartAndGoalStates(start, goal);

    const ompl::base::OptimizationObjectivePtr base_opt =
            std::make_shared<ob::DimtObjective>(si, start_state, goal_state, dimt);
    base_pdef->setOptimizationObjective(base_opt);

    switch(type)
    {
        case HNR:
            std::cout << "Planning using Hit&Run Sampler" << std::endl;
            sampler = std::make_shared<ompl::base::HitAndRunSampler>(si, base_pdef, level_set, max_call_num, batch_size, num_trials);
            samplerName = "HNR";
            break;

        case MCMC:
            std::cout << "Planning using MCMC Sampler" << std::endl;
            sampler = std::make_shared<ompl::base::MCMCSampler>(si, base_pdef, level_set, max_call_num, batch_size, alpha, sigma, max_steps);
            samplerName = "MCMC";
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

    ompl::base::OptimizationObjectivePtr opt = std::make_shared<ompl::base::MyOptimizationObjective>(si, sampler, start_state, goal_state);

    ob::ProblemDefinitionPtr pdef = std::make_shared<ob::ProblemDefinition>(si);
    pdef->setStartAndGoalStates(start, goal);
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

    // Construct the state space we are planning in
    ob::StateSpacePtr space = std::make_shared< ob::DimtStateSpace >(dimt);
    ob::RealVectorBounds bounds(param.dimensions);
    for(uint i=0;i<param.dof;i++)
    {
        bounds.setLow(i, -param.s_max);
        bounds.setHigh(i, param.s_max);
        bounds.setLow(i+param.dof, -param.v_max);
        bounds.setHigh(i+param.dof, param.v_max);
    }
    space->as<ompl::base::DimtStateSpace>()->setBounds(bounds);
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
    
    std::shared_ptr<MultiLinkDI> di = createMultiLinkDI("problem.json");
    ob::StateValidityCheckerPtr svc = createMultiLinkDIStateValidityChecker(si, di);
    //ob::StateValidityCheckerPtr svc = createStateValidityChecker(si, "obstacles.json");
    si->setStateValidityChecker(svc);
    si->setStateValidityCheckingResolution(0.01);  // 3%
    si->setup();

    // Set custom start and goal
    ompl::base::State *start_state = getStart(si, "problem.json");
    ompl::base::State *goal_state = getGoal(si, "problem.json");

    int start_idx = 0;
    int iteration_num = 10;

    double duration = 10.0; //run time in seconds

    std::string caseName = "simple";

    for(int i=start_idx;i<iteration_num;i++)
    {

        // Hit And Run
        {
            std::cout << " Hit And Run " << std::endl;
            auto planner = createPlanner(caseName, i, HNR, si, dimt, start_state, goal_state, duration);
            planner->solveAfterLoadingSamples("samples.txt", duration);
        }


        /*
        // HMC
        {
            std::cout << " HMC " << std::endl;
            auto planner = createPlanner(caseName, i, HMC, si, dimt, start_state, goal_state, duration);
            planner->solveAfterLoadingSamples("samples.txt", duration);
        }*/




        // HRS
        {
            std::cout << " HRS " << std::endl;
            auto planner = createPlanner(caseName, i, HRS, si, dimt, start_state, goal_state, duration);
            planner->solveAfterLoadingSamples("samples.txt", duration);
        }

        // Rejection
        {
            std::cout << " Rejection " << std::endl;
            auto planner = createPlanner(caseName, i, RS, si, dimt, start_state, goal_state, duration);
            planner->solveAfterLoadingSamples("samples.txt", duration);
        }

        // MCMC
        {
            std::cout << " HMC " << std::endl;
            auto planner = createPlanner(caseName, i, MCMC, si, dimt, start_state, goal_state, duration);
            planner->solveAfterLoadingSamples("samples.txt", duration);
        }

    }

}

int main()
{
    planWithSimpleSetup();
}
