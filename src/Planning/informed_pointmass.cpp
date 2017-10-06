#include <iostream>
#include <fstream>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
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
#include "Util/load_problem.h"
#include "../External/multiLinkDI-dart/include/MultiLinkDIUtil.hpp"
#include "Util/file_util.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

#include <limits>

bool MAIN_VERBOSE = true;

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
    si->setStateValidityCheckingResolution(0.002);  // 3%
    si->setup();

    ob::ProblemDefinitionPtr base_pdef = createProblem(si, "problem.json");

    const ompl::base::OptimizationObjectivePtr base_opt = createDimtOptimizationObjective(si, dimt, "problem.json");
    base_pdef->setOptimizationObjective(base_opt);

    // Construct Sampler with the base pdef and base optimization objective
    double sigma = 1;
    //int max_steps = 20;
    int max_steps = 200;
    double alpha = 0.5;
    double max_call_num = 100;
    double batch_size = 100;
    double epsilon = 0.1;
    double L = 5;
    int num_trials = 5;
    const double level_set = std::numeric_limits<double>::infinity();
    //const auto sampler = std::make_shared<ompl::base::HMCSampler>(si, base_pdef, level_set, max_call_num, batch_size, alpha, L, epsilon, sigma, max_steps);
    //const auto sampler = std::make_shared<ompl::base::MCMCSampler>(si, base_pdef, level_set, max_call_num, batch_size, alpha, sigma, max_steps);
    //const auto sampler = std::make_shared<ompl::base::DimtHierarchicalRejectionSampler>(si, base_pdef, dimt, level_set, max_call_num, batch_size);
    const auto sampler = std::make_shared<ompl::base::HitAndRunSampler>(si, base_pdef, level_set, max_call_num, batch_size, num_trials);
    //const auto sampler = std::make_shared<ompl::base::RejectionSampler>(si, base_pdef, level_set, max_call_num, batch_size);

    sampler->setSingleSampleTimelimit(10.);


    const ompl::base::OptimizationObjectivePtr opt = createOptimizationObjective(si, sampler, "problem.json");

    ob::ProblemDefinitionPtr pdef = createProblem(si, "problem.json");
    //opt->setCostThreshold(ob::Cost(1.51));
    pdef->setOptimizationObjective(opt);

    ob::MyInformedRRTstarPtr planner = std::make_shared<ob::MyInformedRRTstar>(si);

    // Set the problem instance for our planner to solve
    planner->setProblemDefinition(pdef);
    planner->setup();

    // Run planner
    ob::PlannerStatus solved = planner->solve(60.0);

    //ob::PlannerStatus solved = planner->solveAndSaveSamples("samples.txt", 60.0);
    //ob::lannerStatus solved = planner->solveAfterLoadingSamples("samples.txt", 60.0);

    //return;
    if(pdef->hasSolution())
    {
        std::cout << "Has a solution" << std::endl;
        ob::PathPtr path = pdef->getSolutionPath();
        dumpPathToFile(path, "waypointpath.txt");
        dumpPathToFile(path, dimt, "path.txt");
    }

    ompl::base::State* start = getStart(si, "problem.txt");
    ompl::base::State* goal = getGoal(si, "problem.txt");
    std::vector<ompl::base::State*> stateSeq;
    stateSeq.push_back(start);
    stateSeq.push_back(goal);
    dumpPathToFile(stateSeq, dimt, "testpath.txt");


}

int main()
{
    planWithSimpleSetup();
}
