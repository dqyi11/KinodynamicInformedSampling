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
#include "load_problem.h"
#include "file_util.hpp"
#include "load_herb.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

#include <limits>

bool MAIN_VERBOSE = true;

void planWithSimpleSetup(void)
{

    // Intiatilizations for sampler
    const int dimension = param.dimensions;

    std::shared_ptr<herb::Herb> herb = loadHerb();
    //aikido::constraint::NonCollidingPtr nonColliding = createWorldNonColliding(herb);
    aikido::constraint::NonCollidingPtr nonColliding = nullptr;    

    // Initializations
    std::vector<double> maxVelocities(param.dof, param.v_max);
    std::vector<double> maxAccelerations(param.dof, param.a_max);
    for(uint i=0;i<param.dof;i++)
    {
        maxVelocities[i] = getHerbRightArmVelUpperLimit(herb, i);
    }
    DIMTPtr dimt = std::make_shared<DIMT>( maxAccelerations, maxVelocities );

    // Construct the state space we are planning in
    ob::StateSpacePtr space = std::make_shared< ob::DimtStateSpace >(dimt);
    ob::RealVectorBounds bounds(param.dimensions);
    for(uint i=0;i<param.dof;i++)
    {

        bounds.setLow(i, getHerbRightArmPosLowerLimit(herb, i));
        bounds.setHigh(i, getHerbRightArmPosUpperLimit(herb, i));
        bounds.setLow(i+param.dof, getHerbRightArmVelLowerLimit(herb, i));
        bounds.setHigh(i+param.dof, getHerbRightArmVelUpperLimit(herb, i));
    }
    space->as<ompl::base::DimtStateSpace>()->setBounds(bounds);
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
    
    ob::StateValidityCheckerPtr svc = createHerbStateValidityChecker(si, herb, nonColliding);
    //ob::StateValidityCheckerPtr svc = createStateValidityChecker(si, "obstacles.json");
    si->setStateValidityChecker(svc);
    si->setStateValidityCheckingResolution(0.002);  // 3%
    si->setup();

    #define PROBLEM_FILENAME "herb_problem.json"

    ob::ProblemDefinitionPtr base_pdef = createProblem(si, PROBLEM_FILENAME);

    const ompl::base::OptimizationObjectivePtr base_opt = createDimtOptimizationObjective(si, dimt, PROBLEM_FILENAME);
    base_pdef->setOptimizationObjective(base_opt);

    std::cout << "STATE SPACE BOUNDARY" << std::endl;
    ompl::base::RealVectorBounds bds = si->getStateSpace()->as<ompl::base::RealVectorStateSpace>()->getBounds();
    for(size_t i=0;i<si->getStateSpace()->getDimension();i++)
    {
        std::cout << "Dimension " << i << " [" << bds.low[i] << " , " << bds.high[i] << "]" << std::endl;
    }

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
    const auto sampler = std::make_shared<ompl::base::DimtHierarchicalRejectionSampler>(si, base_pdef, dimt, level_set, max_call_num, batch_size);
    //const auto sampler = std::make_shared<ompl::base::HitAndRunSampler>(si, base_pdef, level_set, max_call_num, batch_size, num_trials);
    //const auto sampler = std::make_shared<ompl::base::RejectionSampler>(si, base_pdef, level_set, max_call_num, batch_size);
    sampler->setSingleSampleTimelimit(60.);

    const ompl::base::OptimizationObjectivePtr opt = createOptimizationObjective(si, sampler, PROBLEM_FILENAME);

    ob::ProblemDefinitionPtr pdef = createProblem(si, PROBLEM_FILENAME);
    //opt->setCostThreshold(ob::Cost(1.51));
    pdef->setOptimizationObjective(opt);

    ob::MyInformedRRTstarPtr planner = std::make_shared<ob::MyInformedRRTstar>(si);

    // Set the problem instance for our planner to solve
    planner->setProblemDefinition(pdef);
    planner->setup();

    // Run planner
    //ob::PlannerStatus solved = planner->solve(30.0);

    ob::PlannerStatus solved = planner->solveAndSaveSamples("samples.txt", 60.0);
    //ob::lannerStatus solved = planner->solveAfterLoadingSamples("samples.txt", 60.0);

    //return;
    if(pdef->hasSolution())
    {
        std::cout << "Has a solution" << std::endl;
        ob::PathPtr path = pdef->getSolutionPath();
        dumpPathToFile(path, "waypointpath.txt");
        dumpPathToFile(path, dimt, "path.txt");
    }

}

int main()
{
    planWithSimpleSetup();
}
