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
#include "Util/load_problem.h"
#include "load_herb.h"

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
    // Intiatilizations for sampler
    const int dimension = param.dimensions;

    std::shared_ptr<herb::Herb> herb = loadHerb();
    std::shared_ptr<aikido::statespace::dart::MetaSkeletonStateSpace> rightArmSpace
    = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(herb->getRightArm());
    using dart::dynamics::Frame;
    using dart::dynamics::SimpleFrame;
    using dart::collision::CollisionGroup;
    using dart::collision::CollisionDetectorPtr;
    using dart::dynamics::Skeleton;
    using dart::dynamics::SkeletonPtr;

    using aikido::constraint::NonColliding;
    using aikido::statespace::dart::MetaSkeletonStateSpace;
    using dart::dynamics::SimpleFrame;
    using aikido::constraint::TSR;
    using aikido::constraint::NonColliding;


    SkeletonPtr table, glass;
    Eigen::Isometry3d tablePose, glassPose, glassGoalPose;
    //Specify the URDFs of the objects of interest
    const std::string tableURDFUri("package://pr_ordata/data/furniture/table.urdf");
    const std::string glassURDFUri("package://pr_ordata/data/objects/plastic_glass.urdf");

    // Poses for table and glass
    tablePose = Eigen::Isometry3d::Identity();
    tablePose.translation() = Eigen::Vector3d(0.8, 0.4, 0);
    Eigen::Matrix3d rot;
    rot = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
    tablePose.linear() = rot;

    glassPose = Eigen::Isometry3d::Identity();
    glassPose.translation() = tablePose.translation() + Eigen::Vector3d(0, -0.6, 0.73);

    glassGoalPose = glassPose;
    glassGoalPose.translation() += Eigen::Vector3d(0.0, 0.5, 0.0);

    // Load table
    table = makeBodyFromURDF(tableURDFUri, tablePose);
    if(table==nullptr)
    {
       std::cout << "table is null" << std::endl;
    }

    // Load plastic glass
    glass = makeBodyFromURDF(glassURDFUri, glassPose);
    if(glass==nullptr)
    {
       std::cout << "glass is null" << std::endl;
    }

    CollisionDetectorPtr collisionDetector = dart::collision::FCLCollisionDetector::create();
    auto nonCollidingConstraint =
      std::make_shared<NonColliding>(rightArmSpace, collisionDetector);

    std::shared_ptr<CollisionGroup> armGroup = collisionDetector->createCollisionGroup();
    armGroup->addShapeFramesOf(rightArmSpace->getMetaSkeleton().get());

    std::shared_ptr<CollisionGroup> envGroup = collisionDetector->createCollisionGroup();
    envGroup->addShapeFramesOf(table.get());
    //envGroup->addShapeFramesOf(glass.get());

    nonCollidingConstraint->addPairwiseCheck(armGroup, envGroup);
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
    
    ob::StateValidityCheckerPtr svc = createHerbStateValidityChecker(si, herb, rightArmSpace, nonCollidingConstraint);
    //ob::StateValidityCheckerPtr svc = createStateValidityChecker(si, "obstacles.json");
    si->setStateValidityChecker(svc);
    si->setStateValidityCheckingResolution(0.001);  // 3%
    si->setup();

    #define PROBLEM_FILENAME "herb_problem.json"

    // Set custom start and goal
    ompl::base::State *start_state = getStart(si, PROBLEM_FILENAME);
    ompl::base::State *goal_state = getGoal(si, PROBLEM_FILENAME);

    int start_idx = 0;

    int iteration_num = 20;
    double duration = 20.0; //run time in seconds

    std::string caseName = "simple";

    for(int i=start_idx;i<iteration_num;i++)
    {

        // Hit And Run
        {
            std::cout << " Hit And Run " << std::endl;
            auto planner = createPlanner(caseName, i, HNR, si, dimt, start_state, goal_state, duration);
            planner->solveAfterLoadingSamples("samples.txt", duration);
            //ob::PlannerStatus solved = planner->solve(duration);

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
            //ob::PlannerStatus solved = planner->solve(duration);

        }

        // Rejection
        {
            std::cout << " Rejection " << std::endl;
            auto planner = createPlanner(caseName, i, RS, si, dimt, start_state, goal_state, duration);
            planner->solveAfterLoadingSamples("samples.txt", duration);
            //ob::PlannerStatus solved = planner->solve(duration);

        }

        // MCMC
        {
            std::cout << " HMC " << std::endl;
            auto planner = createPlanner(caseName, i, MCMC, si, dimt, start_state, goal_state, duration);
            planner->solveAfterLoadingSamples("samples.txt", duration);
            //ob::PlannerStatus solved = planner->solve(duration);

        }

    }

}

int main()
{
    planWithSimpleSetup();
}
