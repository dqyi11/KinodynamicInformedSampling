#ifndef PROBLEM_GENERATION_H_
#define PROBLEM_GENERATION_H_

#include <random>
#include <Eigen/Dense>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/samplers/InformedStateSampler.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include "Dimt/Params.h"
#include "Dimt/DoubleIntegratorMinimumTime.h"


void sampleStartAndGoal(Eigen::VectorXd& startVec, Eigen::VectorXd& goalVec)
{
    assert(startVec.cols()==goalVec.cols());
    for (int i = 0; i < startVec.cols(); i++)
    {
        //startVec(i) = dist(rng);
        //goalVec(i) = dist(rng);
    }
}

ompl::base::SpaceInformationPtr createDimtSpaceInformation(DIMTPtr dimt,
                                                           int minval,
                                                           int maxval
                                                           )
{
    // Construct the state space we are planning in
    ompl::base::StateSpacePtr space =
            std::make_shared< ompl::base::DimtStateSpace >(dimt);
    ompl::base::RealVectorBounds bounds(param.dimensions);
    bounds.setLow(minval);
    bounds.setHigh(maxval);
    space->as<ompl::base::DimtStateSpace>()->setBounds(bounds);
    ompl::base::SpaceInformationPtr si =
            std::make_shared< ompl::base::SpaceInformation >(space);
    si->setup();
    return si;
}

ompl::base::ProblemDefinitionPtr createDimtProblem(const Eigen::VectorXd& startVec,
                                                   const Eigen::VectorXd& goalVec,
                                                   ompl::base::SpaceInformationPtr si,
                                                   DIMTPtr dimt
                                                  )
{
    ompl::base::StateSpacePtr space = si->getStateSpace();
    // Set custom start and goal
    auto startState = space->allocState();
    auto goalState = space->allocState();
    for (unsigned int d = 0; d < space->getDimension(); d++)
    {
        if (d % 2 == 0)  // position
        {
            startState->as<ompl::base::RealVectorStateSpace::StateType>()->values[d] = startVec[d];
            goalState->as<ompl::base::RealVectorStateSpace::StateType>()->values[d] = goalVec[d];
        }
        else  // velocity
        {
            startState->as<ompl::base::RealVectorStateSpace::StateType>()->values[d] = startVec[d];
            goalState->as<ompl::base::RealVectorStateSpace::StateType>()->values[d] = goalVec[d];
        }
    }
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start(space, startState);
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal(space, goalState);

    // create new problem definition
    ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));
    pdef->setStartAndGoalStates(start, goal);

    const ompl::base::OptimizationObjectivePtr opt =
            std::make_shared<ompl::base::DimtObjective>(si, startVec, goalVec, dimt);
    pdef->setOptimizationObjective(opt);

    space->freeState(startState);
    space->freeState(goalState);

    return pdef;
}

#endif // PROBLEM_GENERATION_H_
