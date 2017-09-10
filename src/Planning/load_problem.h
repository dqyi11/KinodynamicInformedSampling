#ifndef LOAD_PROBLEM_H_
#define LOAD_PROBLEM_H_

#include "jsoncpp/json/json.h"
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include "OmplWrappers/MyOptimizationObjective.h"
#include "Sampler/Sampler.h"

ompl::base::State* getStart(ompl::base::SpaceInformationPtr si, std::string filename)
{
    Json::Reader reader;
    Json::Value root;
    std::ifstream file(filename, std::ifstream::binary);
    bool parseSuccess = reader.parse(file, root, false);

    ompl::base::State *start_state = si->getStateSpace()->allocState();

    if(parseSuccess)
    {
        Json::Value startVal = root["start"];
        for(int i=0;i<startVal.size();i++)
        {
          start_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = startVal[i].asDouble();
        }
    }
    file.close();
    return start_state;
}

ompl::base::State* getGoal(ompl::base::SpaceInformationPtr si, std::string filename)
{
    Json::Reader reader;
    Json::Value root;
    std::ifstream file(filename, std::ifstream::binary);
    bool parseSuccess = reader.parse(file, root, false);

    ompl::base::State *goal_state = si->getStateSpace()->allocState();

    if(parseSuccess)
    {
        Json::Value goalVal = root["goal"];
        for(int i=0;i<goalVal.size();i++)
        {
            goal_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = goalVal[i].asDouble();
        }
    }
    file.close();
    return goal_state;
}


ompl::base::ProblemDefinitionPtr createProblem(ompl::base::SpaceInformationPtr si,
                                               std::string filename)
{
    ompl::base::State *start_state = getStart(si, filename);
    ompl::base::State *goal_state = getGoal(si, filename);

    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start(si->getStateSpace(), start_state);
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal(si->getStateSpace(), goal_state);

    // Set up the final problem with the full optimization objective
    ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));
    pdef->setStartAndGoalStates(start, goal);

    return pdef;
}

const ompl::base::OptimizationObjectivePtr createDimtOptimizationObjective(ompl::base::SpaceInformationPtr si,
                                                                     DIMTPtr dimt,
                                                                     std::string filename)
{
    ompl::base::State *start_state = getStart(si, filename);
    ompl::base::State *goal_state = getGoal(si, filename);
    const ompl::base::OptimizationObjectivePtr base_opt = std::make_shared<ompl::base::DimtObjective>(si, start_state, goal_state, dimt);

    return base_opt;
}

const ompl::base::OptimizationObjectivePtr createOptimizationObjective(ompl::base::SpaceInformationPtr si,
                                                                 ompl::base::MyInformedSamplerPtr sampler,
                                                                 std::string filename)
{
    ompl::base::State *start_state = getStart(si, filename);
    ompl::base::State *goal_state = getGoal(si, filename);

    const ompl::base::OptimizationObjectivePtr opt = std::make_shared<ompl::base::MyOptimizationObjective>(si, sampler, start_state, goal_state);
    return opt;
}

#endif // LOAD_PROBLEM_H_
