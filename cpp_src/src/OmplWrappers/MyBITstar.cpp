#include <OmplWrappers/MyBITstar.h>

#include <iostream>

#include <ompl/base/spaces/RealVectorStateSpace.h>

using namespace ompl;
using namespace ompl::geometric;

// Our stuff
#include <Dimt/Params.h>


namespace ompl
{
namespace base
{

MyBITstar::MyBITstar(const ompl::base::SpaceInformationPtr &si) : BITstar(si)
{
    mode_ = RANDOM_SAMPLES;

}

base::PlannerStatus MyBITstar::solve(const base::PlannerTerminationCondition &ptc)
{

}

bool MyBITstar::toState(std::string stateString, ompl::base::State* toState)
{
    if(toState==nullptr)
    {
        return false;
    }
    if(stateString == "")
    {
        return false;
    }
    std::stringstream iss( stateString );
    int dimIdx = 0;
    double val = 0;
    while ( iss >> val && dimIdx < getSpaceInformation()->getStateDimension() )
    {
        toState->as<ompl::base::RealVectorStateSpace::StateType>()->values[dimIdx] = val;
        dimIdx ++;
    }

    return true;
}

std::string MyBITstar::fromState(ompl::base::State* fromState)
{
    std::stringstream oss;
    for(unsigned int dimIdx = 0; dimIdx < getSpaceInformation()->getStateDimension(); ++dimIdx)
    {
        oss << fromState->as<ompl::base::RealVectorStateSpace::StateType>()->values[dimIdx] << " ";
    }
    return oss.str();
}

}
}
