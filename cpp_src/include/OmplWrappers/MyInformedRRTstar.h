#pragma once

// OMPL
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>

class MyInformedRRTstar: public ompl::geometric::InformedRRTstar
{
public:
    MyInformedRRTstar(const ompl::base::SpaceInformationPtr &si);

    virtual ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc) override;
};
