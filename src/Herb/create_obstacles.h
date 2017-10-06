#ifndef CREATE_OBSTACLES_H_
#define CREATE_OBSTACLES_H_

#include "Dimt/Params.h"
#include "OmplWrappers/ValidityChecker.h"
#include "HerbValidityChecker.hpp"


ompl::base::StateValidityCheckerPtr 
createHerbStateValidityChecker(ompl::base::SpaceInformationPtr si,
                               std::shared_ptr<herb::Herb> herb,
                               std::shared_ptr<aikido::statespace::dart::MetaSkeletonStateSpace> space,
                               aikido::constraint::NonCollidingPtr nonColliding)
{
    ompl::base::StateValidityCheckerPtr pVC = 
        std::make_shared<HerbValidityChecker>(si, herb, space, nonColliding);
    return pVC;
}

#endif // CREATE_OBSTACLES_H_
