#ifndef HERB_VALIDITY_CHECKER_H_
#define HERB_VALIDITY_CHECKER_H_

#include <ompl/base/StateValidityChecker.h>
#include <aikido/constraint/Testable.hpp>
#include <aikido/constraint/NonColliding.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <libherb/herb.hpp>

class HerbValidityChecker : public ompl::base::StateValidityChecker
{
public:
    HerbValidityChecker(const ompl::base::SpaceInformationPtr &si,
                        std::shared_ptr<herb::Herb> herb,
                        std::shared_ptr<aikido::statespace::dart::MetaSkeletonStateSpace> space,
                        aikido::constraint::NonCollidingPtr nonColliding)
                              
        : ompl::base::StateValidityChecker(si)
    {
        testable_ = getCollisionConstraints(herb, space, nonColliding);

        std::cout << "CREATED " << std::endl;
    }

    virtual ~HerbValidityChecker();

    aikido::constraint::TestablePtr getCollisionConstraints(
          std::shared_ptr<herb::Herb> herb,
	  aikido::statespace::dart::MetaSkeletonStateSpacePtr _space,
	  aikido::constraint::NonCollidingPtr _nonColliding) const;

    aikido::constraint::NonCollidingPtr getSelfCollisionConstraint(
          std::shared_ptr<herb::Herb> herb,
          aikido::statespace::dart::MetaSkeletonStateSpacePtr _space) const;

    bool isValid(const ompl::base::State *state) const;
protected:
    aikido::constraint::TestablePtr testable_;
};

#endif // HERB_VALIDITY_CHECKER_H_
