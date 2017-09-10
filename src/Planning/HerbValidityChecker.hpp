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
                        aikido::constraint::NonCollidingPtr nonColliding)
                              
        : ompl::base::StateValidityChecker(si), herb_(herb), nonColliding_(nullptr)
    {
        statespace_ = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>
            (herb_->getRightArm());
        testable_ = getCollisionConstraints(statespace_, nonColliding);
    }

    virtual ~HerbValidityChecker();

    aikido::constraint::TestablePtr getCollisionConstraints(
	  aikido::statespace::dart::MetaSkeletonStateSpacePtr _space,
	  aikido::constraint::NonCollidingPtr _nonColliding) const;

    aikido::constraint::NonCollidingPtr getSelfCollisionConstraint(
          aikido::statespace::dart::MetaSkeletonStateSpacePtr _space) const;

    bool isValid(const ompl::base::State *state) const;
protected:
    std::shared_ptr<herb::Herb> herb_;
    std::shared_ptr<aikido::statespace::dart::MetaSkeletonStateSpace> statespace_;
    aikido::constraint::NonCollidingPtr nonColliding_;
    aikido::constraint::TestablePtr testable_;
};

#endif // HERB_VALIDITY_CHECKER_H_
