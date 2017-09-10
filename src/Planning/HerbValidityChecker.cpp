#include "HerbValidityChecker.hpp"
#include <aikido/constraint/TestableIntersection.hpp>
#include <aikido/planner/ompl/GeometricStateSpace.hpp>

HerbValidityChecker::~HerbValidityChecker()
{
}

bool HerbValidityChecker::isValid(const ompl::base::State *state) const
{
    if(herb_)
    {
        auto st = static_cast<const aikido::planner::ompl::GeometricStateSpace::StateType*>(state);
        if(st==nullptr || st->mState == nullptr)
            return false;
        
        if(testable_ == nullptr)
        {
            return true;
        }
        if(testable_->isSatisfied(st->mState))
        {
            return true;
        }
    }
    return true;
}

aikido::constraint::TestablePtr HerbValidityChecker::getCollisionConstraints(
	  aikido::statespace::dart::MetaSkeletonStateSpacePtr _space,
	  aikido::constraint::NonCollidingPtr _nonColliding) const
{
  using aikido::constraint::TestableIntersection;

  auto selfNonColliding = getSelfCollisionConstraint(_space);

  // Make testable constraints for collision check
  std::vector<aikido::constraint::TestablePtr> constraints;
  constraints.reserve(2);
  constraints.emplace_back(selfNonColliding);
  if (nonColliding_)
  {
    if (nonColliding_->getStateSpace() != _space)
    {
      throw std::runtime_error("NonColliding has incorrect statespace.");
    }
    constraints.emplace_back(_nonColliding);
  }

  return std::make_shared<TestableIntersection>(_space, constraints);
}

aikido::constraint::NonCollidingPtr
HerbValidityChecker::getSelfCollisionConstraint(
    aikido::statespace::dart::MetaSkeletonStateSpacePtr _space) const
{
  using aikido::constraint::NonColliding;

  auto collisionDetector = dart::collision::FCLCollisionDetector::create();

  // TODO: Switch to PRIMITIVE once this is fixed in DART.
  // collisionDetector->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);

  auto nonCollidingConstraint =
      std::make_shared<NonColliding>(_space, collisionDetector);
  /*
  nonCollidingConstraint->addSelfCheck(
      collisionDetector->createCollisionGroupAsSharedPtr(herb_.get()));
  */
  return nonCollidingConstraint;
}
