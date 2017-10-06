#include "HerbValidityChecker.hpp"
#include <aikido/constraint/TestableIntersection.hpp>
#include <aikido/planner/ompl/GeometricStateSpace.hpp>

HerbValidityChecker::~HerbValidityChecker()
{
}

bool HerbValidityChecker::isValid(const ompl::base::State *state) const
{
  
	auto st = static_cast<const aikido::planner::ompl::GeometricStateSpace::StateType*>(state);
	if(st==nullptr || st->mState == nullptr)
	    return false;

	if(testable_->isSatisfied(st->mState))
	{
	    return true;
	}
	else
	{
	    return false; 
	}
    
    return true;
}

aikido::constraint::TestablePtr HerbValidityChecker::getCollisionConstraints(
          std::shared_ptr<herb::Herb> herb,
	  aikido::statespace::dart::MetaSkeletonStateSpacePtr _space,
	  aikido::constraint::NonCollidingPtr _nonColliding) const
{
  using aikido::constraint::TestableIntersection;

  auto selfNonColliding = getSelfCollisionConstraint(herb, _space);

  // Make testable constraints for collision check
  std::vector<aikido::constraint::TestablePtr> constraints;
  constraints.reserve(2);
  constraints.emplace_back(selfNonColliding);
  if (_nonColliding)
  {
    /*
    if (_nonColliding->getStateSpace() != _space)
    {
      throw std::runtime_error("NonColliding has incorrect statespace.");
    }*/
    constraints.emplace_back(_nonColliding);
  }

  return std::make_shared<TestableIntersection>(_space, constraints);
}

aikido::constraint::NonCollidingPtr
HerbValidityChecker::getSelfCollisionConstraint(
    std::shared_ptr<herb::Herb> herb,
    aikido::statespace::dart::MetaSkeletonStateSpacePtr _space) const
{
  using aikido::constraint::NonColliding;

  herb->getSkeleton()->enableSelfCollisionCheck();
  herb->getSkeleton()->disableAdjacentBodyCheck();

  std::vector<std::string> disableCollisions{"/left/hand_base",
                                             "/right/hand_base",
                                             "/left/wam1",
                                             "/right/wam1",
                                             "/left/wam6",
                                             "/right/wam6"};
  for(const auto& bodyNodeName: disableCollisions)
    herb->getSkeleton()->getBodyNode(bodyNodeName)->setCollidable(false);

  auto collisionDetector = dart::collision::FCLCollisionDetector::create();

  // TODO: Switch to PRIMITIVE once this is fixed in DART.
  // collisionDetector->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);

  auto nonCollidingConstraint =
      std::make_shared<aikido::constraint::NonColliding>(_space, collisionDetector);
  nonCollidingConstraint->addSelfCheck(
      collisionDetector->createCollisionGroupAsSharedPtr(herb->getSkeleton().get()));
  return nonCollidingConstraint;
}
