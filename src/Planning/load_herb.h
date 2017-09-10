#ifndef LOAD_HERB_H_
#define LOAD_HERB_H_

#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/constraint/TSR.hpp>
#include <aikido/constraint/NonColliding.hpp>
#include <dart/dart.hpp>

#include <libherb/herb.hpp>
#include <aikido/util/CatkinResourceRetriever.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <dart/collision/CollisionDetector.hpp>
#include <dart/collision/CollisionOption.hpp>
#include <dart/collision/CollisionGroup.hpp>


const dart::dynamics::SkeletonPtr makeBodyFromURDF(const std::string& uri,
  const Eigen::Isometry3d& transform)
{
  // Resolves package:// URIs by emulating the behavior of 'catkin_find'.
  const auto resourceRetriever
    = std::make_shared<aikido::util::CatkinResourceRetriever>();

  dart::utils::DartLoader urdfLoader;
  const dart::dynamics::SkeletonPtr skeleton = urdfLoader.parseSkeleton(
    uri, resourceRetriever);

  if (!skeleton) {
    throw std::runtime_error("unable to load '" + uri + "'");
  }

  dynamic_cast<dart::dynamics::FreeJoint*>(skeleton->getJoint(0))->setTransform(transform);

  return skeleton;
}

std::shared_ptr<herb::Herb> loadHerb()
{
    std::shared_ptr<herb::Herb> herb = std::make_shared<herb::Herb>(true);
    return herb;
}

double getHerbRightArmVelUpperLimit(std::shared_ptr<herb::Herb> herb, int idx)
{
  if(herb->getRightArm())
  {
      return herb->getRightArm()->getVelocityUpperLimit(idx);
  }
  return 0.0;
}

double getHerbRightArmVelLowerLimit(std::shared_ptr<herb::Herb> herb, int idx)
{
  if(herb->getRightArm())
  {
      return herb->getRightArm()->getVelocityLowerLimit(idx);
  }
  return 0.0;
}

double getHerbRightArmPosUpperLimit(std::shared_ptr<herb::Herb> herb, int idx)
{
  if(herb->getRightArm())
  {
      return herb->getRightArm()->getPositionUpperLimit(idx);
  }
  return 0.0;
}

double getHerbRightArmPosLowerLimit(std::shared_ptr<herb::Herb> herb, int idx)
{
  if(herb->getRightArm())
  {
      return herb->getRightArm()->getPositionLowerLimit(idx);
  }
  return 0.0;
}

aikido::constraint::NonCollidingPtr createWorldNonColliding(std::shared_ptr<herb::Herb> herb)
{
	using dart::dynamics::Frame;
	using dart::dynamics::SimpleFrame;
	using dart::collision::CollisionGroup;
	using dart::collision::CollisionDetectorPtr;
	using dart::dynamics::Skeleton;
	using dart::dynamics::SkeletonPtr;

	using aikido::constraint::NonColliding;
	using aikido::statespace::dart::MetaSkeletonStateSpace;
	using aikido::constraint::TSR;
	using aikido::constraint::NonColliding;

  dart::simulation::WorldPtr env(new dart::simulation::World);

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

    // Load plastic glass
    glass = makeBodyFromURDF(glassURDFUri, glassPose);

    auto rightArmSpace =
      std::make_shared<MetaSkeletonStateSpace>(herb->getRightArm());
    CollisionDetectorPtr collisionDetector = dart::collision::FCLCollisionDetector::create();
    auto nonCollidingConstraint =
      std::make_shared<NonColliding>(rightArmSpace, collisionDetector);

    std::shared_ptr<CollisionGroup> armGroup = collisionDetector->createCollisionGroup();
    armGroup->addShapeFramesOf(rightArmSpace->getMetaSkeleton().get());

  std::shared_ptr<CollisionGroup> envGroup = collisionDetector->createCollisionGroup();
  envGroup->addShapeFramesOf(table.get());
  envGroup->addShapeFramesOf(glass.get());

nonCollidingConstraint->addPairwiseCheck(armGroup, envGroup);

  return nonCollidingConstraint;

}


#endif // LOAD_HERB_H_
