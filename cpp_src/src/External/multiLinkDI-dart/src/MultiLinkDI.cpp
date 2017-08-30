#include <sstream>
#include <string>
#include "MultiLinkDI.hpp"
#include "MultiLinkDIWindow.hpp"

using namespace dart::simulation;
using namespace dart::dynamics;

MultiLinkDI::MultiLinkDI(const unsigned int num_of_links, Eigen::Vector3d& pos)
    : basePos_(pos)
{
  num_of_links_ = num_of_links;

  world_ = std::make_shared<dart::simulation::World>();
  assert(world_ != nullptr);

  Eigen::Vector3d gravity(0.0, 0.0, 0.0);
  world_->setGravity(gravity);

  di_ = Skeleton::create("di");

  // Add each body to the last BodyNode in the di
  BodyNode* bn = makeRootBody(di_, "body1");
  bodyNodes_.push_back(bn);

  for(unsigned int i=1; i<num_of_links_; i++)
  {
    std::stringstream ss;
    ss << "body" << i+1;
    bn = addBody(di_, bn, ss.str());
    bodyNodes_.push_back(bn);
  }

  Eigen::Vector3d planePos;
  Eigen::Vector3d planeSize;
  planeSize << 8.0, 8.0, 0.02;
  plane_ = createCube(planePos, planeSize, default_cube_mass);

  world_->addSkeleton(di_);
  world_->addSkeleton(plane_);

  window_.setWorld(world_);
  window_.setMultiLinkDI(this);
}

MultiLinkDI::~MultiLinkDI()
{

}

void MultiLinkDI::setDofConfiguration(unsigned int idx, double config)
{
  di_->getDof(idx)->setPosition(config);
}

void MultiLinkDI::setConfiguration(const Eigen::VectorXd& config)
{
  for(unsigned int idx=0;idx<num_of_links_;idx++)
  {
    di_->getDof(idx)->setPosition(config[idx]);
  }
}

Eigen::VectorXd MultiLinkDI::getConfiguration()
{
   Eigen::VectorXd pos(num_of_links_);
   for(unsigned int idx=0;idx<num_of_links_;idx++)
   {
       pos[idx] = di_->getDof(idx)->getPosition();
   }
   return pos;
}

void MultiLinkDI::addCube(const Eigen::Vector3d& _position,
                          const Eigen::Vector3d& _size)
{
  dart::dynamics::SkeletonPtr cube = createCube(_position, _size, default_cube_mass);
  world_->addSkeleton(cube);
  objects_.push_back(cube);
}

void MultiLinkDI::initVisualization()
{
  int argc = 0;
  char* argv[] = {};
  glutInit(&argc, argv);
  window_.initWindow(640, 480, "multiLinkDI");
  glutMainLoop();
}

void MultiLinkDI::updateVisualization()
{
  window_.draw();
}

bool MultiLinkDI::isCollided(const Eigen::VectorXd& config)
{
  Eigen::VectorXd originalConfig = getConfiguration();
  setConfiguration(config);
  auto collisionEngine = world_->getConstraintSolver()->getCollisionDetector();
  auto collisionGroup = collisionEngine->createCollisionGroup(di_.get());
  auto collisionGroup2 = collisionEngine->createCollisionGroup();
  for(std::vector<dart::dynamics::SkeletonPtr>::iterator it = objects_.begin();
      it != objects_.end();it++)
  {
      dart::dynamics::SkeletonPtr obj = (*it);
      collisionGroup2->addShapeFramesOf(obj.get());
  }

  dart::collision::CollisionOption option;
  dart::collision::CollisionResult result;
  bool collision = collisionGroup->collide(collisionGroup2.get(), option, &result);

  setConfiguration(originalConfig);
  return collision;
}

Eigen::Vector3d MultiLinkDI::getEndEffectorPos(const Eigen::VectorXd& config)
{
    Eigen::VectorXd originalConfig = getConfiguration();
    setConfiguration(config);

    Eigen::Vector3d pos = getEndEffectorPos();

    setConfiguration(originalConfig);
    return pos;
}

Eigen::Vector3d MultiLinkDI::getEndEffectorPos()
{
    dart::dynamics::BodyNode* node = bodyNodes_[num_of_links_-1];
    Eigen::Vector3d pos = node->getWorldTransform() * basePos_;
    return pos;
}

void MultiLinkDI::setGeometry(const BodyNodePtr& bn)
{
  // Create a BoxShape to be used for both visualization and collision checking
  std::shared_ptr<BoxShape> box(new BoxShape(
      Eigen::Vector3d(default_width, default_depth, default_height)));

  // Create a shape node for visualization and collision checking
  auto shapeNode = bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);
  shapeNode->getVisualAspect()->setColor(dart::Color::Red());

  // Set the location of the shape node
  Eigen::Isometry3d box_tf(Eigen::Isometry3d::Identity());
  Eigen::Vector3d center = Eigen::Vector3d(default_width / 2.0, 0, 0);
  box_tf.translation() = center;
  shapeNode->setRelativeTransform(box_tf);

  // Move the center of mass to the center of the object
  bn->setLocalCOM(center);
}

SkeletonPtr MultiLinkDI::createCube(const Eigen::Vector3d& _position,
                         const Eigen::Vector3d& _size,
                         double _mass)
{
  dart::dynamics::SkeletonPtr newCubeSkeleton =
      dart::dynamics::Skeleton::create();

  dart::dynamics::BodyNode::Properties body;
  body.mName = "cube_link";
  body.mInertia.setMass(_mass);
  body.mInertia.setMoment(
      dart::dynamics::BoxShape::computeInertia(_size, _mass));
  dart::dynamics::ShapePtr newBoxShape(new dart::dynamics::BoxShape(_size));

  dart::dynamics::FreeJoint::Properties joint;
  joint.mName = "cube_joint";
  joint.mT_ParentBodyToJoint = Eigen::Translation3d(_position);

  auto pair
      = newCubeSkeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
        nullptr, joint, body);
  auto shapeNode = pair.second->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(newBoxShape);
  shapeNode->getVisualAspect()->setColor(dart::math::randomVector<3>(0.0, 1.0));

  return newCubeSkeleton;
}

BodyNode* MultiLinkDI::makeRootBody(const SkeletonPtr& di, const std::string& name)
{
    // Set up the properties for the Joint
    RevoluteJoint::Properties properties;
    properties.mName = name + "_joint";
    properties.mAxis = Eigen::Vector3d::UnitZ();
    properties.mT_ParentBodyToJoint.translation() =
        Eigen::Vector3d(0, 0, default_depth/2) + basePos_;
    properties.mDampingCoefficients[0] = default_damping;
    properties.mRestPositions[0] = default_rest_position;
    properties.mSpringStiffnesses[0] = default_stiffness;

    // Create a new BodyNode, attached to its parent by a RevoluteJoint
    BodyNodePtr bn = di->createJointAndBodyNodePair<RevoluteJoint>(
          nullptr, properties, BodyNode::AspectProperties(name)).second;

    // Make a shape for the Joint
    const double R = default_height / 2.0;
    const double h = default_depth;
    std::shared_ptr<CylinderShape> cyl(new CylinderShape(R, h));

    // Line up the cylinder with the Joint axis
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.linear() = dart::math::eulerXYZToMatrix(Eigen::Vector3d(0.0 * M_PI / 180.0,
                                                               0.0 * M_PI / 180.0,
                                                               0.0 * M_PI / 180.0));

    auto shapeNode = bn->createShapeNodeWith<VisualAspect>(cyl);
    shapeNode->getVisualAspect()->setColor(dart::Color::Blue());
    shapeNode->setRelativeTransform(tf);

    // Set the geometry of the Body
    setGeometry(bn);

    return bn;
}

BodyNode* MultiLinkDI::addBody(const SkeletonPtr& di, BodyNode* parent,
                               const std::string& name)
{
  // Set up the properties for the Joint
  RevoluteJoint::Properties properties;
  properties.mName = name + "_joint";
  properties.mAxis = Eigen::Vector3d::UnitZ();
  properties.mT_ParentBodyToJoint.translation() =
      Eigen::Vector3d(default_width, 0, 0);
  properties.mRestPositions[0] = default_rest_position;
  properties.mSpringStiffnesses[0] = default_stiffness;
  properties.mDampingCoefficients[0] = default_damping;

  // Create a new BodyNode, attached to its parent by a RevoluteJoint
  BodyNodePtr bn = di->createJointAndBodyNodePair<RevoluteJoint>(
        parent, properties, BodyNode::AspectProperties(name)).second;

  // Make a shape for the Joint
  const double R = default_height / 2.0;
  const double h = default_depth;
  std::shared_ptr<CylinderShape> cyl(new CylinderShape(R, h));

  // Line up the cylinder with the Joint axis
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.linear() = dart::math::eulerXYZToMatrix(Eigen::Vector3d(0.0 * M_PI / 180.0,
                                                             0.0 * M_PI / 180.0,
                                                             0.0 * M_PI / 180.0));

  auto shapeNode = bn->createShapeNodeWith<VisualAspect>(cyl);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());
  shapeNode->setRelativeTransform(tf);

  // Set the geometry of the Body
  setGeometry(bn);

  return bn;
}
