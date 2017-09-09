#include <sstream>
#include <string>
#include "MultiLinkDI.hpp"
#include "MultiLinkDIWindow.hpp"
#include <dart/collision/CollisionFilter.hpp>

using namespace dart::simulation;
using namespace dart::dynamics;

MultiLinkDI::MultiLinkDI(const unsigned int num_of_links, Eigen::Vector3d& pos)
    : basePos_(pos), homeConfig_(num_of_links, 0.0)
{
  num_of_links_ = num_of_links;

  world_ = std::make_shared<dart::simulation::World>();
  assert(world_ != nullptr);

  Eigen::Vector3d gravity(0.0, 0.0, 0.0);
  world_->setGravity(gravity);

  di_ = Skeleton::create("di");
  world_->addSkeleton(di_);
  di_->setSelfCollisionCheck(true);
  di_->setAdjacentBodyCheck(false);

  window_.setWorld(world_);
  window_.setMultiLinkDI(this);

  plane_ = nullptr;
}

MultiLinkDI::~MultiLinkDI()
{

}

void MultiLinkDI::addPlane(const Eigen::Vector3d & pos, const Eigen::Vector3d & size )
{
  plane_ = createCube(pos, size, default_cube_mass);
  world_->addSkeleton(plane_);
}

void MultiLinkDI::setDofConfiguration(unsigned int idx, double config)
{
  di_->getDof(idx)->setPosition(config + homeConfig_[idx]);
}

void MultiLinkDI::setConfiguration(const Eigen::VectorXd& config)
{
  for(unsigned int idx=0;idx<num_of_links_;idx++)
  {
    di_->getDof(idx)->setPosition(config[idx]+homeConfig_[idx]);
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
  initLineSegment();
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

  auto filter = std::make_shared<dart::collision::BodyNodeCollisionFilter>();
  dart::collision::CollisionOption option(false, 1u, nullptr);
  option.collisionFilter = filter;

  if (true==collisionGroup->collide(option))
  {
      return true;
  }
  auto collisionGroup2 = collisionEngine->createCollisionGroup();
  for(std::vector<dart::dynamics::SkeletonPtr>::iterator it = objects_.begin();
      it != objects_.end();it++)
  {
      dart::dynamics::SkeletonPtr obj = (*it);
      collisionGroup2->addShapeFramesOf(obj.get());
  }

  dart::collision::CollisionOption option2;
  dart::collision::CollisionResult result;
  bool collision = collisionGroup->collide(collisionGroup2.get(), option2, &result);

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
    dart::dynamics::BodyNode* node = bodyNodes_.back();
    Eigen::Isometry3d trans = node->getWorldTransform() * node->getRelativeTransform();
    Eigen::Vector3d pos = trans * basePos_;
    return pos;
}

void MultiLinkDI::setGeometry(const BodyNodePtr& bn, const orientationType type,
                              const double width,
                              const double height, const double depth)
{

  // Create a BoxShape to be used for both visualization and collision checking
  std::shared_ptr<BoxShape> box(new BoxShape(
      Eigen::Vector3d(width, depth, height)));

  // Create a shape node for visualization and collision checking
  auto shapeNode = bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);
  shapeNode->getVisualAspect()->setColor(dart::Color::Red());

  // Set the location of the shape node
  Eigen::Isometry3d box_tf(Eigen::Isometry3d::Identity());
  Eigen::Vector3d center;
  switch(type)
  {
  case Z:
    center = Eigen::Vector3d(width/2.0, 0, 0);
    break;
  case Y:
    center = Eigen::Vector3d(0, 0, height/2.0);
    break;
  case X:
    center = Eigen::Vector3d(0, 0, height/2.0);
    break;
  }
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

BodyNode* MultiLinkDI::makeRootBody(const SkeletonPtr& di, const orientationType type,
                                    const std::string& name,
                                    const double width, const double height,
                                    const double depth, const Eigen::Vector3d& relativeEuler,
                                    const double initConfig)
{
    // Set up the properties for the Joint
    RevoluteJoint::Properties properties;
    properties.mName = name + "_joint";
    switch(type)
    {
    case X:
        properties.mAxis = Eigen::Vector3d::UnitX();
        properties.mT_ParentBodyToJoint.translation() =
                Eigen::Vector3d(0, 0, depth/2) + basePos_;
        break;
    case Y:
        properties.mAxis = Eigen::Vector3d::UnitY();
        properties.mT_ParentBodyToJoint.translation() =
                Eigen::Vector3d(0, 0, width/2) + basePos_;
        break;
    case Z:
        properties.mAxis = Eigen::Vector3d::UnitZ();
        properties.mT_ParentBodyToJoint.translation() =
                Eigen::Vector3d(0, 0, height/2) + basePos_;
        break;
    }


    properties.mDampingCoefficients[0] = default_damping;
    properties.mRestPositions[0] = default_rest_position;
    properties.mSpringStiffnesses[0] = default_stiffness;

    // Create a new BodyNode, attached to its parent by a RevoluteJoint
    BodyNodePtr bn = di->createJointAndBodyNodePair<RevoluteJoint>(
          nullptr, properties, BodyNode::AspectProperties(name)).second;

    // Make a shape for the Joint
    double R = 0.0;
    double h = 0.0;

    // Line up the cylinder with the Joint axis
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    switch(type)
    {
    case Z:
        R = depth / 2.0; h = height;
        tf.linear() = dart::math::eulerXYZToMatrix(relativeEuler);
        break;
    case X:
        R = depth / 2.0; h = width;
        tf.linear() = dart::math::eulerXYZToMatrix(relativeEuler);
        break;

    case Y:
        R = width / 2,0; h = depth;
        tf.linear() = dart::math::eulerXYZToMatrix(relativeEuler);
    }
    std::shared_ptr<CylinderShape> cyl(new CylinderShape(R, h));

    auto shapeNode = bn->createShapeNodeWith<VisualAspect>(cyl);
    shapeNode->getVisualAspect()->setColor(dart::Color::Red());
    shapeNode->setRelativeTransform(tf);

    // Set the geometry of the Body
    setGeometry(bn, type, width, height, depth);

    homeConfig_[bodyNodes_.size()] = initConfig;
    bodyNodes_.push_back(bn);
    return bn;
}

BodyNode* MultiLinkDI::addBody(const SkeletonPtr& di, BodyNode* parent,
                               const orientationType type,
                               const std::string& name,
                               const double width, const double height,
                               const double depth,
                               const Eigen::Vector3d& relativeEuler,
                               const Eigen::Vector3d& relativeTrans,
                               const double initConfig)
{
  // Set up the properties for the Joint
  RevoluteJoint::Properties properties;
  properties.mName = name + "_joint";
  switch(type)
  {
  case X:
      properties.mAxis = Eigen::Vector3d::UnitX();
      break;
  case Y:
      properties.mAxis = Eigen::Vector3d::UnitY();
      break;
  case Z:
      properties.mAxis = Eigen::Vector3d::UnitZ();
      break;
  }
  properties.mT_ParentBodyToJoint.translation() = relativeTrans;
  properties.mRestPositions[0] = default_rest_position;
  properties.mSpringStiffnesses[0] = default_stiffness;
  properties.mDampingCoefficients[0] = default_damping;

  // Create a new BodyNode, attached to its parent by a RevoluteJoint
  BodyNodePtr bn = di->createJointAndBodyNodePair<RevoluteJoint>(
        parent, properties, BodyNode::AspectProperties(name)).second;

  // Make a shape for the Joint
  double R = 0.0;
  double h = 0.0;

  // Line up the cylinder with the Joint axis
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  switch(type)
  {
  case Z:
      R = depth / 2.0; h = height;
      tf.linear() = dart::math::eulerXYZToMatrix(relativeEuler);
      break;
  case X:
      R = depth / 2.0; h = width;
      tf.linear() = dart::math::eulerXYZToMatrix(relativeEuler);
      break;
  case Y:
      R = width / 2,0; h = depth;
      tf.linear() = dart::math::eulerXYZToMatrix(relativeEuler);
  }
  std::shared_ptr<CylinderShape> cyl(new CylinderShape(R, h));

  auto shapeNode = bn->createShapeNodeWith<VisualAspect>(cyl);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());
  shapeNode->setRelativeTransform(tf);

  // Set the geometry of the Body
  setGeometry(bn, type, width, height, depth);
  homeConfig_[bodyNodes_.size()] = initConfig;
  bodyNodes_.push_back(bn);

  return bn;
}

void MultiLinkDI::initLineSegment()
{
    if(waypoints_.size() == 0)
    {
        return;
    }

   uint dim = num_of_links_;
   for(size_t idx=0;idx<waypoints_.size()-1;idx++)
   {
       Eigen::VectorXd prevConfig = waypoints_[idx].head(num_of_links_);
       Eigen::VectorXd nextConfig = waypoints_[idx+1].head(num_of_links_);

       /*
       Eigen::VectorXd deltaConfig = nextConfig - prevConfig;
       for(double i=0.0;
           i <= 1.0; i+= getResolutionSize())
       {
           Eigen::VectorXd newConfig = prevConfig + i * deltaConfig;
           Eigen::VectorXd newConfigNext = newConfig + getResolutionSize() * deltaConfig;

           Eigen::Vector3d newPos = getEndEffectorPos(newConfig);
           Eigen::Vector3d newPosNext = getEndEffectorPos(newConfigNext);

           //std::cout << "new " << newPosNext << std::endl;

           dart::dynamics::SimpleFramePtr lineFrame =
                   std::make_shared<dart::dynamics::SimpleFrame>(
                     dart::dynamics::Frame::World());

           dart::dynamics::LineSegmentShapePtr lineSeg =
                   std::make_shared<dart::dynamics::LineSegmentShape>(newPos, newPosNext, 3.0);
           lineSeg->addDataVariance(dart::dynamics::Shape::DYNAMIC_VERTICES);

           lineFrame->setShape(lineSeg);
           lineFrame->createVisualAspect();
           lineFrame->getVisualAspect()->setColor(default_force_line_color);
           world_->addSimpleFrame(lineFrame);
       }*/
       dart::dynamics::SimpleFramePtr lineFrame =
               std::make_shared<dart::dynamics::SimpleFrame>(
                 dart::dynamics::Frame::World());
       Eigen::Vector3d newPos = getEndEffectorPos(prevConfig);
       Eigen::Vector3d newPosNext = getEndEffectorPos(nextConfig);

       dart::dynamics::LineSegmentShapePtr lineSeg =
               std::make_shared<dart::dynamics::LineSegmentShape>(newPos, newPosNext, 3.0);
       lineSeg->addDataVariance(dart::dynamics::Shape::DYNAMIC_VERTICES);

       lineFrame->setShape(lineSeg);
       lineFrame->createVisualAspect();
       lineFrame->getVisualAspect()->setColor(default_force_line_color);
       world_->addSimpleFrame(lineFrame);

   }
}
