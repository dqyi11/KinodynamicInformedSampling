#include <Eigen/Dense>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/constraint/TSR.hpp>
#include <aikido/constraint/NonColliding.hpp>
#include <dart/dart.hpp>
#include <fstream>
#include <iostream>
#include <boost/program_options.hpp>
#include <libherb/herb.hpp>
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <aikido/io/CatkinResourceRetriever.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <dart/collision/CollisionDetector.hpp>
#include <dart/collision/CollisionOption.hpp>
#include <dart/collision/CollisionGroup.hpp>
#include <aikido/perception/AprilTagsModule.hpp>
#include <aikido/perception/YamlAprilTagsDatabase.hpp>
#include <pr_tsr/glass.hpp>


namespace po = boost::program_options;

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

static const std::string topicName("dart_markers");
static const std::string markerTopicName("/apriltags/marker_array");
static const std::string configDataURI("package://pr_ordata/data/objects/tag_data.json");
static const std::string herbFrameName("herb_frame");
static const std::string tableName("table127");
static const std::string glassName("plastic_glass1");
static const std::string baseFrameName("map");

static const double planningTimeout{5.};
static const double detectionTimeout{5.};
static const int tsrPlanningTrials{100};
static const double tsrPlanningTimeout{30.};

const dart::dynamics::SkeletonPtr makeBodyFromURDF(const std::string& uri,
  const Eigen::Isometry3d& transform)
{
  // Resolves package:// URIs by emulating the behavior of 'catkin_find'.
  const auto resourceRetriever
    = std::make_shared<aikido::io::CatkinResourceRetriever>();

  dart::utils::DartLoader urdfLoader;
  const dart::dynamics::SkeletonPtr skeleton = urdfLoader.parseSkeleton(
    uri, resourceRetriever);

  if (!skeleton) {
    throw std::runtime_error("unable to load '" + uri + "'");
  }

  dynamic_cast<dart::dynamics::FreeJoint*>(skeleton->getJoint(0))->setTransform(transform);

  return skeleton;
}

/// Quick hack to extract the Eigen::VectorXd configuration of the last waypoint
/// in the trajectory. There *has* to be a better way to do this.
///
/// \param[in] traj Trajectory to get the last configuration of
/// \param[in] space Trajectory's state space
/// \param[out] configuration The configuration of the last waypoint
void getLastWaypointConfiguration(
  aikido::trajectory::InterpolatedPtr traj,
  aikido::statespace::dart::MetaSkeletonStateSpacePtr space,
  Eigen::VectorXd& configuration)
{
  using aikido::statespace::CartesianProduct;
  CartesianProduct::State* state =
    const_cast<CartesianProduct::State*>(
      static_cast<const CartesianProduct::State*>(
        traj->getWaypoint(traj->getNumWaypoints()-1)));
  space->convertStateToPositions(state, configuration);
}

int main(int argc, char** argv)
{
  // Default options for flags
  bool herbSim = true;
  bool perceptionSim = true;
  bool withObstacle = false;

  std::string pathFilename = "";

  po::options_description po_desc("Pantry-Loader Options");
  po_desc.add_options()
    ("help", "produce help message")
    ("herbsim,h", po::bool_switch(&herbSim), "Run HERB in simulation")
    ("perceptionsim,p", po::bool_switch(&perceptionSim), "Run perception in simulation")
    ("obstacle,o", po::bool_switch(&withObstacle), "With obstacles")
    ("path", po::value(&pathFilename), "path filename")
  ;


  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, po_desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << po_desc << std::endl;
    return 1;
  }

  herbSim = true; perceptionSim = true;
  std::cout << "loading file " << pathFilename << std::endl;

  std::cout << "Starting ROS node." << std::endl;
  ros::init(argc, argv, "load_herb");

  // Load HERB either in simulation or real based on arguments
  std::cout << "Load Herb" << std::endl;
  herb::Herb robot(herbSim);
  SkeletonPtr robotSkeleton = robot.getSkeleton();

  // Start the RViz viewer.
  std::cout << "Starting viewer. Please subscribe to the '" << topicName
            << "' InteractiveMarker topic in RViz." << std::endl;
  aikido::rviz::InteractiveMarkerViewer viewer(topicName,baseFrameName);

  // Add Herb to the viewer.
  viewer.addSkeleton(robotSkeleton);
  dart::simulation::WorldPtr env(new dart::simulation::World);

  SkeletonPtr table, glass;
  Eigen::Isometry3d tablePose, glassPose, glassGoalPose;

  if (perceptionSim && withObstacle) {

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

    // Add all objects to viewer
    viewer.addSkeleton(table);
    viewer.addSkeleton(glass);

  }
  
  viewer.setAutoUpdate(true);

  auto leftArmSpace =
      std::make_shared<MetaSkeletonStateSpace>(robot.getLeftArm());
  auto rightArmSpace =
      std::make_shared<MetaSkeletonStateSpace>(robot.getRightArm());
  auto hand = robot.getRightHand()->getBodyNode();

  // Set up start and goal configurations
  Eigen::VectorXd leftGoalConfiguration(7);
  leftGoalConfiguration << 0.64, -1.50, 0.26, 1.96, 1.16, 0.87, 1.43; // relaxed home
  robot.setConfiguration(leftArmSpace, leftGoalConfiguration);

  Eigen::VectorXd rightStartConfiguration(7);
  rightStartConfiguration << 3.68, -1.90, 0.00, 2.20, 0.00, 0.00, 0.00;  // home
  robot.setConfiguration(rightArmSpace, rightStartConfiguration);

  Eigen::VectorXd rightGoalConfiguration(7);
  rightGoalConfiguration << 5.65, -1.50, -0.26, 1.96, -1.15, 0.87, -1.43;  // relaxed home

  int pause; // for debugging only

  std::cout << "Pausing (enter a number to continue): ";
  std::cin >> pause;

  auto traj = std::make_shared<aikido::trajectory::Interpolated>(rightArmSpace,
              std::make_shared<aikido::statespace::GeodesicInterpolator>(rightArmSpace));

  std::ifstream pathIO(pathFilename, std::ios::in);
  if(pathIO.is_open())
  {
    size_t dim = 7;
    double t = 0.0;
    std::string stateStr;

    Eigen::VectorXd startConfig(dim);
    while( std::getline(pathIO, stateStr) )
    {
      Eigen::VectorXd waypointVec(dim);
      std::stringstream iss(stateStr);
      int dimIdx = 0;
      double val = 0;
      while( iss >> val && dimIdx < dim )
      {
        waypointVec[dimIdx] = val;
        dimIdx ++;
      }

      if(t==0.0)
      {
        startConfig = waypointVec;
      }

      Eigen::VectorXd posVec = waypointVec.segment(0,7);
 
      auto tmpState = rightArmSpace->createState();
      // convert EigenVec to ompl state
      rightArmSpace->convertPositionsToState(posVec, tmpState);
      traj->addWaypoint(t, tmpState);
      std::cout << "ADD POINT ";
      rightArmSpace->print(tmpState, std::cout);
      std::cout << " at " << t << std::endl;
      t += 0.1;
    }

    robot.setConfiguration(rightArmSpace, startConfig);


  }
  
  std::cout << "Ready to execute trajectories (enter a number to continue): ";
  std::cin >> pause;
  std::cout << "point num " << traj->getNumWaypoints() << std::endl;
  robot.executeTrajectory(traj).wait();

  std::cout << "Press <Ctrl> + C to exit." << std::endl;
  ros::spin();
  return 0;

}
