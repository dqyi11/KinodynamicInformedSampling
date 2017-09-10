#ifndef MULTI_LINK_DI_
#define MULTI_LINK_DI_

#include <dart/dart.hpp>
#include "MultiLinkDIWindow.hpp"

constexpr double default_height = 0.1; // m
constexpr double default_width = 1.0;  // m
constexpr double default_depth = 0.1;  // m

constexpr double default_distance_to_ground = 0.05;

constexpr double default_torque = 15.0; // N-m
constexpr double default_force =  15.0; // N
constexpr int default_countdown = 200;  // Number of timesteps for applying force

constexpr double default_rest_position = 0.0;
constexpr double delta_rest_position = 10.0 * M_PI / 180.0;

constexpr double default_stiffness = 0.0;
constexpr double delta_stiffness = 10;

constexpr double default_damping = 5.0;
constexpr double delta_damping = 1.0;

constexpr double default_resolution_size = 0.02;
constexpr double default_cube_mass = 1.0;
const static Eigen::Vector4d default_force_line_color = Eigen::Vector4d(1.0, 0.63, 0.0, 1.0);
const static Eigen::Vector3d default_plane_pos = Eigen::Vector3d(0.0, 0.0, 0.0);
const static Eigen::Vector3d default_plane_size = Eigen::Vector3d(8.0, 8.0, 0.02);

class MultiLinkDI
{
public:
    typedef enum  { X = 0, Y, Z } orientationType;
    MultiLinkDI(const unsigned int num_of_links, const Eigen::Vector3d& pos);
    virtual ~MultiLinkDI();

    Eigen::VectorXd getConfiguration();
    void setConfiguration(const Eigen::VectorXd& config);
    void setDofConfiguration(unsigned int idx, double config);

    void addPlane(const Eigen::Vector3d & pos = default_plane_pos,
                  const Eigen::Vector3d & size = default_plane_size );
    void addCube(const Eigen::Vector3d& _poisition,
                 const Eigen::Vector3d& _size);

    bool isCollided(const Eigen::VectorXd& pos);

    void initVisualization();
    void updateVisualization();

    Eigen::Vector3d getEndEffectorPos(const Eigen::VectorXd& config);
    Eigen::Vector3d getEndEffectorPos();

    unsigned int getNumOfLinks() { return num_of_links_; }

    void addWaypoint( Eigen::VectorXd waypoint ) { waypoints_.push_back(waypoint); }
    void clearWaypoints() { waypoints_.clear(); }
    size_t getWaypointNum() { return waypoints_.size(); }
    Eigen::VectorXd getWaypoint(size_t idx) { return waypoints_[idx]; }

    double getResolutionSize() { return default_resolution_size; }

    dart::dynamics::BodyNode* makeRootBody(const orientationType type,
                                           const std::string& name,
                                           const double width = default_width,
                                           const double height = default_height,
                                           const double depth = default_depth,
                                           const Eigen::Vector3d& relativeEuler = Eigen::Vector3d(),
                                           const double initConfig = 0.0)
    {
        return makeRootBody(di_, type, name, width, height, depth, relativeEuler, initConfig);
    }

    dart::dynamics::BodyNode* addBody(dart::dynamics::BodyNode* parent,
                                      const orientationType type,
                                      const std::string& name,
                                      const double width = default_width,
                                      const double height = default_height,
                                      const double depth = default_depth,
                                      const Eigen::Vector3d& relativeEuler = Eigen::Vector3d(),
                                      const Eigen::Vector3d& relativeTrans = Eigen::Vector3d(),
                                      const double initConfig = 0.0)
    {
        return addBody(di_, parent, type, name, width, height, depth, relativeEuler,
                       relativeTrans, initConfig);
    }

    void setHomeConfig(unsigned int idx, double config)
    {
        homeConfig_[idx] = config;
    }

    void setHomeConfig(const Eigen::VectorXd& homeConfig)
    {
        for(uint i=0;i<num_of_links_;i++)
        {
            homeConfig_[i] = homeConfig[i];
        }
    }

    void updateHomeConfig()
    {
        for(uint i=0;i<num_of_links_;i++)
        {
            di_->getDof(i)->setPosition(homeConfig_[i]);
        }
    }

protected:
    void setGeometry(const dart::dynamics::BodyNodePtr& bn, const orientationType type,
                     const double width, const double height, const double depth);

    dart::dynamics::SkeletonPtr createCube(const Eigen::Vector3d& _position,
                                           const Eigen::Vector3d& _size,
                                           double _mass);

    dart::dynamics::BodyNode* makeRootBody(const dart::dynamics::SkeletonPtr& di,
                                           const orientationType type,
                                           const std::string& name,
                                           const double width, const double height,
                                           const double depth, const Eigen::Vector3d& relativeEuler,
                                           const double initConfig);

    dart::dynamics::BodyNode* addBody(const dart::dynamics::SkeletonPtr& di,
                                      dart::dynamics::BodyNode* parent,
                                      const orientationType type,
                                      const std::string& name,
                                      const double width, const double height,
                                      const double depth, const Eigen::Vector3d& relativeEuler,
                                      const Eigen::Vector3d& relativeTrans,
                                      const double initConfig);

    void initLineSegment();

    unsigned int num_of_links_;

    dart::simulation::WorldPtr world_;
    dart::dynamics::SkeletonPtr di_;
    dart::dynamics::SkeletonPtr plane_;
    std::vector<dart::dynamics::BodyNode*> bodyNodes_;

    Eigen::Vector3d basePos_;
    std::vector<double> homeConfig_;

    std::vector<dart::dynamics::SkeletonPtr> objects_;

    std::vector<Eigen::VectorXd> waypoints_;
    std::vector<dart::dynamics::LineSegmentShapePtr> lineSegments_;

    MultiLinkDIWindow window_;
};

#endif // MULTI_LINK_DI_
