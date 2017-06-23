#ifndef CREATE_OBSTACLES_H_
#define CREATE_OBSTACLES_H_

#include "Dimt/Params.h"
#include "OmplWrappers/ValidityChecker.h"
#include "jsoncpp/json/json.h"

ompl::base::StateValidityCheckerPtr createStateValidityChecker(ompl::base::SpaceInformationPtr si,
                                                               std::string filename)
{
    ValidityChecker* pVC = new ValidityChecker(si);

    Json::Reader reader;
    Json::Value root;
    std::ifstream file(filename, std::ifstream::binary);
    bool parseSuccess = reader.parse(file, root, false);

    if(parseSuccess)
    {
       for(size_t i=0;i<root.size();i++)
       {
          std::string A = root[i];
       }
    }

    // obstacles
    Eigen::VectorXd obs1_center(param.dimensions / 2);
    Eigen::VectorXd obs1_radius(param.dimensions / 2);
    obs1_center << 0.0, 0.0;
    obs1_radius << 1.0, 1.0;
    pVC->addHypercubeObstacle( obs1_center, obs1_radius );

    Eigen::VectorXd obs2_center(param.dimensions / 2);
    Eigen::VectorXd obs2_radius(param.dimensions / 2);
    obs2_center << 2.0, 1.2;
    obs2_radius << 0.5, 0.5;
    pVC->addHypercubeObstacle( obs2_center, obs2_radius );

    Eigen::VectorXd obs3_center(param.dimensions / 2);
    Eigen::VectorXd obs3_radius(param.dimensions / 2);
    obs3_center << -2.0, 1.2;
    obs3_radius << 0.5, 0.5;
    pVC->addHypercubeObstacle( obs3_center, obs3_radius );

    Eigen::VectorXd obs4_center(param.dimensions / 2);
    Eigen::VectorXd obs4_radius(param.dimensions / 2);
    obs4_center << -2.0, -1.2;
    obs4_radius << 0.5, 0.5;
    pVC->addHypercubeObstacle( obs4_center, obs4_radius );

    Eigen::VectorXd obs5_center(param.dimensions / 2);
    Eigen::VectorXd obs5_radius(param.dimensions / 2);
    obs5_center << 2.0, -1.2;
    obs5_radius << 0.5, 0.5;
    pVC->addHypercubeObstacle( obs5_center, obs5_radius );

    return ompl::base::StateValidityCheckerPtr(pVC);
}

#endif // CREATE_OBSTACLES_H_
