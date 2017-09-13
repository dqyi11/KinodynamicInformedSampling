#ifndef CREATE_OBSTACLES_H_
#define CREATE_OBSTACLES_H_

#include "Dimt/Params.h"
#include "OmplWrappers/ValidityChecker.h"
#include "jsoncpp/json/json.h"
#include "HerbValidityChecker.hpp"

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
       Json::Value obstacles = root["obstacles"];
       for(int i=0;i<obstacles.size();i++)
       {
          std::string type = obstacles[i].get("type","").asString();
          Json::Value center, radius;

          Eigen::VectorXd obs_center(param.dimensions / 2);
          Eigen::VectorXd obs_radius(param.dimensions / 2);

          center = obstacles[i].get("center", 0);
          for(int j=0;j<center.size();j++)
          {
            obs_center[j] = center[j].asDouble();
          }
          radius = obstacles[i].get("radius",0);
          for(int j=0;j<radius.size();j++)
          {
            obs_radius[j] = radius[j].asDouble();
          }
          std::cout << "Add obstacle ";

          if(type == "hypercube")
          {
             std::cout << type << " ";
             std::cout << "center " << obs_center;
             std::cout << "radius " << obs_radius;
             pVC->addHypercubeObstacle( obs_center, obs_radius );
          }
          else if(type == "nsphere")
          {
             std::cout << type << " ";
             std::cout << "center " << obs_center;
             std::cout << "radius " << obs_radius;
             pVC->addNSphereObstacle( obs_center, obs_radius[0] );
          }
          std::cout << std::endl;         
       }
    }

    file.close();

    return ompl::base::StateValidityCheckerPtr(pVC);
}

ompl::base::StateValidityCheckerPtr 
createHerbStateValidityChecker(ompl::base::SpaceInformationPtr si,
                               std::shared_ptr<herb::Herb> herb,
                               std::shared_ptr<aikido::statespace::dart::MetaSkeletonStateSpace> space,
                               aikido::constraint::NonCollidingPtr nonColliding)
{
    ompl::base::StateValidityCheckerPtr pVC = 
        std::make_shared<HerbValidityChecker>(si, herb, space, nonColliding);
    return pVC;
}

#endif // CREATE_OBSTACLES_H_
