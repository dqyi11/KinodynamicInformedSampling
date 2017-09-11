#ifndef MULTI_LINK_DI_CREATE_HPP_
#define MULTI_LINK_DI_CREATE_HPP_

#include <string>
#include <fstream>
#include "jsoncpp/json/json.h"
#include "MultiLinkDI.hpp"

MultiLinkDI::orientationType translate(std::string str)
{
    if(str=="X" or str=="x")
    {
        return MultiLinkDI::orientationType::X;
    }
    else if(str=="Y" or str=="y")
    {
        return MultiLinkDI::orientationType::Y;
    }
    else if(str=="Z" or str=="z")
    {
        return MultiLinkDI::orientationType::Z;
    }

    return MultiLinkDI::orientationType::Z;
}

std::shared_ptr<MultiLinkDI> createMultiLinkDI( std::string problemFilename )
{
   std::shared_ptr<MultiLinkDI> di = nullptr;

  Json::Reader reader;
  Json::Value root;
  std::ifstream problemFile(problemFilename, std::ifstream::binary);
  bool parseSuccess = reader.parse(problemFile, root, false);
  if(parseSuccess)
  {
    int link_num = root["link_num"].asInt();
    int dimension = link_num * 2;

    Eigen::VectorXd startConfig(dimension);
    Eigen::VectorXd goalConfig(dimension);
    Json::Value startVal = root["start"];
    Json::Value goalVal = root["goal"];
    for(int i=0;i<startVal.size();i++)
    {
      startConfig[i] = startVal[i].asDouble();
      goalConfig[i] = goalVal[i].asDouble();
    }

    Eigen::Vector3d diPos;
    Json::Value diPosVal = root["di_pos"];
    for(int i=0;i<diPosVal.size();i++)
    {
      diPos[i] = diPosVal[i].asDouble();
    }

    di = std::make_shared<MultiLinkDI>(link_num, diPos);
    Json::Value links = root["links"];
    dart::dynamics::BodyNode* bn = nullptr;
    for(int i=0;i<links.size();i++)
    {
        Eigen::Vector3d relativeTrans;
        Eigen::Vector3d relativeEuler;
        Json::Value eulerVal = links[i].get("euler", 0);
        for(int j=0;j<eulerVal.size();j++)\
        {
            relativeEuler[j] = eulerVal[j].asDouble() * M_PI/180;
        }
        if(i==0)
        {
            bn = di->makeRootBody(translate(links[i].get("orientation", "").asString()),
                                  links[i].get("name", "").asString(),
                                  links[i].get("width", 0).asDouble(),
                                  links[i].get("height", 0).asDouble(),
                                  links[i].get("depth",0).asDouble(),
                                  relativeEuler,
                                  links[i].get("initConfig",0).asDouble());
        }
        else{
            Json::Value transVal = links[i].get("trans", 0);
            for(int j=0;j<transVal.size();j++)\
            {
                relativeTrans[j] = transVal[j].asDouble();
            }
            bn = di->addBody(bn, translate(links[i].get("orientation", "").asString()),
                             links[i].get("name", "").asString(),
                             links[i].get("width", 0).asDouble(),
                             links[i].get("height", 0).asDouble(),
                             links[i].get("depth",0).asDouble(),
                             relativeEuler, relativeTrans,
                             links[i].get("initConfig",0).asDouble());
        }
    }

    Json::Value plane = root["plane"];
    if(plane.isNull() == false)
    {
        for(int i=0;i<plane.size();i++)
        {
            Eigen::Vector3d plane_pos, plane_size;
            Json::Value planePosVal = plane[i].get("plane_pos", 0);
            Json::Value planeSizeVal = plane[i].get("plane_size", 0);
            for(int j=0;j<planePosVal.size();j++)
            {
                plane_pos[j] = planePosVal[j].asDouble();
            }
            for(int j=0;j<planeSizeVal.size();j++)
            {
                plane_size[j] = planeSizeVal[j].asDouble();
            }
            di->addPlane(plane_pos, plane_size);
        }
    }

    Json::Value obstacles = root["obstacles"];
    if(obstacles.isNull()==false)
    {
        for(int i=0;i<obstacles.size();i++)
        {
          std::string type = obstacles[i].get("type","").asString();
          if(type == "hypercube")
          {
            Json::Value centerVal, sizeVal;

            Eigen::Vector3d obs_center;
            Eigen::Vector3d obs_size;

            centerVal = obstacles[i].get("center", 0);
            for(int j=0;j<centerVal.size();j++)
            {
              obs_center[j] = centerVal[j].asDouble();
            }
            sizeVal = obstacles[i].get("size",0);
            for(int j=0;j<sizeVal.size();j++)
            {
              obs_size[j] = sizeVal[j].asDouble();
            }
            di->addCube(obs_center, obs_size);
          }
        }
    }

    di->setConfiguration(startConfig);
    std::cout << "START COLLIDE " << di->isCollided(startConfig) << std::endl;
    std::cout << "GOAL COLLIDE " << di->isCollided(goalConfig) << std::endl;

  }
  problemFile.close();

  return di;
}

void loadMultiLinkDIPath( std::shared_ptr<MultiLinkDI> di, std::string pathFilename )
{
    std::ifstream pathFile(pathFilename, std::ios::in);

    if(pathFile.is_open())
    {
        std::string stateStr;
        while( std::getline(pathFile, stateStr) )
        {
          size_t dim = di->getNumOfLinks()*2;
          Eigen::VectorXd waypointVec(dim);
          std::stringstream iss(stateStr);
          int dimIdx = 0;
          double val = 0;
          while( iss >> val && dimIdx < dim)
          {
            waypointVec[dimIdx] = val;
            dimIdx ++;
          }

          di->addWaypoint( waypointVec );
        }
    }

    pathFile.close();
}

#endif // MULTI_LINK_DI_CREATE_HPP_
