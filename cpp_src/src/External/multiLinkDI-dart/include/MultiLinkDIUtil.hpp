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
    int link_num_a = root["link_num_a"].asInt();
    int link_num_b = root["link_num_b"].asInt();
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

    Eigen::Vector3d diAPos;
    Json::Value diAPosVal = root["di_a_pos"];
    for(int i=0;i<diAPosVal.size();i++)
    {
      diAPos[i] = diAPosVal[i].asDouble();
    }
    Eigen::Vector3d diBPos;
    Json::Value diBPosVal = root["di_b_pos"];
    for(int i=0;i<diBPosVal.size();i++)
    {
      diBPos[i] = diBPosVal[i].asDouble();
    }

    di = std::make_shared<MultiLinkDI>(link_num_a, link_num_b,
                                       diAPos, diBPos);
    Json::Value links = root["links"];
    dart::dynamics::BodyNode* bn = nullptr;
    for(int i=0;i<link_num_a;i++)
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
            bn = di->makeRootBody(MultiLinkDI::sideType::A,
                                  translate(links[i].get("orientation", "").asString()),
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
            bn = di->addBody(bn, MultiLinkDI::sideType::A,
                             translate(links[i].get("orientation", "").asString()),
                             links[i].get("name", "").asString(),
                             links[i].get("width", 0).asDouble(),
                             links[i].get("height", 0).asDouble(),
                             links[i].get("depth",0).asDouble(),
                             relativeEuler, relativeTrans,
                             links[i].get("initConfig",0).asDouble());
        }
    }

    for(int i=link_num_a;i<link_num;i++)
    {
        Eigen::Vector3d relativeTrans;
        Eigen::Vector3d relativeEuler;
        Json::Value eulerVal = links[i].get("euler", 0);
        for(int j=0;j<eulerVal.size();j++)\
        {
            relativeEuler[j] = eulerVal[j].asDouble() * M_PI/180;
        }
        if(i==link_num_a)
        {
            bn = di->makeRootBody(MultiLinkDI::sideType::B,
                                  translate(links[i].get("orientation", "").asString()),
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
            bn = di->addBody(bn, MultiLinkDI::sideType::B,
                             translate(links[i].get("orientation", "").asString()),
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
          size_t dim = di->getNumOfTotalLinks()*2;
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
