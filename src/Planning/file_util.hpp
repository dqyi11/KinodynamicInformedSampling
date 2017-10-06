#ifndef FILE_UTIL_HPP_
#define FILE_UTIL_HPP_

#include <string>
#include <ompl/base/State.h>
#include <ompl/base/Path.h>
#include <ompl/geometric/PathGeometric.h>
#include "Dimt/DoubleIntegratorMinimumTime.h"
#include "Dimt/Params.h"

void dumpPathToFile(ompl::base::PathPtr path, DIMTPtr dimt, std::string filename)
{
  std::ofstream ofout(filename, std::ofstream::out);
  if(ofout.is_open())
  {
    ompl::geometric::PathGeometric * geopath = path->as<ompl::geometric::PathGeometric>();
    size_t node_num = geopath->getStateCount();
    for(size_t idx=0; idx< node_num - 1; idx++)
    { 
      ompl::base::State* state1 = geopath->getState(idx);
      ompl::base::State* state2 = geopath->getState(idx+1);
      std::vector<Eigen::VectorXd> points = dimt->discretize(state1, state2, 0.05);
      for(size_t j=0;j<points.size();j++)
      {
        for(size_t i=0; i<param.dimensions; i++)
        {
          ofout << points[j][i] << " ";
        }
        ofout << std::endl;
      }
    }
  }
  ofout.close();  
}

void dumpPathToFile(std::vector<ompl::base::State*> path, DIMTPtr dimt, std::string filename)
{
    std::ofstream ofout(filename, std::ofstream::out);
    if(ofout.is_open())
    {
      for(size_t idx=0; idx< path.size()-1; idx++)
      {
        ompl::base::State* state1 = path[idx];
        ompl::base::State* state2 = path[idx+1];
        std::vector<Eigen::VectorXd> points = dimt->discretize(state1, state2, 0.05);
        for(size_t j=0;j<points.size();j++)
        {
          for(size_t i=0; i<param.dimensions; i++)
          {
            ofout << points[j][i] << " ";
          }
          ofout << std::endl;
        }
      }
    }
    ofout.close();
}

void dumpPathToFile(ompl::base::PathPtr path, std::string filename)
{
  std::ofstream ofout(filename, std::ofstream::out);
  if(ofout.is_open())
  {
    ompl::geometric::PathGeometric * geopath = path->as<ompl::geometric::PathGeometric>();
    size_t node_num = geopath->getStateCount();
    for(size_t idx=0; idx< node_num; idx++)
    {
      ompl::base::State* state = geopath->getState(idx);


      for(size_t i=0; i<param.dimensions; i++)
      {
          ofout << state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] << " ";
      }
      ofout << std::endl;

    }
  }
  ofout.close();
}

#endif // FILE_UTIL_HPP_
