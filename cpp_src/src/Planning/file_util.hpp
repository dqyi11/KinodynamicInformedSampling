#ifndef FILE_UTIL_HPP_
#define FILE_UTIL_HPP_

#include <string>
#include <ompl/base/State.h>
#include <ompl/base/Path.h>
#include <ompl/geometric/PathGeometric.h>
#include "Dimt/Params.h"

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
