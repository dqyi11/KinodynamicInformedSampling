#pragma once 

// 
// Steps we need to take in order to implement this correclty
//
// 1. We need to implement the required functions for 
//    InformedSampler(http://ompl.kavrakilab.org/InformedStateSampler_8h_source.html). Here
//    is where the informed sampling needs to take place (and the majority of our code)
// 2. We need to take this information and pass it to the the InformedStateSampler,
//    which inherits from StateSampler and takes in and InformedSampler as 
//    a constructor option.
// 3. Get this information about the samplers into the SpaceInformationPtr
// 4. Pass the SpaceInformationPtr to Informed RRT* and then done
// 

// stdlib
#include <iostream>

// OMPL
#include <ompl/base/samplers/InformedStateSampler.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>