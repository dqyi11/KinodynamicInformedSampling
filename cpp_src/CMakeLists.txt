# Set major information of the build
cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")


# Setup the project information
project(GetKinoSamples)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/Modules/")

# Find Eigen OMPL and Boost
#include_directories(${PROJECT_SOURCE_DIR}/include/eigen-eigen-f562a193118d)
find_package(OMPL REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)
find_package(EIGEN3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include)

add_subdirectory("src")
add_subdirectory("test")








