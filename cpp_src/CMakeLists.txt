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

# Find Eigen OMPL and Boost and Eigen
find_package(OMPL REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)
find_package(EIGEN3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
find_package(NLOPT)
if(NLOPT_FOUND)
    message("USE NLOPT")
    include_directories(${NLOPT_INCLUDE_DIRS})
    add_definitions( -DUSE_NLOPT=1 )
endif()

find_package(JsonCpp REQUIRED)
if(NOT JSONCPP_FOUND)
   message("Jsoncpp not found")
endif()
include_directories(${JSONCPP_INCLUDE_DIR})
link_libraries(${JSONCPP_LIBRARY})


add_definitions( -DENABLE_BENCHMARK=1 )

include_directories(${PROJECT_SOURCE_DIR}/include)

FILE(GLOB_RECURSE LibFiles "include/*.h"
                           "include/*.hpp")
add_custom_target(headers SOURCES ${LibFiles})

add_subdirectory("src")
add_subdirectory("test")








