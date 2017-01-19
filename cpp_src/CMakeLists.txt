# Set major information of the build
cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Setup the project information
project(GetKinoSamples)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
add_executable(get_samples ${PROJECT_SOURCE_DIR}/src/get_samples.cpp)
add_executable(unit_test ${PROJECT_SOURCE_DIR}/src/Test.cpp)
include_directories(${PROJECT_SOURCE_DIR}/include)

# Find Eigen OMPL and Boost
include_directories(${PROJECT_SOURCE_DIR}/include/eigen-eigen-f562a193118d)
find_library (OMPL_LIBRARY 
	NAMES ompl
	# PATHS "/opt/ros/indigo/lib/x86_64-linux-gnu/"
	HINTS "/usr/lib")
find_package(Boost COMPONENTS system filesystem REQUIRED)

# Add library for problem definition class
add_library(problemDefinitionLib
			SHARED
			 src/ProblemDefinition.cpp
			)
target_link_libraries(problemDefinitionLib
					  ${EIGEN3_LIBRARIES}
					 )

# Add library for sampler class
add_library(samplerLib
			SHARED
			 src/Sampler.cpp
			 src/MonteCarloSamplers.cpp
			)
target_link_libraries(samplerLib
					  problemDefinitionLib
					  ${EIGEN3_LIBRARIES}
					 )

# Add library for dimt class
add_library(dimtLib
			SHARED
			 include/Dimt/Dimt.h
			)
target_link_libraries(dimtLib
					  ${EIGEN3_LIBRARIES}
					 )
set_target_properties(dimtLib PROPERTIES LINKER_LANGUAGE CXX)

# Link libraries for main file
target_link_libraries(get_samples
					  samplerLib
					  problemDefinitionLib
					  ${EIGEN3_LIBRARIES}
					  )

target_link_libraries(unit_test
					  dimtLib 
					  samplerLib
					  problemDefinitionLib
					  ${EIGEN3_LIBRARIES}
					  )

# Ompl Planner 
add_executable (kdp_pointmass2d src/kdp_pointmass2d.cpp)
target_link_libraries (kdp_pointmass2d ${OMPL_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})