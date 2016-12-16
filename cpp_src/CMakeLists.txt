# Set major information of the build
cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Setup the project information
project(GetKinoSamples)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
add_executable(get_samples ${PROJECT_SOURCE_DIR}/src/get_samples.cpp)
include_directories(${PROJECT_SOURCE_DIR}/include)

# Find Eigen
include_directories(${PROJECT_SOURCE_DIR}/include/eigen-eigen-f562a193118d)

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
			)
target_link_libraries(samplerLib
					  problemDefinitionLib
					  ${EIGEN3_LIBRARIES}
					 )

# Link libraries for main file
target_link_libraries(get_samples
					  samplerLib
					  problemDefinitionLib
					  ${EIGEN3_LIBRARIES}
					  )