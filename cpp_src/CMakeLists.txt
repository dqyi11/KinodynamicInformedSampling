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
add_executable(get_samples ${PROJECT_SOURCE_DIR}/src/get_samples.cpp)
add_executable(get_samples_batch ${PROJECT_SOURCE_DIR}/src/get_samples_batch.cpp)
add_executable(get_samples_diff_levels ${PROJECT_SOURCE_DIR}/src/get_samples_diff_levels.cpp)
add_executable(get_samples_diff_sample_no ${PROJECT_SOURCE_DIR}/src/get_samples_diff_sample_no.cpp)
add_executable(unit_test ${PROJECT_SOURCE_DIR}/src/Test.cpp)
include_directories(${PROJECT_SOURCE_DIR}/include)

# Find Eigen OMPL and Boost
include_directories(${PROJECT_SOURCE_DIR}/include/eigen-eigen-f562a193118d)
find_package(OMPL REQUIRED)
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
			 src/RejectionSampler.cpp
			 src/OmplSamplers.cpp
			)
target_link_libraries(samplerLib
					  problemDefinitionLib
					  ${EIGEN3_LIBRARIES}
					  ${OMPL_LIBRARIES}
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

# Add library for OMPL wrappers
add_library(omplwrapperLib
			SHARED
			 src/OmplSamplers.cpp
			 src/MyOptimizationObjective.cpp
			 src/MyInformedRRTstar.cpp
			)
target_link_libraries(omplwrapperLib
					  ${OMPL_LIBRARIES}
					  )

# Link libraries for main file
target_link_libraries(get_samples
					  samplerLib
					  problemDefinitionLib
					  ${EIGEN3_LIBRARIES}
					  #{OMPL_LIBRARIES}
					  )

target_link_libraries(get_samples_batch
					  samplerLib
					  problemDefinitionLib
					  ${EIGEN3_LIBRARIES}
					  #{OMPL_LIBRARIES}
					  )

target_link_libraries(get_samples_diff_levels
					  samplerLib
					  problemDefinitionLib
					  ${EIGEN3_LIBRARIES}
					  #{OMPL_LIBRARIES}
					  )

target_link_libraries(get_samples_diff_sample_no
					  samplerLib
					  problemDefinitionLib
					  ${EIGEN3_LIBRARIES}
					  #{OMPL_LIBRARIES}
					  )

target_link_libraries(unit_test
					  dimtLib
					  samplerLib
					  problemDefinitionLib
					  ${EIGEN3_LIBRARIES}
					  )

# Ompl Planner
add_executable (kdp_pointmass2d src/kdp_pointmass2d.cpp)
target_link_libraries (kdp_pointmass2d ${OMPL_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})

# Informed Sampling tester
add_executable(informed_sampling_test src/informed_sampling_test.cpp)
target_link_libraries(informed_sampling_test
					  ${OMPL_LIBRARIES}
					  ${Boost_FILESYSTEM_LIBRARY}
					  ${Boost_SYSTEM_LIBRARY}
					  samplerLib
					  omplwrapperLib
					  )

# Informed Pointmass
add_executable (informed_pointmass src/informed_pointmass.cpp)
target_link_libraries (informed_pointmass ${OMPL_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} samplerLib omplwrapperLib)

# Informed Pointmass
add_executable (get_IRRT_star_path src/get_IRRT_star_path.cpp)
target_link_libraries (get_IRRT_star_path ${OMPL_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} samplerLib omplwrapperLib)
