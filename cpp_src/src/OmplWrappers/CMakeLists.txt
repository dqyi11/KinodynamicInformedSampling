# Add library for OMPL wrappers
add_library(omplwrapperLib SHARED
            MyOptimizationObjective.cpp
            MyInformedRRTstar.cpp
            OmplHelpers.cpp
            ValidityChecker.cpp
           )

target_link_libraries(omplwrapperLib
                      #${OMPL_LIBRARIES}
                     )
