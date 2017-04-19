# Add library for OMPL wrappers
add_library(omplwrapperLib SHARED
            MyOptimizationObjective.cpp
            MyInformedRRTstar.cpp
            OmplHelpers.cpp
           )

target_link_libraries(omplwrapperLib
                      #${OMPL_LIBRARIES}
                     )
