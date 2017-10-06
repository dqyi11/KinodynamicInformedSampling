# Add library for OMPL wrappers
add_library(omplwrapperLib SHARED
            MyOptimizationObjective.cpp
            MyInformedRRTstar.cpp
            MyBITstar.cpp
            OmplHelpers.cpp
            ValidityChecker.cpp
           )
target_link_libraries(omplwrapperLib
                      ${OMPL_LIBRARIES}
                     )

install(TARGETS omplwrapperLib EXPORT omplwrapperLibConfig
                    ARCHIVE DESTINATION lib
                    LIBRARY DESTINATION lib
                    RUNTIME DESTINATION bin)
