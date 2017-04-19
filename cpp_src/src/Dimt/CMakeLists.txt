# Add library for dimt class
add_library(dimtLib SHARED
            Interval.cpp
           )

target_link_libraries(dimtLib 
                      ${EIGEN3_LIBRARIES}
                     )

set_target_properties(dimtLib PROPERTIES LINKER_LANGUAGE CXX)
