# Add library for dimt class
add_library(dimtLib SHARED
            Interval.cpp
           )

target_link_libraries(dimtLib 
                     )

set_target_properties(dimtLib PROPERTIES LINKER_LANGUAGE CXX)
