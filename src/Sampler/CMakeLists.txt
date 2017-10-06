# Add library for sampler class
add_library(samplerLib SHARED
            Sampler.cpp
            MonteCarloSampler.cpp
            RejectionSampler.cpp
            HitAndRunSampler.cpp
           )
target_link_libraries(samplerLib
                      ${OMPL_LIBRARIES}
                      ${NLOPT_LIBRARIES}
                     )

install(TARGETS samplerLib EXPORT samplerLibConfig
 ARCHIVE DESTINATION lib
 LIBRARY DESTINATION lib
 RUNTIME DESTINATION bin)
