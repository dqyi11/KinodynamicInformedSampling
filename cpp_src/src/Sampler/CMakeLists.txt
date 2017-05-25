# Add library for sampler class
add_library(samplerLib SHARED
            Sampler.cpp
            MonteCarloSampler.cpp
            RejectionSampler.cpp
            HitAndRunSampler.cpp
           )
target_link_libraries(samplerLib
                      ${OMPL_LIBRARIES}
                     )
