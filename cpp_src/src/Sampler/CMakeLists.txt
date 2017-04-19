# Add library for sampler class
add_library(samplerLib SHARED
            Sampler.cpp
            MonteCarloSamplers.cpp
            RejectionSampler.cpp
            HitAndRun.cpp
           )
target_link_libraries(samplerLib
                      ${OMPL_LIBRARIES}
                     )
