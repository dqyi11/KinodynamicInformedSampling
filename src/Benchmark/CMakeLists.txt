add_executable(get_samples 
               getSamples.cpp
              )
target_link_libraries(get_samples
                      samplerLib
                      dimtLib
                      omplwrapperLib
                      )

add_executable(get_samples_batch_random
               getSamplesBatchRandom.cpp
              )
target_link_libraries(get_samples_batch_random
                      samplerLib
                      dimtLib
                      omplwrapperLib
                     )

