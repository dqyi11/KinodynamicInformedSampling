add_executable(get_samples 
               getSamples.cpp
              )
target_link_libraries(get_samples
                      samplerLib
                      dimtLib
                      omplwrapperLib
                      )

add_executable(get_samples_batch 
               getSamplesBatch.cpp
              )
target_link_libraries(get_samples_batch
                      samplerLib
                      dimtLib
                      omplwrapperLib
                     )

add_executable(get_samples_diff_levels 
               getSamplesDiffLevels.cpp
              )
target_link_libraries(get_samples_diff_levels
                      samplerLib
                      dimtLib
                      omplwrapperLib
                     )

add_executable(get_samples_diff_levels_geometry 
               getSamplesDiffLevelsGeometry.cpp
              )
target_link_libraries(get_samples_diff_levels_geometry
                      samplerLib
                      dimtLib
                      omplwrapperLib
                     )

add_executable(get_samples_diff_sample_num 
               getSamplesDiffSampleNum.cpp
              )
target_link_libraries(get_samples_diff_sample_num
                      samplerLib
                      dimtLib
                      omplwrapperLib
                     )
