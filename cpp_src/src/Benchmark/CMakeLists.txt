add_executable(get_samples 
               get_samples.cpp
              )
target_link_libraries(get_samples
                      samplerLib
                      dimtLib
                      omplwrapperLib
                      )

add_executable(get_samples_batch 
               get_samples_batch.cpp
              )
target_link_libraries(get_samples_batch
                      samplerLib
                      dimtLib
                      omplwrapperLib
                     )

add_executable(get_samples_diff_levels 
               get_samples_diff_levels.cpp
              )
target_link_libraries(get_samples_diff_levels
                      samplerLib
                      dimtLib
                      omplwrapperLib
                     )

add_executable(get_samples_diff_levels_geometry 
               get_samples_diff_levels_geometry.cpp
              )
target_link_libraries(get_samples_diff_levels_geometry
                      samplerLib
                      dimtLib
                      omplwrapperLib
                     )

add_executable(get_samples_diff_sample_no 
               get_samples_diff_sample_no.cpp
              )
target_link_libraries(get_samples_diff_sample_no
                      samplerLib
                      dimtLib
                      omplwrapperLib
                     )
