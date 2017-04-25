add_executable(unit_test 
               Test.cpp)


target_link_libraries(unit_test
                      dimtLib
                      samplerLib
                     )

add_executable(rnd_test 
               RndTest.cpp)


target_link_libraries(rnd_test
                      dimtLib
                      samplerLib
                     )


