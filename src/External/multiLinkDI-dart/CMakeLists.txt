find_package(DART 6.2.0 REQUIRED COMPONENTS utils-urdf gui CONFIG)

#add_compile_options(-std=c++11)

include_directories(${DART_INCLUDE_DIRS}
                    include/)

#FILE(GLOB_RECURSE LibFiles "include/*.h"
#                           "include/*.hpp")
#add_custom_target(headers SOURCES ${LibFiles})

add_library(multilink-di-dart src/MultiLinkDI.cpp
                              src/MultiLinkDIWindow.cpp)
target_link_libraries(multilink-di-dart ${DART_LIBRARIES})

#add_executable(${PROJECT_NAME}_test test/Main.cpp)
#target_link_libraries(${PROJECT_NAME}_test ${DART_LIBRARIES}
#                                           ${PROJECT_NAME})


