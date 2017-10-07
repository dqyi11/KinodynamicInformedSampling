add_library(multilink-di-validity-checker
            MultiLinkDIValidityChecker.cpp
            )
target_link_libraries(multilink-di-validity-checker
                      multilink-di-dart
                     )

add_executable(kdp_pointmass2d 
               kdp_pointmass2d.cpp
              )
target_link_libraries(kdp_pointmass2d 
                      ${OMPL_LIBRARIES} 
                      ${Boost_FILESYSTEM_LIBRARY} 
                      ${Boost_SYSTEM_LIBRARY} 
                      dimtLib
                     )

add_executable(batch_kdp_pointmass2d
               batch_kdp_pointmass2d.cpp
              )
target_link_libraries(batch_kdp_pointmass2d
                      ${OMPL_LIBRARIES}
                      ${Boost_FILESYSTEM_LIBRARY}
                      ${Boost_SYSTEM_LIBRARY}
                      dimtLib
                      omplwrapperLib
                     )

# Informed Pointmass
add_executable(informed_pointmass
               informed_pointmass.cpp
              )
target_link_libraries(informed_pointmass 
                      ${OMPL_LIBRARIES} 
                      ${Boost_FILESYSTEM_LIBRARY} 
                      ${Boost_SYSTEM_LIBRARY} 
                      ${JSONCPP_LIBRARY}
                      samplerLib 
                      dimtLib 
                      omplwrapperLib
                      multilink-di-validity-checker
                     )

# Informed Pointmass all
add_executable(informed_pointmass_all
               informed_pointmass_all.cpp
              )
target_link_libraries(informed_pointmass_all
                   ${OMPL_LIBRARIES}
                   ${Boost_FILESYSTEM_LIBRARY}
                   ${Boost_SYSTEM_LIBRARY}
                   ${JSONCPP_LIBRARY}
                   samplerLib
                   dimtLib
                   omplwrapperLib
                   multilink-di-validity-checker
                  )


install(TARGETS informed_pointmass RUNTIME DESTINATION bin)
install(TARGETS informed_pointmass_all RUNTIME DESTINATION bin)
