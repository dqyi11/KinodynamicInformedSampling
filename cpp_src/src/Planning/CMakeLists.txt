add_library(multilink-di-validity-checker
            MultiLinkDIValidityChecker.cpp
            )
target_link_libraries(multilink-di-validity-checker
                      multilink-di-dart
                     )

# Ompl Planner
add_executable(kdp_pointmass2d 
               kdp_pointmass2d.cpp
              )
target_link_libraries(kdp_pointmass2d 
                      ${OMPL_LIBRARIES} 
                      ${Boost_FILESYSTEM_LIBRARY} 
                      ${Boost_SYSTEM_LIBRARY} 
                      ${JSONCPP_LIBRARY}
                      dimtLib
                      multilink-di-validity-checker
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

# Informed Pointmass diff HMC
add_executable(informed_pointmass_diff_hmc
               informed_pointmass_diff_hmc.cpp
              )
target_link_libraries(informed_pointmass_diff_hmc
                   ${OMPL_LIBRARIES}
                   ${Boost_FILESYSTEM_LIBRARY}
                   ${Boost_SYSTEM_LIBRARY}
                   ${JSONCPP_LIBRARY}
                   samplerLib
                   dimtLib
                   omplwrapperLib
                   multilink-di-validity-checker
                  )

# Informed Pointmass diff velocity bounds
add_executable(informed_pointmass_diff_velocity_bounds
               informed_pointmass_diff_velocity_bounds.cpp
              )
target_link_libraries(informed_pointmass_diff_velocity_bounds
                   ${OMPL_LIBRARIES}
                   ${Boost_FILESYSTEM_LIBRARY}
                   ${Boost_SYSTEM_LIBRARY}
                   ${JSONCPP_LIBRARY}
                   samplerLib
                   dimtLib
                   omplwrapperLib
                   multilink-di-validity-checker
                  )
