add_library(herb-validity-checker
            HerbValidityChecker.cpp
            )
target_link_libraries(herb-validity-checker
                      ${DART_LIBRARIES}
		      ${aikido_LIBRARIES}
		      ${Boost_LIBRARIES} 
	              libherb
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
                      herb-validity-checker
	              libherb
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
                   herb-validity-checker
	           libherb
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
                   herb-validity-checker
                   libherb
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
                   herb-validity-checker
                   libherb
                  )

install(TARGETS informed_pointmass RUNTIME DESTINATION bin)
install(TARGETS informed_pointmass_all RUNTIME DESTINATION bin)
