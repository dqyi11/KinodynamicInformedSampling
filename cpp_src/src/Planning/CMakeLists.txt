# Ompl Planner
add_executable(kdp_pointmass2d 
               kdp_pointmass2d.cpp
              )
target_link_libraries(kdp_pointmass2d 
                      ${OMPL_LIBRARIES} 
                      ${Boost_FILESYSTEM_LIBRARY} 
                      ${Boost_SYSTEM_LIBRARY} 
                      dimtLib
                     )

# Informed Pointmass
add_executable(informed_pointmass 
               informed_pointmass.cpp
              )
target_link_libraries(informed_pointmass 
                      ${OMPL_LIBRARIES} 
                      ${Boost_FILESYSTEM_LIBRARY} 
                      ${Boost_SYSTEM_LIBRARY} 
                      samplerLib 
                      dimtLib 
                      omplwrapperLib
                     )
