find_package(aikido QUIET)
include_directories(${aikido_INCLUDE_DIRS})
if(NOT aikido_FOUND)
  return()
endif()

find_package(libherb QUIET)
include_directories(${libherb_INCLUDE_DIRS})
if(NOT libherb_FOUND)
  return()
endif()

find_package(pr_ordata QUIET)
include_directories(${pr_ordata_INCLUDE_DIRS})
if(NOT pr_ordata_FOUND)
  return()
endif()

find_package(pr_tsr QUIET)
include_directories(${pr_tsr_INCLUDE_DIRS})
if(NOT pr_tsr_FOUND)
  return()
endif()

find_package( Boost COMPONENTS program_options QUIET)
include_directories( ${Boost_INCLUDE_DIR} )
if(NOT Boost_FOUND)
  return()
endif()

# Informed HERB
add_executable(informed_herb
               informed_herb.cpp
              )
target_link_libraries(informed_herb 
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

# Informed HERB all
add_executable(informed_herb_all
               informed_herb_all.cpp
              )
target_link_libraries(informed_herb_all
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

add_library(herb-validity-checker
            HerbValidityChecker.cpp
            )
target_link_libraries(herb-validity-checker
                      ${DART_LIBRARIES}
		      ${aikido_LIBRARIES}
		      ${Boost_LIBRARIES} 
	              libherb
                     )

add_executable(loadpath
               loadpath.cpp)

target_link_libraries(loadpath
                     ${DART_LIBRARIES}
                     ${aikido_LIBRARIES}
                     ${Boost_LIBRARIES}
                     libherb)


install(TARGETS informed_herb RUNTIME DESTINATION bin)
install(TARGETS informed_herb_all RUNTIME DESTINATION bin)
install(TARGETS loadpath RUNTIME DESTINATION bin)
