
find_package( Boost COMPONENTS program_options REQUIRED )
include_directories( ${Boost_INCLUDE_DIR} )

# ===============================================================================
# Dependencies
#


# ===============================================================================
# Executables
#


add_executable(loadpath
  loadpath.cpp)

target_link_libraries(loadpath
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES}
  ${Boost_LIBRARIES} 
  libherb)

# ===============================================================================
# Installation
#

install(TARGETS loadpath RUNTIME DESTINATION bin)



