# Define the headers and sources
set(STEERSIM_INCLUDE)

set(STEERSIM_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/Main.cpp")

# Define inlcude directories
set(STEERSIM_INCLUDE_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/../steerlib/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../steersimlib/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../external"
    "${CMAKE_CURRENT_SOURCE_DIR}/../util/include")

# Specify the executable
add_executable(steersim ${STEERSIM_SRC})

# Specify include directories
target_include_directories(steersim PRIVATE "${STEERSIM_INCLUDE_DIRS}")

# Specifiy link targets
target_link_libraries(steersim steerlib steersimlib util)

# Optionally link agaisnt glfw (and X11?)
if(ENABLE_GUI AND ENABLE_GLFW)
  target_link_libraries(steersim glfw)
  if(NOT WIN32 AND NOT APPLE)
    find_package(X11 REQUIRED)
    target_link_libraries(steersim X11::X11)
  endif()
endif()

# If not windows, link against dl and pthread.
if(NOT WIN32)
  target_link_libraries(steersim Threads::Threads)
  target_link_libraries(steersim ${CMAKE_DL_LIBS})
endif()

# Find and use Cocoa if on apple.
if(APPLE)
  target_link_libraries(steersim ${COCOA_LIBRARY})
endif()

# Install rule.
install(
  TARGETS steersim
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

# Construct source groups for IDEs
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/include PREFIX include FILES ${STEERSIM_INCLUDE})
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/src PREFIX src FILES ${STEERSIM_SRC})
