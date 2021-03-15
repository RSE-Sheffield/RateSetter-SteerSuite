file(GLOB STEERBENCH_SRC src/*.cpp)
#file(GLOB STEERBENCH_HDR include/*.h)

add_executable(steerbench ${STEERBENCH_SRC})
target_include_directories(steerbench PRIVATE
  ./src
  ../external
  ../steerlib/include
  ../util/include
)
target_link_libraries(steerbench steerlib util tinyxml)
add_dependencies(steerbench steerlib util tinyxml)

if(ENABLE_GUI AND ENABLE_GLFW)
  target_link_libraries(steerbench glfw)
  add_dependencies(steerbench glfw)
endif()

if(WIN32)
elseif(APPLE)
  find_library(COCOA_LIBRARY Cocoa)
  mark_as_advanced(COCOA_LIBRARY)
  target_link_libraries(steerbench ${COCOA_LIBRARY} Threads::Threads  dl)
else()
  find_package(X11 REQUIRED)
  target_link_libraries(steerbench Threads::Threads ${X11_LIBRARIES} dl)
endif()

install(TARGETS steerbench
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
