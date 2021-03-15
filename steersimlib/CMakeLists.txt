file(GLOB STEERSIMLIB_SRC src/*.cpp)
file(GLOB STEERSIMLIB_HDR include/*.h)

add_library(steersimlib SHARED ${STEERSIMLIB_SRC} ${STEERSIMLIB_HDR})
target_include_directories(steersimlib PRIVATE
  ./include
  ../external
  ../steerlib/include
  ../util/include
)
target_link_libraries(steersimlib steerlib util tinyxml)
add_dependencies(steersimlib steerlib util tinyxml)
if(ENABLE_GUI AND ENABLE_GLFW)
  target_link_libraries(steersimlib glfw)
  add_dependencies(steersimlib glfw)
endif()
#if(${Qt5OpenGL_FOUND})
#  qt5_use_modules(steersimlib Gui OpenGL Core)
#endif()

install(TARGETS steersimlib
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

if(WIN32)
elseif(APPLE)
  find_library(COCOA_LIBRARY Cocoa)
  mark_as_advanced(COCOA_LIBRARY)
  target_link_libraries(steersimlib ${COCOA_LIBRARY} Threads::Threads dl)
else()
  find_package(X11 REQUIRED)
  target_link_libraries(steersimlib Threads::Threads ${X11_LIBRARIES} dl)
endif()

install(DIRECTORY include/ DESTINATION include/steersimlib)
