file(GLOB STEERSIM_SRC src/*.cpp)
#file(GLOB STEERSIM_HDR include/*.h)

add_executable(steersim ${STEERSIM_SRC})
target_include_directories(steersim PRIVATE
  ./src
  ../external
  ../steerlib/include
  ../steersimlib/include
  ../util/include
)
target_link_libraries(steersim steerlib steersimlib util tinyxml)
add_dependencies(steersim steerlib steersimlib util tinyxml)
if(ENABLE_GUI AND ENABLE_GLFW)
  target_link_libraries(steersim glfw)
  add_dependencies(steersim glfw)
endif()

if(WIN32)
elseif(APPLE)
  find_library(COCOA_LIBRARY Cocoa)
  mark_as_advanced(COCOA_LIBRARY)
  target_link_libraries(steersim ${COCOA_LIBRARY} Threads::Threads dl)
else()
  find_package(X11 REQUIRED)
  target_link_libraries(steersim Threads::Threads ${X11_LIBRARIES} dl)
endif()

install(TARGETS steersim
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
