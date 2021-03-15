set(STEERSIMLIB_INCLUDE
    "${CMAKE_CURRENT_SOURCE_DIR}/include/core/CommandLineEngineDriver.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/core/GLFWEngineDriver.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/core/QtEngineDriver.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/core/SteerSim.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/qtgui/ClockWidget.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/qtgui/ConsoleWidget.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/qtgui/GlobalEventFilter.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/qtgui/GLWidget.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/qtgui/ModuleManagerWidget.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/qtgui/QtEngineController.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/qtgui/RecFilePlayerWidget.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/qtgui/TestCasePlayerWidget.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/qtgui/Tooltip.h")
set(STEERSIMLIB_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/src/ClockWidget.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/CommandLineEngineDriver.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/ConsoleWidget.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/GLFWEngineDriver.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/GlobalEventFilter.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/GLWidget.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/ModuleManagerWidget.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/QtEngineController.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/QtEngineDriver.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/RecFilePlayerWidget.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/SteerSim.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/TestCasePlayerWidget.cpp")
set(STEESIMLIB_INCLUDE_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../external"
    "${CMAKE_CURRENT_SOURCE_DIR}/../steerlib/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../util/include")

add_library(steersimlib SHARED ${STEERSIMLIB_SRC} ${STEERSIMLIB_INCLUDE})
target_include_directories(steersimlib PRIVATE ${STEESIMLIB_INCLUDE_DIRS})
target_link_libraries(steersimlib steerlib util tinyxml)

if(ENABLE_GUI AND ENABLE_GLFW)
  target_link_libraries(steersimlib glfw)
  if(NOT WIN32 AND NOT APPLE)
    find_package(X11 REQUIRED)
    target_link_libraries(steersimlib X11::X11)
  endif()
endif()
# if(${Qt5OpenGL_FOUND}) qt5_use_modules(steersimlib Gui OpenGL Core) endif()

# If not windows, link against dl and pthread.
if(NOT WIN32)
  target_link_libraries(steersimlib Threads::Threads)
  target_link_libraries(steersimlib ${CMAKE_DL_LIBS})
endif()

# Find and use Cocoa if on apple.
if(APPLE)
  target_link_libraries(steersimlib ${COCOA_LIBRARY})
endif()

install(
  TARGETS steersimlib
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(DIRECTORY include/ DESTINATION include/steersimlib)

# @todo - check if make install works nicely? @todo - rpath. @todo - osx console
# mode, -install_name @todo - enforce clang on osx? @todo - test on windows /
# see what msvc changes John has made lately? @todo - config specific
# directories. Currently they just overwrite each other which I think is
# intended? This is fine for linux / osx, don't think msvc will do this (I.e.
# what does msvc currenrlty do)
