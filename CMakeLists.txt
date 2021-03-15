# Enforce vaguley modern cmake. @todo - pick an actual version 
cmake_minimum_required( VERSION 3.1 )
# This file is equivalent to the `solution` in premake4.
# implemented for out-of-tree builds (i.e. in the `build` directory)

# Total project / solution name + the languages required (C++)
project(steersuite LANGUAGES C CXX)

# Define some CMake options, to be passed via CLI or via the GUI, to control build options.

# Enable the GUI.
option(ENABLE_GUI "Enable the GUI" ON)

# If using a GUI, enable the GLFW version of the gui
# @todo - dependent option + mutually exclusive? 
option(ENABLE_GLFW "Enable GLFW" ON)

# If using a GUI, enable the QT version of the gui.
# @todo - dependent option + mutually exclusive? 
option(ENABLE_QT "Enable QT GUI" OFF)

# There are no programatic tests, so no need for this option.
# option(BUILD_TESTS "Enable building tests" OFF)



# Find dependencies and set compiler flags based on options.


message(STATUS "ENABLE_GUI ${ENABLE_GUI}")
message(STATUS "ENABLE_GLFW ${ENABLE_GLFW}")
message(STATUS "ENABLE_QT ${ENABLE_QT}")

if(ENABLE_GUI)
  message(STATUS "Find GUI dependencies @todo.")
  # OpenGL etc are required regardless of GLFW/QT.
  find_package(X11 REQUIRED)
  set(OpenGL_GL_PREFERENCE "GLVND") # Or set to "LEGACY"?
  find_package(OpenGL REQUIRED)
  find_package(GLEW REQUIRED)
  find_package(GLUT REQUIRED)
  # @todo - move this include statement.
  include_directories(${OPENGL_INCLUDE_DIR} ${GLEW_INCLUDE_DIR} ${GLUT_INCLUDE_DIR})
  # @todo - move ths definiton statment?
  add_definitions(-DENABLE_GUI)
  
  if(ENABLE_GLFW)
    message(STATUS "Find GLFW? or download rather than shipping a specific version in external? (If only cmake as build system)")
    add_definitions(-DENABLE_GLFW -DGLFW_DLL)
  endif() # Enable GLFW
  
  if(ENABLE_QT)
  message(STATUS "Find QT? @todo")
    # find_package(Qt5Core)
    #find_package(Qt5Gui)
    #find_package(Qt5OpenGL)
    #if(${Qt5OpenGL_FOUND})
    #  include_directories(${Qt5OpenGL_INCLUDE_DIRS})
    #  add_definitions(-DENABLE_QT)
    #endif()
  endif()
endif()


# C/C++ threads are required by several compontents. Find it once for earlier failure.
find_package(Threads REQUIRED)



# Set the location of output binaries and libraries, into /lib/bin
# @todo - move this to common?
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# CMAKE_C_COMPILER_ID
# CMAKE_CXX_COMPILER_ID
# AppleClang
# Clang
# GNU
# MSVC

# Set a bunch of compiler settings. @todo - move these to common / 
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMakeModules;${CMAKE_MODULE_PATH}")
set(CMAKE_DEBUG_POSTFIX "d")
include(CheckCXXCompilerFlag)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANGXX)
  if(EMSCRIPTEN)
    set(STEERSUITE_GCC_HAS_SSE FALSE)
  else()
    check_cxx_compiler_flag(-msse STEERSUITE_GCC_HAS_SSE)
  endif()
  if(STEERSUITE_GCC_HAS_SSE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse")
  endif ()
  set(STEERSUITE_WARNING_FLAGS "-Wall -Winit-self -Wno-overloaded-virtual -Wcast-qual -Wwrite-strings -Wextra -Wno-unused-parameter -Wshadow -Wno-missing-field-initializers -Wno-long-long")
  if(EMSCRIPTEN)
    set(STEERSUITE_WARNING_FLAGS "${STEERSUITE_WARNING_FLAGS} -Wno-warn-absolute-paths")
  elseif (NOT APPLE)
    set(STEERSUITE_WARNING_FLAGS "${STEERSUITE_WARNING_FLAGS} -Wno-unused-but-set-parameter")
  endif ()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${STEERSUITE_WARNING_FLAGS} -std=c++0x -ggdb")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE} -DNDEBUG")
endif ()
if(MSVC)
  if(CMAKE_BUILD_TOOL STREQUAL "nmake")
    set(NMAKE TRUE)
  endif()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /fp:fast")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Oi")
  if(CMAKE_CL_64)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /bigobj")
    set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /bigobj")
  endif ()
  if(MSVC_VERSION GREATER 1500 OR MSVC_VERSION EQUAL 1500)
    option(STEERSUITE_BUILD_MSVC_MP "Enable build with multiple processes in Visual Studio" TRUE)
  else()
    set(STEERSUITE_BUILD_MSVC_MP FALSE CACHE BOOL "Compiler option /MP requires at least Visual Studio 2008 (VS9) or newer" FORCE)
  endif()
  if(STEERSUITE_BUILD_MSVC_MP)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
  endif ()
endif ()
if(MINGW)
  add_definitions(-D_WIN32_WINNT=0x0601)
  if(CMAKE_SIZEOF_VOID_P EQUAL 4)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=i686")
  endif()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive -fno-tree-slp-vectorize -Wno-error=cast-qual -Wno-unused-local-typedefs")
endif()
if((CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANGXX) AND NOT MINGW)
  if(CMAKE_SIZEOF_VOID_P EQUAL 8 AND NOT APPLE)
    add_definitions(-fPIC)
  endif()
endif()


# @todo OSX specific settings... Not sure how I can test this (without public CI..?)

# @todo - make tinyxml/glfw dynamic via cmake (if removing premake?) 
add_subdirectory( external/tinyxml )
add_subdirectory( external/recastnavigation )

# @todo - only enable glfw if enable_GUI is also on?
if(ENABLE_GUI AND ENABLE_GLFW)
  add_subdirectory( external/glfw )
endif()


# @todo - add all the sub-projects.
add_subdirectory( util )
add_subdirectory( steerlib )
add_subdirectory( steersimlib )
add_subdirectory( steersim )
add_subdirectory( simpleAI )
add_subdirectory( socialForcesAI )
add_subdirectory( rvo2AI )
add_subdirectory( pprAI )
add_subdirectory( navmeshBuilder )
add_subdirectory( steerbench )
add_subdirectory( kdtree )

# Add the documentaiton?
add_subdirectory( documentation )



# @todo - install rule(s) - not sure what premake4 does for this.
# install(DIRECTORY testcases DESTINATION share)


# @todo - replace use of glob in subprojects with explicit lists of sources. Glob is not reccomended by cmake/kitware.