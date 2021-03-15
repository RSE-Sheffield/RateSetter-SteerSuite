# Enforce vaguley modern cmake. @todo - pick an actual version 
# 3.12 for add_compile_definitions.
# 3.14 for x11 imported targets?
cmake_minimum_required(VERSION 3.14...3.19)
include(CMakeDependentOption)

# @Todo - generator expressions?

# This file is equivalent to the `solution` in premake4. implemented for
# out-of-tree builds (i.e. in the `build` directory)

# Total project / solution name + the languages required (C++)
project(
  steersuite 
  LANGUAGES C CXX)

# ##############################################################################

# Global CMAKE_ variables/optionms.

# Set the location of output binaries and libraries, into /lib/bin 
# @todo - move this to common?
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Set fpic the modern cmake way. @todo this could be per target (i.e. libs)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Use C++11 (replacement of c++0x being passed previously)
# This might be better on a per target process?
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 11)
endif()

# ##############################################################################

# Define some CMake options, to be passed via CLI or via the GUI, to control
# build options.

# Enable the GUI.
option(ENABLE_GUI "Enable the GUI" ON)

# If using a GUI, enable the GLFW version of the gui
cmake_dependent_option(ENABLE_GLFW "Enable GLFW" ON "ENABLE_GUI" OFF)

# If using a GUI, enable the QT version of the gui. @note - this is disabled as
# it doesn't appear to be complete functionailty. option(ENABLE_QT "Enable QT
# GUI" OFF)

# There are no programatic tests, so no need for this option. option(BUILD_TESTS
# "Enable building tests" OFF)

# ##############################################################################

# Find dependencies and set compiler flags based on options.
if(ENABLE_GUI)
  # OpenGL etc are required regardless of GLFW/QT.
  set(OpenGL_GL_PREFERENCE "GLVND") # Or set to "LEGACY"?
  find_package(OpenGL REQUIRED)
  find_package(GLEW REQUIRED)
  find_package(GLUT REQUIRED)
  # X11 is required by GLFW in some cases?
  if(NOT WIN32 AND NOT APPLE)
    find_package(X11 REQUIRED)
  endif()

  # @todo - make these per target as required. The correct thing to do will be to let a GLFW project handle them as public?
  include_directories(${OPENGL_INCLUDE_DIR} ${GLEW_INCLUDE_DIR}
                      ${GLUT_INCLUDE_DIR})

  # Add the compile definition.
  add_compile_definitions(ENABLE_GUI)

  if(ENABLE_GLFW)
    add_compile_definitions(ENABLE_GLFW GLFW_DLL)
  endif() # Enable GLFW

  # the ENABLE_QT functionality in SteerSuite seems partial, so not propperly
  # configuring this.
  if(ENABLE_QT)
    # find_package(Qt5Core) find_package(Qt5Gui) find_package(Qt5OpenGL)
    # if(${Qt5OpenGL_FOUND}) include_directories(${Qt5OpenGL_INCLUDE_DIRS})
    # add_definitions(-DENABLE_QT) endif()
  endif()
endif()

# C/C++ threads (pthread) are required by several compontents. Find it once for
# earlier failure. @todo - does this cause issues on msvc?
find_package(Threads REQUIRED)

# If apple, we also need cocoa?
if(APPLE)
  find_library(COCOA_LIBRARY Cocoa REQUIRED)
  mark_as_advanced(COCOA_LIBRARY)
endif()
# ##############################################################################

# CMAKE_C_COMPILER_ID CMAKE_CXX_COMPILER_ID AppleClang Clang GNU MSVC

# Set a bunch of compiler settings. @todo - move these to common /
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMakeModules;${CMAKE_MODULE_PATH}")
set(CMAKE_DEBUG_POSTFIX "d")

include(CheckCXXCompilerFlag)
# GCC/Clang compiler options.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  # Enale SSE if avaiulable.
  check_cxx_compiler_flag(-msse STEERSUITE_GCC_HAS_SSE)
  if(STEERSUITE_GCC_HAS_SSE)
    add_compile_options(-msse)
  endif()

  # Enable a lot of warmings
  add_compile_options(-Wall)
  # The old cmake used to all requrest these warnings, leading to > 4000 warnings without suppressions.
  # add_compile_options(-Winit-self -Wcast-qual -Wwrite-strings -Wextra -Wshadow)

  # Suppress some warnings.
  add_compile_options(-Wno-unused-parameter -Wno-missing-field-initializers -Wno-long-long)
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wno-overloaded-virtual>)

  # Additionaly suppressions currently required to get (almost) warning-free builds with gcc 10.
  # Some of these are valid warnings that should be fixed instead of beind suppressed.
  add_compile_options(-Wno-sign-compare -Wno-ignored-qualifiers -Wno-unused-but-set-variable -Wno-cast-qual -Wno-deprecated-declarations -Wno-format -Wno-unused-variable -Wno-misleading-indentation)

  if(EMSCRIPTEN)
    add_compile_options(-Wno-warn-absolute-paths)
  elseif(NOT APPLE)
    add_compile_options(-Wno-unused-but-set-parameter)
  endif()

  # add debug/ndebug definitions for correct assertion behaviour (no asserts in)
  add_compile_definitions($<IF:$<CONFIG:Debug>,DEBUG,NDEBUG>)
endif()


if(MSVC)
  if(CMAKE_BUILD_TOOL STREQUAL "nmake")
    set(NMAKE TRUE)
  endif()
  add_compile_options(/fp:fast)
  add_compile_options($<$<CONFIG:Debug>:/Oi>)
  if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    add_compile_options($<$<CONFIG:Debug>:/bigobj>)
    add_compile_options($<$<CONFIG:RelWithDebInfo>:/bigobj>)
  endif()
  # If supported by msvc version, perform a parallel build.
  if(MSVC_VERSION GREATER 1500 OR MSVC_VERSION EQUAL 1500)
    add_compile_options(/MP)
  endif()
endif()


if(MINGW)
  add_compile_definitions(_WIN32_WINNT=0x0601)
  if(CMAKE_SIZEOF_VOID_P EQUAL 4)
    add_compile_options(-march=i686)
  endif()
  add_compile_options(-fpermissive -fno-tree-slp-vectorize -Wno-error=cast-qual -Wno-unused-local-typedefs)
endif()


# ##############################################################################

# @todo - make tinyxml/glfw dynamic via cmake (if removing premake?) Add
# exeternal dependency projects
add_subdirectory(external/tinyxml)
add_subdirectory(external/recastnavigation)

# Add the external GLFW project if required
if(ENABLE_GUI AND ENABLE_GLFW)
  add_subdirectory(external/glfw)
endif()

# Add sub projects
add_subdirectory(util)
add_subdirectory(steerlib)
add_subdirectory(steersimlib)
add_subdirectory(steersim)
add_subdirectory(simpleAI)
add_subdirectory(socialForcesAI)
add_subdirectory(rvo2AI)
add_subdirectory(pprAI)
add_subdirectory(navmeshBuilder)
add_subdirectory(steerbench)
add_subdirectory(kdtree)

# Add the documentaiton
add_subdirectory(documentation)

# ##############################################################################

# @todo - install rule(s) - not sure what premake4 does for this.
install(DIRECTORY testcases DESTINATION share)

# ##############################################################################

# @todo OSX specific settings... Not sure how I can test this (without public CI..?)


# @todo - IDE folders 
# set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# @todo - reactiveAI isnt' in premake or in cmake?
# @todo - steertool is not in premake or in cmake 