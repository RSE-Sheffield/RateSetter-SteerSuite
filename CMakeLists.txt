# Enforce vaguley modern cmake. @todo - pick an actual version 
# 3.3  for CMAKE_ARCHIVE_OUTPUT_DIRECTORY_<CONFIG>
# 3.12 for add_compile_definitions.
# 3.14 for x11 imported targets?
cmake_minimum_required(VERSION 3.14...3.19)
include(CMakeDependentOption)

# This file is equivalent to the `solution` in premake4.
# implemented for out-of-tree builds (i.e. in the `build` directory)

project(
  steersuite 
  LANGUAGES C CXX)

# ###########################

# Set the location of output binaries and libraries, into /lib/bin 
# Multi-configuration generators automaticall append the config to the output directories, unless a genex is used. 
# This is sensible, but steersuite is not designed to account for this. 
# Instead, make it a generator expression with the noop genex $<0:> 
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/$<0:>)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/$<0:>)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/$<0:>)

# Set fpic the modern cmake way. This could be per-target.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Use C++11 (replacement of c++0x being passed previously)
# This might be better on a per target process?
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 11)
endif()

# Enable the use of folders in IDEs 
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# ###########################

# Define some CMake options for CLI/GUI

# Enable the GUI.
option(ENABLE_GUI "Enable the GUI" ON)

# If using a GUI, enable the GLFW version of the gui
cmake_dependent_option(ENABLE_GLFW "Enable GLFW" ON "ENABLE_GUI" OFF)

# ENABLE_QT not implemented, as ENABLE_QT support seems incomplete.

# Option to control the output of compiler warnings. 
# This should be ON, however the number of warnings generated is significant, so off by default (for now)
option(ENABLE_WARNINGS "Enable compiler wranings" OFF)


# ###########################

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
# ###########################

# CMAKE_C_COMPILER_ID CMAKE_CXX_COMPILER_ID AppleClang Clang GNU MSVC

# Set a bunch of compiler settings. @todo - move these to common /
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMakeModules;${CMAKE_MODULE_PATH}")
set(CMAKE_DEBUG_POSTFIX "d")

# add debug/ndebug definitions for correct assertion behaviour (no asserts in)
add_compile_definitions($<IF:$<CONFIG:Debug>,DEBUG,NDEBUG>)
add_compile_definitions($<IF:$<CONFIG:Debug>,_DEBUG,_NDEBUG>)

include(CheckCXXCompilerFlag)
# GCC/Clang compiler options.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  # Enale SSE if avaiulable.
  check_cxx_compiler_flag(-msse STEERSUITE_GCC_HAS_SSE)
  if(STEERSUITE_GCC_HAS_SSE)
    add_compile_options(-msse)
  endif()

  # Set warning levels for gcc/clang. Ideally a high level should always be set, but too many outstanding issues. 
  if(ENABLE_WARNINGS)
    # Enable a lot of warmings
    add_compile_options(-Wall)
    # The old cmake used to all requrest these warnings, leading to > 4000 warnings without suppressions.
    # add_compile_options(-Winit-self -Wcast-qual -Wwrite-strings -Wextra -Wshadow)
    # Suppress some warnings.
    add_compile_options(-Wno-unused-parameter -Wno-missing-field-initializers -Wno-long-long)
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wno-overloaded-virtual>)
  else()
    # Additionaly suppressions currently required to get (almost) warning-free builds with gcc 10.
    # Some of these are valid warnings that should be fixed instead of beind suppressed.
    add_compile_options(-Wno-sign-compare -Wno-ignored-qualifiers -Wno-unused-but-set-variable -Wno-cast-qual -Wno-deprecated-declarations -Wno-format -Wno-unused-variable -Wno-misleading-indentation)
  endif()
  if(EMSCRIPTEN)
    add_compile_options(-Wno-warn-absolute-paths)
  elseif(NOT APPLE)
    add_compile_options(-Wno-unused-but-set-parameter)
  endif()
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

  # Set warning levels for MSVC. Always on would be better.
  if(ENABLE_WARNINGS)
    add_compile_options(/W4)
  else()
    add_compile_options(/W0)
  endif()
endif()


if(MINGW)
  add_compile_definitions(_WIN32_WINNT=0x0601)
  if(CMAKE_SIZEOF_VOID_P EQUAL 4)
    add_compile_options(-march=i686)
  endif()
  add_compile_options(-fpermissive -fno-tree-slp-vectorize -Wno-error=cast-qual -Wno-unused-local-typedefs)
endif()


# ###########################

# exeternal dependency projects
add_subdirectory(external)

# Add sub projects
add_subdirectory(util)
add_subdirectory(steerlib)
add_subdirectory(steersimlib)
add_subdirectory(steersim)
add_subdirectory(simpleAI)
add_subdirectory(socialForcesAI)
add_subdirectory(rvo2AI)
add_subdirectory(pprAI)
add_subdirectory(steerbench)
add_subdirectory(kdtree)

# Navmeshbuilder requires opengl / visualiastion (gpecifically glut)
if(ENABLE_GUI AND ENABLE_GLFW)
  add_subdirectory(navmeshBuilder)
endif()

# Add the documentaiton
add_subdirectory(documentation)

# ###########################

install(DIRECTORY testcases DESTINATION share)

# ###########################

# @todo OSX specific settings... Not sure how I can test this (without public CI..?)

# @todo - reactiveAI isnt' in premake or in cmake?
# @todo - steertool is not in premake or in cmake 

# @todo - install rule(s) replicating premake 4
# @todo - rpath. 
# @todo - osx consolemode, -install_name 
# @todo - enforce clang on osx?
# @todo - if windows and gl* packages not found, use vcpkg to install locally for x86/x64 depending on current generator?
# @todo - make tinyxml/glfw dynamic via cmake?
# @todo - fix warnigns / warning levels rather than suppressing 
# @todo - create common cmake file / macros to reduce duplication.
# @todo - update readme / instructions
# @todo - make includes etc per target as required, rather than uniformly applied. Would support public/private specifications.
# @todo - add test_cases to visual studio? Probably best to just add them to an existing project, steerbench/steersim?