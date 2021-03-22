# Enforce vaguley modern cmake. @todo - pick an actual version 
# 3.3  for CMAKE_ARCHIVE_OUTPUT_DIRECTORY_<CONFIG>
# 3.12 for add_compile_definitions.
# 3.14 for x11 imported targets?
cmake_minimum_required(VERSION 3.14...3.19)
include(CMakeDependentOption)
include(FetchContent)
# include(ExternalProject)
cmake_policy(SET CMP0075 NEW)
cmake_policy(SET CMP0077 NEW)
cmake_policy(SET CMP0079 NEW)


# This file is equivalent to the `solution` in premake4.
# implemented for out-of-tree builds (i.e. in the `build` directory)

# Set the defualt CMAKE BUILD TYPE to Release (for make-like generators)
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the build type")

project(
  steersuite 
  LANGUAGES C CXX)

# ###########################


# Set the location of output binaries and libraries, into /lib/bin/<config>. This should be standardised for both generator types.
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/$<CONFIG>)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/$<CONFIG>)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/$<CONFIG>)

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
option(ENABLE_WARNINGS "Enable compiler warnings" OFF)


# ###########################

# Find dependencies and set compiler flags based on options.
if(ENABLE_GUI)
  # OpenGL and GLUT are required regardless of GLFW/QT.
  # GLEW appears to not actually be required, as no extensions are used.
  set(OpenGL_GL_PREFERENCE "GLVND") # Or set to "LEGACY"?
  find_package(OpenGL REQUIRED)
  # find_package(GLEW REQUIRED)
  
  # On linux/osx find system installed glut as required.
  if(NOT WIN32)
    find_package(GLUT REQUIRED)
  else()
    # On windows, find glut if agailable, else download freeglut and make it a target.
    find_package(GLUT QUIET)
    # If GLUT could not be found, download freeglut, and try to find again. 
    if(NOT GLUT_FOUND)
      # Unset GLUT variables, so we can potentially search for it again.
      unset(GLUT_INCLUDE_DIR)
      unset(GLUT_LIBRARIES)
      unset(GLUT_FOUND)
      unset(GLUT_glut_LIBRARY)
      unset(GLUT_Xmu_LIBRARY)
      unset(GLUT_Xi_LIBRARY)

      # Chooses wheter to build freeglut at configure time or at build time.
      # Currently configure time still doesn't allow use of find_package(GLUT) so it's not really advantageous over building at build time.
      set(BUILD_FREEGLUT_CONFIGURE_TIME OFF)

      # Download a specific version of freeglut, checking the hash is correct.
      set(FREEGLUT_DOWNLOAD_VERSION "3.2.1")
      set(FREEGLUT_HASH "SHA512=aced4bbcd36269ce6f4ee1982e0f9e3fffbf18c94f785d3215ac9f4809b992e166c7ada496ed6174e13d77c0f7ef3ca4c57d8a282e96cbbe6ff086339ade3b08")
      FetchContent_Declare(
        freeglut
        URL      http://prdownloads.sourceforge.net/freeglut/freeglut-${FREEGLUT_DOWNLOAD_VERSION}.tar.gz
        URL_HASH ${FREEGLUT_HASH}
      )
      FetchContent_GetProperties(freeglut)
      if(NOT freeglut_POPULATED)
        message(STATUS "[freeglut] Downloading freeglut-${FREEGLUT_DOWNLOAD_VERSION}.tar.gz")
        # Download
        FetchContent_Populate(freeglut)        
        
        if(NOT BUILD_FREEGLUT_CONFIGURE_TIME)
          # Option 1: Add freeglut cmake project as a subfolder and therfore bring in the target (and variables?)
          # Build the stattic and dynamic libraries
          set(FREEGLUT_BUILD_STATIC_LIBS ON CACHE BOOL "")
          set(FREEGLUT_BUILD_SHARED_LIBS ON CACHE BOOL "")
          # Do not build the freeglut demos
          set(FREEGLUT_BUILD_DEMOS OFF CACHE BOOL "")
          # do not use gles
          set(FREEGLUT_GLES OFF CACHE BOOL "")
          # do not rename the dll glut(d).dll/lib, not neccesary.
          set(FREEGLUT_REPLACE_GLUT OFF CACHE BOOL "")

          # Use a folder in visual studio
          set(CMAKE_FOLDER "freeglut")
          # Add the cmake subdirectory, providing the freeglut target
          add_subdirectory(${freeglut_SOURCE_DIR} ${freeglut_BINARY_DIR})
          # Can't use find_package(Glut) as the dll/lib do not exist yet, but we can instead set the same values which findGlut will.
          if(TARGET freeglut)
            add_library(GLUT::GLUT ALIAS freeglut)
            set(GLUT_INCLUDE_DIR "${freeglut_SOURCE_DIR}/include") 
            set(GLUT_LIBRARIES "") # @todo - set a value?
            set(GLUT_FOUND "") # @todo - set a value?
          else()
            message(FATAL_ERROR "An error occured while downloading freeglut. Please manually install glut/freeglut.")
          endif()

          # Stop using the freeglut folder.
          set(CMAKE_FOLDER "")

        elseif(BUILD_FREEGLUT_CONFIGURE_TIME)
          # Option 2: alternative - build at configure time (need to predict debug / release? which would tehn allow find_package(glut) to work?
          message(STATUS "Configuring freeglut via CMake...")
          execute_process(
            COMMAND ${CMAKE_COMMAND} "${freeglut_SOURCE_DIR}" -DFREEGLUT_BUILD_DEMOS=OFF -DFREEGLUT_BUILD_STATIC_LIBS=ON -DFREEGLUT_BUILD_SHARED_LIBS=ON -DFREEGLUT_GLES=OFF -DFREEGLUT_REPLACE_GLUT=OFF
            WORKING_DIRECTORY "${freeglut_BINARY_DIR}"
            OUTPUT_QUIET
          )
          message(STATUS "Compiling freeglut Release ")
          execute_process(
            COMMAND ${CMAKE_COMMAND} --build . --config Release
            WORKING_DIRECTORY "${freeglut_BINARY_DIR}"
            OUTPUT_QUIET
          )
          message(STATUS "Compiling freeglut Debug ")
          execute_process(
            COMMAND ${CMAKE_COMMAND} --build . --config Debug
            WORKING_DIRECTORY "${freeglut_BINARY_DIR}"
            OUTPUT_QUIET
          )

          # Find the built version of freeglut.
          set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};${freeglut_BINARY_DIR};${freeglut_SOURCE_DIR}")
          find_package(FreeGLUT REQUIRED)
          # alias the target, and set the include dir as find_package(GLUT) still doesnt' agree with freeglut's file structure.
          add_library(GLUT::GLUT ALIAS FreeGLUT::freeglut)
          set(GLUT_INCLUDE_DIR "${freeglut_SOURCE_DIR}/include") 

          # This does not work
          # find_package(GLUT REQUIRED)
        endif()
      endif()
    endif()
  endif()

  # X11 is required by GLFW when X is in use, including xrandr.
  if(NOT WIN32 AND NOT APPLE)
    find_package(X11 COMPONENTS Xrandr REQUIRED)
  endif()

  # Add the compile definition.
  add_compile_definitions(ENABLE_GUI)

  if(ENABLE_GLFW)
    add_compile_definitions(ENABLE_GLFW GLFW_DLL)
  endif()

  # the ENABLE_QT functionality in SteerSuite seems partial, so not propperly
  # configuring this.
  if(ENABLE_QT)
    # find_package(Qt5Core) find_package(Qt5Gui) find_package(Qt5OpenGL)
    # if(${Qt5OpenGL_FOUND}) include_directories(${Qt5OpenGL_INCLUDE_DIRS})
    # add_definitions(-DENABLE_QT) endif()
  endif()
endif()

# C/C++ threads (pthread) are required by several compontents
find_package(Threads REQUIRED)

# If apple, we also need cocoa?
if(APPLE)
  find_library(COCOA_LIBRARY Cocoa REQUIRED)
  mark_as_advanced(COCOA_LIBRARY)
endif()
# ###########################

# Set a bunch of compiler settings. @todo - move these to a separate cmake file.
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMakeModules;${CMAKE_MODULE_PATH}")

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
    add_compile_options(-Wno-sign-compare -Wno-ignored-qualifiers -Wno-cast-qual -Wno-deprecated-declarations -Wno-format -Wno-unused-variable -Wno-misleading-indentation)
    # GCC only options
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      add_compile_options(-Wno-unused-but-set-variable -Wno-unused-but-set-parameter)
    endif()
    if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")

    endif()
  endif()
  if(EMSCRIPTEN)
    add_compile_options(-Wno-warn-absolute-paths)
  elseif(NOT APPLE)
    # add_compile_options()
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
# @todo - fix warnigns / warning levels rather than suppressing 
# @todo - create common cmake file / macros to reduce duplication.
# @todo - add test_cases to visual studio? Probably best to just add them to an existing project, steerbench/steersim?
