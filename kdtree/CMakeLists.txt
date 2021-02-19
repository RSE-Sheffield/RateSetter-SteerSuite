set(KDTREE_INCLUDE
    "${CMAKE_CURRENT_SOURCE_DIR}/include/KdTreeDataBase.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/KdTree.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/Obstacle.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/KdTreeDataBaseModule.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/Definitions.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/Vector2.h")
set(KDTREE_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/src/KdTree.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/KdTreeDataBase.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Obstacle.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/KdTreeDataBaseModule.cpp")
set(KDTREE_INCLUDE_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../external"
    "${CMAKE_CURRENT_SOURCE_DIR}/../steerlib/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../util/include")

add_library(kdtree MODULE ${KDTREE_SRC} ${KDTREE_INCLUDE})
target_include_directories(kdtree PRIVATE ${KDTREE_INCLUDE_DIRS})
target_link_libraries(kdtree steerlib util)

install(
  TARGETS kdtree
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

# @todo - osx link options "-install_name @rpath/libkdtree.dylib"

# Construct source groups for IDEs
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/include PREFIX include FILES ${KDTREE_INCLUDE})
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/src PREFIX src FILES ${KDTREE_SRC})