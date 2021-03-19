set(UTIL_INCLUDE
    "${CMAKE_CURRENT_SOURCE_DIR}/include/astar/AStarLite.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/astar/AStarLiteClose.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/astar/AStarLiteNode.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/astar/AStarLiteOpen.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/astar/Environment.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/LogData.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/Logger.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/LogManager.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/LogObject.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/PluginAPI.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/UtilGlobals.h")
set(UTIL_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/src/AStarLite.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/AStarLiteClose.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/AStarLiteNode.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/AStarLiteOpen.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/LogData.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Logger.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/LogManager.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/LogObject.cpp")
set(UTIL_INCLUDE_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

add_library(util SHARED ${UTIL_SRC} ${UTIL_INCLUDE})
target_include_directories(util PRIVATE ${UTIL_INCLUDE_DIRS})

install(TARGETS util
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include/util)

# Construct source groups for IDEs
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/include PREFIX include FILES ${UTIL_INCLUDE})
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/src PREFIX src FILES ${UTIL_SRC})