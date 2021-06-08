set(RVO2AI_INCLUDE
    "${CMAKE_CURRENT_SOURCE_DIR}/include/API.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/RVO2DAgent.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/RVO2DAIModule.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/RVO2D_Parameters.h")
set(RVO2AI_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/RVO2DAgent.cpp"
               "${CMAKE_CURRENT_SOURCE_DIR}/src/RVO2DAIModule.cpp")
set(RVO2AI_INCLUDE_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../external"
    "${CMAKE_CURRENT_SOURCE_DIR}/../steerlib/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../util/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../kdtree/include")

add_library(rvo2dAI MODULE ${RVO2AI_SRC} ${RVO2AI_INCLUDE})
target_include_directories(rvo2dAI PRIVATE ${RVO2AI_INCLUDE_DIRS})
target_link_libraries(rvo2dAI steerlib util)
install(
  TARGETS rvo2dAI
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
install(FILES ${RVO2AI_INCLUDE} DESTINATION include/rvo2AI)

# Construct source groups for IDEs
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/include PREFIX include FILES ${RVO2AI_INCLUDE})
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/src PREFIX src FILES ${RVO2AI_SRC})