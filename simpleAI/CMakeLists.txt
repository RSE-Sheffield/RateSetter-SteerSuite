set(SIMPLEAI_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/include/SimpleAgent.h"
                     "${CMAKE_CURRENT_SOURCE_DIR}/include/SimpleAIModule.h")
set(SIMPLEAI_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/SimpleAgent.cpp"
                 "${CMAKE_CURRENT_SOURCE_DIR}/src/SimpleAIModule.cpp")
set(SIMPLEAI_INCLUDE_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../external"
    "${CMAKE_CURRENT_SOURCE_DIR}/../steerlib/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../util/include")

add_library(simpleAI MODULE ${SIMPLEAI_SRC} ${SIMPLEAI_INCLUDE})
target_include_directories(simpleAI PRIVATE ${SIMPLEAI_INCLUDE_DIRS})
target_link_libraries(simpleAI steerlib util)

install(
  TARGETS simpleAI
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
install(FILES ${SIMPLEAI_INCLUDE} DESTINATION include/simpleAI)

# Construct source groups for IDEs
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/include PREFIX include FILES ${SIMPLEAI_INCLUDE})
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/src PREFIX src FILES ${SIMPLEAI_SRC})