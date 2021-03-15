set(SFAI_INCLUDE
    "${CMAKE_CURRENT_SOURCE_DIR}/include/SocialForcesAgent.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/SocialForcesAIModule.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/SocialForces_Parameters.h")
set(SFAI_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/SocialForcesAgent.cpp"
             "${CMAKE_CURRENT_SOURCE_DIR}/src/SocialForcesAIModule.cpp")
set(SFAI_INCLUDE_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../external"
    "${CMAKE_CURRENT_SOURCE_DIR}/../steerlib/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../util/include")

add_library(sfAI SHARED ${SFAI_SRC} ${SFAI_INCLUDE})
target_include_directories(sfAI PRIVATE ${SFAI_INCLUDE_DIRS})
target_link_libraries(sfAI steerlib util)

install(
  TARGETS sfAI
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
install(FILES ${SFAI_INCLUDE} DESTINATION include/sfAI)
