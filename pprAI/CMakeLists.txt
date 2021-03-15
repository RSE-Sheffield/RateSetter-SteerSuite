set(PPRAI_INCLUDE
    "${CMAKE_CURRENT_SOURCE_DIR}/include/PPRAgent.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/PPRAIModule.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/PPRParameters.h")
set(PPRAI_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/PPRAgent.cpp"
              "${CMAKE_CURRENT_SOURCE_DIR}/src/PPRAIModule.cpp")
set(PPRAI_INCLUDE_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../external"
    "${CMAKE_CURRENT_SOURCE_DIR}/../steerlib/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../util/include")

add_library(pprAI SHARED ${PPRAI_SRC} ${PPRAI_INCLUDE})
target_include_directories(pprAI PRIVATE ${PPRAI_INCLUDE_DIRS})
target_link_libraries(pprAI steerlib util)

install(
  TARGETS pprAI
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(FILES ${PPRAI_INCLUDE} DESTINATION include/pprAI)
