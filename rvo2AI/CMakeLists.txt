file(GLOB RVO2AI_SRC src/*.cpp)
file(GLOB RVO2AI_HDR include/*.h)

add_library(rvo2dAI SHARED ${RVO2AI_SRC} ${RVO2AI_HDR})
target_include_directories(rvo2dAI PRIVATE
  ./include
  ../external
  ../steerlib/include
  ../util/include
  ../kdtree/include
)
target_link_libraries(rvo2dAI steerlib util)
add_dependencies(rvo2dAI steerlib util)

install(TARGETS rvo2dAI
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
install(FILES ${RVO2AI_HDR} DESTINATION include/rvo2AI)
