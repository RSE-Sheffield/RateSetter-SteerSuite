set(NAVMESH_INCLUDE
    "${CMAKE_CURRENT_SOURCE_DIR}/include/ChunkyTriMesh.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/ConvexVolumeTool.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/CrowdTool.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/imgui.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/imguiRenderGL.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/InputGeom.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/Mesh.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/MeshLoaderObj.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/NavMeshModule.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/NavMeshPruneTool.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/NavMeshTesterTool.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/OffMeshConnectionTool.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/opengl.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/PerfTimer.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/RecastNavMeshPlanner.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/Sample.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/SampleInterfaces.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/Sample_SoloMesh.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/stb_truetype.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/Timer.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/ValueHistory.h")
set(NAVMESH_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/src/ChunkyTriMesh.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/ConvexVolumeTool.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/CrowdTool.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/imgui.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/imguiRenderGL.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/InputGeom.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Main.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Mesh.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/MeshLoaderObj.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/NavMeshModule.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/NavMeshPruneTool.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/NavMeshTesterTool.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/OffMeshConnectionTool.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/PerfTimer.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/RecaseNavMeshPlanner.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Sample.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/SampleInterfaces.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Sample_SoloMesh.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Timer.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/ValueHistory.cpp")
set(NAVMESH_INCLUDE_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/./include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../external"
    "${CMAKE_CURRENT_SOURCE_DIR}/../external/recastnavigation/Recast/Include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../external/recastnavigation/DebugUtils/Include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../external/recastnavigation/Detour/Include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../external/recastnavigation/DetourTileCache/Include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../external/recastnavigation/DetourCrowd/Include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../steerlib/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../steersimlib/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../util/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../kdtree/include")

add_library(navmesh SHARED ${NAVMESH_SRC} ${NAVMESH_INCLUDE})
target_include_directories(navmesh PRIVATE ${NAVMESH_INCLUDE_DIRS})
target_link_libraries(
  navmesh
  steerlib
  steersimlib
  util
  Recast
  DebugUtils
  Detour
  DetourCrowd)

install(
  TARGETS navmesh
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

# @todo osx "-install_name @rpath/libnavmesh.dylib"
