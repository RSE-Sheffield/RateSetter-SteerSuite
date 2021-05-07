set(STEERLIB_INCLUDE
    "${CMAKE_CURRENT_SOURCE_DIR}/include/util/CommandLineParser.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/util/XMLParser.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/util/ThreadedTaskManager.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/util/MemoryMapper.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/util/Mutex.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/util/DynamicLibrary.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/util/HighResCounter.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/util/PerformanceProfiler.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/util/DrawLib.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/util/XMLParserPrivate.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/util/GenericException.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/util/StateMachine.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/util/Misc.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/util/Geometry.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/util/Color.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/util/dmatrix.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/util/FrameSaver.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/Globals.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/Util.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/griddatabase/GridCell.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/griddatabase/GridDatabase2D.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/griddatabase/GridDatabasePlanningDomain.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/griddatabase/GridDatabase2DPrivate.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/SteerLib.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/recfileio/RecFileIO.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/recfileio/RecFileIOPrivate.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/testcaseio/TestCaseIOPrivate.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/testcaseio/Behaviour.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/testcaseio/TestCaseIO.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/testcaseio/BehaviourParameter.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/testcaseio/AgentInitialConditions.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/testcaseio/ObstacleInitialConditions.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/obstacles/CircleObstacle.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/obstacles/OrientedBoxObstacle.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/obstacles/PolygonObstacle.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/obstacles/BoxObstacle.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/obstacles/OrientedWallObstacle.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/benchmarking/CompositeTechniqueGraph.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/benchmarking/EnergyEfficiencyTechnique.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/benchmarking/MetricsData.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/benchmarking/CompositeTechnique02.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/benchmarking/BenchmarkEngine.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/benchmarking/SimulationMetricsCollector.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/benchmarking/CompositeTechniquePLE.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/benchmarking/AgentMetricsCollector.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/benchmarking/CompositeTechniqueEntropy.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/benchmarking/CompositeTechnique01.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/benchmarking/BayesianFilter.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/benchmarking/BenchmarkEnginePrivate.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/benchmarking/AccelerationEfficiencyTechnique.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/interfaces/EngineControllerInterface.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/interfaces/PlanningDomainInterface.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/interfaces/ModuleInterface.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/interfaces/SpatialDataBaseModuleInterface.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/interfaces/SpatialDatabaseItem.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/interfaces/ObstacleInterface.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/interfaces/SpatialDataBaseInterface.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/interfaces/BenchmarkTechniqueInterface.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/interfaces/PlanningDomainModuleInterface.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/interfaces/AgentInterface.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/interfaces/EngineInterface.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/SimulationPlugin.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/simulation/SimulationEngine.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/simulation/SimulationOptions.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/simulation/Camera.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/simulation/SteeringCommand.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/simulation/Clock.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/SteerSimPlugin.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/modules/MetricsCollectorModule.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/modules/SimulationRecorderModule.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/modules/TestCasePlayerModule.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/modules/RecFilePlayerModule.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/modules/SteerBugModule.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/modules/DummyAIModule.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/modules/SteerBenchModule.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/planning/BestFirstSearchPlanner.h")
set(STEERLIB_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/src/PerformanceProfiler.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/OrientedWallObstacle.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/SteeringCommand.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Camera.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Misc.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/PolygonObstacle.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/BehaviorParameter.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/XMLParser.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/OrientedBoxObstacle.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/SimulationRecorderModule.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/CompositeTechniqueEntropy.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/CompositeTechniqueGraph.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/StateMachine.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/GridDatabasePlanningDomain.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Behaviour.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/DrawLib.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/RecFileReader.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/CircleObstacle.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/CompositeTechniquePLE.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/BenchmarkEngine.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/HighResCounter.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/TestCaseReader.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/RecFilePlayerModule.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/TestCaseReaderPrivate.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Clock.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/BoxObstacle.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/AgentInterface.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/GridDatabase2D.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/SimulationOptions.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/MemoryMapper.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/SimulationEngine.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/CommandLineParser.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/ThreadedTaskManager.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/SimulationMetricsCollector.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/CompositeTechnique01.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/CompositeTechnique02.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/RecFileWriter.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/AgentMetricsCollector.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/DynamicLibrary.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/FrameSaver.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/TestCasePlayerModule.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/TestCaseWriter.cpp")

set(STEERLIB_INCLUDE_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/util"
    "${CMAKE_CURRENT_SOURCE_DIR}/../external"
    "${CMAKE_CURRENT_SOURCE_DIR}/../util/include")

add_library(steerlib SHARED ${STEERLIB_SRC} ${STEERLIB_INCLUDE})
target_include_directories(steerlib PRIVATE ${STEERLIB_INCLUDE_DIRS})
target_link_libraries(steerlib tinyxml util)
if(ENABLE_GUI)
    target_link_libraries(steerlib OpenGL::GL OpenGL::GLU GLUT::GLUT)
    target_include_directories(steerlib PUBLIC "${GLUT_INCLUDE_DIR}")
    # GLEW is not actually required...
    # target_link_libraries(steerlib GLEW::GLEW)
endif()

# For portable configuration files, steerlib now needs to know the subdirectory to check in if the config file does not directly point to the dll/so folder.
target_compile_definitions(steerlib PRIVATE CMAKE_BUILD_TYPE_STR="$<CONFIG>")

install(
  TARGETS steerlib
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
# @todo - install rules
install(DIRECTORY include/ DESTINATION include/steerlib)
install(DIRECTORY ../external/mersenne DESTINATION include/steerlib)


# Construct source groups for IDEs
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/include PREFIX include FILES ${STEERLIB_INCLUDE})
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/src PREFIX src FILES ${STEERLIB_SRC})
