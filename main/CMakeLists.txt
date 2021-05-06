
find_package (Threads)

FIND_PACKAGE(Wt REQUIRED Wt HTTP)
ADD_DEFINITIONS(-DHPDF_DLL)


include_directories(${framework_SOURCE_DIR}/src/Core)
include_directories(${framework_SOURCE_DIR}/src/Energy)
include_directories(${framework_SOURCE_DIR}/src/Communication)
include_directories(${framework_SOURCE_DIR}/src/AtomicTasks)
include_directories(${framework_SOURCE_DIR}/src/Agents)
include_directories(${framework_SOURCE_DIR}/src/Agents/Default)
include_directories(${framework_SOURCE_DIR}/src/Mission)
include_directories(${framework_SOURCE_DIR}/src/Web)
include_directories(${framework_SOURCE_DIR}/src/Agents/ChargingStation)
include_directories(${framework_SOURCE_DIR}/src/Agents/Logger)
include_directories(${framework_SOURCE_DIR}/src/Agents/P3DX)
include_directories(${framework_SOURCE_DIR}/src/Agents/Rosbot)
include_directories(${framework_SOURCE_DIR}/src/Agents/Mavros)

link_directories(${framework_SOURCE_DIR}/src/Core)
link_directories(${framework_SOURCE_DIR}/src/Energy)
link_directories(${framework_SOURCE_DIR}/src/Communication)
link_directories(${framework_SOURCE_DIR}/src/AtomicTasks)
link_directories(${framework_SOURCE_DIR}/src/Agents)
link_directories(${framework_SOURCE_DIR}/src/Agents/Default)
link_directories(${framework_SOURCE_DIR}/src/Mission)
link_directories(${framework_SOURCE_DIR}/src/Agents/ChargingStation)
link_directories(${framework_SOURCE_DIR}/src/Agents/Logger)
link_directories(${framework_SOURCE_DIR}/src/Web)
link_directories(${framework_SOURCE_DIR}/src/Agents/Rosbot)
link_directories(${framework_SOURCE_DIR}/src/Agents/P3DX)
link_directories(${framework_SOURCE_DIR}/src/Agents/Mavros)

add_executable(framework warehouse.cpp)
target_link_libraries(framework core)
target_link_libraries(framework communication)
target_link_libraries(framework mission)
#target_link_libraries(framework atomicTasks)
#target_link_libraries(framework task)
target_link_libraries(framework default atomicTasks)
target_link_libraries(framework rosbot atomicTasks)
target_link_libraries(framework p3dx atomicTasks)
target_link_libraries(framework coexclover atomicTasks)
target_link_libraries(framework logger)
#target_link_libraries(framework mission)
target_link_libraries (framework ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(framework stdc++fs)

target_link_libraries(framework
    Wt::Wt
    Wt::HTTP
)

#target_link_libraries(framework alive)
target_link_libraries(framework ${catkin_LIBRARIES})
#add_dependencies(framework beginner_tutorials_generate_messages_cpp)
