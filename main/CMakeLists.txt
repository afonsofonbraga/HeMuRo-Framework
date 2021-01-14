
find_package (Threads)


include_directories(${framework_SOURCE_DIR}/src/Core)
include_directories(${framework_SOURCE_DIR}/src/Communication)
include_directories(${framework_SOURCE_DIR}/src/AtomicTasks)
include_directories(${framework_SOURCE_DIR}/src/Agents/Default)
include_directories(${framework_SOURCE_DIR}/src/Mission)
include_directories(${framework_SOURCE_DIR}/src/Agents/ChargingStation)
include_directories(${framework_SOURCE_DIR}/src/Agents/Logger)

link_directories(${framework_SOURCE_DIR}/src/Core)
link_directories(${framework_SOURCE_DIR}/src/Communication)
link_directories(${framework_SOURCE_DIR}/src/AtomicTasks)
link_directories(${framework_SOURCE_DIR}/src/Agents/Default)
link_directories(${framework_SOURCE_DIR}/src/Mission)
link_directories(${framework_SOURCE_DIR}/src/Agents/ChargingStation)
link_directories(${framework_SOURCE_DIR}/src/Agents/Logger)


if(DEFAULT)
#    include_directories(${framework_SOURCE_DIR}/src/Alive)
#    link_directories(${framework_SOURCE_DIR}/src/Alive)
endif()

if(MAVROS)
    include_directories(${framework_SOURCE_DIR}/src/Agents/Mavros)
    link_directories(${framework_SOURCE_DIR}/src/Agents/Mavros)
endif()

if(TURTLEBOT)
    include_directories(${framework_SOURCE_DIR}/src/Agents/Turtlebot)
    link_directories(${framework_SOURCE_DIR}/src/Agents/Turtlebot)
endif()

if(ROSBOT)
    include_directories(${framework_SOURCE_DIR}/src/Agents/Rosbot)
    link_directories(${framework_SOURCE_DIR}/src/Agents/Rosbot)
endif()


add_executable(framework main.cpp)
target_link_libraries(framework core)
target_link_libraries(framework communication)
target_link_libraries(framework task)
#target_link_libraries(framework mission)
target_link_libraries (framework ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(framework stdc++fs)

#target_link_libraries(framework alive)
target_link_libraries(framework ${catkin_LIBRARIES})
#add_dependencies(framework beginner_tutorials_generate_messages_cpp)
