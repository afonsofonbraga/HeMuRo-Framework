
find_package (Threads)


include_directories(${framework_SOURCE_DIR}/src/Core)
include_directories(${framework_SOURCE_DIR}/src/Communication)
include_directories(${framework_SOURCE_DIR}/src/Task)
include_directories(${framework_SOURCE_DIR}/src/Task/Default)
include_directories(${framework_SOURCE_DIR}/src/Mission)

link_directories(${framework_SOURCE_DIR}/src/Core)
link_directories(${framework_SOURCE_DIR}/src/Communication)
link_directories(${framework_SOURCE_DIR}/src/Task)
link_directories(${framework_SOURCE_DIR}/src/Task/Default)
link_directories(${framework_SOURCE_DIR}/src/Mission)


if(DEFAULT)
    include_directories(${framework_SOURCE_DIR}/src/Alive)
    link_directories(${framework_SOURCE_DIR}/src/Alive)
endif()

if(MAVROS)
    include_directories(${framework_SOURCE_DIR}/src/Task/Mavros)
    link_directories(${framework_SOURCE_DIR}/src/Task/Mavros)
endif()

if(TURTLEBOT)
    include_directories(${framework_SOURCE_DIR}/src/Task/Turtlebot)
    link_directories(${framework_SOURCE_DIR}/src/Task/Turtlebot)
endif()

if(ROSBOT)
    include_directories(${framework_SOURCE_DIR}/src/Task/Rosbot)
    link_directories(${framework_SOURCE_DIR}/src/Task/Rosbot)
endif()


add_executable(framework main.cpp)
target_link_libraries(framework core)
target_link_libraries(framework communication)
target_link_libraries(framework task)
#target_link_libraries(Framework mission)
target_link_libraries (framework ${CMAKE_THREAD_LIBS_INIT})

target_link_libraries(framework alive)
target_link_libraries(framework ${catkin_LIBRARIES})
#add_dependencies(framework beginner_tutorials_generate_messages_cpp)
