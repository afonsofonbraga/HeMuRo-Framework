include_directories(${Framework_SOURCE_DIR}/src/Core)
include_directories(${Framework_SOURCE_DIR}/src/Communication)
include_directories(${Framework_SOURCE_DIR}/src/Task)
include_directories(${Framework_SOURCE_DIR}/src/Mission)

find_package (Threads)

link_directories(${Framework_SOURCE_DIR}/src/Core)
link_directories(${Framework_SOURCE_DIR}/src/Communication)
link_directories(${Framework_SOURCE_DIR}/src/Task)
link_directories(${Framework_SOURCE_DIR}/src/Mission)

add_executable(Framework main.cpp)
target_link_libraries(Framework core)
target_link_libraries(Framework communication)
target_link_libraries(Framework task)
#target_link_libraries(Framework mission)
target_link_libraries (Framework ${CMAKE_THREAD_LIBS_INIT})
