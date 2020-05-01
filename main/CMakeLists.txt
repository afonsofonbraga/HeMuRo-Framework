set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(${Framework_SOURCE_DIR}/src/core)

find_package (Threads)

link_directories(${Framework_SOURCE_DIR}/src/core)

add_executable(Framework main.cpp)
target_link_libraries(Framework core)
target_link_libraries (Framework ${CMAKE_THREAD_LIBS_INIT})
