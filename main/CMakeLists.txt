include_directories(${Framework_SOURCE_DIR}/src/Core)
include_directories(${Framework_SOURCE_DIR}/src/Communication)
include_directories(${Framework_SOURCE_DIR}/src/Task)

find_package (Threads)

link_directories(${Framework_SOURCE_DIR}/src/core)

add_executable(Framework main.cpp)
target_link_libraries(Framework core)
target_link_libraries(Framework communication)
target_link_libraries(Framework task)
target_link_libraries (Framework ${CMAKE_THREAD_LIBS_INIT})

#set_target_properties(Framework PROPERTIES
 #   CXX_STANDARD 17
  #  CXX_STANDARD_REQUIRED ON
   # CXX_EXTENSIONS ON
#)
