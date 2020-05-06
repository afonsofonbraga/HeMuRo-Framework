include_directories(${Framework_SOURCE_DIR}/src/Communication)
include_directories(${Framework_SOURCE_DIR}/src/Core)
include_directories(${Framework_SOURCE_DIR}/src/Task)

add_library(communication UDPBroadcast.cpp UDPReceiver.cpp)
