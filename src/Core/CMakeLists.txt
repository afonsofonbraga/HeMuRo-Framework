include_directories(${Framework_SOURCE_DIR}/src/Core)
include_directories(${Framework_SOURCE_DIR}/src/Task)
include_directories(${Framework_SOURCE_DIR}/src/Communication)

add_library(core Blackboard.cpp Module.cpp ModulePeriodic.cpp)
