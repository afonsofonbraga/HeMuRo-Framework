include_directories(${Framework_SOURCE_DIR}/src/Task)
include_directories(${Framework_SOURCE_DIR}/src/Core)
add_library(task Task.cpp TaskManager.cpp AtomicTask.cpp DecomposableTask.cpp GoTo.cpp ChargeBattery.cpp TurnOn.cpp)
