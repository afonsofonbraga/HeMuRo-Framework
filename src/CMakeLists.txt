#subdirs(Core)
#subdirs(Communication)
#subdirs(Task)
#subdirs(Mission)

include_directories(${Framework_SOURCE_DIR}/src/Communication)
include_directories(${Framework_SOURCE_DIR}/src/Core)
include_directories(${Framework_SOURCE_DIR}/src/Task)
include_directories(${Framework_SOURCE_DIR}/src/Mission)

add_library(core Core/BlackBoard.cpp Core/Module.cpp Core/ModulePeriodic.cpp)
add_library(communication Core/BlackBoard.cpp Core/Module.cpp Core/ModulePeriodic.cpp Communication/UDPBroadcast.cpp Communication/UDPReceiver.cpp Communication/UDPSender.cpp)
add_library(task Core/BlackBoard.cpp Core/Module.cpp Core/ModulePeriodic.cpp Task/Task.cpp Task/TaskManager.cpp Task/AtomicTask.cpp Task/DecomposableTask.cpp Task/GoTo.cpp Task/ChargeBattery.cpp Task/TurnOn.cpp Mission/MissionManager.cpp)
