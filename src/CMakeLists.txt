#subdirs(Core)
#subdirs(Communication)
#subdirs(Task)
#subdirs(Mission)
#subdirs(Alive)

include_directories(${framework_SOURCE_DIR}/src/Communication)
include_directories(${framework_SOURCE_DIR}/src/Core)
include_directories(${framework_SOURCE_DIR}/src/Task)
include_directories(${framework_SOURCE_DIR}/src/Mission)
include_directories(${framework_SOURCE_DIR}/src/Task/Default)
include_directories(${framework_SOURCE_DIR}/src/Task/Mavros)
#include_directories(${framework_SOURCE_DIR}/src/Alive)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(core Core/BlackBoard.cpp Core/Module.cpp Core/ModulePeriodic.cpp)
add_library(communication Core/BlackBoard.cpp Core/Module.cpp Core/ModulePeriodic.cpp Communication/UDPBroadcast.cpp Communication/UDPReceiver.cpp Communication/UDPSender.cpp)
add_library(task Core/BlackBoard.cpp Core/Module.cpp Core/ModulePeriodic.cpp Task/AtomicTask.cpp Task/Mavros/Arm.cpp Task/Mavros/GoTo.cpp Task/Mavros/Land.cpp Task/Mavros/TakeOff.cpp Task/Default/ChargeBattery.cpp Task/Default/TurnOn.cpp Task/Default/TakePicture.cpp Mission/MissionManager.cpp Mission/Mission.cpp Mission/MissionRequest.cpp Mission/MissionExecution.cpp)
add_library(alive ${catkin_LIBRARIES} Core/BlackBoard.cpp Task/Mavros/Alive.cpp)
