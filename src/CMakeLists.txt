#subdirs(Core)
#subdirs(Communication)
#subdirs(Task)
#subdirs(Mission)
#subdirs(Alive)

include_directories(${framework_SOURCE_DIR}/src/Communication)
include_directories(${framework_SOURCE_DIR}/src/Core)
include_directories(${framework_SOURCE_DIR}/src/Task)
include_directories(${framework_SOURCE_DIR}/src/Mission)

include_directories(${framework_SOURCE_DIR}/src/Task/ChargingStation)


if(DEFAULT)
    include_directories(${framework_SOURCE_DIR}/src/Task/Default)
endif()

if(MAVROS)
    include_directories(${framework_SOURCE_DIR}/src/Task/Mavros)
endif()

if(TURTLEBOT)
    include_directories(${framework_SOURCE_DIR}/src/Task/Turtlebot)
endif()

if(ROSBOT)
    include_directories(${framework_SOURCE_DIR}/src/Task/Rosbot)
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(core Core/BlackBoard.cpp Core/Module.cpp Core/ModulePeriodic.cpp Core/Logger.cpp Core/BatteryManager.cpp Task/ChargingStation/ChargingSpot.cpp Task/ChargingStation/ChargingStation.cpp Task/Logger/LoggerAgent.cpp)
add_library(communication Core/BlackBoard.cpp Core/Module.cpp Core/ModulePeriodic.cpp Communication/UDPBroadcast.cpp Communication/UDPReceiver.cpp Communication/UDPSender.cpp Communication/UDPReceiverSim.cpp)

if(DEFAULT)
add_library(task Task/Default/DefaultRobot.cpp Core/BlackBoard.cpp Core/Module.cpp Core/ModulePeriodic.cpp Mission/MissionManager.cpp Mission/Mission.cpp Mission/MissionRequest.cpp Mission/MissionExecution.cpp Task/AtomicTask.cpp Task/Default/DecomposableTasks.cpp Task/Default/GoTo.cpp Task/Default/ChargeBattery.cpp Task/Default/TurnOn.cpp Task/Default/TakePicture.cpp)
add_library(alive ${catkin_LIBRARIES} Core/BlackBoard.cpp Task/Alive/Alive.cpp)
endif()

if(MAVROS)
add_library(task Task/Mavros/MavrosRobot.cpp Core/BlackBoard.cpp Core/Module.cpp Core/ModulePeriodic.cpp Mission/MissionManager.cpp Mission/Mission.cpp Mission/MissionRequest.cpp Mission/MissionExecution.cpp Task/AtomicTask.cpp Task/Mavros/DecomposableTasks.cpp Task/Mavros/Arm.cpp Task/Mavros/GoTo.cpp Task/Mavros/Land.cpp Task/Mavros/TakeOff.cpp Task/Default/ChargeBattery.cpp Task/Default/TurnOn.cpp Task/Default/TakePicture.cpp )
add_library(alive ${catkin_LIBRARIES} Core/BlackBoard.cpp Task/Mavros/Alive.cpp)
endif()

if(TURTLEBOT)
add_library(task Task/Turtlebot/TurtlebotRobot.cpp Core/BlackBoard.cpp Core/Module.cpp Core/ModulePeriodic.cpp Mission/MissionManager.cpp Mission/Mission.cpp Mission/MissionRequest.cpp Mission/MissionExecution.cpp Task/AtomicTask.cpp Task/Turtlebot/DecomposableTasks.cpp Task/Turtlebot/GoTo.cpp Task/Default/ChargeBattery.cpp Task/Default/TurnOn.cpp Task/Default/TakePicture.cpp)
add_library(alive ${catkin_LIBRARIES} Core/BlackBoard.cpp Task/Turtlebot/Alive.cpp)
endif()

if(ROSBOT)
add_library(task Task/Rosbot/RosbotRobot.cpp Core/BlackBoard.cpp Core/Module.cpp Core/ModulePeriodic.cpp Mission/MissionManager.cpp Mission/Mission.cpp Mission/MissionRequest.cpp Mission/MissionExecution.cpp Task/AtomicTask.cpp Task/Rosbot/DecomposableTasks.cpp Task/Rosbot/GoTo.cpp Task/Rosbot/MoveBaseGoal.cpp Task/Rosbot/ChargeBattery.cpp Task/Default/TurnOn.cpp Task/Default/TakePicture.cpp)
add_library(alive ${catkin_LIBRARIES} Core/BlackBoard.cpp Task/Rosbot/Alive.cpp)
endif()
