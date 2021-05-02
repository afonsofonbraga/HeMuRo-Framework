#subdirs(Core)
#subdirs(Communication)
#subdirs(Task)
#subdirs(Mission)
#subdirs(Alive)

include_directories(${framework_SOURCE_DIR}/src/AtomicTasks)
include_directories(${framework_SOURCE_DIR}/src/Communication)
include_directories(${framework_SOURCE_DIR}/src/Core)
include_directories(${framework_SOURCE_DIR}/src/Energy)
include_directories(${framework_SOURCE_DIR}/src/Mission)
include_directories(${framework_SOURCE_DIR}/src/Agents)
include_directories(${framework_SOURCE_DIR}/src/Web)

include_directories(${framework_SOURCE_DIR}/src/Agents/ChargingStation)
include_directories(${framework_SOURCE_DIR}/src/Agents/Logger)
include_directories(${framework_SOURCE_DIR}/src/Agents/Default)
include_directories(${framework_SOURCE_DIR}/src/Agents/Rosbot)
include_directories(${framework_SOURCE_DIR}/src/Agents/P3DX)
include_directories(${framework_SOURCE_DIR}/src/Agents/Mavros)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(core
    Core/Blackboard.cpp
    Core/Module.cpp 
    Core/ModulePeriodic.cpp
    Core/Logger.cpp
    Energy/BatteryManager.cpp
    Agents/Agent.cpp
    Energy/ChargingSpot.cpp
    Agents/ChargingStation/ChargingStation.cpp
    #Agents/Logger/LoggerAgent.cpp
    )

add_library(communication
    Communication/UDPBroadcast.cpp
    Communication/UDPReceiver.cpp
    Communication/UDPSender.cpp
    Communication/UDPReceiverSim.cpp
    )

add_library(mission
    Mission/Auction.cpp
    Mission/TaskModule.cpp
    Mission/Mission.cpp
    Mission/MissionRequest.cpp
    Mission/MissionExecution.cpp
    )

add_library(atomicTasks 
    AtomicTasks/AtomicTask.cpp
    AtomicTasks/ArmMavROS.cpp
    AtomicTasks/ChargeBatteryROS.cpp
    AtomicTasks/ChargeBatterySim.cpp
    AtomicTasks/DisarmMavROS.cpp
    AtomicTasks/DropOffSim.cpp
    AtomicTasks/GoToROS.cpp
    AtomicTasks/GoToSim.cpp
    AtomicTasks/LandMavROS.cpp
    AtomicTasks/MeasureTemperatureSim.cpp
    AtomicTasks/MoveBaseGoal.cpp
    AtomicTasks/NavigateMavROS.cpp
    AtomicTasks/PickUpSim.cpp
    AtomicTasks/TakeOffMavROS.cpp
    AtomicTasks/TakePictureSim.cpp
    AtomicTasks/TurnOnSim.cpp
    )

add_library(default
    Agents/Agent.cpp
    Agents/Default/DefaultRobot.cpp
    )

add_library(rosbot
    Agents/Agent.cpp
    Agents/Rosbot/RosbotRobot.cpp
    Agents/Rosbot/ROSModuleRosbot.cpp
    )

add_library(p3dx
    Agents/Agent.cpp
    Agents/P3DX/P3DXRobot.cpp
    Agents/P3DX/ROSModuleP3DX.cpp
    )

add_library(coexclover
    Agents/Agent.cpp
    Agents/Mavros/MavrosRobot.cpp
    Agents/Mavros/ROSModuleMavros.cpp
    )

add_library(logger
    Agents/Agent.cpp  
    Agents/Logger/LoggerAgent.cpp
    Web/WebModule.cpp
    Web/WebApp.cpp
    )
