//
//  datatypes.h
//  MRSFramework
//
//  Created by Afonso Braga on 04/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef dataTypes_h
#define dataTypes_h
#include <iostream>

#ifdef DEFAULT
#include "../Task/Default/dataTask.hpp"
#endif

#ifndef DEFAULT
#include "dataTask.hpp"
#endif

template<typename T>
std::ostream& operator<<(typename std::enable_if<std::is_enum<T>::value, std::ostream>::type& stream, const T& e)
{
    return stream << static_cast<typename std::underlying_type<T>::type>(e);
}


enum class Operation{null, setRobotsPosition, missionMessage, batteryMessage};
enum class enum_RobotCategory{null, uav, ugv, usv};


enum class enum_AtomicTaskStatus{null, waiting, running, completed};
enum class enum_MissionRequest{null, waitingBids, notifyingWinner, executingMission, missionComplete};
enum class enum_MissionExecution{null, waitingAuction, waitingStart, executing, missionComplete};
enum class enum_MissionOperation{null, createMission, addMission, addAndRequestCost, Bid, abortMission, winningBid, acceptMission , startMission, emergency, missionComplete};

enum class enum_ChargingRequest{null, ok, chargingRequest, notfyingWinner, goingToLocation, charging, chargingComplete};
enum class enum_ChargingService{null, waitingRequest, bid, waitingForArrival, charging, chargingComplete};
enum class enum_ChargingOperation{null, chargingRequest, bid, winningBid, acceptRequest, arrivedAtStation, startCharging, chargingComplete, atomicTaskInterrupt};


struct s_pose
{
    float x = 0;
    float y = 0;
    float z = 0;
    float roll = 0;
    float pitch = 0;
    float yaw = 0;
    //float theta = 0;
};

struct s_robotsPose
{
    char robotName[10] = "null";
    s_pose position;
};

struct s_UDPMessage
{
    char address[16] = "null";
    char name[10] = "null";
    char buffer[500] = "null";
    int messageSize = 0;
};

struct s_MissionMessage
{
    char missionCode[10] = "null";
    char senderAddress[16] = "null";
    char senderName[10] = "null";
    enum_MissionOperation operation = enum_MissionOperation::null;
    enum_DecomposableTask taskToBeDecomposed = enum_DecomposableTask::null;
    float Cost = 0;
    //char buffer[500] = "null";
    s_pose goal;
    enum_RobotCategory robotCat;
    int executionTime;
    
};

struct s_BatteryMessage
{
    char requestID[10] = "null";
    char spotID[10] = "null";
    char senderAddress[16] = "null";
    char senderName[10] = "null";
    enum_ChargingOperation operation = enum_ChargingOperation::null;
    float Cost = 0;
    //char buffer[500] = "null";
    s_pose position;
    enum_RobotCategory robotCat;
    int executionTime;
    
};

struct s_ROSBridgeMessage
{
    char topicName[30] = "null";
    char buffer[500]="null";
};

struct s_cmdvel
{
    float x;
    float theta;
};
#endif /* datatypes_h */
