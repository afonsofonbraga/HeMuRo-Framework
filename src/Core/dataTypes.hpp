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
#include <list>
#include <queue>


#ifdef DEFAULT
#include "../Task/Default/dataTask.hpp"
#endif

#ifndef DEFAULT
#include "dataTask.hpp"
#endif

#define MAX_ROBOT_ID 10
#define MAX_ID 13
#define MAX_IP 16



template<typename T>
std::ostream& operator<<(typename std::enable_if<std::is_enum<T>::value, std::ostream>::type& stream, const T& e)
{
    return stream << static_cast<typename std::underlying_type<T>::type>(e);
}


enum class Operation{null, setRobotsPosition, missionMessage, batteryMessage, loggerMessage};
enum class enum_RobotCategory{null, uav, ugv, usv, chargingStation};


enum class enum_AtomicTaskStatus{null, waiting, running, completed};
enum class enum_MissionRequest{null, waitingBids, notifyingWinner, executingMission, missionComplete};
enum class enum_MissionExecution{null, waitingAuction, waitingStart, executing, missionComplete};
enum class enum_MissionOperation{null, createMission, addMission, addAndRequestCost, Bid, abortMission, winningBid, acceptMission , startMission, emergency, missionComplete};

enum class enum_ChargingRequest{null, ok, chargingRequest, notfyingWinner, goingToLocation, charging, chargingComplete};
enum class enum_ChargingService{null, waitingRequest, bid, waitingForArrival, charging, chargingComplete};
enum class enum_ChargingOperation{null, chargingRequest, bid, winningBid, acceptRequest, arrivedAtStation, startCharging, chargingComplete, atomicTaskInterrupt};

enum class enum_LoggerOperation{null, print, save, printAndSave};


enum class enum_RobotStatus{null, available, executing, failure, lowBattery, emergency};

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
    char robotName[MAX_ROBOT_ID] = "null";
    s_pose position;
};

struct s_UDPMessage
{
    char address[MAX_IP] = "null";
    char name[MAX_ROBOT_ID] = "null";
    char buffer[600] = "null";
    int messageSize = 0;
};

struct s_BroadcastMessage
{
    char robotName[MAX_ROBOT_ID] = "null";
    enum_RobotCategory robotCategory = enum_RobotCategory::null;
    enum_RobotStatus robotStatus = enum_RobotStatus::null;
    s_pose robotsPosition;
    float batteryLevel = 0;
};

struct s_MissionMessage
{
    char missionCode[MAX_ID] = "null";
    char senderAddress[MAX_IP] = "null";
    char senderName[MAX_ROBOT_ID] = "null";
    enum_MissionOperation operation = enum_MissionOperation::null;
    enum_DecomposableTask taskToBeDecomposed = enum_DecomposableTask::null;
    float Cost = 0;
    //char buffer[500] = "null";
    enum_RobotCategory robotCat = enum_RobotCategory::null;
    int executionTime = 0;
    int numberOfAttributes = 0;
    char attributesBuffer[500] = "null";
    s_pose goal;
};

struct s_BatteryMessage
{
    char requestID[MAX_ID] = "null";
    char spotID[MAX_ROBOT_ID] = "null";
    char senderAddress[MAX_IP] = "null";
    char senderName[MAX_ROBOT_ID] = "null";
    enum_ChargingOperation operation = enum_ChargingOperation::null;
    float Cost = 0;
    //char buffer[500] = "null";
    s_pose position;
    enum_RobotCategory robotCat;
    int executionTime;
    
};

struct s_LoggerMessage
{
    char robotName[MAX_ROBOT_ID] = "null";
    char buffer[400] = "null";
    enum_LoggerOperation operation = enum_LoggerOperation::null;
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
