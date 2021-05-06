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
#include <chrono>

/*
#ifdef DEFAULT
#include "../Task/Default/dataTask.hpp"
#endif

#ifndef DEFAULT
#include "dataTask.hpp"
#endif
 */

#define MAX_ROBOT_ID 10
#define MAX_ID 13
#define MAX_IP 16


enum class enum_AtomicTask{null, chargeBattery, turnOn, goTo, moveBaseGoal, takePicture, arm, disarm, takeOff, land, pickUpSample, dropOffSample, measureTemperature,goToBasis};
//enum class enum_DecomposableTask{null, checkPosition, lowBattery, takePicture, flightTest, deliverPicture};

enum class enum_DecomposableTask{null, checkPosition, lowBattery, takePicture, flightTest, deliverPicture, deliverSmallSample, deliverBigSample, inspectPlace, measureTemperature, inspectArea, emergencyLanding};



template<typename T>
std::ostream& operator<<(typename std::enable_if<std::is_enum<T>::value, std::ostream>::type& stream, const T& e)
{
    return stream << static_cast<typename std::underlying_type<T>::type>(e);
}


enum class Operation{null, setRobotsPosition, missionMessage, batteryMessage, loggerMessage};
enum class enum_RobotCategory{null, uav, ugv, usv, chargingStation};


enum class enum_AtomicTaskStatus{null, waiting, running, completed};
enum class enum_TaskMessage{null, requestCost, addTask, executeTask, addAndExecute, redirect, addEmergency, failure};


enum class enum_MissionStatus{null, allocating, executing, complete, failure, aborted, timeout, lowBattery, chargingRequested, waitingArrival, charging, chargingCompleted, chargingCancelled};
enum class enum_MissionRequest{null, waitingBids, notifyingWinner, executingMission, missionComplete};
enum class enum_MissionExecution{null, waitingAuction, waitingStart, executing, missionComplete};
enum class enum_MissionOperation{null, createMission, addMission, addAndRequestCost, Bid, redirectRequest, abortMission, winningBid, lockingComplete, acceptMission , startMission, emergency, notifyMissionComplete, missionComplete};


enum class enum_ChargingRequest{null, ok, chargingRequest, notfyingWinner, goingToLocation, charging, chargingComplete};
enum class enum_ChargingService{null, waitingRequest, bid, waitingForArrival, charging, chargingComplete};
enum class enum_ChargingOperation{null, chargingRequest, bid, winningBid, acceptRequest, arrivedAtStation, startCharging, chargingComplete, abortChargingRequest, atomicTaskInterrupt};

enum class enum_LoggerOperation{null, print, save, printAndSave, missionStatus};


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
    enum_RobotCategory robotCat = enum_RobotCategory::null;
    std::chrono::milliseconds relativeDeadline = std::chrono::milliseconds(0);
    int numberOfAttributes = 0;
    char attributesBuffer[500] = "null";
};

struct s_TaskMessage
{
    char missionCode[MAX_ID] = "null";
    char senderAddress[MAX_IP] = "null";
    char senderName[MAX_ROBOT_ID] = "null";
    enum_TaskMessage operation = enum_TaskMessage::null;
    enum_DecomposableTask taskToBeDecomposed = enum_DecomposableTask::null;
    enum_RobotCategory robotCat = enum_RobotCategory::null;
    std::chrono::milliseconds relativeDeadline = std::chrono::milliseconds(0);
    float Cost = 0;
    int numberOfAttributes = 0;
    char attributesBuffer[500] = "null";
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
    std::chrono::milliseconds relativeDeadline = std::chrono::milliseconds(0);
};

struct s_ChargingRequest
{
    char requestID[MAX_ID]= "null";
    enum_ChargingOperation operation = enum_ChargingOperation::null;
    enum_RobotCategory robotCat;
    s_pose robotsPosition;
    char robotsAddress[MAX_IP] = "null";
    char robotsName[MAX_ROBOT_ID] = "null";
};

struct s_LoggerMessage
{
    char robotName[MAX_ROBOT_ID] = "null";
    char buffer[400] = "null";
    enum_LoggerOperation operation = enum_LoggerOperation::null;
};

struct s_ROSModuleMessage
{
    char topicName[30] = "null";
    char buffer[500]="null";
};

struct s_cmdvel
{
    float x;
    float theta;
};

// Beta
struct s_MissionStatus
{
    char missionCode[MAX_ID] = "null";
    char missionOwner[MAX_ROBOT_ID] = "null";
    char missionExecutioner[MAX_ROBOT_ID] = "null";
    //enum_DecomposableTask taskToBeDecomposed = enum_DecomposableTask::null;
    enum_MissionStatus status = enum_MissionStatus::null;
    std::chrono::milliseconds relativeDeadline = std::chrono::seconds(0);
    std::chrono::milliseconds executionTime = std::chrono::seconds(0);
    std::chrono::milliseconds estimatedExecutionTime = std::chrono::seconds(0);
};


#endif /* datatypes_h */
