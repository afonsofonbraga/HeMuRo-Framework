//
//  datatypes.h
//  MRSFramework
//
//  Created by Afonso Braga on 04/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef dataTypes_h
#define dataTypes_h

enum class Operation{null, setRobotsPosition, missionMessage};
enum class enum_RobotCategory{null, uav, ugv, usv};

enum class enum_AtomicTaskStatus{null, waiting, running, completed};
enum class enum_AtomicTask{null, chargeBattery, turnOn, goTo, takePicture};
enum class enum_DecomposableTask{null, checkPosition, lowBattery, takePicture}; //Trocar por DecomposableMission


enum class enum_MissionRequest{null, waitingBids, notifyingWinner, executingMission, missionComplete};
enum class enum_MissionExecution{null, waitingAuction, waitingStart, executing, missionComplete};
enum class enum_MissionOperation{null, createMission, addMission, addAndRequestCost, Bid, abortMission, winningBid, acceptMission , startMission, emergency, missionComplete};


template<typename T>
std::ostream& operator<<(typename std::enable_if<std::is_enum<T>::value, std::ostream>::type& stream, const T& e)
{
    return stream << static_cast<typename std::underlying_type<T>::type>(e);
}


struct s_pose
{
  float x = 0;
  float y = 0;
  float theta = 0;
};

struct s_robotsPose
{
    char robotName[10] = "null";
    s_pose position;
};

struct s_UDPMessage
{
    char address[16] = "null";
    char buffer[500] = "null";
    int messageSize = 0;
};

struct s_MissionMessage
{
    char missionCode[5] = "null";
    char senderAddress[16] = "null";
    enum_MissionOperation operation = enum_MissionOperation::null;
    enum_DecomposableTask taskToBeDecomposed = enum_DecomposableTask::null;
    float Cost = 0;
    //char buffer[500] = "null";
    s_pose goal;
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
