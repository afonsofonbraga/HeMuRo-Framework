//
//  BlackBoard.hpp
//  MRSFramework
//
//  Created by Afonso Braga on 01/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef blackBoard_hpp
#define blackBoard_hpp

#include <iostream>
//#include <string>
#include <string.h> //Não sei pq tem que colocar string.h no linux e pq string não funciona
#include <thread>
#include <chrono>
#include <unordered_map>
#include <mutex>
#include <array>
#include <vector>
#include <condition_variable>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <ifaddrs.h>

//#include "AtomicTask.hpp"
//#include "MissionExecution.hpp"
#include "dataTypes.hpp"

class BlackBoard {
protected:
    
    //Robots Description
    std::string                     robotName;
    char                            robotIP[MAX_IP];
    char                            broadcastIP[MAX_IP];
    enum_RobotCategory              robotCategory= enum_RobotCategory::null;
    
    void                            setRobotIP();
    void                            setRobotsName(std::string name);
    
    float                           batteryLevel;
    std::mutex                      mutex_battery;
    
    //Chrono debug
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
    std::chrono::time_point<std::chrono::high_resolution_clock> end;
    long                            duration;
    
    // Robot's Position Variables
    s_pose                          position;
    std::mutex                      mutex_position;
    std::unordered_map<std::string, s_BroadcastMessage> mapRobotsPosition;
    std::mutex                      mutex_mapRobotsPosition;
    
    
    // Map Related Variables
    std::array<float,2>             mapSize = {0.0,0.0};
    std::mutex                      mutex_map;
    // Obstacles

    // Decomposable Tasks
    
    std::mutex                      mutex_decomposableTask;
    std::unordered_map<enum_DecomposableTask, std::vector<enum_AtomicTask> > decomposableTaskAvaliable;
    
    // Mission Messages
    // Mission List: Available Missions
    std::mutex                      mutex_missionList;
    std::vector<s_MissionMessage>   missionMessageList;
    
    // Battery Messages
    std::mutex mutex_batteryList;
    std::vector<s_BatteryMessage>   batteryMessageList;
    
    // Mission: Selected Mission to execute
    std::mutex                      mutex_mission;
    bool                            executingMission = false;
    enum_RobotStatus                robotStatus = enum_RobotStatus::null;
    //MissionExecution selectedMission;
    
    // UDP Messages
    std::vector<s_UDPMessage>       UDPMessageList;
    std::mutex                      mutex_UDPMessageList;
    
    // ROSBridge
    std::vector<s_ROSBridgeMessage> ROSBridgeMessageList;
    std::mutex                      mutex_ROSBridgeMessageList;
    
    // Logger Messages
    std::vector<s_LoggerMessage>    loggerMessageList;
    std::mutex                      mutex_loggerMessageList;

public:
    std::condition_variable         conditional_UDPMessageList;
    std::condition_variable         conditional_MissionMessageList;
    std::condition_variable         conditional_BatteryMessageList;
    std::condition_variable         conditional_ROSBridgeMessageList;
    std::condition_variable         conditional_LoggerMessageList;
    
    BlackBoard(std::string& name, enum_RobotCategory cat);                          // Constructor
    ~BlackBoard();                                          // Destructor
    BlackBoard(const BlackBoard& other);                    // Copy Constructor
    BlackBoard& operator=(const BlackBoard& other);         // Copy Assignment
    
    //Robot's description
    void                            getRobotsName(std::string& name);                  // Get the name of the Robot
    void                            getRobotsName(char& name);                         // Get the name of the Robot
    void                            setRobotCategory(enum_RobotCategory cat);
    enum_RobotCategory              getRobotsCategory();                               // Get the category of the Robot
    
    void                            getRobotsIP(char& vIP);                            // Get the IP address of the Robot
    void                            getBroadcastIP(char& vBroadcast);                  // Get the IP Broadcast address
    
    void                            chargeBattery(float energy);                       // Add the value to the total amount of available energy
    void                            consumeBattery(float energy);                      // Remove the value from the total amount of available energy
    void                            setBatteryLevel(float energy);                     // Set Battery Level, external battery controller
    float                           getBatteryLevel();                                 // Return the battery
    
    //Robot's Position Functions
    void                            getPosition(s_pose& p);                            // Get the current robot's position
    void                            setPosition(s_pose& p);                            // Set the robot's current position
    
    //Functions for global positions
    void                            setAllRobotsPosition(s_BroadcastMessage& p);       // Add a robot's position to the list
    void                            getAllRobotsPosition(std::unordered_map<std::string, s_BroadcastMessage>& p); // Get all positions from the robot's position list
    void                            removeAllRobotsPosition(s_BroadcastMessage& p);    // Clean the list
    
    // Decomposable Tasks
    
    void                            addDecomposableTaskList(enum_DecomposableTask vTaskToBeDecomposed, std::vector<enum_AtomicTask> vAtomicTask);
    bool                            getDecomposableTask(enum_DecomposableTask vTaskToBeDecomposed, std::vector<enum_AtomicTask>& vAtomicTask);
    bool                            isDecomposable(enum_DecomposableTask vTaskToBeDecomposed);
    float                           getDecomposableTaskCost(enum_DecomposableTask vTaskToBeDecomposed);
    void                            acceptDecomposableTask(enum_DecomposableTask vDecomposableTask);
    
    // Selected Mission
    
    bool                            isMissionCompleted();                              // NAO ESTA IMPLEMENTADA
    bool                            isRobotAvailable();
    bool                            lockRobot(enum_RobotStatus statusRequest);
    bool                            unlockRobot();
    enum_RobotStatus                getRobotStatus();
    bool                            setRobotStatus(enum_RobotStatus statusRequest);
    bool                            freeRobotStatus(enum_RobotStatus statusRequest);


    // Mission Messages
    bool                            isMissionMessageListEmpty();
    void                            addMissionMessage(s_MissionMessage& vMissionMessage);
    void                            getMissionMessage(s_MissionMessage& vMissionMessage);
    
    // Battery Messages
    bool                            isBatteryMessageListEmpty();
    void                            addBatteryMessage(s_BatteryMessage& vBatteryMessage);
    void                            getBatteryMessage(s_BatteryMessage& vBatteryMessage);
    
    // Messages to be sent
    bool                            isUDPMessageListEmpty();
    void                            addUDPMessage(s_UDPMessage& vUDPMessage);
    void                            getUDPMessage(s_UDPMessage& vUDPMessage);
    
    
    // ROSBridge Messages
    bool                            isROSBridgeMessageListEmpty();
    void                            addROSBridgeMessage(s_ROSBridgeMessage& vROSBridgeMessage);
    void                            getROSBridgeMessage(s_ROSBridgeMessage& vROSBridgeMessage);
    
    // Logger and Printing Messages
    bool                            isLoggerMessageListEmpty();
    void                            addLoggerMessage(s_LoggerMessage& vLoggerMessage);
    void                            getLoggerMessage(s_LoggerMessage& vLoggerMessage);
    void                            print(std::string vText);
    
    //NOT USED YET
    
    // Map Related Functions
    void                            setMapCoodinates(std::array<float,2>& coord);      // Set Map MAX Dimensions
    void                            getMapCoodinates(std::array<float,2>& coord);      // Get Map MAX Dimensions
    
    //WILL BE IMPLEMENTED IN MISSION MODULE
    //void addMissionToExecute(MissionExecution& vMission);
    //void startMissionExecution();
    //std::shared_ptr<AtomicTask> getTaskFromMission();
    //void cancelMission();
};

#endif

