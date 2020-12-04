//
//  BlackBoard.hpp
//  MRSFramework
//
//  Created by Afonso Braga on 01/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

/*! \class BlackBoard
 *  \author Afonso Braga
 *  \date 2020
 * \brief This class implements a Monitor with all the global variables.
 *
 * This class implements a Monitor for sharing variables between modules.
 *
 * All the information regarding the agent must be placed here. Mutexes need to be implemented to prevent data corruption.
 *
 * Each agent MUST have one instatiation of this class.
 */

#ifndef blackBoard_hpp
#define blackBoard_hpp

#include <iostream>
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

#include "dataTypes.hpp"

class BlackBoard {
protected:
    
    //Robots Description
    std::string                     agentName;                                      /*!< Name of the agent. */
    char                            agentIP[MAX_IP];                                /*!< IP Address of the agent*/
    char                            broadcastIP[MAX_IP];                            /*!< IP Address for broadcasting a message*/
    enum_RobotCategory              agentCategory= enum_RobotCategory::null;        /*!< Agent Type  */
    
    void                            setAgentsIP();                                  /*!< \brief Set Agent's IP when the agent is created.
                                                                                     * This method will be executed only once and is implemented for linux and Mac OS
                                                                                     */
    void                            setAgentsName(std::string name);                /*!< \brief Set the name of the agent.
                                                                                     * \param name a std::string.
                                                                                     * \warning This method will be executed only once inside the constructor and the name MUST be unique!!!!
                                                                                     */
    
    float                           batteryLevel;                                   /*!< Battery level. */
    std::mutex                      mutex_battery;                                  /*!< Mutex related to batteryLevel access. */
    
    //Chrono debug
    std::chrono::time_point<std::chrono::high_resolution_clock> start;              /*!< Deprecated Start timer. */
    std::chrono::time_point<std::chrono::high_resolution_clock> end;                /*!< Deprecated End timer. */
    long                            duration;                                       /*!< Deprecated Duration. */
    
    // Robot's Position Variables
    s_pose                          position;                                       /*!< Global Position of the Agent*/
    std::mutex                      mutex_position;                                 /*!< Mutex from position */
    std::unordered_map<std::string, s_BroadcastMessage> mapAgentsPosition;          /*!< All agents' position will be stored here*/
    std::mutex                      mutex_mapAgentsPosition;                        /*!< Mutex from mapRobotsPosition */
    
    
    // Map Related Variables
    std::array<float,2>             mapSize = {0.0,0.0};                            /*!< Deprecated test */
    std::mutex                      mutex_map;                                      /*!< Deprecated  test*/
    // Obstacles

    // Decomposable Tasks
    
    std::mutex                      mutex_decomposableTask;                         /*!< Mutex from decomposableTaskAvaliable */
    std::unordered_map<enum_DecomposableTask, std::vector<enum_AtomicTask> > decomposableTaskAvailable; /*!< List of all tasks available to be decomposed into atomicTasks */
    
    // Task Messages
    std::mutex                      mutex_taskList;                                 /*!< Mutex from taskMessageList */
    std::vector<s_TaskMessage>      taskMessageList;                                /*!< TaskModule Messages Buffer */
    
    // Mission Messages
    // Mission List: Available Missions
    std::mutex                      mutex_missionList;                              /*!< Mutex from missionMessageList */
    std::vector<s_MissionMessage>   missionMessageList;                             /*!< MissionMessage Messages Buffer. */
    
    // Battery Messages
    std::mutex mutex_batteryList;                                                   /*!< Mutex from batteryMessageList */
    std::vector<s_BatteryMessage>   batteryMessageList;                             /*!< BatteryManager Messages Buffer */
    
    // Mission: Selected Mission to execute
    std::mutex                      mutex_mission;                                  /*!< Mutex from  agentStatus and executingMission.*/
    bool                            executingMission = false;                       /*!< True when robot is executing a mission, false otherwise.*/
    enum_RobotStatus                agentStatus = enum_RobotStatus::null;           /*!< Agent Status*/
    //MissionExecution selectedMission;
    
    // UDP Messages
    std::vector<s_UDPMessage>       UDPMessageList;                                 /*!< UDPSender Messages Buffer */
    std::mutex                      mutex_UDPMessageList;                           /*!< Mutex from UDPMessageList */
    
    // ROSBridge
    std::vector<s_ROSBridgeMessage> ROSBridgeMessageList;                           /*!< ROSBridge Messages Buffer */
    std::mutex                      mutex_ROSBridgeMessageList;                     /*!< Mutex from ROSBridgeMessageList */
    
    // Logger Messages
    std::vector<s_LoggerMessage>    loggerMessageList;                              /*!< Logger Messages Buffer */
    std::mutex                      mutex_loggerMessageList;                        /*!< Mutex from loggerMessageList */

public:
    std::condition_variable         conditional_UDPMessageList;                     /*!< Conditional variable. Releases Module for reading messages*/
    std::condition_variable         conditional_TaskMessageList;                    /*!< Conditional variable. Releases Module for reading messages*/
    std::condition_variable         conditional_MissionMessageList;                 /*!< Conditional variable. Releases Module for reading messages*/
    std::condition_variable         conditional_BatteryMessageList;                 /*!< Conditional variable. Releases Module for reading messages*/
    std::condition_variable         conditional_ROSBridgeMessageList;               /*!< Conditional variable. Releases Module for reading messages*/
    std::condition_variable         conditional_LoggerMessageList;                  /*!< Conditional variable. Releases Module for reading messages*/
    
    BlackBoard(std::string& name, enum_RobotCategory cat);                          /*!< Constructor */
    ~BlackBoard();                                                                  /*!< Destructor */
    BlackBoard(const BlackBoard& other);                                            /*!< Copy Constructor */
    BlackBoard& operator=(const BlackBoard& other);                                 /*!< Copy Assignment */
    
    //Robot's description
    void                            getRobotsName(std::string& name);               /*!< \brief Get the name of the Agent
                                                                                     * \param name as std::string will receive the agent's name.
                                                                                     */
    void                            getRobotsName(char& name);                      /*!< \brief Get the name of the Agent
                                                                                     * \param name as char[] will receive the agent's name.
                                                                                     */
    void                            setRobotCategory(enum_RobotCategory cat);       /*!< Set the category of the Agent
                                                                                     *\param cat as enum_AgentCategory
                                                                                     */
    enum_RobotCategory              getRobotsCategory();                            /*!< Get the category of the Agent
                                                                                     \return the category of the robot as enum_AgentCategory
                                                                                     */
    
    void                            getRobotsIP(char& vIP);                         /*!< Get the IP address of the Agent
                                                                                     \param vIP a char[].
                                                                                     */
    void                            getBroadcastIP(char& vBroadcast);               /*!< Get the IP Broadcast address
                                                                                     \param vBroadcast a char[] will receive the broadcast IP.
                                                                                     */
    
    void                            chargeBattery(float energy);                    /*!< Add the value to the total amount of available energy
                                                                                     *\param energy a float with the energy to be added.
                                                                                     */
    void                            consumeBattery(float energy);                   /*!< Remove the value from the total amount of available energy
                                                                                     *\param energy a float with the energy to be subtracted.
                                                                                     */
    void                            setBatteryLevel(float energy);                  /*!< Set Battery Level, external battery controller
                                                                                     *\param energy a float as battery level.
                                                                                     */
    float                           getBatteryLevel();                              /*!< Return the battery
                                                                                     *\return a float with current battery level.
                                                                                     */
    
    //Robot's Position Functions
    void                            getPosition(s_pose& p);                         /*!< Get the current robot's position
                                                                                     *\param p a s_pose will receive the agent's position.
                                                                                     */
    void                            setPosition(s_pose& p);                         /*!< Set the robot's current position
                                                                                     *\param p a s_pose with the actual pose of the agent.
                                                                                     */
    
    //Functions for global positions
    void                            setAllRobotsPosition(s_BroadcastMessage& p);    /*!< Add a robot's position to the list
                                                                                     *\param p a s_BroadcastMessage with the information of an Agent.
                                                                                     */
    void                            getAllRobotsPosition(std::unordered_map<std::string, s_BroadcastMessage>& p); /*!< Get all information from the robot's position list
                                                                                                                   *\param p an unordered_map with all the information related to the agents.
                                                                                                                   */
    void                            removeAllRobotsPosition(s_BroadcastMessage& p); /*!< Remove one item of the allRobotsPositionList
                                                                                     * \param p a s_BroadcastMessage with the name of the Agent that will be removed of the list.
                                                                                     */
    
    // Decomposable Tasks
    
    void                            addDecomposableTaskList(enum_DecomposableTask vTaskToBeDecomposed, std::vector<enum_AtomicTask> vAtomicTask); /*!< \brief Add a decomposableTask to the list with their respective atomicTasks sequence.
                                                                                                                                                   *\param vTaskToBeDecomposed an enum_Decomposable task previously defined in DataTypes.h
                                                                                                                                                   *\param vAtomicTask a vector with the sequence of atomicTasks.
                                                                                                                                                   */
    bool                            getDecomposableTask(enum_DecomposableTask vTaskToBeDecomposed, std::vector<enum_AtomicTask>& vAtomicTask); /*!< \brief A function that receives a decomposableTask and if there is a previous sequence of atomicTasks for this decomposableTask it will be added to the vAtomicTask.
                                                                                                                                                *\param vTaskToBeDecomposed a enum_DecomposableTask previously defined
                                                                                                                                                *\param vAtomicTask a vector<enum_AtomicTask> where the sequence of atomicTasks will be stored.
                                                                                                                                                *\return true when the action is successfuly achieved, false otherwise.
                                                                                                                                                */
    bool                            isDecomposable(enum_DecomposableTask vTaskToBeDecomposed); /*!< \brief Returns if there is a sequence of atomicTasks for the decomposableTask
                                                                                                *\param vTaskToBeDecomposed an enum_DecomposableTask.
                                                                                                *\return true if there is an actual sequence, false otherwise.
                                                                                                */
    /*
    float                           getDecomposableTaskCost(enum_DecomposableTask vTaskToBeDecomposed);
    void                            acceptDecomposableTask(enum_DecomposableTask vDecomposableTask);
    */
    // Selected Mission
    
    bool                            isMissionCompleted();                              /*!< \brief A function that return if the mission is completed.
                                                                                        * \warning NOT IMPLEMENTED YET!!!!
                                                                                        * \return true when the mission is complete, false otherwise.
                                                                                        */
    
    bool                            isRobotAvailable();                                /*!< \brief A function that return if the robot is available to execute a mission.
                                                                                        *  \return true when robot is available, false otherwise.
                                                                                        */
    bool                            lockRobot(enum_RobotStatus statusRequest);         /*!< \brief A function that locks the robot to execute an action and retruns its status.
                                                                                        *  \param statusRequest as enum_RobotStatus. This variable contains the requested state for the robot.
                                                                                        *  \return true when the locking is successfull, false otherwise.
                                                                                        */
    bool                            unlockRobot();                                     /*!< \brief A function that releases the robot.
                                                                                        *  \return true when the unlocking is successfull, false otherwise.
                                                                                        */
    enum_RobotStatus                getRobotStatus();                                  /*!< \brief A function that return the agent's current status.
                                                                                        *  \return enum_RobotStatus.
                                                                                        */
    bool                            setRobotStatus(enum_RobotStatus statusRequest);    /*!< \brief A function that return the status of the robot.
                                                                                        *  \param statusRequest as the state the user wants to be.
                                                                                        *  \return true when the locking is successfull, false otherwise.
                                                                                        */
    //bool                            freeRobotStatus(enum_RobotStatus statusRequest);

    // Task Messages
    bool                            isTaskMessageListEmpty();                       /*!< \brief A function that returns the status of the taskMessageList Buffer.
                                                                                     *  \return true if the buffer is empty, false otherwise.
                                                                                     */
    void                            addTaskMessage(s_TaskMessage& vTaskMessage);    /*!< \brief A method responsible for adding a TaskMessage to the taskMessageList Buffer.
                                                                                     *  \param vTaskMessage a s_TaskMessage that will be added to the taskMessageList.
                                                                                     */
    void                            getTaskMessage(s_TaskMessage& vTaskMessage);    /*!< \brief A method responsible for getting one TaskMessage of the taskMessageList Buffer.
                                                                                     *  \param vTaskMessage a s_TaskMessage where the values will be written on.
                                                                                     */
    
    
    // Mission Messages
    bool                            isMissionMessageListEmpty();                          /*!< \brief A function that returns the status of the missionMessageList Buffer.
                                                                                           *  \return true if the buffer is empty, false otherwise.
                                                                                           */
    void                            addMissionMessage(s_MissionMessage& vMissionMessage); /*!< \brief A method responsible for adding a MissionMessage to the missionMessageList Buffer.
                                                                                           *  \param vMissionMessage a s_MissionMessage that will be added to the missionMessageList.
                                                                                           */
    void                            getMissionMessage(s_MissionMessage& vMissionMessage); /*!< \brief A method responsible for getting one MissionMessage of the missionMessageList Buffer.
                                                                                           *  \param vMissionMessage a s_MissionMessage where the values will be written on.
                                                                                           */
    
    // Battery Messages
    bool                            isBatteryMessageListEmpty();                          /*!< \brief A function that returns the status of the batteryMessageList Buffer.
                                                                                           *  \return true if the buffer is empty, false otherwise.
                                                                                           */
    void                            addBatteryMessage(s_BatteryMessage& vBatteryMessage); /*!< \brief A method responsible for adding a BatteryMessage to the batteryMessageList Buffer.
                                                                                           *  \param vBatteryMessage a s_BatteryMessage that will be added to the batteryMessageList.
                                                                                           */
    void                            getBatteryMessage(s_BatteryMessage& vBatteryMessage); /*!< \brief A method responsible for getting one BatteryMessage of the batteryMessageList Buffer.
                                                                                           *  \param vBatteryMessage a s_BatteryMessage where the values will be written on.
                                                                                           */
    
    // Messages to be sent
    bool                            isUDPMessageListEmpty();                              /*!< \brief A function that returns the status of the UDPMessageList Buffer.
                                                                                           *  \return true if the buffer is empty, false otherwise.
                                                                                           */
    void                            addUDPMessage(s_UDPMessage& vUDPMessage);             /*!< \brief A method responsible for adding an UDPMessage to the UDPMessageList Buffer.
                                                                                           *  \param vUDPMessage a s_UDPMessage that will be added to the UDPMessageList.
                                                                                           */
    void                            getUDPMessage(s_UDPMessage& vUDPMessage);             /*!< \brief A method responsible for getting one UDPMessage of the UDPMessageList Buffer.
                                                                                           *  \param vUDPMessage a s_UDPMessage where the values will be written on.
                                                                                           */
    
    
    // ROSBridge Messages
    bool                            isROSBridgeMessageListEmpty();                              /*!< \brief A function that returns the status of the ROSBridgeMessageList Buffer.
                                                                                                 *  \return true if the buffer is empty, false otherwise.
                                                                                                 */
    void                            addROSBridgeMessage(s_ROSBridgeMessage& vROSBridgeMessage); /*!< \brief A method responsible for adding a ROSBridgeMessage to the ROSBridgeMessageList Buffer.
                                                                                                 *  \param vROSBridgeMessage a s_ROSBridgeMessage that will be added to the ROSBridgeMessageList.
                                                                                                 */
    void                            getROSBridgeMessage(s_ROSBridgeMessage& vROSBridgeMessage); /*!< \brief A method responsible for getting one ROSBridgeMessage of the ROSBridgeMessageList Buffer.
                                                                                                 *  \param vROSBridgeMessage a s_ROSBridgeMessage where the values will be written on.
                                                                                                 */
    
    // Logger and Printing Messages
    bool                            isLoggerMessageListEmpty();                            /*!< \brief A function that returns the status of the LoggerMessageList Buffer.
                                                                                            *  \return true if the buffer is empty, false otherwise.
                                                                                            */
    void                            addLoggerMessage(s_LoggerMessage& vLoggerMessage);     /*!< \brief A method responsible for adding a LoggerMessage to the LoggerMessageList Buffer.
                                                                                            *  \param vLoggerMessage a s_LoggerMessage that will be added to the LoggerMessageList.
                                                                                            */
    void                            getLoggerMessage(s_LoggerMessage& vLoggerMessage);     /*!< \brief A method responsible for getting one LoggerMessage of the LoggerMessageList Buffer.
                                                                                            *  \param vLoggerMessage a s_LoggerMessage where the values will be written on.
                                                                                            */
    void                            print(std::string vText);                              /*!< \brief A method responsible for printing a text to the terminal.
                                                                                            *  \param vText a string.
                                                                                            */
    
    
    //NOT USED YET
    
    // Map Related Functions
    void                            setMapCoodinates(std::array<float,2>& coord);      // Set Map MAX Dimensions
    void                            getMapCoodinates(std::array<float,2>& coord);      // Get Map MAX Dimensions
};

#endif

