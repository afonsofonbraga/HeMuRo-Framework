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
#include <string>
#include <thread>
#include <chrono>
#include <unordered_map>
#include <mutex>
#include <array>
#include <vector>
#include <condition_variable>

#include "Task.hpp"
#include "dataTypes.hpp"

class BlackBoard {
protected:
    
    //Robots Description
    std::string robotName;
    void setRobotsName(std::string* name);
    
    float batteryLevel;
    std::mutex mutex_battery;
    
    //Chrono debug
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
    std::chrono::time_point<std::chrono::high_resolution_clock> end;
    long duration;
    
    // Robot's Position Variables
    s_pose position;
    std::mutex mutex_position;
    
    std::mutex mutex_mapRobotsPosition;
    std::unordered_map<std::string, s_pose> mapRobotsPosition;
    
    
    // Map Related Variables
    std::array<float,2> mapSize = {0.0,0.0};
    std::mutex mutex_map;
    // Obstacles
    
    // Tasks
    std::vector<Task> taskList;
    std::mutex mutex_task;


    // UDP Messages
    std::vector<s_UDPMessage> UDPMessageList;
    std::mutex mutex_UDPMessageList;

    
public:
        std::condition_variable conditional_UDPMessageList;
        std::condition_variable conditional_task;
    
    BlackBoard(std::string& name);                          // Constructor
    ~BlackBoard();                                          // Destructor
    BlackBoard(const BlackBoard& other);                    // Copy Constructor
    BlackBoard& operator=(const BlackBoard& other);         // Copy Assignment
    
    //Robot's description
    void getRobotsName(std::string& name);                  // Get the name of the Robot
    
    void chargeBattery(float energy);                       // Add the value to the total amount of available energy
    void consumeBattery(float energy);                      // Remove the value from the total amount of available energy
    float getBatteryLevel();                                // Return the battery
    
    //Robot's Position Functions
    void getPositionAssignment(s_pose& p);                  // This will be soon excluded
    void getPosition(s_pose& p);                            // Get the current robot's position
    void setPosition(s_pose& p);                            // Set the robot's current position
    
    //Functions for global positions
    void setAllRobotsPosition(s_robotsPose& p);             // Add a robot's position to the list
    void getAllRobotsPosition(std::unordered_map<std::string, s_pose>& p); // Get all positions from the robot's position list
    void removeAllRobotsPosition(s_robotsPose& p);          // Clean the list
    
    // Map Related Functions
    void setMapCoodinates(std::array<float,2>& coord);      // Set Map MAX Dimensions
    void getMapCoodinates(std::array<float,2>& coord);      // Get Map MAX Dimensions
    
    // Tasks functions
    bool isTaskListEmpty();                                 // Return if the list of ToDo tasks is empty
    void addTask(Task& vTask);                              // Add task to the ToDo list
    void getTask(Task& vTask);                              // Get the first task from the ToDo list
    
    // Messages to be sent
    bool isUDPMessageListEmpty();
    void addUDPMessage(s_UDPMessage& vUDPMessage);
    void getUDPMessage(s_UDPMessage& vUDPMessage);
    
};

#endif
