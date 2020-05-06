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

    
    
public:
    std::condition_variable conditional_task;
    
    
    BlackBoard(std::string& name); //Constructor
    ~BlackBoard(); // Destructor
    BlackBoard(const BlackBoard& other); // Copy Constructor
    BlackBoard& operator=(const BlackBoard& other); //Copy Assignment
    
    //Robot's description
    void getRobotsName(std::string& name);
    
    void chargeBattery(float energy);
    void consumeBattery(float energy);
    float getBaterryLevel();
    
    //Robot's Position Functions
    void getPositionAssignment(s_pose& p);
    void getPosition(s_pose& p);
    void setPosition(s_pose& p);
    
    //Functions for global positions
    void setAllRobotsPosition(s_robotsPose& p);
    void getAllRobotsPosition(std::unordered_map<std::string, s_pose>& p);
    void removeAllRobotsPosition(s_robotsPose& p);
    
    // Map Related Functions
    void setMapCoodinates(std::array<float,2>& coord);
    void getMapCoodinates(std::array<float,2>& coord);
    
    // Tasks functions
    bool isTaskListEmpty();
    void addTask(Task& vTask);
    void getTask(Task& vTask);
};

#endif
