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



struct s_pose
{
  float x = 0;
  float y = 0;
  float theta = 0;
};

struct s_robotsPose
{
    std::string robotName;
    s_pose position;
};


class BlackBoard {
protected:
    
    //Robots Description
    std::string robotName;
    float batteryLevel;
    void setRobotsName(std::string* name);
    
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
    
    // Obstacles
    
    
public:
    BlackBoard(std::string& name); //Constructor
    ~BlackBoard(); // Destructor
    BlackBoard(const BlackBoard& other); // Copy Constructor
    BlackBoard& operator=(const BlackBoard& other); //Copy Assignment
    
    //Robot's description
    void getRobotsName(std::string& name);
    
    //Robot's Position Functions
    void getPositionAssignment(s_pose& p);
    void getPosition(s_pose& p);
    void setPosition(s_pose& p);
    
    //Functions for global positions
    void setAllRobotsPosition(s_robotsPose& p);
    void getAllRobotsPosition(std::unordered_map<std::string, s_pose>& p);
    void removeAllRobotsPosition(s_robotsPose& p);
    
};

#endif
