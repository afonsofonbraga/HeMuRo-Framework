//
//  main.cpp
//  MRSMac
//
//  Created by Afonso Braga on 01/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include <chrono>
#include <thread>
#include <vector>
#include <iostream>
#include <string.h>
#include <unordered_map>

#include "dataTypes.hpp"
#include "BlackBoard.hpp"
#include "Module.hpp"
#include "ModulePeriodic.hpp"

#include "UDPBroadcast.hpp"
#include "UDPReceiver.hpp"
#include "UDPSender.hpp"
#include "MissionManager.hpp"
#include "Alive.hpp"

#include "AtomicTask.hpp"

int main( int argc, char *argv[ ] ){
    
    if (argc != 2)
    {
        std::cerr << "Please, inform the robot's name:";
        return 0;
    }
    
    std::string name{argv[1]};
    enum_RobotCategory cat = enum_RobotCategory::null;
    
#ifdef MAVROS
    cat = enum_RobotCategory::uav;
#endif
    
#ifdef ROSBOT
    cat = enum_RobotCategory::ugv;
#endif
    
    BlackBoard* memory = new BlackBoard(name, cat);
    UDPBroadcast* broadcast = new UDPBroadcast(memory);
    UDPReceiver* receiver = new UDPReceiver(memory);
    UDPSender* sender = new UDPSender(memory);
    MissionManager* missionManager = new MissionManager(memory);
    
#ifndef DEFAULT
    std::string node = name;
    ros::init(argc, argv ,node);
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(0);
    spinner.start();
    
    Alive* alive = new Alive(memory, n);
#endif
    
#ifdef DEFAULT
    Alive* alive = new Alive(memory);
#endif
    
    while (std::getchar() != 'c'){}
    return 0;
}



