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
#include <string>
#include <iostream>
#include <string.h>
#include <unordered_map>

#include "dataTypes.hpp"
#include "Blackboard.hpp"
#include "UDPReceiverSim.hpp"
#include "LoggerAgent.hpp"


#ifdef DEFAULT
    #include "DefaultRobot.hpp"
#endif
    
#ifdef MAVROS
    #include "MavrosRobot.hpp"
#endif
    
#ifdef ROSBOT
    #include "RosbotRobot.hpp"
#endif
    
#ifdef TRUTLEBOT
    #include "TurtlebotRobot.hpp"
#endif

int main( int argc, char *argv[ ] )
{
    
    if (argc < 2)
    {
        std::cerr << "Please, inform the robot's name:";
        return 0;
    }
    
    std::vector<Blackboard *> v_Blackboard; // = new std::vector<Blackboard>;
    UDPReceiverSim* receiver = new UDPReceiverSim();
    
#ifdef DEFAULT
    std::vector<DefaultRobot* > v_Robot;
#endif
    
#ifdef MAVROS
    std::vector<MavrosRobot* > v_Robot;
#endif
    
#ifdef ROSBOT
    std::vector<RosbotRobot* > v_Robot;
#endif
    
#ifdef TRUTLEBOT
    std::vector<TurtlebotRobot* > v_Robot;
#endif
    
#ifndef DEFAULT
    std::string node = "node"; //name;
    ros::init(argc, argv ,node);
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(0);
    spinner.start();
#endif
    
    std::string robotsName = "Logger";
    Blackboard* memory = new Blackboard(robotsName, enum_RobotCategory::null);
    v_Blackboard.push_back(memory);
    receiver->addRobot(v_Blackboard.at(0));
    bool decentralizedCommunication = false;
    LoggerAgent* logger = new LoggerAgent(v_Blackboard.at(0), decentralizedCommunication);

    for (int i = 1; i < argc; i++)
    {
        robotsName = argv[i];
        Blackboard* memory = new Blackboard(robotsName, enum_RobotCategory::null);
        v_Blackboard.push_back(memory);
        receiver->addRobot(v_Blackboard.at(i));
        decentralizedCommunication = false;
#ifdef DEFAULT
        DefaultRobot* robot = new DefaultRobot(v_Blackboard.at(i), decentralizedCommunication);
#endif
        
#ifdef MAVROS
        MavrosRobot* robot = new MavrosRobot(v_Blackboard.at(i), n , decentralizedCommunication);
#endif
        
#ifdef ROSBOT
        RosbotRobot* robot = new RosbotRobot(v_Blackboard.at(i), n, decentralizedCommunication);
#endif
        
#ifdef TRUTLEBOT
        TurtlebotRobot* robot = new TurtlebotRobot(v_Blackboard.at(i), n, decentralizedCommunication);
#endif
        v_Robot.push_back(robot);
    }
    while (std::getchar() != 'c'){}
    
    //delete logger;
    return 0;
}



