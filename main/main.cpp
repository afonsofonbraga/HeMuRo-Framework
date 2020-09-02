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
#include "BlackBoard.hpp"
#include "Module.hpp"
#include "ModulePeriodic.hpp"

#include "UDPBroadcast.hpp"
#include "UDPReceiver.hpp"
#include "UDPReceiverSim.hpp"
#include "UDPSender.hpp"
#include "MissionManager.hpp"
#include "Alive.hpp"

#include "AtomicTask.hpp"

int main( int argc, char *argv[ ] ){
    
    if (argc <= 2)
    {
        std::cerr << "Please, inform the robot's name:";
        return 0;
    }
    
    //std::string name{argv[1]};
    //int numberOfRobots = std::stoi(argv[2]);
    enum_RobotCategory cat = enum_RobotCategory::null;
    
#ifdef MAVROS
    cat = enum_RobotCategory::uav;
#endif
    
#ifdef ROSBOT
    cat = enum_RobotCategory::ugv;
#endif
    
    
    std::vector<BlackBoard *> v_BlackBoard; // = new std::vector<BlackBoard>;
    std::vector<UDPBroadcast*> v_Broadcast;// = new std::vector<UDPBroadcast>;
    //std::vector<UDPReceiver*> v_Receiver;
    std::vector<UDPSender*> v_Sender;
    std::vector<MissionManager*> v_MissionManager;
    std::vector<Alive*> v_Alive;
    UDPReceiverSim* receiver = new UDPReceiverSim();
    
#ifndef DEFAULT
    std::string node = "node"; //name;
    ros::init(argc, argv ,node);
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(0);
    spinner.start();
#endif
    
    for (int i = 0; i < argc - 1; i++)
    {
        std::string robotsName = argv[i+1];
        BlackBoard* memory = new BlackBoard(robotsName, enum_RobotCategory::ugv);
        v_BlackBoard.push_back(memory);
        UDPBroadcast* broadcast = new UDPBroadcast(v_BlackBoard.at(i));
        //UDPReceiver* receiver = new UDPReceiver(v_BlackBoard.at(i));
        receiver->addRobot(v_BlackBoard.at(i));
        UDPSender* sender = new UDPSender(v_BlackBoard.at(i));
        MissionManager* missionManager = new MissionManager(v_BlackBoard.at(i));
        
#ifndef DEFAULT
        Alive* alive = new Alive(v_BlackBoard.at(i), n);
#endif
#ifdef DEFAULT
        Alive* alive = new Alive(v_BlackBoard.at(i));
#endif
        v_Broadcast.push_back(broadcast);
        //v_Receiver.push_back(receiver);
        v_MissionManager.push_back(missionManager);
        v_Alive.push_back(alive);
    }
    while (std::getchar() != 'c'){}
    return 0;
}



