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
#include <unordered_map>

#include "dataTypes.hpp"
#include "Blackboard.hpp"
#include "Module.hpp"
#include "ModulePeriodic.hpp"

#include "UDPBroadcast.hpp"
#include "UDPReceiver.hpp"
#include "UDPReceiverSim.hpp"
#include "UDPSender.hpp"

#include "AtomicTask.hpp"
//#include "TaskManager.hpp"

#include "MissionManager.hpp"

int main(){
    
    std::string name{"Robo"};
    int numberOfRobots = 1;
    enum_RobotCategory cat = enum_RobotCategory::null;
    
#ifdef MAVROS
    cat = enum_RobotCategory::uav;
#endif
    
#ifdef ROSBOT
    cat = enum_RobotCategory::ugv;
#endif
    
    
    std::vector<Blackboard *> v_Blackboard; // = new std::vector<Blackboard>;
    std::vector<UDPBroadcast*> v_Broadcast;// = new std::vector<UDPBroadcast>;
    std::vector<UDPReceiver*> v_Receiver;
    std::vector<UDPSender*> v_Sender;
    std::vector<MissionManager*> v_MissionManager;
    //std::vector<Alive*> v_Alive;
    //UDPReceiverSim* receiver = new UDPReceiverSim();
    
#ifndef DEFAULT
    //std::string node = name;
    ros::init(argc, argv ,name);
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(0);
    spinner.start();
#endif
    
    for (int i=0; i< numberOfRobots; i++)
    {
        std::string robotsName = name + std::to_string(i);
        Blackboard* memory = new Blackboard(robotsName, enum_RobotCategory::null);
        v_Blackboard.push_back(memory);
        UDPBroadcast* broadcast = new UDPBroadcast(v_Blackboard.at(i));
        UDPReceiver* receiver = new UDPReceiver(v_Blackboard.at(i));
        //receiver->addRobot(v_Blackboard.at(i));
        UDPSender* sender = new UDPSender(v_Blackboard.at(i));
        MissionManager* missionManager = new MissionManager(v_Blackboard.at(i));
        
#ifndef DEFAULT
        //Alive* alive = new Alive(v_Blackboard.at(i), n);
#endif
#ifdef DEFAULT
        //Alive* alive = new Alive(v_Blackboard.at(i));
#endif
        v_Broadcast.push_back(broadcast);
        v_Receiver.push_back(receiver);
        v_MissionManager.push_back(missionManager);
        //v_Alive.push_back(alive);
    }
    
    char vIP[MAX_IP];
    
    
    std::cout << "Time to send a Mission!!!!!"<< std::endl;
    
    s_MissionMessage mission;
    v_Blackboard.at(0)->getRobotsIP(*vIP);
    v_Blackboard.at(0)->getRobotsName(*mission.senderName);
    strcpy(mission.senderAddress , vIP);
    mission.operation = enum_MissionOperation::createMission;
    
    s_UDPMessage message;
    strcpy(message.address , vIP);
    Operation operation = Operation::missionMessage;
    v_Blackboard.at(0)->getRobotsName(*message.name);
    
    /*
    {
        strcpy(mission.missionCode, "Task1");
        mission.taskToBeDecomposed = enum_DecomposableTask::flightTest;
        mission.robotCat = enum_RobotCategory::uav;
        mission.goal.x = -6.0;
        mission.goal.y = 5.0;
        mission.goal.z = 3.0;
        mission.goal.yaw = 0;
        mission.executionTime = 300;
        
        
        memcpy(message.buffer,"Robo0",10);
        *((Operation*)(message.buffer + 10)) = operation;
        *((int*)(message.buffer + 14)) = sizeof(mission);
        memmove(message.buffer+18,(const unsigned char*)&mission,sizeof(mission));
        message.messageSize = sizeof(message.buffer);
        
        v_Blackboard.at(0)->addUDPMessage(message);
    }*/
    
    
    {
        strcpy(mission.missionCode, "Task2");
        mission.operation = enum_MissionOperation::createMission;
        mission.taskToBeDecomposed = enum_DecomposableTask::checkPosition;
        mission.robotCat = enum_RobotCategory::ugv;
        mission.goal.x = 105.0;
        mission.goal.y = 104.0;
        mission.goal.z = 0.0;
        mission.goal.yaw = 0.3;
        mission.executionTime = 300;
        
        memcpy(message.buffer,"Robo0",10);
        *((Operation*)(message.buffer + 10)) = operation;
        *((int*)(message.buffer + 14)) = sizeof(mission);
        memmove(message.buffer+18,(const unsigned char*)&mission,sizeof(mission));
        message.messageSize = sizeof(message.buffer);
        
        v_Blackboard.at(0)->addUDPMessage(message);
    }
    
    {
        strcpy(mission.missionCode, "Task3");
        mission.operation = enum_MissionOperation::createMission;
        mission.taskToBeDecomposed = enum_DecomposableTask::checkPosition;
        mission.robotCat = enum_RobotCategory::ugv;
        mission.goal.x = 0.0;
        mission.goal.y = 0.0;
        mission.goal.z = 0.0;
        mission.goal.yaw = 0.3;
        mission.executionTime = 300;
        
        memcpy(message.buffer,"Robo0",10);
        *((Operation*)(message.buffer + 10)) = operation;
        *((int*)(message.buffer + 14)) = sizeof(mission);
        memmove(message.buffer+18,(const unsigned char*)&mission,sizeof(mission));
        message.messageSize = sizeof(message.buffer);
        
        v_Blackboard.at(0)->addUDPMessage(message);
    }
    
    {
        strcpy(mission.missionCode, "Task4");
        mission.operation = enum_MissionOperation::createMission;
        mission.taskToBeDecomposed = enum_DecomposableTask::checkPosition;
        mission.robotCat = enum_RobotCategory::ugv;
        mission.goal.x = 0.0;
        mission.goal.y = 0.0;
        mission.goal.z = 0.0;
        mission.goal.yaw = 0.3;
        mission.executionTime = 300;
        
        memcpy(message.buffer,"Robo0",10);
        *((Operation*)(message.buffer + 10)) = operation;
        *((int*)(message.buffer + 14)) = sizeof(mission);
        memmove(message.buffer+18,(const unsigned char*)&mission,sizeof(mission));
        message.messageSize = sizeof(message.buffer);
        
        v_Blackboard.at(0)->addUDPMessage(message);
    }
    
    while (std::getchar() != 'c'){}
    return 0;
    
    
}
