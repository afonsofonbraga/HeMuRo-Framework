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
    
    std::vector<Blackboard *> v_Blackboard; // = new std::vector<Blackboard>;
    
    std::vector<UDPBroadcast*> v_Broadcast;// = new std::vector<UDPBroadcast>;
    //std::vector<UDPReceiver*> v_Receiver;
    std::vector<UDPSender*> v_Sender;
    
    //std::vector<TaskManager*> v_TaskManager;
    
    std::vector<MissionManager*> v_MissionManager;
    
    int i = 0;
    
    std::string nome = "Robo" + std::to_string(i);
    Blackboard* memory = new Blackboard(nome, enum_RobotCategory::ugv);
    v_Blackboard.push_back(memory);
    
    //UDPBroadcast* broadcast = new UDPBroadcast(v_Blackboard.at(i));
    //UDPReceiver* receiver = new UDPReceiver(v_Blackboard.at(i));
    UDPReceiverSim* receiver = new UDPReceiverSim();
    
    receiver->addRobot(v_Blackboard.at(i));
    
    UDPSender* sender = new UDPSender(v_Blackboard.at(i));
    
    //TaskManager* taskManager = new TaskManager(v_Blackboard.at(i));
    
    MissionManager* missionManager = new MissionManager(v_Blackboard.at(i));
    
    //v_Broadcast.push_back(broadcast);
    //v_Receiver.push_back(receiver);
    //v_TaskManager.push_back(taskManager);
    v_MissionManager.push_back(missionManager);
    
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
    
    
    {
        strcpy(mission.missionCode, "Task1");
        mission.taskToBeDecomposed = enum_DecomposableTask::flightTest;
        mission.robotCat = enum_RobotCategory::uav;
        mission.goal.x = -6.0;
        mission.goal.y = 5.0;
        mission.goal.z = 3.0;
        mission.goal.yaw = 0;
        mission.relativeDeadline = std::chrono::milliseconds(300);
        
        
        memcpy(message.buffer,"Robo0",10);
        *((Operation*)(message.buffer + 10)) = operation;
        *((int*)(message.buffer + 14)) = sizeof(mission);
        memmove(message.buffer+18,(const unsigned char*)&mission,sizeof(mission));
        message.messageSize = sizeof(message.buffer);
        
        v_Blackboard.at(0)->addUDPMessage(message);
    }
    
    
    {
        strcpy(mission.missionCode, "Task2");
        mission.operation = enum_MissionOperation::createMission;
        mission.taskToBeDecomposed = enum_DecomposableTask::checkPosition;
        mission.robotCat = enum_RobotCategory::ugv;
        mission.goal.x = 5.0;
        mission.goal.y = 4.0;
        mission.goal.z = 0.0;
        mission.goal.yaw = 0.3;
        mission.relativeDeadline = std::chrono::milliseconds(60);
        
        memcpy(message.buffer,"Robo0",10);
        *((Operation*)(message.buffer + 10)) = operation;
        *((int*)(message.buffer + 14)) = sizeof(mission);
        memmove(message.buffer+18,(const unsigned char*)&mission,sizeof(mission));
        message.messageSize = sizeof(message.buffer);
        
        v_Blackboard.at(0)->addUDPMessage(message);
    }
    
    /*{
     strcpy(mission.missionCode, "Task3");
     mission.operation = enum_MissionOperation::createMission;
     mission.taskToBeDecomposed = enum_DecomposableTask::checkPosition;
     mission.robotCat = enum_RobotCategory::ugv;
     mission.goal.x = 2.0;
     mission.goal.y = 1.0;
     mission.goal.z = 0.0;
     mission.goal.yaw = 1;
     mission.relativeDeadline = std::chrono::milliseconds(60);
     
     memcpy(message.buffer,"Robo0",10);
     *((Operation*)(message.buffer + 10)) = operation;
     *((int*)(message.buffer + 14)) = sizeof(mission);
     memmove(message.buffer+18,(const unsigned char*)&mission,sizeof(mission));
     message.messageSize = sizeof(message.buffer);
     
     v_Blackboard.at(0)->addUDPMessage(message);
     } */
    
    /*
     
     std::this_thread::sleep_for(std::chrono::seconds(5));
     strcpy(mission.missionCode, "tag2");
     mission.operation = enum_MissionOperation::emergency;
     mission.taskToBeDecomposed = enum_DecomposableTask::lowBattery;
     *((Operation*)message.buffer) = operation;
     *((int*)(message.buffer + 4)) = sizeof(mission);
     memmove(message.buffer+8,(const unsigned char*)&mission,sizeof(mission));
     message.messageSize = sizeof(message.buffer);
     
     v_Blackboard.at(0)->addUDPMessage(message);
     */
    std::this_thread::sleep_for(std::chrono::seconds(100));
    
    return 0;
}
