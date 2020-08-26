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
#include "BlackBoard.hpp"
#include "Module.hpp"
#include "ModulePeriodic.hpp"

#include "UDPBroadcast.hpp"
#include "UDPReceiver.hpp"
#include "UDPSender.hpp"

#include "AtomicTask.hpp"
//#include "TaskManager.hpp"

#include "MissionManager.hpp"

int main(){
    
    std::vector<BlackBoard *> v_BlackBoard; // = new std::vector<BlackBoard>;
    
    std::vector<UDPBroadcast*> v_Broadcast;// = new std::vector<UDPBroadcast>;
    std::vector<UDPReceiver*> v_Receiver;
    std::vector<UDPSender*> v_Sender;
    
    //std::vector<TaskManager*> v_TaskManager;
    
    std::vector<MissionManager*> v_MissionManager;
    
    int i = 0;
    
    std::string nome = "Robo" + std::to_string(i);
    BlackBoard* memory = new BlackBoard(nome, enum_RobotCategory::ugv);
    v_BlackBoard.push_back(memory);
    
    UDPBroadcast* broadcast = new UDPBroadcast(v_BlackBoard.at(i));
    UDPReceiver* receiver = new UDPReceiver(v_BlackBoard.at(i));
    UDPSender* sender = new UDPSender(v_BlackBoard.at(i));
    
    //TaskManager* taskManager = new TaskManager(v_BlackBoard.at(i));
    
    MissionManager* missionManager = new MissionManager(v_BlackBoard.at(i));
    
    v_Broadcast.push_back(broadcast);
    v_Receiver.push_back(receiver);
    //v_TaskManager.push_back(taskManager);
    v_MissionManager.push_back(missionManager);
    
    char vIP[16];
    
    s_MissionMessage mission;
    v_BlackBoard.at(0)->getRobotsIP(*vIP);
    strcpy(mission.missionCode, "tag1");
    strcpy(mission.senderAddress , vIP);
    mission.operation = enum_MissionOperation::createMission;
    mission.taskToBeDecomposed = enum_DecomposableTask::flightTest;
    mission.goal.x = 33.0;
    mission.goal.y = 22.0;
    mission.goal.z = 4.0;
    mission.executionTime = 30;
    mission.robotCat = enum_RobotCategory::uav;
    
    std::cout << "Time to send a Mission!!!!!"<< std::endl;
    
    s_UDPMessage message;
    strcpy(message.address , vIP);
    
    
    Operation operation = Operation::missionMessage;
    *((Operation*)message.buffer) = operation;
    *((int*)(message.buffer + 4)) = sizeof(mission);
    memmove(message.buffer+8,(const unsigned char*)&mission,sizeof(mission));
    message.messageSize = sizeof(message.buffer);

    v_BlackBoard.at(0)->addUDPMessage(message);
   /*
    strcpy(mission.missionCode, "Pic1");
    mission.operation = enum_MissionOperation::createMission;
    mission.taskToBeDecomposed = enum_DecomposableTask::takePicture;
    mission.goal.x = 9.0;
    mission.goal.y = 9.0;
    mission.goal.theta = 1.0;
    
    
    *((Operation*)message.buffer) = operation;
    *((int*)(message.buffer + 4)) = sizeof(mission);
    memmove(message.buffer+8,(const unsigned char*)&mission,sizeof(mission));
    message.messageSize = sizeof(message.buffer);
    
    v_BlackBoard.at(0)->addUDPMessage(message);
    
    
    std::this_thread::sleep_for(std::chrono::seconds(5));
    strcpy(mission.missionCode, "tag2");
    mission.operation = enum_MissionOperation::emergency;
    mission.taskToBeDecomposed = enum_DecomposableTask::lowBattery;
    *((Operation*)message.buffer) = operation;
    *((int*)(message.buffer + 4)) = sizeof(mission);
    memmove(message.buffer+8,(const unsigned char*)&mission,sizeof(mission));
    message.messageSize = sizeof(message.buffer);
    
    v_BlackBoard.at(0)->addUDPMessage(message);
*/
    std::this_thread::sleep_for(std::chrono::seconds(10));
    
    return 0;
}
