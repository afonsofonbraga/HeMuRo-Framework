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

#include "BlackBoard.hpp"
#include "dataTypes.hpp"
#include "UDPReceiverSim.hpp"
#include "DefaultRobot.hpp"
#include "ChargingStation.hpp"


int main(){
    
    std::string name{"Robo"};
    int numberOfRobots =2;
    
    std::vector<BlackBoard* > v_BlackBoard; // = new std::vector<BlackBoard>;
    std::vector<DefaultRobot* > v_DefaultRobot;
    std::vector<ChargingStation* > v_ChargingStation;
    UDPReceiverSim* receiver = new UDPReceiverSim();
    
    std::string robotsName = "Logger";
    BlackBoard* memory = new BlackBoard(robotsName, enum_RobotCategory::null);
    v_BlackBoard.push_back(memory);
    receiver->addRobot(v_BlackBoard.at(0));
    bool decentralizedCommunication = false;
    DefaultRobot* robot = new DefaultRobot(v_BlackBoard.at(0), decentralizedCommunication);
    v_DefaultRobot.push_back(robot);
    
    
    robotsName = "CStation";
    memory = new BlackBoard(robotsName, enum_RobotCategory::null);
    v_BlackBoard.push_back(memory);
    receiver->addRobot(v_BlackBoard.at(1));
    ChargingStation* station = new ChargingStation(v_BlackBoard.at(1), decentralizedCommunication);
    s_pose position;
    position.x = 0;
    position.y = 10;
    position.z = 0;
    v_BlackBoard.at(1)->setPosition(position);
    v_ChargingStation.push_back(station);
    
    
    /*
    
    for (int i=2; i-2< numberOfRobots; i++)
    {
        std::string robotsName = name + std::to_string(i-2);
        BlackBoard* memory = new BlackBoard(robotsName, enum_RobotCategory::null);
        v_BlackBoard.push_back(memory);
        receiver->addRobot(v_BlackBoard.at(i));
        bool decentralizedCommunication = false;
        DefaultRobot* robot = new DefaultRobot(v_BlackBoard.at(i), decentralizedCommunication);
        v_DefaultRobot.push_back(robot);
        
    }
    */
    char vIP[MAX_IP];
    
    
    
     std::cout << "Time to send a Mission!!!!!"<< std::endl;
     
     s_MissionMessage mission;
     v_BlackBoard.at(0)->getRobotsIP(*vIP);
     v_BlackBoard.at(0)->getRobotsName(*mission.senderName);
     strcpy(mission.senderAddress , vIP);
     mission.operation = enum_MissionOperation::createMission;
     
     s_UDPMessage message;
     strcpy(message.address , vIP);
     Operation operation = Operation::missionMessage;
     v_BlackBoard.at(0)->getRobotsName(*message.name);
     
     {
     strcpy(mission.missionCode, "Task2");
     mission.operation = enum_MissionOperation::createMission;
     mission.taskToBeDecomposed = enum_DecomposableTask::checkPosition;
     mission.robotCat = enum_RobotCategory::ugv;
     mission.goal.x = 10.0;
     mission.goal.y = 10.0;
     mission.goal.z = 0.0;
     mission.goal.yaw = 0.3;
     mission.executionTime = 300;
     
     memcpy(message.buffer,"CStation",10);
     *((Operation*)(message.buffer + 10)) = operation;
     *((int*)(message.buffer + 14)) = sizeof(mission);
     memmove(message.buffer+18,(const unsigned char*)&mission,sizeof(mission));
     message.messageSize = sizeof(message.buffer);
     
     v_BlackBoard.at(0)->addUDPMessage(message);
     }
     
     {
     strcpy(mission.missionCode, "Task3");
     mission.operation = enum_MissionOperation::createMission;
     mission.taskToBeDecomposed = enum_DecomposableTask::checkPosition;
     mission.robotCat = enum_RobotCategory::ugv;
     mission.goal.x = 15.0;
     mission.goal.y = 2.0;
     mission.goal.z = 0.0;
     mission.goal.yaw = 0.3;
     mission.executionTime = 300;
     
     memcpy(message.buffer,"CStation",10);
     *((Operation*)(message.buffer + 10)) = operation;
     *((int*)(message.buffer + 14)) = sizeof(mission);
     memmove(message.buffer+18,(const unsigned char*)&mission,sizeof(mission));
     message.messageSize = sizeof(message.buffer);
     
     v_BlackBoard.at(0)->addUDPMessage(message);
     }
     
     {
     strcpy(mission.missionCode, "Task4");
     mission.operation = enum_MissionOperation::createMission;
     mission.taskToBeDecomposed = enum_DecomposableTask::checkPosition;
     mission.robotCat = enum_RobotCategory::ugv;
     mission.goal.x = -20.0;
     mission.goal.y = 10.0;
     mission.goal.z = 0.0;
     mission.goal.yaw = 0.3;
     mission.executionTime = 300;
     
     memcpy(message.buffer,"CStation",10);
     *((Operation*)(message.buffer + 10)) = operation;
     *((int*)(message.buffer + 14)) = sizeof(mission);
     memmove(message.buffer+18,(const unsigned char*)&mission,sizeof(mission));
     message.messageSize = sizeof(message.buffer);
     
     v_BlackBoard.at(0)->addUDPMessage(message);
     }
    
    while (std::getchar() != 'c'){}
    return 0;
    
    
}

