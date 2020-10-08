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
#include "LoggerAgent.hpp"

int main(){
    
    std::string name{"Robo"};
    int numberOfRobots =0;
    
    std::vector<BlackBoard* > v_BlackBoard; // = new std::vector<BlackBoard>;
    std::vector<DefaultRobot* > v_DefaultRobot;
    std::vector<ChargingStation* > v_ChargingStation;
    UDPReceiverSim* receiver = new UDPReceiverSim();
    LoggerAgent* logger;
    
    std::string robotsName = "Logger";
    BlackBoard* memory = new BlackBoard(robotsName, enum_RobotCategory::null);
    v_BlackBoard.push_back(memory);
    bool decentralizedCommunication = false;
    logger = new LoggerAgent(v_BlackBoard.at(0), decentralizedCommunication);
    receiver->addRobot(v_BlackBoard.at(0));
    
    
    robotsName = "CStation";
    memory = new BlackBoard(robotsName, enum_RobotCategory::null);
    v_BlackBoard.push_back(memory);
    s_pose position;
    position.x = 7;
    position.y = 16.5;
    position.z = 0;
    v_BlackBoard.at(1)->setPosition(position);
    ChargingStation* station = new ChargingStation(v_BlackBoard.at(1), decentralizedCommunication);
    receiver->addRobot(v_BlackBoard.at(1));
    v_ChargingStation.push_back(station);
    
    
    
    
    for (int i=2; i-2< numberOfRobots; i++)
    {
        std::string robotsName = name + std::to_string(i-2);
        BlackBoard* memory = new BlackBoard(robotsName, enum_RobotCategory::null);
        v_BlackBoard.push_back(memory);
        bool decentralizedCommunication = false;
        DefaultRobot* robot = new DefaultRobot(v_BlackBoard.at(i), decentralizedCommunication);
        receiver->addRobot(v_BlackBoard.at(i));
        v_DefaultRobot.push_back(robot);
        
    }
    /*
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
     mission.goal.x = 1.5;
     mission.goal.y = 0.0;
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
         mission.goal.x = -1.5;
     mission.goal.y = 1.0;
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
     mission.goal.x = 0.0;
     mission.goal.y = -1.5;
     mission.goal.z = 0.0;
     mission.goal.yaw = 0.3;
     mission.executionTime = 300;
     
     memcpy(message.buffer,"CStation",10);
     *((Operation*)(message.buffer + 10)) = operation;
     *((int*)(message.buffer + 14)) = sizeof(mission);
     memmove(message.buffer+18,(const unsigned char*)&mission,sizeof(mission));
     message.messageSize = sizeof(message.buffer);
     
     v_BlackBoard.at(0)->addUDPMessage(message);
     }*/
    
    while (std::getchar() != 'c'){}
    return 0;
    
    
}

