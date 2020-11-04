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
    s_pose sala_A01;
    {
        sala_A01.x = 10.5;
        sala_A01.y = 5.5;
        sala_A01.z = 0.0;
    }
    s_pose sala_A02;
    {
        sala_A02.x = 10.5;
        sala_A02.y = 10.5;
        sala_A02.z = 0.0;
    }
    s_pose sala_A03;
    {
        sala_A03.x = -10.5;
        sala_A03.y = 5.5;
        sala_A03.z = 0.0;
    }
    s_pose sala_A04;
    {
        sala_A04.x = -10.5;
        sala_A04.y = 10.5;
        sala_A04.z = 0.0;
    }
    s_pose deposito_01;
    {
        deposito_01.x = 8.0;
        deposito_01.y = 14.0;
        deposito_01.z = 0.0;
    }
    s_pose deposito_02;
    {
        deposito_02.x = -8.0;
        deposito_02.y = 14.0;
        deposito_02.z = 0.0;
    }
    s_pose chargingStation_01;
    {
        chargingStation_01.x = 9.0;
        chargingStation_01.y = 14.0;
        chargingStation_01.z = 0.0;
    }
    s_pose chargingStation_02;
    {
        chargingStation_02.x = -9.0;
        chargingStation_02.y = 14.0;
        chargingStation_02.z = 0.0;
    }
    s_pose escada_01;
    {
        escada_01.x = 5.0;
        escada_01.y = 18.0;
        escada_01.z = 0.0;
    }
    s_pose escada_02;
    {
        escada_02.x = -5.0;
        escada_02.y = 18.0;
        escada_02.z = 0.0;
    }
    s_pose salao_01;
    {
        salao_01.x = 3.0;
        salao_01.y = 12.0;
        salao_01.z = 0.0;
    }
    s_pose salao_02;
    {
        salao_02.x = 0.0;
        salao_02.y = 14.0;
        salao_02.z = 0.0;
    }
    s_pose salao_03;
    {
        salao_03.x = -3.0;
        salao_03.y = 12.0;
        salao_03.z = 0.0;
    }
    s_pose recepcao;
    {
        recepcao.x = 0.0;
        recepcao.y = 2.0;
        recepcao.z = 0.0;
    }
    
    std::string name{"Robo"};
    int numberOfRobots =1;
    
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
    
    
    robotsName = "CStation1";
    memory = new BlackBoard(robotsName, enum_RobotCategory::null);
    v_BlackBoard.push_back(memory);
    v_BlackBoard.at(1)->setPosition(chargingStation_01);
    ChargingStation* station = new ChargingStation(v_BlackBoard.at(1), decentralizedCommunication);
    receiver->addRobot(v_BlackBoard.at(1));
    v_ChargingStation.push_back(station);
    
    
    robotsName = "CStation2";
    memory = new BlackBoard(robotsName, enum_RobotCategory::null);
    v_BlackBoard.push_back(memory);
    v_BlackBoard.at(2)->setPosition(chargingStation_02);
    station = new ChargingStation(v_BlackBoard.at(2), decentralizedCommunication);
    receiver->addRobot(v_BlackBoard.at(2));
    v_ChargingStation.push_back(station);
    

     for (int i=3; i-3< numberOfRobots; i++)
     {
     std::string robotsName = name + std::to_string(i-3);
     BlackBoard* memory = new BlackBoard(robotsName, enum_RobotCategory::null);
     v_BlackBoard.push_back(memory);
     bool decentralizedCommunication = false;
     DefaultRobot* robot = new DefaultRobot(v_BlackBoard.at(i), decentralizedCommunication);
     receiver->addRobot(v_BlackBoard.at(i));
     v_DefaultRobot.push_back(robot);
    
     }
    
    char vIP[MAX_IP];
    
    
    
    std::cout << "Time to send a Mission!!!!!"<< std::endl;
    
    s_MissionMessage mission;
    v_BlackBoard.at(1)->getRobotsIP(*vIP);
    v_BlackBoard.at(1)->getRobotsName(*mission.senderName);
    strcpy(mission.senderAddress , vIP);
    mission.operation = enum_MissionOperation::createMission;
    
    s_UDPMessage message;
    strcpy(message.address , vIP);
    Operation operation = Operation::missionMessage;
    v_BlackBoard.at(1)->getRobotsName(*message.name);
    
    {
        strcpy(mission.missionCode, "Task2");
        mission.operation = enum_MissionOperation::createMission;
        mission.taskToBeDecomposed = enum_DecomposableTask::checkPosition;
        mission.robotCat = enum_RobotCategory::ugv;
        //mission.goal.x = 10.5;
        //mission.goal.y = 5.5;
        //mission.goal.z = 0.0;
        //mission.goal.yaw = 0.3;

        //mission.goal.pop();
        mission.goal = sala_A01;
        mission.executionTime = 30;
        
        memcpy(message.buffer,"CStation1",10);
        *((Operation*)(message.buffer + 10)) = operation;
        *((int*)(message.buffer + 14)) = sizeof(mission);
        memmove(message.buffer+18,(const unsigned char*)&mission,sizeof(mission));
        message.messageSize = sizeof(message.buffer);
        
        v_BlackBoard.at(1)->addUDPMessage(message);
    }
    
    {
        strcpy(mission.missionCode, "Deliver");
        mission.operation = enum_MissionOperation::createMission;
        mission.taskToBeDecomposed = enum_DecomposableTask::deliverPicture;
        mission.robotCat = enum_RobotCategory::ugv;
        //mission.goal.x = 10.5;
        //mission.goal.y = 5.5;
        //mission.goal.z = 0.0;
        //mission.goal.yaw = 0.3;
        mission.goal = recepcao;
        mission.executionTime = 30;
        
        memcpy(message.buffer,"CStation1",10);
        *((Operation*)(message.buffer + 10)) = operation;
        *((int*)(message.buffer + 14)) = sizeof(mission);
        memmove(message.buffer+18,(const unsigned char*)&mission,sizeof(mission));
        message.messageSize = sizeof(message.buffer);
        
        v_BlackBoard.at(1)->addUDPMessage(message);
    }
    
    {
        strcpy(mission.missionCode, "Task3");
        mission.operation = enum_MissionOperation::createMission;
        mission.taskToBeDecomposed = enum_DecomposableTask::checkPosition;
        mission.robotCat = enum_RobotCategory::ugv;
        //mission.goal.x = 0;
        //mission.goal.y = 14.0;
        //mission.goal.z = 0.0;
        //mission.goal.yaw = 0.3;
        mission.goal = deposito_01;
        mission.executionTime = 30;
        
        memcpy(message.buffer,"CStation1",10);
        *((Operation*)(message.buffer + 10)) = operation;
        *((int*)(message.buffer + 14)) = sizeof(mission);
        memmove(message.buffer+18,(const unsigned char*)&mission,sizeof(mission));
        message.messageSize = sizeof(message.buffer);
        
        v_BlackBoard.at(1)->addUDPMessage(message);
    }
    
    {
        strcpy(mission.missionCode, "Task4");
        mission.operation = enum_MissionOperation::createMission;
        mission.taskToBeDecomposed = enum_DecomposableTask::checkPosition;
        mission.robotCat = enum_RobotCategory::ugv;
        //mission.goal.x = -8.0;
        //mission.goal.y = 14;
        //mission.goal.z = 0.0;
        //mission.goal.yaw = 0.3;

        mission.goal = escada_02;
        mission.executionTime = 30;
        
        memcpy(message.buffer,"CStation1",10);
        *((Operation*)(message.buffer + 10)) = operation;
        *((int*)(message.buffer + 14)) = sizeof(mission);
        memmove(message.buffer+18,(const unsigned char*)&mission,sizeof(mission));
        message.messageSize = sizeof(message.buffer);
        
        v_BlackBoard.at(1)->addUDPMessage(message);
    }
    
    while (std::getchar() != 'c'){}
    return 0;
    
    
}

