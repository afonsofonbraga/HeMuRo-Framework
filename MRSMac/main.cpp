/*
 * Copyright (C) 2008 Emweb bv, Herent, Belgium.
 *
 * See the LICENSE file for terms of use.
 */
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




int main(int argc, char **argv){
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
    int numberOfRobots = 1; // Number of robots that will be executing the tasks
    int defaultAgents = 0; // Number of Agents including Logger and Charging Stations
    
    std::vector<BlackBoard* > v_BlackBoard; // = new std::vector<BlackBoard>;
    std::vector<DefaultRobot* > v_DefaultRobot;
    std::vector<ChargingStation* > v_ChargingStation;
    UDPReceiverSim* receiver = new UDPReceiverSim();
    LoggerAgent* logger;
    
    std::string robotsName = "Logger";
    BlackBoard* memory = new BlackBoard(robotsName, enum_RobotCategory::null);
    v_BlackBoard.push_back(memory);
    bool decentralizedCommunication = false;
    logger = new LoggerAgent(v_BlackBoard.at(defaultAgents), decentralizedCommunication, argc, argv);
    receiver->addRobot(v_BlackBoard.at(defaultAgents));
    defaultAgents++;
    
    robotsName = "CStation1";
    memory = new BlackBoard(robotsName, enum_RobotCategory::null);
    v_BlackBoard.push_back(memory);
    v_BlackBoard.at(defaultAgents)->setPosition(chargingStation_01);
    ChargingStation* station = new ChargingStation(v_BlackBoard.at(defaultAgents), decentralizedCommunication);
    receiver->addRobot(v_BlackBoard.at(defaultAgents));
    v_ChargingStation.push_back(station);
    defaultAgents++;
    
    
    robotsName = "CStation2";
    memory = new BlackBoard(robotsName, enum_RobotCategory::null);
    v_BlackBoard.push_back(memory);
    v_BlackBoard.at(defaultAgents)->setPosition(chargingStation_02);
    v_ChargingStation.push_back(station);
    station = new ChargingStation(v_BlackBoard.at(defaultAgents), decentralizedCommunication);
    receiver->addRobot(v_BlackBoard.at(defaultAgents));
    v_ChargingStation.push_back(station);
    defaultAgents++;
    
    
     for (int i=defaultAgents; i-defaultAgents< numberOfRobots; i++)
     {
     std::string robotsName = name + std::to_string(i-defaultAgents);
     BlackBoard* memory = new BlackBoard(robotsName, enum_RobotCategory::null);
     v_BlackBoard.push_back(memory);
     bool decentralizedCommunication = false;
     DefaultRobot* robot = new DefaultRobot(v_BlackBoard.at(i), decentralizedCommunication);
     receiver->addRobot(v_BlackBoard.at(i));
     v_DefaultRobot.push_back(robot);
     usleep(1000);
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
        
        mission.numberOfAttributes = 1;
        *((int*) (mission.attributesBuffer + 4)) = sizeof(sala_A01);
        memcpy(mission.attributesBuffer + 8, &sala_A01, sizeof(sala_A01));
        *((int*) (mission.attributesBuffer)) = sizeof(sala_A01) + 8;
          
        mission.executionTime = 60;
        
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
        mission.numberOfAttributes = 2;
        int total = 4;
        *((int*) (mission.attributesBuffer + total)) = sizeof(s_pose);
        total += 4;
        memcpy(mission.attributesBuffer + total, &escada_01, sizeof(s_pose));
        total += sizeof(s_pose);
        *((int*) (mission.attributesBuffer + total)) = sizeof(s_pose);
        total += 4;
        memcpy(mission.attributesBuffer + total, &recepcao, sizeof(s_pose));
        total += sizeof(s_pose);
        *((int*) (mission.attributesBuffer)) = total;
        
        mission.executionTime = 120;
        
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
        mission.numberOfAttributes = 1;
        *((int*) (mission.attributesBuffer + 4)) = sizeof(s_pose);
        memcpy(mission.attributesBuffer + 8, &deposito_01, sizeof(s_pose));
        *((int*) (mission.attributesBuffer)) = sizeof(s_pose) + 8;
        
        mission.executionTime = 60;
        
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
        mission.numberOfAttributes = 1;
        *((int*) (mission.attributesBuffer + 4)) = sizeof(s_pose);
        memcpy(mission.attributesBuffer + 8, &escada_02, sizeof(s_pose));
        *((int*) (mission.attributesBuffer)) = sizeof(s_pose) + 8;
        
        mission.executionTime = 60;
        
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
