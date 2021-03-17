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
#include "ChargingStation.hpp"

#include "DefaultRobot.hpp"
#include "RosbotRobot.hpp"
#include "P3DXRobot.hpp"



int main( int argc, char *argv[ ] )
{

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
    
    if (argc < 2)
    {
        std::cerr << "Please, inform the robot's name:";
        return 0;
    }
    
    std::vector<Blackboard *> v_Blackboard; // = new std::vector<Blackboard>;
    UDPReceiverSim* receiver = new UDPReceiverSim();
    
#ifdef DEFAULT
    std::vector<Agent* > v_Robot; 
#endif
    
#ifndef DEFAULT
    std::string node = "node"; //name;
    ros::init(argc, argv ,node);
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    std::vector<Agent* > v_Robot; 
#endif
    
    std::string name{"Robo"};
    int defaultAgents = 0;
    int numberOfRobots = 0;

    std::string robotsName = "Logger";
    Blackboard* memory = new Blackboard(robotsName, enum_RobotCategory::null);
    v_Blackboard.push_back(memory);
    receiver->addRobot(v_Blackboard.at(defaultAgents));
    bool decentralizedCommunication = false;
    LoggerAgent* logger = new LoggerAgent(v_Blackboard.at(defaultAgents), decentralizedCommunication, argc, argv);    
    v_Robot.push_back(logger);
    defaultAgents++;
    
    robotsName = "CStation1";
    memory = new Blackboard(robotsName, enum_RobotCategory::null);
    v_Blackboard.push_back(memory);
    v_Blackboard.at(defaultAgents)->setPosition(chargingStation_01);
    ChargingStation* station = new ChargingStation(v_Blackboard.at(defaultAgents), decentralizedCommunication);
    receiver->addRobot(v_Blackboard.at(defaultAgents));
    v_Robot.push_back(station);
    defaultAgents++;
    
    robotsName = "CStation2";
    memory = new Blackboard(robotsName, enum_RobotCategory::null);
    v_Blackboard.push_back(memory);
    v_Blackboard.at(defaultAgents)->setPosition(chargingStation_02);
    station = new ChargingStation(v_Blackboard.at(defaultAgents), decentralizedCommunication);
    receiver->addRobot(v_Blackboard.at(defaultAgents));
    v_Robot.push_back(station);
    defaultAgents++;

    for (int i=defaultAgents; i-defaultAgents< numberOfRobots; i++)
    {
        std::string robotsName = name + std::to_string(i-defaultAgents);
        Blackboard* memory = new Blackboard(robotsName, enum_RobotCategory::null);
        v_Blackboard.push_back(memory);
        bool decentralizedCommunication = false;
        DefaultRobot* robot = new DefaultRobot(v_Blackboard.at(i), decentralizedCommunication);
        receiver->addRobot(v_Blackboard.at(i));
        v_Robot.push_back(robot);
        usleep(1000);
    }
    defaultAgents += numberOfRobots;

    std::vector<std::string> robots;
    robots.push_back("thor");
    robots.push_back("zeus");
    //robots.push_back("joao");
    for (int i = 0; i < robots.size(); i++)
    {
        robotsName = robots.at(i);
        Blackboard* memory = new Blackboard(robotsName, enum_RobotCategory::null);
        v_Blackboard.push_back(memory);
        receiver->addRobot(v_Blackboard.at(i + defaultAgents));
        decentralizedCommunication = false;
#ifndef DEFAULT
        //if(i%2 == 0)
        {
            RosbotRobot* robot = new RosbotRobot(v_Blackboard.at(i + defaultAgents), n, decentralizedCommunication);
            v_Robot.push_back(robot);
        }
        /*if (i%2 == 1)
        {
            P3DXRobot* robot = new P3DXRobot(v_Blackboard.at(i + defaultAgents), n, decentralizedCommunication);
            v_Robot.push_back(robot);
        }*/
#endif            
#ifdef DEFAULT
        DefaultRobot* robot = new DefaultRobot(v_Blackboard.at(i), decentralizedCommunication);
#endif
    }


    char vIP[MAX_IP];
    s_MissionMessage mission;
    v_Blackboard.at(1)->getRobotsIP(*vIP);
    v_Blackboard.at(1)->getRobotsName(*mission.senderName);
    strcpy(mission.senderAddress , vIP);
    mission.operation = enum_MissionOperation::createMission;
    
    s_UDPMessage message;
    strcpy(message.address , vIP);
    Operation operation = Operation::missionMessage;
    v_Blackboard.at(1)->getRobotsName(*message.name);
    
    
    
    {
        strcpy(mission.missionCode, "sure1");
        mission.operation = enum_MissionOperation::createMission;
        mission.taskToBeDecomposed = enum_DecomposableTask::measureTemperature;
        mission.robotCat = enum_RobotCategory::ugv;
        mission.numberOfAttributes = 1;
        *((int*) (mission.attributesBuffer + 4)) = sizeof(s_pose);
        memcpy(mission.attributesBuffer + 8, &salao_02, sizeof(s_pose));
        *((int*) (mission.attributesBuffer)) = sizeof(s_pose) + 8;
          
        mission.relativeDeadline = std::chrono::seconds(160);
        
        memcpy(message.buffer,"CStation1",10);
        *((Operation*)(message.buffer + 10)) = operation;
        *((int*)(message.buffer + 14)) = sizeof(mission);
        memmove(message.buffer+18,(const unsigned char*)&mission,sizeof(mission));
        message.messageSize = sizeof(message.buffer);
        
        v_Blackboard.at(1)->addUDPMessage(message);
    }

        {
        strcpy(mission.missionCode, "sure2");
        mission.operation = enum_MissionOperation::createMission;
        mission.taskToBeDecomposed = enum_DecomposableTask::measureTemperature;
        mission.robotCat = enum_RobotCategory::ugv;
        mission.numberOfAttributes = 1;
        *((int*) (mission.attributesBuffer + 4)) = sizeof(s_pose);
        memcpy(mission.attributesBuffer + 8, &sala_A04, sizeof(s_pose));
        *((int*) (mission.attributesBuffer)) = sizeof(s_pose) + 8;
          
        mission.relativeDeadline = std::chrono::seconds(160);
        
        memcpy(message.buffer,"CStation1",10);
        *((Operation*)(message.buffer + 10)) = operation;
        *((int*)(message.buffer + 14)) = sizeof(mission);
        memmove(message.buffer+18,(const unsigned char*)&mission,sizeof(mission));
        message.messageSize = sizeof(message.buffer);
        
        v_Blackboard.at(1)->addUDPMessage(message);
    }

    {
        strcpy(mission.missionCode, "Inspect1");
        mission.operation = enum_MissionOperation::createMission;
        mission.taskToBeDecomposed = enum_DecomposableTask::inspectPlace;
        mission.robotCat = enum_RobotCategory::ugv;
        mission.numberOfAttributes = 1;
        *((int*) (mission.attributesBuffer + 4)) = sizeof(s_pose);
        memcpy(mission.attributesBuffer + 8, &salao_01, sizeof(s_pose));
        *((int*) (mission.attributesBuffer)) = sizeof(s_pose) + 8;
          
        mission.relativeDeadline = std::chrono::seconds(160);
        
        memcpy(message.buffer,"CStation1",10);
        *((Operation*)(message.buffer + 10)) = operation;
        *((int*)(message.buffer + 14)) = sizeof(mission);
        memmove(message.buffer+18,(const unsigned char*)&mission,sizeof(mission));
        message.messageSize = sizeof(message.buffer);
        
        v_Blackboard.at(1)->addUDPMessage(message);
    }

    {
        strcpy(mission.missionCode, "Inspect2");
        mission.operation = enum_MissionOperation::createMission;
        mission.taskToBeDecomposed = enum_DecomposableTask::inspectPlace;
        mission.robotCat = enum_RobotCategory::ugv;
        mission.numberOfAttributes = 1;
        *((int*) (mission.attributesBuffer + 4)) = sizeof(s_pose);
        memcpy(mission.attributesBuffer + 8, &salao_03, sizeof(s_pose));
        *((int*) (mission.attributesBuffer)) = sizeof(s_pose) + 8;
          
        mission.relativeDeadline = std::chrono::seconds(160);
        
        memcpy(message.buffer,"CStation1",10);
        *((Operation*)(message.buffer + 10)) = operation;
        *((int*)(message.buffer + 14)) = sizeof(mission);
        memmove(message.buffer+18,(const unsigned char*)&mission,sizeof(mission));
        message.messageSize = sizeof(message.buffer);
        
        v_Blackboard.at(1)->addUDPMessage(message);
    }
    

 {
        strcpy(mission.missionCode, "Deliver1");
        mission.operation = enum_MissionOperation::createMission;
        mission.taskToBeDecomposed = enum_DecomposableTask::deliverSmallSample;
        mission.robotCat = enum_RobotCategory::ugv;
        mission.numberOfAttributes = 2;
        int total = 4;
        *((int*) (mission.attributesBuffer + total)) = sizeof(s_pose);
        total += 4;
        memcpy(mission.attributesBuffer + total, &sala_A01, sizeof(s_pose));
        total += sizeof(s_pose);
        *((int*) (mission.attributesBuffer + total)) = sizeof(s_pose);
        total += 4;
        memcpy(mission.attributesBuffer + total, &recepcao, sizeof(s_pose));
        total += sizeof(s_pose);
        *((int*) (mission.attributesBuffer)) = total;
        
        mission.relativeDeadline = std::chrono::seconds(180);
        
        memcpy(message.buffer,"CStation1",10);
        *((Operation*)(message.buffer + 10)) = operation;
        *((int*)(message.buffer + 14)) = sizeof(mission);
        memmove(message.buffer+18,(const unsigned char*)&mission,sizeof(mission));
        message.messageSize = sizeof(message.buffer);
        
        v_Blackboard.at(1)->addUDPMessage(message);
    }

    {
        strcpy(mission.missionCode, "Deliver2");
        mission.operation = enum_MissionOperation::createMission;
        mission.taskToBeDecomposed = enum_DecomposableTask::deliverSmallSample;
        mission.robotCat = enum_RobotCategory::ugv;
        mission.numberOfAttributes = 2;
        int total = 4;
        *((int*) (mission.attributesBuffer + total)) = sizeof(s_pose);
        total += 4;
        memcpy(mission.attributesBuffer + total, &sala_A03, sizeof(s_pose));
        total += sizeof(s_pose);
        *((int*) (mission.attributesBuffer + total)) = sizeof(s_pose);
        total += 4;
        memcpy(mission.attributesBuffer + total, &recepcao, sizeof(s_pose));
        total += sizeof(s_pose);
        *((int*) (mission.attributesBuffer)) = total;
        
        mission.relativeDeadline = std::chrono::seconds(180);
        
        memcpy(message.buffer,"CStation1",10);
        *((Operation*)(message.buffer + 10)) = operation;
        *((int*)(message.buffer + 14)) = sizeof(mission);
        memmove(message.buffer+18,(const unsigned char*)&mission,sizeof(mission));
        message.messageSize = sizeof(message.buffer);
        
        v_Blackboard.at(1)->addUDPMessage(message);
    }

    while (std::getchar() != 'c'){}
    
    //delete logger;
    return 0;
}



