//
//  warehouse.cpp
//  MRSFramework
//
//  Created by Afonso Braga on 03/05/21.
//  Copyright © 2021 Afonso Braga. All rights reserved.
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
#include "MavrosRobot.hpp"



int main( int argc, char *argv[ ] )
{
    s_pose chargingStation_01;
    {
        chargingStation_01.x = 4.5;
        chargingStation_01.y = -9.0;
        chargingStation_01.z = 4;
    }
    s_pose corredor_01;
    {
        corredor_01.x = 4.5;
        corredor_01.y = -5.8;
        corredor_01.z = 4;
    }
    s_pose corredor_02;
    {
        corredor_02.x = 4.5;
        corredor_02.y = -3.8;
        corredor_02.z = 4;
    }
    s_pose corredor_03;
    {
        corredor_03.x = 4.5;
        corredor_03.y = -1.8;
        corredor_03.z = 4;
    }
    s_pose area_01;
    {
        area_01.x = -1;
        area_01.y = -7;
        area_01.z = 4;
    }
    s_pose area_02;
    {
        area_02.x = -1;
        area_02.y = -4;
        area_02.z = 4;
    }
    s_pose area_03;
    {
        area_03.x = -1;
        area_03.y = -1;
        area_03.z = 4;
    }
    s_pose area_04;
    {
        area_04.x = -1;
        area_04.y = 2;
        area_04.z = 4;
    }
    s_pose area_05;
    {
        area_05.x = -1;
        area_05.y = 5;
        area_05.z = 4;
    }
    
    if (argc < 2)
    {
        std::cerr << "Please, inform the robot's name:";
        return 0;
    }
    
    std::vector<Blackboard *> v_Blackboard; // = new std::vector<Blackboard>;
    UDPReceiverSim* receiver = new UDPReceiverSim();
    std::vector<Agent* > v_Robot;
    
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
    
#ifndef DEFAULT
    std::string node = "node"; //name;
    ros::init(argc, argv ,node);
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(0);
    spinner.start();
#endif
    
    std::vector<std::string> robots;
    robots.push_back("mavros");
    robots.push_back("afrodite");
    
    for (int i = 0; i < robots.size(); i++)
    {
        robotsName = robots.at(i);
        Blackboard* memory = new Blackboard(robotsName, enum_RobotCategory::null);
        v_Blackboard.push_back(memory);
        receiver->addRobot(v_Blackboard.at(i + defaultAgents));
        decentralizedCommunication = false;
#ifndef DEFAULT

        if(i%2 == 0)
        {
            MavrosRobot* robot = new MavrosRobot(v_Blackboard.at(i + defaultAgents), n, decentralizedCommunication);
            s_pose basis;
            basis.x = chargingStation_01.x +1;
            basis.y = chargingStation_01.y;
            basis.z = chargingStation_01.z;
            v_Blackboard.at(i + defaultAgents)->setBasisPosition(basis);
            v_Robot.push_back(robot);
        }
        if (i%2 == 1)
        {
            P3DXRobot* robot = new P3DXRobot(v_Blackboard.at(i + defaultAgents), n, decentralizedCommunication);
            v_Robot.push_back(robot);
        }
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
    
    
    /*{
        std::string m = "Measure1";
        strcpy(mission.missionCode, m.c_str());
        mission.operation = enum_MissionOperation::createMission;
        mission.taskToBeDecomposed = enum_DecomposableTask::checkPosition;
        mission.robotCat = enum_RobotCategory::uav;
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
    }*/
    for(int times = 1 ; times < 2; times++ )
    {
        std::string m = "Inspect"+ std::to_string(times);
        strcpy(mission.missionCode, m.c_str());
        mission.operation = enum_MissionOperation::createMission;
        mission.taskToBeDecomposed = enum_DecomposableTask::inspectArea;
        mission.robotCat = enum_RobotCategory::uav;
        mission.numberOfAttributes = 5;
        
        int total = 4;
        *((int*) (mission.attributesBuffer + total)) = sizeof(s_pose);
        total += 4;
        memcpy(mission.attributesBuffer + total, &area_01, sizeof(s_pose));
        total += sizeof(s_pose);
        *((int*) (mission.attributesBuffer + total)) = sizeof(s_pose);
        
        total += 4;
        memcpy(mission.attributesBuffer + total, &area_02, sizeof(s_pose));
        total += sizeof(s_pose);
        *((int*) (mission.attributesBuffer + total)) = sizeof(s_pose);
        
        total += 4;
        memcpy(mission.attributesBuffer + total, &area_03, sizeof(s_pose));
        total += sizeof(s_pose);
        *((int*) (mission.attributesBuffer + total)) = sizeof(s_pose);
        total += 4;
        memcpy(mission.attributesBuffer + total, &area_04, sizeof(s_pose));
        total += sizeof(s_pose);
        *((int*) (mission.attributesBuffer + total)) = sizeof(s_pose);
        total += 4;
        memcpy(mission.attributesBuffer + total, &area_05, sizeof(s_pose));
        total += sizeof(s_pose);
        *((int*) (mission.attributesBuffer)) = total;
        
        mission.relativeDeadline = std::chrono::seconds(380);
        
        memcpy(message.buffer,"CStation1",10);
        *((Operation*)(message.buffer + 10)) = operation;
        *((int*)(message.buffer + 14)) = sizeof(mission);
        memmove(message.buffer+18,(const unsigned char*)&mission,sizeof(mission));
        message.messageSize = sizeof(message.buffer);
        
        v_Blackboard.at(1)->addUDPMessage(message);
    }

    std::string m = "Small2";
     strcpy(mission.missionCode, m.c_str());
     mission.operation = enum_MissionOperation::createMission;
     mission.taskToBeDecomposed = enum_DecomposableTask::deliverSmallSample;
     mission.robotCat = enum_RobotCategory::ugv;
     mission.numberOfAttributes = 2;
     int total = 4;
     *((int*) (mission.attributesBuffer + total)) = sizeof(s_pose);
     total += 4;
     memcpy(mission.attributesBuffer + total, &corredor_01, sizeof(s_pose));
     total += sizeof(s_pose);
     *((int*) (mission.attributesBuffer + total)) = sizeof(s_pose);
     total += 4;
     memcpy(mission.attributesBuffer + total, &area_02, sizeof(s_pose));
     total += sizeof(s_pose);
     *((int*) (mission.attributesBuffer)) = total;
     
     mission.relativeDeadline = std::chrono::seconds(180);
     
     memcpy(message.buffer,"CStation1",10);
     *((Operation*)(message.buffer + 10)) = operation;
     *((int*)(message.buffer + 14)) = sizeof(mission);
     memmove(message.buffer+18,(const unsigned char*)&mission,sizeof(mission));
     message.messageSize = sizeof(message.buffer);
     
     v_Blackboard.at(1)->addUDPMessage(message);
    
    /*
     for(int times = 1 ; times < 5; times++ )
     {
     {
     std::string m = "Measure1"+ std::to_string(times);
     strcpy(mission.missionCode, m.c_str());
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
     //strcpy(mission.missionCode, "Measure2");
     std::string m = "Measure2"+ std::to_string(times);
     strcpy(mission.missionCode, m.c_str());
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
     std::string m = "Inspect1"+ std::to_string(times);
     strcpy(mission.missionCode, m.c_str());
     //strcpy(mission.missionCode, "Inspect1");
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
     //strcpy(mission.missionCode, "Inspect2");
     std::string m = "Inspect2"+ std::to_string(times);
     strcpy(mission.missionCode, m.c_str());
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
     //strcpy(mission.missionCode, "Deliver1");
     std::string m = "Small1"+ std::to_string(times);
     strcpy(mission.missionCode, m.c_str());
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
     //strcpy(mission.missionCode, "Deliver2");
     std::string m = "Small2"+ std::to_string(times);
     strcpy(mission.missionCode, m.c_str());
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
     
     {
     //strcpy(mission.missionCode, "Deliver3");
     std::string m = "Big1"+ std::to_string(times);
     strcpy(mission.missionCode, m.c_str());
     mission.operation = enum_MissionOperation::createMission;
     mission.taskToBeDecomposed = enum_DecomposableTask::deliverBigSample;
     mission.robotCat = enum_RobotCategory::ugv;
     mission.numberOfAttributes = 2;
     int total = 4;
     *((int*) (mission.attributesBuffer + total)) = sizeof(s_pose);
     total += 4;
     memcpy(mission.attributesBuffer + total, &sala_A02, sizeof(s_pose));
     total += sizeof(s_pose);
     *((int*) (mission.attributesBuffer + total)) = sizeof(s_pose);
     total += 4;
     memcpy(mission.attributesBuffer + total, &deposito_01, sizeof(s_pose));
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
     //strcpy(mission.missionCode, "Deliver4");
     std::string m = "Big2"+ std::to_string(times);
     strcpy(mission.missionCode, m.c_str());
     mission.operation = enum_MissionOperation::createMission;
     mission.taskToBeDecomposed = enum_DecomposableTask::deliverBigSample;
     mission.robotCat = enum_RobotCategory::ugv;
     mission.numberOfAttributes = 2;
     int total = 4;
     *((int*) (mission.attributesBuffer + total)) = sizeof(s_pose);
     total += 4;
     memcpy(mission.attributesBuffer + total, &sala_A04, sizeof(s_pose));
     total += sizeof(s_pose);
     *((int*) (mission.attributesBuffer + total)) = sizeof(s_pose);
     total += 4;
     memcpy(mission.attributesBuffer + total, &deposito_02, sizeof(s_pose));
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
     }*/
    while (std::getchar() != 'c'){}
    
    //delete logger;
    return 0;
}



