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

#include "RosbotRobot.hpp"
#include <math.h>

int main( int argc, char *argv[ ] )
{
    
    if (argc < 2)
    {
        std::cerr << "Please, inform the robot's name:";
        return 0;
    }
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

    std::vector<Blackboard *> v_Blackboard; // = new std::vector<Blackboard>;
    UDPReceiverSim* receiver = new UDPReceiverSim();
 
    std::vector<RosbotRobot* > v_Robot;

    std::string node = "node"; //name;
    ros::init(argc, argv ,node);
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(0);
    spinner.start();
    
    std::string robotsName = "Logger";
    Blackboard* memory = new Blackboard(robotsName, enum_RobotCategory::null);
    v_Blackboard.push_back(memory);
    receiver->addRobot(v_Blackboard.at(0));
    bool decentralizedCommunication = false;
    LoggerAgent* logger = new LoggerAgent(v_Blackboard.at(0), decentralizedCommunication);

    for (int i = 1; i < argc; i++)
    {
        robotsName = argv[i];
        Blackboard* memory = new Blackboard(robotsName, enum_RobotCategory::null);
        v_Blackboard.push_back(memory);
        receiver->addRobot(v_Blackboard.at(i));
        decentralizedCommunication = false;

        RosbotRobot* robot = new RosbotRobot(v_Blackboard.at(i), n, decentralizedCommunication);

        v_Robot.push_back(robot);
    }

    std::cout << "Cheguei aqui" << std::endl;



    char vIP[MAX_IP];
    s_TaskMessage mission;
    v_Blackboard.at(1)->getRobotsIP(*vIP);
    v_Blackboard.at(1)->getRobotsName(*mission.senderName);
    strcpy(mission.senderAddress , vIP);
    mission.operation = enum_TaskMessage::addAndExecute;
    
    {
        strcpy(mission.missionCode, "Task2");
        mission.taskToBeDecomposed = enum_DecomposableTask::checkPosition;
        mission.robotCat = enum_RobotCategory::ugv;

        mission.goal = sala_A01;
        mission.numberOfAttributes = 1;
        *((int*) (mission.attributesBuffer + 4)) = sizeof(sala_A01);
        memcpy(mission.attributesBuffer + 8, &sala_A01, sizeof(sala_A01));
        *((int*) (mission.attributesBuffer)) = sizeof(sala_A01) + 8;
          
        mission.relativeDeadline = std::chrono::milliseconds(60);
        
    
        
        v_Blackboard.at(1)->addTaskMessage(mission);
    }

    s_pose position;
    v_Blackboard.at(1)->getPosition(position);
    s_pose new_position;
    float distance = 0;
    std::chrono::milliseconds tick = std::chrono::milliseconds(100);    /*!< Threads period */
    auto t1 = std::chrono::high_resolution_clock::now();
    auto start = std::chrono::high_resolution_clock::now();
    auto t0 = std::chrono::high_resolution_clock::now();
    while(true)
    {
        std::this_thread::sleep_until(t1 + tick);
        std::cout << "Trying" << std::endl;
        if(v_Blackboard.at(1)->isRobotAvailable() == false)
        {
            start = std::chrono::high_resolution_clock::now();
            t0 = std::chrono::high_resolution_clock::now();
            while(v_Blackboard.at(1)->isRobotAvailable() == false)
            {
                std::this_thread::sleep_until(t0 + tick);
                v_Blackboard.at(1)->getPosition(new_position);
                std::cout << new_position.x << " " << new_position.y << " "<< new_position.z << std::endl;
                distance += sqrt(pow(new_position.x-position.x,2) + pow(new_position.y-position.y,2) + pow(new_position.z-position.z,2));
                position = new_position;
                t0 = t0 + tick;
            }
            std::cout << "Acabou a Missao!!!!!!" << std::endl;
            t0 = std::chrono::high_resolution_clock::now();
            break;
        }
        t1 = t1 + tick*10;
    }
    std::chrono::duration<double> elapsed_seconds = t0-start;
    std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";
    std::cout << "Distance:" << distance << std::endl;
    std::cout << "Speed: " << distance/elapsed_seconds.count() << std::endl;
    while (std::getchar() != 'c'){}

    return 0;
}



