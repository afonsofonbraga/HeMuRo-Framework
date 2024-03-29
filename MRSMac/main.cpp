#include <chrono>
#include <thread>
#include <vector>
#include <iostream>
#include <unordered_map>

#include "Blackboard.hpp"
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
    int numberOfRobots = 2; // Number of robots that will be executing the tasks
    int defaultAgents = 0;
    
    std::vector<Blackboard* > v_Blackboard; // = new std::vector<Blackboard>;
    std::vector<DefaultRobot* > v_DefaultRobot;
    std::vector<ChargingStation* > v_ChargingStation;
    UDPReceiverSim* receiver = new UDPReceiverSim();
    LoggerAgent* logger;
    
    std::string robotsName = "Logger";
    Blackboard* memory = new Blackboard(robotsName, enum_RobotCategory::null);
    v_Blackboard.push_back(memory);
    bool decentralizedCommunication = false;
    logger = new LoggerAgent(v_Blackboard.at(defaultAgents), decentralizedCommunication, argc, argv);
    receiver->addRobot(v_Blackboard.at(defaultAgents));
    defaultAgents++;
    
    robotsName = "CStation1";
    memory = new Blackboard(robotsName, enum_RobotCategory::null);
    v_Blackboard.push_back(memory);
    v_Blackboard.at(defaultAgents)->setPosition(chargingStation_01);
    ChargingStation* station = new ChargingStation(v_Blackboard.at(defaultAgents), decentralizedCommunication);
    receiver->addRobot(v_Blackboard.at(defaultAgents));
    v_ChargingStation.push_back(station);
    defaultAgents++;
    
    
    robotsName = "CStation2";
    memory = new Blackboard(robotsName, enum_RobotCategory::null);
    v_Blackboard.push_back(memory);
    v_Blackboard.at(defaultAgents)->setPosition(chargingStation_02);
    station = new ChargingStation(v_Blackboard.at(defaultAgents), decentralizedCommunication);
    receiver->addRobot(v_Blackboard.at(defaultAgents));
    v_ChargingStation.push_back(station);
    defaultAgents++;
    
    
    for (int i=defaultAgents; i-defaultAgents< numberOfRobots; i++)
    {
        std::string robotsName = name + std::to_string(i-defaultAgents);
        Blackboard* memory = new Blackboard(robotsName, enum_RobotCategory::null);
        v_Blackboard.push_back(memory);
        bool decentralizedCommunication = false;
        DefaultRobot* robot = new DefaultRobot(v_Blackboard.at(i), decentralizedCommunication);
        receiver->addRobot(v_Blackboard.at(i));
        v_DefaultRobot.push_back(robot);
        usleep(1000);
    }
    
    char vIP[MAX_IP];
    
    
    
    //std::cout << "Time to send a Mission!!!!!"<< std::endl;
    
    s_MissionMessage mission;
    v_Blackboard.at(1)->getRobotsIP(*vIP);
    v_Blackboard.at(1)->getRobotsName(*mission.senderName);
    strcpy(mission.senderAddress , vIP);
    mission.operation = enum_MissionOperation::createMission;
    
    s_UDPMessage message;
    strcpy(message.address , vIP);
    Operation operation = Operation::missionMessage;
    v_Blackboard.at(1)->getRobotsName(*message.name);
    
    for(int times = 0 ; times < 1; times++ )
    {
        {
            std::string m = "Temperature"+ std::to_string(times);
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
            std::string m = "Inspect"+ std::to_string(times);
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
            //strcpy(mission.missionCode, "Deliver1");
            std::string m = "SmallPKG"+ std::to_string(times);
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
            //strcpy(mission.missionCode, "Deliver3");
            std::string m = "BigPKG"+ std::to_string(times);
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
    }
    while (std::getchar() != 'c'){}
    return 0;
    
    
}
