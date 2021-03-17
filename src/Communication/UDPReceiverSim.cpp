//
//  UDPReceiverSim.cpp
//  MRSMac
//
//  Created by Afonso Braga on 01/09/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "UDPReceiverSim.hpp"


UDPReceiverSim::UDPReceiverSim()
{
    this->vSocket = socket(AF_INET,SOCK_DGRAM,0);
    if(this->vSocket<0)
        error("%s: cannot open socket\n");
    
    int broadcast = 1;
    if(setsockopt(this->vSocket,SOL_SOCKET,SO_BROADCAST,&broadcast,sizeof(broadcast)) < 0)
    {
        close(this->vSocket);
        error("Error in setting Broadcast option \n");
    }
    
    if(setsockopt(vSocket,SOL_SOCKET,SO_REUSEPORT,&broadcast,sizeof(broadcast)) < 0)
    {
        close(this->vSocket);
        error("Error in setting Broadcast option \n");
    }
    
    this->servAddr.sin_family = AF_INET;
    this->servAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    this->servAddr.sin_port = htons(this->port);
    
    rc = bind (vSocket, (struct sockaddr *) &servAddr,sizeof(servAddr));
    if(rc<0)
        error("Cannot bind!!!");
    
    cliLen = sizeof(this->cliAddr);
    
    
    this->isRunning = false;
    this->start();
}

UDPReceiverSim::~UDPReceiverSim()
{
    this->stop();
    //this->monitor->conditional_task.notify_one();
    if (this->t_main.joinable())
        this->t_main.join();
}

void UDPReceiverSim::error(const char *msg){
    perror(msg);
    exit(1);
}

void UDPReceiverSim::mainThread()
{
    while(this->isRunning == true)
    {
        this->run();
    }
}

void UDPReceiverSim::run()
{
    n = (int) recvfrom( vSocket, msg, MAX_MSG, 0, (struct sockaddr *) &cliAddr, (socklen_t*) &cliLen);
    if(n<0)
        std::cout << "Cannot receive data \n";
    else
    {
        this->dataTreatment(msg);
    }
}

bool UDPReceiverSim::getRunningStatus()
{
    return this->isRunning;
}

void UDPReceiverSim::start()
{
    this->isRunning = true;
    t_main = std::thread(&UDPReceiverSim::mainThread,this);
}

void UDPReceiverSim::stop()
{
    this->isRunning = false;
}

void UDPReceiverSim::addRobot(Blackboard *monitor)
{
    std::string name;
    monitor->getRobotsName(name);
    this->robotsList[name] = monitor;
}


void UDPReceiverSim::dataTreatment(char *mensagem)
{
    char name[MAX_ROBOT_ID];
    memcpy(name, mensagem, MAX_ROBOT_ID);
    
    char *temp = mensagem + MAX_ROBOT_ID;
    Operation operation = ((Operation*) temp)[0];
    int dataSize;
    temp = temp + sizeof(operation);
    dataSize = ((int*) temp)[0];
    temp = temp + sizeof(dataSize);
    
    if (strcmp(name, "Broadcast") == 0)
    {
        for (auto n : robotsList)
        {
            switchoperation(operation, temp, n.first.c_str(), dataSize);
            
        }
        
    }else
    {
        auto search = this->robotsList.find(name);
        if (search != this->robotsList.end())
        {
            switchoperation(operation, temp, name, dataSize);
        }
        else
        {
            std::cout << "There is no robot with the name: " << name << std::endl;
        }
    }
    
}

void UDPReceiverSim::switchoperation(Operation operation, char* temp, const char* name, int size)
{
    switch(operation){
        case Operation::null:
            std::cout << "deuruim" <<std::endl;
            break;
            
        case Operation::setRobotsPosition:
        {
//            s_robotsPose robotPosition = ((s_robotsPose*) temp)[0];
//            this->robotsList[name]->setAllRobotsPosition(robotPosition);
            
            s_BroadcastMessage broadcastMessage = ((s_BroadcastMessage*) temp)[0];
            this->robotsList[name]->setAllRobotsPosition(broadcastMessage);
            break;
        }
            
        case Operation::missionMessage:
        {
            //            char missionCode[MAX_ID] = "null";
            //            char senderAddress[MAX_IP] = "null";
            //            char senderName[MAX_ROBOT_ID] = "null";
            //            enum_MissionOperation operation = enum_MissionOperation::null;
            //            enum_DecomposableTask taskToBeDecomposed = enum_DecomposableTask::null;
            //            float Cost = 0;
            //            //char buffer[500] = "null";
            //            enum_RobotCategory robotCat = enum_RobotCategory::null;
            //            int executionTime = 0;
            //            int numberOfGoals = 0;
            //            std::queue<s_pose> goal;
            //std::cout << "teste 1"<< std::endl;
            s_MissionMessage missionMessage = ((s_MissionMessage*) temp)[0];
            //std::cout << "teste 2"<< std::endl;
            //memcpy(&missionMessage,temp,size;
            /*
            s_MissionMessage missionMessage;
            memcpy(missionMessage.missionCode, temp, MAX_ID);
            temp = temp + MAX_ID;
            memcpy(missionMessage.senderAddress, temp, MAX_IP);
            temp = temp + MAX_IP;
            memcpy(missionMessage.senderName, temp, MAX_ROBOT_ID);
            temp = temp + MAX_ROBOT_ID;
            missionMessage.operation = ((enum_MissionOperation*) temp)[0];
            temp = temp + 4;
            missionMessage.taskToBeDecomposed = ((enum_DecomposableTask*) temp)[0];
            temp = temp + 4;
            missionMessage.Cost = ((float*) temp)[0];
            temp = temp + 4;
            missionMessage.robotCat = ((enum_RobotCategory*) temp)[0];
            temp = temp + 4;
            missionMessage.executionTime = ((int*) temp)[0];
            temp = temp + 4;
            missionMessage.numberOfGoals = ((float*) temp)[0];
            temp = temp + 4; */
            this->robotsList[name]->addMissionMessage(missionMessage);
            //std::cout << "teste 3"<< std::endl;
            break;
        }
        case Operation::batteryMessage:
        {
            s_BatteryMessage batteryMessage = ((s_BatteryMessage*) temp)[0];
            this->robotsList[name]->addBatteryMessage(batteryMessage);
            break;
        }
        case Operation::loggerMessage:
        {
            s_LoggerMessage loggerMessage;// = ((s_LoggerMessage*) temp[0]);
            
            memcpy(&loggerMessage,temp,sizeof(s_LoggerMessage));
            this->robotsList[name]->addLoggerMessage(loggerMessage);
            break;
        }
        default:
            break;

    }
}

