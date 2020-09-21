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
    while(this->isRunning)
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

void UDPReceiverSim::addRobot(BlackBoard *monitor)
{
    std::string name;
    monitor->getRobotsName(name);
    this->robotsList[name] = monitor;
}


void UDPReceiverSim::dataTreatment(char *mensagem)
{
    char name[10];
    memcpy(name, mensagem, 10);
    
    char *temp = mensagem + 10;
    Operation operation = ((Operation*) temp)[0];
    int dataSize;
    temp = temp + sizeof(operation);
    dataSize = ((int*) temp)[0];
    temp = temp + sizeof(dataSize);
    
    if (strcmp(name, "Broadcast") == 0)
    {
        for (auto n : robotsList)
        {
            switchoperation(operation, temp, n.first.c_str());
            
        }
        
    }else
    {
        auto search = this->robotsList.find(name);
        if (search != this->robotsList.end())
        {
            switchoperation(operation, temp, name);
        }
        else
        {
            std::cout << "There is no robot with the name: " << name << std::endl;
        }
    }
    
}

void UDPReceiverSim::switchoperation(Operation operation, char* temp, const char* name)
{
    switch(operation){
        case Operation::null:
            std::cout << "deuruim" <<std::endl;
            break;
            
        case Operation::setRobotsPosition:
        {
            s_robotsPose robotPosition = ((s_robotsPose*) temp)[0];
            this->robotsList[name]->setAllRobotsPosition(robotPosition);
            break;
        }
            
        case Operation::missionMessage:
        {
            s_MissionMessage missionMessage = ((s_MissionMessage*) temp)[0];
            this->robotsList[name]->addMissionMessage(missionMessage);
            break;
        }
        case Operation::batteryMessage:
        {
            s_BatteryMessage batteryMessage = ((s_BatteryMessage*) temp)[0];
            this->robotsList[name]->addBatteryMessage(batteryMessage);
            break;
        }
    }
}

