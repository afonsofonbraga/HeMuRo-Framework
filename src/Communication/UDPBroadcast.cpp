//
//  UDPBroadcast.cpp
//  MRSMac
//
//  Created by Afonso Braga on 02/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//
#include "UDPBroadcast.hpp"

UDPBroadcast::UDPBroadcast(Blackboard* monitor): ModulePeriodic(monitor)
{
    this->monitor->getBroadcastIP(*this->broadcastIP);
    this->vSocket = socket(AF_INET,SOCK_DGRAM,0);
    if(this->vSocket<0)
        error("%s: cannot open socket\n");
    
    int broadcast = 1;
    if(setsockopt(this->vSocket,SOL_SOCKET,SO_BROADCAST,&broadcast,sizeof(broadcast)) < 0)
    {
        close(this->vSocket);
        error("Error in setting Broadcast option \n");
    }

    this->cliAddr.sin_family = AF_INET;
    this->cliAddr.sin_addr.s_addr = inet_addr(this->broadcastIP);
    this->cliAddr.sin_port = htons(this->port);
}

UDPBroadcast::~UDPBroadcast()
{
    this->stop();
    close(this->vSocket);
    if (this->t_main.joinable())
        this->t_main.join();
}

UDPBroadcast::UDPBroadcast(const UDPBroadcast& other): ModulePeriodic(other.monitor)
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

    this->cliAddr.sin_family = AF_INET;
    this->cliAddr.sin_addr.s_addr = inet_addr(this->broadcastIP);
    this->cliAddr.sin_port = htons(this->port);
}

UDPBroadcast& UDPBroadcast::operator =(const UDPBroadcast& other)
{
    if(this!= &other)
    {
        this->monitor = other.monitor;
        
        this->vSocket = socket(AF_INET,SOCK_DGRAM,0);
        if(this->vSocket<0)
            error("%s: cannot open socket\n");
        
        int broadcast = 1;
        if(setsockopt(this->vSocket,SOL_SOCKET,SO_BROADCAST,&broadcast,sizeof(broadcast)) < 0)
        {
            close(this->vSocket);
            error("Error in setting Broadcast option \n");
        }

        this->cliAddr.sin_family = AF_INET;
        this->cliAddr.sin_addr.s_addr = inet_addr(this->broadcastIP);
        this->cliAddr.sin_port = htons(this->port);
    }
    return *this;
}




void UDPBroadcast::run(){
    char broadcast[MAX_ROBOT_ID] = "Broadcast";
    memcpy(buffer,broadcast,MAX_ROBOT_ID);
    
    Operation operation = Operation::setRobotsPosition;
    *((Operation*)(buffer + MAX_ROBOT_ID)) = operation;
    
    s_BroadcastMessage broadcastMessage;
    
    monitor->getRobotsName(*broadcastMessage.robotName);
    broadcastMessage.robotCategory = monitor->getRobotsCategory();
    broadcastMessage.robotStatus = monitor->getRobotStatus();
    broadcastMessage.batteryLevel = monitor->getBatteryLevel();
    monitor->getPosition(broadcastMessage.robotsPosition);
    
    
    *((int*)(buffer + MAX_ROBOT_ID + 4)) = sizeof(broadcastMessage);
    
    memmove(buffer + MAX_ROBOT_ID + 8,(const unsigned char*)&broadcastMessage,sizeof(broadcastMessage));
    
    rc = (int) sendto(this->vSocket,buffer, MAX_ROBOT_ID + 8 + sizeof(broadcastMessage), 0, (struct sockaddr *) &cliAddr, sizeof(cliAddr));
}

void UDPBroadcast::error(const char *msg){
    perror(msg);
    exit(1);
}

