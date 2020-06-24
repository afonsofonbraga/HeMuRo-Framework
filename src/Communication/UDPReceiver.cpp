//
//  UDPReceiver.cpp
//  MRSMac
//
//  Created by Afonso Braga on 02/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "UDPReceiver.hpp"

UDPReceiver::UDPReceiver(BlackBoard *monitor): Module(monitor)
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
    
}

UDPReceiver::~UDPReceiver()
{
    
}

void UDPReceiver::run()
{
    n = (int) recvfrom( vSocket, msg, MAX_MSG, 0, (struct sockaddr *) &cliAddr, (socklen_t*) &cliLen);
    if(n<0)
        std::cout << "Cannot receive data \n";
    else
    {
        this->dataTreatment(msg);
    }
}

void UDPReceiver::error(const char *msg){
    perror(msg);
    exit(1);
}

void UDPReceiver::dataTreatment(char *mensagem)
{
    //int operation = mensagem[0];
    Operation operation = ((Operation*) mensagem)[0];
    int dataSize;
    char *temp = mensagem + sizeof(operation);
    dataSize = ((int*) temp)[0];
    temp = temp + sizeof(dataSize);
    switch(operation){
        case Operation::null:
            std::cout << "deuruim" <<std::endl;
            break;
            
        case Operation::setRobotsPosition:
        {
            s_robotsPose robotPosition = ((s_robotsPose*) temp)[0];
            this->monitor->setAllRobotsPosition(robotPosition);
            break;
        }
            
        case Operation::missionMessage:
        {
            s_MissionMessage missionMessage = ((s_MissionMessage*) temp)[0];
            this->monitor->addMissionMessage(missionMessage);
            break;
        }
    }
}
