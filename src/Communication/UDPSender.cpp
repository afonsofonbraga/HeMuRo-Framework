//
//  UDPSender.cpp
//  MRSMac
//
//  Created by Afonso Braga on 28/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "UDPSender.hpp"

UDPSender::UDPSender(BlackBoard* monitor): Module(monitor)
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
    // Receiver IP will be set up on the run void
    // this->cliAddr.sin_addr.s_addr = inet_addr("10.0.0.255");
    this->cliAddr.sin_port = htons(this->port);
}

UDPSender::~UDPSender()
{
    this->stop();
    this->monitor->conditional_UDPMessageList.notify_one();
    close(this->vSocket);
    if (this->t_main.joinable())
        this->t_main.join();
}

UDPSender::UDPSender(const UDPSender& other): Module(other.monitor)
{
    this->vSocket = socket(AF_INET,SOCK_DGRAM,0);
    if(this->vSocket<0)
        error("%s: cannot open socket\n");

    this->cliAddr.sin_family = AF_INET;
    // Receiver IP will be set up on the run void
    // this->cliAddr.sin_addr.s_addr = inet_addr("10.0.0.255");
    this->cliAddr.sin_port = htons(this->port);
}

UDPSender& UDPSender::operator =(const UDPSender& other)
{
    if(this!= &other)
    {
        this->monitor = other.monitor;
        
        this->vSocket = socket(AF_INET,SOCK_DGRAM,0);
        if(this->vSocket<0)
            error("%s: cannot open socket\n");

        this->cliAddr.sin_family = AF_INET;
        // Receiver IP will be set up on the run void
        // this->cliAddr.sin_addr.s_addr = inet_addr("10.0.0.255");
        this->cliAddr.sin_port = htons(this->port);
    }
    return *this;
}




void UDPSender::run(){
    vUDPMessage = new s_UDPMessage();
    this->monitor->getUDPMessage(*vUDPMessage);
    
    if (vUDPMessage != nullptr && this->isRunning == true)
    {
        //std::cout << "Sending a message through UDPSender!!" <<std::endl;
        this->cliAddr.sin_addr.s_addr = inet_addr(vUDPMessage->address);
        rc = (int) sendto(this->vSocket, vUDPMessage->buffer, vUDPMessage->messageSize, 0, (struct sockaddr*) &this->cliAddr, sizeof(this->cliAddr));
    }
}

void UDPSender::error(const char *msg){
    perror(msg);
    exit(1);
}
