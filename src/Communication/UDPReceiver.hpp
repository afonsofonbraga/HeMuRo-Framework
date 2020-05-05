//
//  UDPReceiver.hpp
//  MRSMac
//
//  Created by Afonso Braga on 02/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef UDPReceive_hpp
#define UDPReceive_hpp

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>

#include <string> /* memset() */
#include <chrono> /* select() */
#include <iostream>
#include <thread>
#include <sys/time.h>

#include "BlackBoard.hpp"
#include "Module.hpp"
#include "dataTypes.hpp"
#define MAX_MSG 500
class UDPReceiver: public Module
{
protected:
    int vSocket;
    int rc;
    int n;
    int cliLen;
    int port = 1500;
    char msg[MAX_MSG];
    //std::string broadcastIP{"10.0.0.255"};
    struct sockaddr_in servAddr;
    struct sockaddr_in cliAddr;
    void dataTreatment(char* mensagem);
    
    virtual void run();
public:
    UDPReceiver(BlackBoard* monitor);
    ~UDPReceiver();
    
    void error(const char* msg);
};

#endif /* UDPReceive_hpp */
