//
//  UDPBroadcast.hpp
//  MRSMac
//
//  Created by Afonso Braga on 02/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef UDPBroadcast_hpp
#define UDPBroadcast_hpp

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

#include "BlackBoard.hpp"
#include "ModulePeriodic.hpp"
#include "dataTypes.hpp"

class UDPBroadcast: public ModulePeriodic
{
protected:
    int vSocket;
    int rc;
    char i= '0';
    int port = 1500;
    char broadcastIP[MAX_IP] = "null";
    //std::string broadcastIP{"10.0.0.255"};
    char buffer[500];
    struct sockaddr_in cliAddr;
    struct hostent *h;
    virtual void run();                                     // Implementation
    
public:
    UDPBroadcast(BlackBoard* monitor);
    ~UDPBroadcast();
    UDPBroadcast(const UDPBroadcast& other);                //Copy Constructor
    UDPBroadcast& operator=(const UDPBroadcast& other);     //Copy Assignment
    

    void error(const char *msg);
};

#endif /* UDPBroadcast_hpp */
