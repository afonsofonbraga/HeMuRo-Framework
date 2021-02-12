//
//  UDPSender.hpp
//  MRSMac
//
//  Created by Afonso Braga on 28/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef UDPSender_hpp
#define UDPSender_hpp

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

#include "Blackboard.hpp"
#include "Module.hpp"
#include "dataTypes.hpp"

class UDPSender: public Module
{
protected:
    int vSocket;
    int rc;
    char i= '0';
    int port = 1500;
    s_UDPMessage* vUDPMessage;
    //std::string broadcastIP{"10.0.0.255"};
    char buffer[500];
    struct sockaddr_in cliAddr;
    struct hostent *h;
    void run() override;                                     // Implementation
    
public:
    UDPSender(Blackboard* monitor);
    ~UDPSender();
    UDPSender(const UDPSender& other);                //Copy Constructor
    UDPSender& operator=(const UDPSender& other);     //Copy Assignment
//    template <class T> void print(T a){
//        std::cout<<a<<std::endl;
//    }

    void error(const char *msg);
};

#endif /* UDPSender_hpp */
