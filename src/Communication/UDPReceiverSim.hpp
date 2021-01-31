//
//  UDPReceiverSim.hpp
//  MRSMac
//
//  Created by Afonso Braga on 01/09/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef UDPReceiverSim_hpp
#define UDPReceiverSim_hpp

#include <iostream>
#include <thread>
#include <chrono>
#include <map>
#include <string> /* memset() */

#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>

#include "Blackboard.hpp"
#include "Module.hpp"
#include "dataTypes.hpp"
#define MAX_MSG 500

class UDPReceiverSim{
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
    void dataTreatment(char* mensagem);             // Treat data according to its type
    void switchoperation(Operation operation, char* temp, const char* name, int size);
    
    std::thread t_main;         // Thread object
    bool isRunning;
    std::unordered_map<std::string, Blackboard*> robotsList;
    virtual void mainThread();  // Configure the Tick
    virtual void run();         // Implementation
    
public:
    UDPReceiverSim();
    ~UDPReceiverSim();
    
    void addRobot(Blackboard* monitor);
    bool getRunningStatus();    // Return isRunning variable
    void start();               // Start Thread
    virtual void stop();        // Pause Running Thread
    
    void error(const char* msg);                    // Print an error
};
#endif /* Module_hpp */
