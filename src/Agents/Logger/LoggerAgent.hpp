//
//  Logger.hpp
//  MRSMac
//
//  Created by Afonso Braga on 27/09/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef LoggerAgent_hpp
#define LoggerAgent_hpp

#include <stdio.h>

// Core Modules
#include "Blackboard.hpp"
#include "Logger.hpp"
#include "dataTypes.hpp"

// Communication Modules
#include "UDPBroadcast.hpp"
#include "UDPReceiver.hpp"
#include "UDPSender.hpp"

//#ifndef DEFAULT
    #include "WebModule.hpp"
//#endif

#include "Agent.hpp"

class LoggerAgent: public Agent
{
protected:
    bool decentralized;
    Logger* logger;
    //UDPBroadcast* broadcast;
    UDPReceiver* receiver;
 //   #ifndef DEFAULT
            WebModule* webmodule;
 //   #endif

    //UDPSender* sender;
public:
    LoggerAgent(Blackboard* monitor, bool decentralized); // SEND ARGS
    LoggerAgent(Blackboard* monitor, bool decentralized, int argc, char** argv); // SEND ARGS
    ~LoggerAgent();
};

#endif /* LoggerAgent_hpp */
