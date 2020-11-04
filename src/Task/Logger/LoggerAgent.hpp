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
#include "BlackBoard.hpp"
#include "Logger.hpp"
#include "dataTypes.hpp"

// Communication Modules
#include "UDPBroadcast.hpp"
#include "UDPReceiver.hpp"
#include "UDPSender.hpp"

#ifdef DEFAULT
    #include "WebModule.hpp"
#endif


class LoggerAgent
{
protected:
    bool decentralized;
    Logger* logger;
    //UDPBroadcast* broadcast;
    UDPReceiver* receiver;
    #ifdef DEFAULT
            WebModule* webmodule;
    #endif

    //UDPSender* sender;
public:
    LoggerAgent(BlackBoard* monitor, bool decentralized); // SEND ARGS
    LoggerAgent(BlackBoard* monitor, bool decentralized, int argc, char** argv); // SEND ARGS
    ~LoggerAgent();
};

#endif /* LoggerAgent_hpp */
