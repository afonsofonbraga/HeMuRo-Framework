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


class LoggerAgent
{
protected:
    bool decentralized;
    Logger* logger;
    //UDPBroadcast* broadcast;
    UDPReceiver* receiver;
    //UDPSender* sender;
public:
    LoggerAgent(BlackBoard* monitor, bool decentralized); // SEND ARGS
    ~LoggerAgent();
};

#endif /* LoggerAgent_hpp */
