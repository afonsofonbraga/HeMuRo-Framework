//
//  Logger.cpp
//  MRSMac
//
//  Created by Afonso Braga on 27/09/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "LoggerAgent.hpp"

LoggerAgent::LoggerAgent(BlackBoard* monitor, bool decentralized)
{
    this->decentralized = decentralized;
    monitor->setRobotCategory(enum_RobotCategory::null);
    //broadcast = new UDPBroadcast(monitor);
    if (this->decentralized == true)
        receiver = new UDPReceiver(monitor);
    //sender = new UDPSender(monitor);
    logger = new Logger(monitor);
}

LoggerAgent::~LoggerAgent()
{
    //delete this->broadcast;
    if (this->decentralized == true)
        delete this->receiver;
    //delete this->sender;
    delete this->logger;
}
