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
    {
        receiver = new UDPReceiver(monitor);
        receiver->start();
    }
        
    //sender = new UDPSender(monitor);
    logger = new Logger(monitor);
    logger->Module::start();
    
    
}

LoggerAgent::LoggerAgent(BlackBoard* monitor, bool decentralized , int argc, char** argv)
{
    this->decentralized = decentralized;
    monitor->setRobotCategory(enum_RobotCategory::null);
    //broadcast = new UDPBroadcast(monitor);
    if (this->decentralized == true)
    {
        receiver = new UDPReceiver(monitor);
        receiver->start();
    }
        
    //sender = new UDPSender(monitor);
    logger = new Logger(monitor);
    
    #ifdef DEFAULT
    webmodule = new WebModule(monitor, argc, argv);
    #endif
    
    logger->Module::start();
    #ifdef DEFAULT
    webmodule->Module::start();
    #endif
}

LoggerAgent::~LoggerAgent()
{
    //delete this->broadcast;
    if (this->decentralized == true)
        delete this->receiver;
    //delete this->sender;
    delete this->logger;
    #ifdef DEFAULT
    delete this->webmodule;
    #endif
}
