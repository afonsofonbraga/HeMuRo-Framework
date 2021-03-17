//
//  Logger.cpp
//  MRSMac
//
//  Created by Afonso Braga on 27/09/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "LoggerAgent.hpp"

LoggerAgent::LoggerAgent(Blackboard* monitor, bool decentralized): Agent(monitor)
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

LoggerAgent::LoggerAgent(Blackboard* monitor, bool decentralized , int argc, char** argv): Agent(monitor)
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
    
    #ifndef DEFAULT
    webmodule = new WebModule(monitor, argc, argv);
    #endif
    
    logger->Module::start();
    #ifndef DEFAULT
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
    #ifndef DEFAULT
    delete this->webmodule;
    #endif
}
