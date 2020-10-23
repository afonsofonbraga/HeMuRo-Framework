//
//  DefaultRobot.cpp
//  MRSMac
//
//  Created by Afonso Braga on 02/09/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "DefaultRobot.hpp"

DefaultRobot::DefaultRobot(BlackBoard* monitor, bool decentralized)
{
    strcpy(mode,"Robot");
    this->decentralized = decentralized;
    monitor->setRobotCategory(enum_RobotCategory::ugv);
    broadcast = new UDPBroadcast(monitor);
    sender = new UDPSender(monitor);
    missionManager = new MissionManager(monitor);
    batteryManager = new BatteryManager(monitor,mode);
    
    if (this->decentralized == true)
    {
        receiver = new UDPReceiver(monitor);
        logger = new Logger(monitor);
        
        logger->Module::start();
        receiver->start();
    }

    sender->start();
    broadcast->start();
    batteryManager->start();
    missionManager->start();
}

DefaultRobot::~DefaultRobot()
{
    delete this->broadcast;
    if (this->decentralized == true)
    {
        delete this->receiver;
        delete this->logger;
    }
        
    delete this->sender;
    delete this->missionManager;
    delete this->batteryManager;
}
