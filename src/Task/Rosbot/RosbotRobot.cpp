//
//  RosbotRobot.cpp
//  MRSMac
//
//  Created by Afonso Braga on 02/09/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "RosbotRobot.hpp"

RosbotRobot::RosbotRobot(BlackBoard* monitor, ros::NodeHandle& vNode, bool decentralized)
{
    this->decentralized = decentralized;
    monitor->setRobotCategory(enum_RobotCategory::ugv);
    if (this->decentralized == true)
    {
        receiver = new UDPReceiver(monitor);
        receiver->start();
    }
    
    logger = new Logger(monitor);
    sender = new UDPSender(monitor);
    broadcast = new UDPBroadcast(monitor);
    alive = new Alive(monitor, vNode);
    strcpy(mode,"Robot");
    batteryManager = new BatteryManager(monitor,mode);
    missionManager = new MissionManager(monitor);
    
    logger->Module::start();
    sender->start();
    broadcast->start();
    alive->start();
    batteryManager->start();
    missionManager->start();
}

RosbotRobot::~RosbotRobot()
{
    delete this->broadcast;
    if (this->decentralized == true)
        delete this->receiver;
    delete this->sender;
    delete this->missionManager;
    delete this->alive;
    delete this->batteryManager;
    delete this->logger;
}
