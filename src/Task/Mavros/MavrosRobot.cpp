//
//  MavrosRobot.cpp
//  MRSFramework
//
//  Created by Afonso Braga on 02/09/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "MavrosRobot.hpp"

MavrosRobot::MavrosRobot(BlackBoard* monitor, ros::NodeHandle& vNode, bool decentralized)
{
    this->decentralized = decentralized;
    monitor->setRobotCategory(enum_RobotCategory::uav);
    broadcast = new UDPBroadcast(monitor);
    if (this->decentralized == true)
        receiver = new UDPReceiver(monitor);
    sender = new UDPSender(monitor);
    missionManager = new MissionManager(monitor);
    alive = new Alive(monitor, vNode);
    
    strcpy(mode,"Robot");
    batteryManager = new BatteryManager(monitor,mode);
}

MavrosRobot::~MavrosRobot()
{
    delete this->broadcast;
    if (this->decentralized == true)
        delete this->receiver;
    delete this->sender;
    delete this->missionManager;
    delete this->alive;
    delete this->batteryManager;
}
