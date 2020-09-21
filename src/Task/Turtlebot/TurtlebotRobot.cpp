//
//  TurtlebotRobot.cpp
//  MRSFramework
//
//  Created by Afonso Braga on 02/09/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "TurtlebotRobot.hpp"

TurtlebotRobot::TurtlebotRobot(BlackBoard* monitor, ros::NodeHandle& vNode, bool decentralized)
{
    this->decentralized = decentralized;
    monitor->setRobotCategory(enum_RobotCategory::ugv);
    broadcast = new UDPBroadcast(monitor);
    if (this->decentralized == true)
        receiver = new UDPReceiver(monitor);
    sender = new UDPSender(monitor);
    missionManager = new MissionManager(monitor);
    alive = new Alive(monitor, vNode);
}

TurtlebotRobot::~TurtlebotRobot()
{
    delete this->broadcast;
    if (this->decentralized == true)
        delete this->receiver;
    delete this->sender;
    delete this->missionManager;
    delete this->alive;
}
