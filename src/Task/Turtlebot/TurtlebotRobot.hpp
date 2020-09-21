//
//  TurtlebotRobot.hpp
//  MRSFramework
//
//  Created by Afonso Braga on 02/09/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef TurtlebotRobot_hpp
#define TurtlebotRobot_hpp

#include <stdio.h>

// Core Modules
#include "BlackBoard.hpp"
#include "dataTypes.hpp"

// Communication Modules
#include "UDPBroadcast.hpp"
#include "UDPReceiver.hpp"
#include "UDPSender.hpp"

#include "Alive.hpp"
#include "ros/ros.h"

#include "MissionManager.hpp"

class TurtlebotRobot
{
protected:
    bool decentralized;
    UDPBroadcast* broadcast;
    UDPReceiver* receiver;
    UDPSender* sender;
    MissionManager* missionManager;
    Alive* alive;
public:
    TurtlebotRobot(BlackBoard* monitor, ros::NodeHandle& vNode, bool decentralized); // SEND ARGS
    ~TurtlebotRobot();
#endif /* TurtlebotRobot_hpp */
