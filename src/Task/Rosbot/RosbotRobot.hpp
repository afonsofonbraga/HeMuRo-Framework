//
//  RosbotRobot.hpp
//  MRSMac
//
//  Created by Afonso Braga on 02/09/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef RosbotRobot_hpp
#define RosbotRobot_hpp

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

#include "BatteryManager.hpp"

class RosbotRobot
{
protected:
    bool decentralized;
    UDPBroadcast* broadcast;
    UDPReceiver* receiver;
    UDPSender* sender;
    MissionManager* missionManager;
    Alive* alive;
    
    BatteryManager* batteryManager;
    char mode[16];
public:
    RosbotRobot(BlackBoard* monitor, ros::NodeHandle& vNode, bool decentralized); // SEND ARGS
    ~RosbotRobot();
};
#endif /* Rosbot_hpp */
