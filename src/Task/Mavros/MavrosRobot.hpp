//
//  MavrosRobot.hpp
//  MRSFramework
//
//  Created by Afonso Braga on 02/09/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef MavrosRobot_hpp
#define MavrosRobot_hpp

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

class MavrosRobot
{
protected:
    bool decentralized;
    UDPBroadcast* broadcast;
    UDPReceiver* receiver;
    UDPSender* sender;
    MissionManager* missionManager;
    Alive* alive;
public:
    MavrosRobot(BlackBoard* monitor, ros::NodeHandle& vNode, bool decentralized); // SEND ARGS
    ~MavrosRobot();
};
#endif /* MavrosRobot_hpp */
