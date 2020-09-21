//
//  DefaultRobot.hpp
//  MRSMac
//
//  Created by Afonso Braga on 02/09/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef DefaultRobot_hpp
#define DefaultRobot_hpp

#include <stdio.h>

// Core Modules
#include "BlackBoard.hpp"
#include "dataTypes.hpp"

// Communication Modules
#include "UDPBroadcast.hpp"
#include "UDPReceiver.hpp"
#include "UDPSender.hpp"

#include "MissionManager.hpp"

#include "BatteryManager.hpp"

class DefaultRobot
{
protected:
    bool decentralized;
    UDPBroadcast* broadcast;
    UDPReceiver* receiver;
    UDPSender* sender;
    MissionManager* missionManager;
    BatteryManager* batteryManager;
    char mode[16];
public:
    DefaultRobot(BlackBoard* monitor, bool decentralized); // SEND ARGS
    ~DefaultRobot();
};
#endif /* DefaultRobot_hpp */
