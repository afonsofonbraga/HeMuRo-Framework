//
//  ChargingStation.hpp
//  MRSMac
//
//  Created by Afonso Braga on 03/09/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef ChargingStation_hpp
#define ChargingStation_hpp

#include <stdio.h>

// Core Modules
#include "BlackBoard.hpp"
#include "dataTypes.hpp"
#include "Logger.hpp"

// Communication Modules
#include "UDPBroadcast.hpp"
#include "UDPReceiver.hpp"
#include "UDPSender.hpp"

#include "MissionManager.hpp"

#include "BatteryManager.hpp"

#include "Auction.hpp"
#include "TaskModule.hpp"

#include "Agent.hpp"

class ChargingStation: public Agent
{
protected:
    
    bool decentralized;
    Logger* logger;
    UDPBroadcast* broadcast;
    UDPReceiver* receiver;
    UDPSender* sender;
    //MissionManager* missionManager;
    
    Auction* auction;
    TaskModule* taskModule;
    
    BatteryManager* batteryManager;
    
public:
    ChargingStation(BlackBoard* monitor, bool decentralized);
    ~ChargingStation();
};
#endif /* ChargingStation_hpp */
