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
#include "Logger.hpp"
#include "dataTypes.hpp"
#include "BatteryManager.hpp"
#include "Agent.hpp"

// Communication Modules
#include "UDPBroadcast.hpp"
#include "UDPReceiver.hpp"
#include "UDPSender.hpp"

#include "Auction.hpp"
#include "TaskModule.hpp"

#include "GoToSim.hpp"
#include "ChargeBatterySim.hpp"
#include "TurnOnSim.hpp"
#include "TakePictureSim.hpp"

class DefaultRobot: public Agent
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
    char mode[MAX_IP];
public:
    DefaultRobot(BlackBoard* monitor, bool decentralized); // SEND ARGS
    ~DefaultRobot();
    virtual bool addAtomicTask(MissionExecution& vMissionDecomposable);
    virtual void decomposableTaskList();
};
#endif /* DefaultRobot_hpp */
