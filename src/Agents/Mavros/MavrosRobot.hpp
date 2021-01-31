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
#include "Blackboard.hpp"
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

#include "ROSModuleMavros.hpp"
#include "ros/ros.h"


#include "GoToROS.hpp"
#include "TurnOnSim.hpp"
#include "ChargeBatteryROS.hpp"
#include "TakePictureSim.hpp"
#include "ArmMavROS.hpp"
#include "TakeOffMavROS.hpp"
#include "LandMavROS.hpp"

class MavrosRobot
{
protected:
    bool decentralized;
    Logger* logger;
    UDPBroadcast* broadcast;
    UDPReceiver* receiver;
    UDPSender* sender;
    BatteryManager* batteryManager;
    
    Auction* auction;
    TaskModule* taskModule;
    
    ROSModuleMavros* rosModule;
    char mode[MAX_IP];
public:
    MavrosRobot(Blackboard* monitor, ros::NodeHandle& vNode, bool decentralized); // SEND ARGS
    ~MavrosRobot();
    virtual bool addAtomicTask(MissionExecution& vMissionDecomposable);
    virtual void decomposableTaskList();
};
#endif /* MavrosRobot_hpp */
