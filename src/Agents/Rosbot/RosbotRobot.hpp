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

#include "ROSModuleRosbot.hpp"
#include "ros/ros.h"

#include "GoToROS.hpp"
#include "MoveBaseGoal.hpp"
#include "TurnOnSim.hpp"
#include "ChargeBatteryROS.hpp"
#include "TakePictureSim.hpp"

class RosbotRobot: public Agent
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
    
    ROSModuleRosbot* rosModule;
    char mode[MAX_IP];
public:
    RosbotRobot(Blackboard* monitor, ros::NodeHandle& vNode, bool decentralized); // SEND ARGS
    ~RosbotRobot();
    virtual bool addAtomicTask(MissionExecution& vMissionDecomposable);
    virtual void decomposableTaskList();
};
#endif /* Rosbot_hpp */
