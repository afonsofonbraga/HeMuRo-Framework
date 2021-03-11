//
//  P3DXRobot.hpp
//  MRSMac
//
//  Created by Afonso Braga on 10/03/21.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef P3DXRobot_hpp
#define P3DXRobot_hpp

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

#include "ROSModuleP3DX.hpp"
#include "ros/ros.h"

#include "GoToROS.hpp"
#include "MoveBaseGoal.hpp"
#include "TurnOnSim.hpp"
#include "ChargeBatteryROS.hpp"
#include "TakePictureSim.hpp"

class P3DXRobot: public Agent
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
    
    ROSModuleP3DX* rosModule;
    char mode[MAX_IP];
public:
    P3DXRobot(Blackboard* monitor, ros::NodeHandle& vNode, bool decentralized); // SEND ARGS
    ~P3DXRobot();
    virtual bool addAtomicTask(MissionExecution& vMissionDecomposable);
    virtual void decomposableTaskList();
};
#endif /* P3DXRobot_hpp */
