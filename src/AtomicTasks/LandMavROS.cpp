//
//  LandMavROS.cpp
//  MRSFramework
//
//  Created by Afonso Braga on 25/08/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "LandMavROS.hpp"

LandMavROS::LandMavROS(BlackBoard* vMonitor, s_pose& start, s_pose& end) : AtomicTask(vMonitor, start, end)
{
    calculateCost();
}

LandMavROS::~LandMavROS() {}

void LandMavROS::run()
{
    switch(this->status)
    {
        case enum_AtomicTaskStatus::null:
            break;
            
        case enum_AtomicTaskStatus::waiting:
            this->status = enum_AtomicTaskStatus::running;
            break;
            
        case enum_AtomicTaskStatus::running:
        {
            std::cout << "Landing."<< std::endl;
            s_ROSBridgeMessage teste;
            strcpy(teste.topicName,"Land");
            this->monitor->addROSBridgeMessage(teste);
            this->status = enum_AtomicTaskStatus::completed;
            break;
        }
            
        case enum_AtomicTaskStatus::completed:
            break;
        default:
            break;
    }
}

void LandMavROS::calculateCost()
{
    this->cost = this->costMeter;
}

