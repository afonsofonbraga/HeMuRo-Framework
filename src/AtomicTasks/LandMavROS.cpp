//
//  LandMavROS.cpp
//  MRSFramework
//
//  Created by Afonso Braga on 25/08/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "LandMavROS.hpp"

LandMavROS::LandMavROS(Blackboard* vMonitor, s_pose& start, s_pose& end) : AtomicTask(vMonitor, start, end)
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
            s_ROSModuleMessage teste;
            strcpy(teste.topicName,"Land");
            this->monitor->addROSModuleMessage(teste);
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

