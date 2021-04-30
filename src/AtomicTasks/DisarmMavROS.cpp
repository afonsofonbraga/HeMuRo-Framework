//
//  DisarmMavROS.cpp
//  MRSMac
//
//  Created by Afonso Braga on 30/04/21.
//  Copyright © 2021 Afonso Braga. All rights reserved.
//

#include "DisarmMavROS.hpp"

DisarmMavROS::DisarmMavROS(Blackboard* vMonitor, s_pose& start, s_pose& end) : AtomicTask(vMonitor, start, end)
{
    calculateCost();
    
    this->timeFactor = 1000;
    calculateTime();
}

DisarmMavROS::~DisarmMavROS() {}

void DisarmMavROS::run()
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
            std::cout << "Disarming."<< std::endl;
            s_ROSModuleMessage teste;
            strcpy(teste.topicName,"Disarm");
            this->monitor->addROSModuleMessage(teste);
            
            auto t0 = std::chrono::system_clock::now();
            std::this_thread::sleep_until(t0 + this->time);
            
            this->status = enum_AtomicTaskStatus::completed;
            break;
        }
            
        case enum_AtomicTaskStatus::completed:
            break;
        default:
            break;
    }
}

void DisarmMavROS::calculateCost()
{
    this->cost = this->costMeter;
}

void DisarmMavROS::calculateTime()
{
    this->time = std::chrono::milliseconds(int(this->timeFactor));
}
