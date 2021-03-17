//
//  PickUpSim.cpp
//  MRSMac
//
//  Created by Afonso Braga on 10/03/21.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "PickUpSim.hpp"

PickUpSim::PickUpSim(Blackboard* vMonitor, s_pose& start, s_pose& end) : AtomicTask(vMonitor, start, end)
{
    costFactor = 1.0;
    this->timeFactor = 2000; // 2s
    calculateCost();
}

PickUpSim::~PickUpSim() {}

void PickUpSim::run()
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
            this->monitor->print("Picking up Sample.");
            
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

void PickUpSim::calculateCost()
{
    this->cost = this->costFactor;
}

void PickUpSim::calculateTime()
{
    this->time = std::chrono::milliseconds(int(this->timeFactor));
}
