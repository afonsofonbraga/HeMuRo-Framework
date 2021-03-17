//
//  DropOffSim.cpp
//  MRSMac
//
//  Created by Afonso Braga on 10/03/21.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "DropOffSim.hpp"

DropOffSim::DropOffSim(Blackboard* vMonitor, s_pose& start, s_pose& end) : AtomicTask(vMonitor, start, end)
{
    costFactor = 1.0;
    this->timeFactor = 2000; // 2s
    calculateCost();
}

DropOffSim::~DropOffSim() {}

void DropOffSim::run()
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
            this->monitor->print("Dropping off Sample.");
            
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

void DropOffSim::calculateCost()
{
    this->cost = this->costFactor;
}

void DropOffSim::calculateTime()
{
    this->time = std::chrono::milliseconds(int(this->timeFactor));
}
