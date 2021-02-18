//
//  TakePictureSim.cpp
//  MRSMac
//
//  Created by Afonso Braga on 27/07/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "TakePictureSim.hpp"

TakePictureSim::TakePictureSim(Blackboard* vMonitor, s_pose& start, s_pose& end) : AtomicTask(vMonitor, start, end)
{
    costFactor = 1.0;
    this->timeFactor = 50; // 50ms
    calculateCost();
}

TakePictureSim::~TakePictureSim() {}

void TakePictureSim::run()
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
            this->monitor->print("Taking a Picture.");
            
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

void TakePictureSim::calculateCost()
{
    this->cost = this->costFactor;
}

void TakePictureSim::calculateTime()
{
    this->time = std::chrono::milliseconds(int(this->timeFactor));
}
