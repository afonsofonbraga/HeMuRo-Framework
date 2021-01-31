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
            this->monitor->print("Taking a Picture.");
            //std::cout << "Taking a Picture."<< std::endl;
            this->status = enum_AtomicTaskStatus::completed;
            break;
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

