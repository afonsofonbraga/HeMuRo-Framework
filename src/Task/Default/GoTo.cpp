//
//  GoTo.cpp
//  MRSMac
//
//  Created by Afonso Braga on 06/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "GoTo.hpp"

GoTo::GoTo(BlackBoard* vMonitor, s_pose& start, s_pose& end) : AtomicTask(vMonitor, start, end)
{
    calculateCost();
}

GoTo::~GoTo(){}

void GoTo::run()
{
    switch(this->status)
    {
        case enum_AtomicTaskStatus::null:
            break;
            
        case enum_AtomicTaskStatus::waiting:
            std::cout << "Going to the location -> x: " << this->endPosition.x << " Y: " << this->endPosition.y << std::endl;
            this->status = enum_AtomicTaskStatus::running;
            
            break;
            
        case enum_AtomicTaskStatus::running:
        {
            s_pose p;
            this->monitor->getPosition(p);
            if(p.x == this->endPosition.x && p.y== this->endPosition.y && p.roll == this->endPosition.roll)
            {
                std::cout << "Arrived at the destination!"<< std::endl;
                this->status = enum_AtomicTaskStatus::completed;
            } else
            {
                p.x = this->endPosition.x;
                p.y = this->endPosition.y;
                p.roll = this->endPosition.roll;
                this->monitor->setPosition(p);
                std::cout << "COST: " << this->cost << std::endl;
                this->monitor->consumeBattery(this->cost);
            }
        }
            break;
        case enum_AtomicTaskStatus::completed:
            break;
    }
}

void GoTo::calculateCost()
{
    this->cost = sqrtf(pow(this->endPosition.x - this->startPosition.x, 2) + pow(this->endPosition.y - this->startPosition.y, 2)) * this->costMeter;
}


