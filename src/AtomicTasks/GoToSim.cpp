//
//  GoToSim.cpp
//  MRSMac
//
//  Created by Afonso Braga on 06/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "GoToSim.hpp"

GoToSim::GoToSim(Blackboard* vMonitor, s_pose& start, s_pose& end) : AtomicTask(vMonitor, start, end)
{
    costFactor = factor * (battery_discharge /(robots_max_speed*3600))/battery_capacity;
    this->timeFactor = this->robots_max_speed;
    
    calculateCost();
    calculateTime();
}

GoToSim::~GoToSim(){}

void GoToSim::run()
{
    switch(this->status)
    {
        case enum_AtomicTaskStatus::null:
            break;
            
        case enum_AtomicTaskStatus::waiting:
            this->monitor->print("Going to the location -> x: " + std::to_string(this->endPosition.x) + " Y: " + std::to_string(this->endPosition.y));
            //std::cout << "Going to the location -> x: " << this->endPosition.x << " Y: " << this->endPosition.y << std::endl;
            this->status = enum_AtomicTaskStatus::running;
            
            break;
            
        case enum_AtomicTaskStatus::running:
        {
            s_pose p;
            this->monitor->getPosition(p);
            if(p.x == this->endPosition.x && p.y== this->endPosition.y && p.roll == this->endPosition.roll)
            {
                this->monitor->print("Arrived at the destination!");
                //std::cout << "Arrived at the destination!"<< std::endl;
                this->status = enum_AtomicTaskStatus::completed;
            } else
            {
                p.x = this->endPosition.x;
                p.y = this->endPosition.y;
                p.roll = this->endPosition.roll;
                this->monitor->setPosition(p);
                this->monitor->print("COST: " + std::to_string(this->cost));
                //std::cout << "COST: " << this->cost << std::endl;
                this->monitor->consumeBattery(this->cost);
                usleep(1000000);
            }
        }
            break;
        case enum_AtomicTaskStatus::completed:
            break;
            
        default:
            break;
    }
}

void GoToSim::calculateCost()
{
    this->cost = sqrtf(pow(this->endPosition.x - this->startPosition.x, 2) + pow(this->endPosition.y - this->startPosition.y, 2)) * this->costFactor;
}

void GoToSim::calculateTime()
{
    int time_seconds = round(sqrtf(pow(this->endPosition.x - this->startPosition.x, 2) + pow(this->endPosition.y - this->startPosition.y, 2) + pow(this->endPosition.z - this->startPosition.z, 2))/this->timeFactor);
    this->time = std::chrono::milliseconds(time_seconds*1000);
}
