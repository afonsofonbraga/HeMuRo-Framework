//
//  NavigateMavROS.cpp
//  MRSMac
//
//  Created by Afonso Braga on 01/10/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "NavigateMavROS.hpp"
#include <map>

NavigateMavROS::NavigateMavROS(Blackboard* vMonitor, s_pose& start, s_pose& end) : AtomicTask(vMonitor, start, end)
{
    this->costFactor = factor * (battery_discharge /(robots_max_speed*3600))/battery_capacity;
    this->timeFactor = this->robots_max_speed;
    
    calculateCost();
    calculateTime();
}

NavigateMavROS::~NavigateMavROS(){}

void NavigateMavROS::run()
{
    switch(this->status)
    {
        case enum_AtomicTaskStatus::null:
            break;
            
        case enum_AtomicTaskStatus::waiting:
        {
            this->monitor->print("Going to the location -> x: " + std::to_string(this->endPosition.x) + " Y: " + std::to_string(this->endPosition.y));
            this->status = enum_AtomicTaskStatus::running;
            
            t0 = std::chrono::system_clock::now();
            break;
        }
            
        case enum_AtomicTaskStatus::running:
        {
            std::this_thread::sleep_until(t0 + this->tick);
            
            if(this->status == enum_AtomicTaskStatus::running){
                s_pose p;
                this->monitor->getPosition(p);
                s_pose deltaError;
                
                deltaError.x = this->endPosition.x - p.x;
                deltaError.y = this->endPosition.y - p.y;
                
                if(sqrt(pow(deltaError.x, 2) + pow(deltaError.y, 2)) <= 0.1)
                {
                    this->status = enum_AtomicTaskStatus::completed;
                }else
                {
                    s_ROSModuleMessage teste;
                    strcpy(teste.topicName,"Navigate");
                    memmove(teste.buffer,(char*)&this->endPosition,sizeof(s_pose));
                    this->monitor->addROSModuleMessage(teste);
                }
            }
            
            t0 = t0 + this->tick;
        }
            break;
        case enum_AtomicTaskStatus::completed:
            this->monitor->print("Arrived at the destination!");
            break;
        default:
            break;
    }
}

void NavigateMavROS::stop()
{
    this->status = enum_AtomicTaskStatus::null;
    s_ROSModuleMessage teste;
    strcpy(teste.topicName,"Move_base/Cancel");
    this->monitor->addROSModuleMessage(teste);
}

void NavigateMavROS::calculateCost()
{
    s_ROSModuleMessage teste;
    strcpy(teste.topicName,"Move_base/Cost");
    memmove(teste.buffer,(char*)&this->endPosition,sizeof(s_pose));
    this->cost = sqrtf(pow(this->endPosition.x - this->startPosition.x, 2) + pow(this->endPosition.y - this->startPosition.y, 2) + pow(this->endPosition.z - this->startPosition.z, 2)) * this->costFactor;
}

void NavigateMavROS::calculateTime()
{
    int time_seconds = round(sqrtf(pow(this->endPosition.x - this->startPosition.x, 2) + pow(this->endPosition.y - this->startPosition.y, 2) + pow(this->endPosition.z - this->startPosition.z, 2))/this->timeFactor);
    this->time = std::chrono::milliseconds(time_seconds*1000);
}
