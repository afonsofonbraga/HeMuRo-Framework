//
//  TakeOffMavROS.cpp
//  MRSFramework
//
//  Created by Afonso Braga on 25/08/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "TakeOffMavROS.hpp"

TakeOffMavROS::TakeOffMavROS(Blackboard* vMonitor, s_pose& start, s_pose& end) : AtomicTask(vMonitor, start, end)
{
    calculateCost();
    
    this->timeFactor = 10000;
    calculateTime();
    
}

TakeOffMavROS::~TakeOffMavROS(){}

void TakeOffMavROS::run()
{
    
    switch(this->status)
    {
        case enum_AtomicTaskStatus::null:
            break;
            
        case enum_AtomicTaskStatus::waiting:
        {
            std::cout << "Taking Off."<< std::endl;
            this->status = enum_AtomicTaskStatus::running;
            break;
        }
            
        case enum_AtomicTaskStatus::running:
        {
            if(this->status != enum_AtomicTaskStatus::completed){
                s_pose p;
                this->monitor->getPosition(p);
                s_pose deltaError;
                
                deltaError.z = this->endPosition.z - p.z;
                
                if(sqrt(pow(deltaError.z, 2)) <= 0.2)
                {
                    this->status = enum_AtomicTaskStatus::completed;
                } else
                {
                    s_ROSModuleMessage teste;
                    strcpy(teste.topicName,"TakeOff");
                    memmove(teste.buffer,(char*)&this->endPosition,sizeof(this->endPosition));
                    this->monitor->addROSModuleMessage(teste);
                    
                    auto t0 = std::chrono::system_clock::now();
                    std::this_thread::sleep_until(t0 + this->tick);
                }
                
            }
            break;
        case enum_AtomicTaskStatus::completed:
            std::cout << "Took off!"<< std::endl;
            break;
        default:
            break;
        }
    }
}
void TakeOffMavROS::calculateCost()
{
    this->cost = sqrtf(pow(this->endPosition.x - this->startPosition.x, 2) + pow(this->endPosition.y - this->startPosition.y, 2) + pow(this->endPosition.z - this->startPosition.z, 2)) * this->costMeter;
}


void TakeOffMavROS::calculateTime()
{
    this->time = std::chrono::milliseconds(int(this->timeFactor));
    this->tick = std::chrono::milliseconds(int(this->timeFactor));
}
