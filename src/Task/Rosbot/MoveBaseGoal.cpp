//
//  MoveBaseGoal.cpp
//  MRSMac
//
//  Created by Afonso Braga on 01/10/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "MoveBaseGoal.hpp"
#include <map>

MoveBaseGoal::MoveBaseGoal(BlackBoard* vMonitor, s_pose& start, s_pose& end) : AtomicTask(vMonitor, start, end)
{
    calculateCost();
}

MoveBaseGoal::~MoveBaseGoal(){}

void MoveBaseGoal::run()
{
    switch(this->status)
    {
        case enum_AtomicTaskStatus::null:
            break;
            
        case enum_AtomicTaskStatus::waiting:
        {
            this->monitor->print("Going to the location -> x: " + std::to_string(this->endPosition.x) + " Y: " + std::to_string(this->endPosition.y));
            this->status = enum_AtomicTaskStatus::running;
            
            s_ROSBridgeMessage teste;
            strcpy(teste.topicName,"Move_base/Goal");
            memmove(teste.buffer,(char*)&this->endPosition,sizeof(s_pose));
            this->monitor->addROSBridgeMessage(teste);
            
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
                deltaError.yaw = adjustAngle(this->endPosition.yaw - p.yaw);
                
                if(sqrt(pow(deltaError.x, 2) + pow(deltaError.y, 2)) <= 0.1)
                {
                    s_ROSBridgeMessage teste;
                    strcpy(teste.topicName,"Move_base/Cancel");
                    this->monitor->addROSBridgeMessage(teste);
                    this->status = enum_AtomicTaskStatus::completed;
                }
            }
            
            t0 = t0 + this->tick;
        }
            break;
        case enum_AtomicTaskStatus::completed:
            this->monitor->print("Arrived at the destination!");
            break;
    }
}

void MoveBaseGoal::calculateCost()
{
    this->cost = sqrtf(pow(this->endPosition.x - this->startPosition.x, 2) + pow(this->endPosition.y - this->startPosition.y, 2)) * this->costMeter;
}

float MoveBaseGoal::adjustAngle(float angle)
{
    angle = fmod(angle,2*M_PI);
    if (angle > M_PI)
        angle -= 2*M_PI;
    return angle;
}
