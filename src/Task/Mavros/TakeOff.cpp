//
//  TakeOff.cpp
//  MRSFramework
//
//  Created by Afonso Braga on 25/08/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "TakeOff.hpp"

TakeOff::TakeOff(BlackBoard* vMonitor, s_pose& start, s_pose& end) : AtomicTask(vMonitor, start, end)
{
    calculateCost();
}

TakeOff::~TakeOff(){}

void TakeOff::run()
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
                
                if(sqrt(pow(deltaError.z, 2)) <= 0.1)
                {
                    this->status = enum_AtomicTaskStatus::completed;
                } else
                {
                    s_ROSBridgeMessage teste;
                    strcpy(teste.topicName,"TakeOff");
                    memmove(teste.buffer,(char*)&this->endPosition,sizeof(this->endPosition));
                    this->monitor->addROSBridgeMessage(teste);
                    this->status = enum_AtomicTaskStatus::completed;
                }
                
            }
            break;
        case enum_AtomicTaskStatus::completed:
            usleep(1000000); //Vamos Precisar de um buffer
            std::cout << "Arrived at the destination!"<< std::endl;
            break;
        }
    }
}
void TakeOff::calculateCost()
{
    this->cost = sqrtf(pow(this->endPosition.x - this->startPosition.x, 2) + pow(this->endPosition.y - this->startPosition.y, 2) + pow(this->endPosition.z - this->startPosition.z, 2)) * this->costMeter;
}


