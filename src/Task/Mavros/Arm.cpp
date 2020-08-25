//
//  Arm.cpp
//  MRSFramework
//
//  Created by Afonso Braga on 25/08/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "Arm.hpp"


Arm::Arm(BlackBoard* vMonitor, s_pose& start, s_pose& end) : AtomicTask(vMonitor, start, end)
{
    calculateCost();
}

Arm::~Arm() {}

void Arm::run()
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
            std::cout << "Arming."<< std::endl;
            s_ROSBridgeMessage teste;
            strcpy(teste.topicName,"Arm");
            this->monitor->addROSBridgeMessage(teste);
            this->status = enum_AtomicTaskStatus::completed;
            break;
        }
            
        case enum_AtomicTaskStatus::completed:
            break;
    }
}

void Arm::calculateCost()
{
    this->cost = this->costMeter;
}

