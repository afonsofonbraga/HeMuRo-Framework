//
//  ArmMavROS.cpp
//  MRSFramework
//
//  Created by Afonso Braga on 25/08/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "ArmMavROS.hpp"


ArmMavROS::ArmMavROS(Blackboard* vMonitor, s_pose& start, s_pose& end) : AtomicTask(vMonitor, start, end)
{
    calculateCost();
}

ArmMavROS::~ArmMavROS() {}

void ArmMavROS::run()
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
            s_ROSModuleMessage teste;
            strcpy(teste.topicName,"Arm");
            this->monitor->addROSModuleMessage(teste);
            usleep(1000000); //Vamos Precisar de um buffer
            this->status = enum_AtomicTaskStatus::completed;
            break;
        }
            
        case enum_AtomicTaskStatus::completed:
            break;
        default:
            break;
    }
}

void ArmMavROS::calculateCost()
{
    this->cost = this->costMeter;
}

