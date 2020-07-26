//
//  chargeBattery.cpp
//  MRSMac
//
//  Created by Afonso Braga on 15/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "ChargeBattery.hpp"



ChargeBattery::ChargeBattery(BlackBoard* vMonitor, s_pose& start, s_pose& end) : AtomicTask(vMonitor, start, end)
{
    calculateCost();
}

ChargeBattery::~ChargeBattery(){}

void ChargeBattery::run()
{
    switch(this->status)
    {
        case enum_AtomicTaskStatus::null:
            break;
            
        case enum_AtomicTaskStatus::waiting:
            std::cout << "Carregando Bateria."<< std::endl;
            this->status = enum_AtomicTaskStatus::running;
            break;
            
        case enum_AtomicTaskStatus::running:
            if(this->monitor->getBatteryLevel() < 100)
            {
                this->monitor->chargeBattery(1.0);
            } else
            {
                std::cout << "Bateria Carregada!"<< std::endl;
                this->status = enum_AtomicTaskStatus::completed;
            }
            break;
        case enum_AtomicTaskStatus::completed:
            break;
    }
}

void ChargeBattery::calculateCost()
{
    this->cost = this->costMeter;
}

