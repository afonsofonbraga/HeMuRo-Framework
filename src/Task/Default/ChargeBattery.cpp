//
//  chargeBattery.cpp
//  MRSMac
//
//  Created by Afonso Braga on 15/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "ChargeBattery.hpp"



ChargeBattery::ChargeBattery(s_pose& start, s_pose& end) : AtomicTask(start, end)
{
    calculateCost();
}

ChargeBattery::~ChargeBattery(){}

void ChargeBattery::run()
{
    this->status = enum_AtomicTaskStatus::running;
    std::cout << " Carregando Bateria."<< std::endl;
    this->status = enum_AtomicTaskStatus::completed;
}

void ChargeBattery::calculateCost()
{
    this->cost = this->costMeter; //sqrtf(pow(this->endPosition.x - this->startPosition.x, 2) + pow(this->endPosition.y - this->startPosition.y, 2)) * this->costMeter;
}

