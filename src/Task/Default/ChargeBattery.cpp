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
    std::cout << " Ainda tenho que colocar como ele vai andar ou acessar valores la de cima."<< std::endl;
}

void ChargeBattery::calculateCost()
{
    this->cost = sqrtf(pow(this->endPosition.x - this->startPosition.x, 2) + pow(this->endPosition.y - this->startPosition.y, 2)) * this->costMeter;
}

