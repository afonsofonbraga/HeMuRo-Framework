//
//  GoTo.cpp
//  MRSMac
//
//  Created by Afonso Braga on 06/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "GoTo.hpp"

GoTo::GoTo(s_pose& start, s_pose& end) : AtomicTask(start, end)
{
    calculateCost();
}

GoTo::~GoTo(){}

void GoTo::run()
{
    this->status = enum_AtomicTaskStatus::running;
    std::cout << " Indo para destino selecionado."<< std::endl;
    this->status = enum_AtomicTaskStatus::completed;
}

void GoTo::calculateCost()
{
    this->cost = sqrtf(pow(this->endPosition.x - this->startPosition.x, 2) + pow(this->endPosition.y - this->startPosition.y, 2)) * this->costMeter;
}

