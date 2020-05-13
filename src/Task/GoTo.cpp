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
    std::cout << " Ainda tenho que colocar como ele vai andar ou acessar valores la de cima."<< std::endl;
}

void GoTo::calculateCost()
{
    this->cost = sqrtf(pow(this->endPosition.x - this->startPosition.x, 2) + pow(this->endPosition.y - this->startPosition.y, 2)) * this->costMeter;
}

