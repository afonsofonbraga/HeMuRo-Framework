//
//  turnOn.cpp
//  MRSMac
//
//  Created by Afonso Braga on 15/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "TurnOn.hpp"


TurnOn::TurnOn(s_pose& start, s_pose& end) : AtomicTask(start, end)
{
    calculateCost();
}

TurnOn::~TurnOn(){}

void TurnOn::run()
{
    this->status = enum_AtomicTaskStatus::running;
    std::cout << " Ligando."<< std::endl;
    this->status = enum_AtomicTaskStatus::completed;
}

void TurnOn::calculateCost()
{
    this->cost = 3;
}

