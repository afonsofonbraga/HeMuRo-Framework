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
    std::cout << " Ainda tenho que colocar como ele vai andar ou acessar valores la de cima."<< std::endl;
}

void TurnOn::calculateCost()
{
    this->cost = 3;
}

