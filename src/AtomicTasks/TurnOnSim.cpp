//
//  TurnOnSim.cpp
//  MRSMac
//
//  Created by Afonso Braga on 15/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "TurnOnSim.hpp"


TurnOnSim::TurnOnSim(Blackboard* vMonitor,s_pose& start, s_pose& end) : AtomicTask(vMonitor, start, end)
{
    costFactor = 1.0;
    calculateCost();
}

TurnOnSim::~TurnOnSim(){}

void TurnOnSim::run()
{
    this->status = enum_AtomicTaskStatus::running;
    this->monitor->print("Turning on");
    //std::cout << "Ligando."<< std::endl;
    this->monitor->consumeBattery(this->cost);
    this->status = enum_AtomicTaskStatus::completed;
}

void TurnOnSim::calculateCost()
{
    this->cost = this->costFactor;
}

