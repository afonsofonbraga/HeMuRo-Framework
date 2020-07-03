//
//  AtomicTask.cpp
//  MRSMac
//
//  Created by Afonso Braga on 06/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "AtomicTask.hpp"

AtomicTask::AtomicTask(s_pose& start, s_pose& end)
{
    this->status = enum_AtomicTaskStatus::waiting;
    this->startPosition = start;
    this->endPosition = end;
    this->calculateCost();
}

AtomicTask::AtomicTask()
{
    
}

AtomicTask::AtomicTask(s_pose& end)
{
    this->status = enum_AtomicTaskStatus::waiting;
    this->startPosition = end;
    this->endPosition = end;
    this->calculateCost();
}

AtomicTask::~AtomicTask()
{
    
}

void AtomicTask::run() {}

void AtomicTask::calculateCost() { this->cost = 0; }

float AtomicTask::getCost(){ return this->cost; }

enum_AtomicTaskStatus AtomicTask::getStatus()
{
    return this->status;
}
