//
//  AtomicTask.cpp
//  MRSMac
//
//  Created by Afonso Braga on 06/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "AtomicTask.hpp"

AtomicTask::AtomicTask(BlackBoard* vMonitor, s_pose& start, s_pose& end)
{
    this->monitor = vMonitor;
    this->status = enum_AtomicTaskStatus::waiting;
    this->startPosition = start;
    this->endPosition = end;
    this->calculateCost();
}

AtomicTask::AtomicTask()
{
    
}

AtomicTask::AtomicTask(BlackBoard* vMonitor, s_pose& end)
{
    this->monitor = vMonitor;
    this->status = enum_AtomicTaskStatus::waiting;
    this->startPosition = end;
    this->endPosition = end;
    this->calculateCost();
}

AtomicTask::~AtomicTask()
{
    
}

void AtomicTask::run() {
    this->status = enum_AtomicTaskStatus::running;
    std::cout << "Default"<< std::endl;
    this->status = enum_AtomicTaskStatus::completed;
}

void AtomicTask::stop()
{
    this->status = enum_AtomicTaskStatus::null;
}

void AtomicTask::calculateCost() { this->cost = 0; }

float AtomicTask::getCost(){ return this->cost; }

enum_AtomicTaskStatus AtomicTask::getStatus()
{
    return this->status;
}
