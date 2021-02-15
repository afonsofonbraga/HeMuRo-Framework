//
//  AtomicTask.cpp
//  MRSMac
//
//  Created by Afonso Braga on 06/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "AtomicTask.hpp"

AtomicTask::AtomicTask(Blackboard* vMonitor, s_pose& start, s_pose& end)
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

AtomicTask::AtomicTask(Blackboard* vMonitor, s_pose& end)
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

void AtomicTask::calculateCost() { this->cost = this->costFactor; }

float AtomicTask::getCost(){ return this->cost; }

void AtomicTask::setCostFactor(float value) { this->costFactor = value; }

enum_AtomicTaskStatus AtomicTask::getStatus()
{
    return this->status;
}

void AtomicTask::calculateTime() {this->time = std::chrono::milliseconds(this->timeFactor);}

void AtomicTask::setTimeFactor(int factor) {this->timeFactor = factor; }

std::chrono::milliseconds AtomicTask::getTime() { return this->time; }


