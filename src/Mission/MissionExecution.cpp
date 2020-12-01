//
//  MissionExecution.cpp
//  MRSMac
//
//  Created by Afonso Braga on 30/06/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#include "MissionExecution.hpp"

MissionExecution::MissionExecution()
{
    
};

MissionExecution::~MissionExecution()
{
    
};

void MissionExecution::run()
{
    if(this->atomicTaskIndex == this->atomicTaskList.size() && this->atomicTaskList.size() != 0)
    {
        this->enum_execution = enum_MissionExecution::missionComplete;
        //std::cout <<"Mission Complete!"<< std::endl;
    }
    
    if(this->enum_execution == enum_MissionExecution::executing)
    {
        switch(this->atomicTaskList.at(atomicTaskIndex)->getStatus())
        {
            case enum_AtomicTaskStatus::null:
                break;
            case enum_AtomicTaskStatus::waiting:
               this->atomicTaskList.at(atomicTaskIndex)->run();
                break;
            case enum_AtomicTaskStatus::running:
               this->atomicTaskList.at(atomicTaskIndex)->run();
                break;
            case enum_AtomicTaskStatus::completed:
                atomicTaskIndex++;
                break;
        }
    }
}

void MissionExecution::stop()
{
    this->atomicTaskList.at(atomicTaskIndex)->stop();
}

void MissionExecution::clear()
{
    this->vAtomicTaskVector.clear();
    this->atomicTaskList.clear();
    missionCost = 0;
    enum_execution = enum_MissionExecution::null;
    atomicTaskIndex = 0;
    
    mission = enum_DecomposableTask::null;
    robotCategory = enum_RobotCategory::null;
    executionTime = 0;
    numberOfAttributes = 0;
}
