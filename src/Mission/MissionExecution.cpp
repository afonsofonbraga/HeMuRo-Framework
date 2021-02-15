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
    if(this->atomicTaskIndex == this->atomicTaskSequence.size() && this->atomicTaskSequence.size() != 0)
    {
        this->enum_execution = enum_MissionExecution::missionComplete;
        //std::cout <<"Mission Complete!"<< std::endl;
    }
    
    if(this->enum_execution == enum_MissionExecution::executing)
    {
        switch(this->atomicTaskSequence.at(atomicTaskIndex)->getStatus())
        {
            case enum_AtomicTaskStatus::null:
                break;
            case enum_AtomicTaskStatus::waiting:
               this->atomicTaskSequence.at(atomicTaskIndex)->run();
                break;
            case enum_AtomicTaskStatus::running:
               this->atomicTaskSequence.at(atomicTaskIndex)->run();
                break;
            case enum_AtomicTaskStatus::completed:
                atomicTaskIndex++;
                break;
            default:
                break;
        }
    }
}

void MissionExecution::stop()
{
    this->atomicTaskSequence.at(atomicTaskIndex)->stop();
}

void MissionExecution::clear()
{
    this->atomicTaskEnumerator.clear();
    this->atomicTaskSequence.clear();
    missionCost = 0;
    enum_execution = enum_MissionExecution::null;
    atomicTaskIndex = 0;
    
    mission = enum_DecomposableTask::null;
    robotCategory = enum_RobotCategory::null;
    relativeDeadline = std::chrono::milliseconds(0);
    numberOfAttributes = 0;
}
