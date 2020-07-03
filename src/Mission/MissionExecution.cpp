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

AtomicTask* MissionExecution::selectNextAction()
{
    if(this->atomicTaskIndex == this->atomicTaskList.size() && this->atomicTaskList.size() != 0)
    {
        this->enum_execution = enum_MissionExecution::missionComplete;
        std::cout <<"Mission Complete!"<< std::endl;
        return nullptr;
    }
    if(this->enum_execution == enum_MissionExecution::executing)
    {
        this->atomicTaskIndex++;
        return this->atomicTaskList.at(this->atomicTaskIndex-1);
    }
    return nullptr;
}
