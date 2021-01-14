//
//  Agent.cpp
//  MRSFramework
//
//  Created by Afonso Braga on 13/01/21.
//  Copyright © 2021 Afonso Braga. All rights reserved.
//

#include "Agent.hpp"

Agent::Agent(BlackBoard* monitor)
{
    this->monitor = monitor;
}

Agent::~Agent()
{
    
}

bool Agent::addAtomicTask(MissionExecution& vMissionDecomposable)
{
    return false;
}


void Agent::decomposableTaskList()
{

}
