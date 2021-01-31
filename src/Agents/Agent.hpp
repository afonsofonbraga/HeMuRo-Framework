//
//  Agent.hpp
//  MRSFramework
//
//  Created by Afonso Braga on 13/01/21.
//  Copyright © 2021 Afonso Braga. All rights reserved.
//

#ifndef Agent_hpp
#define Agent_hpp

#include <stdio.h>
#include "Blackboard.hpp"
#include "dataTypes.hpp"
#include "MissionExecution.hpp"

#include "GoToSim.hpp"
#include "ChargeBatterySim.hpp"
#include "TurnOnSim.hpp"
#include "TakePictureSim.hpp"

class Agent
{
private:
protected:
    Blackboard* monitor;
public:
    Agent(Blackboard* monitor);
    ~Agent();
    virtual bool addAtomicTask(MissionExecution& vMissionDecomposable);
    virtual void decomposableTaskList();
};


#endif /* Agent_hpp */
