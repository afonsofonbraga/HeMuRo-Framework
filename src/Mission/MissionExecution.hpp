//
//  MissionExecution.hpp
//  MRSMac
//
//  Created by Afonso Braga on 30/06/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef MissionExecution_hpp
#define MissionExecution_hpp

#include <vector>
#include <thread>
#include <chrono>
#include <condition_variable>

#include "AtomicTask.hpp"
#include "Mission.hpp"
#include "dataTypes.hpp"

class MissionExecution: public Mission{
public:
    enum_MissionExecution enum_execution = enum_MissionExecution::null;
    //char winnerAddress[15];
    //bool missionAccepted = false;
    std::vector<AtomicTask*> atomicTaskList;
    int atomicTaskIndex = 0;
    float missionCost = 0;
public:
    MissionExecution();
    ~MissionExecution();
    AtomicTask* selectNextAction();
};
#endif /* MissionExecution_hpp */