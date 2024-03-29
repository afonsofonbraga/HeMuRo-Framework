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
    std::vector<enum_AtomicTask> atomicTaskEnumerator;
    std::vector<std::shared_ptr<AtomicTask>> atomicTaskSequence;
    int atomicTaskIndex = 0;
    float missionCost = 0;
    std::chrono::milliseconds timeToExecute = std::chrono::milliseconds(0);
    std::chrono::time_point<std::chrono::system_clock> startTime;
    //auto now = std::chrono::system_clock::now();
public:
    MissionExecution();
    ~MissionExecution();
    void run();
    void stop();
    void clear();
};
#endif /* MissionExecution_hpp */

