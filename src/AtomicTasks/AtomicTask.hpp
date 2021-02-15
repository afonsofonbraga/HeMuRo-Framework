//
//  AtomicTask.hpp
//  MRSMac
//
//  Created by Afonso Braga on 06/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef AtomicTask_hpp
#define AtomicTask_hpp

#include <iostream>
#include <chrono>
#include "dataTypes.hpp"
#include "Blackboard.hpp"


class AtomicTask
{
protected:
    s_pose startPosition;
    s_pose endPosition;
    float cost;
    float costFactor = 0;
    std::chrono::milliseconds time = std::chrono::milliseconds(0); //Time to execute the atomicTask
    int timeFactor = 0;
    enum_AtomicTaskStatus status;
    Blackboard* monitor;
public:
    AtomicTask(Blackboard* vMonitor, s_pose& start, s_pose& end);
    AtomicTask(Blackboard* vMonitor, s_pose& end);
    AtomicTask();
    ~AtomicTask();
    
    virtual void run();
    virtual void stop();
    enum_AtomicTaskStatus getStatus();
    virtual void calculateCost();
    void setCostFactor(float value);
    float getCost();
    virtual void calculateTime();
    void setTimeFactor(int factor);
    std::chrono::milliseconds getTime();
};
#endif /* AtomicTask_hpp */
