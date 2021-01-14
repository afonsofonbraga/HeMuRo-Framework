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
#include "dataTypes.hpp"
#include "BlackBoard.hpp"

class AtomicTask
{
protected:
    
    s_pose startPosition;
    s_pose endPosition;
    float cost;
    float costFactor;
    enum_AtomicTaskStatus status;
    BlackBoard* monitor;
public:
    AtomicTask(BlackBoard* vMonitor, s_pose& start, s_pose& end);
    AtomicTask(BlackBoard* vMonitor, s_pose& end);
    AtomicTask();
    ~AtomicTask();
    
    virtual void run();
    virtual void stop();
    enum_AtomicTaskStatus getStatus();
    virtual void calculateCost();
    void setCostFactor(float value);
    float getCost();
};
#endif /* AtomicTask_hpp */
