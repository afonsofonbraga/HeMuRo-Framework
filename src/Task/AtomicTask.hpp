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

class AtomicTask
{
protected:
    
    s_pose startPosition;
    s_pose endPosition;
    float cost;
    enum_AtomicTaskStatus status;

public:
    AtomicTask(s_pose& start, s_pose& end);
    AtomicTask(s_pose& end);
    AtomicTask();
    ~AtomicTask();
    
    virtual void run();
    enum_AtomicTaskStatus getStatus();
    virtual void calculateCost();
    float getCost();
};
#endif /* AtomicTask_hpp */
