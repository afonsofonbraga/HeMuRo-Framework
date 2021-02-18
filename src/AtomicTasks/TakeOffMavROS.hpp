//
//  TakeOffMavROS.hpp
//  MRSFramework
//
//  Created by Afonso Braga on 25/08/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef TakeOffMavROS_hpp
#define TakeOffMavROS_hpp

#include <stdio.h>
#include "AtomicTask.hpp"
#include <math.h>
#include <unistd.h>

class TakeOffMavROS: public AtomicTask
{
protected:
    float costMeter = 1.0;
    std::chrono::milliseconds tick = std::chrono::seconds(10); // 1s
public:
    TakeOffMavROS(Blackboard* vMonitor, s_pose& start, s_pose& end);
    ~TakeOffMavROS();
    void run() override;
    void calculateCost() override;
    void calculateTime() override;
};

#endif /* TakeOffMavROS_hpp */
