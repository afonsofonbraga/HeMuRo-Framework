//
//  LandMavROS.hpp
//  MRSFramework
//
//  Created by Afonso Braga on 25/08/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef LandMavROS_hpp
#define LandMavROS_hpp

#include <stdio.h>
#include "AtomicTask.hpp"

class LandMavROS: public AtomicTask
{
protected:
    float costMeter = 1.0;
public:
    LandMavROS(Blackboard* vMonitor, s_pose& start, s_pose& end);
    ~LandMavROS();
    void run() override;
    void calculateCost() override;
    void calculateTime() override;
};

#endif /* LandMavROS_hpp */
