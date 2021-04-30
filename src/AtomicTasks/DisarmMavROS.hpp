//
//  DisarmMavROS.hpp
//  MRSMac
//
//  Created by Afonso Braga on 30/04/21.
//  Copyright © 2021 Afonso Braga. All rights reserved.
//

#ifndef DisarmMavROS_hpp
#define DisarmMavROS_hpp

#include <stdio.h>

#include <stdio.h>
#include "AtomicTask.hpp"
#include <unistd.h>

class DisarmMavROS: public AtomicTask
{
protected:
    float costMeter = 1.0;
public:
    DisarmMavROS(Blackboard* vMonitor, s_pose& start, s_pose& end);
    ~DisarmMavROS();
    void run() override;
    void calculateCost() override;
    void calculateTime() override;
};

#endif /* DisarmMavROS_hpp */
