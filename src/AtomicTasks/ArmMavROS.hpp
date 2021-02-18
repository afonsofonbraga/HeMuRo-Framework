//
//  ArmMavROS.hpp
//  MRSFramework
//
//  Created by Afonso Braga on 25/08/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef ArmMavROS_hpp
#define ArmMavROS_hpp

#include <stdio.h>
#include "AtomicTask.hpp"
#include <unistd.h>

class ArmMavROS: public AtomicTask
{
protected:
    float costMeter = 1.0;
public:
    ArmMavROS(Blackboard* vMonitor, s_pose& start, s_pose& end);
    ~ArmMavROS();
    void run() override;
    void calculateCost() override;
    void calculateTime() override;
};

#endif /* TakePicture_hpp */
