//
//  Arm.hpp
//  MRSFramework
//
//  Created by Afonso Braga on 25/08/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef Arm_hpp
#define Arm_hpp

#include <stdio.h>
#include "AtomicTask.hpp"

class Arm: public AtomicTask
{
protected:
    float costMeter = 1.0;
public:
    Arm(BlackBoard* vMonitor, s_pose& start, s_pose& end);
    ~Arm();
    void run() override;
    void calculateCost() override;
};

#endif /* TakePicture_hpp */
