//
//  TakeOff.hpp
//  MRSFramework
//
//  Created by Afonso Braga on 25/08/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef TakeOff_hpp
#define TakeOff_hpp

#include <stdio.h>
#include "AtomicTask.hpp"

class TakeOff: public AtomicTask
{
protected:
    float costMeter = 1.0;
public:
    TakeOff(BlackBoard* vMonitor, s_pose& start, s_pose& end);
    ~TakeOff();
    void run() override;
    void calculateCost() override;
};

#endif /* TakePicture_hpp */
