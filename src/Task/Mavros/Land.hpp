//
//  Land.hpp
//  MRSFramework
//
//  Created by Afonso Braga on 25/08/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef Land_hpp
#define Land_hpp

#include <stdio.h>
#include "AtomicTask.hpp"

class Land: public AtomicTask
{
protected:
    float costMeter = 1.0;
public:
    Land(BlackBoard* vMonitor, s_pose& start, s_pose& end);
    ~Land();
    void run() override;
    void calculateCost() override;
};

#endif /* TakePicture_hpp */