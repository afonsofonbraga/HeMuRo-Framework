//
//  TakePicture.hpp
//  MRSMac
//
//  Created by Afonso Braga on 27/07/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef TakePicture_hpp
#define TakePicture_hpp

#include <stdio.h>
#include "AtomicTask.hpp"

class TakePicture: public AtomicTask
{
protected:
    float costMeter = 1.0;
public:
    TakePicture(BlackBoard* vMonitor, s_pose& start, s_pose& end);
    ~TakePicture();
    void run() override;
    void calculateCost() override;
};

#endif /* TakePicture_hpp */
