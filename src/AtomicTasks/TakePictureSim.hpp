//
//  TakePictureSim.hpp
//  MRSMac
//
//  Created by Afonso Braga on 27/07/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef TakePictureSim_hpp
#define TakePictureSim_hpp

#include <stdio.h>
#include "AtomicTask.hpp"

class TakePictureSim: public AtomicTask
{
protected:
public:
    TakePictureSim(Blackboard* vMonitor, s_pose& start, s_pose& end);
    ~TakePictureSim();
    void run() override;
    void calculateCost() override;
    void calculateTime() override;
};

#endif /* TakePictureSim_hpp */
