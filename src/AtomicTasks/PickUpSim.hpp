//
//  PickUpSim.hpp
//  MRSMac
//
//  Created by Afonso Braga on 10/03/21.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef PickUpSim_hpp
#define PickUpSim_hpp

#include <stdio.h>
#include "AtomicTask.hpp"

class PickUpSim: public AtomicTask
{
protected:
public:
    PickUpSim(Blackboard* vMonitor, s_pose& start, s_pose& end);
    ~PickUpSim();
    void run() override;
    void calculateCost() override;
    void calculateTime() override;
};

#endif /* PickUpSim_hpp */
