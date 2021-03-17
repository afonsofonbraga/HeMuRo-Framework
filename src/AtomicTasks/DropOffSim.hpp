//
//  DropOffSim.hpp
//  MRSMac
//
//  Created by Afonso Braga on 10/03/21.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef DropOffSim_hpp
#define DropOffSim_hpp

#include <stdio.h>
#include "AtomicTask.hpp"

class DropOffSim: public AtomicTask
{
protected:
public:
    DropOffSim(Blackboard* vMonitor, s_pose& start, s_pose& end);
    ~DropOffSim();
    void run() override;
    void calculateCost() override;
    void calculateTime() override;
};

#endif /* DropOffSim_hpp */
