//
//  MeasureTemperatureSim.hpp
//  MRSMac
//
//  Created by Afonso Braga on 10/03/21.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef MeasureTemperatureSim_hpp
#define MeasureTemperatureSim_hpp

#include <stdio.h>
#include "AtomicTask.hpp"

class MeasureTemperatureSim: public AtomicTask
{
protected:
public:
    MeasureTemperatureSim(Blackboard* vMonitor, s_pose& start, s_pose& end);
    ~MeasureTemperatureSim();
    void run() override;
    void calculateCost() override;
    void calculateTime() override;
};

#endif /* MeasureTemperatureSim_hpp */
