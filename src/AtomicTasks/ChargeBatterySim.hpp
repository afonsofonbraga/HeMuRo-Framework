//
//  ChargeBatterySim.hpp
//  MRSMac
//
//  Created by Afonso Braga on 15/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef ChargeBatterySim_hpp
#define ChargeBatterySim_hpp

#include "AtomicTask.hpp"
#include <cmath>

class ChargeBatterySim: public AtomicTask
{
protected:
    
    std::chrono::milliseconds tick = std::chrono::milliseconds(100); // 0.1s
    std::chrono::system_clock::time_point t0;
public:
    ChargeBatterySim(BlackBoard* vMonitor, s_pose& start, s_pose& end);
    ~ChargeBatterySim();
    void run() override;
    void calculateCost() override;
    
};
#endif /* chargeBattery_hpp */
