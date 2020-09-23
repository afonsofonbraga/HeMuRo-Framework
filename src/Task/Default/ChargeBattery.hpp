//
//  chargeBattery.hpp
//  MRSMac
//
//  Created by Afonso Braga on 15/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef ChargeBattery_hpp
#define ChargeBattery_hpp

#include "AtomicTask.hpp"
#include <cmath>

class ChargeBattery: public AtomicTask
{
protected:
    float costMeter = 2.0;
    std::chrono::milliseconds tick = std::chrono::milliseconds(1000); // 0.1s
    std::chrono::system_clock::time_point t0;
public:
    ChargeBattery(BlackBoard* vMonitor, s_pose& start, s_pose& end);
    ~ChargeBattery();
    void run() override;
    void calculateCost() override;
    
};
#endif /* chargeBattery_hpp */
