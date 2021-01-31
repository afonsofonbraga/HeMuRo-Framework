//
//  ChargeBatteryROS.hpp
//  MRSMac
//
//  Created by Afonso Braga on 15/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef ChargeBatteryROS_hpp
#define ChargeBatteryROS_hpp

#include "AtomicTask.hpp"
#include <cmath>

class ChargeBatteryROS: public AtomicTask
{
protected:
    std::chrono::milliseconds tick = std::chrono::milliseconds(100); // 0.1s
    std::chrono::system_clock::time_point t0;
public:
    ChargeBatteryROS(Blackboard* vMonitor, s_pose& start, s_pose& end);
    ~ChargeBatteryROS();
    void run() override;
    void calculateCost() override;
    
};
#endif /* ChargeBatteryROS_hpp */
