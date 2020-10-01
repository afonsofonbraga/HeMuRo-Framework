//
//  MoveBaseGoal.hpp
//  MRSMac
//
//  Created by Afonso Braga on 01/10/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef MoveBaseGoal_hpp
#define MoveBaseGoal_hpp


#include <stdio.h>

#include "AtomicTask.hpp"
#include <chrono>
#include <thread>
#include <math.h>
#include <unistd.h>

class MoveBaseGoal : public AtomicTask
{
protected:
    std::chrono::system_clock::time_point t0;
    std::chrono::milliseconds tick = std::chrono::milliseconds(1000); // 1s
    float costMeter = 2.0;
    
    float kp = 3, ka = 8, kb = -0.5;
    float alpha_t = 0;
    float v = 0;
    
    float adjustAngle(float angle);
    
public:
    MoveBaseGoal(BlackBoard* vMonitor, s_pose& start, s_pose& end);
    ~MoveBaseGoal();
    void run() override;
    void calculateCost() override;

};
#endif /* MoveBaseGoal_hpp */
