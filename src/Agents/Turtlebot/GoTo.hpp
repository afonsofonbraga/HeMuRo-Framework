//
//  GoTo.hpp
//  MRSMac
//
//  Created by Afonso Braga on 06/05/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef GoTo_hpp
#define GoTo_hpp

#include "AtomicTask.hpp"
#include <chrono>
#include <thread>
#include <math.h>
#include <unistd.h>

class GoTo : public AtomicTask
{
protected:
    std::chrono::system_clock::time_point t0;
    std::chrono::milliseconds tick = std::chrono::milliseconds(100); // 0.1s
    float costMeter = 2.0;
    
    float kp = 1, ki = 0.05, kd = 0.5;
    float sum_Alpha_t = 0;
    float alpha_t_old = 0;
    float omega = 0;
    float alpha_t = 0;
    float v = 0;
    
public:
    GoTo(BlackBoard* vMonitor, s_pose& start, s_pose& end);
    ~GoTo();
    void run() override;
    void calculateCost() override;
    
};

#endif /* GoTo_hpp */
