//
//  GoTo.hpp
//  MRSFramework
//
//  Created by Afonso Braga on 26/08/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//
//
//  GoTo.cpp
//  MRSFramework
//
//  Created by Afonso Braga on 26/08/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//


#ifndef GoTo_hpp
#define GoTo_hpp

#include <stdio.h>

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
    
    float kp = 3, ka = 8, kb = -0.5;
    float alpha_t = 0;
    float v = 0;
    
    float adjustAngle(float angle);
    
public:
    GoTo(BlackBoard* vMonitor, s_pose& start, s_pose& end);
    ~GoTo();
    void run() override;
    void calculateCost() override;
    
};

#endif /* GoTo_hpp */

