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
#include <unistd.h>
#include <cmath>

class GoTo : public AtomicTask
{
protected:
    float battery_discharge = 50000; // Motors discharge [mAh]
    float robots_max_speed = 0.2; // Robot's maximum speed [m/s]
    float battery_capacity = 7000; // Battery's capacity [mAh]
    int factor = 10; // The robot does not go straight to the goal LACOXAMBRE
    float costMeter = factor * (battery_discharge /(robots_max_speed*3600))/battery_capacity;
    
    int i = 0;

    float kp = 1, ki = 1, kd = 1;
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
