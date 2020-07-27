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
#include <math.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

class GoTo : public AtomicTask
{
protected:
    float costMeter = 2.0;
    
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
