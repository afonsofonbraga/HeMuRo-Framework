//
//  GoTo.hpp
//  MRSFramework
//
//  Created by Afonso Braga on 25/08/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef GoTo_hpp
#define GoTo_hpp

#include "AtomicTask.hpp"
#include <math.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>

class GoTo : public AtomicTask
{
public:
    float costMeter = 2.0;
    
    float kp = 1, ki = 0.2, kd = 0.5;
    float sum_Alpha_t = 0;
    float alpha_t_old = 0;
    float omega = 0;
    float alpha_t = 0;
    float v = 0;

//ros::NodeHandle* n;
//ros::Publisher* chatter_pub; //= n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
//ros::Rate* loop_rate;
    
public:
    GoTo(BlackBoard* vMonitor, s_pose& start, s_pose& end);
    ~GoTo();
    void run() override;
    void calculateCost() override;
    
};

#endif /* GoTo_hpp */
