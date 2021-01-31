//
//  GoToROS.hpp
//  MRSFramework
//
//  Created by Afonso Braga on 26/08/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//
//
//  GoToROS.cpp
//  MRSFramework
//
//  Created by Afonso Braga on 26/08/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//


#ifndef GoToROS_hpp
#define GoToROS_hpp

#include <stdio.h>

#include "AtomicTask.hpp"
#include <chrono>
#include <thread>
#include <math.h>
#include <unistd.h>

class GoToROS : public AtomicTask
{
protected:
    std::chrono::system_clock::time_point t0;
    std::chrono::milliseconds tick = std::chrono::milliseconds(100); // 0.1s
    
    float battery_discharge = 50000; // Motors discharge [mAh]
    float robots_max_speed = 0.5; // Robot's maximum speed [m/s]
    float battery_capacity = 7000; // Battery's capacity [mAh]
     
    int factor = 10; // The robot does not go straight to the goal LACOXAMBRE
    
    float kp = 3, ka = 8, kb = -0.5;
    float alpha_t = 0;
    float v = 0;
    
    float adjustAngle(float angle);
    
public:
    GoToROS(Blackboard* vMonitor, s_pose& start, s_pose& end);
    ~GoToROS();
    void run() override;
    void calculateCost() override;
    
};

#endif /* GoToROS_hpp */

