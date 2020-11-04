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
    std::chrono::milliseconds tick = std::chrono::milliseconds(4000); // 1s
    
    float battery_discharge = 50000; // Motors discharge [mAh]
    float robots_max_speed = 0.5; // Robot's maximum speed [m/s]
    float battery_capacity = 7000; // Battery's capacity [mAh]
    
    int factor = 10; // The robot does not go straight to the goal LACOXAMBRE
    float costMeter = factor * (battery_discharge /(robots_max_speed*3600))/battery_capacity;
    
    
    float kp = 3, ka = 8, kb = -0.5;
    float alpha_t = 0;
    float v = 0;
    
    float adjustAngle(float angle);
    
public:
    MoveBaseGoal(BlackBoard* vMonitor, s_pose& start, s_pose& end);
    ~MoveBaseGoal();
    void run() override;
    void stop() override;
    void calculateCost() override;

};
#endif /* MoveBaseGoal_hpp */
