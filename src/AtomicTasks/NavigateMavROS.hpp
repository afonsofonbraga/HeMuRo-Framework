//
//  NavigateMavROS.hpp
//  MRSMac
//
//  Created by Afonso Braga on 01/10/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef NavigateMavROS_hpp
#define NavigateMavROS_hpp


#include <stdio.h>

#include "AtomicTask.hpp"
#include <chrono>
#include <thread>
#include <math.h>
#include <unistd.h>

class NavigateMavROS : public AtomicTask
{
protected:
    std::chrono::system_clock::time_point t0;
    std::chrono::milliseconds tick = std::chrono::milliseconds(4000); // 1s
    
    float battery_discharge = 50000; // Motors discharge [mAh]
    float robots_max_speed = 0.2; // Robot's maximum speed [m/s]
    float battery_capacity = 7000; // Battery's capacity [mAh]
    
    int factor = 10; // The robot does not go straight to the goal LACOXAMBRE

    
public:
    NavigateMavROS(Blackboard* vMonitor, s_pose& start, s_pose& end);
    ~NavigateMavROS();
    void run() override;
    void stop() override;
    void calculateCost() override;
    void calculateTime() override;

};
#endif /* NavigateMavROS_hpp */
