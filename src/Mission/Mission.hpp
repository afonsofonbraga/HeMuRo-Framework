//
//  Mission.hpp
//  MRSMac
//
//  Created by Afonso Braga on 12/06/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef Mission_hpp
#define Mission_hpp

#include <vector>
#include <thread>
#include <chrono>
#include <condition_variable>
#include "AtomicTask.hpp"

struct Bid
{
    char bidderIP[15];
    char bidderName[10];
    float price;
};

class Mission
{
public:
    Mission();
    ~Mission();
    // Common Variables
    char missionCode[10];
    char senderAddress[15];
    char senderName[10];
    char winnerAddress[15];
    char winnerName[10];
    enum_DecomposableTask mission = enum_DecomposableTask::null;
    enum_RobotCategory robotCategory;
    int executionTime = 30;
    s_pose goal;
};

#endif /* Mission_hpp */

