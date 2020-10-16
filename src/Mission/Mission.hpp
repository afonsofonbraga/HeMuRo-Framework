//
//  Mission.hpp
//  MRSMac
//
//  Created by Afonso Braga on 12/06/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef Mission_hpp
#define Mission_hpp

#include <queue>
#include <thread>
#include <chrono>
#include <condition_variable>
#include "AtomicTask.hpp"

struct Bid
{
    char bidderIP[MAX_IP];
    char bidderName[MAX_ROBOT_ID];
    float price;
};

class Mission
{
public:
    Mission();
    ~Mission();
    // Common Variables
    char missionCode[MAX_ID];
    char senderAddress[MAX_IP];
    char senderName[MAX_ROBOT_ID];
    char winnerAddress[MAX_IP];
    char winnerName[MAX_ROBOT_ID];
    enum_DecomposableTask mission = enum_DecomposableTask::null;
    enum_RobotCategory robotCategory;
    int executionTime = 30;
    std::queue<s_pose> goal;
};

#endif /* Mission_hpp */

