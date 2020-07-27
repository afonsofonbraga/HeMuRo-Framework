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
    float price;
};

class Mission
{
public:
    Mission();
    ~Mission();
    // Common Variables
    char missionCode[5];
    char senderAddress[15];
    char winnerAddress[15];
    enum_DecomposableTask mission = enum_DecomposableTask::null;
    s_pose goal;
};

#endif /* Mission_hpp */
