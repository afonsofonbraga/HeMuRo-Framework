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

struct Mission
{
    // Common Variables
    char missionCode[5];
    char senderAddress[15];
    enum_DecomposableTask mission = enum_DecomposableTask::null;
    //REQUIREMENTS
    
    // Owner Variables
    //WHOBID
    enum_MissionRequest enum_request = enum_MissionRequest::null;
    std::thread* t5;
    int biddingTime = 5;
    int communicationTime = 1;
    int executionTime = 30;
    std::vector<Bid> vectorBids;
    std::condition_variable* cv;
    std::mutex* cv_m;
    
    
    
    // Execution Variables
    enum_MissionExecution enum_execution = enum_MissionExecution::null;
    char winnerAddress[15];
    bool missionAccepted = false;
    std::vector<AtomicTask> atomicTaskList;
    int atomicTaskIndex = 0;
    float missionCost = 0;
};

#endif /* Mission_hpp */
