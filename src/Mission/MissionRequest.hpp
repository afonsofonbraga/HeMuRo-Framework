//
//  MissionRequest.hpp
//  MRSMac
//
//  Created by Afonso Braga on 30/06/20.
//  Copyright © 2020 Afonso Braga. All rights reserved.
//

#ifndef MissionRequest_hpp
#define MissionRequest_hpp

#include <vector>
#include <thread>
#include <chrono>
#include <condition_variable>
#include "Mission.hpp"
#include "dataTypes.hpp"

class MissionRequest: public Mission{
public:
    // Owner Variables
    //WHOBID
    enum_MissionRequest enum_request = enum_MissionRequest::null;
    std::thread* t5;
    int biddingTime = 5;
    int communicationTime = 2;
    std::vector<Bid> vectorBids;
    std::unique_ptr<std::condition_variable> cv;
    std::unique_ptr<std::mutex> cv_m;
    bool missionAccepted = false;
    bool endMission = false;

public:
    MissionRequest();
    ~MissionRequest();
    
};
#endif /* MissionRequest_hpp */
